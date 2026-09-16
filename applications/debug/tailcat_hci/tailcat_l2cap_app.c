/*
 * Tailcat host-L2CAP USB bridge (TASK-646). Unlike the raw-HCI mode, this keeps
 * the resident NimBLE host running and bridges L2CAP CoC control + SDUs over USB
 * CDC interface 1 using the on-host ble_l2cap_coc_* API (KNOW-636). A small
 * external tool speaks the TLV protocol in l2cap_frame.h; the Flipper does all
 * the BLE. Because the host stays up, the companion link can coexist with the
 * DCT CoC (KNOW-625) — which raw HCI cannot.
 */

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_usb_cdc.h>
#include <gui/gui.h>
#include <cli/cli_vcp.h>

#include <furi_ble/l2cap_coc.h>
#include <furi_ble/adv.h>
#include <furi_ble/furi_ble_session.h>
#include "l2cap_frame.h"

#define TAG "TailcatL2cap"

#define L2_DEFAULT_RUNTIME_SECONDS 300UL
#define L2_TX_STREAM_SIZE          2048U

typedef struct {
    FuriThread* worker;
    FuriSemaphore* usb_tx;
    FuriMessageQueue* keys;
    FuriStreamBuffer* tx; /* FAP -> host bytes, framed */
    volatile bool stop;
    volatile bool failed;
    volatile bool connected;
    volatile uint32_t rx_sdus; /* SDUs BLE -> USB */
    volatile uint32_t tx_sdus; /* SDUs USB -> BLE */
    /* L2F_CONNECT: modal central session (companion suspended while it runs). */
    FuriBleSession* central;
    uint16_t central_psm;
    volatile bool central_coc_opened; /* a CoC opened on the central link */
} L2App;

/* Queue one framed message for the USB worker to send. */
static void l2_emit(L2App* app, uint8_t type, const uint8_t* payload, uint16_t len) {
    uint8_t hdr[3] = {type, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    furi_stream_buffer_send(app->tx, hdr, sizeof(hdr), 0);
    if(len && payload) furi_stream_buffer_send(app->tx, payload, len, 0);
}

/* CoC events arrive on the NimBLE host thread; frame them into the tx stream. */
static void on_coc(BleL2capCocEvent* ev, void* context) {
    L2App* app = context;
    switch(ev->type) {
    case BleL2capCocEventConnected: {
        uint8_t p[3] = {
            ev->channel_index,
            (uint8_t)(ev->connection_handle & 0xFF),
            (uint8_t)(ev->connection_handle >> 8)};
        app->connected = true;
        if(app->central) app->central_coc_opened = true;
        l2_emit(app, L2F_CONNECTED, p, sizeof(p));
    } break;
    case BleL2capCocEventDataReceived: {
        static uint8_t buf[1 + L2F_PAYLOAD_MAX];
        uint16_t n = ev->data.data_len;
        if(n > L2F_PAYLOAD_MAX - 1) n = L2F_PAYLOAD_MAX - 1;
        buf[0] = ev->channel_index;
        memcpy(buf + 1, ev->data.data, n);
        app->rx_sdus++;
        l2_emit(app, L2F_DATA, buf, n + 1);
    } break;
    case BleL2capCocEventDisconnected: {
        uint8_t p[1] = {ev->channel_index};
        app->connected = false;
        l2_emit(app, L2F_DISCONNECTED, p, sizeof(p));
    } break;
    case BleL2capCocEventError: {
        uint8_t p[2] = {(uint8_t)(ev->error.code & 0xFF), (uint8_t)(ev->error.code >> 8)};
        l2_emit(app, L2F_ERROR, p, sizeof(p));
    } break;
    default:
        break;
    }
}

static void l2_handle_frame(L2App* app, const L2Frame* f) {
    switch(f->type) {
    case L2F_LISTEN:
        if(f->len >= 2) {
            uint16_t psm = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            ble_l2cap_coc_listen(psm, BLE_L2CAP_COC_MTU_DEFAULT);
        }
        break;
    case L2F_SEND:
        if(f->len >= 1) {
            uint8_t ch = f->data[0];
            if(ble_l2cap_coc_send(ch, f->data + 1, f->len - 1)) app->tx_sdus++;
        }
        break;
    case L2F_CLOSE:
        if(f->len >= 1) ble_l2cap_coc_disconnect(f->data[0]);
        /* On a central session, closing the channel ends the session too: the
         * link drops and the companion is restored. */
        if(app->central) {
            furi_ble_session_free(app->central);
            app->central = NULL;
            app->central_coc_opened = false;
        }
        break;
    case L2F_ADVERTISE:
        /* [adv_len:1][adv...][rsp...]: install the caller's advertisement (e.g.
         * the DCT FC73 session adv) on the resident host so peers discover us. */
        if(f->len >= 1 && (uint16_t)(1 + f->data[0]) <= f->len) {
            uint8_t adv_len = f->data[0];
            const uint8_t* adv = f->data + 1;
            const uint8_t* rsp = f->data + 1 + adv_len;
            uint16_t rsp_len = f->len - 1 - adv_len;
            if(rsp_len > 31 || !furi_ble_adv_set(adv, adv_len, rsp, (uint8_t)rsp_len)) {
                uint8_t code[2] = {0x01, 0x00};
                l2_emit(app, L2F_ERROR, code, sizeof(code));
            }
        }
        break;
    case L2F_CONNECT: {
        /* [psm:2][name...]: suspend the companion, scan for `name`, connect as
         * central, then open a CoC client on psm once the link is up (the
         * session's Connected event is handled in l2_poll_central). */
        if(f->len < 3) break;
        if(app->central) {
            uint8_t code[2] = {0x02, 0x00}; /* a central session is already active */
            l2_emit(app, L2F_ERROR, code, sizeof(code));
            break;
        }
        char name[24];
        uint16_t nlen = f->len - 2;
        if(nlen > sizeof(name) - 1) nlen = sizeof(name) - 1;
        memcpy(name, f->data + 2, nlen);
        name[nlen] = '\0';
        app->central_psm = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
        app->central_coc_opened = false;
        FuriBleSessionConfig cfg = {.role = FuriBleRoleCentralModal, .central_name = name};
        app->central = furi_ble_session_alloc(&cfg);
        if(!app->central) {
            uint8_t code[2] = {0x02, 0x00}; /* refused: host not ready or busy */
            l2_emit(app, L2F_ERROR, code, sizeof(code));
        }
    } break;
    default:
        break;
    }
}

/* Drain the central session's off-thread event queue (worker thread). */
static void l2_poll_central(L2App* app) {
    if(!app->central) return;
    FuriBleEvent ev;
    if(!furi_ble_session_get_event(app->central, &ev, 0)) return;
    switch(ev.type) {
    case FuriBleEventCentralConnected:
        /* Link is up: open the CoC as the client. Its Connected/Data events
         * then flow through on_coc like the server path. */
        if(!ble_l2cap_coc_connect(
               ev.conn_handle,
               app->central_psm,
               BLE_L2CAP_COC_MTU_DEFAULT,
               BLE_L2CAP_COC_MPS_MAX,
               BLE_L2CAP_COC_CREDITS_DEFAULT)) {
            uint8_t code[2] = {0x03, 0x00}; /* CoC client open failed */
            l2_emit(app, L2F_ERROR, code, sizeof(code));
            furi_ble_session_free(app->central);
            app->central = NULL;
        }
        break;
    case FuriBleEventCentralFailed: {
        uint8_t code[2] = {0x02, 0x00}; /* peer not found / connect failed */
        l2_emit(app, L2F_ERROR, code, sizeof(code));
        furi_ble_session_free(app->central);
        app->central = NULL;
    } break;
    case FuriBleEventCentralDisconnected:
        /* A CoC that opened already reported DISCONNECTED via on_coc; if the
         * link went before any CoC opened, tell the host. */
        if(!app->central_coc_opened) {
            uint8_t code[2] = {0x04, 0x00}; /* link lost before the CoC opened */
            l2_emit(app, L2F_ERROR, code, sizeof(code));
        }
        furi_ble_session_free(app->central);
        app->central = NULL;
        app->central_coc_opened = false;
        break;
    default:
        break;
    }
}

static void usb_tx_done(void* context) {
    L2App* app = context;
    furi_semaphore_release(app->usb_tx);
}

static void usb_rx_ready(void* context) {
    L2App* app = context;
    furi_thread_flags_set(furi_thread_get_id(app->worker), 1);
}

static void usb_state(void* context, CdcState state) {
    UNUSED(context);
    UNUSED(state);
}

static CdcCallbacks usb_callbacks = {
    .tx_ep_callback = usb_tx_done,
    .rx_ep_callback = usb_rx_ready,
    .state_callback = usb_state,
};

static int32_t l2_worker(void* context) {
    L2App* app = context;
    uint8_t usb_bytes[CDC_DATA_SZ];
    uint8_t out[64];
    /* L2F_PAYLOAD_MAX is 2 KiB, so keep the reassembly frame off the 2 KiB
     * worker stack. One worker, so a static is safe. */
    static L2Frame frame;
    l2f_reset(&frame);

    while(!app->stop && !app->failed) {
        /* USB -> BLE */
        int32_t count = furi_hal_cdc_receive(1, usb_bytes, sizeof(usb_bytes));
        for(int32_t i = 0; i < count; i++) {
            int r = l2f_push(&frame, usb_bytes[i]);
            if(r < 0) {
                l2f_reset(&frame);
            } else if(r == 1) {
                l2_handle_frame(app, &frame);
                l2f_reset(&frame);
            }
        }
        /* Central session lifecycle (L2F_CONNECT). */
        l2_poll_central(app);
        /* BLE -> USB: drain the framed tx stream in <=63-byte chunks. */
        while(furi_stream_buffer_bytes_available(app->tx) && !app->failed) {
            size_t chunk = furi_stream_buffer_receive(app->tx, out, 63, 0);
            if(!chunk) break;
            if(furi_semaphore_acquire(app->usb_tx, 1000) != FuriStatusOk) {
                app->failed = true;
                break;
            }
            furi_hal_cdc_send(1, out, chunk);
        }
        if(!count && !furi_stream_buffer_bytes_available(app->tx))
            furi_thread_flags_wait(1, FuriFlagWaitAny, 5);
    }
    return 0;
}

static void draw(Canvas* canvas, void* context) {
    L2App* app = context;
    char line[48];
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 1, 12, "Tailcat L2CAP bridge");
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 1, 26, app->connected ? "CoC connected" : "on-host / CDC #2");
    snprintf(line, sizeof(line), "SDU  >%lu  <%lu", app->tx_sdus, app->rx_sdus);
    canvas_draw_str(canvas, 1, 40, line);
    canvas_draw_str(canvas, 1, 55, "Back: exit");
}

static void input(InputEvent* event, void* context) {
    L2App* app = context;
    furi_message_queue_put(app->keys, event, 0);
}

int32_t tailcat_l2cap_app(void* context) {
    UNUSED(context);

    L2App* app = malloc(sizeof(L2App));
    memset(app, 0, sizeof(L2App));
    app->usb_tx = furi_semaphore_alloc(1, 1);
    app->keys = furi_message_queue_alloc(8, sizeof(InputEvent));
    app->tx = furi_stream_buffer_alloc(L2_TX_STREAM_SIZE, 1);
    app->worker = furi_thread_alloc_ex("L2capUsb", 2048, l2_worker, app);

    /* Use the resident host's CoC API — no raw HCI, no controller acquire. */
    ble_l2cap_coc_init();
    ble_l2cap_coc_set_callback(0, on_coc, app);

    Gui* gui = furi_record_open(RECORD_GUI);
    ViewPort* viewport = view_port_alloc();
    view_port_draw_callback_set(viewport, draw, app);
    view_port_input_callback_set(viewport, input, app);
    gui_add_view_port(gui, viewport, GuiLayerFullscreen);

    CliVcp* cli = furi_record_open(RECORD_CLI_VCP);
    FuriHalUsbInterface* previous = furi_hal_usb_get_config();
    furi_hal_usb_unlock();
    if(furi_hal_usb_set_config(&usb_cdc_dual, NULL)) {
        cli_vcp_enable(cli);
        furi_hal_cdc_set_callbacks(1, &usb_callbacks, app);
        furi_thread_start(app->worker);

        InputEvent key;
        uint32_t started = furi_get_tick();
        const uint32_t runtime_ticks = furi_ms_to_ticks(L2_DEFAULT_RUNTIME_SECONDS * 1000UL);
        while(!app->failed && furi_get_tick() - started < runtime_ticks) {
            if(furi_message_queue_get(app->keys, &key, 100) == FuriStatusOk &&
               key.key == InputKeyBack && key.type == InputTypeShort)
                break;
            view_port_update(viewport);
        }
        app->stop = true;
        furi_thread_join(app->worker);
        furi_hal_cdc_set_callbacks(1, NULL, NULL);
        cli_vcp_disable(cli);
        furi_hal_usb_set_config(previous, NULL);
        cli_vcp_enable(cli);
    }

    if(app->central) {
        furi_ble_session_free(app->central); /* drop the link, restore the companion */
        app->central = NULL;
    }
    furi_ble_adv_clear(); /* restore the companion advertisement */
    ble_l2cap_coc_set_callback(0, NULL, NULL);
    ble_l2cap_coc_deinit();

    gui_remove_view_port(gui, viewport);
    view_port_free(viewport);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_CLI_VCP);
    furi_thread_free(app->worker);
    furi_stream_buffer_free(app->tx);
    furi_message_queue_free(app->keys);
    furi_semaphore_free(app->usb_tx);
    free(app);
    return 0;
}
