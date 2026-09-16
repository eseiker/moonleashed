#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_usb_cdc.h>
#include <furi_hal_bt_hci.h>
#include <gui/gui.h>
#include <cli/cli_vcp.h>
#include "h4_frame.h"

#define TAILCAT_HCI_DEFAULT_RUNTIME_SECONDS 180UL
#define TAILCAT_HCI_MIN_RUNTIME_SECONDS     30UL
#define TAILCAT_HCI_MAX_RUNTIME_SECONDS     3600UL

typedef struct {
    FuriThread* worker;
    FuriSemaphore* usb_tx;
    FuriMessageQueue* keys;
    volatile bool stop;
    volatile bool failed;
    volatile uint32_t sent;
    volatile uint32_t received;
} HciApp;

static void usb_tx_done(void* context) {
    HciApp* app = context;
    furi_semaphore_release(app->usb_tx);
}

static void usb_rx_ready(void* context) {
    HciApp* app = context;
    furi_thread_flags_set(furi_thread_get_id(app->worker), 1);
}

static void usb_state(void* context, CdcState state) {
    HciApp* app = context;
    if(state == CdcStateDisconnected && app->sent) app->failed = true;
}

static CdcCallbacks usb_callbacks = {
    .tx_ep_callback = usb_tx_done,
    .rx_ep_callback = usb_rx_ready,
    .state_callback = usb_state,
};

static int32_t bridge_worker(void* context) {
    HciApp* app = context;
    uint8_t usb_bytes[CDC_DATA_SZ];
    uint8_t response[FURI_HAL_BT_HCI_FRAME_MAX];
    H4Frame frame = {0};
    while(!app->stop && !app->failed) {
        int32_t count = furi_hal_cdc_receive(1, usb_bytes, sizeof(usb_bytes));
        for(int32_t i = 0; i < count && !app->failed; i++) {
            int result = h4_push(&frame, usb_bytes[i]);
            if(result < 0)
                app->failed = true;
            else if(result == 1) {
                if(!furi_hal_bt_hci_send(frame.bytes, frame.used, 1000))
                    app->failed = true;
                else
                    app->sent++;
                frame.used = 0;
            }
        }
        int32_t length = furi_hal_bt_hci_receive(response, sizeof(response), 0);
        if(length < 0) app->failed = true;
        for(int32_t offset = 0; offset < length && !app->failed;) {
            /* Short USB transfers avoid requiring a separate terminating ZLP. */
            uint16_t chunk = MIN(63, length - offset);
            if(furi_semaphore_acquire(app->usb_tx, 1000) != FuriStatusOk) {
                app->failed = true;
                break;
            }
            furi_hal_cdc_send(1, response + offset, chunk);
            offset += chunk;
        }
        if(length > 0 && !app->failed) app->received++;
        if(!count && !length) furi_thread_flags_wait(1, FuriFlagWaitAny, 5);
    }
    return 0;
}

static void draw(Canvas* canvas, void* context) {
    HciApp* app = context;
    char line[48];
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 1, 12, "Tailcat HCI / ABI 2");
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 1, 26, app->failed ? "STOPPED: exit and retry" : "HCILayer / CDC #2");
    snprintf(line, sizeof(line), "Host > %lu   < %lu", app->sent, app->received);
    canvas_draw_str(canvas, 1, 40, line);
    canvas_draw_str(canvas, 1, 55, "Back: reset controller / exit");
}

static void input(InputEvent* event, void* context) {
    HciApp* app = context;
    furi_message_queue_put(app->keys, event, 0);
}

static uint32_t runtime_seconds_from_context(const void* context) {
    if(!context) return TAILCAT_HCI_DEFAULT_RUNTIME_SECONDS;
    const char* text = context;
    char* end = NULL;
    unsigned long seconds = strtoul(text, &end, 10);
    if(end == text || *end != '\0' || seconds < TAILCAT_HCI_MIN_RUNTIME_SECONDS ||
       seconds > TAILCAT_HCI_MAX_RUNTIME_SECONDS) {
        FURI_LOG_W(
            "TailcatHci",
            "Invalid runtime '%s'; using %lus",
            text,
            TAILCAT_HCI_DEFAULT_RUNTIME_SECONDS);
        return TAILCAT_HCI_DEFAULT_RUNTIME_SECONDS;
    }
    return (uint32_t)seconds;
}

int32_t tailcat_hci_app(void* context) {
    const uint32_t runtime_seconds = runtime_seconds_from_context(context);
    if(furi_hal_bt_hci_get_abi() != FURI_HAL_BT_HCI_ABI ||
       !furi_hal_bt_hci_acquire(FURI_HAL_BT_HCI_ABI)) {
        FURI_LOG_E("TailcatHci", "HCILayer 1.20.0 controller unavailable");
        furi_hal_power_reset();
        return -1;
    }
    HciApp* app = malloc(sizeof(HciApp));
    memset(app, 0, sizeof(HciApp));
    app->usb_tx = furi_semaphore_alloc(1, 1);
    app->keys = furi_message_queue_alloc(8, sizeof(InputEvent));
    app->worker = furi_thread_alloc_ex("HciUsb", 2048, bridge_worker, app);
    Gui* gui = furi_record_open(RECORD_GUI);
    ViewPort* viewport = view_port_alloc();
    view_port_draw_callback_set(viewport, draw, app);
    view_port_input_callback_set(viewport, input, app);
    gui_add_view_port(gui, viewport, GuiLayerFullscreen);
    CliVcp* cli = furi_record_open(RECORD_CLI_VCP);
    FuriHalUsbInterface* previous = furi_hal_usb_get_config();
    /* Firmware that already runs two CDC ports by default: just claim
     * interface 1, no USB re-enumeration. Older firmware: switch as before. */
    bool reconfigure = previous != &usb_cdc_dual;
    bool usb_ok = true;
    if(reconfigure) {
        furi_hal_usb_unlock();
        usb_ok = furi_hal_usb_set_config(&usb_cdc_dual, NULL);
        if(usb_ok) cli_vcp_enable(cli);
    }
    if(usb_ok) {
        furi_hal_cdc_set_callbacks(1, &usb_callbacks, app);
        furi_thread_start(app->worker);
        InputEvent key;
        uint32_t started = furi_get_tick();
        const uint32_t runtime_ticks = furi_ms_to_ticks(runtime_seconds * 1000UL);
        while(!app->failed && furi_get_tick() - started < runtime_ticks) {
            if(furi_message_queue_get(app->keys, &key, 100) == FuriStatusOk &&
               key.key == InputKeyBack && key.type == InputTypeShort)
                break;
            view_port_update(viewport);
        }
        app->stop = true;
        furi_thread_join(app->worker);
        furi_hal_cdc_set_callbacks(1, NULL, NULL);
        if(reconfigure) {
            cli_vcp_disable(cli);
            furi_hal_usb_set_config(previous, NULL);
            cli_vcp_enable(cli);
        }
    }
    bool reset_ok = furi_hal_bt_hci_release();
    gui_remove_view_port(gui, viewport);
    view_port_free(viewport);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_CLI_VCP);
    furi_thread_free(app->worker);
    furi_message_queue_free(app->keys);
    furi_semaphore_free(app->usb_tx);
    free(app);
    if(!reset_ok) furi_hal_power_reset();
    return 0;
}
