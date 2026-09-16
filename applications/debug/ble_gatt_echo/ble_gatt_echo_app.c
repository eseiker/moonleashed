/*
 * BLE GATT Echo (TASK-632 test app). Defines a custom 128-bit GATT service with
 * only the stock app-facing API — a FuriHalBleProfileTemplate started through
 * bt_profile_start, ble_gatt_service_add + ble_gatt_characteristic_* and a
 * ble_event_dispatcher handler — exactly as an OFW app would. Under the NimBLE
 * host this exercises the runtime GATT table rebuild: RX (write) is echoed to
 * TX (read + notify).
 *
 * Service 7a1c0001-7f2a-4a7b-9c3d-5e6f70818293
 *   RX  7a1c0002-...  write / write-without-response
 *   TX  7a1c0003-...  read / notify
 */

#include <furi.h>
#include <gui/gui.h>
#include <bt/bt_service/bt.h>

#include <app_common.h>
#include <ble/ble.h>
#include <furi_ble/event_dispatcher.h>
#include <furi_ble/gatt.h>
#include <furi_ble/profile_interface.h>

#define TAG      "GattEcho"
#define ECHO_MAX 64

/* 128-bit UUIDs, little-endian as the stack expects. */
#define ECHO_UUID(last) \
    {0x93, 0x82, 0x81, 0x70, 0x6f, 0x5e, 0x3d, 0x9c, 0x7b, 0x4a, 0x2a, 0x7f, last, 0x00, 0x1c, 0x7a}
static const Service_UUID_t echo_svc_uuid = {.Service_UUID_128 = ECHO_UUID(0x01)};

typedef enum {
    EchoCharRx,
    EchoCharTx,
    EchoCharCount,
} EchoChar;

typedef struct {
    const uint8_t* data;
    uint16_t len;
} EchoValue;

/* Size query at init (data == NULL) and value source on update. */
static bool echo_value_cb(const void* context, const uint8_t** data, uint16_t* data_len) {
    if(data == NULL) {
        *data_len = ECHO_MAX;
        return false;
    }
    const EchoValue* v = context;
    *data = v ? v->data : NULL;
    *data_len = v ? v->len : 0;
    return false;
}

static const BleGattCharacteristicParams echo_chars[EchoCharCount] = {
    [EchoCharRx] =
        {.name = "Echo RX",
         .data_prop_type = FlipperGattCharacteristicDataCallback,
         .data.callback.fn = echo_value_cb,
         .data.callback.context = NULL,
         .uuid.Char_UUID_128 = ECHO_UUID(0x02),
         .uuid_type = UUID_TYPE_128,
         .char_properties = CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP,
         .security_permissions = ATTR_PERMISSION_NONE,
         .gatt_evt_mask = GATT_NOTIFY_ATTRIBUTE_WRITE,
         .is_variable = CHAR_VALUE_LEN_VARIABLE},
    [EchoCharTx] =
        {.name = "Echo TX",
         .data_prop_type = FlipperGattCharacteristicDataCallback,
         .data.callback.fn = echo_value_cb,
         .data.callback.context = NULL,
         .uuid.Char_UUID_128 = ECHO_UUID(0x03),
         .uuid_type = UUID_TYPE_128,
         .char_properties = CHAR_PROP_READ | CHAR_PROP_NOTIFY,
         .security_permissions = ATTR_PERMISSION_NONE,
         .gatt_evt_mask = GATT_DONT_NOTIFY_EVENTS,
         .is_variable = CHAR_VALUE_LEN_VARIABLE},
};

typedef struct {
    FuriHalBleProfileBase base;
    uint16_t svc_handle;
    BleGattCharacteristicInstance chars[EchoCharCount];
    GapSvcEventHandler* event_handler;
    uint8_t last[ECHO_MAX];
    volatile uint16_t last_len;
    volatile uint32_t rx_count;
    volatile bool subscribed;
} EchoProfile;

static const FuriHalBleProfileTemplate* echo_profile_template(void);

/* Runs on the BLE dispatch thread under the NimBLE host. */
static BleEventAckStatus echo_event_handler(void* event, void* context) {
    EchoProfile* p = context;
    hci_event_pckt* event_pckt = (hci_event_pckt*)(((hci_uart_pckt*)event)->data);
    if(event_pckt->evt != HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE) return BleEventNotAck;
    evt_blecore_aci* blecore_evt = (evt_blecore_aci*)event_pckt->data;
    if(blecore_evt->ecode != ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE) return BleEventNotAck;

    aci_gatt_attribute_modified_event_rp0* am =
        (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
    if(am->Attr_Handle == p->chars[EchoCharRx].handle + 1) {
        uint16_t n = MIN(am->Attr_Data_Length, (uint16_t)ECHO_MAX);
        memcpy(p->last, am->Attr_Data, n);
        p->last_len = n;
        p->rx_count++;
        EchoValue v = {.data = p->last, .len = n};
        ble_gatt_characteristic_update(p->svc_handle, &p->chars[EchoCharTx], &v);
        FURI_LOG_I(TAG, "RX %u bytes -> TX", n);
        return BleEventAckFlowEnable;
    }
    if(am->Attr_Handle == p->chars[EchoCharTx].handle + 2 && am->Attr_Data_Length >= 1) {
        p->subscribed = (am->Attr_Data[0] & 0x01) != 0;
        FURI_LOG_I(TAG, "TX notifications %s", p->subscribed ? "on" : "off");
        return BleEventAckFlowEnable;
    }
    return BleEventNotAck;
}

static FuriHalBleProfileBase* echo_profile_start(FuriHalBleProfileParams params) {
    UNUSED(params);
    EchoProfile* p = malloc(sizeof(EchoProfile));
    memset(p, 0, sizeof(EchoProfile));
    p->base.config = echo_profile_template();

    p->event_handler = ble_event_dispatcher_register_svc_handler(echo_event_handler, p);
    if(!ble_gatt_service_add(UUID_TYPE_128, &echo_svc_uuid, PRIMARY_SERVICE, 1 + 2 + 3, &p->svc_handle)) {
        ble_event_dispatcher_unregister_svc_handler(p->event_handler);
        free(p);
        return NULL;
    }
    for(size_t i = 0; i < EchoCharCount; i++) {
        ble_gatt_characteristic_init(p->svc_handle, &echo_chars[i], &p->chars[i]);
    }
    return &p->base;
}

static void echo_profile_stop(FuriHalBleProfileBase* profile) {
    EchoProfile* p = (EchoProfile*)profile;
    ble_event_dispatcher_unregister_svc_handler(p->event_handler);
    for(size_t i = 0; i < EchoCharCount; i++) {
        ble_gatt_characteristic_delete(p->svc_handle, &p->chars[i]);
    }
    ble_gatt_service_delete(p->svc_handle);
    free(p);
}

static void echo_profile_gap_config(GapConfig* config, FuriHalBleProfileParams params) {
    UNUSED(params);
    /* Not consulted by the NimBLE host (it keeps its own advertising). */
    memset(config, 0, sizeof(GapConfig));
}

static const FuriHalBleProfileTemplate echo_template = {
    .start = echo_profile_start,
    .stop = echo_profile_stop,
    .get_gap_config = echo_profile_gap_config,
};

static const FuriHalBleProfileTemplate* echo_profile_template(void) {
    return &echo_template;
}

typedef struct {
    EchoProfile* profile;
    FuriMessageQueue* keys;
} EchoApp;

static void echo_draw(Canvas* canvas, void* context) {
    EchoApp* app = context;
    char line[40];
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 1, 12, "BLE GATT Echo");
    canvas_set_font(canvas, FontSecondary);
    if(!app->profile) {
        canvas_draw_str(canvas, 1, 26, "profile start FAILED");
    } else {
        EchoProfile* p = app->profile;
        snprintf(line, sizeof(line), "RX %lu  notify:%s", p->rx_count, p->subscribed ? "on" : "off");
        canvas_draw_str(canvas, 1, 26, line);
        snprintf(line, sizeof(line), "last: %.*s", (int)MIN(p->last_len, (uint16_t)20), (const char*)p->last);
        canvas_draw_str(canvas, 1, 40, line);
    }
    canvas_draw_str(canvas, 1, 55, "Back: exit");
}

static void echo_input(InputEvent* event, void* context) {
    EchoApp* app = context;
    furi_message_queue_put(app->keys, event, 0);
}

int32_t ble_gatt_echo_app(void* arg) {
    UNUSED(arg);
    EchoApp app = {0};
    app.keys = furi_message_queue_alloc(8, sizeof(InputEvent));

    Bt* bt = furi_record_open(RECORD_BT);
    app.profile = (EchoProfile*)bt_profile_start(bt, echo_profile_template(), NULL);
    FURI_LOG_I(TAG, "profile start -> %p", (void*)app.profile);

    Gui* gui = furi_record_open(RECORD_GUI);
    ViewPort* vp = view_port_alloc();
    view_port_draw_callback_set(vp, echo_draw, &app);
    view_port_input_callback_set(vp, echo_input, &app);
    gui_add_view_port(gui, vp, GuiLayerFullscreen);

    InputEvent key;
    for(;;) {
        if(furi_message_queue_get(app.keys, &key, 200) == FuriStatusOk && key.key == InputKeyBack &&
           key.type == InputTypeShort)
            break;
        view_port_update(vp);
    }

    gui_remove_view_port(gui, vp);
    view_port_free(vp);
    furi_record_close(RECORD_GUI);

    bt_profile_restore_default(bt); /* stops the echo profile; table rebuilds without it */
    furi_record_close(RECORD_BT);
    furi_message_queue_free(app.keys);
    return 0;
}
