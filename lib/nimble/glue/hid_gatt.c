/*
 * BLE HID (HOGP) GATT server on NimBLE (TASK-604). See hid_gatt.h.
 *
 * Registered alongside serial_gatt.c in one GATT table: the Flipper exposes the
 * Serial Service AND HID + Device Information + Battery services on a single
 * connection. NimBLE fixes the GATT table before the host starts, so both are
 * registered at boot (a service cannot be added while connected); this exceeds
 * the stock CPU2 host, which could only switch between serial and HID.
 *
 * The report map bytes, report structs, report ids and HID information value are
 * copied verbatim from the stock profile (lib/ble_profile/extra_profiles/
 * hid_profile.c) so a host sees an identical HID device.
 */

#include <furi.h>
#include <string.h>
#include <usb_hid.h>
#include <hid_usage_desktop.h>
#include <hid_usage_button.h>
#include <hid_usage_consumer.h>
#include <hid_usage_led.h>
#include <hid_usage_keyboard.h>

#include "os/os_mbuf.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "host/ble_att.h"

#include "hid_gatt.h"

#define TAG "HidGatt"

/* Report ids (must be non-zero) and the input-report ordinals, verbatim from the
 * stock profile. The Report Reference descriptor of each input report carries
 * {report_id, 0x01 (input)}. */
#define HID_REPORT_ID_KEYBOARD 1
#define HID_REPORT_ID_MOUSE    2
#define HID_REPORT_ID_CONSUMER 3

#define HID_KB_MAX_KEYS   6
#define HID_CONSUMER_KEYS 1

/* HID information: bcdHID 0x0101, country 0, flags remote-wake|normally-connectable. */
#define HID_INFO_SPEC_LSB 0x01
#define HID_INFO_SPEC_MSB 0x01
#define HID_INFO_COUNTRY  0x00
#define HID_INFO_FLAGS    0x03

typedef struct {
    uint8_t mods;
    uint8_t reserved;
    uint8_t key[HID_KB_MAX_KEYS];
} FURI_PACKED HidKbReport;

typedef struct {
    uint8_t btn;
    int8_t x;
    int8_t y;
    int8_t wheel;
} FURI_PACKED HidMouseReport;

typedef struct {
    uint16_t key[HID_CONSUMER_KEYS];
} FURI_PACKED HidConsumerReport;

/* keyboard+mouse+consumer report map, byte-identical to the stock profile. */
static const uint8_t hid_report_map[] = {
    // Keyboard Report
    HID_USAGE_PAGE(HID_PAGE_DESKTOP),
    HID_USAGE(HID_DESKTOP_KEYBOARD),
    HID_COLLECTION(HID_APPLICATION_COLLECTION),
    HID_REPORT_ID(HID_REPORT_ID_KEYBOARD),
    HID_USAGE_PAGE(HID_DESKTOP_KEYPAD),
    HID_USAGE_MINIMUM(HID_KEYBOARD_L_CTRL),
    HID_USAGE_MAXIMUM(HID_KEYBOARD_R_GUI),
    HID_LOGICAL_MINIMUM(0),
    HID_LOGICAL_MAXIMUM(1),
    HID_REPORT_SIZE(1),
    HID_REPORT_COUNT(8),
    HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
    HID_REPORT_COUNT(1),
    HID_REPORT_SIZE(8),
    HID_INPUT(HID_IOF_CONSTANT | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
    HID_USAGE_PAGE(HID_PAGE_LED),
    HID_REPORT_COUNT(8),
    HID_REPORT_SIZE(1),
    HID_USAGE_MINIMUM(1),
    HID_USAGE_MAXIMUM(8),
    HID_OUTPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
    HID_REPORT_COUNT(HID_KB_MAX_KEYS),
    HID_REPORT_SIZE(8),
    HID_LOGICAL_MINIMUM(0),
    HID_RI_LOGICAL_MAXIMUM(16, 255),
    HID_USAGE_PAGE(HID_DESKTOP_KEYPAD),
    HID_USAGE_MINIMUM(0),
    HID_RI_USAGE_MAXIMUM(16, 255),
    HID_INPUT(HID_IOF_DATA | HID_IOF_ARRAY | HID_IOF_ABSOLUTE),
    HID_END_COLLECTION,
    // Mouse Report
    HID_USAGE_PAGE(HID_PAGE_DESKTOP),
    HID_USAGE(HID_DESKTOP_MOUSE),
    HID_COLLECTION(HID_APPLICATION_COLLECTION),
    HID_USAGE(HID_DESKTOP_POINTER),
    HID_COLLECTION(HID_PHYSICAL_COLLECTION),
    HID_REPORT_ID(HID_REPORT_ID_MOUSE),
    HID_USAGE_PAGE(HID_PAGE_BUTTON),
    HID_USAGE_MINIMUM(1),
    HID_USAGE_MAXIMUM(3),
    HID_LOGICAL_MINIMUM(0),
    HID_LOGICAL_MAXIMUM(1),
    HID_REPORT_COUNT(3),
    HID_REPORT_SIZE(1),
    HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
    HID_REPORT_SIZE(1),
    HID_REPORT_COUNT(5),
    HID_INPUT(HID_IOF_CONSTANT | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
    HID_USAGE_PAGE(HID_PAGE_DESKTOP),
    HID_USAGE(HID_DESKTOP_X),
    HID_USAGE(HID_DESKTOP_Y),
    HID_USAGE(HID_DESKTOP_WHEEL),
    HID_LOGICAL_MINIMUM(-127),
    HID_LOGICAL_MAXIMUM(127),
    HID_REPORT_SIZE(8),
    HID_REPORT_COUNT(3),
    HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_RELATIVE),
    HID_END_COLLECTION,
    HID_END_COLLECTION,
    // Consumer Report
    HID_USAGE_PAGE(HID_PAGE_CONSUMER),
    HID_USAGE(HID_CONSUMER_CONTROL),
    HID_COLLECTION(HID_APPLICATION_COLLECTION),
    HID_REPORT_ID(HID_REPORT_ID_CONSUMER),
    HID_LOGICAL_MINIMUM(0),
    HID_RI_LOGICAL_MAXIMUM(16, 0x3FF),
    HID_USAGE_MINIMUM(0),
    HID_RI_USAGE_MAXIMUM(16, 0x3FF),
    HID_REPORT_COUNT(HID_CONSUMER_KEYS),
    HID_REPORT_SIZE(16),
    HID_INPUT(HID_IOF_DATA | HID_IOF_ARRAY | HID_IOF_ABSOLUTE),
    HID_END_COLLECTION,
};

/* 16-bit SIG UUIDs. */
#define UUID_SVC_HID       0x1812
#define UUID_CHR_PROTOCOL  0x2A4E
#define UUID_CHR_REPORT    0x2A4D
#define UUID_CHR_REPMAP    0x2A4B
#define UUID_CHR_HIDINFO   0x2A4A
#define UUID_CHR_CTRLPOINT 0x2A4C
#define UUID_DSC_REPORTREF 0x2908
#define UUID_SVC_DIS       0x180A
#define UUID_CHR_PNP_ID    0x2A50
#define UUID_SVC_BAS       0x180F
#define UUID_CHR_BATTERY   0x2A19

static const ble_uuid16_t uuid_svc_hid = BLE_UUID16_INIT(UUID_SVC_HID);
static const ble_uuid16_t uuid_chr_protocol = BLE_UUID16_INIT(UUID_CHR_PROTOCOL);
static const ble_uuid16_t uuid_chr_report = BLE_UUID16_INIT(UUID_CHR_REPORT);
static const ble_uuid16_t uuid_chr_repmap = BLE_UUID16_INIT(UUID_CHR_REPMAP);
static const ble_uuid16_t uuid_chr_hidinfo = BLE_UUID16_INIT(UUID_CHR_HIDINFO);
static const ble_uuid16_t uuid_chr_ctrlpoint = BLE_UUID16_INIT(UUID_CHR_CTRLPOINT);
static const ble_uuid16_t uuid_dsc_reportref = BLE_UUID16_INIT(UUID_DSC_REPORTREF);
static const ble_uuid16_t uuid_svc_dis = BLE_UUID16_INIT(UUID_SVC_DIS);
static const ble_uuid16_t uuid_chr_pnp = BLE_UUID16_INIT(UUID_CHR_PNP_ID);
static const ble_uuid16_t uuid_svc_bas = BLE_UUID16_INIT(UUID_SVC_BAS);
static const ble_uuid16_t uuid_chr_battery = BLE_UUID16_INIT(UUID_CHR_BATTERY);

/* Characteristic value handles for notifications. */
static uint16_t h_report_kb;
static uint16_t h_report_mouse;
static uint16_t h_report_consumer;
static uint16_t h_battery;

/* Held report state; report senders mutate then notify. */
static HidKbReport s_kb;
static HidMouseReport s_mouse;
static HidConsumerReport s_consumer;
static uint8_t s_battery_level = 100;

static uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;

/* Report Reference descriptor value {report_id, report_type=input}. The report id
 * is passed through the descriptor's arg. */
static int
    dsc_report_ref_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt* ctxt, void* arg) {
    UNUSED(c);
    UNUSED(a);
    if(ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) return BLE_ATT_ERR_UNLIKELY;
    uint8_t ref[2] = {(uint8_t)(uintptr_t)arg, 0x01};
    int rc = os_mbuf_append(ctxt->om, ref, sizeof(ref));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/* Input report characteristic access: READ returns the held report. NOTIFY is
 * driven by the report senders. Which report is selected by arg (report number). */
static int
    chr_report_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt* ctxt, void* arg) {
    UNUSED(c);
    UNUSED(a);
    if(ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    const void* data;
    uint16_t len;
    switch((int)(uintptr_t)arg) {
    case 0:
        data = &s_kb;
        len = sizeof(s_kb);
        break;
    case 1:
        data = &s_mouse;
        len = sizeof(s_mouse);
        break;
    case 2:
        data = &s_consumer;
        len = sizeof(s_consumer);
        break;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
    int rc = os_mbuf_append(ctxt->om, data, len);
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int chr_static_access(
    uint16_t c,
    uint16_t a,
    struct ble_gatt_access_ctxt* ctxt,
    void* arg) {
    UNUSED(c);
    UNUSED(a);
    const ble_uuid_t* uuid = ctxt->chr->uuid;

    if(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if(ble_uuid_cmp(uuid, &uuid_chr_repmap.u) == 0) {
            int rc = os_mbuf_append(ctxt->om, hid_report_map, sizeof(hid_report_map));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else if(ble_uuid_cmp(uuid, &uuid_chr_hidinfo.u) == 0) {
            uint8_t info[4] = {HID_INFO_SPEC_LSB, HID_INFO_SPEC_MSB, HID_INFO_COUNTRY, HID_INFO_FLAGS};
            int rc = os_mbuf_append(ctxt->om, info, sizeof(info));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else if(ble_uuid_cmp(uuid, &uuid_chr_protocol.u) == 0) {
            uint8_t mode = 1; /* report protocol */
            int rc = os_mbuf_append(ctxt->om, &mode, 1);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else if(ble_uuid_cmp(uuid, &uuid_chr_pnp.u) == 0) {
            /* PnP ID: source USB-IF(0x02), VID 0x0483 (ST), PID 0x5741, ver 0x0100. */
            uint8_t pnp[7] = {0x02, 0x83, 0x04, 0x41, 0x57, 0x00, 0x01};
            int rc = os_mbuf_append(ctxt->om, pnp, sizeof(pnp));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else if(ble_uuid_cmp(uuid, &uuid_chr_battery.u) == 0) {
            int rc = os_mbuf_append(ctxt->om, &s_battery_level, 1);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    } else if(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        /* Protocol Mode and HID Control Point are accepted and ignored. */
        UNUSED(arg);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/* Report Reference descriptors, one per input report, carrying the report id. */
static struct ble_gatt_dsc_def dsc_kb[] = {
    {.uuid = &uuid_dsc_reportref.u,
     .att_flags = BLE_ATT_F_READ,
     .access_cb = dsc_report_ref_access,
     .arg = (void*)(uintptr_t)HID_REPORT_ID_KEYBOARD},
    {0},
};
static struct ble_gatt_dsc_def dsc_mouse[] = {
    {.uuid = &uuid_dsc_reportref.u,
     .att_flags = BLE_ATT_F_READ,
     .access_cb = dsc_report_ref_access,
     .arg = (void*)(uintptr_t)HID_REPORT_ID_MOUSE},
    {0},
};
static struct ble_gatt_dsc_def dsc_consumer[] = {
    {.uuid = &uuid_dsc_reportref.u,
     .att_flags = BLE_ATT_F_READ,
     .access_cb = dsc_report_ref_access,
     .arg = (void*)(uintptr_t)HID_REPORT_ID_CONSUMER},
    {0},
};

static const struct ble_gatt_svc_def svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_svc_hid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = &uuid_chr_protocol.u,
                    .access_cb = chr_static_access,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
                },
                {
                    .uuid = &uuid_chr_report.u,
                    .access_cb = chr_report_access,
                    .arg = (void*)(uintptr_t)0, /* keyboard */
                    .val_handle = &h_report_kb,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                    .descriptors = dsc_kb,
                },
                {
                    .uuid = &uuid_chr_report.u,
                    .access_cb = chr_report_access,
                    .arg = (void*)(uintptr_t)1, /* mouse */
                    .val_handle = &h_report_mouse,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                    .descriptors = dsc_mouse,
                },
                {
                    .uuid = &uuid_chr_report.u,
                    .access_cb = chr_report_access,
                    .arg = (void*)(uintptr_t)2, /* consumer */
                    .val_handle = &h_report_consumer,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                    .descriptors = dsc_consumer,
                },
                {
                    .uuid = &uuid_chr_repmap.u,
                    .access_cb = chr_static_access,
                    .flags = BLE_GATT_CHR_F_READ,
                },
                {
                    .uuid = &uuid_chr_hidinfo.u,
                    .access_cb = chr_static_access,
                    .flags = BLE_GATT_CHR_F_READ,
                },
                {
                    .uuid = &uuid_chr_ctrlpoint.u,
                    .access_cb = chr_static_access,
                    .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
                },
                {0},
            },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_svc_dis.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = &uuid_chr_pnp.u,
                    .access_cb = chr_static_access,
                    .flags = BLE_GATT_CHR_F_READ,
                },
                {0},
            },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_svc_bas.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = &uuid_chr_battery.u,
                    .access_cb = chr_static_access,
                    .val_handle = &h_battery,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                },
                {0},
            },
    },
    {0},
};

int hid_gatt_register(void) {
    int rc = ble_gatts_count_cfg(svcs);
    if(rc != 0) {
        FURI_LOG_E(TAG, "count_cfg failed: %d", rc);
        return rc;
    }
    rc = ble_gatts_add_svcs(svcs);
    if(rc != 0) {
        FURI_LOG_E(TAG, "add_svcs failed: %d", rc);
        return rc;
    }
    return 0;
}

void hid_gatt_set_conn(uint16_t conn_handle, bool connected) {
    if(connected) {
        s_conn = conn_handle;
        memset(&s_kb, 0, sizeof(s_kb));
        memset(&s_mouse, 0, sizeof(s_mouse));
        memset(&s_consumer, 0, sizeof(s_consumer));
    } else {
        s_conn = BLE_HS_CONN_HANDLE_NONE;
    }
}

/* Notify the held report on its input characteristic. */
static bool notify_report(uint16_t handle, const void* data, uint16_t len) {
    if(s_conn == BLE_HS_CONN_HANDLE_NONE) return false;
    struct os_mbuf* om = ble_hs_mbuf_from_flat(data, len);
    if(!om) return false;
    return ble_gatts_notify_custom(s_conn, handle, om) == 0;
}

bool hid_gatt_kb_press(uint16_t button) {
    for(uint8_t i = 0; i < HID_KB_MAX_KEYS; i++) {
        if(s_kb.key[i] == 0) {
            s_kb.key[i] = button & 0xFF;
            break;
        }
    }
    s_kb.mods |= (button >> 8);
    return notify_report(h_report_kb, &s_kb, sizeof(s_kb));
}

bool hid_gatt_kb_release(uint16_t button) {
    for(uint8_t i = 0; i < HID_KB_MAX_KEYS; i++) {
        if(s_kb.key[i] == (button & 0xFF)) {
            s_kb.key[i] = 0;
            break;
        }
    }
    s_kb.mods &= ~(button >> 8);
    return notify_report(h_report_kb, &s_kb, sizeof(s_kb));
}

bool hid_gatt_kb_release_all(void) {
    memset(&s_kb, 0, sizeof(s_kb));
    return notify_report(h_report_kb, &s_kb, sizeof(s_kb));
}

bool hid_gatt_consumer_press(uint16_t button) {
    for(uint8_t i = 0; i < HID_CONSUMER_KEYS; i++) {
        if(s_consumer.key[i] == 0) {
            s_consumer.key[i] = button;
            break;
        }
    }
    return notify_report(h_report_consumer, &s_consumer, sizeof(s_consumer));
}

bool hid_gatt_consumer_release(uint16_t button) {
    for(uint8_t i = 0; i < HID_CONSUMER_KEYS; i++) {
        if(s_consumer.key[i] == button) {
            s_consumer.key[i] = 0;
            break;
        }
    }
    return notify_report(h_report_consumer, &s_consumer, sizeof(s_consumer));
}

bool hid_gatt_consumer_release_all(void) {
    memset(&s_consumer, 0, sizeof(s_consumer));
    return notify_report(h_report_consumer, &s_consumer, sizeof(s_consumer));
}

bool hid_gatt_mouse_move(int8_t dx, int8_t dy) {
    s_mouse.x = dx;
    s_mouse.y = dy;
    bool ok = notify_report(h_report_mouse, &s_mouse, sizeof(s_mouse));
    s_mouse.x = 0;
    s_mouse.y = 0;
    return ok;
}

bool hid_gatt_mouse_press(uint8_t button) {
    s_mouse.btn |= button;
    return notify_report(h_report_mouse, &s_mouse, sizeof(s_mouse));
}

bool hid_gatt_mouse_release(uint8_t button) {
    s_mouse.btn &= ~button;
    return notify_report(h_report_mouse, &s_mouse, sizeof(s_mouse));
}

bool hid_gatt_mouse_release_all(void) {
    s_mouse.btn = 0;
    return notify_report(h_report_mouse, &s_mouse, sizeof(s_mouse));
}

bool hid_gatt_mouse_scroll(int8_t delta) {
    s_mouse.wheel = delta;
    bool ok = notify_report(h_report_mouse, &s_mouse, sizeof(s_mouse));
    s_mouse.wheel = 0;
    return ok;
}
