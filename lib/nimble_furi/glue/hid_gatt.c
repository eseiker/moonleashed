/* HID over GATT for lib/ble_profile's HID profile: report ids 1 keyboard,
 * 2 mouse, 3 consumer. The report map comes from the profile. Device
 * Information and Battery come from info_gatt.c. */

#include <furi.h>
#include <string.h>

#include "os/os_mbuf.h"
#include "nimble/ble.h"
#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "host/ble_att.h"
#include "services/gatt/ble_svc_gatt.h"

#include "hid_gatt.h"

#define TAG "HidGatt"

#define HID_REPORT_ID_KEYBOARD 1
#define HID_REPORT_ID_MOUSE    2
#define HID_REPORT_ID_CONSUMER 3

#define HID_KB_MAX_KEYS   6
#define HID_CONSUMER_KEYS 1

/* bcdHID 1.01, no country, remote wake and normally connectable. */
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

#define UUID_SVC_HID       0x1812
#define UUID_CHR_PROTOCOL  0x2A4E
#define UUID_CHR_REPORT    0x2A4D
#define UUID_CHR_REPMAP    0x2A4B
#define UUID_CHR_HIDINFO   0x2A4A
#define UUID_CHR_CTRLPOINT 0x2A4C
#define UUID_DSC_REPORTREF 0x2908

static const ble_uuid16_t uuid_svc_hid = BLE_UUID16_INIT(UUID_SVC_HID);
static const ble_uuid16_t uuid_chr_protocol = BLE_UUID16_INIT(UUID_CHR_PROTOCOL);
static const ble_uuid16_t uuid_chr_report = BLE_UUID16_INIT(UUID_CHR_REPORT);
static const ble_uuid16_t uuid_chr_repmap = BLE_UUID16_INIT(UUID_CHR_REPMAP);
static const ble_uuid16_t uuid_chr_hidinfo = BLE_UUID16_INIT(UUID_CHR_HIDINFO);
static const ble_uuid16_t uuid_chr_ctrlpoint = BLE_UUID16_INIT(UUID_CHR_CTRLPOINT);
static const ble_uuid16_t uuid_dsc_reportref = BLE_UUID16_INIT(UUID_DSC_REPORTREF);

static uint16_t h_report_kb;
static uint16_t h_report_mouse;
static uint16_t h_report_consumer;

/* Last value of each report, for reads. */
static HidKbReport s_kb;
static HidMouseReport s_mouse;
static HidConsumerReport s_consumer;

static uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
/* Bit per report, in chr_report_access order. */
static volatile uint8_t s_subscribed;
static bool s_visible;

static FuriMutex* s_map_mtx;
static uint8_t* s_map;
static uint16_t s_map_len;

static void hid_tx_init(void);

/* Report Reference: {report id from arg, input}. */
static int
    dsc_report_ref_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt* ctxt, void* arg) {
    UNUSED(c);
    UNUSED(a);
    if(ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) return BLE_ATT_ERR_UNLIKELY;
    uint8_t ref[2] = {(uint8_t)(uintptr_t)arg, 0x01};
    int rc = os_mbuf_append(ctxt->om, ref, sizeof(ref));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

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

static int
    chr_static_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt* ctxt, void* arg) {
    UNUSED(c);
    UNUSED(a);
    UNUSED(arg);
    const ble_uuid_t* uuid = ctxt->chr->uuid;

    if(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if(ble_uuid_cmp(uuid, &uuid_chr_repmap.u) == 0) {
            furi_mutex_acquire(s_map_mtx, FuriWaitForever);
            int rc = s_map ? os_mbuf_append(ctxt->om, s_map, s_map_len) : 0;
            furi_mutex_release(s_map_mtx);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else if(ble_uuid_cmp(uuid, &uuid_chr_hidinfo.u) == 0) {
            uint8_t info[4] = {
                HID_INFO_SPEC_LSB, HID_INFO_SPEC_MSB, HID_INFO_COUNTRY, HID_INFO_FLAGS};
            int rc = os_mbuf_append(ctxt->om, info, sizeof(info));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else if(ble_uuid_cmp(uuid, &uuid_chr_protocol.u) == 0) {
            uint8_t mode = 1; /* Report Protocol */
            int rc = os_mbuf_append(ctxt->om, &mode, 1);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    } else if(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        /* Protocol Mode and Control Point writes are ignored. */
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

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
    {0},
};

int hid_gatt_register(void) {
    hid_tx_init();
    if(!s_map_mtx) s_map_mtx = furi_mutex_alloc(FuriMutexTypeNormal);
    s_visible = true;
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
    s_subscribed = 0;
    if(connected) {
        s_conn = conn_handle;
        memset(&s_kb, 0, sizeof(s_kb));
        memset(&s_mouse, 0, sizeof(s_mouse));
        memset(&s_consumer, 0, sizeof(s_consumer));
    } else {
        s_conn = BLE_HS_CONN_HANDLE_NONE;
    }
}

void hid_gatt_on_subscribe(uint16_t attr_handle, bool subscribed) {
    uint8_t bit = attr_handle == h_report_kb       ? 1 :
                  attr_handle == h_report_mouse    ? 2 :
                  attr_handle == h_report_consumer ? 4 :
                                                     0;
    if(subscribed) {
        s_subscribed |= bit;
    } else {
        s_subscribed &= ~bit;
    }
}

void hid_gatt_set_visible(bool visible, bool announce) {
    uint16_t handle;
    if(visible == s_visible || ble_gatts_find_svc(&uuid_svc_hid.u, &handle) != 0) return;
    if(ble_gatts_svc_set_visibility(handle, visible) != 0) return;
    s_visible = visible;
    if(announce) ble_svc_gatt_changed(handle, 0xFFFF);
}

void hid_gatt_set_report_map(const uint8_t* data, uint16_t len) {
    uint8_t* map = malloc(len);
    memcpy(map, data, len);
    furi_mutex_acquire(s_map_mtx, FuriWaitForever);
    uint8_t* old = s_map;
    s_map = map;
    s_map_len = len;
    furi_mutex_release(s_map_mtx);
    free(old);
}

/* Reports are sent from the host thread: a notify from a 2 KB app stack
 * overflows it. */
#define HID_TX_QUEUE_LEN 16

typedef struct {
    uint16_t handle;
    uint16_t len;
    uint8_t data[8];
} HidTxItem;

static HidTxItem s_txq[HID_TX_QUEUE_LEN];
static volatile uint8_t s_txq_head;
static volatile uint8_t s_txq_tail;
static FuriMutex* s_txq_mtx;
static struct ble_npl_event s_tx_event;
static bool s_tx_ready;

static void hid_tx_event_cb(struct ble_npl_event* ev) {
    UNUSED(ev);
    for(;;) {
        HidTxItem item;
        furi_mutex_acquire(s_txq_mtx, FuriWaitForever);
        if(s_txq_head == s_txq_tail) {
            furi_mutex_release(s_txq_mtx);
            break;
        }
        item = s_txq[s_txq_tail];
        s_txq_tail = (s_txq_tail + 1) % HID_TX_QUEUE_LEN;
        furi_mutex_release(s_txq_mtx);

        if(s_conn == BLE_HS_CONN_HANDLE_NONE) continue;
        struct os_mbuf* om = ble_hs_mbuf_from_flat(item.data, item.len);
        if(!om) {
            FURI_LOG_E(TAG, "tx_cb mbuf fail");
            continue;
        }
        int rc = ble_gatts_notify_custom(s_conn, item.handle, om);
        if(rc != 0) FURI_LOG_W(TAG, "Report notify failed: %d", rc);
    }
}

static void hid_tx_init(void) {
    if(s_tx_ready) return;
    if(!s_txq_mtx) s_txq_mtx = furi_mutex_alloc(FuriMutexTypeNormal);
    ble_npl_event_init(&s_tx_event, hid_tx_event_cb, NULL);
    s_txq_head = 0;
    s_txq_tail = 0;
    s_tx_ready = true;
}

static bool notify_report(uint16_t handle, uint8_t bit, const void* data, uint16_t len) {
    if(!s_tx_ready || handle == 0 || !(s_subscribed & bit)) return false;
    if(len > sizeof(((HidTxItem*)0)->data)) len = sizeof(((HidTxItem*)0)->data);

    furi_mutex_acquire(s_txq_mtx, FuriWaitForever);
    uint8_t next = (s_txq_head + 1) % HID_TX_QUEUE_LEN;
    if(next == s_txq_tail) {
        furi_mutex_release(s_txq_mtx); /* full: drop the report */
        return false;
    }
    s_txq[s_txq_head].handle = handle;
    s_txq[s_txq_head].len = len;
    memcpy(s_txq[s_txq_head].data, data, len);
    s_txq_head = next;
    furi_mutex_release(s_txq_mtx);

    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &s_tx_event);
    return true;
}

bool hid_gatt_input_report(uint8_t report_id, const uint8_t* data, uint16_t len) {
    if(!data || len == 0) return false;

    uint16_t handle;
    uint8_t bit;
    void* held;
    uint16_t held_len;
    switch(report_id) {
    case HID_REPORT_ID_KEYBOARD:
        handle = h_report_kb;
        bit = 1;
        held = &s_kb;
        held_len = sizeof(s_kb);
        break;
    case HID_REPORT_ID_MOUSE:
        handle = h_report_mouse;
        bit = 2;
        held = &s_mouse;
        held_len = sizeof(s_mouse);
        break;
    case HID_REPORT_ID_CONSUMER:
        handle = h_report_consumer;
        bit = 4;
        held = &s_consumer;
        held_len = sizeof(s_consumer);
        break;
    default:
        FURI_LOG_W(TAG, "unknown report id %u", report_id);
        return false;
    }

    if(len > held_len) len = held_len;
    memcpy(held, data, len);
    FURI_LOG_D(TAG, "input report id=%u len=%u", report_id, len);
    return notify_report(handle, bit, data, len);
}
