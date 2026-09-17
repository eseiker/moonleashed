/*
 * Neutral GATT client core over NimBLE's ble_gattc_* procedures (TASK-633).
 * See gattc_glue.h. The firmware furi_ble/gatt_client.c adapter maps these
 * events onto the Moon-Firmware-compatible ble_gatt_client_* API (KNOW-636).
 *
 * One discovery/read/write runs at a time; results accumulate in static buffers
 * and are emitted on completion. Callbacks run on the NimBLE host thread.
 */

#include <furi.h>
#include <string.h>

#include "nimble/ble.h"
#include "os/os_mbuf.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

#include "gattc_glue.h"

#define TAG "GattcGlue"

static GattcApiCallback gattc_cb;
static void* gattc_ctx;

/* Discovery accumulators, allocated together on first use so a device with no
 * GATT client consumer keeps the 2 KB (TASK-707). Never freed: the NimBLE host
 * thread fills them (KNOW-703). */
static GattcApiService* gattc_svcs;
static uint8_t gattc_svc_count;
static GattcApiChar* gattc_chrs;
static uint8_t gattc_chr_count;

static bool gattc_tables_get(void) {
    if(!gattc_svcs) gattc_svcs = malloc(sizeof(GattcApiService) * GATTC_MAX_SERVICES);
    if(!gattc_chrs) gattc_chrs = malloc(sizeof(GattcApiChar) * GATTC_MAX_CHARS);
    return gattc_svcs && gattc_chrs;
}
/* Read/notify staging buffer, allocated on first use so a device with no GATT
 * client consumer keeps it (TASK-707). Never freed: the NimBLE host thread
 * reads it (KNOW-703). */
#define GATTC_BUF_SIZE 512
static uint8_t* gattc_buf;

static uint8_t* gattc_buf_get(void) {
    if(!gattc_buf) gattc_buf = malloc(GATTC_BUF_SIZE);
    return gattc_buf;
}

static void gattc_emit(const GattcApiEvent* ev) {
    if(gattc_cb) gattc_cb(ev, gattc_ctx);
}

static void gattc_emit_error(uint16_t conn, uint8_t code) {
    GattcApiEvent ev = {.type = GattcApiError, .conn_handle = conn, .error_code = code};
    gattc_emit(&ev);
}

/* Map a NimBLE UUID into the neutral 16/128-bit fields. */
static void gattc_map_uuid(
    const ble_uuid_any_t* u,
    uint8_t* type,
    uint16_t* u16,
    uint8_t* u128) {
    if(u->u.type == BLE_UUID_TYPE_16) {
        *type = 1;
        *u16 = ble_uuid_u16(&u->u);
    } else {
        *type = 2;
        memcpy(u128, u->u128.value, 16);
    }
}

/* ---- Service discovery ---------------------------------------------------- */

static int gattc_on_svc(
    uint16_t conn_handle,
    const struct ble_gatt_error* error,
    const struct ble_gatt_svc* service,
    void* arg) {
    UNUSED(arg);
    if(error->status == 0 && service) {
        if(gattc_tables_get() && gattc_svc_count < GATTC_MAX_SERVICES) {
            GattcApiService* s = &gattc_svcs[gattc_svc_count++];
            memset(s, 0, sizeof(*s));
            s->start_handle = service->start_handle;
            s->end_handle = service->end_handle;
            gattc_map_uuid(&service->uuid, &s->uuid_type, &s->uuid_16, s->uuid_128);
        }
        return 0;
    }
    if(error->status == BLE_HS_EDONE) {
        GattcApiEvent ev = {
            .type = GattcApiServicesDiscovered,
            .conn_handle = conn_handle,
            .services = gattc_svcs,
            .count = gattc_svc_count};
        gattc_emit(&ev);
    } else {
        gattc_emit_error(conn_handle, (uint8_t)error->status);
    }
    return 0;
}

/* ---- Characteristic discovery --------------------------------------------- */

static int gattc_on_chr(
    uint16_t conn_handle,
    const struct ble_gatt_error* error,
    const struct ble_gatt_chr* chr,
    void* arg) {
    UNUSED(arg);
    if(error->status == 0 && chr) {
        if(gattc_tables_get() && gattc_chr_count < GATTC_MAX_CHARS) {
            GattcApiChar* c = &gattc_chrs[gattc_chr_count++];
            memset(c, 0, sizeof(*c));
            c->decl_handle = chr->def_handle;
            c->value_handle = chr->val_handle;
            c->properties = chr->properties;
            gattc_map_uuid(&chr->uuid, &c->uuid_type, &c->uuid_16, c->uuid_128);
        }
        return 0;
    }
    if(error->status == BLE_HS_EDONE) {
        GattcApiEvent ev = {
            .type = GattcApiCharsDiscovered,
            .conn_handle = conn_handle,
            .chars = gattc_chrs,
            .count = gattc_chr_count};
        gattc_emit(&ev);
    } else {
        gattc_emit_error(conn_handle, (uint8_t)error->status);
    }
    return 0;
}

/* ---- Read / Write --------------------------------------------------------- */

static int gattc_on_read(
    uint16_t conn_handle,
    const struct ble_gatt_error* error,
    struct ble_gatt_attr* attr,
    void* arg) {
    UNUSED(arg);
    if(error->status != 0 || !attr) {
        gattc_emit_error(conn_handle, (uint8_t)error->status);
        return 0;
    }
    uint16_t len = OS_MBUF_PKTLEN(attr->om);
    uint8_t* buf = gattc_buf_get();
    if(!buf) return 0;
    if(len > GATTC_BUF_SIZE) len = GATTC_BUF_SIZE;
    if(len) os_mbuf_copydata(attr->om, 0, len, buf);
    GattcApiEvent ev = {
        .type = GattcApiReadComplete,
        .conn_handle = conn_handle,
        .data = buf,
        .data_len = len,
        .value_handle = attr->handle};
    gattc_emit(&ev);
    return 0;
}

static int gattc_on_write(
    uint16_t conn_handle,
    const struct ble_gatt_error* error,
    struct ble_gatt_attr* attr,
    void* arg) {
    UNUSED(arg);
    if(error->status != 0) {
        gattc_emit_error(conn_handle, (uint8_t)error->status);
        return 0;
    }
    GattcApiEvent ev = {
        .type = GattcApiWriteComplete,
        .conn_handle = conn_handle,
        .value_handle = attr ? attr->handle : 0};
    gattc_emit(&ev);
    return 0;
}

static int gattc_on_mtu(
    uint16_t conn_handle,
    const struct ble_gatt_error* error,
    uint16_t mtu,
    void* arg) {
    UNUSED(arg);
    FURI_LOG_I(TAG, "MTU exchanged conn=%u mtu=%u status=%d", conn_handle, mtu, error->status);
    return 0;
}

/* ---- Public API ----------------------------------------------------------- */

void gattc_api_init(GattcApiCallback dispatch, void* context) {
    gattc_cb = dispatch;
    gattc_ctx = context;
    gattc_svc_count = 0;
    gattc_chr_count = 0;
    FURI_LOG_I(TAG, "gattc_api initialized");
}

void gattc_api_deinit(void) {
    gattc_cb = NULL;
    gattc_ctx = NULL;
}

bool gattc_api_discover_services(uint16_t conn_handle) {
    gattc_svc_count = 0;
    return ble_gattc_disc_all_svcs(conn_handle, gattc_on_svc, NULL) == 0;
}

bool gattc_api_discover_characteristics(
    uint16_t conn_handle,
    uint16_t start_handle,
    uint16_t end_handle) {
    gattc_chr_count = 0;
    return ble_gattc_disc_all_chrs(conn_handle, start_handle, end_handle, gattc_on_chr, NULL) == 0;
}

bool gattc_api_read(uint16_t conn_handle, uint16_t value_handle) {
    return ble_gattc_read(conn_handle, value_handle, gattc_on_read, NULL) == 0;
}

bool gattc_api_write(
    uint16_t conn_handle,
    uint16_t value_handle,
    const uint8_t* data,
    uint16_t len) {
    return ble_gattc_write_flat(conn_handle, value_handle, data, len, gattc_on_write, NULL) == 0;
}

bool gattc_api_exchange_mtu(uint16_t conn_handle) {
    return ble_gattc_exchange_mtu(conn_handle, gattc_on_mtu, NULL) == 0;
}

bool gattc_api_subscribe(uint16_t conn_handle, uint16_t value_handle, bool enable) {
    /* Write the Client Characteristic Configuration descriptor, assumed to sit
     * at value_handle + 1 (the common layout). enable -> Notifications bit. */
    uint8_t val[2] = {enable ? 0x01 : 0x00, 0x00};
    return ble_gattc_write_flat(conn_handle, value_handle + 1, val, sizeof(val), gattc_on_write, NULL) ==
           0;
}

void gattc_api_on_notify(
    uint16_t conn_handle,
    uint16_t attr_handle,
    const uint8_t* data,
    uint16_t len) {
    GattcApiEvent ev = {
        .type = GattcApiNotification,
        .conn_handle = conn_handle,
        .data = data,
        .data_len = len,
        .value_handle = attr_handle};
    gattc_emit(&ev);
}
