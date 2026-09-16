/*
 * Runtime GATT server API (TASK-666), backed by the resident NimBLE host's
 * dyn_gatt registry. See gatt_server.h.
 *
 * Services added here are owned by DYN_GATT_OWNER_DIRECT, so their writes go to
 * this API and not to the stock ble_gatt_* shim, which owns
 * DYN_GATT_OWNER_SHIM. Events are copied into a heap blob on the NimBLE host
 * thread and delivered on the BLE dispatch thread (KNOW-654).
 */

#include "gatt_server.h"

#include <dyn_gatt.h>
#include <nimble_glue.h>
#include <ble_dispatch.h>
#include <furi.h>
#include <string.h>

#define TAG "BleGattServer"

_Static_assert(BLE_GATT_SERVER_F_READ == DYN_GATT_F_READ, "flag mismatch");
_Static_assert(BLE_GATT_SERVER_F_WRITE == DYN_GATT_F_WRITE, "flag mismatch");
_Static_assert(BLE_GATT_SERVER_F_NOTIFY == DYN_GATT_F_NOTIFY, "flag mismatch");
_Static_assert(BLE_GATT_SERVER_F_INDICATE == DYN_GATT_F_INDICATE, "flag mismatch");
_Static_assert(BLE_GATT_SERVER_VALUE_MAX == DYN_GATT_VALUE_MAX, "value bound mismatch");

static BleGattServerCallback s_callback;
static void* s_context;
static bool s_started;
/* Services this API added, so deinit removes only ours. */
static uint32_t s_own_svcs;

typedef struct {
    BleGattServerEvent event;
    uint8_t data[];
} GattServerEventBlob;

/* Dispatch thread. */
static void gatt_server_deliver(void* blob) {
    GattServerEventBlob* b = blob;
    if(b->event.data_len) b->event.data = b->data;
    ble_dispatch_lock();
    if(s_callback) s_callback(&b->event, s_context);
    ble_dispatch_unlock();
}

static void gatt_server_post(const BleGattServerEvent* event, const uint8_t* data, uint16_t len) {
    GattServerEventBlob* b = malloc(sizeof(GattServerEventBlob) + len);
    b->event = *event;
    b->event.data = NULL;
    b->event.data_len = len;
    if(len) memcpy(b->data, data, len);
    ble_dispatch_post(gatt_server_deliver, b);
}

/* ---- dyn_gatt hooks (NimBLE host thread) ---------------------------------- */

static void gatt_server_on_write(
    int chr_id,
    DynGattWriteKind kind,
    uint16_t conn_handle,
    uint16_t attr_handle,
    const uint8_t* data,
    uint16_t len,
    void* ctx) {
    UNUSED(attr_handle);
    UNUSED(ctx);
    BleGattServerEvent event = {0};
    event.char_id = chr_id;
    event.connection_handle = conn_handle;

    if(kind == DynGattWriteCccd) {
        event.type = BleGattServerEventTypeSubscribe;
        uint16_t cccd = len >= 2 ? (uint16_t)(data[0] | ((uint16_t)data[1] << 8)) : 0;
        event.notify = (cccd & 0x0001) != 0;
        event.indicate = (cccd & 0x0002) != 0;
        gatt_server_post(&event, NULL, 0);
        return;
    }
    event.type = BleGattServerEventTypeWrite;
    gatt_server_post(&event, data, len);
}

static void gatt_server_on_committed(void* ctx) {
    UNUSED(ctx);
    BleGattServerEvent event = {0};
    event.type = BleGattServerEventTypeCommitted;
    event.char_id = -1;
    gatt_server_post(&event, NULL, 0);
}

/* ---- API ------------------------------------------------------------------ */

static bool to_dyn_uuid(const BleGattServerUuid* in, DynGattUuid* out) {
    if(!in) return false;
    memset(out, 0, sizeof(*out));
    out->type = in->type;
    out->u16 = in->uuid16;
    memcpy(out->u128, in->uuid128, sizeof(out->u128));
    return in->type == 16 || in->type == 128;
}

void ble_gatt_server_init(void) {
    if(s_started) return;
    ble_dispatch_init();
    DynGattHooks hooks = {
        .on_write = gatt_server_on_write,
        .on_indicate_done = NULL,
        .on_committed = gatt_server_on_committed,
        .ctx = NULL,
    };
    dyn_gatt_set_owner_hooks(DYN_GATT_OWNER_DIRECT, &hooks);
    s_started = true;
    FURI_LOG_I(TAG, "runtime GATT server ready");
}

void ble_gatt_server_deinit(void) {
    if(!s_started) return;
    bool removed = false;
    for(int i = 0; i < DYN_GATT_MAX_SVCS; i++) {
        if(s_own_svcs & (1u << i)) {
            dyn_gatt_service_remove(i);
            removed = true;
        }
    }
    s_own_svcs = 0;
    dyn_gatt_set_owner_hooks(DYN_GATT_OWNER_DIRECT, NULL);
    ble_dispatch_lock();
    s_callback = NULL;
    s_context = NULL;
    ble_dispatch_unlock();
    s_started = false;
    if(removed) nimble_glue_gatt_rebuild_request();
}

void ble_gatt_server_set_callback(BleGattServerCallback callback, void* context) {
    ble_dispatch_lock();
    s_callback = callback;
    s_context = context;
    ble_dispatch_unlock();
}

int32_t ble_gatt_server_service_add(const BleGattServerUuid* uuid, bool primary) {
    if(!s_started) ble_gatt_server_init();
    DynGattUuid duuid;
    if(!to_dyn_uuid(uuid, &duuid)) return -1;
    int svc = dyn_gatt_service_add_owned(&duuid, primary, DYN_GATT_OWNER_DIRECT);
    if(svc >= 0) s_own_svcs |= (1u << svc);
    return svc;
}

bool ble_gatt_server_service_remove(int32_t service_id) {
    if(service_id < 0 || service_id >= DYN_GATT_MAX_SVCS) return false;
    if(!(s_own_svcs & (1u << service_id))) return false;
    s_own_svcs &= ~(1u << service_id);
    return dyn_gatt_service_remove((int)service_id);
}

int32_t ble_gatt_server_char_add(
    int32_t service_id,
    const BleGattServerUuid* uuid,
    uint16_t flags,
    uint16_t max_len,
    const uint8_t* init_value,
    uint16_t init_len) {
    if(service_id < 0 || service_id >= DYN_GATT_MAX_SVCS) return -1;
    if(!(s_own_svcs & (1u << service_id))) return -1;
    DynGattUuid duuid;
    if(!to_dyn_uuid(uuid, &duuid)) return -1;
    return dyn_gatt_char_add(
        (int)service_id, &duuid, flags, max_len, init_value, init_len, NULL, NULL, 0);
}

bool ble_gatt_server_char_remove(int32_t char_id) {
    return dyn_gatt_char_remove((int)char_id);
}

bool ble_gatt_server_char_set_value(int32_t char_id, const uint8_t* data, uint16_t len) {
    return dyn_gatt_char_set_value((int)char_id, data, len);
}

bool ble_gatt_server_char_handles(int32_t char_id, uint16_t* decl_handle, uint16_t* value_handle) {
    uint16_t decl = 0;
    uint16_t dsc = 0;
    if(!dyn_gatt_char_handles((int)char_id, &decl, &dsc)) return false;
    if(decl_handle) *decl_handle = decl;
    /* Handle layout: declaration H, value H + 1 (dyn_gatt.h). */
    if(value_handle) *value_handle = decl ? (uint16_t)(decl + 1) : 0;
    return true;
}

bool ble_gatt_server_commit(void) {
    if(!s_started) return false;
    nimble_glue_gatt_rebuild_request();
    return true;
}
