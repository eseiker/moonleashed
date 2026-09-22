/* furi_ble/gatt_server.h on dyn_gatt. Its services have their own owner, so
 * their writes come here and not to the ble_gatt_* shim. */

#include <furi.h>
#include <string.h>

#include <furi_ble/gatt_server.h>
#include <ble_dispatch.h>

#include "dyn_gatt.h"
#include "nimble_glue.h"

_Static_assert(BLE_GATT_SERVER_F_WRITE_AUTHEN == DYN_GATT_F_WRITE_AUTHEN, "flags");
_Static_assert(BLE_GATT_SERVER_VALUE_MAX == DYN_GATT_VALUE_MAX, "value size");

static BleGattServerCallback callback;
static void* context;
static bool started;
static volatile uint32_t own_services; /* bit per dyn_gatt service id */
static volatile uint32_t live_services; /* own_services at the last rebuild */

typedef struct {
    BleGattServerEvent event;
    uint8_t data[];
} EventBlob;

// Dispatch thread
static void deliver(void* blob) {
    EventBlob* b = blob;
    b->event.data = b->event.data_len ? b->data : NULL;
    ble_dispatch_lock();
    if(callback) callback(&b->event, context);
    ble_dispatch_unlock();
}

static void post(const BleGattServerEvent* event, const uint8_t* data, uint16_t len) {
    EventBlob* b = malloc(sizeof(EventBlob) + len);
    b->event = *event;
    b->event.data_len = len;
    if(len) memcpy(b->data, data, len);
    ble_dispatch_post(deliver, b);
}

// Host thread. attr_handle is a value handle, or the CCCD right after it.
static void on_write(uint16_t conn, uint16_t attr_handle, const uint8_t* data, uint16_t len) {
    for(int svc = 0; svc < DYN_GATT_MAX_SVCS; svc++) {
        if(!(own_services & (1u << svc))) continue;
        for(int i = 0; i < DYN_GATT_MAX_CHRS_PER_SVC; i++) {
            int32_t id = svc * DYN_GATT_MAX_CHRS_PER_SVC + i;
            uint16_t decl;
            if(!dyn_gatt_char_handles(id, &decl, NULL)) continue;
            BleGattServerEvent event = {.char_id = id, .connection_handle = conn};
            if(attr_handle == decl + 1) {
                event.type = BleGattServerEventTypeWrite;
                post(&event, data, len);
                return;
            }
            if(attr_handle == decl + 2 && len >= 1) {
                event.type = BleGattServerEventTypeSubscribe;
                event.notify = data[0] & 0x01;
                event.indicate = data[0] & 0x02;
                post(&event, NULL, 0);
                return;
            }
        }
    }
}

static void on_committed(void) {
    live_services = own_services;
    BleGattServerEvent event = {.type = BleGattServerEventTypeCommitted, .char_id = -1};
    post(&event, NULL, 0);
}

static bool to_dyn_uuid(const BleGattServerUuid* in, DynGattUuid* out) {
    if(!in || (in->type != 16 && in->type != 128)) return false;
    out->type = in->type;
    out->u16 = in->uuid16;
    memcpy(out->u128, in->uuid128, sizeof(out->u128));
    return true;
}

static bool is_own(int32_t service_id) {
    return service_id >= 0 && service_id < DYN_GATT_MAX_SVCS &&
           (own_services & (1u << service_id));
}

void ble_gatt_server_init(void) {
    if(started) return;
    ble_dispatch_init();
    nimble_glue_on_app_stop(ble_gatt_server_deinit);
    DynGattHooks hooks = {.on_write = on_write, .on_committed = on_committed};
    dyn_gatt_set_hooks(DYN_GATT_OWNER_SERVER, &hooks);
    started = true;
}

void ble_gatt_server_deinit(void) {
    if(!started) return;
    uint32_t removed = own_services;
    own_services = 0;
    for(int i = 0; i < DYN_GATT_MAX_SVCS; i++) {
        if(removed & (1u << i)) dyn_gatt_service_remove(i);
    }
    dyn_gatt_set_hooks(DYN_GATT_OWNER_SERVER, NULL);
    ble_gatt_server_set_callback(NULL, NULL);
    started = false;
    // A rebuild drops the companion link; skip it when nothing of ours is on the air
    if(removed & live_services) nimble_glue_gatt_rebuild();
}

void ble_gatt_server_set_callback(BleGattServerCallback cb, void* ctx) {
    ble_dispatch_lock();
    callback = cb;
    context = ctx;
    ble_dispatch_unlock();
}

int32_t ble_gatt_server_service_add(const BleGattServerUuid* uuid, bool primary) {
    DynGattUuid duuid;
    if(!started || !to_dyn_uuid(uuid, &duuid)) return -1;
    int svc = dyn_gatt_service_add(&duuid, primary, DYN_GATT_OWNER_SERVER);
    if(svc >= 0) own_services |= 1u << svc;
    return svc;
}

bool ble_gatt_server_service_remove(int32_t service_id) {
    if(!is_own(service_id)) return false;
    own_services &= ~(1u << service_id);
    return dyn_gatt_service_remove(service_id);
}

int32_t ble_gatt_server_char_add(
    int32_t service_id,
    const BleGattServerUuid* uuid,
    uint16_t flags,
    uint16_t max_len,
    const uint8_t* init_value,
    uint16_t init_len) {
    DynGattUuid duuid;
    if(!is_own(service_id) || !to_dyn_uuid(uuid, &duuid)) return -1;
    return dyn_gatt_char_add(
        service_id, &duuid, flags, max_len, init_value, init_len, NULL, NULL, 0);
}

bool ble_gatt_server_char_remove(int32_t char_id) {
    return is_own(dyn_gatt_char_service(char_id)) && dyn_gatt_char_remove(char_id);
}

bool ble_gatt_server_char_set_value(int32_t char_id, const uint8_t* data, uint16_t len) {
    return is_own(dyn_gatt_char_service(char_id)) && dyn_gatt_char_set_value(char_id, data, len);
}

bool ble_gatt_server_char_handles(int32_t char_id, uint16_t* decl_handle, uint16_t* value_handle) {
    uint16_t decl = 0;
    if(!is_own(dyn_gatt_char_service(char_id))) return false;
    dyn_gatt_char_handles(char_id, &decl, NULL);
    if(decl_handle) *decl_handle = decl;
    if(value_handle) *value_handle = decl ? decl + 1 : 0;
    return true;
}

bool ble_gatt_server_commit(void) {
    if(!started) return false;
    // Counted as live now: a deinit before the rebuild runs must still rebuild
    live_services |= own_services;
    nimble_glue_gatt_rebuild();
    return true;
}
