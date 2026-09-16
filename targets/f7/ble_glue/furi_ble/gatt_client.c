/*
 * Moon-Firmware-compatible ble_gatt_client_* API (KNOW-636), backed by the
 * resident NimBLE host's ble_gattc_* procedures instead of ST ACI. This
 * firmware-side adapter forwards to the neutral GATT client core in the NimBLE
 * glue (gattc_api_*, lib/nimble/glue/gattc_glue.c) and maps its events onto the
 * BleGattClientEvent the FAP-facing API defines (TASK-633). A FAP written for
 * Moon-Firmware's ble_gatt_client_* recompiles against this unchanged.
 *
 * Notifications reach this module through the central link's GAP handler
 * (nimble_glue), which calls gattc_api_on_notify. Registered callbacks run on
 * the BLE dispatch thread (ble_dispatch.h), never on the NimBLE host thread; the
 * discovered arrays and read/notify bytes are copied with each event. After
 * set_callback(h, NULL, ...) or deinit returns, that callback is not invoked
 * again.
 */

#include "gatt_client.h"

#include <gattc_glue.h>
#include <ble_dispatch.h>
#include <furi.h>
#include <string.h>

#define TAG "BleGattClient"

#define GATT_CLIENT_MAX_CONNECTIONS 2

typedef struct {
    uint16_t connection_handle;
    bool active;
    BleGattClientCallback callback;
    void* context;
} GattClientConnection;

static GattClientConnection gc_connections[GATT_CLIENT_MAX_CONNECTIONS];

static GattClientConnection* gc_find(uint16_t connection_handle) {
    for(int i = 0; i < GATT_CLIENT_MAX_CONNECTIONS; i++) {
        if(gc_connections[i].active && gc_connections[i].connection_handle == connection_handle) {
            return &gc_connections[i];
        }
    }
    if(connection_handle != 0) {
        for(int i = 0; i < GATT_CLIENT_MAX_CONNECTIONS; i++) {
            if(gc_connections[i].active && gc_connections[i].connection_handle == 0) {
                return &gc_connections[i];
            }
        }
    }
    return NULL;
}

static GattClientConnection* gc_alloc(uint16_t connection_handle) {
    GattClientConnection* existing = gc_find(connection_handle);
    if(existing) return existing;
    for(int i = 0; i < GATT_CLIENT_MAX_CONNECTIONS; i++) {
        if(!gc_connections[i].active) {
            memset(&gc_connections[i], 0, sizeof(GattClientConnection));
            gc_connections[i].connection_handle = connection_handle;
            gc_connections[i].active = true;
            return &gc_connections[i];
        }
    }
    FURI_LOG_E(TAG, "No free GATT client connection slots");
    return NULL;
}

static void gc_free(uint16_t connection_handle) {
    for(int i = 0; i < GATT_CLIENT_MAX_CONNECTIONS; i++) {
        if(gc_connections[i].active && gc_connections[i].connection_handle == connection_handle) {
            gc_connections[i].active = false;
        }
    }
}

/* An event copied off the NimBLE host thread. Array/data pointers in `event`
 * point into `payload`, so they stay valid for the whole callback. */
typedef struct {
    BleGattClientEvent event;
    uint8_t payload[];
} GattClientEventBlob;

/* Dispatch thread: route to the callback registered for the connection now. */
static void gc_deliver(void* blob) {
    GattClientEventBlob* b = blob;
    ble_dispatch_lock();
    GattClientConnection* conn = gc_find(b->event.connection_handle);
    if(conn && conn->callback) conn->callback(&b->event, conn->context);
    ble_dispatch_unlock();
}

/* NimBLE host thread: translate, copy into one heap blob, post. */
static void gc_dispatch(const GattcApiEvent* ev, void* context) {
    UNUSED(context);
    uint8_t n = 0;
    size_t extra = 0;
    switch(ev->type) {
    case GattcApiServicesDiscovered:
        n = ev->count > BLE_GATT_CLIENT_MAX_SERVICES ? BLE_GATT_CLIENT_MAX_SERVICES : ev->count;
        extra = n * sizeof(BleGattService);
        break;
    case GattcApiCharsDiscovered:
        n = ev->count > BLE_GATT_CLIENT_MAX_CHARS ? BLE_GATT_CLIENT_MAX_CHARS : ev->count;
        extra = n * sizeof(BleGattCharacteristic);
        break;
    case GattcApiReadComplete:
    case GattcApiNotification:
        extra = ev->data ? ev->data_len : 0;
        break;
    default:
        break;
    }

    GattClientEventBlob* b = malloc(sizeof(GattClientEventBlob) + extra);
    memset(&b->event, 0, sizeof(b->event));
    b->event.connection_handle = ev->conn_handle;

    switch(ev->type) {
    case GattcApiServicesDiscovered: {
        BleGattService* svcs = (BleGattService*)b->payload;
        for(uint8_t i = 0; i < n; i++) {
            svcs[i].uuid_type = ev->services[i].uuid_type;
            svcs[i].uuid_16 = ev->services[i].uuid_16;
            memcpy(svcs[i].uuid_128, ev->services[i].uuid_128, 16);
            svcs[i].start_handle = ev->services[i].start_handle;
            svcs[i].end_handle = ev->services[i].end_handle;
        }
        b->event.type = BleGattClientEventDiscoverComplete;
        b->event.discover.services = svcs;
        b->event.discover.count = n;
        break;
    }
    case GattcApiCharsDiscovered: {
        BleGattCharacteristic* chrs = (BleGattCharacteristic*)b->payload;
        for(uint8_t i = 0; i < n; i++) {
            chrs[i].uuid_type = ev->chars[i].uuid_type;
            chrs[i].uuid_16 = ev->chars[i].uuid_16;
            memcpy(chrs[i].uuid_128, ev->chars[i].uuid_128, 16);
            chrs[i].decl_handle = ev->chars[i].decl_handle;
            chrs[i].value_handle = ev->chars[i].value_handle;
            chrs[i].properties = ev->chars[i].properties;
        }
        b->event.type = BleGattClientEventCharDiscoverComplete;
        b->event.char_discover.chars = chrs;
        b->event.char_discover.count = n;
        break;
    }
    case GattcApiReadComplete:
        b->event.type = BleGattClientEventReadComplete;
        if(extra) memcpy(b->payload, ev->data, extra);
        b->event.read.data = b->payload;
        b->event.read.data_len = extra;
        b->event.read.value_handle = ev->value_handle;
        break;
    case GattcApiWriteComplete:
        b->event.type = BleGattClientEventWriteComplete;
        break;
    case GattcApiNotification:
        b->event.type = BleGattClientEventNotification;
        if(extra) memcpy(b->payload, ev->data, extra);
        b->event.notification.data = b->payload;
        b->event.notification.data_len = extra;
        b->event.notification.value_handle = ev->value_handle;
        b->event.notification.offset = 0;
        break;
    case GattcApiError:
        b->event.type = BleGattClientEventError;
        b->event.error.error_code = ev->error_code;
        break;
    default:
        free(b);
        return;
    }
    ble_dispatch_post(gc_deliver, b);
}

void ble_gatt_client_init(void) {
    ble_dispatch_init();
    ble_dispatch_lock();
    memset(gc_connections, 0, sizeof(gc_connections));
    ble_dispatch_unlock();
    gattc_api_init(gc_dispatch, NULL);
    FURI_LOG_I(TAG, "GATT client initialized (NimBLE-backed)");
}

void ble_gatt_client_deinit(void) {
    gattc_api_deinit();
    /* After this returns no FAP callback runs: queued events find no slot. */
    ble_dispatch_lock();
    memset(gc_connections, 0, sizeof(gc_connections));
    ble_dispatch_unlock();
}

void ble_gatt_client_set_callback(
    uint16_t connection_handle,
    BleGattClientCallback callback,
    void* context) {
    ble_dispatch_lock();
    if(callback) {
        GattClientConnection* conn = gc_alloc(connection_handle);
        if(conn) {
            conn->callback = callback;
            conn->context = context;
        }
    } else {
        gc_free(connection_handle);
    }
    ble_dispatch_unlock();
}

bool ble_gatt_client_discover_services(uint16_t connection_handle) {
    return gattc_api_discover_services(connection_handle);
}

bool ble_gatt_client_discover_characteristics(
    uint16_t connection_handle,
    const BleGattService* service) {
    if(!service) return false;
    return gattc_api_discover_characteristics(
        connection_handle, service->start_handle, service->end_handle);
}

bool ble_gatt_client_read(uint16_t connection_handle, uint16_t value_handle) {
    return gattc_api_read(connection_handle, value_handle);
}

bool ble_gatt_client_write(
    uint16_t connection_handle,
    uint16_t value_handle,
    const uint8_t* data,
    uint16_t data_len) {
    return gattc_api_write(connection_handle, value_handle, data, data_len);
}

bool ble_gatt_client_exchange_mtu(uint16_t connection_handle) {
    return gattc_api_exchange_mtu(connection_handle);
}

bool ble_gatt_client_subscribe_notifications(
    uint16_t connection_handle,
    uint16_t value_handle,
    bool enable) {
    return gattc_api_subscribe(connection_handle, value_handle, enable);
}
