/*
 * Moon-Firmware-compatible ble_gatt_client_* API (KNOW-636), backed by the
 * resident NimBLE host's ble_gattc_* procedures instead of ST ACI. This
 * firmware-side adapter forwards to the neutral GATT client core in the NimBLE
 * glue (gattc_api_*, lib/nimble/glue/gattc_glue.c) and maps its events onto the
 * BleGattClientEvent the FAP-facing API defines (TASK-633). A FAP written for
 * Moon-Firmware's ble_gatt_client_* recompiles against this unchanged.
 *
 * Notifications reach this module through the central link's GAP handler
 * (nimble_glue), which calls gattc_api_on_notify. The registered callback runs
 * on the NimBLE host thread; off-thread delivery is the furi_ble broker
 * foundation (TASK-631).
 */

#include "gatt_client.h"

#include <gattc_glue.h>
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
static BleGattService gc_svcs[BLE_GATT_CLIENT_MAX_SERVICES];
static BleGattCharacteristic gc_chrs[BLE_GATT_CLIENT_MAX_CHARS];

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

/* Translate a neutral core event into BleGattClientEvent and route it. */
static void gc_dispatch(const GattcApiEvent* ev, void* context) {
    UNUSED(context);
    GattClientConnection* conn = gc_find(ev->conn_handle);
    if(!conn || !conn->callback) return;

    BleGattClientEvent out = {.connection_handle = ev->conn_handle};
    switch(ev->type) {
    case GattcApiServicesDiscovered: {
        uint8_t n = ev->count > BLE_GATT_CLIENT_MAX_SERVICES ? BLE_GATT_CLIENT_MAX_SERVICES :
                                                               ev->count;
        for(uint8_t i = 0; i < n; i++) {
            gc_svcs[i].uuid_type = ev->services[i].uuid_type;
            gc_svcs[i].uuid_16 = ev->services[i].uuid_16;
            memcpy(gc_svcs[i].uuid_128, ev->services[i].uuid_128, 16);
            gc_svcs[i].start_handle = ev->services[i].start_handle;
            gc_svcs[i].end_handle = ev->services[i].end_handle;
        }
        out.type = BleGattClientEventDiscoverComplete;
        out.discover.services = gc_svcs;
        out.discover.count = n;
        break;
    }
    case GattcApiCharsDiscovered: {
        uint8_t n = ev->count > BLE_GATT_CLIENT_MAX_CHARS ? BLE_GATT_CLIENT_MAX_CHARS : ev->count;
        for(uint8_t i = 0; i < n; i++) {
            gc_chrs[i].uuid_type = ev->chars[i].uuid_type;
            gc_chrs[i].uuid_16 = ev->chars[i].uuid_16;
            memcpy(gc_chrs[i].uuid_128, ev->chars[i].uuid_128, 16);
            gc_chrs[i].decl_handle = ev->chars[i].decl_handle;
            gc_chrs[i].value_handle = ev->chars[i].value_handle;
            gc_chrs[i].properties = ev->chars[i].properties;
        }
        out.type = BleGattClientEventCharDiscoverComplete;
        out.char_discover.chars = gc_chrs;
        out.char_discover.count = n;
        break;
    }
    case GattcApiReadComplete:
        out.type = BleGattClientEventReadComplete;
        out.read.data = ev->data;
        out.read.data_len = ev->data_len;
        out.read.value_handle = ev->value_handle;
        break;
    case GattcApiWriteComplete:
        out.type = BleGattClientEventWriteComplete;
        break;
    case GattcApiNotification:
        out.type = BleGattClientEventNotification;
        out.notification.data = ev->data;
        out.notification.data_len = ev->data_len;
        out.notification.value_handle = ev->value_handle;
        out.notification.offset = 0;
        break;
    case GattcApiError:
        out.type = BleGattClientEventError;
        out.error.error_code = ev->error_code;
        break;
    default:
        return;
    }
    conn->callback(&out, conn->context);
}

void ble_gatt_client_init(void) {
    memset(gc_connections, 0, sizeof(gc_connections));
    gattc_api_init(gc_dispatch, NULL);
    FURI_LOG_I(TAG, "GATT client initialized (NimBLE-backed)");
}

void ble_gatt_client_deinit(void) {
    gattc_api_deinit();
    memset(gc_connections, 0, sizeof(gc_connections));
}

void ble_gatt_client_set_callback(
    uint16_t connection_handle,
    BleGattClientCallback callback,
    void* context) {
    if(callback) {
        GattClientConnection* conn = gc_alloc(connection_handle);
        if(conn) {
            conn->callback = callback;
            conn->context = context;
        }
    } else {
        gc_free(connection_handle);
    }
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
