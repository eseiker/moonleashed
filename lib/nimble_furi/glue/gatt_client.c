/* furi_ble/gatt_client.h on NimBLE's ble_gattc_* procedures. Requests run on
 * the host thread; results are copied into one blob per event and delivered
 * on the BLE dispatch thread. */

#include <furi.h>
#include <string.h>

#include "os/os_mbuf.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

#include <furi_ble/gatt_client.h>
#include <ble_dispatch.h>

#include "gatt_client_glue.h"
#include "nimble_glue.h"

#define MAX_CONNECTIONS MYNEWT_VAL(BLE_MAX_CONNECTIONS)

/* ATT "Unlikely Error", reported for failures on our side of the link */
#define ERROR_HOST 0x0E

static struct {
    bool active;
    uint16_t connection_handle;
    BleGattClientCallback callback;
    void* context;
} connections[MAX_CONNECTIONS];

/* One discovery and one subscribe at a time; a second call is refused. */
static volatile bool discovering;
static volatile bool subscribing;

/* Discovery results collect here. Host thread only. */
static BleGattService* services;
static uint8_t service_count;
static BleGattCharacteristic* chars;
static uint8_t char_count;

typedef struct {
    BleGattClientEvent event;
    uint8_t payload[];
} EventBlob;

// Dispatch thread
static void deliver(void* blob) {
    EventBlob* b = blob;
    ble_dispatch_lock();
    for(size_t i = 0; i < MAX_CONNECTIONS; i++) {
        if(connections[i].active &&
           connections[i].connection_handle == b->event.connection_handle) {
            connections[i].callback(&b->event, connections[i].context);
            break;
        }
    }
    ble_dispatch_unlock();
}

// Host thread; the caller fills in the event and posts it
static EventBlob* event_alloc(uint16_t conn, BleGattClientEventType type, size_t payload_len) {
    EventBlob* b = malloc(sizeof(EventBlob) + payload_len);
    memset(&b->event, 0, sizeof(b->event));
    b->event.type = type;
    b->event.connection_handle = conn;
    return b;
}

static void post_error(uint16_t conn, int status) {
    EventBlob* b = event_alloc(conn, BleGattClientEventError, 0);
    bool att = status >= BLE_HS_ERR_ATT_BASE && status < BLE_HS_ERR_ATT_BASE + 0x100;
    b->event.error.error_code = att ? status - BLE_HS_ERR_ATT_BASE : ERROR_HOST;
    ble_dispatch_post(deliver, b);
}

static void map_uuid(const ble_uuid_any_t* u, uint8_t* type, uint16_t* u16, uint8_t* u128) {
    if(u->u.type == BLE_UUID_TYPE_16) {
        *type = 1;
        *u16 = ble_uuid_u16(&u->u);
        return;
    }
    *type = 2;
    if(u->u.type == BLE_UUID_TYPE_128) {
        memcpy(u128, u->u128.value, 16);
        return;
    }
    // 32-bit, on the Bluetooth base UUID
    static const uint8_t base[12] = {
        0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00};
    memcpy(u128, base, sizeof(base));
    put_le32(&u128[12], u->u32.value);
}

static int on_service(
    uint16_t conn,
    const struct ble_gatt_error* error,
    const struct ble_gatt_svc* service,
    void* arg) {
    UNUSED(arg);
    if(error->status == 0 && service) {
        if(service_count < BLE_GATT_CLIENT_MAX_SERVICES) {
            BleGattService* s = &services[service_count++];
            memset(s, 0, sizeof(*s));
            s->start_handle = service->start_handle;
            s->end_handle = service->end_handle;
            map_uuid(&service->uuid, &s->uuid_type, &s->uuid_16, s->uuid_128);
        }
        return 0;
    }
    discovering = false;
    if(error->status == BLE_HS_EDONE) {
        size_t len = service_count * sizeof(BleGattService);
        EventBlob* b = event_alloc(conn, BleGattClientEventDiscoverComplete, len);
        memcpy(b->payload, services, len);
        b->event.discover.services = (BleGattService*)b->payload;
        b->event.discover.count = service_count;
        ble_dispatch_post(deliver, b);
    } else {
        post_error(conn, error->status);
    }
    return 0;
}

static int on_char(
    uint16_t conn,
    const struct ble_gatt_error* error,
    const struct ble_gatt_chr* chr,
    void* arg) {
    UNUSED(arg);
    if(error->status == 0 && chr) {
        if(char_count < BLE_GATT_CLIENT_MAX_CHARS) {
            BleGattCharacteristic* c = &chars[char_count++];
            memset(c, 0, sizeof(*c));
            c->decl_handle = chr->def_handle;
            c->value_handle = chr->val_handle;
            c->properties = chr->properties;
            map_uuid(&chr->uuid, &c->uuid_type, &c->uuid_16, c->uuid_128);
        }
        return 0;
    }
    discovering = false;
    if(error->status == BLE_HS_EDONE) {
        size_t len = char_count * sizeof(BleGattCharacteristic);
        EventBlob* b = event_alloc(conn, BleGattClientEventCharDiscoverComplete, len);
        memcpy(b->payload, chars, len);
        b->event.char_discover.chars = (BleGattCharacteristic*)b->payload;
        b->event.char_discover.count = char_count;
        ble_dispatch_post(deliver, b);
    } else {
        post_error(conn, error->status);
    }
    return 0;
}

static void post_data(
    uint16_t conn,
    BleGattClientEventType type,
    uint16_t value_handle,
    struct os_mbuf* om) {
    uint16_t len = OS_MBUF_PKTLEN(om);
    EventBlob* b = event_alloc(conn, type, len);
    os_mbuf_copydata(om, 0, len, b->payload);
    // read and notification have the same layout
    b->event.read.data = b->payload;
    b->event.read.data_len = len;
    b->event.read.value_handle = value_handle;
    ble_dispatch_post(deliver, b);
}

static int on_read(
    uint16_t conn,
    const struct ble_gatt_error* error,
    struct ble_gatt_attr* attr,
    void* arg) {
    UNUSED(arg);
    if(error->status != 0 || !attr) {
        post_error(conn, error->status);
    } else {
        post_data(conn, BleGattClientEventReadComplete, attr->handle, attr->om);
    }
    return 0;
}

// arg is non-NULL for the CCCD write that ends a subscribe
static int on_write(
    uint16_t conn,
    const struct ble_gatt_error* error,
    struct ble_gatt_attr* attr,
    void* arg) {
    UNUSED(attr);
    if(arg) subscribing = false;
    if(error->status != 0) {
        post_error(conn, error->status);
    } else {
        ble_dispatch_post(deliver, event_alloc(conn, BleGattClientEventWriteComplete, 0));
    }
    return 0;
}

static int on_mtu(uint16_t conn, const struct ble_gatt_error* error, uint16_t mtu, void* arg) {
    UNUSED(conn);
    UNUSED(error);
    UNUSED(mtu);
    UNUSED(arg);
    return 0;
}

void gatt_client_on_notify(uint16_t conn, uint16_t attr_handle, struct os_mbuf* om) {
    post_data(conn, BleGattClientEventNotification, attr_handle, om);
}

/* The CCCD is the 0x2902 descriptor before the next characteristic
 * declaration; it is not always right after the value. */
#define CCCD_SEARCH_SPAN 8

static struct {
    uint8_t value[2];
    uint16_t cccd_handle;
    bool past_char;
} subscribe;

static int on_descriptor(
    uint16_t conn,
    const struct ble_gatt_error* error,
    uint16_t chr_val_handle,
    const struct ble_gatt_dsc* dsc,
    void* arg) {
    UNUSED(chr_val_handle);
    UNUSED(arg);
    if(error->status == 0 && dsc) {
        uint16_t uuid = dsc->uuid.u.type == BLE_UUID_TYPE_16 ? ble_uuid_u16(&dsc->uuid.u) : 0;
        if(uuid == BLE_ATT_UUID_CHARACTERISTIC) subscribe.past_char = true;
        if(uuid == BLE_GATT_DSC_CLT_CFG_UUID16 && !subscribe.past_char && !subscribe.cccd_handle) {
            subscribe.cccd_handle = dsc->handle;
        }
        return 0;
    }
    int rc = error->status;
    if(rc == BLE_HS_EDONE) {
        rc = BLE_HS_ERR_ATT_BASE + BLE_ATT_ERR_ATTR_NOT_FOUND;
        if(subscribe.cccd_handle) {
            rc = ble_gattc_write_flat(
                conn,
                subscribe.cccd_handle,
                subscribe.value,
                sizeof(subscribe.value),
                on_write,
                &subscribe);
        }
    }
    if(rc != 0) {
        subscribing = false;
        post_error(conn, rc);
    }
    return 0;
}

typedef enum {
    RequestServices,
    RequestChars,
    RequestRead,
    RequestWrite,
    RequestMtu,
    RequestSubscribe,
} RequestOp;

typedef struct {
    RequestOp op;
    uint16_t conn;
    uint16_t handle;
    uint16_t end_handle;
    uint16_t len;
    uint8_t data[];
} Request;

// Host thread
static void run_request(void* arg) {
    Request* r = arg;
    int rc = 0;
    switch(r->op) {
    case RequestServices:
        if(!services) services = malloc(sizeof(BleGattService) * BLE_GATT_CLIENT_MAX_SERVICES);
        service_count = 0;
        rc = ble_gattc_disc_all_svcs(r->conn, on_service, NULL);
        if(rc != 0) discovering = false;
        break;
    case RequestChars:
        if(!chars) chars = malloc(sizeof(BleGattCharacteristic) * BLE_GATT_CLIENT_MAX_CHARS);
        char_count = 0;
        rc = ble_gattc_disc_all_chrs(r->conn, r->handle, r->end_handle, on_char, NULL);
        if(rc != 0) discovering = false;
        break;
    case RequestRead:
        rc = ble_gattc_read(r->conn, r->handle, on_read, NULL);
        break;
    case RequestWrite:
        rc = ble_gattc_write_flat(r->conn, r->handle, r->data, r->len, on_write, NULL);
        break;
    case RequestMtu:
        rc = ble_gattc_exchange_mtu(r->conn, on_mtu, NULL);
        break;
    case RequestSubscribe:
        subscribe.value[0] = r->data[0];
        subscribe.value[1] = 0x00;
        subscribe.cccd_handle = 0;
        subscribe.past_char = false;
        rc = ble_gattc_disc_all_dscs(r->conn, r->handle, r->end_handle, on_descriptor, NULL);
        if(rc != 0) subscribing = false;
        break;
    }
    if(rc != 0) post_error(r->conn, rc);
}

static bool request(RequestOp op, uint16_t conn, uint16_t handle, uint16_t end_handle) {
    Request r = {.op = op, .conn = conn, .handle = handle, .end_handle = end_handle};
    return nimble_glue_run_on_host(run_request, &r, sizeof(r));
}

// Claims a busy flag; false if it was already taken
static bool claim(volatile bool* busy) {
    FURI_CRITICAL_ENTER();
    bool was_free = !*busy;
    *busy = true;
    FURI_CRITICAL_EXIT();
    return was_free;
}

void ble_gatt_client_init(void) {
    ble_dispatch_init();
    nimble_glue_on_app_stop(ble_gatt_client_deinit);
    ble_dispatch_lock();
    memset(connections, 0, sizeof(connections));
    ble_dispatch_unlock();
}

void ble_gatt_client_deinit(void) {
    ble_dispatch_lock();
    memset(connections, 0, sizeof(connections));
    ble_dispatch_unlock();
}

void ble_gatt_client_set_callback(
    uint16_t connection_handle,
    BleGattClientCallback callback,
    void* context) {
    if(callback) nimble_glue_on_app_stop(ble_gatt_client_deinit);
    ble_dispatch_lock();
    for(size_t i = 0; i < MAX_CONNECTIONS; i++) {
        if(connections[i].active && connections[i].connection_handle == connection_handle) {
            connections[i].active = false;
        }
    }
    for(size_t i = 0; callback && i < MAX_CONNECTIONS; i++) {
        if(!connections[i].active) {
            connections[i].active = true;
            connections[i].connection_handle = connection_handle;
            connections[i].callback = callback;
            connections[i].context = context;
            break;
        }
    }
    ble_dispatch_unlock();
}

bool ble_gatt_client_discover_services(uint16_t connection_handle) {
    if(!claim(&discovering)) return false;
    if(request(RequestServices, connection_handle, 0, 0)) return true;
    discovering = false;
    return false;
}

bool ble_gatt_client_discover_characteristics(
    uint16_t connection_handle,
    const BleGattService* service) {
    furi_check(service);
    if(!claim(&discovering)) return false;
    if(request(RequestChars, connection_handle, service->start_handle, service->end_handle)) {
        return true;
    }
    discovering = false;
    return false;
}

bool ble_gatt_client_read(uint16_t connection_handle, uint16_t value_handle) {
    return request(RequestRead, connection_handle, value_handle, 0);
}

static bool request_with_data(
    RequestOp op,
    uint16_t conn,
    uint16_t handle,
    uint16_t end_handle,
    const uint8_t* data,
    uint16_t len) {
    Request* r = malloc(sizeof(Request) + len);
    r->op = op;
    r->conn = conn;
    r->handle = handle;
    r->end_handle = end_handle;
    r->len = len;
    if(len) memcpy(r->data, data, len);
    bool queued = nimble_glue_run_on_host(run_request, r, sizeof(Request) + len);
    free(r);
    return queued;
}

bool ble_gatt_client_write(
    uint16_t connection_handle,
    uint16_t value_handle,
    const uint8_t* data,
    uint16_t data_len) {
    if(data_len && !data) return false;
    return request_with_data(RequestWrite, connection_handle, value_handle, 0, data, data_len);
}

bool ble_gatt_client_exchange_mtu(uint16_t connection_handle) {
    return request(RequestMtu, connection_handle, 0, 0);
}

bool ble_gatt_client_subscribe_notifications(
    uint16_t connection_handle,
    uint16_t value_handle,
    bool enable) {
    if(!claim(&subscribing)) return false;
    uint16_t end = value_handle > 0xFFFF - CCCD_SEARCH_SPAN ? 0xFFFF :
                                                              value_handle + CCCD_SEARCH_SPAN;
    uint8_t value = enable ? 0x01 : 0x00;
    if(request_with_data(RequestSubscribe, connection_handle, value_handle, end, &value, 1)) {
        return true;
    }
    subscribing = false;
    return false;
}
