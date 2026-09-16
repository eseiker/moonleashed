/*
 * Fixed L2CAP CID relay API (TASK-663), backed by the resident NimBLE host's
 * fixedcid_glue. See l2cap_fixed.h. Received PDUs are copied into a heap blob on
 * the NimBLE host thread and delivered on the BLE dispatch thread, so the FAP
 * callback never runs on the host thread (like the CoC / GATT-client APIs,
 * KNOW-654).
 */

#include "l2cap_fixed.h"

#include <fixedcid_glue.h>
#include <ble_dispatch.h>
#include <furi.h>
#include <string.h>

#define TAG "BleL2capFixed"

static BleL2capFixedCallback s_callback;
static void* s_context;
static bool s_started;

typedef struct {
    uint16_t conn_handle;
    uint16_t cid;
    uint16_t len;
    uint8_t data[];
} FixedEventBlob;

/* Dispatch thread. */
static void fixed_deliver(void* blob) {
    FixedEventBlob* b = blob;
    ble_dispatch_lock();
    if(s_callback) s_callback(b->conn_handle, b->cid, b->data, b->len, s_context);
    ble_dispatch_unlock();
}

/* NimBLE host thread: copy and hand off, never run the FAP here. */
static void fixed_dispatch(
    uint16_t conn_handle,
    uint16_t cid,
    const uint8_t* data,
    uint16_t len,
    void* ctx) {
    UNUSED(ctx);
    FixedEventBlob* b = malloc(sizeof(FixedEventBlob) + len);
    b->conn_handle = conn_handle;
    b->cid = cid;
    b->len = len;
    if(len) memcpy(b->data, data, len);
    ble_dispatch_post(fixed_deliver, b);
}

void ble_l2cap_fixed_init(void) {
    if(s_started) return;
    ble_dispatch_init();
    fixedcid_init(fixed_dispatch, NULL);
    s_started = true;
    FURI_LOG_I(TAG, "fixed-CID relay initialized");
}

void ble_l2cap_fixed_deinit(void) {
    fixedcid_deinit();
    ble_dispatch_lock();
    s_callback = NULL;
    s_context = NULL;
    ble_dispatch_unlock();
    s_started = false;
}

void ble_l2cap_fixed_set_callback(BleL2capFixedCallback callback, void* context) {
    ble_dispatch_lock();
    s_callback = callback;
    s_context = context;
    ble_dispatch_unlock();
}

bool ble_l2cap_fixed_register(uint16_t cid, uint16_t mtu) {
    if(!s_started) ble_l2cap_fixed_init();
    return fixedcid_register(cid, mtu);
}

bool ble_l2cap_fixed_unregister(uint16_t cid) {
    return fixedcid_unregister(cid);
}

bool ble_l2cap_fixed_send(
    uint16_t connection_handle,
    uint16_t cid,
    const uint8_t* data,
    uint16_t data_len) {
    return fixedcid_send(connection_handle, cid, data, data_len);
}
