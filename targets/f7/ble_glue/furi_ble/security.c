/*
 * BLE pairing (SMP) control API (TASK-665), backed by the resident NimBLE
 * host's sm_glue. See security.h. Events are copied into a heap blob on the
 * NimBLE host thread and delivered on the BLE dispatch thread, so the FAP
 * callback never runs on the host thread (KNOW-654).
 */

#include "security.h"

#include <sm_glue.h>
#include <ble_dispatch.h>
#include <furi.h>
#include <string.h>

#define TAG "BleSecurity"

static BleSecurityCallback s_callback;
static void* s_context;
static bool s_started;

/* Dispatch thread. */
static void security_deliver(void* blob) {
    BleSecurityEvent* event = blob;
    ble_dispatch_lock();
    if(s_callback) s_callback(event, s_context);
    ble_dispatch_unlock();
}

/* NimBLE host thread: copy and hand off, never run the FAP here. */
static void security_dispatch(const SmGlueEvent* in, void* ctx) {
    UNUSED(ctx);
    BleSecurityEvent* out = malloc(sizeof(BleSecurityEvent));
    memset(out, 0, sizeof(*out));

    switch(in->kind) {
    case SmGlueEventPasskeyDisplay:
        out->type = BleSecurityEventTypePasskeyDisplay;
        break;
    case SmGlueEventPasskeyRequest:
        out->type = BleSecurityEventTypePasskeyRequest;
        break;
    case SmGlueEventNumericCompare:
        out->type = BleSecurityEventTypeNumericComparison;
        break;
    case SmGlueEventOobRequest:
        out->type = BleSecurityEventTypeOobRequest;
        break;
    case SmGlueEventEncChange:
        out->type = BleSecurityEventTypeEncryptionChanged;
        break;
    case SmGlueEventRepeatPairing:
    default:
        out->type = BleSecurityEventTypeRepeatPairing;
        break;
    }
    out->connection_handle = in->conn_handle;
    out->passkey = in->passkey;
    out->status = in->status;
    out->encrypted = in->encrypted;
    out->authenticated = in->authenticated;
    out->bonded = in->bonded;
    out->key_size = in->key_size;

    ble_dispatch_post(security_deliver, out);
}

void ble_security_init(void) {
    if(s_started) return;
    ble_dispatch_init();
    sm_glue_set_consumer(security_dispatch, NULL);
    s_started = true;
    FURI_LOG_I(TAG, "pairing control taken over");
}

void ble_security_deinit(void) {
    if(!s_started) return;
    sm_glue_set_consumer(NULL, NULL);
    sm_glue_config_default();
    ble_dispatch_lock();
    s_callback = NULL;
    s_context = NULL;
    ble_dispatch_unlock();
    s_started = false;
    FURI_LOG_I(TAG, "pairing control returned to the firmware");
}

void ble_security_set_callback(BleSecurityCallback callback, void* context) {
    ble_dispatch_lock();
    s_callback = callback;
    s_context = context;
    ble_dispatch_unlock();
}

bool ble_security_configure(
    BleSecurityIoCapability io_capability,
    bool bonding,
    bool mitm,
    bool secure_connections,
    uint8_t our_key_distribution,
    uint8_t their_key_distribution) {
    if(!s_started) ble_security_init();
    if(io_capability > BleSecurityIoKeyboardDisplay) return false;
    sm_glue_configure(
        (uint8_t)io_capability,
        bonding,
        mitm,
        secure_connections,
        our_key_distribution,
        their_key_distribution);
    return true;
}

bool ble_security_pair(uint16_t connection_handle) {
    return sm_glue_pair(connection_handle);
}

bool ble_security_passkey_reply(uint16_t connection_handle, uint32_t passkey) {
    return sm_glue_passkey_reply(connection_handle, passkey);
}

bool ble_security_numeric_comparison_reply(uint16_t connection_handle, bool accept) {
    return sm_glue_numeric_reply(connection_handle, accept);
}
