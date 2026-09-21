/* furi_ble/security.h. Replies and pairing starts can send HCI commands, so
 * they run on the host thread; events go out on the BLE dispatch thread. */

#include <furi.h>
#include <string.h>

#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_sm.h"

#include <furi_ble/security.h>
#include <ble_dispatch.h>

#include "security_glue.h"
#include "nimble_glue.h"

_Static_assert(BleSecurityIoKeyboardDisplay == BLE_HS_IO_KEYBOARD_DISPLAY, "io caps");
_Static_assert(BLE_SECURITY_KEY_LINK == BLE_SM_PAIR_KEY_DIST_LINK, "key distribution");

static volatile bool active;
static BleSecurityCallback callback;
static void* context;

// The companion's parameters, restored on deinit
static struct {
    uint8_t io_cap, bonding, mitm, sc, our_key_dist, their_key_dist;
} saved;

typedef enum {
    RequestPair = 1,
    RequestPasskey,
    RequestNumcmp,
    RequestOobPeer,
} RequestOp;

static struct {
    RequestOp op;
    uint16_t conn;
    uint32_t value;
} requests[4];
static struct ble_npl_event request_event;
static bool request_event_ready;

// Host thread only
static struct {
    uint16_t conn;
    uint8_t action;
} pending[MYNEWT_VAL(BLE_MAX_CONNECTIONS)];
static uint16_t oob_waiting = BLE_HS_CONN_HANDLE_NONE;

// NimBLE keeps pointers to these for the whole pairing
static struct ble_sm_sc_oob_data oob_local;
static struct ble_sm_sc_oob_data oob_remote;
static volatile bool oob_local_valid;
static volatile bool oob_remote_valid;

// Dispatch thread
static void deliver(void* blob) {
    ble_dispatch_lock();
    if(callback) callback(blob, context);
    ble_dispatch_unlock();
}

static void post_event(const BleSecurityEvent* event) {
    BleSecurityEvent* copy = malloc(sizeof(BleSecurityEvent));
    *copy = *event;
    ble_dispatch_post(deliver, copy);
}

static void pending_set(uint16_t conn, uint8_t action) {
    size_t free_slot = COUNT_OF(pending);
    for(size_t i = 0; i < COUNT_OF(pending); i++) {
        if(pending[i].action && pending[i].conn == conn) free_slot = i;
        if(!pending[i].action && free_slot == COUNT_OF(pending)) free_slot = i;
    }
    if(free_slot < COUNT_OF(pending)) {
        pending[free_slot].conn = conn;
        pending[free_slot].action = action;
    }
}

static uint8_t pending_take(uint16_t conn) {
    for(size_t i = 0; i < COUNT_OF(pending); i++) {
        if(pending[i].action && pending[i].conn == conn) {
            uint8_t action = pending[i].action;
            pending[i].action = 0;
            return action;
        }
    }
    return 0;
}

// Host thread. NimBLE reads the local values unchecked, so they must exist.
static void oob_inject(uint16_t conn) {
    if(!oob_local_valid) oob_local_valid = ble_sm_sc_oob_generate_data(&oob_local) == 0;
    struct ble_sm_io io = {.action = BLE_SM_IOACT_OOB_SC};
    io.oob_sc_data.local = oob_local_valid ? &oob_local : NULL;
    io.oob_sc_data.remote = oob_remote_valid ? &oob_remote : NULL;
    ble_sm_inject_io(conn, &io);
}

// Host thread
static void request_event_fn(struct ble_npl_event* ev) {
    UNUSED(ev);
    for(size_t i = 0; i < COUNT_OF(requests); i++) {
        FURI_CRITICAL_ENTER();
        RequestOp op = requests[i].op;
        uint16_t conn = requests[i].conn;
        uint32_t value = requests[i].value;
        requests[i].op = 0;
        FURI_CRITICAL_EXIT();

        struct ble_sm_io io = {0};
        switch(op) {
        case RequestPair: {
            int rc = ble_gap_security_initiate(conn);
            if(rc != 0) {
                BleSecurityEvent failed = {
                    .type = BleSecurityEventTypeEncryptionChanged,
                    .connection_handle = conn,
                    .status = rc,
                };
                post_event(&failed);
            }
            break;
        }
        case RequestPasskey: {
            uint8_t action = pending_take(conn);
            io.action = action ? action : BLE_SM_IOACT_INPUT;
            io.passkey = value;
            ble_sm_inject_io(conn, &io);
            break;
        }
        case RequestNumcmp:
            pending_take(conn);
            io.action = BLE_SM_IOACT_NUMCMP;
            io.numcmp_accept = value;
            ble_sm_inject_io(conn, &io);
            break;
        case RequestOobPeer:
            if(oob_waiting != BLE_HS_CONN_HANDLE_NONE) {
                pending_take(oob_waiting);
                oob_inject(oob_waiting);
                oob_waiting = BLE_HS_CONN_HANDLE_NONE;
            }
            break;
        default:
            break;
        }
    }
}

static bool request(RequestOp op, uint16_t conn, uint32_t value) {
    if(!active || !request_event_ready) return false;
    bool queued = false;
    FURI_CRITICAL_ENTER();
    for(size_t i = 0; i < COUNT_OF(requests) && !queued; i++) {
        if(!requests[i].op) {
            requests[i].op = op;
            requests[i].conn = conn;
            requests[i].value = value;
            queued = true;
        }
    }
    FURI_CRITICAL_EXIT();
    if(queued) ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &request_event);
    return queued;
}

bool security_glue_on_gap_event(const struct ble_gap_event* event) {
    if(!active) return false;
    BleSecurityEvent out = {0};
    switch(event->type) {
    case BLE_GAP_EVENT_PASSKEY_ACTION:
        out.connection_handle = event->passkey.conn_handle;
        pending_set(out.connection_handle, event->passkey.params.action);
        switch(event->passkey.params.action) {
        case BLE_SM_IOACT_DISP:
            out.type = BleSecurityEventTypePasskeyDisplay;
            break;
        case BLE_SM_IOACT_INPUT:
            out.type = BleSecurityEventTypePasskeyRequest;
            break;
        case BLE_SM_IOACT_NUMCMP:
            out.type = BleSecurityEventTypeNumericComparison;
            out.passkey = event->passkey.params.numcmp;
            break;
        case BLE_SM_IOACT_OOB_SC:
            if(oob_local_valid && oob_remote_valid) {
                pending_take(out.connection_handle);
                oob_inject(out.connection_handle);
                return true;
            }
            oob_waiting = out.connection_handle;
            out.type = BleSecurityEventTypeOobRequest;
            break;
        default:
            // Legacy OOB has no API here; end the pairing instead of stalling it
            pending_take(out.connection_handle);
            ble_gap_terminate(out.connection_handle, BLE_ERR_AUTH_FAIL);
            out.type = BleSecurityEventTypeEncryptionChanged;
            out.status = BLE_HS_ENOTSUP;
            break;
        }
        break;
    case BLE_GAP_EVENT_ENC_CHANGE: {
        out.type = BleSecurityEventTypeEncryptionChanged;
        out.connection_handle = event->enc_change.conn_handle;
        out.status = event->enc_change.status;
        pending_take(out.connection_handle);
        struct ble_gap_conn_desc desc;
        if(ble_gap_conn_find(out.connection_handle, &desc) == 0) {
            out.encrypted = desc.sec_state.encrypted;
            out.authenticated = desc.sec_state.authenticated;
            out.bonded = desc.sec_state.bonded;
            out.key_size = desc.sec_state.key_size;
        }
        break;
    }
    case BLE_GAP_EVENT_REPEAT_PAIRING:
        out.type = BleSecurityEventTypeRepeatPairing;
        out.connection_handle = event->repeat_pairing.conn_handle;
        break;
    default:
        return false;
    }
    post_event(&out);
    return true;
}

void ble_security_init(void) {
    ble_dispatch_init();
    nimble_glue_on_app_stop(ble_security_deinit);
    if(!request_event_ready) {
        ble_npl_event_init(&request_event, request_event_fn, NULL);
        request_event_ready = true;
    }
    if(active) return;
    saved.io_cap = ble_hs_cfg.sm_io_cap;
    saved.bonding = ble_hs_cfg.sm_bonding;
    saved.mitm = ble_hs_cfg.sm_mitm;
    saved.sc = ble_hs_cfg.sm_sc;
    saved.our_key_dist = ble_hs_cfg.sm_our_key_dist;
    saved.their_key_dist = ble_hs_cfg.sm_their_key_dist;
    active = true;
}

void ble_security_deinit(void) {
    if(!active) return;
    active = false;
    ble_security_set_callback(NULL, NULL);
    ble_security_oob_clear();
    ble_hs_cfg.sm_io_cap = saved.io_cap;
    ble_hs_cfg.sm_bonding = saved.bonding;
    ble_hs_cfg.sm_mitm = saved.mitm;
    ble_hs_cfg.sm_sc = saved.sc;
    ble_hs_cfg.sm_our_key_dist = saved.our_key_dist;
    ble_hs_cfg.sm_their_key_dist = saved.their_key_dist;
}

void ble_security_set_callback(BleSecurityCallback cb, void* ctx) {
    ble_dispatch_lock();
    callback = cb;
    context = ctx;
    ble_dispatch_unlock();
}

bool ble_security_configure(
    BleSecurityIoCapability io_capability,
    bool bonding,
    bool mitm,
    bool secure_connections,
    uint8_t our_key_distribution,
    uint8_t their_key_distribution) {
    if(!active || io_capability > BleSecurityIoKeyboardDisplay) return false;
    ble_hs_cfg.sm_io_cap = io_capability;
    ble_hs_cfg.sm_bonding = bonding;
    ble_hs_cfg.sm_mitm = mitm;
    ble_hs_cfg.sm_sc = secure_connections;
    ble_hs_cfg.sm_our_key_dist = our_key_distribution;
    ble_hs_cfg.sm_their_key_dist = their_key_distribution;
    return true;
}

bool ble_security_pair(uint16_t connection_handle) {
    return request(RequestPair, connection_handle, 0);
}

bool ble_security_passkey_reply(uint16_t connection_handle, uint32_t passkey) {
    return request(RequestPasskey, connection_handle, passkey);
}

bool ble_security_numeric_comparison_reply(uint16_t connection_handle, bool accept) {
    return request(RequestNumcmp, connection_handle, accept);
}

bool ble_security_secure_connections_supported(void) {
    return true;
}

typedef struct {
    FuriSemaphore* done;
} GenerateJob;

// Host thread: key generation shares the SM's keys and sends LE Rand
static void generate_on_host(void* arg) {
    GenerateJob* job = arg;
    oob_local_valid = ble_sm_sc_oob_generate_data(&oob_local) == 0;
    furi_semaphore_release(job->done);
}

// Waits for the host thread, which never waits on an app thread
bool ble_security_oob_generate(uint8_t* out_random, uint8_t* out_confirm) {
    GenerateJob job = {.done = furi_semaphore_alloc(1, 0)};
    bool queued = nimble_glue_run_on_host(generate_on_host, &job, sizeof(job));
    if(queued) furi_semaphore_acquire(job.done, FuriWaitForever);
    furi_semaphore_free(job.done);
    if(!queued || !oob_local_valid) return false;
    if(out_random) memcpy(out_random, oob_local.r, sizeof(oob_local.r));
    if(out_confirm) memcpy(out_confirm, oob_local.c, sizeof(oob_local.c));
    return true;
}

bool ble_security_oob_set_peer(const uint8_t* random, const uint8_t* confirm) {
    if(!random || !confirm) return false;
    memcpy(oob_remote.r, random, sizeof(oob_remote.r));
    memcpy(oob_remote.c, confirm, sizeof(oob_remote.c));
    oob_remote_valid = true;
    ble_hs_cfg.sm_oob_data_flag = 1;
    request(RequestOobPeer, 0, 0);
    return true;
}

void ble_security_oob_clear(void) {
    oob_local_valid = false;
    oob_remote_valid = false;
    ble_hs_cfg.sm_oob_data_flag = 0;
}
