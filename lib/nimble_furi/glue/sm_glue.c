/*
 * SMP (pairing) control for the resident NimBLE host (TASK-665). See sm_glue.h.
 *
 * This file owns nothing by default. nimble_glue calls sm_glue_on_* from both
 * GAP paths; without a consumer those calls return immediately and the
 * companion's own pairing runs unchanged.
 */

#include <furi.h>
#include <string.h>

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_sm.h"

#include "sm_glue.h"

#define TAG "NimbleSm"

_Static_assert(SM_GLUE_IO_DISPLAY_ONLY == BLE_HS_IO_DISPLAY_ONLY, "io cap mismatch");
_Static_assert(SM_GLUE_IO_DISPLAY_YESNO == BLE_HS_IO_DISPLAY_YESNO, "io cap mismatch");
_Static_assert(SM_GLUE_IO_KEYBOARD_ONLY == BLE_HS_IO_KEYBOARD_ONLY, "io cap mismatch");
_Static_assert(SM_GLUE_IO_NO_INPUT_OUTPUT == BLE_HS_IO_NO_INPUT_OUTPUT, "io cap mismatch");
_Static_assert(SM_GLUE_IO_KEYBOARD_DISPLAY == BLE_HS_IO_KEYBOARD_DISPLAY, "io cap mismatch");
_Static_assert(SM_GLUE_KEY_ENC == BLE_SM_PAIR_KEY_DIST_ENC, "key dist mismatch");
_Static_assert(SM_GLUE_KEY_ID == BLE_SM_PAIR_KEY_DIST_ID, "key dist mismatch");
_Static_assert(SM_GLUE_KEY_SIGN == BLE_SM_PAIR_KEY_DIST_SIGN, "key dist mismatch");
_Static_assert(SM_GLUE_KEY_LINK == BLE_SM_PAIR_KEY_DIST_LINK, "key dist mismatch");

#define SM_GLUE_PENDING_MAX 4

typedef struct {
    uint16_t conn_handle;
    uint8_t action; /* BLE_SM_IOACT_* the peer is waiting on */
} SmPending;

static SmGlueCb s_cb;
static void* s_ctx;
static SmPending s_pending[SM_GLUE_PENDING_MAX];

/* NimBLE keeps pointers to these for the whole pairing procedure
 * (ble_sm.c stores pkey->oob_sc_data.local / .remote in the proc), so they must
 * outlive the inject call. */
static struct ble_sm_sc_oob_data s_oob_local;
static struct ble_sm_sc_oob_data s_oob_remote;
static bool s_oob_local_valid;
static bool s_oob_remote_valid;
/* Link whose OOB action fired before the peer's values were armed. */
static uint16_t s_oob_waiting = BLE_HS_CONN_HANDLE_NONE;

static void pending_set(uint16_t conn_handle, uint8_t action) {
    for(size_t i = 0; i < COUNT_OF(s_pending); i++) {
        if(s_pending[i].action && s_pending[i].conn_handle == conn_handle) {
            s_pending[i].action = action;
            return;
        }
    }
    for(size_t i = 0; i < COUNT_OF(s_pending); i++) {
        if(!s_pending[i].action) {
            s_pending[i].conn_handle = conn_handle;
            s_pending[i].action = action;
            return;
        }
    }
}

static uint8_t pending_take(uint16_t conn_handle) {
    for(size_t i = 0; i < COUNT_OF(s_pending); i++) {
        if(s_pending[i].action && s_pending[i].conn_handle == conn_handle) {
            uint8_t action = s_pending[i].action;
            s_pending[i].action = 0;
            return action;
        }
    }
    return 0;
}

static void emit(const SmGlueEvent* event) {
    if(s_cb) s_cb(event, s_ctx);
}

void sm_glue_set_consumer(SmGlueCb cb, void* ctx) {
    s_cb = cb;
    s_ctx = ctx;
    memset(s_pending, 0, sizeof(s_pending));
}

bool sm_glue_has_consumer(void) {
    return s_cb != NULL;
}

void sm_glue_configure(
    uint8_t io_cap,
    bool bonding,
    bool mitm,
    bool secure_connections,
    uint8_t our_key_dist,
    uint8_t their_key_dist) {
    if(io_cap > SM_GLUE_IO_KEYBOARD_DISPLAY) io_cap = SM_GLUE_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_io_cap = io_cap;
    ble_hs_cfg.sm_bonding = bonding ? 1 : 0;
    ble_hs_cfg.sm_mitm = mitm ? 1 : 0;
    ble_hs_cfg.sm_sc = secure_connections ? 1 : 0;
    ble_hs_cfg.sm_our_key_dist = our_key_dist;
    ble_hs_cfg.sm_their_key_dist = their_key_dist;
    FURI_LOG_I(
        TAG,
        "SM config io=%u bond=%u mitm=%u sc=%u kd=%02x/%02x",
        io_cap,
        bonding,
        mitm,
        secure_connections,
        our_key_dist,
        their_key_dist);
}

void sm_glue_config_default(void) {
    /* The companion's parameters, mirroring nimble_glue_start. */
    sm_glue_configure(
        SM_GLUE_IO_DISPLAY_ONLY,
        true,
        true,
        false,
        SM_GLUE_KEY_ENC | SM_GLUE_KEY_ID,
        SM_GLUE_KEY_ENC | SM_GLUE_KEY_ID);
}

bool sm_glue_pair(uint16_t conn_handle) {
    int rc = ble_gap_security_initiate(conn_handle);
    if(rc != 0) FURI_LOG_W(TAG, "security_initiate(%u) rc=%d", conn_handle, rc);
    return rc == 0;
}

bool sm_glue_passkey_reply(uint16_t conn_handle, uint32_t passkey) {
    struct ble_sm_io io = {0};
    uint8_t action = pending_take(conn_handle);
    io.action = action ? action : BLE_SM_IOACT_INPUT;
    io.passkey = passkey;
    int rc = ble_sm_inject_io(conn_handle, &io);
    if(rc != 0) FURI_LOG_W(TAG, "passkey inject rc=%d", rc);
    return rc == 0;
}

bool sm_glue_numeric_reply(uint16_t conn_handle, bool accept) {
    struct ble_sm_io io = {0};
    pending_take(conn_handle);
    io.action = BLE_SM_IOACT_NUMCMP;
    io.numcmp_accept = accept ? 1 : 0;
    int rc = ble_sm_inject_io(conn_handle, &io);
    if(rc != 0) FURI_LOG_W(TAG, "numcmp inject rc=%d", rc);
    return rc == 0;
}

/* --- LE Secure Connections OOB ---------------------------------------------- */

bool sm_glue_sc_supported(void) {
    return MYNEWT_VAL(BLE_SM_SC) != 0;
}

static bool oob_inject(uint16_t conn_handle) {
    struct ble_sm_io io = {0};
    io.action = BLE_SM_IOACT_OOB_SC;
    io.oob_sc_data.local = s_oob_local_valid ? &s_oob_local : NULL;
    io.oob_sc_data.remote = s_oob_remote_valid ? &s_oob_remote : NULL;
    int rc = ble_sm_inject_io(conn_handle, &io);
    if(rc != 0) FURI_LOG_W(TAG, "OOB inject rc=%d", rc);
    return rc == 0;
}

bool sm_glue_oob_generate(uint8_t* out_random, uint8_t* out_confirm) {
    int rc = ble_sm_sc_oob_generate_data(&s_oob_local);
    if(rc != 0) {
        FURI_LOG_E(TAG, "OOB generate rc=%d", rc);
        s_oob_local_valid = false;
        return false;
    }
    s_oob_local_valid = true;
    if(out_random) memcpy(out_random, s_oob_local.r, sizeof(s_oob_local.r));
    if(out_confirm) memcpy(out_confirm, s_oob_local.c, sizeof(s_oob_local.c));
    FURI_LOG_I(TAG, "OOB local data generated");
    return true;
}

bool sm_glue_oob_set_peer(const uint8_t* random, const uint8_t* confirm) {
    if(!random || !confirm) return false;
    memcpy(s_oob_remote.r, random, sizeof(s_oob_remote.r));
    memcpy(s_oob_remote.c, confirm, sizeof(s_oob_remote.c));
    s_oob_remote_valid = true;
    /* The pairing request must say we hold the peer's OOB data. */
    ble_hs_cfg.sm_oob_data_flag = 1;
    FURI_LOG_I(TAG, "OOB peer data armed");

    if(s_oob_waiting != BLE_HS_CONN_HANDLE_NONE) {
        uint16_t conn = s_oob_waiting;
        s_oob_waiting = BLE_HS_CONN_HANDLE_NONE;
        pending_take(conn);
        return oob_inject(conn);
    }
    return true;
}

void sm_glue_oob_clear(void) {
    s_oob_local_valid = false;
    s_oob_remote_valid = false;
    s_oob_waiting = BLE_HS_CONN_HANDLE_NONE;
    memset(&s_oob_local, 0, sizeof(s_oob_local));
    memset(&s_oob_remote, 0, sizeof(s_oob_remote));
    ble_hs_cfg.sm_oob_data_flag = 0;
}

bool sm_glue_on_passkey_action(uint16_t conn_handle, uint8_t action, uint32_t numcmp) {
    if(!s_cb) return false;

    SmGlueEvent event = {0};
    event.conn_handle = conn_handle;
    pending_set(conn_handle, action);

    switch(action) {
    case BLE_SM_IOACT_DISP:
        /* The consumer picks the number; it must call sm_glue_passkey_reply. */
        event.kind = SmGlueEventPasskeyDisplay;
        break;
    case BLE_SM_IOACT_INPUT:
        event.kind = SmGlueEventPasskeyRequest;
        break;
    case BLE_SM_IOACT_NUMCMP:
        event.kind = SmGlueEventNumericCompare;
        event.passkey = numcmp;
        break;
    case BLE_SM_IOACT_OOB_SC:
        /* Both halves already present: finish without troubling the consumer. */
        if(s_oob_local_valid && s_oob_remote_valid) {
            pending_take(conn_handle);
            oob_inject(conn_handle);
            return true;
        }
        s_oob_waiting = conn_handle;
        event.kind = SmGlueEventOobRequest;
        break;
    default:
        event.kind = SmGlueEventOobRequest;
        break;
    }
    FURI_LOG_I(TAG, "Pairing action %u on handle %u", action, conn_handle);
    emit(&event);
    return true;
}

void sm_glue_on_enc_change(uint16_t conn_handle, int status) {
    if(!s_cb) return;
    pending_take(conn_handle);

    SmGlueEvent event = {0};
    event.kind = SmGlueEventEncChange;
    event.conn_handle = conn_handle;
    event.status = (int16_t)status;

    struct ble_gap_conn_desc desc;
    if(ble_gap_conn_find(conn_handle, &desc) == 0) {
        event.encrypted = desc.sec_state.encrypted;
        event.authenticated = desc.sec_state.authenticated;
        event.bonded = desc.sec_state.bonded;
        event.key_size = desc.sec_state.key_size;
    }
    emit(&event);
}

void sm_glue_on_repeat_pairing(uint16_t conn_handle) {
    if(!s_cb) return;
    SmGlueEvent event = {0};
    event.kind = SmGlueEventRepeatPairing;
    event.conn_handle = conn_handle;
    emit(&event);
}
