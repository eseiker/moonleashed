/*
 * SMP (pairing) control for the resident NimBLE host (TASK-665).
 *
 * nimble_glue pairs the way the mobile companion app expects: DISPLAY_ONLY IO
 * capability, bonding, MITM, legacy pairing, and a random 6-digit passkey the
 * Flipper shows and injects itself. That path is verified and must not change.
 *
 * A consumer registered here takes pairing over instead. While one is
 * registered, nimble_glue hands it every pairing event and runs none of the
 * companion code; the consumer decides the passkey, answers a numeric
 * comparison, and watches encryption come up. Unregistering restores the
 * companion behaviour exactly.
 *
 * Callbacks run on the NimBLE host thread. Keep them short; post work elsewhere.
 *
 * Plain C, no NimBLE types: the firmware includes this header.
 */

#ifndef SM_GLUE_H_
#define SM_GLUE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IO capabilities. Values equal NimBLE's BLE_HS_IO_* (static assert in sm_glue.c). */
#define SM_GLUE_IO_DISPLAY_ONLY     0
#define SM_GLUE_IO_DISPLAY_YESNO    1
#define SM_GLUE_IO_KEYBOARD_ONLY    2
#define SM_GLUE_IO_NO_INPUT_OUTPUT  3
#define SM_GLUE_IO_KEYBOARD_DISPLAY 4

/* Key distribution flags. Values equal BLE_SM_PAIR_KEY_DIST_* (static assert). */
#define SM_GLUE_KEY_ENC  0x01
#define SM_GLUE_KEY_ID   0x02
#define SM_GLUE_KEY_SIGN 0x04
#define SM_GLUE_KEY_LINK 0x08

typedef enum {
    SmGlueEventPasskeyDisplay, /* show event->passkey to the user */
    SmGlueEventPasskeyRequest, /* the peer displays it: call sm_glue_passkey_reply */
    SmGlueEventNumericCompare, /* compare event->passkey: call sm_glue_numeric_reply */
    SmGlueEventOobRequest, /* legacy OOB data requested; not supported */
    SmGlueEventEncChange, /* encryption changed; status and flags are valid */
    SmGlueEventRepeatPairing, /* the peer re-pairs over an existing bond */
} SmGlueEventKind;

typedef struct {
    SmGlueEventKind kind;
    uint16_t conn_handle;
    uint32_t passkey; /* display / numeric comparison value */
    int16_t status; /* SmGlueEventEncChange only */
    bool encrypted;
    bool authenticated;
    bool bonded;
    uint8_t key_size;
} SmGlueEvent;

typedef void (*SmGlueCb)(const SmGlueEvent* event, void* ctx);

/* Register the pairing consumer. NULL restores the companion behaviour. */
void sm_glue_set_consumer(SmGlueCb cb, void* ctx);
bool sm_glue_has_consumer(void);

/* Pairing parameters. Takes effect on the next pairing procedure. */
void sm_glue_configure(
    uint8_t io_cap,
    bool bonding,
    bool mitm,
    bool secure_connections,
    uint8_t our_key_dist,
    uint8_t their_key_dist);
/* Restore the companion parameters (DISPLAY_ONLY, bonding, MITM, legacy). */
void sm_glue_config_default(void);

/* Start pairing or encryption on a link this host owns. */
bool sm_glue_pair(uint16_t conn_handle);
/* Answer SmGlueEventPasskeyRequest (or supply a display passkey). */
bool sm_glue_passkey_reply(uint16_t conn_handle, uint32_t passkey);
/* Answer SmGlueEventNumericCompare. */
bool sm_glue_numeric_reply(uint16_t conn_handle, bool accept);

/* --- LE Secure Connections OOB (TASK-689) ------------------------------------
 *
 * The confirm value derives from this host's SC public key, so only the Flipper
 * can produce it. Generate ours, hand it to the peer out of band, take the
 * peer's back, and pairing uses both.
 *
 * Generate our OOB random and confirm values. The key pair behind them is made
 * once per host run, so the values stay valid for the pairing that follows.
 * Both buffers are 16 bytes. */
bool sm_glue_oob_generate(uint8_t* out_random, uint8_t* out_confirm);

/* Arm the peer's OOB values, normally before pairing starts. This also tells
 * the peer we hold its OOB data, which its pairing request must agree with. If
 * a pairing is already waiting for them, this completes it. Both buffers are
 * 16 bytes. */
bool sm_glue_oob_set_peer(const uint8_t* random, const uint8_t* confirm);

/* Forget both sides' OOB values and stop advertising that we hold the peer's. */
void sm_glue_oob_clear(void);

/* True when Secure Connections is built into this firmware. */
bool sm_glue_sc_supported(void);

/* --- nimble_glue internal ---------------------------------------------------- */
/* True when the consumer took the event and the caller must not run its own
 * companion handling. action is NimBLE's BLE_SM_IOACT_*. */
bool sm_glue_on_passkey_action(uint16_t conn_handle, uint8_t action, uint32_t numcmp);
void sm_glue_on_enc_change(uint16_t conn_handle, int status);
void sm_glue_on_repeat_pairing(uint16_t conn_handle);

#ifdef __cplusplus
}
#endif

#endif /* SM_GLUE_H_ */
