#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BLE pairing (SMP) control on the resident NimBLE host (TASK-665).
 *
 * The firmware pairs the way the mobile companion app expects: it shows a
 * 6-digit passkey on the Flipper and injects it itself. An app that calls
 * ble_security_init takes pairing over for as long as it lives: it chooses the
 * pairing parameters, answers the passkey or numeric-comparison prompt, and
 * sees encryption come up. ble_security_deinit restores the companion
 * behaviour.
 *
 * Events are delivered on the BLE dispatch thread, off the NimBLE host thread
 * (see the CoC, GATT-client and fixed-CID APIs, which do the same).
 */

typedef enum {
    BleSecurityIoDisplayOnly = 0,
    BleSecurityIoDisplayYesNo = 1,
    BleSecurityIoKeyboardOnly = 2,
    BleSecurityIoNoInputNoOutput = 3,
    BleSecurityIoKeyboardDisplay = 4,
} BleSecurityIoCapability;

/** Key distribution flags for ble_security_configure. */
#define BLE_SECURITY_KEY_ENC  0x01
#define BLE_SECURITY_KEY_ID   0x02
#define BLE_SECURITY_KEY_SIGN 0x04
#define BLE_SECURITY_KEY_LINK 0x08

typedef enum {
    /** Show a passkey to the user, then answer with ble_security_passkey_reply.
     *  The value is the app's choice; the peer types the same number. */
    BleSecurityEventTypePasskeyDisplay,
    /** The peer displays a passkey: answer with ble_security_passkey_reply. */
    BleSecurityEventTypePasskeyRequest,
    /** Compare event->passkey with the peer's, then answer with
     *  ble_security_numeric_comparison_reply. Secure connections only. */
    BleSecurityEventTypeNumericComparison,
    /** Secure Connections OOB data is needed and the peer's values are not
     *  armed yet. Supply them with ble_security_oob_set_peer, which finishes
     *  the pairing. Arming them before pairing avoids this event entirely. */
    BleSecurityEventTypeOobRequest,
    /** Encryption changed. status, encrypted, authenticated, bonded and
     *  key_size are valid. */
    BleSecurityEventTypeEncryptionChanged,
    /** The peer re-pairs over an existing bond. The host drops the stale bond
     *  and lets pairing proceed; this event is informational. */
    BleSecurityEventTypeRepeatPairing,
} BleSecurityEventType;

typedef struct {
    BleSecurityEventType type;
    uint16_t connection_handle;
    uint32_t passkey;
    int16_t status;
    bool encrypted;
    bool authenticated;
    bool bonded;
    uint8_t key_size;
} BleSecurityEvent;

/** Event callback. event is valid only during the call. */
typedef void (*BleSecurityCallback)(const BleSecurityEvent* event, void* context);

/** Take pairing over from the firmware (idempotent). */
void ble_security_init(void);

/** Give pairing back to the firmware and restore its parameters. */
void ble_security_deinit(void);

/** Set the event callback (NULL to clear). */
void ble_security_set_callback(BleSecurityCallback callback, void* context);

/** Set the pairing parameters. Takes effect on the next pairing procedure. */
bool ble_security_configure(
    BleSecurityIoCapability io_capability,
    bool bonding,
    bool mitm,
    bool secure_connections,
    uint8_t our_key_distribution,
    uint8_t their_key_distribution);

/** Start pairing (or re-encryption from a stored bond) on a connection. */
bool ble_security_pair(uint16_t connection_handle);

/** Answer a passkey display or request. */
bool ble_security_passkey_reply(uint16_t connection_handle, uint32_t passkey);

/** Answer a numeric comparison. */
bool ble_security_numeric_comparison_reply(uint16_t connection_handle, bool accept);

/** Length of each Secure Connections OOB value. */
#define BLE_SECURITY_OOB_LEN 16

/** True when Secure Connections is built into this firmware. When it is false,
 *  ble_security_configure rejects secure_connections and the OOB calls fail. */
bool ble_security_secure_connections_supported(void);

/*
 * LE Secure Connections OOB pairing.
 *
 * The OOB confirm value derives from this host's Secure Connections public key,
 * so only the Flipper can produce it: a remote driver cannot compute a valid
 * one. Generate ours, carry it to the peer over whatever out-of-band channel
 * the protocol uses, take the peer's values back, and pairing consumes both.
 */

/** Generate our OOB random and confirm values. Each buffer takes
 *  BLE_SECURITY_OOB_LEN bytes. The values stay valid for the pairing that
 *  follows, because the key pair behind them is made once per host run. */
bool ble_security_oob_generate(uint8_t* out_random, uint8_t* out_confirm);

/** Arm the peer's OOB values, normally before pairing starts. This also makes
 *  our pairing request say we hold the peer's OOB data. If a pairing is already
 *  waiting for them, this completes it. */
bool ble_security_oob_set_peer(const uint8_t* random, const uint8_t* confirm);

/** Forget both sides' OOB values and stop advertising that we hold the peer's. */
void ble_security_oob_clear(void);

#ifdef __cplusplus
}
#endif
