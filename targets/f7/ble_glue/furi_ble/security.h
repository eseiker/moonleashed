#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pairing control. Between ble_security_init and ble_security_deinit the app
 * drives pairing on every link: it sets the parameters, answers the prompts
 * and sees encryption come up. Otherwise the companion's PIN pairing runs, also
 * after an app exits without deinit. Events arrive on the BLE dispatch thread.
 * Legacy OOB is not supported: that pairing ends the link with an
 * EncryptionChanged status of 8 (not supported). */

typedef enum {
    BleSecurityIoDisplayOnly = 0,
    BleSecurityIoDisplayYesNo = 1,
    BleSecurityIoKeyboardOnly = 2,
    BleSecurityIoNoInputNoOutput = 3,
    BleSecurityIoKeyboardDisplay = 4,
} BleSecurityIoCapability;

#define BLE_SECURITY_KEY_ENC  0x01
#define BLE_SECURITY_KEY_ID   0x02
#define BLE_SECURITY_KEY_SIGN 0x04
#define BLE_SECURITY_KEY_LINK 0x08

typedef enum {
    /** Pick a passkey, show it, and pass it to ble_security_passkey_reply. */
    BleSecurityEventTypePasskeyDisplay,
    /** Enter the peer's passkey with ble_security_passkey_reply. */
    BleSecurityEventTypePasskeyRequest,
    /** Compare passkey with the peer's; answer with
     *  ble_security_numeric_comparison_reply. */
    BleSecurityEventTypeNumericComparison,
    /** The peer's OOB values are needed; ble_security_oob_set_peer finishes
     *  the pairing. Arming them before pairing skips this event. Without a
     *  reply the pairing fails at the 30 s timeout. */
    BleSecurityEventTypeOobRequest,
    /** status, encrypted, authenticated, bonded and key_size are valid. A
     *  pairing that could not start reports its host error here too. */
    BleSecurityEventTypeEncryptionChanged,
    /** The peer pairs again over a bond; the old bond is dropped. */
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

typedef void (*BleSecurityCallback)(const BleSecurityEvent* event, void* context);

void ble_security_init(void);

/** Give pairing back to the companion and restore its parameters. */
void ble_security_deinit(void);

void ble_security_set_callback(BleSecurityCallback callback, void* context);

/** Takes effect on the next pairing. */
bool ble_security_configure(
    BleSecurityIoCapability io_capability,
    bool bonding,
    bool mitm,
    bool secure_connections,
    uint8_t our_key_distribution,
    uint8_t their_key_distribution);

/** Pair, or encrypt with a stored bond. */
bool ble_security_pair(uint16_t connection_handle);

bool ble_security_passkey_reply(uint16_t connection_handle, uint32_t passkey);

bool ble_security_numeric_comparison_reply(uint16_t connection_handle, bool accept);

#define BLE_SECURITY_OOB_LEN 16

/** Always true: Secure Connections is built in. */
bool ble_security_secure_connections_supported(void);

/** Our Secure Connections OOB random and confirm values, BLE_SECURITY_OOB_LEN
 *  bytes each, for the peer to receive out of band. */
bool ble_security_oob_generate(uint8_t* out_random, uint8_t* out_confirm);

/** Arm the peer's OOB values; pairing requests then say we hold them. If a
 *  pairing is waiting for them, this finishes it. */
bool ble_security_oob_set_peer(const uint8_t* random, const uint8_t* confirm);

void ble_security_oob_clear(void);

#ifdef __cplusplus
}
#endif
