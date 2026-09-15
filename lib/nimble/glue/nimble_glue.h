/*
 * Plain-C surface over the NimBLE host for the Flipper FAP.
 *
 * The FAP entry point includes only this header, never a NimBLE header, so the
 * app-dir sources stay clear of NimBLE's MYNEWT_VAL macros (which trip the
 * firmware build's -Wundef). Everything that touches NimBLE lives in the
 * vendored lib and is built with -w.
 */

#ifndef NIMBLE_GLUE_H_
#define NIMBLE_GLUE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the NimBLE host on the already-acquired LL_ONLY controller and start
 * its event thread. Returns false if the host thread could not be created. */
bool nimble_glue_start(void);

/* True once NimBLE finished controller startup and called its sync callback. */
bool nimble_glue_is_synced(void);

/* Copies the 6-byte identity address the controller reported. Returns false
 * until the host has synced. addr is little-endian (addr[0] is the LSB). */
bool nimble_glue_get_addr(uint8_t addr[6]);

/* Number of advertising reports seen since the scan started. */
uint32_t nimble_glue_scan_count(void);

/* Copies the address of the most recently seen advertiser (little-endian).
 * Returns false until at least one report has arrived. */
bool nimble_glue_get_last_addr(uint8_t addr[6]);

/* True once the host is advertising the Serial Service as a peripheral. */
bool nimble_glue_is_advertising(void);

/* True while a central is connected. */
bool nimble_glue_is_connected(void);

/* True while legacy pairing is in progress (passkey shown, awaiting the phone). */
bool nimble_glue_is_pairing(void);

/* True once the link is encrypted/bonded. */
bool nimble_glue_is_bonded(void);

/* The 6-digit passkey to show during pairing (valid while is_pairing). */
uint32_t nimble_glue_passkey(void);

/* Bytes the connected central has written to the RX characteristic. */
uint32_t nimble_glue_rx_bytes(void);

/* Terminate the active connection, if any. */
void nimble_glue_disconnect(void);

/* Clear all bonds: the in-RAM store and the persisted file. */
void nimble_glue_forget_bonds(void);

/* BLE HID report senders (TASK-604), routed to the HID GATT server that is
 * registered alongside the Serial Service. The keyboard button is
 * (mods << 8) | keycode, matching the stock ble_profile_hid API. Each returns
 * true if the notification was queued (a central must be connected). */
bool nimble_glue_hid_kb_press(uint16_t button);
bool nimble_glue_hid_kb_release(uint16_t button);
bool nimble_glue_hid_kb_release_all(void);
bool nimble_glue_hid_consumer_press(uint16_t button);
bool nimble_glue_hid_consumer_release(uint16_t button);
bool nimble_glue_hid_consumer_release_all(void);
bool nimble_glue_hid_mouse_move(int8_t dx, int8_t dy);
bool nimble_glue_hid_mouse_press(uint8_t button);
bool nimble_glue_hid_mouse_release(uint8_t button);
bool nimble_glue_hid_mouse_release_all(void);
bool nimble_glue_hid_mouse_scroll(int8_t delta);

/* True if the HCI transport latched a fault. */
bool nimble_glue_faulted(void);

/* Gracefully stops the host (ble_hs_stop), exits the host thread, stops the HCI
 * reader, and frees every NPL FURI object. After this the FAP can unload
 * without a reboot; the caller still releases the controller afterwards. */
void nimble_glue_stop(void);

/* Internal: implemented by the transport shim, used by nimble_glue_stop and the
 * fault query. Declared here so both glue files agree on the prototype. */
void nimble_transport_furi_stop(void);
bool nimble_transport_furi_faulted(void);

#ifdef __cplusplus
}
#endif

#endif /* NIMBLE_GLUE_H_ */
