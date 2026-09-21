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

/* Selectable BLE operating mode (KNOW-606). The bt service picks one instead of
 * hard-coding a behavior. Peripheral variants ship now; central and raw-HCI are
 * declared slots for later. */
typedef enum {
    NimbleModePeripheralCombined = 0, /* advertise Serial Service + HID together (default) */
    NimbleModePeripheralSerial, /* Serial Service only */
    NimbleModePeripheralHid, /* HID only (+ Device Information + Battery) */
    NimbleModeCentral, /* future: scan/initiate instead of advertise */
    NimbleModeRawHci, /* future: hand the controller to a raw-HCI consumer */
    NimbleModeCount,
} NimbleMode;

/* True if the mode registers/advertises the Serial Service. */
bool nimble_mode_has_serial(NimbleMode mode);
/* True if the mode registers/advertises the HID service. */
bool nimble_mode_has_hid(NimbleMode mode);

/* Bring up the NimBLE host on the already-acquired LL_ONLY controller in the
 * given mode and start its event thread. Returns false if the mode is not
 * implemented or the host thread could not be created. */
bool nimble_glue_start(NimbleMode mode);

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

/* True while the host is scanning as observer/central. */
bool nimble_glue_is_scanning(void);

/* Raw advertising override (TASK-646). Install a caller-supplied advertising
 * payload (and optional scan response, each <= 31 bytes) that replaces the
 * companion advertisement until nimble_glue_adv_clear. Used for the DCT FC73
 * session advertisement. Existing connections are untouched; only what new
 * peers discover changes. Returns false on bad lengths. */
bool nimble_glue_adv_set_raw(
    const uint8_t* adv,
    uint8_t adv_len,
    const uint8_t* rsp,
    uint8_t rsp_len);

/* Same, with the advertise-once mode (TASK-721). When stop_on_connect is true,
 * the advertisement ends as soon as a peer connects and stays off until
 * nimble_glue_adv_restart, which is how a raw-HCI advertiser with auto_restart
 * off behaves. A protocol that expects one connection per stage needs that.
 * When it is false the host keeps advertising, which is the old behaviour. */
bool nimble_glue_adv_set_raw_ex(
    const uint8_t* adv,
    uint8_t adv_len,
    const uint8_t* rsp,
    uint8_t rsp_len,
    bool stop_on_connect);

/* Advertise the HID Service and the keyboard appearance, or not (TASK-765).
 * Off by default: a phone that sees a keyboard runs its keyboard pairing flow,
 * where the phone displays the passkey and the keyboard types it, and the
 * companion cannot type. The bt service turns it on while a HID app runs. */
void nimble_glue_set_hid_advertised(bool advertised);

/* Numeric comparison pairing (TASK-765). While pending, both ends show the same
 * six-digit number, which nimble_glue_passkey returns, and the user has to
 * confirm it. Answer with nimble_glue_numcmp_reply from any thread; the answer
 * reaches NimBLE on its host thread. */
bool nimble_glue_numcmp_pending(void);
void nimble_glue_numcmp_reply(bool accept);

/* Extra beacon (TASK-810), behind furi_hal_bt_extra_beacon_*. A non-connectable
 * advertisement of `data` from the random `address`, on the given channel map
 * (bit 0 = channel 37) and interval range. It takes over the one advertising set
 * this radio has: the companion advertisement pauses while it runs and returns
 * when it stops; connections stay up. Safe from any thread; the work runs on the
 * host thread, so start returns before the beacon is on the air. start again
 * with a new config restarts it; set_data updates it in place. */
bool nimble_glue_beacon_start(
    const uint8_t* data,
    uint8_t len,
    uint16_t min_interval_ms,
    uint16_t max_interval_ms,
    uint8_t channel_map,
    const uint8_t address[6]);
void nimble_glue_beacon_set_data(const uint8_t* data, uint8_t len);
void nimble_glue_beacon_stop(void);
/* True while the beacon is on the air. */
bool nimble_glue_beacon_is_running(void);

/* Advertise the installed payload again after advertise-once stopped it. */
void nimble_glue_adv_restart(void);

void nimble_glue_adv_clear(void);

/* Rebuild the GATT table so dynamic services (dyn_gatt.h) go live (TASK-632).
 * Asynchronous and safe from any thread. It stops advertising and drops every
 * peripheral link (like a stock profile change), rebuilds once the last link is
 * gone, sends Service Changed, then re-advertises. Deferred while a central
 * session runs. Returns false if the host is not started. */
bool nimble_glue_gatt_rebuild_request(void);

/* Central scan probe (TASK-615). Pauses advertising and starts an active scan as
 * observer/central. Peripheral links, including the companion's, stay up: the
 * HCILayer radio scans alongside them (KNOW-817, TASK-818). Scan reports arrive
 * as log lines; read nimble_glue_scan_count(). Returns false if the host is not
 * synced or a probe is already running. nimble_glue_central_probe_stop()
 * cancels the scan and resumes advertising. */
bool nimble_glue_central_probe_start(void);
void nimble_glue_central_probe_stop(void);

/* General central session (TASK-615 M2 / TASK-633). Scan for a peer advertising
 * `name`, connect as central, and report lifecycle through cb (called on the
 * host thread). The caller then drives the link with ble_gatt_client_* /
 * ble_l2cap_coc_* on the reported conn_handle. The companion's peripheral link
 * stays up throughout (TASK-818); advertising pauses until the session ends.
 * Only one central session runs at a time; nimble_glue_central_stop ends it.
 * The DCT helper below is a thin wrapper over this. */
typedef enum {
    NimbleCentralConnected, /* conn_handle valid */
    NimbleCentralDisconnected, /* status = disconnect reason */
    NimbleCentralFailed, /* status = connect failure code */
} NimbleCentralEventKind;
typedef void (
    *NimbleCentralCb)(NimbleCentralEventKind kind, uint16_t conn_handle, int status, void* ctx);
bool nimble_glue_central_start(const char* name, NimbleCentralCb cb, void* ctx);
void nimble_glue_central_stop(void);
bool nimble_glue_central_is_active(void);
uint16_t nimble_glue_central_conn_handle(void);

/* DCT central session (TASK-615, Milestone 2). Scans for a peer advertising the
 * name "FlipperDCT", connects to it as central, and opens an L2CAP CoC as the
 * client on the given PSM, while the companion stays connected (TASK-818). On
 * CoC connect the Flipper sends an opening payload; received bytes are counted
 * (nimble_glue_dct_rx_bytes). The session ends when the link drops or
 * nimble_glue_dct_stop is called. Returns false if not synced or a session is
 * already active. */
bool nimble_glue_dct_connect(uint16_t psm);
void nimble_glue_dct_stop(void);
bool nimble_glue_dct_is_active(void);
uint32_t nimble_glue_dct_rx_bytes(void);
/* Why the last DCT session ended: the connect failure code or the disconnect
 * reason (NimBLE error space: 0x2xx is an HCI status). 0 while it runs. */
int nimble_glue_dct_end_status(void);

/* True while legacy pairing is in progress (passkey shown, awaiting the phone). */
bool nimble_glue_is_pairing(void);

/* True once the link is encrypted/bonded. */
bool nimble_glue_is_bonded(void);

/* The 6-digit passkey to show during pairing (valid while is_pairing). */
uint32_t nimble_glue_passkey(void);

/* Bytes the connected central has written to the RX characteristic. */
uint32_t nimble_glue_rx_bytes(void);

/* The user's Bluetooth setting (TASK-702). Disabling stops advertising and
 * drops every peripheral link, and the host stays quiet until it is enabled
 * again. Safe from any thread. */
void nimble_glue_set_advertising_enabled(bool enabled);

/* Direct Test Mode (TASK-704). These are the standard HCI LE test commands, so
 * the HCILayer controller implements them. Stop advertising and drop links
 * first — the controller cannot run a test while it is doing anything else.
 * phy is 1 for 1M and 2 for 2M; payload is the HCI packet payload type. */
bool nimble_glue_dtm_tx_start(uint8_t channel, uint8_t payload, uint8_t phy);
bool nimble_glue_dtm_rx_start(uint8_t channel, uint8_t phy);
/* Ends the running test and reports how many packets it counted. */
bool nimble_glue_dtm_stop(uint16_t* out_packets);

/* ST vendor radio tests the HCILayer radio implements (TASK-809): an unmodulated
 * carrier at a PA level, and the received signal level. The level is only
 * available while a receiver runs, which is a DTM receiver test here, because
 * this radio lacks ACI_HAL_RX_START; read_rssi returns false otherwise. Every
 * test start waits for the link layer to go idle first. Same calling rules as
 * DTM. */
bool nimble_glue_tone_start(uint8_t channel, uint8_t pa_level);
void nimble_glue_tone_stop(void);
bool nimble_glue_read_rssi(int8_t* dbm);

/* Terminate the active connection, if any. */
void nimble_glue_disconnect(void);

/* Terminate one link by handle, whatever role it has (TASK-688). reason is an
 * HCI error code; 0 uses BLE_ERR_REM_USER_CONN_TERM. Returns the NimBLE rc: 0
 * on success, BLE_HS_ENOTCONN when the link is already gone. */
int nimble_glue_link_terminate(uint16_t conn_handle, uint8_t reason);

/* Clear all bonds: the in-RAM store and the persisted file. */
void nimble_glue_forget_bonds(void);

/* BLE HID (TASK-604 / TASK-612): send one complete input report to the HID
 * GATT server registered alongside the Serial Service. report_id 1 = keyboard
 * (8 bytes), 2 = mouse (4 bytes), 3 = consumer (2 bytes) — the byte layout the
 * stock ble_profile_hid code builds. The bt service feeds this from the
 * ble_gatt_* host shim, so HID FAPs (hid_app, bad_usb) run unmodified. Returns
 * true if the notification was queued (a central must be connected). */
bool nimble_glue_hid_input_report(uint8_t report_id, const uint8_t* data, uint16_t len);

/* Update the Battery Level characteristic (0..100) and notify the peer. */
bool nimble_glue_hid_battery_level(uint8_t level);

/* True if the HCI transport latched a fault. */
bool nimble_glue_faulted(void);

/* Check the PKA P-256 backend against a NIST test vector (TASK-698). Returns
 * false when Secure Connections is not built. */
bool sm_alg_pka_selftest(void);

/* Check the AES-CMAC that replaced mbedtls' cipher layer against the RFC 4493
 * vectors (TASK-705). The Security Manager's f4, f5, f6 and g2 all use it. */
bool sm_cmac_selftest(void);

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
