#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Raw advertising control on the resident NimBLE host (TASK-646). A FAP that
 * needs peers to discover it under its own payload — e.g. the MigrationKit DCT
 * FC73 service advertisement with the session id — installs it here. The
 * payload replaces the companion advertisement until furi_ble_adv_clear();
 * connections already up are unaffected, only what new peers discover changes.
 * The legacy advertiser carries one payload at a time, so this is modal for the
 * advertisement (not for the links).
 */

/** Install a raw advertising payload and optional scan response.
 *  @param adv      advertising data (AD structures), at most 31 bytes
 *  @param adv_len  length of adv
 *  @param rsp      scan response data, at most 31 bytes (may be NULL when rsp_len is 0)
 *  @param rsp_len  length of rsp
 *  @return true if installed (and re-advertised when allowed) */
bool furi_ble_adv_set(const uint8_t* adv, uint8_t adv_len, const uint8_t* rsp, uint8_t rsp_len);

/** Install a raw advertisement that stops when the first peer connects
 *  (TASK-721). A protocol that walks one connection per stage needs that: it is
 *  how a raw-HCI advertiser with auto_restart off behaves. The advertisement
 *  stays off until furi_ble_adv_restart. Arguments are those of furi_ble_adv_set.
 *  Pass stop_on_connect false for the plain behaviour, where the host keeps
 *  advertising after a connection. */
bool furi_ble_adv_set_ex(
    const uint8_t* adv,
    uint8_t adv_len,
    const uint8_t* rsp,
    uint8_t rsp_len,
    bool stop_on_connect);

/** Advertise the installed payload again after advertise-once stopped it. */
void furi_ble_adv_restart(void);

/** Drop the raw payload and restore the companion advertisement. */
void furi_ble_adv_clear(void);

#ifdef __cplusplus
}
#endif
