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

/** Drop the raw payload and restore the companion advertisement. */
void furi_ble_adv_clear(void);

#ifdef __cplusplus
}
#endif
