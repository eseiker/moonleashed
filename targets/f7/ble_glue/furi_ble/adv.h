#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Advertise a raw payload in place of the companion advertisement, also while
 *  the companion is connected. Links already up stay up. adv and rsp hold AD
 *  structures, at most 31 bytes each.
 *
 *  If stop_on_connect is true, advertising stops when a peer connects and stays
 *  off until furi_ble_adv_restart. The payload is dropped when the app exits.
 *  Turning Bluetooth off in settings stops advertising and drops the
 *  companion's link, but leaves links the app brought in. */
bool furi_ble_adv_set_ex(
    const uint8_t* adv,
    uint8_t adv_len,
    const uint8_t* rsp,
    uint8_t rsp_len,
    bool stop_on_connect);

/** furi_ble_adv_set_ex with stop_on_connect false. */
bool furi_ble_adv_set(const uint8_t* adv, uint8_t adv_len, const uint8_t* rsp, uint8_t rsp_len);

/** Advertise the payload again after a connection stopped it. */
void furi_ble_adv_restart(void);

/** Drop the payload and bring the companion advertisement back. */
void furi_ble_adv_clear(void);

#ifdef __cplusplus
}
#endif
