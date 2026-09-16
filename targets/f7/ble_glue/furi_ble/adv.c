/*
 * Raw advertising control (TASK-646). Thin firmware-side forwarder onto the
 * resident NimBLE host's advertising override in nimble_glue; see adv.h.
 */

#include "adv.h"

#include <nimble_glue.h>

bool furi_ble_adv_set(const uint8_t* adv, uint8_t adv_len, const uint8_t* rsp, uint8_t rsp_len) {
    return nimble_glue_adv_set_raw(adv, adv_len, rsp, rsp_len);
}

void furi_ble_adv_clear(void) {
    nimble_glue_adv_clear();
}
