#pragma once

#include <stdbool.h>

struct ble_gap_event;

/* Host thread: passkey, encryption and repeat-pairing events on any link.
 * Returns true if an app drives pairing, so the companion's PIN path skips. */
bool security_glue_on_gap_event(const struct ble_gap_event* event);
