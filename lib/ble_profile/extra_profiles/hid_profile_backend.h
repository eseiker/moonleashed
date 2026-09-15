#pragma once

/*
 * Firmware-internal HID report backend hook (KNOW-609). NOT part of the
 * app-facing SDK: this header is deliberately excluded from ble_profile's
 * SDK_HEADERS so the public API version does not change.
 *
 * When a backend is installed, every ble_profile_hid_* sender routes to it
 * instead of the stock CPU2 HID service. The firmware bt service installs a
 * NimBLE-backed implementation when a NimBLE host drives the radio, so existing
 * apps (bad_usb, hid_app, u2f) keep working unmodified. Install NULL to restore
 * the CPU2 path.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool (*kb_press)(uint16_t button);
    bool (*kb_release)(uint16_t button);
    bool (*kb_release_all)(void);
    bool (*consumer_press)(uint16_t button);
    bool (*consumer_release)(uint16_t button);
    bool (*consumer_release_all)(void);
    bool (*mouse_move)(int8_t dx, int8_t dy);
    bool (*mouse_press)(uint8_t button);
    bool (*mouse_release)(uint8_t button);
    bool (*mouse_release_all)(void);
    bool (*mouse_scroll)(int8_t delta);
} BleProfileHidBackend;

/** Install (or clear, with NULL) the HID report backend. */
void ble_profile_hid_set_backend(const BleProfileHidBackend* backend);

#ifdef __cplusplus
}
#endif
