/* Plain-C interface to the NimBLE host. Includes no NimBLE headers. */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start the host on an acquired controller. */
bool nimble_glue_start(void);

/* True once started: the HAL then routes to NimBLE, synced or not. */
bool nimble_glue_is_started(void);
bool nimble_glue_is_synced(void);
bool nimble_glue_is_advertising(void);
bool nimble_glue_is_connected(void);
bool nimble_glue_is_pairing(void);
/* Passkey to show while pairing. */
uint32_t nimble_glue_passkey(void);

/* The Bluetooth setting. Off stops advertising and drops the link. */
void nimble_glue_set_advertising_enabled(bool enabled);
void nimble_glue_disconnect(void);
void nimble_glue_forget_bonds(void);
/* Loads bonds.bin from a card that mounted after the start. */
void nimble_glue_reload_bonds(void);

void nimble_glue_set_battery_level(uint8_t level);
void nimble_glue_set_power_state(bool charging);

/* Show and advertise HID with the keyboard appearance, while a HID app runs. */
void nimble_glue_set_hid_advertised(bool advertised);
void nimble_glue_hid_set_report_map(const uint8_t* data, uint16_t len);
/* See hid_gatt_input_report. */
bool nimble_glue_hid_input_report(uint8_t report_id, const uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif
