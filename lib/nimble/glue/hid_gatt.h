/*
 * BLE HID (HID-over-GATT / HOGP) server on NimBLE (TASK-604).
 *
 * Reproduces the stock CPU2 HID profile (lib/ble_profile/extra_profiles/
 * hid_profile.c + extra_services/hid_service.c) as a NimBLE GATT server so
 * bad_usb and hid_app keep working on a BLE HCI Layer radio, where the CPU2
 * host is gone. Registered alongside the Serial Service (serial_gatt.c): both
 * services live in one GATT table on one connection, so the verified companion
 * RPC flow is untouched and HID is added on top.
 *
 * The report map bytes, report structs, report ids and HID information value are
 * copied verbatim from the stock profile so a host sees the identical device.
 *
 * Only glue .c files include this header; it stays plain C so the firmware may
 * include the report-send API through nimble_glue.h.
 */

#ifndef HID_GATT_H_
#define HID_GATT_H_

#include <stdbool.h>
#include <stdint.h>

/* Register the HID, Device Information and Battery services with the NimBLE GATT
 * server. Call in nimble_glue_start after ble_svc_gatt_init and before sync,
 * next to serial_gatt_register(). Returns 0 on success. */
int hid_gatt_register(void);

/* Track the active connection so report notifications target it. Called from the
 * glue GAP connect/disconnect handler, alongside serial_gatt_set_conn(). */
void hid_gatt_set_conn(uint16_t conn_handle, bool connected);

/* Keyboard/consumer/mouse report senders. Each updates the held report and
 * notifies the matching input-report characteristic. button for the keyboard is
 * (mods << 8) | keycode, matching the stock ble_profile_hid API. Return true if
 * the notification was queued. */
bool hid_gatt_kb_press(uint16_t button);
bool hid_gatt_kb_release(uint16_t button);
bool hid_gatt_kb_release_all(void);
bool hid_gatt_consumer_press(uint16_t button);
bool hid_gatt_consumer_release(uint16_t button);
bool hid_gatt_consumer_release_all(void);
bool hid_gatt_mouse_move(int8_t dx, int8_t dy);
bool hid_gatt_mouse_press(uint8_t button);
bool hid_gatt_mouse_release(uint8_t button);
bool hid_gatt_mouse_release_all(void);
bool hid_gatt_mouse_scroll(int8_t delta);

#endif /* HID_GATT_H_ */
