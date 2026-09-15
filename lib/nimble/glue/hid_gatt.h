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

/* Send one complete input report, exactly as the stock ble_profile_hid code
 * builds it: report_id 1 = keyboard (8 bytes), 2 = mouse (4 bytes),
 * 3 = consumer (2 bytes). The bytes are copied into the held report (so a READ
 * returns the latest value) and a notification is queued for the host thread.
 * Returns true if the notification was queued. Safe to call from any thread. */
bool hid_gatt_input_report(uint8_t report_id, const uint8_t* data, uint16_t len);

/* Update the Battery Level characteristic (0..100) and notify a subscribed
 * peer. Returns true if the notification was queued. */
bool hid_gatt_battery_level(uint8_t level);

#endif /* HID_GATT_H_ */
