#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Before the host syncs. Returns 0 on success. */
int hid_gatt_register(void);

void hid_gatt_set_conn(uint16_t conn_handle, bool connected);
/* Reports go only to a peer that subscribed to them. */
void hid_gatt_on_subscribe(uint16_t attr_handle, bool subscribed);

/* Host thread, after sync. Hidden, the service does not make a bonded phone
 * take the Flipper for a keyboard. announce sends Service Changed. */
void hid_gatt_set_visible(bool visible, bool announce);

/* The profile's report map. Any thread. */
void hid_gatt_set_report_map(const uint8_t* data, uint16_t len);

/* A complete input report as lib/ble_profile builds it. Any thread. */
bool hid_gatt_input_report(uint8_t report_id, const uint8_t* data, uint16_t len);
