#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Device Information and Battery, as the stock serial profile serves them. */
void info_gatt_init(void);
int info_gatt_register(void);

/* Any thread. */
void info_gatt_set_battery_level(uint8_t level);
void info_gatt_set_power_state(bool charging);
