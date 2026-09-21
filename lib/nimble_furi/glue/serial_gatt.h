#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Before the host syncs. Returns 0 on success. */
int serial_gatt_register(void);
void serial_gatt_init(void);

void serial_gatt_set_conn(uint16_t conn_handle, bool connected);
void serial_gatt_on_subscribe(uint16_t attr_handle, bool subscribed);
void serial_gatt_on_notify_tx(uint16_t attr_handle, int status);
