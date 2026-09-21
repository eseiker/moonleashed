#pragma once

#include <stdint.h>

struct os_mbuf;

/* Host thread: a notification or indication on a central link. */
void gatt_client_on_notify(uint16_t conn, uint16_t attr_handle, struct os_mbuf* om);
