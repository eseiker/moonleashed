/*
 * Flipper Serial Service GATT server on NimBLE (TASK-589, direction "B",
 * milestone B1).
 *
 * Reproduces the stock CPU2 Serial Service (targets/f7/ble_glue/services/
 * serial_service.c) with the same 128-bit UUIDs and characteristic properties,
 * so the mobile companion app sees the same service when NimBLE is the host on
 * a BLE HCI Layer radio (radio_stack_type 2, KNOW-580). B1 stands the server up
 * and answers reads/writes; the RPC bridge and flow control land in B3 and
 * security (bonded, encrypted access) in B2.
 *
 * Only glue .c files include this header, so it may use NimBLE types; the FAP
 * entry point stays behind the plain-C nimble_glue.h.
 */

#ifndef SERIAL_GATT_H_
#define SERIAL_GATT_H_

#include <stdbool.h>
#include <stdint.h>

/* Registers the Serial Service (and its four characteristics) with the NimBLE
 * GATT server. Call after ble_svc_gap_init/ble_svc_gatt_init and before the
 * host syncs (ble_gatts_start runs during sync). Returns 0 on success. */
int serial_gatt_register(void);

/* Open the RPC record and allocate the TX-confirmation semaphore. Call once
 * after nimble_glue_start; pair with serial_gatt_deinit before the FAP exits. */
void serial_gatt_init(void);
void serial_gatt_deinit(void);

/* Track the active connection. On connect this opens an RPC session bridged to
 * the RX/TX/flow characteristics; on disconnect it closes the session and
 * unblocks any pending TX. */
void serial_gatt_set_conn(uint16_t conn_handle, bool connected);

/* Feed an indication-confirmation event (BLE_GAP_EVENT_NOTIFY_TX) so the RPC
 * send path can advance to the next chunk. */
void serial_gatt_on_notify_tx(uint16_t attr_handle, int status);

/* Total bytes the central has written to the RX characteristic on the current
 * connection. B1 introspection for the FAP UI. */
uint32_t serial_gatt_rx_bytes(void);

#endif /* SERIAL_GATT_H_ */
