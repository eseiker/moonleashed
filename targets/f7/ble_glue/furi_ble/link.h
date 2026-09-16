#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BLE link control on the resident NimBLE host (TASK-688).
 *
 * A caller that drives the host over a bridge learns connection handles from
 * the link reports it already receives, and sometimes has to drop a link
 * itself: a protocol that walks several connections abandons a stage by
 * disconnecting rather than waiting for the peer.
 *
 * This drops one link by handle, whatever its role. The disconnection itself is
 * reported the usual way, through whichever link callback the caller registered.
 */

typedef enum {
    /** The controller accepted the request; the link goes down shortly after. */
    BleLinkDisconnectOk,
    /** No such link: it is already gone, or the handle was never valid. */
    BleLinkDisconnectNotConnected,
    /** The host refused the request. */
    BleLinkDisconnectFailed,
} BleLinkDisconnectStatus;

/** Common HCI reasons. Any HCI error code is accepted. */
#define BLE_LINK_REASON_DEFAULT       0x00 /* uses remote user terminated */
#define BLE_LINK_REASON_USER          0x13 /* remote user terminated */
#define BLE_LINK_REASON_LOW_RESOURCES 0x14
#define BLE_LINK_REASON_POWER_OFF     0x15

/** Drop one link by connection handle. */
BleLinkDisconnectStatus ble_link_disconnect(uint16_t connection_handle, uint8_t reason);

#ifdef __cplusplus
}
#endif
