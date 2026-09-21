#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BleLinkDisconnectOk, /**< The link goes down shortly after. */
    BleLinkDisconnectNotConnected, /**< No such link. */
    BleLinkDisconnectFailed, /**< The host could not take the request. */
} BleLinkDisconnectStatus;

/** Common HCI reasons; any HCI error code is accepted. */
#define BLE_LINK_REASON_DEFAULT       0x00 /* remote user terminated */
#define BLE_LINK_REASON_USER          0x13
#define BLE_LINK_REASON_LOW_RESOURCES 0x14
#define BLE_LINK_REASON_POWER_OFF     0x15

/** Drop one link by connection handle, whatever its role. The disconnection
 *  is reported through the owner's usual events. */
BleLinkDisconnectStatus ble_link_disconnect(uint16_t connection_handle, uint8_t reason);

#ifdef __cplusplus
}
#endif
