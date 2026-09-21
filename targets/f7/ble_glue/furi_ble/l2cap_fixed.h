#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Raw L2CAP B-frames on a fixed CID of the app's choice, such as 0x003A. No
 * PSM, credits or segmentation: each PDU fits one frame. Callbacks run on the
 * BLE dispatch thread and their data is valid during the call. */

typedef void (*BleL2capFixedCallback)(
    uint16_t connection_handle,
    uint16_t cid,
    const uint8_t* data,
    uint16_t data_len,
    void* context);

/** type is 0 public, 1 random, 2 public identity, 3 random identity. value is
 *  little-endian. */
typedef struct {
    uint8_t type;
    uint8_t value[6];
} BleL2capFixedAddr;

/** A link came up or went down. disconnect_reason is the HCI reason (19 is
 *  remote user terminated), 0 when connected. The ota addresses are the
 *  link-layer ones; the id addresses are resolved. The same link can be
 *  reported up more than once. */
typedef struct {
    uint16_t connection_handle;
    bool connected;
    uint8_t disconnect_reason;
    BleL2capFixedAddr peer_ota;
    BleL2capFixedAddr peer_id;
    BleL2capFixedAddr our_ota;
    BleL2capFixedAddr our_id;
} BleL2capFixedLinkInfo;

typedef void (*BleL2capFixedLinkCallback)(const BleL2capFixedLinkInfo* info, void* context);

void ble_l2cap_fixed_init(void);

/** Drop the callbacks and every registered CID. */
void ble_l2cap_fixed_deinit(void);

void ble_l2cap_fixed_set_callback(BleL2capFixedCallback callback, void* context);

/** Reports every link. ble_l2cap_fixed_register also reports the links that
 *  already exist. */
void ble_l2cap_fixed_set_link_callback(BleL2capFixedLinkCallback callback, void* context);

/** Relay a CID on every current and future link. mtu 0 or above 2044 means
 *  2044. Rejects 0, the standard CIDs 4, 5 and 6, and 0x40-0x7F (CoC). */
bool ble_l2cap_fixed_register(uint16_t cid, uint16_t mtu);

bool ble_l2cap_fixed_unregister(uint16_t cid);

bool ble_l2cap_fixed_send(
    uint16_t connection_handle,
    uint16_t cid,
    const uint8_t* data,
    uint16_t data_len);

#ifdef __cplusplus
}
#endif
