#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Fixed L2CAP CID relay on the resident NimBLE host (TASK-663). Distinct from
 * the CoC API (l2cap_coc.h): a fixed channel carries raw L2CAP B-frames on a
 * caller-chosen CID (e.g. 0x003A for virtual watch pairing / Magnet), with no
 * PSM, no CoC credits and no SDU segmentation. Each PDU must fit one L2CAP
 * frame within the negotiated MTU.
 *
 * Received PDUs are delivered on the BLE dispatch thread, off the NimBLE host
 * thread (see ble_dispatch), with the PDU bytes copied.
 */

/** Received-PDU callback. data is valid only during the call. */
typedef void (*BleL2capFixedCallback)(
    uint16_t connection_handle,
    uint16_t cid,
    const uint8_t* data,
    uint16_t data_len,
    void* context);

/** Link callback. connected is true when a link on which the registered CIDs
 *  are available came up, false when it went down. The same link can be
 *  reported up more than once, so treat the reports as idempotent. */
typedef void (
    *BleL2capFixedLinkCallback)(uint16_t connection_handle, bool connected, void* context);

/** One BLE address: type is 0 public, 1 random, 2 public identity, 3 random
 *  identity, and value is little-endian, the order NimBLE and HCI use. */
typedef struct {
    uint8_t type;
    uint8_t value[6];
} BleL2capFixedAddr;

/** What the link-info callback reports. The over-the-air addresses are the
 *  link-layer ones, which is how each end identifies the link; the identity
 *  addresses are the resolved ones, and match the over-the-air pair when no
 *  resolvable private address is in use. disconnect_reason is the HCI reason on
 *  a down report, for example 19 for remote user terminated, and 0 on an up
 *  report. */
typedef struct {
    uint16_t connection_handle;
    bool connected;
    uint8_t disconnect_reason;
    BleL2capFixedAddr peer_ota;
    BleL2capFixedAddr peer_id;
    BleL2capFixedAddr our_ota;
    BleL2capFixedAddr our_id;
} BleL2capFixedLinkInfo;

/** Link callback carrying addresses and the disconnect reason. info is valid
 *  only during the call. */
typedef void (*BleL2capFixedLinkInfoCallback)(const BleL2capFixedLinkInfo* info, void* context);

/** Initialize the fixed-CID relay (idempotent). */
void ble_l2cap_fixed_init(void);

/** Tear it down: drop the callback and every registered CID. */
void ble_l2cap_fixed_deinit(void);

/** Set the single receive callback (NULL to clear). */
void ble_l2cap_fixed_set_callback(BleL2capFixedCallback callback, void* context);

/** Set the single link callback (NULL to clear). Delivered on the BLE dispatch
 *  thread for every peripheral and central link. ble_l2cap_fixed_register also
 *  reports each link that already exists, so a caller learns the connection
 *  handle to send on before the peer has sent anything. */
void ble_l2cap_fixed_set_link_callback(BleL2capFixedLinkCallback callback, void* context);

/** Set the single link-info callback (NULL to clear). It reports the same links
 *  as ble_l2cap_fixed_set_link_callback, on the same thread, and adds the
 *  addresses and the disconnect reason. Both callbacks can be set at once; this
 *  one runs first, so a consumer that forwards both sends the detailed report
 *  before the plain one. */
void ble_l2cap_fixed_set_link_info_callback(BleL2capFixedLinkInfoCallback callback, void* context);

/** Relay a fixed CID on every connection (current and future). mtu 0, or an mtu
 *  above 2044, uses 2044; NimBLE rejects a PDU above the channel MTU.
 *  Rejects 0 and the standard CIDs 4 (ATT), 5 (SIG), 6 (SM). */
bool ble_l2cap_fixed_register(uint16_t cid, uint16_t mtu);

/** Stop relaying a CID. */
bool ble_l2cap_fixed_unregister(uint16_t cid);

/** Transmit one raw PDU on a registered fixed CID of a connection. */
bool ble_l2cap_fixed_send(
    uint16_t connection_handle,
    uint16_t cid,
    const uint8_t* data,
    uint16_t data_len);

#ifdef __cplusplus
}
#endif
