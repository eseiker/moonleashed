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

/** Initialize the fixed-CID relay (idempotent). */
void ble_l2cap_fixed_init(void);

/** Tear it down: drop the callback and every registered CID. */
void ble_l2cap_fixed_deinit(void);

/** Set the single receive callback (NULL to clear). */
void ble_l2cap_fixed_set_callback(BleL2capFixedCallback callback, void* context);

/** Relay a fixed CID on every connection (current and future). mtu 0 = default.
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
