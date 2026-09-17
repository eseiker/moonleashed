/*
 * Fixed L2CAP CID relay on the resident NimBLE host (TASK-663).
 *
 * NimBLE opens only the three standard fixed channels (ATT 0x0004, SIG 0x0005,
 * SM 0x0006) and gives no public way to register another one. Virtual watch
 * pairing (Magnet) runs on fixed CID 0x003A, which is neither of those and not
 * a CoC, so the CoC bridge cannot carry it (KNOW-662).
 *
 * This core installs an extra fixed channel on each connection: incoming
 * L2CAP B-frames on the registered CID are handed to a callback, and a caller
 * can transmit a raw PDU on that CID. It reaches NimBLE internals
 * (ble_l2cap_priv.h etc.), so it is built inside lib/nimble and this header is
 * kept plain C for the firmware to include.
 *
 * This is a raw B-frame relay: no CoC credits, no SDU segmentation. The peer's
 * PDU must fit one ACL L2CAP frame within the negotiated MTU.
 */

#ifndef FIXEDCID_GLUE_H_
#define FIXEDCID_GLUE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Runs on the NimBLE host thread. data is valid only during the call. */
typedef void (*FixedCidRxCb)(
    uint16_t conn_handle,
    uint16_t cid,
    const uint8_t* data,
    uint16_t len,
    void* ctx);

/* One BLE address, laid out like NimBLE's ble_addr_t: type is 0 public,
 * 1 random, 2 public identity, 3 random identity, and val is little-endian. */
typedef struct {
    uint8_t type;
    uint8_t val[6];
} FixedCidAddr;

/* What a link report carries. The over-the-air addresses are the link-layer
 * ones, which is what a peer identifies the link by; the identity addresses are
 * the resolved ones, and match the over-the-air pair when no resolvable private
 * address is in use. reason is the HCI disconnect reason on a down report, for
 * example 19 for remote user terminated, and 0 on an up report (TASK-713). */
typedef struct {
    uint16_t conn_handle;
    bool up;
    uint8_t reason;
    FixedCidAddr peer_ota;
    FixedCidAddr peer_id;
    FixedCidAddr our_ota;
    FixedCidAddr our_id;
} FixedCidLinkInfo;

/* Runs on the NimBLE host thread, without the ble_hs lock. info->up is true
 * when a link on which the registered CIDs are available came up, false when it
 * went down. The same link can be reported up more than once. info is valid
 * only during the call. */
typedef void (*FixedCidLinkCb)(const FixedCidLinkInfo* info, void* ctx);

/* Set the single receive dispatcher. */
void fixedcid_init(FixedCidRxCb dispatch, void* ctx);

/* Set the single link dispatcher (NULL to clear). Links are reported from a
 * GAP event listener, so peripheral and central links both appear, and
 * fixedcid_register reports every link that already exists. */
void fixedcid_set_link_cb(FixedCidLinkCb dispatch, void* ctx);
void fixedcid_deinit(void);

/* Register a fixed CID (e.g. 0x003A) with a channel MTU. The channel is created
 * on every current connection and on each later one. mtu 0, or an mtu above the
 * receive buffer (2044), uses 2044: NimBLE then rejects a larger PDU instead of
 * the relay truncating it. Returns false on a bad CID (0, or a standard 4/5/6)
 * or when the table is full. */
bool fixedcid_register(uint16_t cid, uint16_t mtu);

/* Stop relaying a CID. Existing channels stay until their link drops, but
 * their PDUs are no longer delivered. */
bool fixedcid_unregister(uint16_t cid);

/* Transmit one raw PDU on a registered fixed CID of a connection. */
bool fixedcid_send(uint16_t conn_handle, uint16_t cid, const uint8_t* data, uint16_t len);

/* Install the registered fixed channels on a new connection. Called from the
 * GAP connect handler in nimble_glue (host thread). */
void fixedcid_on_connect(uint16_t conn_handle);

#ifdef __cplusplus
}
#endif

#endif /* FIXEDCID_GLUE_H_ */
