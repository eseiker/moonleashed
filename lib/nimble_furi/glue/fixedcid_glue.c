/*
 * Fixed L2CAP CID relay over NimBLE (TASK-663). See fixedcid_glue.h.
 *
 * A fixed L2CAP channel in NimBLE is just a ble_l2cap_chan in the connection's
 * channel list, keyed by scid, with an rx_fn — exactly how ATT/SIG/SM are set
 * up (ble_att_create_chan). We install one for the caller's CID on each
 * connection: ble_l2cap_rx dispatches an incoming B-frame to the channel it
 * finds by scid, and ble_l2cap_tx sends a raw PDU by prepending the L2CAP
 * header with the channel's dcid.
 *
 * This file reaches NimBLE internals, so it is compiled with the host/src
 * include path (see lib/nimble.scons).
 */

#include <furi.h>
#include <string.h>

#include "nimble/ble.h"
#include "os/os_mbuf.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_l2cap.h"

#include "ble_hs_priv.h"
#include "ble_hs_conn_priv.h"
#include "ble_l2cap_priv.h"

#include "fixedcid_glue.h"

#define TAG "FixedCid"

#define FIXEDCID_MAX   4
#define FIXEDCID_RXBUF 600

static FixedCidRxCb s_cb;
static void* s_ctx;
static struct {
    uint16_t cid;
    uint16_t mtu;
    bool used;
} s_reg[FIXEDCID_MAX];
static uint8_t s_rxbuf[FIXEDCID_RXBUF];

/* Host thread: copy the B-frame flat and hand it up. ble_l2cap_rx frees the
 * mbuf after we return, so we do not touch *om. */
static int fixedcid_rx(struct ble_l2cap_chan* chan, struct os_mbuf** om) {
    uint16_t len = OS_MBUF_PKTLEN(*om);
    if(len > sizeof(s_rxbuf)) len = sizeof(s_rxbuf);
    if(len && os_mbuf_copydata(*om, 0, len, s_rxbuf) == 0 && s_cb) {
        s_cb(chan->conn_handle, chan->scid, s_rxbuf, len, s_ctx);
    }
    return 0;
}

/* Caller holds ble_hs lock. */
static bool fixedcid_install(struct ble_hs_conn* conn, uint16_t cid, uint16_t mtu) {
    if(ble_hs_conn_chan_find_by_scid(conn, cid)) return true; /* already present */
    struct ble_l2cap_chan* chan = ble_l2cap_chan_alloc(conn->bhc_handle);
    if(!chan) {
        FURI_LOG_E(TAG, "chan alloc failed (pool full?)");
        return false;
    }
    chan->scid = cid;
    chan->dcid = cid;
    chan->my_mtu = mtu;
    chan->rx_fn = fixedcid_rx;
    if(ble_hs_conn_chan_insert(conn, chan) != 0) {
        ble_l2cap_chan_free(conn, chan);
        return false;
    }
    return true;
}

void fixedcid_init(FixedCidRxCb dispatch, void* ctx) {
    ble_hs_lock();
    s_cb = dispatch;
    s_ctx = ctx;
    ble_hs_unlock();
}

void fixedcid_deinit(void) {
    ble_hs_lock();
    s_cb = NULL;
    s_ctx = NULL;
    memset(s_reg, 0, sizeof(s_reg));
    ble_hs_unlock();
}

bool fixedcid_register(uint16_t cid, uint16_t mtu) {
    if(cid == 0 || cid == BLE_L2CAP_CID_ATT || cid == BLE_L2CAP_CID_SIG ||
       cid == BLE_L2CAP_CID_SM) {
        return false;
    }
    if(mtu == 0) mtu = FIXEDCID_RXBUF;

    ble_hs_lock();
    int slot = -1;
    for(int i = 0; i < FIXEDCID_MAX; i++) {
        if(s_reg[i].used && s_reg[i].cid == cid) {
            slot = i;
            break;
        }
        if(slot < 0 && !s_reg[i].used) slot = i;
    }
    if(slot < 0) {
        ble_hs_unlock();
        FURI_LOG_E(TAG, "no free CID slot");
        return false;
    }
    s_reg[slot].cid = cid;
    s_reg[slot].mtu = mtu;
    s_reg[slot].used = true;

    /* Install on every current connection. */
    for(int i = 0;; i++) {
        struct ble_hs_conn* conn = ble_hs_conn_find_by_idx(i);
        if(!conn) break;
        fixedcid_install(conn, cid, mtu);
    }
    ble_hs_unlock();
    FURI_LOG_I(TAG, "registered fixed CID 0x%04X mtu=%u", cid, mtu);
    return true;
}

bool fixedcid_unregister(uint16_t cid) {
    bool found = false;
    ble_hs_lock();
    for(int i = 0; i < FIXEDCID_MAX; i++) {
        if(s_reg[i].used && s_reg[i].cid == cid) {
            s_reg[i].used = false;
            found = true;
        }
    }
    ble_hs_unlock();
    return found;
}

bool fixedcid_send(uint16_t conn_handle, uint16_t cid, const uint8_t* data, uint16_t len) {
    ble_hs_lock();
    struct ble_hs_conn* conn = ble_hs_conn_find(conn_handle);
    struct ble_l2cap_chan* chan = conn ? ble_hs_conn_chan_find_by_scid(conn, cid) : NULL;
    if(!chan) {
        ble_hs_unlock();
        return false;
    }
    struct os_mbuf* txom = ble_hs_mbuf_from_flat(data, len);
    if(!txom) {
        ble_hs_unlock();
        return false;
    }
    int rc = ble_l2cap_tx(conn, chan, txom); /* consumes txom */
    ble_hs_unlock();
    if(rc != 0) FURI_LOG_W(TAG, "tx rc=%d", rc);
    return rc == 0;
}

void fixedcid_on_connect(uint16_t conn_handle) {
    ble_hs_lock();
    struct ble_hs_conn* conn = ble_hs_conn_find(conn_handle);
    if(conn) {
        for(int i = 0; i < FIXEDCID_MAX; i++) {
            if(s_reg[i].used) fixedcid_install(conn, s_reg[i].cid, s_reg[i].mtu);
        }
    }
    ble_hs_unlock();
}
