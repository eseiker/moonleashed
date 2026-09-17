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
/* Largest PDU a fixed channel accepts. Magnet carries IKE messages, so leave
 * room well past the ATT-sized 600; 2044 is also the largest PDU a Tailcat
 * FIXED_DATA frame holds ([conn:2][cid:2] inside a 2048-byte payload). */
#define FIXEDCID_RXBUF 2044

static FixedCidRxCb s_cb;
static void* s_ctx;
static FixedCidLinkCb s_link_cb;
static void* s_link_ctx;
static struct ble_gap_event_listener s_gap_listener;
static bool s_gap_listening;
static struct {
    uint16_t cid;
    uint16_t mtu;
    bool used;
} s_reg[FIXEDCID_MAX];
/* Allocated the first time a PDU arrives on a relayed CID, so a device with no
 * fixed-CID consumer keeps the 2 KB (TASK-707). Never freed: it is read on the
 * NimBLE host thread (KNOW-703). */
static uint8_t* s_rxbuf;

static uint8_t* fixedcid_rxbuf_get(void) {
    if(!s_rxbuf) s_rxbuf = malloc(FIXEDCID_RXBUF);
    return s_rxbuf;
}

/* Caller holds ble_hs lock. */
static bool fixedcid_is_registered(uint16_t cid) {
    for(int i = 0; i < FIXEDCID_MAX; i++) {
        if(s_reg[i].used && s_reg[i].cid == cid) return true;
    }
    return false;
}

/* Host thread: copy the B-frame flat and hand it up. ble_l2cap_rx frees the
 * mbuf after we return, so we do not touch *om. The channel MTU is at most
 * FIXEDCID_RXBUF and ble_l2cap_rx rejects anything above it, so the copy is
 * never short. A channel whose CID was unregistered stays installed until its
 * link drops, so check the table before delivering. */
static int fixedcid_rx(struct ble_l2cap_chan* chan, struct os_mbuf** om) {
    uint16_t len = OS_MBUF_PKTLEN(*om);
    if(len > FIXEDCID_RXBUF) return 0;
    ble_hs_lock();
    bool deliver = fixedcid_is_registered(chan->scid);
    FixedCidRxCb cb = s_cb;
    void* ctx = s_ctx;
    ble_hs_unlock();
    uint8_t* rxbuf = deliver && cb ? fixedcid_rxbuf_get() : NULL;
    if(rxbuf && os_mbuf_copydata(*om, 0, len, rxbuf) == 0) {
        cb(chan->conn_handle, chan->scid, rxbuf, len, ctx);
    }
    return 0;
}

static void fixedcid_report_link(uint16_t conn_handle, bool up) {
    ble_hs_lock();
    FixedCidLinkCb cb = s_link_cb;
    void* ctx = s_link_ctx;
    ble_hs_unlock();
    if(cb) cb(conn_handle, up, ctx);
}

/* Host thread, ble_hs lock not held. NimBLE calls listeners for every link,
 * peripheral or central, before the link's own GAP callback. */
static int fixedcid_gap_listener(struct ble_gap_event* event, void* arg) {
    UNUSED(arg);
    switch(event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if(event->connect.status == 0) {
            fixedcid_on_connect(event->connect.conn_handle);
            fixedcid_report_link(event->connect.conn_handle, true);
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        fixedcid_report_link(event->disconnect.conn.conn_handle, false);
        break;
    default:
        break;
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
    /* The listener stays registered for the life of the host: NimBLE keeps the
     * struct in a list, and a second register returns BLE_HS_EALREADY. */
    if(!s_gap_listening) {
        s_gap_listening =
            ble_gap_event_listener_register(&s_gap_listener, fixedcid_gap_listener, NULL) == 0;
        if(!s_gap_listening) FURI_LOG_E(TAG, "GAP listener register failed");
    }
}

void fixedcid_set_link_cb(FixedCidLinkCb dispatch, void* ctx) {
    ble_hs_lock();
    s_link_cb = dispatch;
    s_link_ctx = ctx;
    ble_hs_unlock();
}

void fixedcid_deinit(void) {
    ble_hs_lock();
    s_cb = NULL;
    s_ctx = NULL;
    s_link_cb = NULL;
    s_link_ctx = NULL;
    memset(s_reg, 0, sizeof(s_reg));
    ble_hs_unlock();
}

bool fixedcid_register(uint16_t cid, uint16_t mtu) {
    if(cid == 0 || cid == BLE_L2CAP_CID_ATT || cid == BLE_L2CAP_CID_SIG ||
       cid == BLE_L2CAP_CID_SM) {
        return false;
    }
    if(mtu == 0 || mtu > FIXEDCID_RXBUF) mtu = FIXEDCID_RXBUF;

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

    /* Install on every current connection, and remember which links carry the
     * CID so they can be reported once the lock is released. */
    uint16_t links[MYNEWT_VAL(BLE_MAX_CONNECTIONS)];
    size_t link_count = 0;
    for(int i = 0;; i++) {
        struct ble_hs_conn* conn = ble_hs_conn_find_by_idx(i);
        if(!conn) break;
        if(fixedcid_install(conn, cid, mtu) && link_count < COUNT_OF(links)) {
            links[link_count++] = conn->bhc_handle;
        }
    }
    ble_hs_unlock();
    FURI_LOG_I(TAG, "registered fixed CID 0x%04X mtu=%u", cid, mtu);
    for(size_t i = 0; i < link_count; i++)
        fixedcid_report_link(links[i], true);
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
