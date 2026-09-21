/* furi_ble/l2cap_fixed.h. NimBLE opens only ATT, SIG and SM as fixed channels
 * and has no API for another, so this inserts a channel into each connection's
 * list the way ble_att_create_chan does. ble_l2cap_rx then routes the CID's
 * B-frames here by scid. */

#include <furi.h>
#include <string.h>

#include "os/os_mbuf.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"

#include "ble_hs_priv.h"

#include <furi_ble/l2cap_fixed.h>
#include <ble_dispatch.h>

#include "nimble_glue.h"

#define TAG "BleFixedCid"

#define MAX_CIDS 4
#define MAX_MTU  2044

static BleL2capFixedCallback rx_callback;
static void* rx_context;
static BleL2capFixedLinkCallback link_callback;
static void* link_context;

// Guarded by the ble_hs lock
static struct {
    bool used;
    uint16_t cid;
    uint16_t mtu;
} cids[MAX_CIDS];

static struct ble_gap_event_listener gap_listener;
static bool gap_listening;

typedef struct {
    uint16_t conn;
    uint16_t cid;
    uint16_t len;
    uint8_t data[];
} RxBlob;

// Dispatch thread
static void deliver_rx(void* blob) {
    RxBlob* b = blob;
    ble_dispatch_lock();
    if(rx_callback) rx_callback(b->conn, b->cid, b->data, b->len, rx_context);
    ble_dispatch_unlock();
}

// Dispatch thread
static void deliver_link(void* blob) {
    ble_dispatch_lock();
    if(link_callback) link_callback(blob, link_context);
    ble_dispatch_unlock();
}

// Caller holds the ble_hs lock
static bool is_registered(uint16_t cid) {
    for(size_t i = 0; i < MAX_CIDS; i++) {
        if(cids[i].used && cids[i].cid == cid) return true;
    }
    return false;
}

// Host thread; ble_l2cap_rx frees the mbuf
static int on_rx(struct ble_l2cap_chan* chan, struct os_mbuf** om) {
    ble_hs_lock();
    bool registered = is_registered(chan->scid);
    ble_hs_unlock();
    if(!registered) return 0;
    uint16_t len = OS_MBUF_PKTLEN(*om);
    RxBlob* b = malloc(sizeof(RxBlob) + len);
    b->conn = chan->conn_handle;
    b->cid = chan->scid;
    b->len = len;
    os_mbuf_copydata(*om, 0, len, b->data);
    ble_dispatch_post(deliver_rx, b);
    return 0;
}

static void copy_addr(BleL2capFixedAddr* out, const ble_addr_t* in) {
    out->type = in->type;
    memcpy(out->value, in->val, sizeof(out->value));
}

// Host thread, without the ble_hs lock
static void report_link(const struct ble_gap_conn_desc* desc, bool connected, uint8_t reason) {
    BleL2capFixedLinkInfo* info = malloc(sizeof(BleL2capFixedLinkInfo));
    info->connection_handle = desc->conn_handle;
    info->connected = connected;
    info->disconnect_reason = reason;
    copy_addr(&info->peer_ota, &desc->peer_ota_addr);
    copy_addr(&info->peer_id, &desc->peer_id_addr);
    copy_addr(&info->our_ota, &desc->our_ota_addr);
    copy_addr(&info->our_id, &desc->our_id_addr);
    ble_dispatch_post(deliver_link, info);
}

// Caller holds the ble_hs lock
static bool install(struct ble_hs_conn* conn, uint16_t cid, uint16_t mtu) {
    struct ble_l2cap_chan* chan = ble_hs_conn_chan_find_by_scid(conn, cid);
    if(chan) {
        chan->my_mtu = mtu; // a re-register may change it
        return true;
    }
    chan = ble_l2cap_chan_alloc(conn->bhc_handle);
    if(!chan) {
        FURI_LOG_E(TAG, "No channel for CID 0x%04X on %u", cid, conn->bhc_handle);
        return false;
    }
    chan->scid = cid;
    chan->dcid = cid;
    chan->my_mtu = mtu;
    chan->rx_fn = on_rx;
    if(ble_hs_conn_chan_insert(conn, chan) != 0) {
        ble_l2cap_chan_free(conn, chan);
        return false;
    }
    return true;
}

// Host thread. Listeners see every link before its own callback.
static int on_gap_event(struct ble_gap_event* event, void* arg) {
    UNUSED(arg);
    struct ble_gap_conn_desc desc;
    if(event->type == BLE_GAP_EVENT_CONNECT && event->connect.status == 0) {
        ble_hs_lock();
        struct ble_hs_conn* conn = ble_hs_conn_find(event->connect.conn_handle);
        for(size_t i = 0; conn && i < MAX_CIDS; i++) {
            if(cids[i].used) install(conn, cids[i].cid, cids[i].mtu);
        }
        ble_hs_unlock();
        if(ble_gap_conn_find(event->connect.conn_handle, &desc) == 0) {
            report_link(&desc, true, 0);
        }
    } else if(event->type == BLE_GAP_EVENT_DISCONNECT) {
        // NimBLE offsets HCI reasons by BLE_HS_ERR_HCI_BASE
        int reason = event->disconnect.reason - BLE_HS_ERR_HCI_BASE;
        report_link(&event->disconnect.conn, false, reason >= 0 && reason < 0x100 ? reason : 0);
    }
    return 0;
}

void ble_l2cap_fixed_init(void) {
    ble_dispatch_init();
    nimble_glue_on_app_stop(ble_l2cap_fixed_deinit);
    // NimBLE keeps the listener in a list for the life of the host
    if(!gap_listening) {
        gap_listening = ble_gap_event_listener_register(&gap_listener, on_gap_event, NULL) == 0;
    }
}

void ble_l2cap_fixed_deinit(void) {
    ble_dispatch_lock();
    rx_callback = NULL;
    link_callback = NULL;
    ble_dispatch_unlock();
    ble_hs_lock();
    memset(cids, 0, sizeof(cids));
    ble_hs_unlock();
}

void ble_l2cap_fixed_set_callback(BleL2capFixedCallback callback, void* context) {
    ble_dispatch_lock();
    rx_callback = callback;
    rx_context = context;
    ble_dispatch_unlock();
}

void ble_l2cap_fixed_set_link_callback(BleL2capFixedLinkCallback callback, void* context) {
    ble_dispatch_lock();
    link_callback = callback;
    link_context = context;
    ble_dispatch_unlock();
}

bool ble_l2cap_fixed_register(uint16_t cid, uint16_t mtu) {
    // The standard fixed channels, and the range NimBLE assigns to CoCs
    if(cid == 0 || cid == BLE_L2CAP_CID_ATT || cid == BLE_L2CAP_CID_SIG ||
       cid == BLE_L2CAP_CID_SM ||
       (cid >= BLE_L2CAP_COC_CID_START && cid <= BLE_L2CAP_COC_CID_END)) {
        return false;
    }
    if(mtu == 0 || mtu > MAX_MTU) mtu = MAX_MTU;

    ble_hs_lock();
    int slot = -1;
    for(int i = 0; i < MAX_CIDS; i++) {
        if(cids[i].used && cids[i].cid == cid) {
            slot = i;
            break;
        }
        if(slot < 0 && !cids[i].used) slot = i;
    }
    if(slot < 0) {
        ble_hs_unlock();
        return false;
    }
    cids[slot].used = true;
    cids[slot].cid = cid;
    cids[slot].mtu = mtu;

    uint16_t links[MYNEWT_VAL(BLE_MAX_CONNECTIONS)];
    size_t link_count = 0;
    struct ble_hs_conn* conn;
    for(int i = 0; (conn = ble_hs_conn_find_by_idx(i)) != NULL; i++) {
        if(install(conn, cid, mtu) && link_count < COUNT_OF(links)) {
            links[link_count++] = conn->bhc_handle;
        }
    }
    ble_hs_unlock();

    struct ble_gap_conn_desc desc;
    for(size_t i = 0; i < link_count; i++) {
        if(ble_gap_conn_find(links[i], &desc) == 0) report_link(&desc, true, 0);
    }
    return true;
}

// A CID's channels stay until their link drops, but stop delivering
bool ble_l2cap_fixed_unregister(uint16_t cid) {
    bool found = false;
    ble_hs_lock();
    for(size_t i = 0; i < MAX_CIDS; i++) {
        if(cids[i].used && cids[i].cid == cid) {
            cids[i].used = false;
            found = true;
        }
    }
    ble_hs_unlock();
    return found;
}

typedef struct {
    uint16_t conn;
    uint16_t cid;
    uint16_t len;
    FuriSemaphore* done;
    bool* sent;
    uint8_t data[];
} SendJob;

// Host thread
static void send_on_host(void* arg) {
    SendJob* job = arg;
    ble_hs_lock();
    struct ble_hs_conn* conn = ble_hs_conn_find(job->conn);
    struct ble_l2cap_chan* chan = conn ? ble_hs_conn_chan_find_by_scid(conn, job->cid) : NULL;
    struct os_mbuf* om = chan ? ble_hs_mbuf_from_flat(job->data, job->len) : NULL;
    // ble_l2cap_tx consumes the mbuf
    int rc = om ? ble_l2cap_tx(conn, chan, om) : BLE_HS_ENOTCONN;
    ble_hs_unlock();
    *job->sent = rc == 0;
    furi_semaphore_release(job->done);
}

// Waits for the host thread, which never waits on an app thread
bool ble_l2cap_fixed_send(
    uint16_t connection_handle,
    uint16_t cid,
    const uint8_t* data,
    uint16_t data_len) {
    if(data_len && !data) return false;
    bool sent = false;
    FuriSemaphore* done = furi_semaphore_alloc(1, 0);
    SendJob* job = malloc(sizeof(SendJob) + data_len);
    job->conn = connection_handle;
    job->cid = cid;
    job->len = data_len;
    job->done = done;
    job->sent = &sent;
    if(data_len) memcpy(job->data, data, data_len);
    bool queued = nimble_glue_run_on_host(send_on_host, job, sizeof(SendJob) + data_len);
    free(job);
    if(queued) furi_semaphore_acquire(done, FuriWaitForever);
    furi_semaphore_free(done);
    return sent;
}
