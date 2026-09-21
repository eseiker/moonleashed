/* furi_ble/l2cap_coc.h on NimBLE's L2CAP CoC. Channel work runs on the host
 * thread; events are copied into one blob each, SDU bytes included, and
 * delivered on the BLE dispatch thread. */

#include <furi.h>
#include <string.h>

#include "os/os_mbuf.h"
#include "mem/mem.h"
#include "host/ble_hs.h"
#include "host/ble_l2cap.h"

#include <furi_ble/l2cap_coc.h>
#include <ble_dispatch.h>

#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"

#include "nimble_glue.h"

#define MAX_CONNECTIONS MYNEWT_VAL(BLE_MAX_CONNECTIONS)
#define MAX_CHANNELS    MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM)
#define MAX_SERVERS     4

/* NimBLE asserts if a receive SDU cannot grow, so receive buffers come from a
 * pool of their own. Per channel it holds the SDU being filled plus one block
 * for each other receive slot NimBLE may hold; a received SDU is freed before
 * the slot is re-armed. */
#define SDU_MAX         BLE_L2CAP_COC_MTU_DEFAULT
#define SDU_SLOTS       MYNEWT_VAL(BLE_L2CAP_COC_SDU_BUFF_COUNT)
#define POOL_BLOCK      MYNEWT_VAL(MSYS_1_BLOCK_SIZE)
#define POOL_BLOCK_DATA (POOL_BLOCK - sizeof(struct os_mbuf) - sizeof(struct os_mbuf_pkthdr))
#define POOL_BLOCKS \
    (MAX_CHANNELS * ((SDU_MAX + POOL_BLOCK_DATA - 1) / POOL_BLOCK_DATA + SDU_SLOTS))

static struct {
    bool active;
    uint16_t connection_handle;
    BleL2capCocCallback callback;
    void* context;
} connections[MAX_CONNECTIONS];

// Host thread only
static struct ble_l2cap_chan* channels[MAX_CHANNELS];
static uint16_t channel_conn[MAX_CHANNELS];
static os_membuf_t* pool_mem;
static struct os_mempool pool;
static struct os_mbuf_pool mbuf_pool;

static uint16_t servers[MAX_SERVERS];
static struct ble_npl_event close_all_event;
static bool close_all_ready;

typedef struct {
    BleL2capCocEvent event;
    uint8_t payload[];
} EventBlob;

// Exact handle first, then the default slot (handle 0)
static size_t connection_find(uint16_t handle) {
    for(size_t i = 0; i < MAX_CONNECTIONS; i++) {
        if(connections[i].active && connections[i].connection_handle == handle) return i;
    }
    for(size_t i = 0; i < MAX_CONNECTIONS; i++) {
        if(connections[i].active && connections[i].connection_handle == 0) return i;
    }
    return MAX_CONNECTIONS;
}

// Dispatch thread
static void deliver(void* blob) {
    EventBlob* b = blob;
    ble_dispatch_lock();
    size_t i = connection_find(b->event.connection_handle);
    if(i < MAX_CONNECTIONS) connections[i].callback(&b->event, connections[i].context);
    ble_dispatch_unlock();
}

static EventBlob* event_alloc(BleL2capCocEventType type, uint16_t conn, size_t payload_len) {
    EventBlob* b = malloc(sizeof(EventBlob) + payload_len);
    memset(&b->event, 0, sizeof(b->event));
    b->event.type = type;
    b->event.connection_handle = conn;
    return b;
}

static void post_error(uint16_t conn, uint8_t index, int code) {
    EventBlob* b = event_alloc(BleL2capCocEventError, conn, 0);
    b->event.channel_index = index;
    b->event.error.code = code;
    ble_dispatch_post(deliver, b);
}

static int channel_index(struct ble_l2cap_chan* chan) {
    for(int i = 0; i < MAX_CHANNELS; i++) {
        if(channels[i] == chan) return i;
    }
    return -1;
}

// Allocated on first use and kept: channels may hold its blocks until they close
static struct os_mbuf* sdu_alloc(void) {
    if(!pool_mem) {
        pool_mem = malloc(OS_MEMPOOL_BYTES(POOL_BLOCKS, POOL_BLOCK));
        mem_init_mbuf_pool(pool_mem, &pool, &mbuf_pool, POOL_BLOCKS, POOL_BLOCK, "coc_rx");
    }
    return os_mbuf_get_pkthdr(&mbuf_pool, 0);
}

// The host keeps the buffer unless every receive slot is already filled
static bool rearm(struct ble_l2cap_chan* chan) {
    struct os_mbuf* sdu = sdu_alloc();
    if(!sdu) return false;
    if(ble_l2cap_recv_ready(chan, sdu) == BLE_HS_EBUSY) {
        os_mbuf_free_chain(sdu);
        return false;
    }
    return true;
}

static int on_l2cap(struct ble_l2cap_event* event, void* arg) {
    UNUSED(arg);
    EventBlob* b = NULL;
    int i;
    switch(event->type) {
    case BLE_L2CAP_EVENT_COC_ACCEPT:
        // Refused here, NimBLE answers "no resources" and frees the channel
        if(channel_index(NULL) < 0) return BLE_HS_ENOMEM;
        return rearm(event->accept.chan) ? 0 : BLE_HS_ENOMEM;
    case BLE_L2CAP_EVENT_COC_CONNECTED:
        if(event->connect.status) {
            b = event_alloc(BleL2capCocEventError, event->connect.conn_handle, 0);
            b->event.error.code = event->connect.status;
            break;
        }
        i = channel_index(NULL);
        if(i < 0) {
            ble_l2cap_disconnect(event->connect.chan);
            post_error(event->connect.conn_handle, 0, BLE_HS_ENOMEM);
            return 0;
        }
        channels[i] = event->connect.chan;
        channel_conn[i] = event->connect.conn_handle;
        b = event_alloc(BleL2capCocEventConnected, event->connect.conn_handle, 0);
        b->event.channel_index = i;
        struct ble_l2cap_chan_info info;
        if(ble_l2cap_get_chan_info(event->connect.chan, &info) == 0) {
            b->event.connected.peer_mtu = info.peer_coc_mtu;
        }
        break;
    case BLE_L2CAP_EVENT_COC_DISCONNECTED:
        i = channel_index(event->disconnect.chan);
        if(i < 0) return 0;
        channels[i] = NULL;
        b = event_alloc(BleL2capCocEventDisconnected, event->disconnect.conn_handle, 0);
        b->event.channel_index = i;
        break;
    case BLE_L2CAP_EVENT_COC_DATA_RECEIVED: {
        struct os_mbuf* sdu = event->receive.sdu_rx;
        i = channel_index(event->receive.chan);
        uint16_t len = OS_MBUF_PKTLEN(sdu);
        if(i >= 0) {
            b = event_alloc(BleL2capCocEventDataReceived, channel_conn[i], len);
            b->event.channel_index = i;
            os_mbuf_copydata(sdu, 0, len, b->payload);
            b->event.data.data = b->payload;
            b->event.data.data_len = len;
        }
        // Free first: the pool is sized for one SDU per channel
        os_mbuf_free_chain(sdu);
        rearm(event->receive.chan);
        break;
    }
    case BLE_L2CAP_EVENT_COC_TX_UNSTALLED:
        i = channel_index(event->tx_unstalled.chan);
        if(i < 0) return 0;
        if(event->tx_unstalled.status) {
            post_error(channel_conn[i], i, event->tx_unstalled.status);
            return 0;
        }
        b = event_alloc(BleL2capCocEventTxDone, channel_conn[i], 0);
        b->event.channel_index = i;
        break;
    default:
        return 0;
    }
    if(b) ble_dispatch_post(deliver, b);
    return 0;
}

typedef enum {
    RequestConnect,
    RequestSend,
    RequestGrant,
    RequestDisconnect,
} RequestOp;

typedef struct {
    RequestOp op;
    uint16_t conn;
    uint16_t psm;
    uint16_t mtu;
    uint8_t index;
    uint16_t len;
    uint8_t data[];
} Request;

// Host thread
static void run_request(void* arg) {
    Request* r = arg;
    struct ble_l2cap_chan* chan = r->index < MAX_CHANNELS ? channels[r->index] : NULL;
    int rc = 0;
    switch(r->op) {
    case RequestConnect: {
        // With no slot left the channel would be dropped on connect anyway
        if(channel_index(NULL) < 0) {
            post_error(r->conn, 0, BLE_HS_ENOMEM);
            return;
        }
        struct os_mbuf* sdu = sdu_alloc();
        rc = sdu ? ble_l2cap_connect(r->conn, r->psm, r->mtu, sdu, on_l2cap, NULL) : BLE_HS_ENOMEM;
        // Past these checks the host owns the buffer, even on failure
        if(sdu && (rc == BLE_HS_EINVAL || rc == BLE_HS_ENOTCONN)) os_mbuf_free_chain(sdu);
        if(rc != 0) post_error(r->conn, 0, rc);
        return;
    }
    case RequestSend: {
        if(!chan) break;
        struct os_mbuf* sdu = os_msys_get_pkthdr(r->len, 0);
        rc = BLE_HS_ENOMEM;
        if(sdu && os_mbuf_append(sdu, r->data, r->len) == 0) {
            rc = ble_l2cap_send(chan, sdu);
            // On ESTALLED the host keeps the SDU and TX_UNSTALLED reports TxDone
            if(rc == 0) {
                EventBlob* b = event_alloc(BleL2capCocEventTxDone, channel_conn[r->index], 0);
                b->event.channel_index = r->index;
                ble_dispatch_post(deliver, b);
            }
            if(rc == BLE_HS_ESTALLED) rc = 0;
            // Only these leave the SDU with us; on other errors the host freed it
            if(rc == BLE_HS_EBADDATA || rc == BLE_HS_EBUSY) os_mbuf_free_chain(sdu);
        } else if(sdu) {
            os_mbuf_free_chain(sdu);
        }
        break;
    }
    case RequestGrant:
        if(chan && !rearm(chan)) rc = BLE_HS_ENOMEM;
        break;
    case RequestDisconnect:
        if(chan) rc = ble_l2cap_disconnect(chan);
        break;
    }
    if(!chan) {
        post_error(0, r->index, BLE_HS_ENOTCONN);
    } else if(rc != 0) {
        post_error(channel_conn[r->index], r->index, rc);
    }
}

static bool request(RequestOp op, uint8_t index, uint16_t conn, uint16_t psm, uint16_t mtu) {
    Request r = {.op = op, .index = index, .conn = conn, .psm = psm, .mtu = mtu};
    return nimble_glue_run_on_host(run_request, &r, sizeof(r));
}

static uint16_t clamp_mtu(uint16_t mtu) {
    return mtu == 0 || mtu > SDU_MAX ? SDU_MAX : mtu;
}

// Host thread
static void close_all_fn(struct ble_npl_event* ev) {
    UNUSED(ev);
    for(size_t i = 0; i < MAX_CHANNELS; i++) {
        if(channels[i]) ble_l2cap_disconnect(channels[i]);
    }
}

void ble_l2cap_coc_init(void) {
    ble_dispatch_init();
    if(!close_all_ready) {
        ble_npl_event_init(&close_all_event, close_all_fn, NULL);
        close_all_ready = true;
    }
    nimble_glue_on_app_stop(ble_l2cap_coc_deinit);
}

void ble_l2cap_coc_deinit(void) {
    ble_dispatch_lock();
    memset(connections, 0, sizeof(connections));
    ble_dispatch_unlock();
    // Only a list update under the host lock, like create_server
    for(size_t i = 0; i < MAX_SERVERS; i++) {
        if(servers[i]) ble_l2cap_remove_server(servers[i]);
        servers[i] = 0;
    }
    // Its own event, so a full job queue cannot leave channels open
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &close_all_event);
}

void ble_l2cap_coc_set_callback(
    uint16_t connection_handle,
    BleL2capCocCallback callback,
    void* context) {
    if(callback) nimble_glue_on_app_stop(ble_l2cap_coc_deinit);
    ble_dispatch_lock();
    size_t slot = MAX_CONNECTIONS;
    for(size_t i = 0; i < MAX_CONNECTIONS; i++) {
        if(connections[i].active && connections[i].connection_handle == connection_handle) {
            slot = i;
        } else if(!connections[i].active && slot == MAX_CONNECTIONS && callback) {
            slot = i;
        }
    }
    if(slot < MAX_CONNECTIONS) {
        connections[slot].active = callback != NULL;
        connections[slot].connection_handle = connection_handle;
        connections[slot].callback = callback;
        connections[slot].context = context;
    }
    ble_dispatch_unlock();
}

bool ble_l2cap_coc_connect(
    uint16_t conn_handle,
    uint16_t spsm,
    uint16_t mtu,
    uint16_t mps,
    uint16_t initial_credits) {
    UNUSED(mps);
    UNUSED(initial_credits);
    return request(RequestConnect, 0, conn_handle, spsm, clamp_mtu(mtu));
}

bool ble_l2cap_coc_accept(
    uint16_t conn_handle,
    uint16_t mtu,
    uint16_t mps,
    uint16_t initial_credits,
    uint16_t result) {
    UNUSED(conn_handle);
    UNUSED(mtu);
    UNUSED(mps);
    UNUSED(initial_credits);
    return result == 0;
}

bool ble_l2cap_coc_send(uint8_t channel_index, const uint8_t* data, uint16_t data_len) {
    if(channel_index >= MAX_CHANNELS || !data || !data_len) return false;
    Request* r = malloc(sizeof(Request) + data_len);
    memset(r, 0, sizeof(Request));
    r->op = RequestSend;
    r->index = channel_index;
    r->len = data_len;
    memcpy(r->data, data, data_len);
    bool queued = nimble_glue_run_on_host(run_request, r, sizeof(Request) + data_len);
    free(r);
    return queued;
}

bool ble_l2cap_coc_flow_control(uint8_t channel_index, uint16_t credits) {
    UNUSED(credits);
    return channel_index < MAX_CHANNELS && request(RequestGrant, channel_index, 0, 0, 0);
}

bool ble_l2cap_coc_disconnect(uint8_t channel_index) {
    return channel_index < MAX_CHANNELS && request(RequestDisconnect, channel_index, 0, 0, 0);
}

bool ble_l2cap_coc_listen(uint16_t spsm, uint16_t mtu) {
    ble_l2cap_coc_init();
    int rc = ble_l2cap_create_server(spsm, clamp_mtu(mtu), on_l2cap, NULL);
    if(rc == 0) {
        for(size_t i = 0; i < MAX_SERVERS; i++) {
            if(!servers[i]) {
                servers[i] = spsm;
                break;
            }
        }
    }
    return rc == 0 || rc == BLE_HS_EALREADY;
}
