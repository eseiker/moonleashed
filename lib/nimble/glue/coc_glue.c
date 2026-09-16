/*
 * L2CAP LE Credit-Based CoC on the resident NimBLE host (TASK-615). See
 * coc_glue.h. Contains the controller capability probe (diagnostic, unhooked)
 * and an L2CAP CoC echo server used to prove CoC works on the HCILayer radio on
 * the on-device NimBLE host (KNOW-620/622), before wiring the real DCT session
 * logic and the on-host CoC API (TASK-621).
 */

#include <furi.h>
#include <string.h>

#include "nimble/ble.h"
#include "nimble/hci_common.h"
#include "os/os_mbuf.h"
#include "host/ble_hs.h"
#include "host/ble_l2cap.h"

#include "coc_glue.h"

#define TAG "Coc"

/* Test PSM for the echo server. The real DCT channel uses a dynamic PSM
 * (KNOW-516); this fixed PSM is only for the on-device CoC proof. */
#define COC_TEST_PSM 0x0025
#define COC_MTU      512

/* ---- Controller capability probe (diagnostic, currently unhooked) --------- */

extern int ble_hs_hci_cmd_tx(
    uint16_t opcode,
    const void* cmd,
    uint8_t cmd_len,
    void* rsp,
    uint8_t rsp_len);

void coc_probe_capabilities(void) {
    struct ble_hci_le_rd_supp_states_rp states = {0};
    int rc = ble_hs_hci_cmd_tx(
        BLE_HCI_OP(BLE_HCI_OGF_LE, BLE_HCI_OCF_LE_RD_SUPP_STATES),
        NULL,
        0,
        &states,
        sizeof(states));
    if(rc == 0) {
        uint32_t hi = (uint32_t)(states.states >> 32);
        uint32_t lo = (uint32_t)(states.states & 0xFFFFFFFFu);
        FURI_LOG_I(TAG, "LE supported states: 0x%08lX%08lX", hi, lo);
    } else {
        FURI_LOG_E(TAG, "LE Read Supported States failed: %d", rc);
    }

    struct ble_hci_le_rd_buf_size_rp buf = {0};
    rc = ble_hs_hci_cmd_tx(
        BLE_HCI_OP(BLE_HCI_OGF_LE, BLE_HCI_OCF_LE_RD_BUF_SIZE),
        NULL,
        0,
        &buf,
        sizeof(buf));
    if(rc == 0) {
        FURI_LOG_I(
            TAG, "LE buffer size: pkt_len=%u num_pkts=%u", buf.data_len, buf.data_packets);
    } else {
        FURI_LOG_E(TAG, "LE Read Buffer Size failed: %d", rc);
    }
}

/* ---- L2CAP CoC echo server ------------------------------------------------ */

static struct {
    struct ble_l2cap_chan* chan;
    volatile bool connected;
    volatile bool is_client; /* true when the Flipper opened the CoC (central/DCT) */
    volatile uint16_t conn_handle;
    volatile uint32_t rx_bytes;
} coc;

/* Allocate a receive SDU buffer from the system mbuf pool. */
static struct os_mbuf* coc_sdu_alloc(void) {
    return os_msys_get_pkthdr(COC_MTU, 0);
}

/* Echo one received SDU back to the peer. Copies into a fresh tx SDU so the
 * received buffer can be freed by the caller. */
static void coc_echo(struct ble_l2cap_chan* chan, struct os_mbuf* sdu_rx) {
    uint16_t len = OS_MBUF_PKTLEN(sdu_rx);
    if(len == 0) return;
    struct os_mbuf* tx = os_msys_get_pkthdr(len, 0);
    if(!tx) {
        FURI_LOG_W(TAG, "echo: no tx mbuf");
        return;
    }
    if(os_mbuf_appendfrom(tx, sdu_rx, 0, len) != 0) {
        os_mbuf_free_chain(tx);
        return;
    }
    int rc = ble_l2cap_send(chan, tx);
    /* rc 0: sent; BLE_HS_ESTALLED: queued, stack owns the mbuf, TX_UNSTALLED
     * will follow. Any other error: we still own the mbuf and free it. */
    if(rc != 0 && rc != BLE_HS_ESTALLED) {
        FURI_LOG_W(TAG, "echo send rc=%d", rc);
        os_mbuf_free_chain(tx);
    }
}

static int coc_l2cap_event(struct ble_l2cap_event* event, void* arg) {
    UNUSED(arg);
    switch(event->type) {
    case BLE_L2CAP_EVENT_COC_ACCEPT: {
        /* A peer is opening the channel: hand the stack a receive buffer. */
        struct os_mbuf* sdu = coc_sdu_alloc();
        if(!sdu) return BLE_HS_ENOMEM;
        ble_l2cap_recv_ready(event->accept.chan, sdu);
        return 0;
    }

    case BLE_L2CAP_EVENT_COC_CONNECTED:
        coc.chan = event->connect.chan;
        coc.conn_handle = event->connect.conn_handle;
        coc.connected = (event->connect.status == 0);
        coc.rx_bytes = 0;
        FURI_LOG_I(
            TAG,
            "CoC connected status=%d handle=%u client=%d",
            event->connect.status,
            event->connect.conn_handle,
            (int)coc.is_client);
        /* When the Flipper is the client (central/DCT), send an opening payload
         * so the round trip is observable from this side. */
        if(coc.is_client && event->connect.status == 0) {
            static const char hello[] = "flipper-dct-hello";
            struct os_mbuf* tx = os_msys_get_pkthdr(sizeof(hello) - 1, 0);
            if(tx && os_mbuf_append(tx, hello, sizeof(hello) - 1) == 0) {
                int rc = ble_l2cap_send(event->connect.chan, tx);
                if(rc != 0 && rc != BLE_HS_ESTALLED) os_mbuf_free_chain(tx);
                FURI_LOG_I(TAG, "CoC client sent hello rc=%d", rc);
            } else if(tx) {
                os_mbuf_free_chain(tx);
            }
        }
        return 0;

    case BLE_L2CAP_EVENT_COC_DISCONNECTED:
        coc.connected = false;
        coc.chan = NULL;
        FURI_LOG_I(TAG, "CoC disconnected");
        return 0;

    case BLE_L2CAP_EVENT_COC_DATA_RECEIVED: {
        struct os_mbuf* sdu_rx = event->receive.sdu_rx;
        uint16_t len = OS_MBUF_PKTLEN(sdu_rx);
        coc.rx_bytes += len;
        /* Server (companion peripheral) echoes back. Client (central/DCT) just
         * consumes — echoing the server's echo would ping-pong forever. */
        if(coc.is_client) {
            FURI_LOG_I(TAG, "CoC client rx %u bytes (total %lu)", len, coc.rx_bytes);
        } else {
            coc_echo(event->receive.chan, sdu_rx);
        }
        /* Re-arm reception with a fresh buffer, then free the received one. */
        struct os_mbuf* next = coc_sdu_alloc();
        if(next) ble_l2cap_recv_ready(event->receive.chan, next);
        os_mbuf_free_chain(sdu_rx);
        return 0;
    }

    case BLE_L2CAP_EVENT_COC_TX_UNSTALLED:
        return 0;

    default:
        return 0;
    }
}

int coc_server_start(void) {
    memset(&coc, 0, sizeof(coc));
    int rc = ble_l2cap_create_server(COC_TEST_PSM, COC_MTU, coc_l2cap_event, NULL);
    FURI_LOG_I(TAG, "CoC echo server on PSM 0x%04X rc=%d", COC_TEST_PSM, rc);
    return rc;
}

int coc_client_connect(uint16_t conn_handle, uint16_t psm) {
    memset(&coc, 0, sizeof(coc));
    coc.is_client = true;
    /* ble_l2cap_connect takes ownership of sdu_rx; it is freed here only if the
     * call fails synchronously. */
    struct os_mbuf* sdu_rx = coc_sdu_alloc();
    if(!sdu_rx) return BLE_HS_ENOMEM;
    int rc = ble_l2cap_connect(conn_handle, psm, COC_MTU, sdu_rx, coc_l2cap_event, NULL);
    FURI_LOG_I(TAG, "CoC client connect handle=%u PSM 0x%04X rc=%d", conn_handle, psm, rc);
    if(rc != 0) os_mbuf_free_chain(sdu_rx);
    return rc;
}

bool coc_is_connected(void) {
    return coc.connected;
}

uint32_t coc_rx_bytes(void) {
    return coc.rx_bytes;
}

/* ---- Neutral multi-channel CoC core (TASK-621) ---------------------------- */

#define COC_API_MAX_CHAN 4

static struct {
    bool in_use;
    struct ble_l2cap_chan* chan;
    uint16_t conn_handle;
} coc_api_chans[COC_API_MAX_CHAN];

static CocApiCallback coc_api_cb;
static void* coc_api_ctx;
static uint8_t coc_api_rxbuf[2048]; /* holds a full DCT SDU (MTU 1550) */

static int coc_api_alloc_index(struct ble_l2cap_chan* chan, uint16_t conn_handle) {
    for(int i = 0; i < COC_API_MAX_CHAN; i++) {
        if(coc_api_chans[i].in_use && coc_api_chans[i].chan == chan) return i;
    }
    for(int i = 0; i < COC_API_MAX_CHAN; i++) {
        if(!coc_api_chans[i].in_use) {
            coc_api_chans[i].in_use = true;
            coc_api_chans[i].chan = chan;
            coc_api_chans[i].conn_handle = conn_handle;
            return i;
        }
    }
    return -1;
}

static int coc_api_index_of(struct ble_l2cap_chan* chan) {
    for(int i = 0; i < COC_API_MAX_CHAN; i++) {
        if(coc_api_chans[i].in_use && coc_api_chans[i].chan == chan) return i;
    }
    return -1;
}

static struct ble_l2cap_chan* coc_api_chan_at(uint8_t index) {
    if(index >= COC_API_MAX_CHAN || !coc_api_chans[index].in_use) return NULL;
    return coc_api_chans[index].chan;
}

static void coc_api_emit(const CocApiEvent* ev) {
    if(coc_api_cb) coc_api_cb(ev, coc_api_ctx);
}

static struct os_mbuf* coc_api_sdu(void) {
    return os_msys_get_pkthdr(sizeof(coc_api_rxbuf), 0);
}

static int coc_api_l2cap_event(struct ble_l2cap_event* event, void* arg) {
    UNUSED(arg);
    switch(event->type) {
    case BLE_L2CAP_EVENT_COC_ACCEPT: {
        /* Auto-accept an incoming channel by handing the stack a rx buffer. */
        struct os_mbuf* sdu = coc_api_sdu();
        if(!sdu) return BLE_HS_ENOMEM;
        ble_l2cap_recv_ready(event->accept.chan, sdu);
        return 0;
    }

    case BLE_L2CAP_EVENT_COC_CONNECTED: {
        if(event->connect.status != 0) {
            CocApiEvent ev = {
                .type = CocApiError,
                .conn_handle = event->connect.conn_handle,
                .error_code = (uint16_t)event->connect.status};
            coc_api_emit(&ev);
            return 0;
        }
        int idx = coc_api_alloc_index(event->connect.chan, event->connect.conn_handle);
        if(idx < 0) {
            FURI_LOG_E(TAG, "coc_api: no free channel slot");
            ble_l2cap_disconnect(event->connect.chan);
            return 0;
        }
        struct ble_l2cap_chan_info info;
        uint16_t peer_mtu = 0;
        if(ble_l2cap_get_chan_info(event->connect.chan, &info) == 0) peer_mtu = info.peer_l2cap_mtu;
        CocApiEvent ev = {
            .type = CocApiConnected,
            .channel_index = (uint8_t)idx,
            .conn_handle = event->connect.conn_handle,
            .peer_mtu = peer_mtu};
        coc_api_emit(&ev);
        return 0;
    }

    case BLE_L2CAP_EVENT_COC_DISCONNECTED: {
        int idx = coc_api_index_of(event->disconnect.chan);
        CocApiEvent ev = {
            .type = CocApiDisconnected,
            .channel_index = (uint8_t)(idx < 0 ? 0 : idx),
            .conn_handle = event->disconnect.conn_handle};
        coc_api_emit(&ev);
        if(idx >= 0) coc_api_chans[idx].in_use = false;
        return 0;
    }

    case BLE_L2CAP_EVENT_COC_DATA_RECEIVED: {
        struct os_mbuf* sdu_rx = event->receive.sdu_rx;
        int idx = coc_api_index_of(event->receive.chan);
        uint16_t len = OS_MBUF_PKTLEN(sdu_rx);
        if(len > sizeof(coc_api_rxbuf)) len = sizeof(coc_api_rxbuf);
        if(len && os_mbuf_copydata(sdu_rx, 0, len, coc_api_rxbuf) == 0 && idx >= 0) {
            CocApiEvent ev = {
                .type = CocApiData,
                .channel_index = (uint8_t)idx,
                .conn_handle = coc_api_chans[idx].conn_handle,
                .data = coc_api_rxbuf,
                .data_len = len};
            coc_api_emit(&ev);
        }
        /* Re-arm reception, then free the received buffer. */
        struct os_mbuf* next = coc_api_sdu();
        if(next) ble_l2cap_recv_ready(event->receive.chan, next);
        os_mbuf_free_chain(sdu_rx);
        return 0;
    }

    case BLE_L2CAP_EVENT_COC_TX_UNSTALLED: {
        int idx = coc_api_index_of(event->tx_unstalled.chan);
        if(idx >= 0) {
            CocApiEvent ev = {
                .type = CocApiTxUnstalled,
                .channel_index = (uint8_t)idx,
                .conn_handle = coc_api_chans[idx].conn_handle};
            coc_api_emit(&ev);
        }
        return 0;
    }

    default:
        return 0;
    }
}

void coc_api_init(CocApiCallback dispatch, void* context) {
    coc_api_cb = dispatch;
    coc_api_ctx = context;
    memset(coc_api_chans, 0, sizeof(coc_api_chans));
    FURI_LOG_I(TAG, "coc_api initialized");
}

void coc_api_deinit(void) {
    coc_api_cb = NULL;
    coc_api_ctx = NULL;
    memset(coc_api_chans, 0, sizeof(coc_api_chans));
}

bool coc_api_listen(uint16_t psm, uint16_t mtu) {
    int rc = ble_l2cap_create_server(psm, mtu, coc_api_l2cap_event, NULL);
    FURI_LOG_I(TAG, "coc_api listen PSM 0x%04X mtu=%u rc=%d", psm, mtu, rc);
    return rc == 0;
}

bool coc_api_connect(uint16_t conn_handle, uint16_t psm, uint16_t mtu) {
    struct os_mbuf* sdu_rx = os_msys_get_pkthdr(mtu, 0);
    if(!sdu_rx) return false;
    int rc = ble_l2cap_connect(conn_handle, psm, mtu, sdu_rx, coc_api_l2cap_event, NULL);
    FURI_LOG_I(TAG, "coc_api connect handle=%u PSM 0x%04X rc=%d", conn_handle, psm, rc);
    if(rc != 0) os_mbuf_free_chain(sdu_rx);
    return rc == 0;
}

bool coc_api_send(uint8_t channel_index, const uint8_t* data, uint16_t len) {
    struct ble_l2cap_chan* chan = coc_api_chan_at(channel_index);
    if(!chan || len == 0) return false;
    struct os_mbuf* tx = os_msys_get_pkthdr(len, 0);
    if(!tx) return false;
    if(os_mbuf_append(tx, data, len) != 0) {
        os_mbuf_free_chain(tx);
        return false;
    }
    int rc = ble_l2cap_send(chan, tx);
    if(rc != 0 && rc != BLE_HS_ESTALLED) {
        os_mbuf_free_chain(tx);
        return false;
    }
    return true;
}

bool coc_api_grant(uint8_t channel_index) {
    struct ble_l2cap_chan* chan = coc_api_chan_at(channel_index);
    if(!chan) return false;
    struct os_mbuf* sdu = coc_api_sdu();
    if(!sdu) return false;
    return ble_l2cap_recv_ready(chan, sdu) == 0;
}

bool coc_api_disconnect(uint8_t channel_index) {
    struct ble_l2cap_chan* chan = coc_api_chan_at(channel_index);
    if(!chan) return false;
    return ble_l2cap_disconnect(chan) == 0;
}
