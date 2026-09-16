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
            "CoC connected status=%d handle=%u",
            event->connect.status,
            event->connect.conn_handle);
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
        coc_echo(event->receive.chan, sdu_rx);
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

bool coc_is_connected(void) {
    return coc.connected;
}

uint32_t coc_rx_bytes(void) {
    return coc.rx_bytes;
}
