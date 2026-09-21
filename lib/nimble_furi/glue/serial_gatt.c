/* The stock Serial Service on NimBLE, bridged to an RPC session. */

#include <furi.h>
#include <string.h>

#include <rpc/rpc.h>

#include "os/os_mbuf.h"
#include "nimble/ble.h"
#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "host/ble_att.h"

#include "serial_gatt.h"

#define TAG "SerialGatt"

#define SERIAL_DATA_LEN_MAX       486
#define SERIAL_CHAR_VALUE_LEN_MAX 243

/* Same UUIDs as serial_service_uuid.inc. */
static const ble_uuid128_t uuid_svc = BLE_UUID128_INIT(
    0x00,
    0x00,
    0xfe,
    0x60,
    0xcc,
    0x7a,
    0x48,
    0x2a,
    0x98,
    0x4a,
    0x7f,
    0x2e,
    0xd5,
    0xb3,
    0xe5,
    0x8f);
static const ble_uuid128_t uuid_tx = BLE_UUID128_INIT(
    0x00,
    0x00,
    0xfe,
    0x61,
    0x8e,
    0x22,
    0x45,
    0x41,
    0x9d,
    0x4c,
    0x21,
    0xed,
    0xae,
    0x82,
    0xed,
    0x19);
static const ble_uuid128_t uuid_rx = BLE_UUID128_INIT(
    0x00,
    0x00,
    0xfe,
    0x62,
    0x8e,
    0x22,
    0x45,
    0x41,
    0x9d,
    0x4c,
    0x21,
    0xed,
    0xae,
    0x82,
    0xed,
    0x19);
static const ble_uuid128_t uuid_flow = BLE_UUID128_INIT(
    0x00,
    0x00,
    0xfe,
    0x63,
    0x8e,
    0x22,
    0x45,
    0x41,
    0x9d,
    0x4c,
    0x21,
    0xed,
    0xae,
    0x82,
    0xed,
    0x19);
static const ble_uuid128_t uuid_rpc = BLE_UUID128_INIT(
    0x00,
    0x00,
    0xfe,
    0x64,
    0x8e,
    0x22,
    0x45,
    0x41,
    0x9d,
    0x4c,
    0x21,
    0xed,
    0xae,
    0x82,
    0xed,
    0x19);

static uint16_t h_tx;
static uint16_t h_rx;
static uint16_t h_flow;
static uint16_t h_rpc;

static uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static uint32_t s_rpc_status;
/* As the stock service: the peer may send RPC_BUFFER_SIZE bytes, and gets a
 * new credit once they are used up and the RPC buffer is empty. */
static uint32_t s_ready;

static Rpc* s_rpc;
static RpcSession* s_session;
static volatile bool s_disconnected;

/* TX: the RPC thread hands over one chunk at a time; the host thread sends it
 * and s_tx_ack is released when the peer confirms, or when sending fails. */
static struct ble_npl_event s_tx_event;
static struct ble_npl_event s_flow_event;
static FuriSemaphore* s_tx_ack;
static const uint8_t* s_tx_data;
static size_t s_tx_len;
static volatile size_t s_tx_sent;

static void serial_gatt_ensure_session(void);

static void post(struct ble_npl_event* ev) {
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), ev);
}

static void notify_flow_credit(void) {
    if(s_conn == BLE_HS_CONN_HANDLE_NONE) return;
    uint32_t be = __builtin_bswap32(RPC_BUFFER_SIZE);
    struct os_mbuf* om = ble_hs_mbuf_from_flat(&be, sizeof(be));
    if(om) ble_gatts_notify_custom(s_conn, h_flow, om);
}

static void flow_event_fn(struct ble_npl_event* ev) {
    UNUSED(ev);
    notify_flow_credit();
}

static void set_rpc_status(uint32_t status) {
    s_rpc_status = status;
    if(s_conn == BLE_HS_CONN_HANDLE_NONE) return;
    struct os_mbuf* om = ble_hs_mbuf_from_flat(&status, sizeof(status));
    if(om) ble_gatts_notify_custom(s_conn, h_rpc, om);
}

/* Host thread: indicate the next chunk. */
static void tx_event_fn(struct ble_npl_event* ev) {
    UNUSED(ev);
    size_t sent = s_tx_sent;
    if(s_disconnected || s_conn == BLE_HS_CONN_HANDLE_NONE) {
        furi_semaphore_release(s_tx_ack);
        return;
    }
    uint16_t mtu = ble_att_mtu(s_conn);
    size_t n = MIN(MIN((size_t)(mtu - 3), (size_t)SERIAL_CHAR_VALUE_LEN_MAX), s_tx_len - sent);
    struct os_mbuf* om = ble_hs_mbuf_from_flat(s_tx_data + sent, n);
    int rc = om ? ble_gatts_indicate_custom(s_conn, h_tx, om) : BLE_HS_ENOMEM;
    if(rc != 0) {
        FURI_LOG_E(TAG, "indicate rc %d", rc);
        furi_semaphore_release(s_tx_ack);
        return;
    }
    s_tx_sent = sent + n;
}

/* RPC thread: one indication per chunk, each waiting for its confirmation. */
static void rpc_send_cb(void* context, uint8_t* bytes, size_t bytes_len) {
    UNUSED(context);
    s_tx_data = bytes;
    s_tx_len = bytes_len;
    s_tx_sent = 0;
    while(s_tx_sent < bytes_len && !s_disconnected) {
        while(furi_semaphore_acquire(s_tx_ack, 0) == FuriStatusOk) {
        }
        /* The drain may have eaten the disconnect's release. */
        if(s_disconnected) break;
        size_t before = s_tx_sent;
        post(&s_tx_event);
        /* The confirmation, a failed send or a disconnect always releases it;
         * NimBLE drops an unresponsive link after 30 s. */
        furi_semaphore_acquire(s_tx_ack, FuriWaitForever);
        if(s_tx_sent == before) break;
    }
    s_tx_data = NULL;
}

static void rpc_buffer_is_empty_cb(void* context) {
    UNUSED(context);
    bool renew = false;
    FURI_CRITICAL_ENTER();
    if(s_ready == 0) {
        s_ready = RPC_BUFFER_SIZE;
        renew = true;
    }
    FURI_CRITICAL_EXIT();
    if(renew) post(&s_flow_event);
}

static int chr_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt* ctxt,
    void* arg) {
    UNUSED(conn_handle);
    UNUSED(arg);

    switch(ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if(attr_handle == h_rx) {
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            uint8_t buf[SERIAL_DATA_LEN_MAX];
            if(len > sizeof(buf)) len = sizeof(buf);
            if(os_mbuf_copydata(ctxt->om, 0, len, buf) != 0) return BLE_ATT_ERR_UNLIKELY;
            serial_gatt_ensure_session();
            if(s_session) {
                FURI_CRITICAL_ENTER();
                s_ready -= MIN(s_ready, len);
                FURI_CRITICAL_EXIT();
                size_t fed = rpc_session_feed(s_session, buf, len, 1000);
                if(fed != len) FURI_LOG_W(TAG, "RPC fed %zu of %u", fed, len);
            }
            return 0;
        } else if(attr_handle == h_rpc) {
            uint32_t v = 0;
            uint16_t l = OS_MBUF_PKTLEN(ctxt->om);
            os_mbuf_copydata(ctxt->om, 0, MIN(l, (uint16_t)sizeof(v)), &v);
            /* As in the stock service, 0 asks for a BLE reset. */
            if(v == 0 && s_conn != BLE_HS_CONN_HANDLE_NONE) {
                ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
            }
            return 0;
        }
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

    case BLE_GATT_ACCESS_OP_READ_CHR:
        if(attr_handle == h_flow) {
            uint32_t be = __builtin_bswap32(RPC_BUFFER_SIZE);
            int rc = os_mbuf_append(ctxt->om, &be, sizeof(be));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else if(attr_handle == h_rpc) {
            int rc = os_mbuf_append(ctxt->om, &s_rpc_status, sizeof(s_rpc_status));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else if(attr_handle == h_tx || attr_handle == h_rx) {
            return 0;
        }
        return BLE_ATT_ERR_READ_NOT_PERMITTED;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static const struct ble_gatt_svc_def svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_svc.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = &uuid_rx.u,
                    .access_cb = chr_access,
                    .val_handle = &h_rx,
                    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP |
                             BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC |
                             BLE_GATT_CHR_F_READ_AUTHEN | BLE_GATT_CHR_F_WRITE_ENC |
                             BLE_GATT_CHR_F_WRITE_AUTHEN,
                },
                {
                    .uuid = &uuid_tx.u,
                    .access_cb = chr_access,
                    .val_handle = &h_tx,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE |
                             BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ_AUTHEN,
                },
                {
                    .uuid = &uuid_flow.u,
                    .access_cb = chr_access,
                    .val_handle = &h_flow,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY |
                             BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ_AUTHEN,
                },
                {
                    .uuid = &uuid_rpc.u,
                    .access_cb = chr_access,
                    .val_handle = &h_rpc,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY |
                             BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ_AUTHEN |
                             BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_WRITE_AUTHEN,
                },
                {0},
            },
    },
    {0},
};

int serial_gatt_register(void) {
    int rc = ble_gatts_count_cfg(svcs);
    if(rc != 0) {
        FURI_LOG_E(TAG, "count_cfg failed: %d", rc);
        return rc;
    }
    rc = ble_gatts_add_svcs(svcs);
    if(rc != 0) {
        FURI_LOG_E(TAG, "add_svcs failed: %d", rc);
        return rc;
    }
    return 0;
}

void serial_gatt_init(void) {
    s_tx_ack = furi_semaphore_alloc(1, 0);
    ble_npl_event_init(&s_tx_event, tx_event_fn, NULL);
    ble_npl_event_init(&s_flow_event, flow_event_fn, NULL);
    s_rpc = furi_record_open(RECORD_RPC);
}

/* Opened when the peer first uses the service, not on every connection. */
static void serial_gatt_ensure_session(void) {
    if(s_conn == BLE_HS_CONN_HANDLE_NONE || s_session || !s_rpc) return;
    s_session = rpc_session_open(s_rpc, RpcOwnerBle);
    if(s_session) {
        rpc_session_set_context(s_session, NULL);
        rpc_session_set_send_bytes_callback(s_session, rpc_send_cb);
        rpc_session_set_buffer_is_empty_callback(s_session, rpc_buffer_is_empty_cb);
        s_ready = RPC_BUFFER_SIZE;
        set_rpc_status(1);
        notify_flow_credit();
    } else {
        FURI_LOG_E(TAG, "rpc_session_open failed");
    }
}

void serial_gatt_set_conn(uint16_t conn_handle, bool connected) {
    if(connected) {
        s_conn = conn_handle;
        s_disconnected = false;
    } else {
        s_disconnected = true;
        if(s_tx_ack) furi_semaphore_release(s_tx_ack);
        if(s_session) {
            rpc_session_close(s_session);
            s_session = NULL;
        }
        s_rpc_status = 0;
        s_conn = BLE_HS_CONN_HANDLE_NONE;
    }
}

void serial_gatt_on_subscribe(uint16_t attr_handle, bool subscribed) {
    if(subscribed && (attr_handle == h_tx || attr_handle == h_flow || attr_handle == h_rpc)) {
        serial_gatt_ensure_session();
    }
}

void serial_gatt_on_notify_tx(uint16_t attr_handle, int status) {
    /* BLE_HS_EDONE: the peer confirmed the indication. */
    if(attr_handle == h_tx && status == BLE_HS_EDONE && s_tx_ack) {
        furi_semaphore_release(s_tx_ack);
    }
}
