/*
 * Flipper Serial Service GATT server on NimBLE (TASK-589, milestones B1-B3).
 * See serial_gatt.h. B3 bridges the service to the RPC subsystem, mirroring the
 * stock applications/services/bt/bt_service/bt.c wiring:
 *   RX write (phone -> Flipper)  -> rpc_session_feed
 *   RPC tx    (Flipper -> phone) -> TX characteristic indications, one ATT-sized
 *                                   chunk at a time, waiting for each indication
 *                                   confirmation (BLE_GAP_EVENT_NOTIFY_TX).
 *   flow control                 -> notify the RPC session's available size on
 *                                   the flow-control characteristic.
 *   RPC status write 0           -> BLE reset (terminate the link).
 */

#include <furi.h>
#include <string.h>

#include <rpc/rpc.h>

#include "os/os_mbuf.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "host/ble_att.h"

#include "serial_gatt.h"

#define TAG "SerialGatt"

/* Match the stock service constants. */
#define SERIAL_DATA_LEN_MAX      486
#define SERIAL_CHAR_VALUE_LEN_MAX 243
#define TX_ACK_TIMEOUT_MS         3000

/*
 * 128-bit UUIDs copied verbatim (same little-endian byte order) from the stock
 * serial_service_uuid.inc so the on-air UUID matches and the mobile app finds
 * the service it already knows.
 */
static const ble_uuid128_t uuid_svc = BLE_UUID128_INIT(
    0x00, 0x00, 0xfe, 0x60, 0xcc, 0x7a, 0x48, 0x2a, 0x98, 0x4a, 0x7f, 0x2e, 0xd5, 0xb3, 0xe5, 0x8f);
static const ble_uuid128_t uuid_tx = BLE_UUID128_INIT(
    0x00, 0x00, 0xfe, 0x61, 0x8e, 0x22, 0x45, 0x41, 0x9d, 0x4c, 0x21, 0xed, 0xae, 0x82, 0xed, 0x19);
static const ble_uuid128_t uuid_rx = BLE_UUID128_INIT(
    0x00, 0x00, 0xfe, 0x62, 0x8e, 0x22, 0x45, 0x41, 0x9d, 0x4c, 0x21, 0xed, 0xae, 0x82, 0xed, 0x19);
static const ble_uuid128_t uuid_flow = BLE_UUID128_INIT(
    0x00, 0x00, 0xfe, 0x63, 0x8e, 0x22, 0x45, 0x41, 0x9d, 0x4c, 0x21, 0xed, 0xae, 0x82, 0xed, 0x19);
static const ble_uuid128_t uuid_rpc = BLE_UUID128_INIT(
    0x00, 0x00, 0xfe, 0x64, 0x8e, 0x22, 0x45, 0x41, 0x9d, 0x4c, 0x21, 0xed, 0xae, 0x82, 0xed, 0x19);

static uint16_t h_tx;
static uint16_t h_rx;
static uint16_t h_flow;
static uint16_t h_rpc;

static uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static uint32_t s_rx_bytes;
static uint32_t s_buff_credit = SERIAL_DATA_LEN_MAX;
static uint32_t s_rpc_status; /* 0 = inactive, 1 = active */

static Rpc* s_rpc;
static RpcSession* s_session;
static FuriSemaphore* s_tx_ack;
static volatile bool s_disconnected;

static void notify_flow_credit(void) {
    if(s_conn == BLE_HS_CONN_HANDLE_NONE) return;
    size_t avail = s_session ? rpc_session_get_available_size(s_session) : SERIAL_DATA_LEN_MAX;
    s_buff_credit = (uint32_t)avail;
    uint32_t be = __builtin_bswap32(s_buff_credit);
    struct os_mbuf* om = ble_hs_mbuf_from_flat(&be, sizeof(be));
    if(om) ble_gatts_notify_custom(s_conn, h_flow, om);
}

static void set_rpc_status(uint32_t status) {
    s_rpc_status = status;
    if(s_conn == BLE_HS_CONN_HANDLE_NONE) return;
    struct os_mbuf* om = ble_hs_mbuf_from_flat(&status, sizeof(status));
    if(om) ble_gatts_notify_custom(s_conn, h_rpc, om);
}

/* RPC thread: stream one RPC message out over TX indications, chunked to the
 * negotiated ATT payload, blocking for each indication's confirmation so we
 * never outrun the link. rpc.h forbids calling RPC APIs from here, so we don't. */
static void rpc_send_cb(void* context, uint8_t* bytes, size_t bytes_len) {
    UNUSED(context);
    if(s_disconnected || s_conn == BLE_HS_CONN_HANDLE_NONE) return;

    uint16_t mtu = ble_att_mtu(s_conn);
    size_t chunk_max = (mtu > 3) ? (size_t)(mtu - 3) : 20;
    if(chunk_max > SERIAL_CHAR_VALUE_LEN_MAX) chunk_max = SERIAL_CHAR_VALUE_LEN_MAX;

    size_t sent = 0;
    while(sent < bytes_len && !s_disconnected) {
        size_t n = MIN(chunk_max, bytes_len - sent);
        struct os_mbuf* om = ble_hs_mbuf_from_flat(bytes + sent, n);
        if(!om) {
            FURI_LOG_E(TAG, "mbuf alloc failed");
            break;
        }
        /* Drop any stale confirmation before sending this chunk. */
        while(furi_semaphore_acquire(s_tx_ack, 0) == FuriStatusOk) {
        }
        int rc = ble_gatts_indicate_custom(s_conn, h_tx, om);
        if(rc != 0) {
            FURI_LOG_E(TAG, "indicate rc %d", rc);
            break;
        }
        if(furi_semaphore_acquire(s_tx_ack, furi_ms_to_ticks(TX_ACK_TIMEOUT_MS)) != FuriStatusOk) {
            FURI_LOG_W(TAG, "TX ack timeout");
            break;
        }
        sent += n;
    }
}

static void rpc_buffer_is_empty_cb(void* context) {
    UNUSED(context);
    notify_flow_credit();
}

static int chr_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt* ctxt,
    void* arg) {
    UNUSED(conn_handle);
    UNUSED(arg);

    FURI_LOG_D(TAG, "access op=%d attr=%u", ctxt->op, attr_handle);

    switch(ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if(attr_handle == h_rx) {
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            uint8_t buf[SERIAL_DATA_LEN_MAX];
            if(len > sizeof(buf)) len = sizeof(buf);
            if(os_mbuf_copydata(ctxt->om, 0, len, buf) != 0) return BLE_ATT_ERR_UNLIKELY;
            s_rx_bytes += len;
            if(s_session) {
                size_t fed = rpc_session_feed(s_session, buf, len, 1000);
                if(fed != len) FURI_LOG_W(TAG, "RPC fed %zu of %u", fed, len);
                notify_flow_credit();
            }
            return 0;
        } else if(attr_handle == h_rpc) {
            uint32_t v = 0;
            uint16_t l = OS_MBUF_PKTLEN(ctxt->om);
            os_mbuf_copydata(ctxt->om, 0, MIN(l, (uint16_t)sizeof(v)), &v);
            FURI_LOG_I(TAG, "RPC status write %lu", v);
            if(v == 0 && s_conn != BLE_HS_CONN_HANDLE_NONE) {
                /* Stock treats a 0 write as a BLE reset request. */
                ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
            }
            return 0;
        }
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

    case BLE_GATT_ACCESS_OP_READ_CHR:
        if(attr_handle == h_flow) {
            uint32_t be = __builtin_bswap32(s_buff_credit);
            int rc = os_mbuf_append(ctxt->om, &be, sizeof(be));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else if(attr_handle == h_rpc) {
            int rc = os_mbuf_append(ctxt->om, &s_rpc_status, sizeof(s_rpc_status));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else if(attr_handle == h_tx || attr_handle == h_rx) {
            return 0; /* nothing buffered to read back */
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
                    /* RX: phone -> Flipper, encrypted+authenticated. */
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
    s_rpc = furi_record_open(RECORD_RPC);
}

void serial_gatt_deinit(void) {
    if(s_session) {
        rpc_session_close(s_session);
        s_session = NULL;
    }
    if(s_rpc) {
        furi_record_close(RECORD_RPC);
        s_rpc = NULL;
    }
    if(s_tx_ack) {
        furi_semaphore_free(s_tx_ack);
        s_tx_ack = NULL;
    }
}

void serial_gatt_set_conn(uint16_t conn_handle, bool connected) {
    if(connected) {
        s_conn = conn_handle;
        s_disconnected = false;
        s_rx_bytes = 0;
        s_buff_credit = SERIAL_DATA_LEN_MAX;
        if(s_rpc && !s_session) {
            s_session = rpc_session_open(s_rpc, RpcOwnerBle);
            if(s_session) {
                rpc_session_set_context(s_session, NULL);
                rpc_session_set_send_bytes_callback(s_session, rpc_send_cb);
                rpc_session_set_buffer_is_empty_callback(s_session, rpc_buffer_is_empty_cb);
                set_rpc_status(1);
                notify_flow_credit();
            } else {
                FURI_LOG_E(TAG, "rpc_session_open failed");
            }
        }
    } else {
        s_disconnected = true;
        if(s_tx_ack) furi_semaphore_release(s_tx_ack); /* unblock a pending TX */
        if(s_session) {
            rpc_session_close(s_session);
            s_session = NULL;
        }
        s_rpc_status = 0;
        s_conn = BLE_HS_CONN_HANDLE_NONE;
    }
}

void serial_gatt_on_notify_tx(uint16_t attr_handle, int status) {
    /* An indication is confirmed when NimBLE reports BLE_HS_EDONE for it. */
    if(attr_handle == h_tx && status == BLE_HS_EDONE && s_tx_ack) {
        furi_semaphore_release(s_tx_ack);
    }
}

uint32_t serial_gatt_rx_bytes(void) {
    return s_rx_bytes;
}
