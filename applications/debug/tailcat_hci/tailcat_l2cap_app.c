/*
 * Tailcat host-L2CAP USB bridge (TASK-646). Unlike the raw-HCI mode, this keeps
 * the resident NimBLE host running and bridges L2CAP CoC control + SDUs over USB
 * CDC interface 1 using the on-host ble_l2cap_coc_* API (KNOW-636). A small
 * external tool speaks the TLV protocol in l2cap_frame.h; the Flipper does all
 * the BLE. Because the host stays up, the companion link can coexist with the
 * DCT CoC (KNOW-625) — which raw HCI cannot.
 */

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_usb_cdc.h>
#include <gui/gui.h>
#include <cli/cli_vcp.h>

#include <furi_ble/l2cap_coc.h>
#include <furi_ble/l2cap_fixed.h>
#include <furi_ble/security.h>
#include <furi_ble/gatt_server.h>
#include <furi_ble/link.h>
#include <furi_ble/gatt_client.h>
#include <furi_ble/adv.h>
#include <furi_ble/furi_ble_session.h>
#include "l2cap_frame.h"

#define TAG "TailcatL2cap"

#define L2_DEFAULT_RUNTIME_SECONDS 300UL
/* Room for two largest frames ([type:1][len:2] + L2F_PAYLOAD_MAX), since l2_emit
 * queues a frame whole or not at all. */
#define L2_TX_STREAM_SIZE          (2U * (3U + L2F_PAYLOAD_MAX))
/* Characteristics the host may define through L2F_GATT_CHAR. */
#define L2_GATT_MAX_CHRS           32U

typedef struct {
    FuriThread* worker;
    FuriSemaphore* usb_tx;
    FuriMessageQueue* keys;
    FuriStreamBuffer* tx; /* FAP -> host bytes, framed */
    FuriMutex* tx_lock; /* l2_emit runs on the dispatch and USB worker threads */
    volatile bool stop;
    volatile bool failed;
    volatile bool connected;
    volatile uint32_t rx_sdus; /* SDUs BLE -> USB */
    volatile uint32_t tx_sdus; /* SDUs USB -> BLE */
    /* L2F_CONNECT: modal central session (companion suspended while it runs). */
    FuriBleSession* central;
    uint16_t central_psm;
    volatile bool central_coc_opened; /* a CoC opened on the central link */
    /* Characteristics defined over L2F_GATT_CHAR, so a commit can report every
     * assigned handle back to the host. */
    int32_t chr_ids[L2_GATT_MAX_CHRS];
    uint8_t chr_count;
    /* True once the host sent a SEC_* frame and this bridge took pairing over.
     * Until then the firmware keeps pairing the companion's way. */
    bool sec_claimed;
} L2App;

/* Queue one framed message for the USB worker to send. The BLE dispatch thread
 * and the USB worker both emit, and a stream buffer allows one writer, so the
 * header and payload go in under a lock. A frame that does not fit is dropped
 * whole: a header without its payload would desynchronize the host. */
static void l2_emit(L2App* app, uint8_t type, const uint8_t* payload, uint16_t len) {
    uint8_t hdr[3] = {type, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    if(!payload) len = 0;
    furi_mutex_acquire(app->tx_lock, FuriWaitForever);
    if(furi_stream_buffer_spaces_available(app->tx) >= sizeof(hdr) + len) {
        furi_stream_buffer_send(app->tx, hdr, sizeof(hdr), 0);
        if(len) furi_stream_buffer_send(app->tx, payload, len, 0);
    } else {
        FURI_LOG_W(TAG, "tx full, frame 0x%02X (%u B) dropped", type, len);
    }
    furi_mutex_release(app->tx_lock);
}

/* CoC events arrive on the NimBLE host thread; frame them into the tx stream. */
static void on_coc(BleL2capCocEvent* ev, void* context) {
    L2App* app = context;
    switch(ev->type) {
    case BleL2capCocEventConnected: {
        uint8_t p[5] = {
            ev->channel_index,
            (uint8_t)(ev->connection_handle & 0xFF),
            (uint8_t)(ev->connection_handle >> 8),
            (uint8_t)(ev->connected.peer_mtu & 0xFF),
            (uint8_t)(ev->connected.peer_mtu >> 8)};
        app->connected = true;
        if(app->central) app->central_coc_opened = true;
        l2_emit(app, L2F_CONNECTED, p, sizeof(p));
    } break;
    case BleL2capCocEventDataReceived: {
        static uint8_t buf[L2F_PAYLOAD_MAX];
        uint16_t n = ev->data.data_len;
        /* The CoC was opened with MTU L2F_COC_MTU and NimBLE rejects larger
         * SDUs, so n always fits. Guard memory anyway, but never forward a
         * shortened SDU: drop it rather than truncate. */
        if(n > L2F_COC_MTU) {
            FURI_LOG_E(
                TAG, "SDU %u above MTU %u on ch %u dropped", n, L2F_COC_MTU, ev->channel_index);
            break;
        }
        buf[0] = ev->channel_index;
        memcpy(buf + 1, ev->data.data, n);
        app->rx_sdus++;
        l2_emit(app, L2F_DATA, buf, n + 1);
    } break;
    case BleL2capCocEventDisconnected: {
        uint8_t p[1] = {ev->channel_index};
        app->connected = false;
        l2_emit(app, L2F_DISCONNECTED, p, sizeof(p));
    } break;
    case BleL2capCocEventError: {
        uint8_t p[2] = {(uint8_t)(ev->error.code & 0xFF), (uint8_t)(ev->error.code >> 8)};
        l2_emit(app, L2F_ERROR, p, sizeof(p));
    } break;
    default:
        break;
    }
}

/* Inbound fixed-CID PDU (dispatch thread). Frame it as [conn:2][cid:2][pdu...]. */
static void
    on_fixed(uint16_t conn, uint16_t cid, const uint8_t* data, uint16_t len, void* context) {
    L2App* app = context;
    static uint8_t buf[4 + L2F_PAYLOAD_MAX];
    if(len > L2F_PAYLOAD_MAX - 4) len = L2F_PAYLOAD_MAX - 4;
    buf[0] = (uint8_t)conn;
    buf[1] = (uint8_t)(conn >> 8);
    buf[2] = (uint8_t)cid;
    buf[3] = (uint8_t)(cid >> 8);
    memcpy(buf + 4, data, len);
    app->rx_sdus++;
    l2_emit(app, L2F_FIXED_DATA, buf, len + 4);
}

/* A link came up or went down (dispatch thread), with the addresses and the
 * disconnect reason the enrollment stage machine needs (TASK-713). Sent before
 * the plain FIXED_LINK below, which a host may be the only one reading. */
static uint8_t* fixed_put_addr(uint8_t* p, const BleL2capFixedAddr* addr) {
    *p++ = addr->type;
    memcpy(p, addr->value, sizeof(addr->value));
    return p + sizeof(addr->value);
}

static void on_fixed_link_info(const BleL2capFixedLinkInfo* info, void* context) {
    L2App* app = context;
    uint8_t p[32];
    p[0] = (uint8_t)info->connection_handle;
    p[1] = (uint8_t)(info->connection_handle >> 8);
    p[2] = info->connected ? 1 : 0;
    p[3] = info->disconnect_reason;
    uint8_t* w = p + 4;
    w = fixed_put_addr(w, &info->peer_ota);
    w = fixed_put_addr(w, &info->peer_id);
    w = fixed_put_addr(w, &info->our_ota);
    w = fixed_put_addr(w, &info->our_id);
    l2_emit(app, L2F_FIXED_LINK_INFO, p, (uint16_t)(w - p));
}

/* A link came up or went down (dispatch thread). Frame it as [conn:2][up:1]. */
static void on_fixed_link(uint16_t conn, bool connected, void* context) {
    L2App* app = context;
    uint8_t p[3] = {(uint8_t)conn, (uint8_t)(conn >> 8), connected ? 1 : 0};
    l2_emit(app, L2F_FIXED_LINK, p, sizeof(p));
}

/* Pairing events (dispatch thread): frame them for the host, which answers. */
static void on_security(const BleSecurityEvent* ev, void* context) {
    L2App* app = context;
    uint8_t kind;
    switch(ev->type) {
    case BleSecurityEventTypePasskeyDisplay:
        kind = 0;
        break;
    case BleSecurityEventTypePasskeyRequest:
        kind = 1;
        break;
    case BleSecurityEventTypeNumericComparison:
        kind = 2;
        break;
    case BleSecurityEventTypeOobRequest:
        kind = 3;
        break;
    case BleSecurityEventTypeEncryptionChanged:
        kind = 4;
        break;
    default:
        kind = 5;
        break;
    }
    uint8_t flags = (uint8_t)((ev->encrypted ? 0x01 : 0) | (ev->authenticated ? 0x02 : 0) |
                              (ev->bonded ? 0x04 : 0));
    uint8_t p[11] = {
        kind,
        (uint8_t)(ev->connection_handle & 0xFF),
        (uint8_t)(ev->connection_handle >> 8),
        (uint8_t)((uint16_t)ev->status & 0xFF),
        (uint8_t)((uint16_t)ev->status >> 8),
        (uint8_t)(ev->passkey & 0xFF),
        (uint8_t)((ev->passkey >> 8) & 0xFF),
        (uint8_t)((ev->passkey >> 16) & 0xFF),
        (uint8_t)((ev->passkey >> 24) & 0xFF),
        flags,
        ev->key_size};
    l2_emit(app, L2F_SEC_EVENT, p, sizeof(p));
}

/* Report one characteristic's assigned handles. */
static void l2_emit_char_id(L2App* app, int32_t chr_id) {
    uint16_t decl = 0;
    uint16_t value = 0;
    ble_gatt_server_char_handles(chr_id, &decl, &value);
    uint8_t p[6] = {
        1,
        (uint8_t)chr_id,
        (uint8_t)(decl & 0xFF),
        (uint8_t)(decl >> 8),
        (uint8_t)(value & 0xFF),
        (uint8_t)(value >> 8)};
    l2_emit(app, L2F_GATT_ID, p, sizeof(p));
}

/* GATT server events (dispatch thread). */
static void on_gatt_server(const BleGattServerEvent* ev, void* context) {
    L2App* app = context;
    switch(ev->type) {
    case BleGattServerEventTypeWrite: {
        static uint8_t buf[4 + L2F_PAYLOAD_MAX];
        uint16_t n = ev->data_len;
        if(n > L2F_PAYLOAD_MAX - 4) n = L2F_PAYLOAD_MAX - 4;
        buf[0] = (uint8_t)ev->char_id;
        buf[1] = (uint8_t)(ev->connection_handle & 0xFF);
        buf[2] = (uint8_t)(ev->connection_handle >> 8);
        buf[3] = 0; /* value write */
        if(n) memcpy(buf + 4, ev->data, n);
        l2_emit(app, L2F_GATT_WRITE, buf, n + 4);
    } break;
    case BleGattServerEventTypeSubscribe: {
        uint8_t p[6] = {
            (uint8_t)ev->char_id,
            (uint8_t)(ev->connection_handle & 0xFF),
            (uint8_t)(ev->connection_handle >> 8),
            1, /* CCCD */
            (uint8_t)(ev->notify ? 1 : 0),
            (uint8_t)(ev->indicate ? 1 : 0)};
        l2_emit(app, L2F_GATT_WRITE, p, sizeof(p));
    } break;
    case BleGattServerEventTypeCommitted:
        for(uint8_t i = 0; i < app->chr_count; i++)
            l2_emit_char_id(app, app->chr_ids[i]);
        l2_emit(app, L2F_GATT_READY, NULL, 0);
        break;
    default:
        break;
    }
}

/* Read a UUID laid out as [type:1][uuid:2 or 16]. Returns its length, or 0. */
static uint16_t l2_read_uuid(const uint8_t* p, uint16_t avail, BleGattServerUuid* out) {
    if(avail < 1) return 0;
    memset(out, 0, sizeof(*out));
    out->type = p[0];
    if(p[0] == 16) {
        if(avail < 3) return 0;
        out->uuid16 = (uint16_t)p[1] | ((uint16_t)p[2] << 8);
        return 3;
    }
    if(p[0] == 128) {
        if(avail < 17) return 0;
        memcpy(out->uuid128, p + 1, 16);
        return 17;
    }
    return 0;
}

/* Append a GATT client UUID as [uuid_type:1][uuid:2 or 16]. Returns the length
 * written. The type follows the ble_gatt_client_* API: 1 is 16-bit, 2 is
 * 128-bit. */
static uint16_t
    l2_put_gattc_uuid(uint8_t* p, uint8_t uuid_type, uint16_t uuid16, const uint8_t* uuid128) {
    p[0] = uuid_type;
    if(uuid_type == 1) {
        p[1] = (uint8_t)(uuid16 & 0xFF);
        p[2] = (uint8_t)(uuid16 >> 8);
        return 3;
    }
    memcpy(p + 1, uuid128, 16);
    return 17;
}

/* GATT client results (dispatch thread). */
static void on_gattc(BleGattClientEvent* ev, void* context) {
    L2App* app = context;
    static uint8_t buf[8 + L2F_PAYLOAD_MAX];
    uint16_t conn = ev->connection_handle;
    buf[0] = (uint8_t)(conn & 0xFF);
    buf[1] = (uint8_t)(conn >> 8);

    switch(ev->type) {
    case BleGattClientEventDiscoverComplete:
        for(uint8_t i = 0; i < ev->discover.count; i++) {
            const BleGattService* s = &ev->discover.services[i];
            uint16_t n = 2;
            n += l2_put_gattc_uuid(buf + n, s->uuid_type, s->uuid_16, s->uuid_128);
            buf[n++] = (uint8_t)(s->start_handle & 0xFF);
            buf[n++] = (uint8_t)(s->start_handle >> 8);
            buf[n++] = (uint8_t)(s->end_handle & 0xFF);
            buf[n++] = (uint8_t)(s->end_handle >> 8);
            l2_emit(app, L2F_GATTC_SERVICE, buf, n);
        }
        buf[2] = 0; /* services */
        l2_emit(app, L2F_GATTC_DONE, buf, 3);
        break;
    case BleGattClientEventCharDiscoverComplete:
        for(uint8_t i = 0; i < ev->char_discover.count; i++) {
            const BleGattCharacteristic* c = &ev->char_discover.chars[i];
            uint16_t n = 2;
            n += l2_put_gattc_uuid(buf + n, c->uuid_type, c->uuid_16, c->uuid_128);
            buf[n++] = (uint8_t)(c->decl_handle & 0xFF);
            buf[n++] = (uint8_t)(c->decl_handle >> 8);
            buf[n++] = (uint8_t)(c->value_handle & 0xFF);
            buf[n++] = (uint8_t)(c->value_handle >> 8);
            buf[n++] = c->properties;
            l2_emit(app, L2F_GATTC_CHAR, buf, n);
        }
        buf[2] = 1; /* characteristics */
        l2_emit(app, L2F_GATTC_DONE, buf, 3);
        break;
    case BleGattClientEventReadComplete: {
        uint16_t n = ev->read.data_len;
        if(n > L2F_PAYLOAD_MAX - 4) n = L2F_PAYLOAD_MAX - 4;
        buf[2] = (uint8_t)(ev->read.value_handle & 0xFF);
        buf[3] = (uint8_t)(ev->read.value_handle >> 8);
        if(n) memcpy(buf + 4, ev->read.data, n);
        l2_emit(app, L2F_GATTC_READ_DATA, buf, n + 4);
    } break;
    case BleGattClientEventNotification: {
        uint16_t n = ev->notification.data_len;
        if(n > L2F_PAYLOAD_MAX - 4) n = L2F_PAYLOAD_MAX - 4;
        buf[2] = (uint8_t)(ev->notification.value_handle & 0xFF);
        buf[3] = (uint8_t)(ev->notification.value_handle >> 8);
        if(n) memcpy(buf + 4, ev->notification.data, n);
        app->rx_sdus++;
        l2_emit(app, L2F_GATTC_NOTIFY, buf, n + 4);
    } break;
    case BleGattClientEventWriteComplete:
        buf[2] = 0;
        buf[3] = 0;
        l2_emit(app, L2F_GATTC_WRITE_DONE, buf, 4);
        break;
    case BleGattClientEventError:
    default:
        buf[2] = ev->error.error_code;
        l2_emit(app, L2F_GATTC_ERROR, buf, 3);
        break;
    }
}

/* Take pairing over on the first SEC_* frame, not at startup: a bridge session
 * that never pairs must leave the companion's pairing alone. */
static void l2_security_claim(L2App* app) {
    if(app->sec_claimed) return;
    ble_security_init();
    ble_security_set_callback(on_security, app);
    app->sec_claimed = true;
}

static void l2_error(L2App* app, uint16_t code) {
    uint8_t p[2] = {(uint8_t)(code & 0xFF), (uint8_t)(code >> 8)};
    l2_emit(app, L2F_ERROR, p, sizeof(p));
}

static void l2_handle_frame(L2App* app, const L2Frame* f) {
    switch(f->type) {
    case L2F_LISTEN:
        if(f->len >= 2) {
            uint16_t psm = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            ble_l2cap_coc_listen(psm, L2F_COC_MTU);
        }
        break;
    case L2F_SEND:
        if(f->len >= 1) {
            uint8_t ch = f->data[0];
            if(ble_l2cap_coc_send(ch, f->data + 1, f->len - 1)) {
                app->tx_sdus++;
            } else {
                /* Unknown channel, SDU above the negotiated MTU, or no mbufs:
                 * tell the host instead of failing silently. */
                uint8_t code[2] = {0x05, 0x00};
                l2_emit(app, L2F_ERROR, code, sizeof(code));
            }
        }
        break;
    case L2F_FIXED_REGISTER:
        /* [cid:2][mtu:2]: relay a fixed L2CAP CID (e.g. 0x003A). */
        if(f->len >= 4) {
            uint16_t cid = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            uint16_t mtu = (uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8);
            if(!ble_l2cap_fixed_register(cid, mtu)) {
                uint8_t code[2] = {0x06, 0x00}; /* fixed-CID register refused */
                l2_emit(app, L2F_ERROR, code, sizeof(code));
            }
        }
        break;
    case L2F_FIXED_SEND:
        /* [conn:2][cid:2][pdu...]: transmit a raw PDU on a fixed CID. */
        if(f->len >= 4) {
            uint16_t conn = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            uint16_t cid = (uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8);
            if(ble_l2cap_fixed_send(conn, cid, f->data + 4, f->len - 4)) {
                app->tx_sdus++;
            } else {
                uint8_t code[2] = {0x07, 0x00}; /* fixed-CID send failed */
                l2_emit(app, L2F_ERROR, code, sizeof(code));
            }
        }
        break;
    case L2F_FIXED_UNREG:
        if(f->len >= 2)
            ble_l2cap_fixed_unregister((uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8));
        break;
    case L2F_SEC_CONFIG:
        /* [io_cap:1][flags:1][our_kd:1][their_kd:1] */
        l2_security_claim(app);
        if(f->len >= 4) {
            uint8_t flags = f->data[1];
            if(!ble_security_configure(
                   (BleSecurityIoCapability)f->data[0],
                   (flags & 0x01) != 0,
                   (flags & 0x02) != 0,
                   (flags & 0x04) != 0,
                   f->data[2],
                   f->data[3]))
                l2_error(app, 0x0008);
        }
        break;
    case L2F_SEC_PAIR:
        l2_security_claim(app);
        if(f->len >= 2) {
            uint16_t conn = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            if(!ble_security_pair(conn)) l2_error(app, 0x0008);
        }
        break;
    case L2F_SEC_PASSKEY:
        /* [conn:2][passkey:4] */
        if(f->len >= 6) {
            uint16_t conn = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            uint32_t passkey = (uint32_t)f->data[2] | ((uint32_t)f->data[3] << 8) |
                               ((uint32_t)f->data[4] << 16) | ((uint32_t)f->data[5] << 24);
            if(!ble_security_passkey_reply(conn, passkey)) l2_error(app, 0x0008);
        }
        break;
    case L2F_SEC_CONFIRM:
        if(f->len >= 3) {
            uint16_t conn = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            if(!ble_security_numeric_comparison_reply(conn, f->data[2] != 0))
                l2_error(app, 0x0008);
        }
        break;
    case L2F_SEC_OOB_GENERATE: {
        /* Answer with our OOB values; an empty payload reports a failure. */
        l2_security_claim(app);
        uint8_t p[2 * BLE_SECURITY_OOB_LEN];
        if(ble_security_oob_generate(p, p + BLE_SECURITY_OOB_LEN)) {
            l2_emit(app, L2F_SEC_OOB_LOCAL, p, sizeof(p));
        } else {
            l2_emit(app, L2F_SEC_OOB_LOCAL, NULL, 0);
            l2_error(app, 0x0008);
        }
    } break;
    case L2F_SEC_OOB_PEER:
        /* [random:16][confirm:16] */
        l2_security_claim(app);
        if(f->len >= 2 * BLE_SECURITY_OOB_LEN) {
            if(!ble_security_oob_set_peer(f->data, f->data + BLE_SECURITY_OOB_LEN))
                l2_error(app, 0x0008);
        }
        break;
    case L2F_SEC_OOB_CLEAR:
        ble_security_oob_clear();
        break;
    case L2F_GATT_SERVICE: {
        /* [primary:1][uuid_type:1][uuid...] */
        BleGattServerUuid uuid;
        if(f->len < 2 || !l2_read_uuid(f->data + 1, f->len - 1, &uuid)) {
            l2_error(app, 0x0009);
            break;
        }
        int32_t svc = ble_gatt_server_service_add(&uuid, f->data[0] != 0);
        if(svc < 0) {
            l2_error(app, 0x0009);
            break;
        }
        uint8_t p[6] = {0, (uint8_t)svc, 0, 0, 0, 0};
        l2_emit(app, L2F_GATT_ID, p, sizeof(p));
    } break;
    case L2F_GATT_CHAR: {
        /* [svc_id:1][flags:2][max_len:2][uuid_type:1][uuid...][init...] */
        BleGattServerUuid uuid;
        uint16_t used = 5;
        uint16_t ulen = f->len > used ? l2_read_uuid(f->data + used, f->len - used, &uuid) : 0;
        if(f->len < 6 || !ulen || app->chr_count >= L2_GATT_MAX_CHRS) {
            l2_error(app, 0x0009);
            break;
        }
        used += ulen;
        uint16_t flags = (uint16_t)f->data[1] | ((uint16_t)f->data[2] << 8);
        uint16_t max_len = (uint16_t)f->data[3] | ((uint16_t)f->data[4] << 8);
        int32_t chr = ble_gatt_server_char_add(
            f->data[0], &uuid, flags, max_len, f->data + used, f->len - used);
        if(chr < 0) {
            l2_error(app, 0x0009);
            break;
        }
        app->chr_ids[app->chr_count++] = chr;
        l2_emit_char_id(app, chr);
    } break;
    case L2F_GATTC_DISCOVER:
        if(f->len >= 2) {
            uint16_t conn = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            if(!ble_gatt_client_discover_services(conn)) l2_error(app, 0x000A);
        }
        break;
    case L2F_GATTC_CHARS:
        /* [conn:2][start:2][end:2] */
        if(f->len >= 6) {
            uint16_t conn = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            BleGattService svc = {
                .start_handle = (uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8),
                .end_handle = (uint16_t)f->data[4] | ((uint16_t)f->data[5] << 8),
            };
            if(!ble_gatt_client_discover_characteristics(conn, &svc)) l2_error(app, 0x000A);
        }
        break;
    case L2F_GATTC_READ:
        if(f->len >= 4) {
            uint16_t conn = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            uint16_t handle = (uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8);
            if(!ble_gatt_client_read(conn, handle)) l2_error(app, 0x000A);
        }
        break;
    case L2F_GATTC_WRITE:
        /* [conn:2][value_handle:2][data...] */
        if(f->len >= 4) {
            uint16_t conn = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            uint16_t handle = (uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8);
            if(ble_gatt_client_write(conn, handle, f->data + 4, f->len - 4)) {
                app->tx_sdus++;
            } else {
                l2_error(app, 0x000A);
            }
        }
        break;
    case L2F_GATTC_SUBSCRIBE:
        if(f->len >= 5) {
            uint16_t conn = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            uint16_t handle = (uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8);
            if(!ble_gatt_client_subscribe_notifications(conn, handle, f->data[4] != 0))
                l2_error(app, 0x000A);
        }
        break;
    case L2F_GATTC_MTU:
        if(f->len >= 2) {
            uint16_t conn = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            if(!ble_gatt_client_exchange_mtu(conn)) l2_error(app, 0x000A);
        }
        break;
    case L2F_LINK_DISCONNECT:
        /* [conn:2][reason:1]: drop a link the host no longer wants. */
        if(f->len >= 3) {
            uint16_t conn = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
            BleLinkDisconnectStatus st = ble_link_disconnect(conn, f->data[2]);
            uint8_t p[3] = {(uint8_t)conn, (uint8_t)(conn >> 8), (uint8_t)st};
            l2_emit(app, L2F_LINK_STATUS, p, sizeof(p));
        }
        break;
    case L2F_GATT_COMMIT:
        if(!ble_gatt_server_commit()) l2_error(app, 0x0009);
        break;
    case L2F_GATT_SET:
        if(f->len >= 1) {
            if(!ble_gatt_server_char_set_value(f->data[0], f->data + 1, f->len - 1))
                l2_error(app, 0x0009);
        }
        break;
    case L2F_GATT_RESET:
        /* Drop everything this bridge defined and rebuild without it. */
        ble_gatt_server_deinit();
        app->chr_count = 0;
        ble_gatt_server_init();
        ble_gatt_server_set_callback(on_gatt_server, app);
        break;
    case L2F_CLOSE:
        if(f->len >= 1) ble_l2cap_coc_disconnect(f->data[0]);
        /* On a central session, closing the channel ends the session too: the
         * link drops and the companion is restored. */
        if(app->central) {
            furi_ble_session_free(app->central);
            app->central = NULL;
            app->central_coc_opened = false;
        }
        break;
    case L2F_ADVERTISE:
    case L2F_ADVERTISE_ONCE:
        /* [adv_len:1][adv...][rsp...]: install the caller's advertisement (e.g.
         * the DCT FC73 session adv) on the resident host so peers discover us.
         * ADVERTISE_ONCE stops it when the first peer connects (TASK-721). */
        if(f->len >= 1 && (uint16_t)(1 + f->data[0]) <= f->len) {
            uint8_t adv_len = f->data[0];
            const uint8_t* adv = f->data + 1;
            const uint8_t* rsp = f->data + 1 + adv_len;
            uint16_t rsp_len = f->len - 1 - adv_len;
            bool once = f->type == L2F_ADVERTISE_ONCE;
            if(rsp_len > 31 || !furi_ble_adv_set_ex(adv, adv_len, rsp, (uint8_t)rsp_len, once)) {
                uint8_t code[2] = {0x01, 0x00};
                l2_emit(app, L2F_ERROR, code, sizeof(code));
            }
        }
        break;
    case L2F_ADV_RESTART:
        /* []: advertise the installed payload again, which is how the stage
         * machine re-arms between stages after ADVERTISE_ONCE stopped it. */
        furi_ble_adv_restart();
        break;
    case L2F_ADV_STOP:
        /* []: drop the raw payload; the companion advertisement comes back. */
        furi_ble_adv_clear();
        break;
    case L2F_CONNECT: {
        /* [psm:2][name...]: suspend the companion, scan for `name`, connect as
         * central, then open a CoC client on psm once the link is up (the
         * session's Connected event is handled in l2_poll_central). */
        if(f->len < 3) break;
        if(app->central) {
            uint8_t code[2] = {0x02, 0x00}; /* a central session is already active */
            l2_emit(app, L2F_ERROR, code, sizeof(code));
            break;
        }
        char name[24];
        uint16_t nlen = f->len - 2;
        if(nlen > sizeof(name) - 1) nlen = sizeof(name) - 1;
        memcpy(name, f->data + 2, nlen);
        name[nlen] = '\0';
        app->central_psm = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
        app->central_coc_opened = false;
        FuriBleSessionConfig cfg = {.role = FuriBleRoleCentralModal, .central_name = name};
        app->central = furi_ble_session_alloc(&cfg);
        if(!app->central) {
            uint8_t code[2] = {0x02, 0x00}; /* refused: host not ready or busy */
            l2_emit(app, L2F_ERROR, code, sizeof(code));
        }
    } break;
    default:
        break;
    }
}

/* Drain the central session's off-thread event queue (worker thread). */
static void l2_poll_central(L2App* app) {
    if(!app->central) return;
    FuriBleEvent ev;
    if(!furi_ble_session_get_event(app->central, &ev, 0)) return;
    switch(ev.type) {
    case FuriBleEventCentralConnected:
        /* A psm of 0 means the host only wanted the link, to drive GATT over it
         * with the GATTC_* frames (TASK-633). FIXED_LINK already reported the
         * connection handle, so there is nothing more to do here. */
        if(app->central_psm == 0) {
            app->central_coc_opened = true; /* no CoC to lose, so no link-lost error */
            break;
        }
        /* Link is up: open the CoC as the client. Its Connected/Data events
         * then flow through on_coc like the server path. */
        if(!ble_l2cap_coc_connect(
               ev.conn_handle,
               app->central_psm,
               L2F_COC_MTU,
               BLE_L2CAP_COC_MPS_MAX,
               BLE_L2CAP_COC_CREDITS_DEFAULT)) {
            uint8_t code[2] = {0x03, 0x00}; /* CoC client open failed */
            l2_emit(app, L2F_ERROR, code, sizeof(code));
            furi_ble_session_free(app->central);
            app->central = NULL;
        }
        break;
    case FuriBleEventCentralFailed: {
        uint8_t code[2] = {0x02, 0x00}; /* peer not found / connect failed */
        l2_emit(app, L2F_ERROR, code, sizeof(code));
        furi_ble_session_free(app->central);
        app->central = NULL;
    } break;
    case FuriBleEventCentralDisconnected:
        /* A CoC that opened already reported DISCONNECTED via on_coc; if the
         * link went before any CoC opened, tell the host. */
        if(!app->central_coc_opened) {
            uint8_t code[2] = {0x04, 0x00}; /* link lost before the CoC opened */
            l2_emit(app, L2F_ERROR, code, sizeof(code));
        }
        furi_ble_session_free(app->central);
        app->central = NULL;
        app->central_coc_opened = false;
        break;
    default:
        break;
    }
}

static void usb_tx_done(void* context) {
    L2App* app = context;
    furi_semaphore_release(app->usb_tx);
}

static void usb_rx_ready(void* context) {
    L2App* app = context;
    furi_thread_flags_set(furi_thread_get_id(app->worker), 1);
}

static void usb_state(void* context, CdcState state) {
    UNUSED(context);
    UNUSED(state);
}

static CdcCallbacks usb_callbacks = {
    .tx_ep_callback = usb_tx_done,
    .rx_ep_callback = usb_rx_ready,
    .state_callback = usb_state,
};

static int32_t l2_worker(void* context) {
    L2App* app = context;
    uint8_t usb_bytes[CDC_DATA_SZ];
    uint8_t out[64];
    /* L2F_PAYLOAD_MAX is 2 KiB, so keep the reassembly frame off the 2 KiB
     * worker stack. One worker, so a static is safe. */
    static L2Frame frame;
    l2f_reset(&frame);

    while(!app->stop && !app->failed) {
        /* USB -> BLE */
        int32_t count = furi_hal_cdc_receive(1, usb_bytes, sizeof(usb_bytes));
        for(int32_t i = 0; i < count; i++) {
            int r = l2f_push(&frame, usb_bytes[i]);
            if(r < 0) {
                l2f_reset(&frame);
            } else if(r == 1) {
                l2_handle_frame(app, &frame);
                l2f_reset(&frame);
            }
        }
        /* Central session lifecycle (L2F_CONNECT). */
        l2_poll_central(app);
        /* BLE -> USB: drain the framed tx stream in <=63-byte chunks. */
        while(furi_stream_buffer_bytes_available(app->tx) && !app->failed) {
            size_t chunk = furi_stream_buffer_receive(app->tx, out, 63, 0);
            if(!chunk) break;
            if(furi_semaphore_acquire(app->usb_tx, 1000) != FuriStatusOk) {
                app->failed = true;
                break;
            }
            furi_hal_cdc_send(1, out, chunk);
        }
        if(!count && !furi_stream_buffer_bytes_available(app->tx))
            furi_thread_flags_wait(1, FuriFlagWaitAny, 5);
    }
    return 0;
}

static void draw(Canvas* canvas, void* context) {
    L2App* app = context;
    char line[48];
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 1, 12, "Tailcat L2CAP bridge");
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 1, 26, app->connected ? "CoC connected" : "on-host / CDC #2");
    snprintf(line, sizeof(line), "SDU  >%lu  <%lu", app->tx_sdus, app->rx_sdus);
    canvas_draw_str(canvas, 1, 40, line);
    canvas_draw_str(canvas, 1, 55, "Back: exit");
}

static void input(InputEvent* event, void* context) {
    L2App* app = context;
    furi_message_queue_put(app->keys, event, 0);
}

int32_t tailcat_l2cap_app(void* context) {
    UNUSED(context);

    L2App* app = malloc(sizeof(L2App));
    memset(app, 0, sizeof(L2App));
    app->usb_tx = furi_semaphore_alloc(1, 1);
    app->keys = furi_message_queue_alloc(8, sizeof(InputEvent));
    app->tx = furi_stream_buffer_alloc(L2_TX_STREAM_SIZE, 1);
    app->tx_lock = furi_mutex_alloc(FuriMutexTypeNormal);
    app->worker = furi_thread_alloc_ex("L2capUsb", 2048, l2_worker, app);

    /* Use the resident host's CoC API — no raw HCI, no controller acquire. */
    ble_l2cap_coc_init();
    ble_l2cap_coc_set_callback(0, on_coc, app);
    ble_l2cap_fixed_init();
    ble_l2cap_fixed_set_callback(on_fixed, app);
    ble_l2cap_fixed_set_link_callback(on_fixed_link, app);
    ble_l2cap_fixed_set_link_info_callback(on_fixed_link_info, app);
    /* GATT fixtures the host defines (TASK-666). Registering the consumer adds
     * no services, so the companion's GATT table is untouched until it does. */
    ble_gatt_server_init();
    ble_gatt_server_set_callback(on_gatt_server, app);
    /* GATT client, for when the host drives the Flipper as a central. */
    ble_gatt_client_init();
    ble_gatt_client_set_callback(0, on_gattc, app); /* 0: any connection */

    Gui* gui = furi_record_open(RECORD_GUI);
    ViewPort* viewport = view_port_alloc();
    view_port_draw_callback_set(viewport, draw, app);
    view_port_input_callback_set(viewport, input, app);
    gui_add_view_port(gui, viewport, GuiLayerFullscreen);

    CliVcp* cli = furi_record_open(RECORD_CLI_VCP);
    FuriHalUsbInterface* previous = furi_hal_usb_get_config();
    /* The second port only exists while an app needs it, so claiming it means
     * re-enumerating USB. That drops the host's CLI connection, and a host that
     * launched this app with `loader open` would otherwise lose the reply
     * mid-flight and read it as a failed launch (KNOW-659). The loader prints
     * its reply only once this app is running, so wait a second before
     * claiming the port: the host then reads a finished command, and sees the
     * port go away and come back instead of a connection dying mid-sentence.
     * 300 ms was not enough, and the reply arrived empty. */
    bool reconfigure = previous != &usb_cdc_dual;
    bool usb_ok = true;
    if(reconfigure) {
        /* Only wait, never toggle the CLI here: cli_vcp_enable selects
         * usb_cdc_single, so disabling it first and enabling it after would put
         * the single-port config straight back and the second port would never
         * appear. Enabling an already-enabled CLI is a no-op. */
        furi_delay_ms(1000);
        furi_hal_usb_unlock();
        usb_ok = furi_hal_usb_set_config(&usb_cdc_dual, NULL);
        if(usb_ok) cli_vcp_enable(cli);
    }
    if(usb_ok) {
        furi_hal_cdc_set_callbacks(1, &usb_callbacks, app);
        furi_thread_start(app->worker);

        InputEvent key;
        uint32_t started = furi_get_tick();
        const uint32_t runtime_ticks = furi_ms_to_ticks(L2_DEFAULT_RUNTIME_SECONDS * 1000UL);
        while(!app->failed && furi_get_tick() - started < runtime_ticks) {
            if(furi_message_queue_get(app->keys, &key, 100) == FuriStatusOk &&
               key.key == InputKeyBack && key.type == InputTypeShort)
                break;
            view_port_update(viewport);
        }
        app->stop = true;
        furi_thread_join(app->worker);
        furi_hal_cdc_set_callbacks(1, NULL, NULL);
        if(reconfigure) {
            cli_vcp_disable(cli);
            furi_hal_usb_set_config(previous, NULL);
            cli_vcp_enable(cli);
        }
    }

    if(app->central) {
        furi_ble_session_free(app->central); /* drop the link, restore the companion */
        app->central = NULL;
    }
    furi_ble_adv_clear(); /* restore the companion advertisement */
    if(app->sec_claimed) {
        ble_security_set_callback(NULL, NULL);
        ble_security_deinit(); /* give pairing back to the firmware */
    }
    ble_gatt_client_set_callback(0, NULL, NULL);
    ble_gatt_client_deinit();
    ble_gatt_server_set_callback(NULL, NULL);
    ble_gatt_server_deinit(); /* removes our services and rebuilds the table */
    ble_l2cap_fixed_set_link_info_callback(NULL, NULL);
    ble_l2cap_fixed_set_link_callback(NULL, NULL);
    ble_l2cap_fixed_set_callback(NULL, NULL);
    ble_l2cap_fixed_deinit();
    ble_l2cap_coc_set_callback(0, NULL, NULL);
    ble_l2cap_coc_deinit();

    gui_remove_view_port(gui, viewport);
    view_port_free(viewport);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_CLI_VCP);
    furi_thread_free(app->worker);
    furi_stream_buffer_free(app->tx);
    furi_mutex_free(app->tx_lock);
    furi_message_queue_free(app->keys);
    furi_semaphore_free(app->usb_tx);
    free(app);
    return 0;
}
