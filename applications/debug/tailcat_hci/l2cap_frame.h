#pragma once
#include <stddef.h>
#include <stdint.h>

/*
 * Simple TLV framing for the host-L2CAP USB bridge (TASK-646). Unlike the raw
 * HCI mode's H4 framing, here the Flipper is the BLE host and the USB link only
 * carries L2CAP CoC control + SDUs. Frame layout: [type:1][len:2 LE][payload:len].
 */

/* Frame payload bound. Kept a round 2048; a SEND/DATA payload is
 * [channel:1][sdu...], so the largest SDU that fits one frame is 2047. */
#define L2F_PAYLOAD_MAX 2048U

/* CoC MTU the bridge negotiates for both LISTEN and CONNECT. It is tied to the
 * frame bound (L2F_PAYLOAD_MAX - 1) instead of BLE_L2CAP_COC_MTU_DEFAULT (2048)
 * so a full-MTU SDU plus the channel byte always fits one DATA frame. NimBLE
 * rejects any received SDU above the MTU it advertised, so an SDU can never be
 * truncated on its way to the host. DCT negotiates 1550 and is unaffected. */
#define L2F_COC_MTU (L2F_PAYLOAD_MAX - 1U)

_Static_assert(L2F_COC_MTU + 1U <= L2F_PAYLOAD_MAX, "CoC MTU + channel byte must fit a frame");

/* Host -> FAP */
#define L2F_LISTEN 0x01 /* [psm:2]                     start a CoC server   */
#define L2F_SEND   0x02 /* [channel:1][data...]        send an SDU          */
#define L2F_CLOSE  0x03 /* [channel:1]                 disconnect a channel */
#define L2F_CONNECT \
    0x04 /* [psm:2][name...]            central: scan+connect, then CoC client on psm */
#define L2F_ADVERTISE      0x05 /* [adv_len:1][adv...][rsp...] install raw advertisement */
/* Advertise-once (TASK-721, KNOW-720). ADVERTISE_ONCE takes the same payload as
 * ADVERTISE, but the advertisement stops when the first peer connects, the way a
 * raw-HCI advertiser with auto_restart off behaves. A protocol that expects one
 * connection per stage needs that, and re-arms with ADV_RESTART between stages.
 * ADV_STOP drops the raw payload, so the companion advertisement comes back. */
#define L2F_ADVERTISE_ONCE 0x06 /* [adv_len:1][adv...][rsp...] */
#define L2F_ADV_RESTART    0x07 /* []                          */
#define L2F_ADV_STOP       0x08 /* []                          */

/* Fixed L2CAP CID relay (TASK-663), a separate family from the CoC frames so
 * their numbering is untouched. These carry a raw L2CAP CID and a connection
 * handle rather than a CoC channel index. */
#define L2F_FIXED_REGISTER 0x10 /* [cid:2][mtu:2]                   relay a fixed CID */
#define L2F_FIXED_SEND     0x11 /* [conn:2][cid:2][pdu...]          transmit a PDU    */
#define L2F_FIXED_UNREG    0x12 /* [cid:2]                          stop relaying     */

/* Pairing / SMP control (TASK-665). The first SEC_* frame takes pairing over
 * from the firmware; until then the firmware pairs the companion's way.
 * SEC_CONFIG payload is [io_cap:1][flags:1][our_kd:1][their_kd:1], where flags
 * is bit0 bonding, bit1 MITM, bit2 secure connections. */
#define L2F_SEC_CONFIG       0x20
#define L2F_SEC_PAIR         0x21 /* [conn:2]                 start pairing / encryption */
#define L2F_SEC_PASSKEY      0x22 /* [conn:2][passkey:4]      answer a passkey prompt     */
#define L2F_SEC_CONFIRM      0x23 /* [conn:2][accept:1]       answer a numeric comparison */
/* LE Secure Connections OOB (TASK-689). The confirm value derives from the
 * Flipper's SC public key, so the host cannot compute one: it asks for ours
 * with SEC_OOB_GENERATE, which answers SEC_OOB_LOCAL, carries it to the peer
 * out of band, and arms the peer's with SEC_OOB_PEER before pairing starts. */
#define L2F_SEC_OOB_GENERATE 0x24 /* []                            */
#define L2F_SEC_OOB_PEER     0x25 /* [random:16][confirm:16]       */
#define L2F_SEC_OOB_CLEAR    0x26 /* []                            */

/* Runtime GATT server (TASK-666): define and serve fixtures from the host. */
#define L2F_GATT_SERVICE 0x30 /* [primary:1][uuid_type:1][uuid:2|16]  add a service */
#define L2F_GATT_CHAR    0x31 /* [svc_id:1][flags:2][max_len:2][uuid_type:1][uuid:2|16][init...] */
#define L2F_GATT_COMMIT  0x32 /* []                       rebuild the attribute table */
#define L2F_GATT_SET     0x33 /* [chr_id:1][value...]     store a value, notify peers */
#define L2F_GATT_RESET   0x34 /* []                       remove every service we added */

/* Link control (TASK-688). A protocol that walks several connections abandons a
 * stage by dropping the link itself. The connection handle is the one
 * FIXED_LINK reported; reason 0 means remote user terminated. The link going
 * down is reported by FIXED_LINK as usual. */
#define L2F_LINK_DISCONNECT 0x40 /* [conn:2][reason:1] */

/* GATT client (TASK-633): drive the Flipper as a central against a peer's GATT
 * server. Get a link first with CONNECT; a psm of 0 there connects without
 * opening a CoC, which is what these frames want. uuid_type follows the
 * ble_gatt_client_* API, where 1 is a 16-bit UUID and 2 a 128-bit one. */
#define L2F_GATTC_DISCOVER  0x50 /* [conn:2]                                  services   */
#define L2F_GATTC_CHARS     0x51 /* [conn:2][start:2][end:2]                  chars      */
#define L2F_GATTC_READ      0x52 /* [conn:2][value_handle:2]                             */
#define L2F_GATTC_WRITE     0x53 /* [conn:2][value_handle:2][data...]                    */
#define L2F_GATTC_SUBSCRIBE 0x54 /* [conn:2][value_handle:2][enable:1]                   */
#define L2F_GATTC_MTU       0x55 /* [conn:2]                                  exchange   */

/* FAP -> Host */
#define L2F_CONNECTED \
    0x81 /* [channel:1][conn:2][peer_mtu:2] — peer_mtu is the
                                * peer's CoC SDU MTU; cap SENDs at min(peer_mtu,
                                * L2F_COC_MTU). Appended field: readers of the old
                                * 3-byte payload keep working. */
#define L2F_DATA         0x82 /* [channel:1][data...] */
#define L2F_DISCONNECTED 0x83 /* [channel:1]          */
#define L2F_ERROR        0x84 /* [code:2]             */
#define L2F_FIXED_DATA   0x90 /* [conn:2][cid:2][pdu...] inbound fixed-CID PDU */
/* FIXED_LINK: a BLE link came up (up=1) or went down (up=0). Sent for every
 * link while the bridge runs, and again for each existing link after a
 * FIXED_REGISTER, so the host learns the conn to FIXED_SEND on before the peer
 * speaks (Magnet's VersionInfo goes first). An up report can repeat. */
#define L2F_FIXED_LINK   0x91 /* [conn:2][up:1] */
/* FIXED_LINK_INFO: the same report as FIXED_LINK with what a multi-connection
 * protocol needs (TASK-713, KNOW-711). It is sent immediately before each
 * FIXED_LINK, so a host that only reads FIXED_LINK is unaffected. An address is
 * [type:1][addr:6], little-endian, with type 0 public, 1 random, 2 public
 * identity, 3 random identity. The over-the-air pair is the link-layer
 * addresses; the identity pair is the resolved one. reason is the HCI
 * disconnect reason on a down report, 19 for remote user terminated, and 0 on
 * an up report. */
#define L2F_FIXED_LINK_INFO \
    0x92 /* [conn:2][up:1][reason:1][peer_ota:7][peer_id:7][our_ota:7][our_id:7] */
/* SEC_EVENT payload: [kind:1][conn:2][status:2][passkey:4][flags:1][key_size:1].
 * kind is 0 passkey display, 1 passkey request, 2 numeric comparison,
 * 3 OOB request, 4 encryption changed, 5 repeat pairing.
 * flags is bit0 encrypted, bit1 authenticated, bit2 bonded. */
#define L2F_SEC_EVENT        0xA0
#define L2F_GATT_ID          0xB0 /* [kind:1][id:1][decl:2][value:2] kind 0 service, 1 characteristic */
/* GATT_WRITE payload: [chr_id:1][conn:2][kind:1][data...], where kind 0 is a
 * value write and kind 1 a CCCD change with data [notify:1][indicate:1]. */
#define L2F_GATT_WRITE       0xB1
#define L2F_GATT_READY       0xB2 /* []  the attribute table was rebuilt; handles are valid */
/* Answer to LINK_DISCONNECT: status 0 accepted, 1 no such link (already gone),
 * 2 refused. It reports only that the request was taken, not that the link is
 * down; FIXED_LINK reports that. */
#define L2F_LINK_STATUS      0xC0 /* [conn:2][status:1] */
/* Answer to SEC_OOB_GENERATE: our Secure Connections OOB values, to carry to
 * the peer out of band. An empty payload means the generator failed. */
#define L2F_SEC_OOB_LOCAL    0xA1 /* [random:16][confirm:16] */
/* GATT client results (TASK-633). SERVICE and CHAR arrive one per entry, then
 * DONE closes the run: what is 0 for services and 1 for characteristics. */
#define L2F_GATTC_SERVICE    0xD0 /* [conn:2][uuid_type:1][uuid:2|16][start:2][end:2]     */
#define L2F_GATTC_CHAR       0xD1 /* [conn:2][uuid_type:1][uuid:2|16][decl:2][value:2][props:1] */
#define L2F_GATTC_DONE       0xD2 /* [conn:2][what:1]                                     */
#define L2F_GATTC_READ_DATA  0xD3 /* [conn:2][value_handle:2][data...]                    */
#define L2F_GATTC_NOTIFY     0xD4 /* [conn:2][value_handle:2][data...]                    */
#define L2F_GATTC_WRITE_DONE 0xD5 /* [conn:2][value_handle:2]                             */
#define L2F_GATTC_ERROR      0xD6 /* [conn:2][code:1]                                     */

typedef struct {
    uint8_t type;
    uint16_t len;
    uint16_t used;
    uint8_t data[L2F_PAYLOAD_MAX];
    uint8_t hdr[3];
    uint8_t hdr_used;
} L2Frame;

static inline void l2f_reset(L2Frame* f) {
    f->hdr_used = 0;
    f->used = 0;
}

/* Feed one byte. -1 rejects the stream; 0 needs more; 1 emits a complete frame
 * (read f->type / f->len / f->data, then call l2f_reset). */
static inline int l2f_push(L2Frame* f, uint8_t b) {
    if(f->hdr_used < 3) {
        f->hdr[f->hdr_used++] = b;
        if(f->hdr_used == 3) {
            f->type = f->hdr[0];
            f->len = (uint16_t)f->hdr[1] | ((uint16_t)f->hdr[2] << 8);
            f->used = 0;
            if(f->len > L2F_PAYLOAD_MAX) return -1;
            if(f->len == 0) return 1;
        }
        return 0;
    }
    f->data[f->used++] = b;
    return f->used == f->len ? 1 : 0;
}
