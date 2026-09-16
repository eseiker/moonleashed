#pragma once
#include <stddef.h>
#include <stdint.h>

/*
 * Simple TLV framing for the host-L2CAP USB bridge (TASK-646). Unlike the raw
 * HCI mode's H4 framing, here the Flipper is the BLE host and the USB link only
 * carries L2CAP CoC control + SDUs. Frame layout: [type:1][len:2 LE][payload:len].
 */

#define L2F_PAYLOAD_MAX 512U

/* Host -> FAP */
#define L2F_LISTEN     0x01 /* [psm:2]                     start a CoC server   */
#define L2F_SEND       0x02 /* [channel:1][data...]        send an SDU          */
#define L2F_CLOSE      0x03 /* [channel:1]                 disconnect a channel */
#define L2F_CONNECT    0x04 /* [name...]                   central: scan+connect */

/* FAP -> Host */
#define L2F_CONNECTED    0x81 /* [channel:1][conn:2]  */
#define L2F_DATA         0x82 /* [channel:1][data...] */
#define L2F_DISCONNECTED 0x83 /* [channel:1]          */
#define L2F_ERROR        0x84 /* [code:2]             */

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
