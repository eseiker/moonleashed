#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define H4_FRAME_MAX 260U
typedef struct {
    uint8_t bytes[H4_FRAME_MAX];
    size_t used;
    size_t expected;
} H4Frame;

/* -1 rejects the stream; 0 needs more bytes; 1 emits one complete frame. */
static inline int h4_push(H4Frame* frame, uint8_t byte) {
    if(frame->used == 0) {
        if(byte != 1 && byte != 2) return -1;
        frame->expected = byte == 1 ? 4 : 5;
    }
    if(frame->used >= H4_FRAME_MAX) return -1;
    frame->bytes[frame->used++] = byte;
    if(frame->used == (frame->bytes[0] == 1 ? 4U : 5U)) {
        size_t payload = frame->bytes[0] == 1 ? frame->bytes[3] :
                                                frame->bytes[3] | ((size_t)frame->bytes[4] << 8);
        frame->expected += payload;
        if(frame->expected > H4_FRAME_MAX) return -1;
    }
    return frame->used == frame->expected ? 1 : 0;
}
