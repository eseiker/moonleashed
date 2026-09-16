/*
 * BLE event dispatch thread (TASK-631). See ble_dispatch.h.
 */

#include "ble_dispatch.h"

#include <furi.h>

#define TAG "BleDispatch"

#define BLE_DISPATCH_QUEUE_DEPTH 64
#define BLE_DISPATCH_STACK_SIZE  3072

typedef struct {
    BleDispatchFn fn;
    void* blob;
} BleDispatchMsg;

static FuriMessageQueue* s_queue;
static FuriMutex* s_lock;
static FuriThread* s_thread;
static volatile bool s_ready;
static bool s_initializing;
static volatile uint32_t s_dropped;

static int32_t ble_dispatch_worker(void* context) {
    UNUSED(context);
    BleDispatchMsg msg;
    for(;;) {
        if(furi_message_queue_get(s_queue, &msg, FuriWaitForever) != FuriStatusOk) continue;
        if(msg.fn) msg.fn(msg.blob);
        free(msg.blob);
    }
    return 0;
}

void ble_dispatch_init(void) {
    if(s_ready) return;

    bool mine = false;
    FURI_CRITICAL_ENTER();
    if(!s_initializing) {
        s_initializing = true;
        mine = true;
    }
    FURI_CRITICAL_EXIT();

    if(!mine) {
        /* Another thread is starting it; wait until it is up. */
        while(!s_ready)
            furi_delay_ms(1);
        return;
    }

    s_queue = furi_message_queue_alloc(BLE_DISPATCH_QUEUE_DEPTH, sizeof(BleDispatchMsg));
    s_lock = furi_mutex_alloc(FuriMutexTypeRecursive);
    s_thread =
        furi_thread_alloc_ex("BleDispatch", BLE_DISPATCH_STACK_SIZE, ble_dispatch_worker, NULL);
    furi_thread_start(s_thread);
    s_ready = true;
    FURI_LOG_I(TAG, "started");
}

bool ble_dispatch_post(BleDispatchFn fn, void* blob) {
    BleDispatchMsg msg = {.fn = fn, .blob = blob};
    if(s_ready && furi_message_queue_put(s_queue, &msg, 0) == FuriStatusOk) return true;
    s_dropped++;
    FURI_LOG_W(
        TAG, "event dropped (%s, total %lu)", s_ready ? "queue full" : "not started", s_dropped);
    free(blob);
    return false;
}

uint32_t ble_dispatch_dropped(void) {
    return s_dropped;
}

void ble_dispatch_lock(void) {
    if(s_lock) furi_mutex_acquire(s_lock, FuriWaitForever);
}

void ble_dispatch_unlock(void) {
    if(s_lock) furi_mutex_release(s_lock);
}
