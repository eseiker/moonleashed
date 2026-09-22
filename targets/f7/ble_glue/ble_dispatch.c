#include "ble_dispatch.h"

#include <furi.h>

#define TAG "BleDispatch"

#define QUEUE_DEPTH 64
#define STACK_SIZE  2048

typedef struct {
    BleDispatchFn fn;
    void* blob;
} BleDispatchMsg;

static FuriMessageQueue* queue;
static FuriMutex* lock;
static FuriThread* thread;

static int32_t ble_dispatch_worker(void* context) {
    UNUSED(context);
    BleDispatchMsg msg;
    for(;;) {
        furi_check(furi_message_queue_get(queue, &msg, FuriWaitForever) == FuriStatusOk);
        msg.fn(msg.blob);
        free(msg.blob);
    }
    return 0;
}

void ble_dispatch_init(void) {
    if(thread) return;
    queue = furi_message_queue_alloc(QUEUE_DEPTH, sizeof(BleDispatchMsg));
    lock = furi_mutex_alloc(FuriMutexTypeRecursive);
    thread = furi_thread_alloc_ex("BleDispatch", STACK_SIZE, ble_dispatch_worker, NULL);
    furi_thread_start(thread);
}

bool ble_dispatch_post(BleDispatchFn fn, void* blob) {
    BleDispatchMsg msg = {.fn = fn, .blob = blob};
    if(thread && furi_message_queue_put(queue, &msg, 0) == FuriStatusOk) return true;
    FURI_LOG_W(TAG, "Event dropped");
    free(blob);
    return false;
}

void ble_dispatch_lock(void) {
    if(lock) furi_mutex_acquire(lock, FuriWaitForever);
}

void ble_dispatch_unlock(void) {
    if(lock) furi_mutex_release(lock);
}
