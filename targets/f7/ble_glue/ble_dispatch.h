#pragma once

/* Firmware-internal. App callbacks run on this thread, not on the NimBLE host
 * thread: a slow callback would stall the host. Its stack is 2 KB: a callback
 * should hand work to its own thread. */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Runs on the dispatch thread; blob is freed after it returns. */
typedef void (*BleDispatchFn)(void* blob);

/** Idempotent. Not from the NimBLE host thread: it allocates. */
void ble_dispatch_init(void);

/** Never blocks. Takes ownership of the malloc'd blob, and frees it if the
 *  event is dropped. */
bool ble_dispatch_post(BleDispatchFn fn, void* blob);

/** Recursive. Held around a callback and around its unregistration, so a
 *  callback never runs after it is unregistered. */
void ble_dispatch_lock(void);
void ble_dispatch_unlock(void);

#ifdef __cplusplus
}
#endif
