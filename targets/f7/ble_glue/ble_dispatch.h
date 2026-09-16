#pragma once

/*
 * Firmware-internal BLE event dispatch thread (TASK-631). NOT part of the
 * app-facing SDK: it lives next to gatt_host_shim.h, outside ble_glue/furi_ble,
 * so the SDK checker never sees it.
 *
 * Why: NimBLE delivers GAP/GATT/L2CAP events on its host thread. FAP callbacks
 * registered through the furi_ble APIs (ble_l2cap_coc_*, ble_gatt_client_*,
 * and stock ble_event_dispatcher service handlers) must not run there — a slow
 * or blocking callback would stall the whole host, and a callback into an
 * unloaded FAP would crash it. The host thread copies each event into a heap
 * blob and posts it here; one dedicated thread invokes the FAP callback.
 *
 * Unregister safety: adapters take ble_dispatch_lock() around both "look up the
 * callback and call it" (on the dispatch thread) and "clear the callback" (on
 * the FAP thread). The lock is recursive, so a callback may unregister itself.
 * Once an unregister call returns, that callback never runs again.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Deliver function; runs on the dispatch thread. `blob` is freed after it returns. */
typedef void (*BleDispatchFn)(void* blob);

/** Start the dispatch thread. Idempotent and safe to call from any thread
 *  except the NimBLE host thread (it allocates). */
void ble_dispatch_init(void);

/** Queue `fn(blob)` for the dispatch thread. Never blocks: call it from the
 *  NimBLE host thread. Takes ownership of `blob` (a malloc'd block) in every
 *  case — if the queue is full or dispatch is not running, the blob is freed
 *  and false is returned (the event is dropped and counted). */
bool ble_dispatch_post(BleDispatchFn fn, void* blob);

/** Number of events dropped because the queue was full or not started. */
uint32_t ble_dispatch_dropped(void);

/** Recursive lock serializing callback invocation against unregistration. */
void ble_dispatch_lock(void);
void ble_dispatch_unlock(void);

#ifdef __cplusplus
}
#endif
