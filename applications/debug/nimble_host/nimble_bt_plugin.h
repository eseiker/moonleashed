/*
 * Bridge <-> module ABI for the resident NimBLE BLE host (TASK-594, B4).
 *
 * The NimBLE host ships as a Flipper PLUGIN (.fal) on the SD card. A thin
 * firmware bt bridge loads it with plugin_manager, takes the NimbleBtBackend
 * function table from the plugin descriptor's entry_point, and drives it to
 * implement bt.h. The module keeps the whole NimBLE stack (host, NPL-over-FURI,
 * Serial Service GATT server, RPC bridge) and resolves its firmware imports via
 * firmware_api_interface, so nothing NimBLE-sized lives in firmware flash.
 *
 * This header is the only thing the firmware bridge and the plugin share.
 */

#ifndef NIMBLE_BT_PLUGIN_H_
#define NIMBLE_BT_PLUGIN_H_

#include <stdbool.h>
#include <stdint.h>

#define NIMBLE_BT_PLUGIN_APP_ID      "nimble_bt"
#define NIMBLE_BT_PLUGIN_API_VERSION 1

/* Backend state the bridge polls to drive BtStatus. */
typedef enum {
    NimbleBtStateOff = 0,
    NimbleBtStateAdvertising,
    NimbleBtStateConnected,
    NimbleBtStateFault,
} NimbleBtState;

/*
 * Function table the plugin exposes as its descriptor entry_point. All calls run
 * on the bridge's thread; the module owns its own host thread internally.
 */
typedef struct {
    /* Acquire the raw HCI controller, bring up the NimBLE host, and start
     * advertising the Serial Service. Returns false if the controller could not
     * be acquired (e.g. wrong radio stack). */
    bool (*start)(void);

    /* Stop the host and release the controller. Safe to call after a failed
     * start. */
    void (*stop)(void);

    /* Current link state, for the bridge to map onto BtStatus. */
    NimbleBtState (*get_state)(void);

    /* Terminate the active connection, if any. */
    void (*disconnect)(void);

    /* Wipe persisted bonds. */
    void (*forget_bonds)(void);

    /* Point the module's bond store at this file path (bt_keys_storage_*). NULL
     * or unset uses the module default. */
    void (*set_keys_path)(const char* path);
} NimbleBtBackend;

#endif /* NIMBLE_BT_PLUGIN_H_ */
