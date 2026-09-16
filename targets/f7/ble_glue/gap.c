/*
 * Stock GAP state machine — retired (TASK-696).
 *
 * This used to drive advertising, pairing and connection-parameter negotiation
 * by sending ST vendor ACI commands to a BLE host running on CPU2. This fork
 * ships the HCILayer radio, which is a bare controller with no host, and the
 * resident NimBLE host on CPU1 owns GAP instead (lib/nimble_furi/glue).
 *
 * The path was already unreachable: furi_hal_bt_start_app only calls gap_init
 * when furi_hal_bt_is_gatt_gap_supported() is true, and that is false for this
 * radio. The 23 ACI and HCI calls were removed so their command builders leave
 * the link, which is about 19.5 KB of flash.
 *
 * The entry points stay, because gap.h is consumed by furi_hal_bt.c,
 * ble_app.c, extra_beacon.c, bt_keys_storage.c and the profile interface, and
 * the SDK major must stay 88.
 */

#include "gap.h"

#include "furi_ble/event_dispatcher.h"

#include <furi.h>
#include <stdint.h>

#define TAG "BleGap"

static GapState gap_state = GapStateUninitialized;

bool gap_init(
    GapConfig* config,
    const GapRootSecurityKeys* root_keys,
    GapEventCallback on_event_cb,
    void* context) {
    UNUSED(config);
    UNUSED(root_keys);
    UNUSED(on_event_cb);
    UNUSED(context);
    FURI_LOG_E(TAG, "stock GAP needs a CPU2 host; this radio has none");
    return false;
}

void gap_start_advertising(void) {
    FURI_LOG_E(TAG, "stock GAP advertising is unavailable; NimBLE owns GAP");
}

void gap_stop_advertising(void) {
}

GapState gap_get_state(void) {
    return gap_state;
}

void gap_thread_stop(void) {
    gap_state = GapStateUninitialized;
}

void gap_emit_ble_beacon_status_event(bool active) {
    UNUSED(active);
}

BleEventFlowStatus ble_event_app_notification(void* pckt) {
    /* Reached only if an event arrives with no stock GAP to consume it. The
     * NimBLE event dispatcher acks its own events, so this stays quiet rather
     * than dereferencing state that was never initialized. */
    UNUSED(pckt);
    return BleEventFlowEnable;
}
