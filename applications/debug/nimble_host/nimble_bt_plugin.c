/*
 * NimBLE BLE host as a Flipper PLUGIN (TASK-594, B4.1).
 *
 * This is the headless, resident form of the nimble_host module: instead of a
 * GUI FAP it exposes a NimbleBtBackend function table through the standard
 * plugin descriptor, so a firmware bt bridge can load it from SD with
 * plugin_manager and drive it to implement bt.h. All the BLE work (host,
 * Serial Service, RPC bridge, bonds) stays here in the module; the plugin
 * resolves its firmware imports via firmware_api_interface.
 */

#include "nimble_bt_plugin.h"

#include <furi.h>
#include <furi_hal_bt_hci.h>
#include <flipper_application/flipper_application.h>

#include "nimble_glue.h"

#define TAG "NimbleBt"

static bool nimble_bt_start(void) {
    if(furi_hal_bt_hci_get_abi() != FURI_HAL_BT_HCI_ABI) {
        FURI_LOG_E(TAG, "HCI ABI mismatch");
        return false;
    }
    if(!furi_hal_bt_hci_acquire(FURI_HAL_BT_HCI_ABI)) {
        FURI_LOG_E(TAG, "Raw HCI controller acquire failed");
        return false;
    }
    if(!nimble_glue_start()) {
        FURI_LOG_E(TAG, "NimBLE host start failed");
        furi_hal_bt_hci_release();
        return false;
    }
    return true;
}

static void nimble_bt_stop(void) {
    nimble_glue_stop();
    furi_hal_bt_hci_release();
}

static NimbleBtState nimble_bt_get_state(void) {
    if(nimble_glue_faulted()) return NimbleBtStateFault;
    if(nimble_glue_is_connected()) return NimbleBtStateConnected;
    if(nimble_glue_is_advertising()) return NimbleBtStateAdvertising;
    return NimbleBtStateOff;
}

static void nimble_bt_disconnect(void) {
    nimble_glue_disconnect();
}

static void nimble_bt_forget_bonds(void) {
    nimble_glue_forget_bonds();
}

static void nimble_bt_set_keys_path(const char* path) {
    /* B4.2: honor the bt_keys_storage path. For now the module uses its default
     * store file. */
    UNUSED(path);
}

static const NimbleBtBackend nimble_bt_backend = {
    .start = &nimble_bt_start,
    .stop = &nimble_bt_stop,
    .get_state = &nimble_bt_get_state,
    .disconnect = &nimble_bt_disconnect,
    .forget_bonds = &nimble_bt_forget_bonds,
    .set_keys_path = &nimble_bt_set_keys_path,
};

static const FlipperAppPluginDescriptor nimble_bt_descriptor = {
    .appid = NIMBLE_BT_PLUGIN_APP_ID,
    .ep_api_version = NIMBLE_BT_PLUGIN_API_VERSION,
    .entry_point = &nimble_bt_backend,
};

const FlipperAppPluginDescriptor* nimble_bt_ep(void) {
    return &nimble_bt_descriptor;
}
