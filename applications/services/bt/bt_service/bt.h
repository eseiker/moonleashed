#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <furi_ble/profile_interface.h>
#include <core/common_defines.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RECORD_BT "bt"

typedef struct Bt Bt;

typedef enum {
    BtStatusUnavailable,
    BtStatusOff,
    BtStatusAdvertising,
    BtStatusConnected,
} BtStatus;

typedef struct {
    uint8_t rssi;
    uint32_t since;
} BtRssi;

typedef void (*BtStatusChangedCallback)(BtStatus status, void* context);

/** Change BLE Profile
 * @note Call of this function leads to 2nd core restart
 *
 * @param bt                 Bt instance
 * @param profile_template   Profile template to change to
 * @param params             Profile parameters. Can be NULL
 *
 * @return          true on success
 */
FURI_WARN_UNUSED FuriHalBleProfileBase* bt_profile_start(
    Bt* bt,
    const FuriHalBleProfileTemplate* profile_template,
    FuriHalBleProfileParams params);

/** Stop current BLE Profile and restore default profile
 * @note Call of this function leads to 2nd core restart
 *
 * @param bt        Bt instance
 *
 * @return          true on success
 */
bool bt_profile_restore_default(Bt* bt);

/** Suspend the BT service while an external controller host owns Core2.
 * @note The caller must restore the default profile or reboot before exiting.
 *
 * @param bt        Bt instance
 *
 * @return          true on success
 */
bool bt_profile_suspend(Bt* bt);

/** Resume the default BLE profile after external controller ownership.
 * @note Call of this function leads to 2nd core restart
 *
 * @param bt        Bt instance
 *
 * @return          true on success
 */
bool bt_profile_resume_default(Bt* bt);

/** Disconnect from Central
 *
 * @param bt        Bt instance
 */
void bt_disconnect(Bt* bt);

/** Set callback for Bluetooth status change notification
 *
 * @param bt        Bt instance
 * @param callback  BtStatusChangedCallback instance
 * @param context   pointer to context
 */
void bt_set_status_changed_callback(Bt* bt, BtStatusChangedCallback callback, void* context);

/** Forget bonded devices
 * @note Leads to wipe ble key storage and deleting bt.keys
 *
 * @param bt        Bt instance
 */
void bt_forget_bonded_devices(Bt* bt);

/** Hand the BLE controller to a raw-HCI app (TASK-759).
 *
 * The resident NimBLE host stops and the controller is released, so the caller
 * can take it with furi_hal_bt_hci_acquire and speak H4 to an external host.
 * Bluetooth is unavailable until bt_reclaim_controller returns it. Calling this
 * twice is harmless. Returns false if the controller could not be released, in
 * which case BLE stays down until the device reboots.
 *
 * @param bt                    Bt instance
 * @return                      true if the controller is free for the caller
 */
bool bt_release_controller_to_raw_hci(Bt* bt);

/** Take the controller back and restart the resident NimBLE host.
 *
 * The caller must have released the controller with furi_hal_bt_hci_release
 * first. Returns false if the host did not come back, which leaves Bluetooth
 * unavailable until the device reboots.
 *
 * @param bt                    Bt instance
 * @return                      true if the host is running again
 */
bool bt_reclaim_controller(Bt* bt);

/** Set keys storage file path
 *
 * @param bt                    Bt instance
 * @param keys_storage_path     Path to file with saved keys
 */
void bt_keys_storage_set_storage_path(Bt* bt, const char* keys_storage_path);

/** Set default keys storage file path
 *
 * @param bt                    Bt instance
 */
void bt_keys_storage_set_default_path(Bt* bt);

#ifdef __cplusplus
}
#endif
