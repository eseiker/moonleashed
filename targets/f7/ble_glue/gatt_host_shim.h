#pragma once

/*
 * Firmware-internal GATT host shim (TASK-612 / KNOW-613). NOT part of the
 * app-facing SDK: it lives next to gap.h, outside the ble_glue/furi_ble
 * directory that targets/f7/target.json exports as SDK headers, so the SDK
 * checker never sees it and the public API version does not change.
 *
 * Why it exists: HID FAPs (hid_app, bad_usb) statically link their own copy of
 * lib/ble_profile (fap_libs=["ble_profile"]; the ble_profile_hid_* and
 * ble_svc_hid_* symbols are not exported). The only firmware symbols that copy
 * reaches are the exported ble_gatt_* primitives in furi_ble/gatt.h, which in
 * the stock firmware issue ACI commands to the CPU2 GATT server. On a BLE HCI
 * Layer radio there is no CPU2 host: the firmware-resident NimBLE host owns the
 * controller and already serves the HID / Device Information / Battery services
 * itself.
 *
 * While a shim is installed, the ble_gatt_* primitives no longer talk to CPU2.
 * They hand out synthetic attribute handles, remember each characteristic's
 * Report Reference descriptor, and forward every characteristic value update to
 * on_update. The installer (the bt service) maps the updates it cares about
 * (HID input reports, battery level) onto the NimBLE GATT server. Everything
 * else (report map, HID information, device information strings) is ignored:
 * NimBLE serves those statically.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Called on the updating thread for every ble_gatt_characteristic_update while
 * the shim is installed.
 *
 * @param char_uuid16  16-bit characteristic UUID, or 0 for a 128-bit UUID
 * @param report_ref   Report Reference descriptor value as (report_type << 8) |
 *                     report_id when the characteristic carries a 0x2908
 *                     descriptor, else 0
 * @param data         characteristic value bytes (valid only during the call)
 * @param len          value length
 * @param context      installer context
 */
typedef void (*BleGattHostShimUpdateCb)(
    uint16_t char_uuid16,
    uint16_t report_ref,
    const uint8_t* data,
    uint16_t len,
    void* context);

typedef struct {
    BleGattHostShimUpdateCb on_update;
    void* context;
} BleGattHostShim;

/**
 * Install (or clear, with NULL) the GATT host shim. The shim struct must outlive
 * the installation. Installing also initialises the BLE event dispatcher so
 * service code that registers an event handler does not trip its init check.
 */
void ble_gatt_host_shim_set(const BleGattHostShim* shim);

/** @return true while a shim is installed (the CPU2 ACI path is bypassed). */
bool ble_gatt_host_shim_active(void);

#ifdef __cplusplus
}
#endif
