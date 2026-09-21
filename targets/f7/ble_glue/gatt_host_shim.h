#pragma once

/* Firmware-internal, outside the SDK headers. While a shim is installed, the
 * ble_gatt_* primitives do not talk to CPU2: they hand out placeholder handles
 * and pass every characteristic update to on_update. App profiles linked from
 * lib/ble_profile then run on a host that serves the services itself. */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @param char_uuid16  16-bit characteristic UUID, 0 for a 128-bit one
 * @param report_ref   (report type << 8) | report id from a Report Reference
 *                     descriptor, else 0
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

/** Install, or clear with NULL. The shim must outlive the installation. */
void ble_gatt_host_shim_set(const BleGattHostShim* shim);

/** A running profile added the HID service. */
bool ble_gatt_host_shim_has_hid(void);

#ifdef __cplusplus
}
#endif
