#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool enabled;
    /* NimBLE BLE operating mode (NimbleMode value; 0 = combined serial+HID
     * peripheral, the default). Kept as a plain integer so the settings FAP,
     * which cannot include the NimBLE headers, can carry it; the bt service maps
     * it to NimbleMode. Only meaningful when the NimBLE host drives the radio. */
    uint8_t ble_mode;
} BtSettings;

void bt_settings_load(BtSettings* bt_settings);

void bt_settings_save(const BtSettings* bt_settings);

#ifdef __cplusplus
}
#endif
