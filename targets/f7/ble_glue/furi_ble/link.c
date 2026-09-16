/*
 * BLE link control (TASK-688), backed by the resident NimBLE host. See link.h.
 */

#include "link.h"

#include <nimble_glue.h>
#include <furi.h>

#define TAG "BleLink"

/* NimBLE's BLE_HS_ENOTCONN. Declared here so this file keeps to the plain-C
 * glue header instead of pulling in the NimBLE host headers. */
#define BLE_LINK_HS_ENOTCONN 7

BleLinkDisconnectStatus ble_link_disconnect(uint16_t connection_handle, uint8_t reason) {
    int rc = nimble_glue_link_terminate(connection_handle, reason);
    if(rc == 0) return BleLinkDisconnectOk;
    if(rc == BLE_LINK_HS_ENOTCONN) return BleLinkDisconnectNotConnected;
    FURI_LOG_W(TAG, "disconnect(%u) refused, rc=%d", connection_handle, rc);
    return BleLinkDisconnectFailed;
}
