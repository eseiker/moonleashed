#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* GATT services defined at runtime, without a profile. Add services and
 * characteristics, then commit: the host rebuilds its table, and handles are
 * valid once Committed arrives. Events arrive on the BLE dispatch thread. The
 * services are removed when the app exits, also without deinit. */

/* Bluetooth characteristic properties plus NimBLE's permission bits */
#define BLE_GATT_SERVER_F_BROADCAST       0x0001
#define BLE_GATT_SERVER_F_READ            0x0002
#define BLE_GATT_SERVER_F_WRITE_NO_RSP    0x0004
#define BLE_GATT_SERVER_F_WRITE           0x0008
#define BLE_GATT_SERVER_F_NOTIFY          0x0010
#define BLE_GATT_SERVER_F_INDICATE        0x0020
#define BLE_GATT_SERVER_F_AUTH_SIGN_WRITE 0x0040
#define BLE_GATT_SERVER_F_READ_ENC        0x0200
#define BLE_GATT_SERVER_F_READ_AUTHEN     0x0400
#define BLE_GATT_SERVER_F_WRITE_ENC       0x1000
#define BLE_GATT_SERVER_F_WRITE_AUTHEN    0x2000

#define BLE_GATT_SERVER_VALUE_MAX 512

typedef struct {
    uint8_t type; /**< 16 or 128 */
    uint16_t uuid16;
    uint8_t uuid128[16]; /**< little-endian */
} BleGattServerUuid;

typedef enum {
    /** data and data_len are valid */
    BleGattServerEventTypeWrite,
    /** notify and indicate are valid */
    BleGattServerEventTypeSubscribe,
    /** The table was rebuilt, by this app or by another owner such as a
     *  profile: read the handles again. */
    BleGattServerEventTypeCommitted,
} BleGattServerEventType;

typedef struct {
    BleGattServerEventType type;
    int32_t char_id;
    uint16_t connection_handle;
    bool notify;
    bool indicate;
    uint16_t data_len;
    const uint8_t* data;
} BleGattServerEvent;

typedef void (*BleGattServerCallback)(const BleGattServerEvent* event, void* context);

void ble_gatt_server_init(void);

/** Drop the callback and remove this API's services. */
void ble_gatt_server_deinit(void);

void ble_gatt_server_set_callback(BleGattServerCallback callback, void* context);

/** Returns a service id, or -1. */
int32_t ble_gatt_server_service_add(const BleGattServerUuid* uuid, bool primary);

bool ble_gatt_server_service_remove(int32_t service_id);

/** Returns a characteristic id, or -1. max_len is capped at
 *  BLE_GATT_SERVER_VALUE_MAX. */
int32_t ble_gatt_server_char_add(
    int32_t service_id,
    const BleGattServerUuid* uuid,
    uint16_t flags,
    uint16_t max_len,
    const uint8_t* init_value,
    uint16_t init_len);

bool ble_gatt_server_char_remove(int32_t char_id);

/** Store a value; once committed, subscribers are notified or indicated. */
bool ble_gatt_server_char_set_value(int32_t char_id, const uint8_t* data, uint16_t len);

/** 0 until committed. */
bool ble_gatt_server_char_handles(int32_t char_id, uint16_t* decl_handle, uint16_t* value_handle);

/** Rebuild the table on the host thread; Committed follows. The rebuild waits
 *  while a central session or the extra beacon runs. */
bool ble_gatt_server_commit(void);

#ifdef __cplusplus
}
#endif
