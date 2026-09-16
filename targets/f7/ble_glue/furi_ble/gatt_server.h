#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Runtime GATT server on the resident NimBLE host (TASK-666).
 *
 * The stock ble_gatt_* API (gatt.h) defines services through a profile's
 * service instances. This API defines them directly: an app builds services and
 * characteristics, commits once, and the host rebuilds its attribute table. It
 * exists so a caller that is not a profile — the Tailcat bridge, driven by a USB
 * host — can serve GATT fixtures.
 *
 * Both APIs share one registry, so services defined here coexist with a
 * profile's. Handles are assigned at commit: read them back with
 * ble_gatt_server_char_handles once BleGattServerEventTypeCommitted arrives.
 *
 * Events are delivered on the BLE dispatch thread, off the NimBLE host thread.
 */

/** Characteristic properties. Values match the Bluetooth characteristic
 *  property bits, plus NimBLE's permission bits. */
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

/** Largest characteristic value this registry stores. */
#define BLE_GATT_SERVER_VALUE_MAX 512

typedef struct {
    uint8_t type; /**< 16 or 128 */
    uint16_t uuid16;
    uint8_t uuid128[16]; /**< little-endian, as on air */
} BleGattServerUuid;

typedef enum {
    /** A peer wrote the characteristic value. data and data_len are valid. */
    BleGattServerEventTypeWrite,
    /** A peer changed the CCCD. notify and indicate are valid. */
    BleGattServerEventTypeSubscribe,
    /** The attribute table was rebuilt; handles are valid now. */
    BleGattServerEventTypeCommitted,
} BleGattServerEventType;

typedef struct {
    BleGattServerEventType type;
    int32_t char_id;
    uint16_t connection_handle;
    bool notify;
    bool indicate;
    uint16_t data_len;
    const uint8_t* data; /**< valid only during the call */
} BleGattServerEvent;

typedef void (*BleGattServerCallback)(const BleGattServerEvent* event, void* context);

/** Start using the runtime GATT server (idempotent). */
void ble_gatt_server_init(void);

/** Drop the callback and remove every service this API added. */
void ble_gatt_server_deinit(void);

/** Set the event callback (NULL to clear). */
void ble_gatt_server_set_callback(BleGattServerCallback callback, void* context);

/** Add a service. Returns its id, or -1 when the registry is full. */
int32_t ble_gatt_server_service_add(const BleGattServerUuid* uuid, bool primary);

/** Remove a service and its characteristics. */
bool ble_gatt_server_service_remove(int32_t service_id);

/** Add a characteristic to a service. Returns its id, or -1 on failure.
 *  max_len is capped at BLE_GATT_SERVER_VALUE_MAX. */
int32_t ble_gatt_server_char_add(
    int32_t service_id,
    const BleGattServerUuid* uuid,
    uint16_t flags,
    uint16_t max_len,
    const uint8_t* init_value,
    uint16_t init_len);

/** Remove a characteristic. */
bool ble_gatt_server_char_remove(int32_t char_id);

/** Store a new value. Once the table is live, subscribed peers are notified or
 *  indicated according to the characteristic's flags. */
bool ble_gatt_server_char_set_value(int32_t char_id, const uint8_t* data, uint16_t len);

/** Read the assigned handles. Valid after a commit; 0 before it. */
bool ble_gatt_server_char_handles(int32_t char_id, uint16_t* decl_handle, uint16_t* value_handle);

/** Rebuild the attribute table so the definitions take effect. The rebuild runs
 *  on the host thread; BleGattServerEventTypeCommitted reports it finished. */
bool ble_gatt_server_commit(void);

#ifdef __cplusplus
}
#endif
