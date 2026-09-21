#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_GATT_CLIENT_MAX_SERVICES 16
#define BLE_GATT_CLIENT_MAX_CHARS    64

/* Moon Firmware passes this to aci_gatt_set_event_mask. Kept so its apps
 * compile; the NimBLE host ignores it. */
#define BLE_GATT_FULL_EVENT_MASK                                               \
    (0x00000001u | /* ACI_GATT_ATTRIBUTE_MODIFIED_EVENT                     */ \
     0x00000002u | /* ACI_GATT_PROC_TIMEOUT_EVENT                           */ \
     0x00000004u | /* ACI_ATT_EXCHANGE_MTU_RESP_EVENT                       */ \
     0x00000008u | /* ACI_ATT_FIND_INFO_RESP_EVENT                          */ \
     0x00000010u | /* ACI_ATT_FIND_BY_TYPE_VALUE_RESP_EVENT                 */ \
     0x00000020u | /* ACI_ATT_READ_BY_TYPE_RESP_EVENT                       */ \
     0x00000040u | /* ACI_ATT_READ_RESP_EVENT                               */ \
     0x00000080u | /* ACI_ATT_READ_BLOB_RESP_EVENT                          */ \
     0x00000100u | /* ACI_ATT_READ_MULTIPLE_RESP_EVENT                      */ \
     0x00000200u | /* ACI_ATT_READ_BY_GROUP_TYPE_RESP_EVENT                 */ \
     0x00000800u | /* ACI_ATT_PREPARE_WRITE_RESP_EVENT                      */ \
     0x00001000u | /* ACI_ATT_EXEC_WRITE_RESP_EVENT                         */ \
     0x00002000u | /* ACI_GATT_INDICATION_EVENT                             */ \
     0x00004000u | /* ACI_GATT_NOTIFICATION_EVENT                           */ \
     0x00008000u | /* ACI_GATT_ERROR_RESP_EVENT                             */ \
     0x00010000u | /* ACI_GATT_PROC_COMPLETE_EVENT                          */ \
     0x00020000u | /* ACI_GATT_DISC_READ_CHAR_BY_UUID_RESP_EVENT            */ \
     0x00040000u | /* ACI_GATT_TX_POOL_AVAILABLE_EVENT                      */ \
     0x00100000u | /* ACI_GATT_READ_EXT_EVENT                               */ \
     0x00200000u | /* ACI_GATT_INDICATION_EXT_EVENT                         */ \
     0x00400000u) /* ACI_GATT_NOTIFICATION_EXT_EVENT                        */

typedef struct {
    uint8_t uuid_type; // 1 = 16-bit, 2 = 128-bit
    uint16_t uuid_16;
    uint8_t uuid_128[16];
    uint16_t start_handle;
    uint16_t end_handle;
} BleGattService;

typedef struct {
    uint8_t uuid_type; // 1 = 16-bit, 2 = 128-bit
    uint16_t uuid_16;
    uint8_t uuid_128[16];
    uint16_t decl_handle;
    uint16_t value_handle;
    uint8_t properties;
} BleGattCharacteristic;

typedef enum {
    BleGattClientEventDiscoverComplete,
    BleGattClientEventCharDiscoverComplete,
    BleGattClientEventReadComplete,
    BleGattClientEventWriteComplete,
    BleGattClientEventNotification,
    BleGattClientEventError,
} BleGattClientEventType;

typedef struct {
    BleGattClientEventType type;
    uint16_t connection_handle; /**< Connection this event belongs to */
    union {
        struct {
            BleGattService* services;
            uint8_t count;
        } discover;
        struct {
            BleGattCharacteristic* chars;
            uint8_t count;
        } char_discover;
        struct {
            const uint8_t* data;
            uint16_t data_len;
            uint16_t value_handle;
        } read;
        struct {
            const uint8_t* data;
            uint16_t data_len;
            uint16_t value_handle;
            uint16_t offset; // Always 0: NimBLE delivers whole notifications
        } notification;
        struct {
            uint8_t error_code; // ATT error; 0x0E for a failure on our side
        } error;
    };
} BleGattClientEvent;

typedef void (*BleGattClientCallback)(BleGattClientEvent* event, void* context);

/* Moon-Firmware-compatible GATT client. Callbacks run on the BLE dispatch
 * thread; pointers in an event are valid during the callback. After
 * set_callback(handle, NULL) or deinit returns, the callback is not called.
 * Requests return true once queued; a failure arrives as an Error event. One
 * discovery and one subscribe run at a time; another one returns false. */

/** Initialize GATT client */
void ble_gatt_client_init(void);

/** Deinitialize GATT client (unregister event handler) */
void ble_gatt_client_deinit(void);

/** Set GATT client event callback for a specific connection.
 *  Each connection can have its own callback and context.
 *  Call with callback=NULL to unregister a connection.
 *
 *  @param connection_handle  BLE connection handle
 *  @param callback           event callback, or NULL to unregister
 *  @param context            user context passed to callback
 */
void ble_gatt_client_set_callback(
    uint16_t connection_handle,
    BleGattClientCallback callback,
    void* context);

/** Discover all primary services on a connected device */
bool ble_gatt_client_discover_services(uint16_t connection_handle);

/** Discover all characteristics within a service */
bool ble_gatt_client_discover_characteristics(
    uint16_t connection_handle,
    const BleGattService* service);

/** Read a characteristic value */
bool ble_gatt_client_read(uint16_t connection_handle, uint16_t value_handle);

/** Write a characteristic value */
bool ble_gatt_client_write(
    uint16_t connection_handle,
    uint16_t value_handle,
    const uint8_t* data,
    uint16_t data_len);

/** Request an MTU exchange. The result is not reported as an event. */
bool ble_gatt_client_exchange_mtu(uint16_t connection_handle);

/** Enable or disable notifications: find the characteristic's CCCD, then
 *  write it. WriteComplete follows. */
bool ble_gatt_client_subscribe_notifications(
    uint16_t connection_handle,
    uint16_t value_handle,
    bool enable);

#ifdef __cplusplus
}
#endif
