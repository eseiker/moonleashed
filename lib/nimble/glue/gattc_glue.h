/*
 * Neutral, plain-C GATT client core (TASK-633). The firmware furi_ble layer
 * builds the Moon-Firmware-compatible ble_gatt_client_* API on this (KNOW-636).
 * It wraps NimBLE's ble_gattc_* procedures, accumulates discovery results, and
 * reports typed events through one registered dispatcher. Safe to include from
 * the firmware: no NimBLE types leak through this header.
 */

#ifndef GATTC_GLUE_H_
#define GATTC_GLUE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GATTC_MAX_SERVICES 16
#define GATTC_MAX_CHARS    64

typedef struct {
    uint8_t uuid_type; /* 1 = 16-bit, 2 = 128-bit */
    uint16_t uuid_16;
    uint8_t uuid_128[16];
    uint16_t start_handle;
    uint16_t end_handle;
} GattcApiService;

typedef struct {
    uint8_t uuid_type;
    uint16_t uuid_16;
    uint8_t uuid_128[16];
    uint16_t decl_handle;
    uint16_t value_handle;
    uint8_t properties;
} GattcApiChar;

typedef enum {
    GattcApiServicesDiscovered, /* services[] + count valid */
    GattcApiCharsDiscovered, /* chars[] + count valid */
    GattcApiReadComplete, /* data/data_len + value_handle valid */
    GattcApiWriteComplete, /* value_handle valid */
    GattcApiNotification, /* data/data_len + value_handle valid */
    GattcApiError, /* error_code valid */
} GattcApiEventType;

typedef struct {
    GattcApiEventType type;
    uint16_t conn_handle;
    const GattcApiService* services; /* ServicesDiscovered; valid during callback */
    const GattcApiChar* chars; /* CharsDiscovered; valid during callback */
    uint8_t count;
    const uint8_t* data; /* Read/Notification; valid during callback */
    uint16_t data_len;
    uint16_t value_handle;
    uint8_t error_code;
} GattcApiEvent;

typedef void (*GattcApiCallback)(const GattcApiEvent* event, void* context);

void gattc_api_init(GattcApiCallback dispatch, void* context);
void gattc_api_deinit(void);

bool gattc_api_discover_services(uint16_t conn_handle);
bool gattc_api_discover_characteristics(
    uint16_t conn_handle,
    uint16_t start_handle,
    uint16_t end_handle);
bool gattc_api_read(uint16_t conn_handle, uint16_t value_handle);
bool gattc_api_write(uint16_t conn_handle, uint16_t value_handle, const uint8_t* data, uint16_t len);
bool gattc_api_exchange_mtu(uint16_t conn_handle);
/* Enable/disable notifications by writing the CCCD at value_handle+1. */
bool gattc_api_subscribe(uint16_t conn_handle, uint16_t value_handle, bool enable);

/* Feed an inbound notification/indication (from the central link's GAP handler)
 * so it reaches the dispatcher as GattcApiNotification. */
void gattc_api_on_notify(uint16_t conn_handle, uint16_t attr_handle, const uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* GATTC_GLUE_H_ */
