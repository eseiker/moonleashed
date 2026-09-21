/* GATT services defined at runtime. NimBLE fixes its table at ble_gatts_start,
 * so definitions collect here and go live when nimble_glue rebuilds the table.
 *
 * Handles follow the ST layout the stock API assumes: declaration H, value
 * H+1, CCCD H+2 with notify or indicate, then the optional descriptor.
 * Plain C: the firmware includes this header. */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DYN_GATT_MAX_SVCS         4
#define DYN_GATT_MAX_CHRS_PER_SVC 8
#define DYN_GATT_VALUE_MAX        512
#define DYN_GATT_DSC_VALUE_MAX    32

/* Same values as NimBLE's BLE_GATT_CHR_F_* */
#define DYN_GATT_F_BROADCAST       0x0001
#define DYN_GATT_F_READ            0x0002
#define DYN_GATT_F_WRITE_NO_RSP    0x0004
#define DYN_GATT_F_WRITE           0x0008
#define DYN_GATT_F_NOTIFY          0x0010
#define DYN_GATT_F_INDICATE        0x0020
#define DYN_GATT_F_AUTH_SIGN_WRITE 0x0040
#define DYN_GATT_F_READ_ENC        0x0200
#define DYN_GATT_F_READ_AUTHEN     0x0400
#define DYN_GATT_F_READ_AUTHOR     0x0800
#define DYN_GATT_F_WRITE_ENC       0x1000
#define DYN_GATT_F_WRITE_AUTHEN    0x2000
#define DYN_GATT_F_WRITE_AUTHOR    0x4000

typedef struct {
    uint8_t type; /* 16 or 128 */
    uint16_t u16;
    uint8_t u128[16]; /* little-endian */
} DynGattUuid;

/* Run on the host thread; keep them short. */
typedef struct {
    /* attr_handle is the value handle, or the CCCD handle with a 2-byte CCCD */
    void (*on_write)(uint16_t conn_handle, uint16_t attr_handle, const uint8_t* data, uint16_t len);
    void (*on_indicate_done)(uint16_t conn_handle);
    /* After a rebuild: dyn_gatt_char_handles is valid */
    void (*on_committed)(void);
} DynGattHooks;

/* A service's events go to its owner's hooks */
#define DYN_GATT_OWNER_SHIM   0 /* the stock ble_gatt_* API */
#define DYN_GATT_OWNER_SERVER 1 /* ble_gatt_server_* */
#define DYN_GATT_OWNERS       2

void dyn_gatt_set_hooks(uint8_t owner, const DynGattHooks* hooks);

/* Any thread. Return an id >= 0, or -1. */
int dyn_gatt_service_add(const DynGattUuid* uuid, bool primary, uint8_t owner);
bool dyn_gatt_service_remove(int svc_id);
int dyn_gatt_char_add(
    int svc_id,
    const DynGattUuid* uuid,
    uint16_t flags,
    uint16_t max_len,
    const uint8_t* init_value,
    uint16_t init_len,
    const DynGattUuid* dsc_uuid, /* NULL: no descriptor */
    const uint8_t* dsc_value,
    uint8_t dsc_len);
bool dyn_gatt_char_remove(int chr_id);
int dyn_gatt_char_service(int chr_id);

/* Stores the value and notifies or indicates subscribers, on the host
 * thread and in call order. */
bool dyn_gatt_char_set_value(int chr_id, const uint8_t* data, uint16_t len);
bool dyn_gatt_char_handles(int chr_id, uint16_t* decl_handle, uint16_t* dsc_handle);

/* Definitions changed since the last rebuild. */
bool dyn_gatt_dirty(void);

/* nimble_glue, host thread */
void dyn_gatt_init(void);
void dyn_gatt_register_all(void);
void dyn_gatt_after_start(void);
void dyn_gatt_on_subscribe(uint16_t conn_handle, uint16_t attr_handle, bool notify, bool indicate);
void dyn_gatt_on_notify_tx(uint16_t conn_handle, uint16_t attr_handle, int status, bool indication);

#ifdef __cplusplus
}
#endif
