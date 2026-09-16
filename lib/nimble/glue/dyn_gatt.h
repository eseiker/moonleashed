/*
 * Dynamic GATT services on the resident NimBLE host (TASK-632).
 *
 * The stock BLE API lets app code define GATT services at runtime:
 * ble_gatt_service_add + ble_gatt_characteristic_init/update/delete
 * (targets/f7/ble_glue/furi_ble/gatt.h). NimBLE fixes its attribute table when
 * ble_gatts_start() runs, so this registry collects those definitions and the
 * host re-registers them when nimble_glue rebuilds the table
 * (nimble_glue_gatt_rebuild_request): ble_gatts_reset -> built-in services ->
 * dyn_gatt_register_all -> ble_gatts_start -> dyn_gatt_after_start.
 *
 * Dynamic services are registered after the built-in ones, so built-in handles
 * (and bonded peers' cached handles and CCCDs) stay the same.
 *
 * Handle layout matches the ST stack the stock API was written for:
 * declaration H, value H+1, CCCD H+2 when notify/indicate is set, then the
 * optional descriptor. Stock service code that compares Attr_Handle against
 * char.handle + 1 / + 2 therefore works unchanged.
 *
 * Plain C, no NimBLE types: the firmware includes this header.
 */

#ifndef DYN_GATT_H_
#define DYN_GATT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DYN_GATT_MAX_SVCS          4
#define DYN_GATT_MAX_CHRS_PER_SVC  8
#define DYN_GATT_VALUE_MAX         512
#define DYN_GATT_DSC_VALUE_MAX     32

/* Characteristic flags. Values equal NimBLE's BLE_GATT_CHR_F_* (checked by a
 * static assert in dyn_gatt.c). */
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
    uint8_t u128[16]; /* little-endian, as on air */
} DynGattUuid;

typedef enum {
    DynGattWriteValue, /* attr_handle = value handle (declaration + 1) */
    DynGattWriteCccd, /* attr_handle = CCCD handle (declaration + 2), data = 2-byte CCCD */
} DynGattWriteKind;

/* Hooks run on the NimBLE host thread. Keep them short; post work elsewhere. */
typedef struct {
    void (*on_write)(
        int chr_id,
        DynGattWriteKind kind,
        uint16_t conn_handle,
        uint16_t attr_handle,
        const uint8_t* data,
        uint16_t len,
        void* ctx);
    void (*on_indicate_done)(int chr_id, uint16_t conn_handle, void* ctx);
    /* After a rebuild: handles from dyn_gatt_char_handles are valid now. */
    void (*on_committed)(void* ctx);
    void* ctx;
} DynGattHooks;

void dyn_gatt_set_hooks(const DynGattHooks* hooks);

/* Definition API (any thread). Returns an id >= 0, or -1 when full/invalid. */
int dyn_gatt_service_add(const DynGattUuid* uuid, bool primary);
bool dyn_gatt_service_remove(int svc_id);
int dyn_gatt_char_add(
    int svc_id,
    const DynGattUuid* uuid,
    uint16_t flags,
    uint16_t max_len,
    const uint8_t* init_value,
    uint16_t init_len,
    const DynGattUuid* dsc_uuid, /* NULL = no descriptor */
    const uint8_t* dsc_value,
    uint8_t dsc_len);
bool dyn_gatt_char_remove(int chr_id);

/* Store a new value; if the table is live, notify/indicate subscribed peers. */
bool dyn_gatt_char_set_value(int chr_id, const uint8_t* data, uint16_t len);

/* Declaration and descriptor handles (0 when absent). Valid after commit. */
bool dyn_gatt_char_handles(int chr_id, uint16_t* decl_handle, uint16_t* dsc_handle);

/* True when definitions changed since the last rebuild. */
bool dyn_gatt_dirty(void);

/* --- nimble_glue internal ---------------------------------------------------- */
void dyn_gatt_init(void);
/* After ble_gatts_reset, before ble_gatts_start (host thread). */
void dyn_gatt_register_all(void);
/* After a successful ble_gatts_start (host thread). */
void dyn_gatt_after_start(void);
/* GAP event forwarding (host thread). */
void dyn_gatt_on_subscribe(uint16_t conn_handle, uint16_t attr_handle, bool notify, bool indicate);
void dyn_gatt_on_notify_tx(uint16_t conn_handle, uint16_t attr_handle, int status, bool indication);

#ifdef __cplusplus
}
#endif

#endif /* DYN_GATT_H_ */
