#include <furi.h>
#include <storage/storage.h>

#include "host/ble_store.h"

#include "bond_store.h"

#define TAG "BondStore"

#define STORE_DIR         EXT_PATH("apps_data/nimble_host")
#define STORE_PATH        STORE_DIR "/bonds.bin"
#define STORE_MAGIC       0x3153424Eu /* "NBS1" */
/* Records are raw union ble_store_value: a NimBLE update may change them. */
#define STORE_RECORD_SIZE ((uint32_t)sizeof(union ble_store_value))

/* Set once the file was read, or found missing, on a mounted card. */
static bool loaded;

static int save_cb(int obj_type, union ble_store_value* val, void* cookie) {
    File* f = cookie;
    uint8_t t = (uint8_t)obj_type;
    if(storage_file_write(f, &t, sizeof(t)) != sizeof(t)) return 1;
    if(storage_file_write(f, val, sizeof(*val)) != sizeof(*val)) return 1;
    return 0;
}

void bond_store_save(void) {
    /* Saving before the load would overwrite the file with fewer bonds. */
    if(!loaded) bond_store_load();
    if(!loaded) return;
    Storage* storage = furi_record_open(RECORD_STORAGE);
    storage_common_mkdir(storage, STORE_DIR);
    File* f = storage_file_alloc(storage);
    if(storage_file_open(f, STORE_PATH, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        uint32_t header[2] = {STORE_MAGIC, STORE_RECORD_SIZE};
        storage_file_write(f, header, sizeof(header));
        ble_store_iterate(BLE_STORE_OBJ_TYPE_OUR_SEC, save_cb, f);
        ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, save_cb, f);
        ble_store_iterate(BLE_STORE_OBJ_TYPE_CCCD, save_cb, f);
    } else {
        FURI_LOG_W(TAG, "Cannot write %s", STORE_PATH);
    }
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
}

void bond_store_load(void) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    if(storage_sd_status(storage) != FSE_OK) {
        furi_record_close(RECORD_STORAGE);
        return;
    }
    File* f = storage_file_alloc(storage);
    uint32_t header[2] = {0};
    if(storage_file_open(f, STORE_PATH, FSAM_READ, FSOM_OPEN_EXISTING)) {
        if(storage_file_read(f, header, sizeof(header)) == sizeof(header) &&
           header[0] == STORE_MAGIC && header[1] == STORE_RECORD_SIZE) {
            uint8_t t;
            union ble_store_value val;
            union ble_store_value current;
            union ble_store_key key;
            while(storage_file_read(f, &t, sizeof(t)) == sizeof(t) &&
                  storage_file_read(f, &val, sizeof(val)) == sizeof(val)) {
                /* A late load must not replace a bond made since boot. */
                ble_store_key_from_value(t, &key, &val);
                if(ble_store_read(t, &key, &current) != 0) ble_store_write(t, &val);
            }
        }
        loaded = true;
    } else if(storage_file_get_error(f) == FSE_NOT_EXIST) {
        loaded = true;
    }
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
}

void bond_store_forget(void) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    /* RAM is the truth now: the next save replaces whatever the file held. */
    storage_common_remove(storage, STORE_PATH);
    loaded = true;
    furi_record_close(RECORD_STORAGE);
}
