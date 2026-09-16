/*
 * Bond persistence for the NimBLE host FAP (TASK-589 follow-up #3).
 * See serial_store.h.
 */

#include <furi.h>
#include <storage/storage.h>

#include "host/ble_store.h"

#include "serial_store.h"

#define TAG        "SerialStore"
#define STORE_DIR  "/ext/apps_data/nimble_host"
#define STORE_PATH STORE_DIR "/bonds.bin"
#define STORE_MAGIC 0x3153424Eu /* "NBS1" */

static int save_cb(int obj_type, union ble_store_value* val, void* cookie) {
    File* f = cookie;
    uint8_t t = (uint8_t)obj_type;
    if(storage_file_write(f, &t, sizeof(t)) != sizeof(t)) return 1;
    if(storage_file_write(f, val, sizeof(*val)) != sizeof(*val)) return 1;
    return 0; /* continue iterating */
}

void serial_store_save(void) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    storage_common_mkdir(storage, STORE_DIR);
    File* f = storage_file_alloc(storage);
    if(storage_file_open(f, STORE_PATH, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        uint32_t magic = STORE_MAGIC;
        storage_file_write(f, &magic, sizeof(magic));
        ble_store_iterate(BLE_STORE_OBJ_TYPE_OUR_SEC, save_cb, f);
        ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, save_cb, f);
        ble_store_iterate(BLE_STORE_OBJ_TYPE_CCCD, save_cb, f);
        storage_file_close(f);
        FURI_LOG_I(TAG, "Bonds saved");
    } else {
        FURI_LOG_W(TAG, "Could not open %s for write", STORE_PATH);
    }
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
}

void serial_store_load(void) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    File* f = storage_file_alloc(storage);
    if(storage_file_open(f, STORE_PATH, FSAM_READ, FSOM_OPEN_EXISTING)) {
        uint32_t magic = 0;
        if(storage_file_read(f, &magic, sizeof(magic)) == sizeof(magic) && magic == STORE_MAGIC) {
            unsigned restored = 0;
            for(;;) {
                uint8_t t = 0;
                if(storage_file_read(f, &t, sizeof(t)) != sizeof(t)) break;
                union ble_store_value val;
                if(storage_file_read(f, &val, sizeof(val)) != sizeof(val)) break;
                if(ble_store_write((int)t, &val) == 0) restored++;
            }
            FURI_LOG_I(TAG, "Bonds restored: %u", restored);
        }
        storage_file_close(f);
    }
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
}

void serial_store_forget(void) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    FS_Error err = storage_common_remove(storage, STORE_PATH);
    FURI_LOG_I(TAG, "Forget bonds (remove %s): %s", STORE_PATH, storage_error_get_desc(err));
    furi_record_close(RECORD_STORAGE);
}
