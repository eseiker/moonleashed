#include "gatt.h"
#include "event_dispatcher.h"
#include <gatt_host_shim.h>
#include <ble_dispatch.h>
#include <dyn_gatt.h>
#include <nimble_glue.h>
#include <ble/ble.h>

#include <furi.h>

#define TAG "GattChar"

#define GATT_MIN_READ_KEY_SIZE (10)

#ifdef BLE_GATT_STRICT
#define ble_gatt_strict_crash(message) furi_crash(message)
#else
#define ble_gatt_strict_crash(message)
#endif

#define SHIM_HANDLE_FIRST (0x0100)
#define SHIM_REF_MAX      (8)

static const BleGattHostShim* shim = NULL;
static uint16_t shim_hid_handle;
static uint16_t shim_next_handle = SHIM_HANDLE_FIRST;

// Report Reference per characteristic handle, to tell HID reports apart
static struct {
    uint16_t handle;
    uint16_t report_ref;
} shim_refs[SHIM_REF_MAX];

/* Services other than the ones NimBLE serves itself become real NimBLE
 * services. Writes, CCCD changes and indication confirmations come back as the
 * ACI events stock service code parses, on the dispatch thread. */

static struct {
    uint16_t handle; /* 0 = free */
    int svc_id;
} dyn_svcs[DYN_GATT_MAX_SVCS];

static struct {
    BleGattCharacteristicInstance* inst; /* NULL = free */
    int chr_id;
} dyn_chrs[DYN_GATT_MAX_SVCS * DYN_GATT_MAX_CHRS_PER_SVC];

// Guards dyn_svcs and dyn_chrs, and the instances they point to
static FuriMutex* dyn_tables_mutex;

static void dyn_tables_lock(void) {
    furi_mutex_acquire(dyn_tables_mutex, FuriWaitForever);
}

static void dyn_tables_unlock(void) {
    furi_mutex_release(dyn_tables_mutex);
}

#define ACI_EVT_HDR (5U)
_Static_assert(offsetof(aci_gatt_attribute_modified_event_rp0, Attr_Data) == 8, "layout");

static bool shim_service_is_static(uint8_t uuid_type, const Service_UUID_t* uuid) {
    if(uuid_type != UUID_TYPE_16) return false;
    uint16_t u = uuid->Service_UUID_16;
    return u == HUMAN_INTERFACE_DEVICE_SERVICE_UUID || u == BATTERY_SERVICE_UUID ||
           u == DEVICE_INFORMATION_SERVICE_UUID;
}

static void dyn_uuid(DynGattUuid* out, uint8_t uuid_type, const void* uuid) {
    memset(out, 0, sizeof(*out));
    if(uuid_type == UUID_TYPE_16) {
        out->type = 16;
        memcpy(&out->u16, uuid, 2);
    } else {
        out->type = 128;
        memcpy(out->u128, uuid, 16);
    }
}

static int dyn_svc_find(uint16_t svc_handle) {
    int id = -1;
    dyn_tables_lock();
    for(size_t i = 0; i < COUNT_OF(dyn_svcs); i++) {
        if(dyn_svcs[i].handle && dyn_svcs[i].handle == svc_handle) {
            id = dyn_svcs[i].svc_id;
            break;
        }
    }
    dyn_tables_unlock();
    return id;
}

static int dyn_chr_find(const BleGattCharacteristicInstance* inst) {
    int id = -1;
    dyn_tables_lock();
    for(size_t i = 0; i < COUNT_OF(dyn_chrs); i++) {
        if(dyn_chrs[i].inst == inst) {
            id = dyn_chrs[i].chr_id;
            break;
        }
    }
    dyn_tables_unlock();
    return id;
}

static void dyn_deliver(void* blob) {
    ble_event_dispatcher_process_event(blob);
}

// Host thread: wrap a vendor event payload as hci_uart_pckt -> hci_event_pckt
static void dyn_post_aci(uint16_t ecode, const uint8_t* payload, uint16_t len) {
    uint8_t* b = malloc(ACI_EVT_HDR + len);
    b[0] = HCI_EVENT_PKT_TYPE;
    b[1] = HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE;
    b[2] = MIN(255U, 2U + len);
    b[3] = ecode & 0xFF;
    b[4] = ecode >> 8;
    memcpy(b + ACI_EVT_HDR, payload, len);
    ble_dispatch_post(dyn_deliver, b);
}

static void
    dyn_on_write(uint16_t conn_handle, uint16_t attr_handle, const uint8_t* data, uint16_t len) {
    // aci_gatt_attribute_modified_event_rp0
    uint8_t* p = malloc(8U + len);
    p[0] = conn_handle;
    p[1] = conn_handle >> 8;
    p[2] = attr_handle;
    p[3] = attr_handle >> 8;
    p[4] = 0;
    p[5] = 0;
    p[6] = len;
    p[7] = len >> 8;
    memcpy(p + 8, data, len);
    dyn_post_aci(ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE, p, 8U + len);
    free(p);
}

static void dyn_on_indicate_done(uint16_t conn_handle) {
    uint8_t p[2] = {conn_handle, conn_handle >> 8};
    dyn_post_aci(ACI_GATT_SERVER_CONFIRMATION_VSEVT_CODE, p, sizeof(p));
}

// Host thread, after a rebuild: stock code compares Attr_Handle to handle + 1
static void dyn_on_committed(void) {
    dyn_tables_lock();
    for(size_t i = 0; i < COUNT_OF(dyn_chrs); i++) {
        BleGattCharacteristicInstance* inst = dyn_chrs[i].inst;
        uint16_t decl, dsc;
        if(inst && dyn_gatt_char_handles(dyn_chrs[i].chr_id, &decl, &dsc)) {
            inst->handle = decl;
            inst->descriptor_handle = dsc;
        }
    }
    dyn_tables_unlock();
}

static const DynGattHooks dyn_hooks = {
    .on_write = dyn_on_write,
    .on_indicate_done = dyn_on_indicate_done,
    .on_committed = dyn_on_committed,
};

static uint16_t dyn_flags(uint8_t props, uint8_t security) {
    // Broadcast to signed write: same bits in both stacks
    uint16_t f = props & 0x7F;
    if(security & ATTR_PERMISSION_AUTHEN_READ) f |= DYN_GATT_F_READ_ENC | DYN_GATT_F_READ_AUTHEN;
    if(security & ATTR_PERMISSION_ENCRY_READ) f |= DYN_GATT_F_READ_ENC;
    if(security & ATTR_PERMISSION_AUTHOR_READ) f |= DYN_GATT_F_READ_AUTHOR;
    if(security & ATTR_PERMISSION_AUTHEN_WRITE)
        f |= DYN_GATT_F_WRITE_ENC | DYN_GATT_F_WRITE_AUTHEN;
    if(security & ATTR_PERMISSION_ENCRY_WRITE) f |= DYN_GATT_F_WRITE_ENC;
    if(security & ATTR_PERMISSION_AUTHOR_WRITE) f |= DYN_GATT_F_WRITE_AUTHOR;
    return f;
}

void ble_gatt_host_shim_set(const BleGattHostShim* new_shim) {
    // Profile services register event handlers; gap_init normally sets this up
    if(new_shim) ble_event_dispatcher_init();
    if(!dyn_tables_mutex) dyn_tables_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    dyn_gatt_set_hooks(DYN_GATT_OWNER_SHIM, new_shim ? &dyn_hooks : NULL);
    shim = new_shim;
}

bool ble_gatt_host_shim_has_hid(void) {
    return shim_hid_handle != 0;
}

void ble_gatt_host_shim_commit(void) {
    if(shim && dyn_gatt_dirty()) nimble_glue_gatt_rebuild();
}

static uint16_t shim_alloc_handle(void) {
    uint16_t handle = shim_next_handle++;
    if(shim_next_handle == 0) shim_next_handle = SHIM_HANDLE_FIRST;
    return handle;
}

static void shim_ref_set(uint16_t handle, uint16_t report_ref) {
    FURI_CRITICAL_ENTER();
    for(size_t i = 0; i < SHIM_REF_MAX; i++) {
        if(shim_refs[i].handle == 0) {
            shim_refs[i].handle = handle;
            shim_refs[i].report_ref = report_ref;
            break;
        }
    }
    FURI_CRITICAL_EXIT();
}

static uint16_t shim_ref_take(uint16_t handle, bool clear) {
    uint16_t report_ref = 0;
    FURI_CRITICAL_ENTER();
    for(size_t i = 0; i < SHIM_REF_MAX; i++) {
        if(shim_refs[i].handle == handle) {
            report_ref = shim_refs[i].report_ref;
            if(clear) shim_refs[i].handle = 0;
            break;
        }
    }
    FURI_CRITICAL_EXIT();
    return report_ref;
}

static void dyn_characteristic_init(
    int svc_id,
    const BleGattCharacteristicParams* char_descriptor,
    BleGattCharacteristicInstance* char_instance) {
    uint16_t size = 0;
    const uint8_t* init = NULL;
    if(char_descriptor->data_prop_type == FlipperGattCharacteristicDataFixed) {
        size = char_descriptor->data.fixed.length;
        init = char_descriptor->data.fixed.ptr;
    } else {
        char_descriptor->data.callback.fn(char_descriptor->data.callback.context, NULL, &size);
    }

    DynGattUuid uuid;
    dyn_uuid(&uuid, char_descriptor->uuid_type, &char_descriptor->uuid);

    const BleGattCharacteristicDescriptorParams* desc = char_descriptor->descriptor_params;
    DynGattUuid dsc_uuid;
    uint8_t dsc_value[DYN_GATT_DSC_VALUE_MAX];
    uint8_t dsc_len = 0;
    if(desc) {
        uint8_t const* data = NULL;
        uint16_t len = 0;
        bool release_data = desc->data_callback.fn(desc->data_callback.context, &data, &len);
        dyn_uuid(&dsc_uuid, desc->uuid_type, &desc->uuid);
        dsc_len = MIN(len, sizeof(dsc_value));
        if(data) memcpy(dsc_value, data, dsc_len);
        if(release_data) free((void*)data);
    }

    int chr_id = dyn_gatt_char_add(
        svc_id,
        &uuid,
        dyn_flags(char_descriptor->char_properties, char_descriptor->security_permissions),
        size,
        init,
        init ? size : 0,
        desc ? &dsc_uuid : NULL,
        dsc_value,
        dsc_len);

    // Placeholders until the table is rebuilt
    char_instance->handle = shim_alloc_handle();
    char_instance->descriptor_handle = desc ? shim_alloc_handle() : 0;
    if(chr_id < 0) return;
    dyn_tables_lock();
    for(size_t i = 0; i < COUNT_OF(dyn_chrs); i++) {
        if(!dyn_chrs[i].inst) {
            dyn_chrs[i].inst = char_instance;
            dyn_chrs[i].chr_id = chr_id;
            break;
        }
    }
    dyn_tables_unlock();
}

static void shim_characteristic_init(
    uint16_t svc_handle,
    const BleGattCharacteristicParams* char_descriptor,
    BleGattCharacteristicInstance* char_instance) {
    int svc_id = dyn_svc_find(svc_handle);
    if(svc_id >= 0) {
        dyn_characteristic_init(svc_id, char_descriptor, char_instance);
        return;
    }

    char_instance->handle = shim_alloc_handle();
    char_instance->descriptor_handle = 0;

    const BleGattCharacteristicDescriptorParams* desc = char_descriptor->descriptor_params;
    if(!desc) return;
    char_instance->descriptor_handle = shim_alloc_handle();

    // Read it now: the context is only valid while the profile starts
    uint8_t const* data = NULL;
    uint16_t len = 0;
    bool release_data = desc->data_callback.fn(desc->data_callback.context, &data, &len);
    if(desc->uuid_type == UUID_TYPE_16 &&
       desc->uuid.Char_UUID_16 == REPORT_REFERENCE_DESCRIPTOR_UUID && data && len == 2) {
        shim_ref_set(char_instance->handle, data[0] | (data[1] << 8));
    }
    if(release_data) free((void*)data);
}

void ble_gatt_characteristic_init(
    uint16_t svc_handle,
    const BleGattCharacteristicParams* char_descriptor,
    BleGattCharacteristicInstance* char_instance) {
    furi_check(char_descriptor);
    furi_check(char_instance);

    // Copy the descriptor to the instance, since it may point to stack memory
    char_instance->characteristic = malloc(sizeof(BleGattCharacteristicParams));
    memcpy(
        (void*)char_instance->characteristic,
        char_descriptor,
        sizeof(BleGattCharacteristicParams));

    if(shim) {
        shim_characteristic_init(svc_handle, char_descriptor, char_instance);
        return;
    }

    uint16_t char_data_size = 0;
    if(char_descriptor->data_prop_type == FlipperGattCharacteristicDataFixed) {
        char_data_size = char_descriptor->data.fixed.length;
    } else if(char_descriptor->data_prop_type == FlipperGattCharacteristicDataCallback) {
        char_descriptor->data.callback.fn(
            char_descriptor->data.callback.context, NULL, &char_data_size);
    }

    tBleStatus status = aci_gatt_add_char(
        svc_handle,
        char_descriptor->uuid_type,
        &char_descriptor->uuid,
        char_data_size,
        char_descriptor->char_properties,
        char_descriptor->security_permissions,
        char_descriptor->gatt_evt_mask,
        GATT_MIN_READ_KEY_SIZE,
        char_descriptor->is_variable,
        &char_instance->handle);
    if(status) {
        FURI_LOG_E(TAG, "Failed to add %s char: %d", char_descriptor->name, status);
        ble_gatt_strict_crash("Failed to add characteristic");
    }

    char_instance->descriptor_handle = 0;
    if((status == 0) && char_descriptor->descriptor_params) {
        uint8_t const* char_data = NULL;
        const BleGattCharacteristicDescriptorParams* char_data_descriptor =
            char_descriptor->descriptor_params;
        bool release_data = char_data_descriptor->data_callback.fn(
            char_data_descriptor->data_callback.context, &char_data, &char_data_size);

        status = aci_gatt_add_char_desc(
            svc_handle,
            char_instance->handle,
            char_data_descriptor->uuid_type,
            &char_data_descriptor->uuid,
            char_data_descriptor->max_length,
            char_data_size,
            char_data,
            char_data_descriptor->security_permissions,
            char_data_descriptor->access_permissions,
            char_data_descriptor->gatt_evt_mask,
            GATT_MIN_READ_KEY_SIZE,
            char_data_descriptor->is_variable,
            &char_instance->descriptor_handle);
        if(status) {
            FURI_LOG_E(TAG, "Failed to add %s char descriptor: %d", char_descriptor->name, status);
            ble_gatt_strict_crash("Failed to add characteristic descriptor");
        }
        if(release_data) {
            free((void*)char_data);
        }
    }
}

void ble_gatt_characteristic_delete(
    uint16_t svc_handle,
    BleGattCharacteristicInstance* char_instance) {
    if(shim) {
        int chr_id = dyn_chr_find(char_instance);
        if(chr_id >= 0) {
            dyn_gatt_char_remove(chr_id);
            dyn_tables_lock();
            for(size_t i = 0; i < COUNT_OF(dyn_chrs); i++) {
                if(dyn_chrs[i].inst == char_instance) dyn_chrs[i].inst = NULL;
            }
            dyn_tables_unlock();
        } else {
            shim_ref_take(char_instance->handle, true);
        }
        free((void*)char_instance->characteristic);
        return;
    }

    tBleStatus status = aci_gatt_del_char(svc_handle, char_instance->handle);
    if(status) {
        FURI_LOG_E(
            TAG, "Failed to delete %s char: %d", char_instance->characteristic->name, status);
        ble_gatt_strict_crash("Failed to delete characteristic");
    }
    free((void*)char_instance->characteristic);
}

bool ble_gatt_characteristic_update(
    uint16_t svc_handle,
    BleGattCharacteristicInstance* char_instance,
    const void* source) {
    furi_check(char_instance);
    const BleGattCharacteristicParams* char_descriptor = char_instance->characteristic;
    FURI_LOG_D(TAG, "Updating %s char", char_descriptor->name);

    const uint8_t* char_data = NULL;
    uint16_t char_data_size = 0;
    bool release_data = false;
    if(char_descriptor->data_prop_type == FlipperGattCharacteristicDataFixed) {
        char_data = char_descriptor->data.fixed.ptr;
        if(source) {
            char_data = (uint8_t*)source;
        }
        char_data_size = char_descriptor->data.fixed.length;
    } else if(char_descriptor->data_prop_type == FlipperGattCharacteristicDataCallback) {
        const void* context = char_descriptor->data.callback.context;
        if(source) {
            context = source;
        }
        release_data = char_descriptor->data.callback.fn(context, &char_data, &char_data_size);
    }

    int dyn_chr_id = shim ? dyn_chr_find(char_instance) : -1;
    if(dyn_chr_id >= 0) {
        bool ok = dyn_gatt_char_set_value(dyn_chr_id, char_data, char_data_size);
        if(release_data) {
            free((void*)char_data);
        }
        return !ok;
    }

    if(shim) {
        uint16_t uuid16 =
            char_descriptor->uuid_type == UUID_TYPE_16 ? char_descriptor->uuid.Char_UUID_16 : 0;
        shim->on_update(
            uuid16,
            shim_ref_take(char_instance->handle, false),
            char_data,
            char_data_size,
            shim->context);
        if(release_data) {
            free((void*)char_data);
        }
        return false;
    }

    tBleStatus result;
    size_t retries_left = 1000;
    do {
        retries_left--;
        result = aci_gatt_update_char_value(
            svc_handle, char_instance->handle, 0, char_data_size, char_data);
        if(result == BLE_STATUS_INSUFFICIENT_RESOURCES) {
            FURI_LOG_W(TAG, "Insufficient resources for %s characteristic", char_descriptor->name);
            furi_delay_ms(1);
        }
    } while(result == BLE_STATUS_INSUFFICIENT_RESOURCES && retries_left);

    if(release_data) {
        free((void*)char_data);
    }

    if(result != BLE_STATUS_SUCCESS) {
        FURI_LOG_E(TAG, "Failed updating %s characteristic: %d", char_descriptor->name, result);
        ble_gatt_strict_crash("Failed to update characteristic");
    }

    return result != BLE_STATUS_SUCCESS;
}

bool ble_gatt_service_add(
    uint8_t Service_UUID_Type,
    const Service_UUID_t* Service_UUID,
    uint8_t Service_Type,
    uint8_t Max_Attribute_Records,
    uint16_t* Service_Handle) {
    if(shim) {
        *Service_Handle = shim_alloc_handle();
        if(Service_UUID_Type == UUID_TYPE_16 &&
           Service_UUID->Service_UUID_16 == HUMAN_INTERFACE_DEVICE_SERVICE_UUID) {
            shim_hid_handle = *Service_Handle;
        }
        if(shim_service_is_static(Service_UUID_Type, Service_UUID)) return true;

        DynGattUuid uuid;
        dyn_uuid(&uuid, Service_UUID_Type, Service_UUID);
        int svc_id =
            dyn_gatt_service_add(&uuid, Service_Type != SECONDARY_SERVICE, DYN_GATT_OWNER_SHIM);
        if(svc_id < 0) return false;
        dyn_tables_lock();
        for(size_t i = 0; i < COUNT_OF(dyn_svcs); i++) {
            if(!dyn_svcs[i].handle) {
                dyn_svcs[i].handle = *Service_Handle;
                dyn_svcs[i].svc_id = svc_id;
                break;
            }
        }
        dyn_tables_unlock();
        return true;
    }

    tBleStatus result = aci_gatt_add_service(
        Service_UUID_Type, Service_UUID, Service_Type, Max_Attribute_Records, Service_Handle);
    if(result) {
        FURI_LOG_E(TAG, "Failed to add service: %x", result);
        ble_gatt_strict_crash("Failed to add service");
    }

    return result == BLE_STATUS_SUCCESS;
}

bool ble_gatt_service_delete(uint16_t svc_handle) {
    if(shim) {
        if(svc_handle == shim_hid_handle) shim_hid_handle = 0;
        int svc_id = dyn_svc_find(svc_handle);
        if(svc_id < 0) return true;
        dyn_gatt_service_remove(svc_id);
        dyn_tables_lock();
        for(size_t i = 0; i < COUNT_OF(dyn_svcs); i++) {
            if(dyn_svcs[i].handle == svc_handle) dyn_svcs[i].handle = 0;
        }
        for(size_t i = 0; i < COUNT_OF(dyn_chrs); i++) {
            if(dyn_chrs[i].inst && dyn_gatt_char_service(dyn_chrs[i].chr_id) == svc_id)
                dyn_chrs[i].inst = NULL;
        }
        dyn_tables_unlock();
        return true;
    }

    tBleStatus result = aci_gatt_del_service(svc_handle);
    if(result) {
        FURI_LOG_E(TAG, "Failed to delete service: %x", result);
        ble_gatt_strict_crash("Failed to delete service");
    }

    return result == BLE_STATUS_SUCCESS;
}
