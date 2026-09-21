#include "gatt.h"
#include "event_dispatcher.h"
#include <gatt_host_shim.h>
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

void ble_gatt_host_shim_set(const BleGattHostShim* new_shim) {
    // Profile services register event handlers; gap_init normally sets this up
    if(new_shim) ble_event_dispatcher_init();
    shim = new_shim;
}

bool ble_gatt_host_shim_has_hid(void) {
    return shim_hid_handle != 0;
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

static void shim_characteristic_init(
    const BleGattCharacteristicParams* char_descriptor,
    BleGattCharacteristicInstance* char_instance) {
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
        shim_characteristic_init(char_descriptor, char_instance);
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
        shim_ref_take(char_instance->handle, true);
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
        return true;
    }

    tBleStatus result = aci_gatt_del_service(svc_handle);
    if(result) {
        FURI_LOG_E(TAG, "Failed to delete service: %x", result);
        ble_gatt_strict_crash("Failed to delete service");
    }

    return result == BLE_STATUS_SUCCESS;
}
