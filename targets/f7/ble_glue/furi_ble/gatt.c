#include "gatt.h"
#include <gatt_host_shim.h>
#include "event_dispatcher.h"
#include <ble/ble.h>

#include <furi.h>

#define TAG "GattChar"

#define GATT_MIN_READ_KEY_SIZE (10)

#ifdef BLE_GATT_STRICT
#define ble_gatt_strict_crash(message) furi_crash(message)
#else
#define ble_gatt_strict_crash(message)
#endif

/* ---- Host shim (see gatt_host_shim.h) ------------------------------------
 * While s_shim is set, every primitive below skips the CPU2 ACI call. Handles
 * are synthetic and only need to be unique per instance; the Report Reference
 * table lets an update be attributed to a specific HID report. */

#define GATT_SHIM_HANDLE_FIRST (0x0100)
#define GATT_SHIM_REF_MAX      (8)

static const BleGattHostShim* s_shim = NULL;
static uint16_t s_shim_next_handle = GATT_SHIM_HANDLE_FIRST;

static struct {
    uint16_t handle; /* 0 = free slot */
    uint16_t report_ref;
} s_shim_refs[GATT_SHIM_REF_MAX];

void ble_gatt_host_shim_set(const BleGattHostShim* shim) {
    if(shim) {
        /* Service code (e.g. ble_svc_hid_start in a FAP) registers an event
         * handler; the dispatcher init normally happens in gap_init, which never
         * runs without the CPU2 host. Idempotent. */
        ble_event_dispatcher_init();
    }
    s_shim = shim;
}

bool ble_gatt_host_shim_active(void) {
    return s_shim != NULL;
}

static uint16_t ble_gatt_shim_alloc_handle(void) {
    uint16_t handle = s_shim_next_handle++;
    if(s_shim_next_handle == 0) s_shim_next_handle = GATT_SHIM_HANDLE_FIRST;
    return handle;
}

static void ble_gatt_shim_ref_set(uint16_t handle, uint16_t report_ref) {
    FURI_CRITICAL_ENTER();
    for(size_t i = 0; i < GATT_SHIM_REF_MAX; i++) {
        if(s_shim_refs[i].handle == 0) {
            s_shim_refs[i].handle = handle;
            s_shim_refs[i].report_ref = report_ref;
            break;
        }
    }
    FURI_CRITICAL_EXIT();
}

static void ble_gatt_shim_ref_clear(uint16_t handle) {
    FURI_CRITICAL_ENTER();
    for(size_t i = 0; i < GATT_SHIM_REF_MAX; i++) {
        if(s_shim_refs[i].handle == handle) {
            s_shim_refs[i].handle = 0;
            s_shim_refs[i].report_ref = 0;
        }
    }
    FURI_CRITICAL_EXIT();
}

static uint16_t ble_gatt_shim_ref_get(uint16_t handle) {
    uint16_t report_ref = 0;
    FURI_CRITICAL_ENTER();
    for(size_t i = 0; i < GATT_SHIM_REF_MAX; i++) {
        if(s_shim_refs[i].handle == handle) {
            report_ref = s_shim_refs[i].report_ref;
            break;
        }
    }
    FURI_CRITICAL_EXIT();
    return report_ref;
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

    uint16_t char_data_size = 0;
    if(char_descriptor->data_prop_type == FlipperGattCharacteristicDataFixed) {
        char_data_size = char_descriptor->data.fixed.length;
    } else if(char_descriptor->data_prop_type == FlipperGattCharacteristicDataCallback) {
        char_descriptor->data.callback.fn(
            char_descriptor->data.callback.context, NULL, &char_data_size);
    }

    if(s_shim) {
        char_instance->handle = ble_gatt_shim_alloc_handle();
        char_instance->descriptor_handle = 0;
        if(char_descriptor->descriptor_params) {
            /* Read the descriptor value now: for HID reports it is the Report
             * Reference {report_id, report_type}, and the FAP's context pointer
             * is only valid during its ble_svc_hid_start. */
            const BleGattCharacteristicDescriptorParams* desc = char_descriptor->descriptor_params;
            uint8_t const* desc_data = NULL;
            uint16_t desc_len = 0;
            bool release_data = desc->data_callback.fn(desc->data_callback.context, &desc_data, &desc_len);
            if(desc->uuid_type == UUID_TYPE_16 &&
               desc->uuid.Char_UUID_16 == REPORT_REFERENCE_DESCRIPTOR_UUID && desc_data &&
               desc_len == 2) {
                ble_gatt_shim_ref_set(
                    char_instance->handle, (uint16_t)desc_data[0] | ((uint16_t)desc_data[1] << 8));
            }
            if(release_data) {
                free((void*)desc_data);
            }
            char_instance->descriptor_handle = ble_gatt_shim_alloc_handle();
        }
        FURI_LOG_D(
            TAG, "shim: %s char -> handle %u", char_descriptor->name, char_instance->handle);
        return;
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
    if(s_shim) {
        ble_gatt_shim_ref_clear(char_instance->handle);
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

    if(s_shim) {
        /* Hand the value to the resident host instead of CPU2. Mirrors the
         * stock return convention below: false means success. */
        if(s_shim->on_update) {
            uint16_t uuid16 =
                (char_descriptor->uuid_type == UUID_TYPE_16) ? char_descriptor->uuid.Char_UUID_16 : 0;
            s_shim->on_update(
                uuid16,
                ble_gatt_shim_ref_get(char_instance->handle),
                char_data,
                char_data_size,
                s_shim->context);
        }
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
    if(s_shim) {
        *Service_Handle = ble_gatt_shim_alloc_handle();
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
    if(s_shim) {
        return true;
    }

    tBleStatus result = aci_gatt_del_service(svc_handle);
    if(result) {
        FURI_LOG_E(TAG, "Failed to delete service: %x", result);
        ble_gatt_strict_crash("Failed to delete service");
    }

    return result == BLE_STATUS_SUCCESS;
}
