#include "gatt.h"
#include <gatt_host_shim.h>
#include <ble_dispatch.h>
#include <dyn_gatt.h>
#include <nimble_glue.h>
#include "event_dispatcher.h"
#include <ble/ble.h>
#include <interface/patterns/ble_thread/tl/tl.h>

#include <furi.h>
#include <stddef.h>

#define TAG "GattChar"

#define GATT_MIN_READ_KEY_SIZE (10)

#ifdef BLE_GATT_STRICT
#define ble_gatt_strict_crash(message) furi_crash(message)
#else
#define ble_gatt_strict_crash(message)
#endif

/* The shim is always installed under the resident NimBLE host, so the branches
 * that used to talk to a CPU2 GATT server are unreachable. This fork ships the
 * HCILayer radio, a bare controller with no host, so there is nothing to talk
 * to; the ACI paths were removed with their command builders (TASK-696). */
static void ble_gatt_no_host(const char* what) {
    FURI_LOG_E(TAG, "no GATT server on CPU2 for %s: the shim is not installed", what);
    ble_gatt_strict_crash("GATT shim not installed");
}

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

/* ---- Dynamic services under the shim (TASK-632) ----------------------------
 * Services the resident NimBLE host already serves statically (HID, Battery,
 * Device Information) keep the legacy mapping above: synthetic handles plus
 * on_update. Any other service a caller adds through ble_gatt_service_add
 * becomes a real NimBLE GATT service via dyn_gatt, and goes live when
 * ble_gatt_host_shim_commit() rebuilds the table. Writes, CCCD changes and
 * indication confirmations are turned back into the ACI vendor events stock
 * service code already parses (ACI_GATT_ATTRIBUTE_MODIFIED /
 * ACI_GATT_SERVER_CONFIRMATION) and delivered through ble_event_dispatcher on
 * the BLE dispatch thread. */

#define GATT_DYN_SVC_MAX (DYN_GATT_MAX_SVCS)
#define GATT_DYN_CHR_MAX (DYN_GATT_MAX_SVCS * DYN_GATT_MAX_CHRS_PER_SVC)

static struct {
    uint16_t handle; /* 0 = free */
    int svc_id;
} s_dyn_svcs[GATT_DYN_SVC_MAX];

static struct {
    BleGattCharacteristicInstance* inst; /* NULL = free */
    int chr_id;
} s_dyn_chrs[GATT_DYN_CHR_MAX];

/* Raw layout of a vendor event as hci_uart_pckt -> hci_event_pckt ->
 * evt_blecore_aci -> payload, so the handlers' casts line up. */
_Static_assert(offsetof(hci_uart_pckt, data) == 1, "hci_uart_pckt layout");
_Static_assert(offsetof(hci_event_pckt, data) == 2, "hci_event_pckt layout");
_Static_assert(offsetof(evt_blecore_aci, data) == 2, "evt_blecore_aci layout");
_Static_assert(
    offsetof(aci_gatt_attribute_modified_event_rp0, Attr_Data) == 8,
    "attr modified layout");
#define GATT_ACI_EVT_HDR (5U)

static bool gatt_service_is_nimble_static(uint8_t uuid_type, const Service_UUID_t* uuid) {
    if(uuid_type != UUID_TYPE_16) return false;
    uint16_t u = uuid->Service_UUID_16;
    return u == 0x1812 /* HID */ || u == 0x180F /* Battery */ || u == 0x180A /* DIS */;
}

static int gatt_dyn_svc_find(uint16_t svc_handle) {
    int id = -1;
    FURI_CRITICAL_ENTER();
    for(size_t i = 0; i < GATT_DYN_SVC_MAX; i++) {
        if(s_dyn_svcs[i].handle && s_dyn_svcs[i].handle == svc_handle) {
            id = s_dyn_svcs[i].svc_id;
            break;
        }
    }
    FURI_CRITICAL_EXIT();
    return id;
}

static int gatt_dyn_chr_find(const BleGattCharacteristicInstance* inst) {
    int id = -1;
    FURI_CRITICAL_ENTER();
    for(size_t i = 0; i < GATT_DYN_CHR_MAX; i++) {
        if(s_dyn_chrs[i].inst == inst) {
            id = s_dyn_chrs[i].chr_id;
            break;
        }
    }
    FURI_CRITICAL_EXIT();
    return id;
}

static void gatt_dyn_deliver(void* blob) {
    ble_event_dispatcher_process_event(blob);
}

/* Host thread: wrap an ACI vendor event payload and post it for delivery. */
static void gatt_dyn_post_aci(uint16_t ecode, const uint8_t* payload, uint16_t len) {
    uint8_t* b = malloc(GATT_ACI_EVT_HDR + len);
    b[0] = HCI_EVENT_PKT_TYPE;
    b[1] = HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE;
    b[2] = (uint8_t)MIN(255U, 2U + len); /* informational; handlers read lengths */
    b[3] = (uint8_t)(ecode & 0xFF);
    b[4] = (uint8_t)(ecode >> 8);
    if(len) memcpy(b + GATT_ACI_EVT_HDR, payload, len);
    ble_dispatch_post(gatt_dyn_deliver, b);
}

static void gatt_dyn_on_write(
    int chr_id,
    DynGattWriteKind kind,
    uint16_t conn_handle,
    uint16_t attr_handle,
    const uint8_t* data,
    uint16_t len,
    void* ctx) {
    UNUSED(chr_id);
    UNUSED(kind);
    UNUSED(ctx);
    /* aci_gatt_attribute_modified_event_rp0: Connection_Handle, Attr_Handle,
     * Offset, Attr_Data_Length, Attr_Data[] (little-endian). */
    uint8_t* p = malloc(8U + len);
    p[0] = (uint8_t)conn_handle;
    p[1] = (uint8_t)(conn_handle >> 8);
    p[2] = (uint8_t)attr_handle;
    p[3] = (uint8_t)(attr_handle >> 8);
    p[4] = 0;
    p[5] = 0;
    p[6] = (uint8_t)len;
    p[7] = (uint8_t)(len >> 8);
    if(len) memcpy(p + 8, data, len);
    gatt_dyn_post_aci(ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE, p, 8U + len);
    free(p);
}

static void gatt_dyn_on_indicate_done(int chr_id, uint16_t conn_handle, void* ctx) {
    UNUSED(chr_id);
    UNUSED(ctx);
    uint8_t p[2] = {(uint8_t)conn_handle, (uint8_t)(conn_handle >> 8)};
    gatt_dyn_post_aci(ACI_GATT_SERVER_CONFIRMATION_VSEVT_CODE, p, sizeof(p));
}

/* Host thread, after a rebuild: give each instance its real handles so stock
 * code comparing Attr_Handle to handle + 1 / + 2 matches. */
static void gatt_dyn_on_committed(void* ctx) {
    UNUSED(ctx);
    for(size_t i = 0; i < GATT_DYN_CHR_MAX; i++) {
        BleGattCharacteristicInstance* inst = s_dyn_chrs[i].inst;
        if(!inst) continue;
        uint16_t decl = 0, dsc = 0;
        if(dyn_gatt_char_handles(s_dyn_chrs[i].chr_id, &decl, &dsc)) {
            inst->handle = decl;
            inst->descriptor_handle = dsc;
        }
    }
}

static const DynGattHooks gatt_dyn_hooks = {
    .on_write = gatt_dyn_on_write,
    .on_indicate_done = gatt_dyn_on_indicate_done,
    .on_committed = gatt_dyn_on_committed,
    .ctx = NULL,
};

static uint16_t gatt_dyn_flags(uint8_t props, uint8_t security) {
    /* Broadcast..signed-write property bits match NimBLE's flag bits. */
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

void ble_gatt_host_shim_commit(void) {
    if(s_shim && dyn_gatt_dirty()) {
        FURI_LOG_I(TAG, "shim: dynamic GATT services changed, rebuilding table");
        nimble_glue_gatt_rebuild_request();
    }
}

void ble_gatt_host_shim_set(const BleGattHostShim* shim) {
    if(shim) {
        /* Service code (e.g. ble_svc_hid_start in a FAP) registers an event
         * handler; the dispatcher init normally happens in gap_init, which never
         * runs without the CPU2 host. Idempotent. */
        ble_event_dispatcher_init();
        dyn_gatt_set_hooks(&gatt_dyn_hooks);
    } else {
        dyn_gatt_set_hooks(NULL);
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

    if(s_shim && gatt_dyn_svc_find(svc_handle) >= 0) {
        /* Dynamic service: define a real NimBLE characteristic. */
        int svc_id = gatt_dyn_svc_find(svc_handle);
        DynGattUuid uuid = {0};
        if(char_descriptor->uuid_type == UUID_TYPE_16) {
            uuid.type = 16;
            uuid.u16 = char_descriptor->uuid.Char_UUID_16;
        } else {
            uuid.type = 128;
            memcpy(uuid.u128, char_descriptor->uuid.Char_UUID_128, 16);
        }

        const uint8_t* init = NULL;
        uint16_t init_len = 0;
        if(char_descriptor->data_prop_type == FlipperGattCharacteristicDataFixed) {
            init = char_descriptor->data.fixed.ptr;
            init_len = char_descriptor->data.fixed.length;
        }

        DynGattUuid dsc_uuid = {0};
        uint8_t dsc_value[DYN_GATT_DSC_VALUE_MAX];
        uint8_t dsc_len = 0;
        bool has_dsc = false;
        if(char_descriptor->descriptor_params) {
            const BleGattCharacteristicDescriptorParams* desc = char_descriptor->descriptor_params;
            uint8_t const* desc_data = NULL;
            uint16_t desc_data_len = 0;
            bool release =
                desc->data_callback.fn(desc->data_callback.context, &desc_data, &desc_data_len);
            if(desc->uuid_type == UUID_TYPE_16) {
                dsc_uuid.type = 16;
                dsc_uuid.u16 = desc->uuid.Char_UUID_16;
            } else {
                dsc_uuid.type = 128;
                memcpy(dsc_uuid.u128, desc->uuid.Char_UUID_128, 16);
            }
            dsc_len = (uint8_t)MIN(desc_data_len, (uint16_t)sizeof(dsc_value));
            if(desc_data && dsc_len) memcpy(dsc_value, desc_data, dsc_len);
            if(release) free((void*)desc_data);
            has_dsc = true;
        }

        int chr_id = dyn_gatt_char_add(
            svc_id,
            &uuid,
            gatt_dyn_flags(char_descriptor->char_properties, char_descriptor->security_permissions),
            char_data_size,
            init,
            init_len,
            has_dsc ? &dsc_uuid : NULL,
            dsc_value,
            dsc_len);

        /* Placeholder handles until the table is rebuilt (gatt_dyn_on_committed). */
        char_instance->handle = ble_gatt_shim_alloc_handle();
        char_instance->descriptor_handle = has_dsc ? ble_gatt_shim_alloc_handle() : 0;
        if(chr_id < 0) {
            FURI_LOG_E(TAG, "shim: dynamic %s char rejected", char_descriptor->name);
            return;
        }
        FURI_CRITICAL_ENTER();
        for(size_t i = 0; i < GATT_DYN_CHR_MAX; i++) {
            if(!s_dyn_chrs[i].inst) {
                s_dyn_chrs[i].inst = char_instance;
                s_dyn_chrs[i].chr_id = chr_id;
                break;
            }
        }
        FURI_CRITICAL_EXIT();
        FURI_LOG_D(TAG, "shim: dynamic %s char -> id %d", char_descriptor->name, chr_id);
        return;
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
            bool release_data =
                desc->data_callback.fn(desc->data_callback.context, &desc_data, &desc_len);
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

    /* No shim means no host: CPU2 runs the HCILayer radio, which is a bare
     * controller with no GATT server to add a characteristic to. */
    ble_gatt_no_host(char_descriptor->name);
    char_instance->handle = 0;
    char_instance->descriptor_handle = 0;
}

void ble_gatt_characteristic_delete(
    uint16_t svc_handle,
    BleGattCharacteristicInstance* char_instance) {
    if(s_shim) {
        int chr_id = gatt_dyn_chr_find(char_instance);
        if(chr_id >= 0) {
            dyn_gatt_char_remove(chr_id); /* goes away on the next commit */
            FURI_CRITICAL_ENTER();
            for(size_t i = 0; i < GATT_DYN_CHR_MAX; i++) {
                if(s_dyn_chrs[i].inst == char_instance) s_dyn_chrs[i].inst = NULL;
            }
            FURI_CRITICAL_EXIT();
        } else {
            ble_gatt_shim_ref_clear(char_instance->handle);
        }
        free((void*)char_instance->characteristic);
        return;
    }

    UNUSED(svc_handle);
    ble_gatt_no_host(char_instance->characteristic->name);
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

    int dyn_chr_id = s_shim ? gatt_dyn_chr_find(char_instance) : -1;
    if(dyn_chr_id >= 0) {
        /* Dynamic service: store the value and notify/indicate subscribers.
         * Stock return convention: false means success. */
        bool ok = dyn_gatt_char_set_value(dyn_chr_id, char_data, char_data_size);
        if(release_data) {
            free((void*)char_data);
        }
        return !ok;
    }

    if(s_shim) {
        /* Hand the value to the resident host instead of CPU2. Mirrors the
         * stock return convention below: false means success. */
        if(s_shim->on_update) {
            uint16_t uuid16 = (char_descriptor->uuid_type == UUID_TYPE_16) ?
                                  char_descriptor->uuid.Char_UUID_16 :
                                  0;
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

    UNUSED(svc_handle);
    if(release_data) {
        free((void*)char_data);
    }
    ble_gatt_no_host(char_descriptor->name);
    return false;
}

bool ble_gatt_service_add(
    uint8_t Service_UUID_Type,
    const Service_UUID_t* Service_UUID,
    uint8_t Service_Type,
    uint8_t Max_Attribute_Records,
    uint16_t* Service_Handle) {
    if(s_shim) {
        *Service_Handle = ble_gatt_shim_alloc_handle();
        if(gatt_service_is_nimble_static(Service_UUID_Type, Service_UUID)) {
            return true; /* legacy mapping: NimBLE serves this statically */
        }
        UNUSED(Max_Attribute_Records);
        DynGattUuid uuid = {0};
        if(Service_UUID_Type == UUID_TYPE_16) {
            uuid.type = 16;
            uuid.u16 = Service_UUID->Service_UUID_16;
        } else {
            uuid.type = 128;
            memcpy(uuid.u128, Service_UUID->Service_UUID_128, 16);
        }
        int svc_id = dyn_gatt_service_add(&uuid, Service_Type != SECONDARY_SERVICE);
        if(svc_id < 0) {
            FURI_LOG_E(TAG, "shim: dynamic service rejected (no slot)");
            return false;
        }
        bool stored = false;
        FURI_CRITICAL_ENTER();
        for(size_t i = 0; i < GATT_DYN_SVC_MAX; i++) {
            if(!s_dyn_svcs[i].handle) {
                s_dyn_svcs[i].handle = *Service_Handle;
                s_dyn_svcs[i].svc_id = svc_id;
                stored = true;
                break;
            }
        }
        FURI_CRITICAL_EXIT();
        if(!stored) {
            dyn_gatt_service_remove(svc_id);
            return false;
        }
        return true;
    }

    UNUSED(Max_Attribute_Records);
    *Service_Handle = 0;
    ble_gatt_no_host("service");
    return false;
}

bool ble_gatt_service_delete(uint16_t svc_handle) {
    if(s_shim) {
        int svc_id = gatt_dyn_svc_find(svc_handle);
        if(svc_id >= 0) {
            dyn_gatt_service_remove(svc_id); /* goes away on the next commit */
            FURI_CRITICAL_ENTER();
            for(size_t i = 0; i < GATT_DYN_SVC_MAX; i++) {
                if(s_dyn_svcs[i].handle == svc_handle) s_dyn_svcs[i].handle = 0;
            }
            for(size_t i = 0; i < GATT_DYN_CHR_MAX; i++) {
                if(s_dyn_chrs[i].inst &&
                   s_dyn_chrs[i].chr_id / DYN_GATT_MAX_CHRS_PER_SVC == svc_id)
                    s_dyn_chrs[i].inst = NULL;
            }
            FURI_CRITICAL_EXIT();
        }
        return true;
    }

    UNUSED(svc_handle);
    ble_gatt_no_host("service");
    return false;
}
