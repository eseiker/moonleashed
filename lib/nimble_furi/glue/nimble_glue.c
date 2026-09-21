/* NimBLE host bring-up, the companion advertisement and pairing.
 * Every GAP call runs on the host thread; other threads post events. */

#include <furi.h>
#include <furi_hal_version.h>
#include <furi_hal_random.h>

#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_hs_hci.h"
#include "host/ble_gap.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "nimble_glue.h"
#include "serial_gatt.h"
#include "info_gatt.h"
#include "hid_gatt.h"
#include "bond_store.h"
#include "msys_pool.h"

#define TAG "NimbleGlue"

#define HOST_STACK_SIZE 4096

/* ST vendor command: write the public address into the controller. */
#define ACI_HAL_WRITE_CONFIG_DATA_OCF 0x000C
#define CONFIG_DATA_PUBADDR_OFFSET    0x00

extern void ble_store_config_init(void);
/* ble_hs_id_priv.h */
extern void ble_hs_id_set_pub(const uint8_t* pub_addr);

static struct {
    FuriThread* host;
    volatile bool started;
    volatile bool synced;
    volatile bool advertising;
    volatile bool pairing;
    volatile bool adv_disabled;
    volatile bool hid_advertised;
    volatile uint32_t passkey;
    volatile uint16_t conn_handle;
    uint8_t addr_type;
} glue;

static struct ble_npl_event adv_setting_event;
static struct ble_npl_event disconnect_event;
static struct ble_npl_event forget_event;
static struct ble_npl_event reload_event;
static struct ble_npl_event readvertise_event;

static int gap_event(struct ble_gap_event* event, void* arg);

static void start_advertise(void) {
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* The stock serial profile advertises 0x3080 | hw_color. A phone that sees
     * HID runs its keyboard pairing flow, which the companion cannot answer, so
     * HID is advertised only while a HID app runs. */
    ble_uuid16_t uuids[2] = {
        BLE_UUID16_INIT(0x3080 | (uint16_t)furi_hal_version_get_hw_color()),
        BLE_UUID16_INIT(0x1812),
    };
    fields.uuids16 = uuids;
    fields.num_uuids16 = glue.hid_advertised ? 2 : 1;
    fields.uuids16_is_complete = 1;

    const char* name = furi_hal_version_get_device_name_ptr();
    if(name) {
        fields.name = (uint8_t*)name;
        fields.name_len = strlen(name);
        fields.name_is_complete = 1;
    }

    int rc = ble_gap_adv_set_fields(&fields);
    if(rc != 0) {
        FURI_LOG_E(TAG, "adv_set_fields: %d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp = {0};
    rsp.appearance = glue.hid_advertised ? 0x03C1 /* keyboard */ : 0x8600;
    rsp.appearance_is_present = 1;
    ble_gap_adv_rsp_set_fields(&rsp);

    struct ble_gap_adv_params params = {0};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(glue.addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    if(rc != 0) {
        FURI_LOG_E(TAG, "adv_start: %d", rc);
        return;
    }
    glue.advertising = true;
}

static void maybe_advertise(void) {
    if(!glue.synced || glue.adv_disabled || glue.advertising) return;
    if(glue.conn_handle != BLE_HS_CONN_HANDLE_NONE) return;
    start_advertise();
}

static void on_passkey_action(const struct ble_gap_event* event) {
    if(event->passkey.params.action != BLE_SM_IOACT_DISP) {
        FURI_LOG_W(TAG, "Unsupported pairing action %u", event->passkey.params.action);
        return;
    }
    struct ble_sm_io io = {0};
    io.action = BLE_SM_IOACT_DISP;
    io.passkey = furi_hal_random_get() % 1000000u;
    glue.passkey = io.passkey;
    glue.pairing = true;
    ble_sm_inject_io(event->passkey.conn_handle, &io);
}

static int gap_event(struct ble_gap_event* event, void* arg) {
    UNUSED(arg);
    switch(event->type) {
    case BLE_GAP_EVENT_CONNECT:
        glue.advertising = false;
        if(event->connect.status == 0) {
            glue.conn_handle = event->connect.conn_handle;
            glue.pairing = false;
            serial_gatt_set_conn(event->connect.conn_handle, true);
            hid_gatt_set_conn(event->connect.conn_handle, true);
            FURI_LOG_I(TAG, "Connected, handle %u", event->connect.conn_handle);
        } else {
            maybe_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        FURI_LOG_I(TAG, "Disconnected, reason %d", event->disconnect.reason);
        glue.conn_handle = BLE_HS_CONN_HANDLE_NONE;
        glue.pairing = false;
        serial_gatt_set_conn(0, false);
        hid_gatt_set_conn(0, false);
        /* CCCDs written during the connection. */
        bond_store_save();
        maybe_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        glue.advertising = false;
        maybe_advertise();
        break;

    case BLE_GAP_EVENT_NOTIFY_TX:
        serial_gatt_on_notify_tx(event->notify_tx.attr_handle, event->notify_tx.status);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        serial_gatt_on_subscribe(
            event->subscribe.attr_handle,
            event->subscribe.cur_notify || event->subscribe.cur_indicate);
        hid_gatt_on_subscribe(event->subscribe.attr_handle, event->subscribe.cur_notify);
        break;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        on_passkey_action(event);
        break;

    case BLE_GAP_EVENT_ENC_CHANGE: {
        glue.pairing = false;
        FURI_LOG_I(TAG, "Encryption status %d", event->enc_change.status);
        if(event->enc_change.status == 0) bond_store_save();
        break;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        /* Drop the stale bond but keep the link: ble_gap_unpair would
         * terminate the connection the retry continues on. */
        struct ble_gap_conn_desc desc;
        if(ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    default:
        break;
    }
    return 0;
}

static int set_public_address(void) {
    const uint8_t* mac = furi_hal_version_get_ble_mac();
    if(!mac) return -1;
    uint8_t cmd[2 + 6] = {CONFIG_DATA_PUBADDR_OFFSET, 6};
    memcpy(&cmd[2], mac, 6);
    int rc = ble_hs_hci_send_vs_cmd(ACI_HAL_WRITE_CONFIG_DATA_OCF, cmd, sizeof(cmd), NULL, 0);
    /* The SM computes legacy confirm values from this address. */
    if(rc == 0) ble_hs_id_set_pub(mac);
    return rc;
}

static void on_sync(void) {
    if(set_public_address() == 0) {
        glue.addr_type = BLE_OWN_ADDR_PUBLIC;
    } else if(ble_hs_id_infer_auto(0, &glue.addr_type) != 0) {
        FURI_LOG_E(TAG, "No usable address");
        return;
    }
    glue.synced = true;
    hid_gatt_set_visible(glue.hid_advertised, false);
    maybe_advertise();
}

static void on_reset(int reason) {
    FURI_LOG_W(TAG, "Controller reset, reason %d", reason);
    glue.synced = false;
    glue.advertising = false;
}

static void adv_setting_event_fn(struct ble_npl_event* ev) {
    UNUSED(ev);
    if(!glue.adv_disabled) {
        maybe_advertise();
        return;
    }
    if(glue.advertising) {
        ble_gap_adv_stop();
        glue.advertising = false;
    }
    if(glue.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(glue.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

static void disconnect_event_fn(struct ble_npl_event* ev) {
    UNUSED(ev);
    if(glue.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(glue.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

static void forget_event_fn(struct ble_npl_event* ev) {
    UNUSED(ev);
    /* As stock: a link whose bond is going ends too. */
    if(glue.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(glue.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    ble_store_clear();
    bond_store_forget();
}

static void reload_event_fn(struct ble_npl_event* ev) {
    UNUSED(ev);
    bond_store_load();
}

static void readvertise_event_fn(struct ble_npl_event* ev) {
    UNUSED(ev);
    if(glue.synced) hid_gatt_set_visible(glue.hid_advertised, true);
    if(glue.advertising) {
        ble_gap_adv_stop();
        glue.advertising = false;
    }
    maybe_advertise();
}

static void post(struct ble_npl_event* ev) {
    if(glue.started) ble_npl_eventq_put(nimble_port_get_dflt_eventq(), ev);
}

static int32_t host_task(void* context) {
    UNUSED(context);
    nimble_port_run();
    return 0;
}

bool nimble_glue_start(void) {
    memset(&glue, 0, sizeof(glue));
    glue.conn_handle = BLE_HS_CONN_HANDLE_NONE;

    nimble_port_init();
    /* After nimble_port_init, which resets the msys pool list. */
    if(!msys_pool_init()) return false;

    ble_npl_event_init(&adv_setting_event, adv_setting_event_fn, NULL);
    ble_npl_event_init(&disconnect_event, disconnect_event_fn, NULL);
    ble_npl_event_init(&forget_event, forget_event_fn, NULL);
    ble_npl_event_init(&reload_event, reload_event_fn, NULL);
    ble_npl_event_init(&readvertise_event, readvertise_event_fn, NULL);
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_store_config_init();
    bond_store_load();

    const char* name = furi_hal_version_get_device_name_ptr();
    ble_svc_gap_device_name_set(name ? name : "Flipper");

    if(info_gatt_register() != 0 || serial_gatt_register() != 0 || hid_gatt_register() != 0)
        return false;
    serial_gatt_init();

    glue.host = furi_thread_alloc_ex("NimbleHost", HOST_STACK_SIZE, host_task, NULL);
    furi_thread_start(glue.host);
    glue.started = true;
    return true;
}

bool nimble_glue_is_started(void) {
    return glue.started;
}

bool nimble_glue_is_synced(void) {
    return glue.synced;
}

bool nimble_glue_is_advertising(void) {
    return glue.advertising;
}

bool nimble_glue_is_connected(void) {
    return glue.conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

bool nimble_glue_is_pairing(void) {
    return glue.pairing;
}

uint32_t nimble_glue_passkey(void) {
    return glue.passkey;
}

void nimble_glue_set_advertising_enabled(bool enabled) {
    if(glue.adv_disabled == !enabled) return;
    glue.adv_disabled = !enabled;
    post(&adv_setting_event);
}

void nimble_glue_disconnect(void) {
    post(&disconnect_event);
}

void nimble_glue_forget_bonds(void) {
    post(&forget_event);
}

void nimble_glue_reload_bonds(void) {
    post(&reload_event);
}

void nimble_glue_set_battery_level(uint8_t level) {
    if(glue.started) info_gatt_set_battery_level(level);
}

void nimble_glue_set_power_state(bool charging) {
    if(glue.started) info_gatt_set_power_state(charging);
}

void nimble_glue_set_hid_advertised(bool advertised) {
    if(glue.hid_advertised == advertised) return;
    glue.hid_advertised = advertised;
    post(&readvertise_event);
}

void nimble_glue_hid_set_report_map(const uint8_t* data, uint16_t len) {
    if(glue.started && data && len) hid_gatt_set_report_map(data, len);
}

bool nimble_glue_hid_input_report(uint8_t report_id, const uint8_t* data, uint16_t len) {
    return glue.started && hid_gatt_input_report(report_id, data, len);
}
