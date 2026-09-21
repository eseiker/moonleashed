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
#include "host/ble_dtm.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "nimble_glue.h"
#include "serial_gatt.h"
#include "info_gatt.h"
#include "hid_gatt.h"
#include "dyn_gatt.h"
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
/* ble_hs_priv.h */
extern uint16_t ble_hs_max_attrs;
extern uint16_t ble_hs_max_services;
extern uint16_t ble_hs_max_client_configs;

static struct {
    FuriThread* host;
    volatile bool started;
    volatile bool synced;
    volatile uint32_t sync_count;
    volatile bool advertising;
    volatile bool pairing;
    volatile bool adv_disabled;
    volatile bool hid_advertised;
    volatile bool gatt_rebuilding;
    /* The last rebuild failed; the table needs one even if nothing changed. */
    bool gatt_retry;
    volatile uint32_t passkey;
    volatile uint16_t conn_handle;
    uint8_t addr_type;
} glue;

static struct ble_npl_event adv_setting_event;
static struct ble_npl_event disconnect_event;
static struct ble_npl_event forget_event;
static struct ble_npl_event reload_event;
static struct ble_npl_event readvertise_event;
static struct ble_npl_event gatt_rebuild_event;
static struct ble_npl_callout gatt_retry_callout;

static int gap_event(struct ble_gap_event* event, void* arg);

/* An app's extra beacon takes over the radio's one advertising set: the
 * companion advertisement pauses and links stay up. Apps that spam beacons
 * restart them every few tens of ms, which leaves no window to alternate. */
static struct {
    volatile bool wanted;
    volatile bool restart; /* started again with a new config */
    volatile bool data_dirty;
    bool running; /* host thread */
    /* Written by the app thread, read by the host: both under the critical section. */
    uint8_t data[31];
    uint8_t len;
    uint16_t itvl_min, itvl_max;
    uint8_t chan_map;
    bool public_addr;
    uint8_t addr[6];
} beacon;
static struct ble_npl_event beacon_event;

static void beacon_set_data(void) {
    /* Copy only under the critical section: set_data waits for the controller,
     * and waiting with interrupts off hangs the device. */
    uint8_t data[sizeof(beacon.data)];
    FURI_CRITICAL_ENTER();
    uint8_t len = beacon.len;
    memcpy(data, beacon.data, len);
    FURI_CRITICAL_EXIT();
    ble_gap_adv_set_data(data, len);
}

/* Host thread. True while the beacon holds the advertising set. */
static bool beacon_run(void) {
    if(!beacon.wanted) return false;
    if(beacon.running && !beacon.restart) {
        if(beacon.data_dirty) {
            beacon.data_dirty = false;
            beacon_set_data();
        }
        return true;
    }
    if(beacon.running || glue.advertising) ble_gap_adv_stop();
    glue.advertising = false;
    beacon.running = false;
    beacon.restart = false;
    beacon.data_dirty = false;

    struct ble_gap_adv_params p = {
        .conn_mode = BLE_GAP_CONN_MODE_NON,
        .disc_mode = BLE_GAP_DISC_MODE_NON,
    };
    uint8_t addr[6];
    FURI_CRITICAL_ENTER();
    p.itvl_min = beacon.itvl_min;
    p.itvl_max = beacon.itvl_max;
    p.channel_map = beacon.chan_map;
    bool public_addr = beacon.public_addr;
    memcpy(addr, beacon.addr, sizeof(addr));
    FURI_CRITICAL_EXIT();

    uint8_t own = BLE_OWN_ADDR_RANDOM;
    if(public_addr) {
        own = BLE_OWN_ADDR_PUBLIC;
    } else if(ble_hs_id_set_rnd(addr) != 0) {
        /* NimBLE takes static or non-resolvable random addresses only */
        addr[5] |= 0xC0;
        if(ble_hs_id_set_rnd(addr) != 0) own = glue.addr_type;
    }
    beacon_set_data();
    int rc = ble_gap_adv_start(own, NULL, BLE_HS_FOREVER, &p, NULL, NULL);
    if(rc != 0) {
        /* Give the advertising set back to the companion. */
        FURI_LOG_E(TAG, "Beacon start: %d", rc);
        beacon.wanted = false;
        return false;
    }
    beacon.running = true;
    return true;
}

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
    if(glue.synced && beacon_run()) return;
    if(!glue.synced || glue.adv_disabled || glue.advertising || glue.gatt_rebuilding) return;
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
        if(glue.gatt_rebuilding) {
            ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &gatt_rebuild_event);
        } else {
            maybe_advertise();
        }
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        glue.advertising = false;
        maybe_advertise();
        break;

    case BLE_GAP_EVENT_NOTIFY_TX:
        serial_gatt_on_notify_tx(event->notify_tx.attr_handle, event->notify_tx.status);
        dyn_gatt_on_notify_tx(
            event->notify_tx.conn_handle,
            event->notify_tx.attr_handle,
            event->notify_tx.status,
            event->notify_tx.indication);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        serial_gatt_on_subscribe(
            event->subscribe.attr_handle,
            event->subscribe.cur_notify || event->subscribe.cur_indicate);
        hid_gatt_on_subscribe(event->subscribe.attr_handle, event->subscribe.cur_notify);
        dyn_gatt_on_subscribe(
            event->subscribe.conn_handle,
            event->subscribe.attr_handle,
            event->subscribe.cur_notify,
            event->subscribe.cur_indicate);
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
    glue.sync_count++;
    hid_gatt_set_visible(glue.hid_advertised, false);
    if(dyn_gatt_dirty() || glue.gatt_retry) {
        ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &gatt_rebuild_event);
    }
    maybe_advertise();
}

static void on_reset(int reason) {
    FURI_LOG_W(TAG, "Controller reset, reason %d", reason);
    glue.synced = false;
    glue.advertising = false;
    beacon.running = false;
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

static int register_services(void) {
    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = info_gatt_register();
    if(rc == 0) rc = serial_gatt_register();
    if(rc == 0) rc = hid_gatt_register();
    return rc;
}

/* NimBLE changes its table only with no link and no GAP procedure. The
 * built-in services register first, in boot order, so their handles and the
 * CCCDs bonded peers cached stay put. */
static void gatt_rebuild_event_fn(struct ble_npl_event* ev) {
    UNUSED(ev);
    if(!glue.synced || !(dyn_gatt_dirty() || glue.gatt_retry)) {
        /* After a controller reset, on_sync starts it again. */
        glue.gatt_rebuilding = false;
        maybe_advertise();
        return;
    }
    glue.gatt_rebuilding = true;
    /* Waits for the beacon to stop: the table changes only with no GAP procedure */
    if(beacon.wanted) return;
    if(glue.advertising) {
        ble_gap_adv_stop();
        glue.advertising = false;
    }
    if(glue.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        /* Continues on the disconnect event. */
        ble_gap_terminate(glue.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    int rc = ble_gatts_reset();
    if(rc == 0) {
        /* NimBLE 1.10.0's ble_gatts_reset leaves these, and they would grow
         * with every rebuild. Fixed upstream in af4baa41. */
        ble_hs_max_attrs = 0;
        ble_hs_max_services = 0;
        ble_hs_max_client_configs = 0;
        rc = register_services();
        if(rc == 0) {
            dyn_gatt_register_all();
            rc = ble_gatts_start();
        }
    }
    glue.gatt_retry = rc != 0;
    if(rc == 0) {
        dyn_gatt_after_start();
        hid_gatt_set_visible(glue.hid_advertised, false);
        ble_svc_gatt_changed(0x0001, 0xFFFF);
    } else {
        FURI_LOG_E(TAG, "GATT rebuild failed: %d", rc);
    }
    if(glue.gatt_retry) {
        /* Stays gatt_rebuilding, so nothing advertises the broken table. */
        ble_npl_callout_reset(&gatt_retry_callout, ble_npl_time_ms_to_ticks32(200));
    } else if(dyn_gatt_dirty()) {
        ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &gatt_rebuild_event);
    } else {
        glue.gatt_rebuilding = false;
        maybe_advertise();
    }
}

static void post(struct ble_npl_event* ev) {
    if(glue.started) ble_npl_eventq_put(nimble_port_get_dflt_eventq(), ev);
}

static void beacon_event_fn(struct ble_npl_event* ev) {
    UNUSED(ev);
    if(!glue.synced) return; /* on_sync starts it */
    if(!beacon.wanted && beacon.running) {
        ble_gap_adv_stop();
        beacon.running = false;
    }
    maybe_advertise();
    /* A rebuild waits while the beacon is wanted. */
    if(!beacon.wanted && (dyn_gatt_dirty() || glue.gatt_retry)) post(&gatt_rebuild_event);
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
    ble_npl_event_init(&gatt_rebuild_event, gatt_rebuild_event_fn, NULL);
    ble_npl_callout_init(
        &gatt_retry_callout, nimble_port_get_dflt_eventq(), gatt_rebuild_event_fn, NULL);
    ble_npl_event_init(&beacon_event, beacon_event_fn, NULL);
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_store_config_init();
    bond_store_load();

    const char* name = furi_hal_version_get_device_name_ptr();
    ble_svc_gap_device_name_set(name ? name : "Flipper");

    info_gatt_init();
    dyn_gatt_init();
    if(register_services() != 0) return false;
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

void nimble_glue_gatt_rebuild(void) {
    post(&gatt_rebuild_event);
}

bool nimble_glue_beacon_start(
    const uint8_t* data,
    uint8_t len,
    uint16_t min_interval_ms,
    uint16_t max_interval_ms,
    uint8_t channel_map,
    bool public_address,
    const uint8_t address[6]) {
    if(!glue.started || len > sizeof(beacon.data)) return false;
    FURI_CRITICAL_ENTER();
    memcpy(beacon.data, data, len);
    beacon.len = len;
    beacon.itvl_min = BLE_GAP_ADV_ITVL_MS(min_interval_ms);
    beacon.itvl_max = BLE_GAP_ADV_ITVL_MS(max_interval_ms);
    beacon.chan_map = channel_map;
    beacon.public_addr = public_address;
    memcpy(beacon.addr, address, sizeof(beacon.addr));
    FURI_CRITICAL_EXIT();
    beacon.restart = true;
    beacon.wanted = true;
    post(&beacon_event);
    return true;
}

void nimble_glue_beacon_set_data(const uint8_t* data, uint8_t len) {
    if(len > sizeof(beacon.data)) return;
    FURI_CRITICAL_ENTER();
    memcpy(beacon.data, data, len);
    beacon.len = len;
    FURI_CRITICAL_EXIT();
    beacon.data_dirty = true;
    if(beacon.wanted) post(&beacon_event);
}

bool nimble_glue_beacon_is_wanted(void) {
    return beacon.wanted;
}

void nimble_glue_beacon_stop(void) {
    beacon.wanted = false;
    post(&beacon_event);
}

/* Radio tests. The caller stops advertising and drops the link first; those
 * run later on the host thread, so each test waits for the link layer to go
 * idle. ACI_HAL_GET_LINK_STATUS reports one state per slot: idle 0x00,
 * RX test 0x07. */
#define ACI_HAL_SET_TX_POWER_LEVEL_OCF 0x000F
#define ACI_HAL_TONE_START_OCF         0x0015
#define ACI_HAL_TONE_STOP_OCF          0x0016
#define ACI_HAL_LE_TX_TEST_PACKETS_OCF 0x0014
#define ACI_HAL_GET_LINK_STATUS_OCF    0x0017
#define ACI_HAL_READ_RSSI_OCF          0x0022
#define LL_STATE_IDLE                  0x00
#define LL_STATE_RX_TEST               0x07

/* Slot 0 in state, the other seven idle. */
static bool ll_wait(uint8_t state, uint32_t timeout_ms) {
    uint8_t rsp[24]; /* 8 states, 8 connection handles */
    for(uint32_t waited = 0;; waited += 10) {
        if(ble_hs_hci_send_vs_cmd(ACI_HAL_GET_LINK_STATUS_OCF, NULL, 0, rsp, sizeof(rsp)) != 0)
            return false;
        bool match = rsp[0] == state;
        for(uint8_t i = 1; i < 8 && match; i++)
            match = rsp[i] == LL_STATE_IDLE;
        if(match) return true;
        if(waited >= timeout_ms) {
            if(timeout_ms) FURI_LOG_W(TAG, "Link layer busy, test not started");
            return false;
        }
        furi_delay_ms(10);
    }
}

/* CPU2 hard-faulted now and then after a test unless the controller was reset.
 * NimBLE resyncs and on_sync advertises again if Bluetooth is on. */
static volatile bool test_reset_pending;
static volatile uint32_t test_reset_sync_count;

static void test_end_reset(void) {
    test_reset_sync_count = glue.sync_count;
    test_reset_pending = true;
    ble_hs_sched_reset(BLE_HS_ECONTROLLER);
}

/* A test started right after a stop waits here for that reset's resync. */
static bool test_wait_resync(void) {
    for(uint32_t waited = 0; test_reset_pending; waited += 10) {
        if(glue.synced && glue.sync_count != test_reset_sync_count) {
            test_reset_pending = false;
        } else if(waited >= 3000) {
            FURI_LOG_W(TAG, "No resync after the last test");
            return false;
        } else {
            furi_delay_ms(10);
        }
    }
    return true;
}

bool nimble_glue_dtm_tx_start(uint8_t channel, uint8_t payload, uint8_t phy) {
    if(!test_wait_resync() || !ll_wait(LL_STATE_IDLE, 1000)) return false;
    struct ble_dtm_tx_params p = {
        .channel = channel,
        .test_data_len = 37,
        .payload = payload,
        .phy = phy,
    };
    return ble_dtm_tx_start(&p) == 0;
}

bool nimble_glue_dtm_rx_start(uint8_t channel, uint8_t phy) {
    if(!test_wait_resync() || !ll_wait(LL_STATE_IDLE, 1000)) return false;
    struct ble_dtm_rx_params p = {.channel = channel, .phy = phy};
    return ble_dtm_rx_start(&p) == 0;
}

bool nimble_glue_dtm_stop(uint16_t* rx_packets, uint32_t* tx_packets) {
    /* Test End reports received packets only; ST's command counts sent ones. */
    uint8_t rsp[4] = {0};
    ble_hs_hci_send_vs_cmd(ACI_HAL_LE_TX_TEST_PACKETS_OCF, NULL, 0, rsp, sizeof(rsp));
    *tx_packets = rsp[0] | (rsp[1] << 8) | (rsp[2] << 16) | ((uint32_t)rsp[3] << 24);
    int rc = ble_dtm_stop(rx_packets);
    test_end_reset();
    return rc == 0;
}

bool nimble_glue_tone_start(uint8_t channel, uint8_t pa_level) {
    if(!test_wait_resync() || !ll_wait(LL_STATE_IDLE, 1000)) return false;
    uint8_t power[2] = {0 /* normal mode */, pa_level};
    uint8_t tone[2] = {channel, 0 /* no offset */};
    int rc = ble_hs_hci_send_vs_cmd(ACI_HAL_SET_TX_POWER_LEVEL_OCF, power, 2, NULL, 0);
    if(rc == 0) rc = ble_hs_hci_send_vs_cmd(ACI_HAL_TONE_START_OCF, tone, 2, NULL, 0);
    return rc == 0;
}

void nimble_glue_tone_stop(void) {
    ble_hs_hci_send_vs_cmd(ACI_HAL_TONE_STOP_OCF, NULL, 0, NULL, 0);
    test_end_reset();
}

bool nimble_glue_read_rssi(int8_t* dbm) {
    /* This radio has no ACI_HAL_RX_START; the level exists only during an RX test. */
    if(!ll_wait(LL_STATE_RX_TEST, 0)) return false;
    uint8_t value = 0x7F;
    int rc = ble_hs_hci_send_vs_cmd(ACI_HAL_READ_RSSI_OCF, NULL, 0, &value, 1);
    /* 0x7F: nothing received */
    if(rc != 0 || value == 0x7F) return false;
    *dbm = (int8_t)value;
    return true;
}
