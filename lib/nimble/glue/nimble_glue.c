/*
 * NimBLE host bring-up, peripheral advertising, and teardown for the Flipper
 * FAP (TASK-575 milestones 1-2; TASK-589 milestone B1).
 *
 * nimble_glue_start initializes the port, registers GAP/GATT, the RAM store and
 * the Flipper Serial Service GATT server (serial_gatt.c), installs the
 * sync/reset callbacks, and starts an exitable host event thread. When NimBLE
 * finishes controller startup it calls on_sync, which reads the identity address
 * and starts connectable advertising of the Serial Service (0x3080 | hw_color)
 * so the mobile companion app can find and connect to the Flipper. Connect and
 * disconnect events flow into gap_event, which re-arms advertising after a
 * disconnect. The earlier central passive-scan path (scan_count/last_addr) is
 * retained in gap_event but no longer started.
 *
 * nimble_glue_stop tears the host down gracefully with ble_hs_stop, exits the
 * host thread, stops the HCI reader, and frees every NPL FURI object (timers,
 * queues, mutexes, semaphores). Freeing the callout timers is what lets the FAP
 * unload without a reboot.
 */

#include <furi.h>
#include <furi_hal_version.h>
#include <furi_hal_random.h>

#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs_hci.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "host/ble_uuid.h"
#include "nimble/ble.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "nimble_glue.h"
#include "serial_gatt.h"
#include "serial_store.h"
#include "hid_gatt.h"
#include "coc_glue.h"

#define TAG "NimbleGlue"

extern void ble_store_config_init(void);
extern void npl_furi_shutdown(void);
/* Internal NimBLE setter (ble_hs_id_priv.h): overrides the host's cached public
 * identity address. We use it to keep NimBLE's SM crypto in sync with the public
 * address we write into the controller after sync. */
extern void ble_hs_id_set_pub(const uint8_t* pub_addr);

static struct {
    FuriThread* host;
    volatile bool host_run;
    volatile bool started;
    volatile bool synced;
    volatile bool scanning;
    volatile bool advertising;
    volatile bool connected;
    volatile bool pairing;
    volatile bool bonded;
    volatile uint32_t passkey;
    volatile uint16_t conn_handle;
    volatile uint32_t scan_count;
    uint8_t addr_type;
    uint8_t addr[6];
    uint8_t last_addr[6];
    NimbleMode mode;
} glue;

bool nimble_mode_has_serial(NimbleMode mode) {
    return mode == NimbleModePeripheralCombined || mode == NimbleModePeripheralSerial;
}

bool nimble_mode_has_hid(NimbleMode mode) {
    return mode == NimbleModePeripheralCombined || mode == NimbleModePeripheralHid;
}

static struct ble_hs_stop_listener stop_listener;
static FuriSemaphore* stop_sem;

static void start_advertise(void);

static int gap_event(struct ble_gap_event* event, void* arg) {
    UNUSED(arg);
    switch(event->type) {
    case BLE_GAP_EVENT_DISC:
        glue.scan_count++;
        memcpy(glue.last_addr, event->disc.addr.val, 6);
        break;

    case BLE_GAP_EVENT_CONNECT:
        if(event->connect.status == 0) {
            glue.connected = true;
            glue.advertising = false;
            glue.bonded = false;
            glue.pairing = false;
            glue.conn_handle = event->connect.conn_handle;
            if(nimble_mode_has_serial(glue.mode))
                serial_gatt_set_conn(event->connect.conn_handle, true);
            if(nimble_mode_has_hid(glue.mode))
                hid_gatt_set_conn(event->connect.conn_handle, true);
            FURI_LOG_I(TAG, "Central connected, handle %u", event->connect.conn_handle);
        } else {
            FURI_LOG_W(TAG, "Connect failed: %d", event->connect.status);
            start_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        FURI_LOG_I(TAG, "Central disconnected, reason %d", event->disconnect.reason);
        glue.connected = false;
        glue.pairing = false;
        glue.bonded = false;
        glue.conn_handle = BLE_HS_CONN_HANDLE_NONE;
        if(nimble_mode_has_serial(glue.mode)) serial_gatt_set_conn(0, false);
        if(nimble_mode_has_hid(glue.mode)) hid_gatt_set_conn(0, false);
        serial_store_save(); /* persist any CCCDs written during the connection */
        start_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        FURI_LOG_I(TAG, "Advertising complete, restarting");
        start_advertise();
        break;

    case BLE_GAP_EVENT_NOTIFY_TX:
        if(nimble_mode_has_serial(glue.mode))
            serial_gatt_on_notify_tx(event->notify_tx.attr_handle, event->notify_tx.status);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if(nimble_mode_has_serial(glue.mode))
            serial_gatt_on_subscribe(
                event->subscribe.attr_handle,
                event->subscribe.cur_notify || event->subscribe.cur_indicate);
        break;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        /* DISPLAY_ONLY IO cap with MITM: we pick a 6-digit passkey, show it on
         * the Flipper screen, and inject it; the phone types the same number. */
        if(event->passkey.params.action == BLE_SM_IOACT_DISP) {
            struct ble_sm_io io = {0};
            io.action = BLE_SM_IOACT_DISP;
            io.passkey = furi_hal_random_get() % 1000000u;
            glue.passkey = io.passkey;
            glue.pairing = true;
            int rc = ble_sm_inject_io(event->passkey.conn_handle, &io);
            FURI_LOG_I(TAG, "Passkey display %06lu (inject rc %d)", io.passkey, rc);
        }
        break;

    case BLE_GAP_EVENT_ENC_CHANGE: {
        glue.pairing = false;
        struct ble_gap_conn_desc desc;
        if(ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
            glue.bonded = desc.sec_state.encrypted;
            FURI_LOG_I(
                TAG,
                "ENC change st=%d enc=%d authen=%d bonded=%d keysz=%d",
                event->enc_change.status,
                desc.sec_state.encrypted,
                desc.sec_state.authenticated,
                desc.sec_state.bonded,
                desc.sec_state.key_size);
        } else {
            glue.bonded = (event->enc_change.status == 0);
            FURI_LOG_I(TAG, "ENC change st=%d (no desc)", event->enc_change.status);
        }
        if(glue.bonded) serial_store_save(); /* persist the new bond keys */
        break;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        /* The peer re-pairs while a bond already exists (RAM store lost it, or
         * the phone forgot). Drop the stale bond and let pairing proceed. */
        struct ble_gap_conn_desc desc;
        if(ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_gap_unpair(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    default:
        break;
    }
    return 0;
}

static void start_advertise(void) {
    /* Advertising payload: flags + the 16-bit Serial Service UUID (0x3080 |
     * hw_color, exactly what the stock serial profile advertises) + the device
     * name. Appearance goes in the scan response to keep the 31-byte adv packet
     * within budget. */
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Advertise the service UUIDs the active mode exposes. The Serial Service
     * UUID (0x3080 | hw_color) goes first so the verified companion scan still
     * finds it; the HID Service UUID (0x1812) lets a phone offer the Flipper as a
     * BLE input device. */
    ble_uuid16_t adv_uuids[2];
    uint8_t n_uuids = 0;
    if(nimble_mode_has_serial(glue.mode)) {
        uint16_t svc16 = 0x3080 | (uint16_t)furi_hal_version_get_hw_color();
        adv_uuids[n_uuids++] = (ble_uuid16_t)BLE_UUID16_INIT(svc16);
    }
    if(nimble_mode_has_hid(glue.mode)) {
        adv_uuids[n_uuids++] = (ble_uuid16_t)BLE_UUID16_INIT(0x1812);
    }
    fields.uuids16 = adv_uuids;
    fields.num_uuids16 = n_uuids;
    fields.uuids16_is_complete = 1;

    const char* name = furi_hal_version_get_device_name_ptr();
    if(name) {
        fields.name = (uint8_t*)name;
        fields.name_len = strlen(name);
        fields.name_is_complete = 1;
    }

    int rc = ble_gap_adv_set_fields(&fields);
    if(rc != 0) {
        FURI_LOG_E(TAG, "adv_set_fields failed: %d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    /* Advertise a keyboard appearance when HID is exposed so hosts offer BLE HID
     * pairing; otherwise the generic Flipper appearance. */
    rsp.appearance = nimble_mode_has_hid(glue.mode) ? 0x03C1 : 0x8600;
    rsp.appearance_is_present = 1;
    ble_gap_adv_rsp_set_fields(&rsp);

    struct ble_gap_adv_params advp;
    memset(&advp, 0, sizeof(advp));
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(glue.addr_type, NULL, BLE_HS_FOREVER, &advp, gap_event, NULL);
    if(rc != 0) {
        FURI_LOG_E(TAG, "adv_start failed: %d", rc);
        return;
    }
    glue.advertising = true;
    FURI_LOG_I(TAG, "Advertising started");
}

static void on_reset(int reason) {
    FURI_LOG_W(TAG, "Controller reset, reason=%d", reason);
    glue.synced = false;
    glue.scanning = false;
}

/* ST vendor command ACI_HAL_WRITE_CONFIG_DATA (OGF 0x3F, OCF 0x0C). Writing the
 * public device address at offset CONFIG_DATA_PUBADDR_OFFSET makes the
 * controller advertise the real Flipper BLE MAC, exactly as the stock GAP init
 * does (targets/f7/ble_glue/gap.c). Must run after the HCI reset NimBLE issues
 * during sync and before advertising. */
#define ACI_HAL_WRITE_CONFIG_DATA_OCF 0x000C
#define CONFIG_DATA_PUBADDR_OFFSET    0x00
#define CONFIG_DATA_PUBADDR_LEN       6

static int set_public_address(void) {
    const uint8_t* mac = furi_hal_version_get_ble_mac();
    if(!mac) return -1;
    uint8_t cmd[2 + CONFIG_DATA_PUBADDR_LEN];
    cmd[0] = CONFIG_DATA_PUBADDR_OFFSET;
    cmd[1] = CONFIG_DATA_PUBADDR_LEN;
    memcpy(&cmd[2], mac, CONFIG_DATA_PUBADDR_LEN);
    int rc = ble_hs_hci_send_vs_cmd(ACI_HAL_WRITE_CONFIG_DATA_OCF, cmd, sizeof(cmd), NULL, 0);
    FURI_LOG_I(
        TAG,
        "Write pub addr %02X:%02X:%02X:%02X:%02X:%02X rc=%d",
        mac[5],
        mac[4],
        mac[3],
        mac[2],
        mac[1],
        mac[0],
        rc);
    return rc;
}

static void on_sync(void) {
    if(set_public_address() == 0) {
        /* Advertise with the controller's (now real) public address. */
        glue.addr_type = BLE_OWN_ADDR_PUBLIC;
        const uint8_t* mac = furi_hal_version_get_ble_mac();
        if(mac) {
            /* Keep NimBLE's cached public identity equal to the address we just
             * wrote into the controller. Otherwise the SM legacy-pairing confirm
             * value c1 (computed host-side from the responder address) uses the
             * stale BD_ADDR NimBLE read at sync, while the phone uses the real
             * on-air MAC, so the confirm mismatches and pairing fails with SM
             * error 0x04 (Confirm Value Failed). */
            ble_hs_id_set_pub(mac);
            memcpy(glue.addr, mac, sizeof(glue.addr));
        }
    } else {
        int rc = ble_hs_id_infer_auto(0, &glue.addr_type);
        if(rc != 0) {
            FURI_LOG_E(TAG, "infer_auto failed: %d", rc);
            return;
        }
        rc = ble_hs_id_copy_addr(glue.addr_type, glue.addr, NULL);
        if(rc != 0) {
            FURI_LOG_E(TAG, "copy_addr failed: %d", rc);
            return;
        }
    }
    glue.synced = true;
    FURI_LOG_I(
        TAG,
        "Host synced, addr %02X:%02X:%02X:%02X:%02X:%02X",
        glue.addr[5],
        glue.addr[4],
        glue.addr[3],
        glue.addr[2],
        glue.addr[1],
        glue.addr[0]);
    /* Probe the controller's multi-role capability once (TASK-615). Diagnostic
     * only; it does not change behavior. Its log tells whether a CoC central can
     * run alongside the companion peripheral link. */
    coc_probe_capabilities();
    start_advertise();
}

static int32_t host_task(void* context) {
    UNUSED(context);
    struct ble_npl_eventq* q = nimble_port_get_dflt_eventq();
    while(glue.host_run) {
        struct ble_npl_event* ev = ble_npl_eventq_get(q, ble_npl_time_ms_to_ticks32(100));
        if(ev) {
            ble_npl_event_run(ev);
        }
    }
    return 0;
}

bool nimble_glue_start(NimbleMode mode) {
    if(mode == NimbleModeCentral || mode == NimbleModeRawHci || mode >= NimbleModeCount) {
        FURI_LOG_E(TAG, "NimBLE mode %d not implemented", mode);
        return false;
    }

    memset(&glue, 0, sizeof(glue));
    glue.conn_handle = BLE_HS_CONN_HANDLE_NONE;
    glue.mode = mode;

    nimble_port_init();
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;

    /* Legacy passkey-display pairing with bonding, matching the stock serial
     * profile (GapPairingPinCodeShow). MITM makes the resulting LTK
     * authenticated, which the Serial Service's AUTHEN characteristics require.
     * Distribute and accept the encryption and identity keys. */
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_store_config_init();
    serial_store_load();

    const char* name = furi_hal_version_get_device_name_ptr();
    ble_svc_gap_device_name_set(name ? name : "Flipper");

    /* Register the GATT services the selected mode uses. NimBLE fixes the GATT
     * table before the host starts, so the whole set is registered here. In the
     * combined mode the Serial Service and HID coexist on one connection. */
    if(nimble_mode_has_serial(mode)) {
        int rc = serial_gatt_register();
        if(rc != 0) {
            FURI_LOG_E(TAG, "serial_gatt_register failed: %d", rc);
            return false;
        }
        serial_gatt_init();
    }

    if(nimble_mode_has_hid(mode)) {
        int rc = hid_gatt_register();
        if(rc != 0) {
            FURI_LOG_E(TAG, "hid_gatt_register failed: %d", rc);
            return false;
        }
    }

    glue.host_run = true;
    glue.host = furi_thread_alloc_ex("NimbleHost", 4096, host_task, NULL);
    if(!glue.host) {
        return false;
    }
    furi_thread_start(glue.host);
    glue.started = true;
    return true;
}

bool nimble_glue_is_synced(void) {
    return glue.synced;
}

bool nimble_glue_get_addr(uint8_t addr[6]) {
    if(!glue.synced) {
        return false;
    }
    memcpy(addr, glue.addr, 6);
    return true;
}

uint32_t nimble_glue_scan_count(void) {
    return glue.scan_count;
}

bool nimble_glue_get_last_addr(uint8_t addr[6]) {
    if(glue.scan_count == 0) {
        return false;
    }
    memcpy(addr, glue.last_addr, 6);
    return true;
}

bool nimble_glue_is_advertising(void) {
    return glue.advertising;
}

bool nimble_glue_is_connected(void) {
    return glue.connected;
}

bool nimble_glue_is_pairing(void) {
    return glue.pairing;
}

bool nimble_glue_is_bonded(void) {
    return glue.bonded;
}

uint32_t nimble_glue_passkey(void) {
    return glue.passkey;
}

uint32_t nimble_glue_rx_bytes(void) {
    return serial_gatt_rx_bytes();
}

void nimble_glue_disconnect(void) {
    if(glue.connected && glue.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(glue.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

void nimble_glue_forget_bonds(void) {
    /* Drop the in-RAM bonds and the persisted copy. */
    ble_store_clear();
    serial_store_forget();
}

/* BLE HID — thin passthrough to the HID GATT server. */
bool nimble_glue_hid_input_report(uint8_t report_id, const uint8_t* data, uint16_t len) {
    if(!nimble_mode_has_hid(glue.mode)) return false;
    return hid_gatt_input_report(report_id, data, len);
}

bool nimble_glue_hid_battery_level(uint8_t level) {
    if(!nimble_mode_has_hid(glue.mode)) return false;
    return hid_gatt_battery_level(level);
}

bool nimble_glue_faulted(void) {
    return nimble_transport_furi_faulted();
}

static void on_hs_stopped(int status, void* arg) {
    UNUSED(status);
    UNUSED(arg);
    if(stop_sem) {
        furi_semaphore_release(stop_sem);
    }
}

void nimble_glue_stop(void) {
    /* Ask the host to wind down (stops scan/connections) and wait for it. The
     * host thread must keep running here so it can process the stop, and the HCI
     * reader must keep feeding it controller responses. */
    if(glue.started) {
        stop_sem = furi_semaphore_alloc(1, 0);
        int rc = ble_hs_stop(&stop_listener, on_hs_stopped, NULL);
        if(rc == 0) {
            furi_semaphore_acquire(stop_sem, furi_ms_to_ticks(2000));
        }
        furi_semaphore_free(stop_sem);
        stop_sem = NULL;
    }

    /* Exit the host event thread. */
    glue.host_run = false;
    if(glue.host) {
        furi_thread_join(glue.host);
        furi_thread_free(glue.host);
        glue.host = NULL;
    }

    /* Close the RPC session/record and free the TX semaphore now that no host
     * callback or RPC send can still run. */
    serial_gatt_deinit();

    /* Stop the HCI reader, then free every NPL FURI object so no timer fires
     * into freed FAP memory after unload. */
    nimble_transport_furi_stop();
    npl_furi_shutdown();
    glue.started = false;
}
