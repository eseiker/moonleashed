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

#include "os/os_mbuf.h"

#include "nimble_glue.h"
#include "serial_gatt.h"
#include "serial_store.h"
#include "hid_gatt.h"
#include "coc_glue.h"
#include "gattc_glue.h"
#include "dyn_gatt.h"
#include "fixedcid_glue.h"
#include "sm_glue.h"

#define TAG "NimbleGlue"

extern void ble_store_config_init(void);
extern void npl_furi_shutdown(void);
/* Internal NimBLE setter (ble_hs_id_priv.h): overrides the host's cached public
 * identity address. We use it to keep NimBLE's SM crypto in sync with the public
 * address we write into the controller after sync. */
extern void ble_hs_id_set_pub(const uint8_t* pub_addr);
/* GATT resource counters (ble_hs_priv.h). See gatt_rebuild_now. */
extern uint16_t ble_hs_max_attrs;
extern uint16_t ble_hs_max_services;
extern uint16_t ble_hs_max_client_configs;

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
    volatile uint8_t conn_count;
    /* All peripheral link handles (the GATT rebuild must drop every link). */
    uint16_t conn_handles[MYNEWT_VAL(BLE_MAX_CONNECTIONS)];
    /* GATT table rebuild (TASK-632): pending = requested; rebuilding = links
     * are being dropped / table is being rebuilt (blocks advertising). */
    volatile bool gatt_rebuild_pending;
    volatile bool gatt_rebuilding;
    volatile bool gatt_rebuild_queued;
    volatile bool central_probe; /* Milestone 2 gate: scanning as central after suspend */
    /* Raw advertising override (TASK-646): when set, start_advertise installs this
     * payload instead of the companion fields, e.g. the DCT FC73 session adv. */
    bool has_custom_adv;
    uint8_t custom_adv[31];
    uint8_t custom_adv_len;
    uint8_t custom_rsp[31];
    uint8_t custom_rsp_len;
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
static void start_scan(void);

/* Modal central session (TASK-615 M2 / TASK-633). Peer discovered by name. */
#define DCT_PEER_NAME      "FlipperDCT"
#define CENTRAL_IDLE       0
#define CENTRAL_SCANNING   1
#define CENTRAL_CONNECTING 2
#define CENTRAL_RUNNING    3
static struct {
    volatile bool active;
    volatile int state;
    uint8_t peer[6];
    uint8_t peer_type;
    uint16_t conn_handle;
    char name[24];
    NimbleCentralCb cb;
    void* ctx;
} central;
static int central_gap_event(struct ble_gap_event* event, void* arg);
static void central_finish(void);

/* Re-arm connectable advertising when a peripheral slot is still free, so the
 * Flipper can hold a second incoming link (companion + a separate CoC central,
 * KNOW-623). No-op when already advertising or when all links are in use. The
 * controller allows advertising while a peripheral connection is up (KNOW-618,
 * supported-states bits 24/25). */
static void gatt_rebuild_post(void);
static void gatt_rebuild_event_fn(struct ble_npl_event* ev);
static struct ble_npl_event gatt_rebuild_event;

static void conn_handle_track(uint16_t handle, bool up) {
    for(size_t i = 0; i < COUNT_OF(glue.conn_handles); i++) {
        if(up && glue.conn_handles[i] == BLE_HS_CONN_HANDLE_NONE) {
            glue.conn_handles[i] = handle;
            return;
        }
        if(!up && glue.conn_handles[i] == handle) {
            glue.conn_handles[i] = BLE_HS_CONN_HANDLE_NONE;
            return;
        }
    }
}

static void maybe_advertise(void) {
    if(glue.central_probe) return; /* companion is suspended for a central scan */
    if(glue.gatt_rebuilding) return; /* table rebuild needs no GAP procedure */
    if(glue.advertising) return;
    if(glue.conn_count >= MYNEWT_VAL(BLE_MAX_CONNECTIONS)) return;
    start_advertise();
}

static int gap_event(struct ble_gap_event* event, void* arg) {
    UNUSED(arg);
    switch(event->type) {
    case BLE_GAP_EVENT_DISC:
        glue.scan_count++;
        memcpy(glue.last_addr, event->disc.addr.val, 6);
        FURI_LOG_I(
            TAG,
            "Scan #%lu: %02X:%02X:%02X:%02X:%02X:%02X type=%u rssi=%d",
            glue.scan_count,
            event->disc.addr.val[5],
            event->disc.addr.val[4],
            event->disc.addr.val[3],
            event->disc.addr.val[2],
            event->disc.addr.val[1],
            event->disc.addr.val[0],
            event->disc.addr.type,
            event->disc.rssi);
        /* During a modal central session, match the peer by advertised name,
         * then stop scanning and connect to it as central. */
        if(central.active && central.state == CENTRAL_SCANNING) {
            struct ble_hs_adv_fields f;
            size_t nlen = strlen(central.name);
            if(nlen && ble_hs_adv_parse_fields(&f, event->disc.data, event->disc.length_data) == 0 &&
               f.name != NULL && f.name_len == nlen && memcmp(f.name, central.name, nlen) == 0) {
                memcpy(central.peer, event->disc.addr.val, 6);
                central.peer_type = event->disc.addr.type;
                central.state = CENTRAL_CONNECTING;
                glue.scanning = false;
                ble_gap_disc_cancel();
                ble_addr_t peer = {.type = central.peer_type};
                memcpy(peer.val, central.peer, 6);
                int rc = ble_gap_connect(
                    glue.addr_type, &peer, 5000, NULL, central_gap_event, NULL);
                FURI_LOG_I(TAG, "Central peer '%s' found, connect rc=%d", central.name, rc);
                if(rc != 0) central_finish();
            }
        }
        break;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        FURI_LOG_I(TAG, "Scan complete (reason %d), found %lu", event->disc_complete.reason, glue.scan_count);
        glue.scanning = false;
        break;

    case BLE_GAP_EVENT_CONNECT:
        if(event->connect.status == 0) {
            glue.connected = true;
            glue.advertising = false; /* the controller stops advertising on connect */
            glue.conn_count++;
            conn_handle_track(event->connect.conn_handle, true);
            /* Install any registered fixed L2CAP channels (TASK-663). */
            fixedcid_on_connect(event->connect.conn_handle);
            /* Bind the companion services (Serial/HID) to the FIRST peripheral
             * link only. A second incoming link (e.g. a CoC-only central) must
             * not clobber the companion's GATT connection. The CoC data path is
             * bound per-channel by NimBLE, independent of glue.conn_handle. */
            if(glue.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
                glue.conn_handle = event->connect.conn_handle;
                glue.bonded = false;
                glue.pairing = false;
                if(nimble_mode_has_serial(glue.mode))
                    serial_gatt_set_conn(event->connect.conn_handle, true);
                if(nimble_mode_has_hid(glue.mode))
                    hid_gatt_set_conn(event->connect.conn_handle, true);
            }
            FURI_LOG_I(
                TAG,
                "Peripheral link up, handle %u (links=%u)",
                event->connect.conn_handle,
                glue.conn_count);
            maybe_advertise(); /* keep a slot open for a second incoming link */
        } else {
            FURI_LOG_W(TAG, "Connect failed: %d", event->connect.status);
            maybe_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT: {
        uint16_t h = event->disconnect.conn.conn_handle;
        FURI_LOG_I(TAG, "Peripheral link down, handle %u reason %d", h, event->disconnect.reason);
        if(glue.conn_count > 0) glue.conn_count--;
        conn_handle_track(h, false);
        glue.connected = (glue.conn_count > 0);
        if(h == glue.conn_handle) {
            /* The companion link dropped: tear down its services and free the
             * binding so the next first link can claim them. */
            glue.pairing = false;
            glue.bonded = false;
            glue.conn_handle = BLE_HS_CONN_HANDLE_NONE;
            if(nimble_mode_has_serial(glue.mode)) serial_gatt_set_conn(0, false);
            if(nimble_mode_has_hid(glue.mode)) hid_gatt_set_conn(0, false);
            serial_store_save(); /* persist any CCCDs written during the connection */
        }
        /* If a central scan probe is waiting for the companion to drop, start the
         * scan now that the last peripheral link is gone (KNOW-618: the ST
         * controller cannot scan while a peripheral link is up). */
        if(glue.gatt_rebuilding) {
            /* Rebuild once the last link is gone. Re-post instead of rebuilding
             * here: NimBLE may still count the dying link inside this callback. */
            if(glue.conn_count == 0) gatt_rebuild_post();
        } else if(glue.central_probe && glue.conn_count == 0) {
            start_scan();
        } else {
            maybe_advertise();
        }
        break;
    }

    case BLE_GAP_EVENT_ADV_COMPLETE:
        FURI_LOG_I(TAG, "Advertising complete, restarting");
        glue.advertising = false;
        maybe_advertise();
        break;

    case BLE_GAP_EVENT_NOTIFY_TX:
        if(nimble_mode_has_serial(glue.mode))
            serial_gatt_on_notify_tx(event->notify_tx.attr_handle, event->notify_tx.status);
        dyn_gatt_on_notify_tx(
            event->notify_tx.conn_handle,
            event->notify_tx.attr_handle,
            event->notify_tx.status,
            event->notify_tx.indication);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if(nimble_mode_has_serial(glue.mode))
            serial_gatt_on_subscribe(
                event->subscribe.attr_handle,
                event->subscribe.cur_notify || event->subscribe.cur_indicate);
        dyn_gatt_on_subscribe(
            event->subscribe.conn_handle,
            event->subscribe.attr_handle,
            event->subscribe.cur_notify,
            event->subscribe.cur_indicate);
        break;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        glue.pairing = true;
        /* A registered security consumer (TASK-665) drives pairing instead. */
        if(sm_glue_on_passkey_action(
               event->passkey.conn_handle,
               event->passkey.params.action,
               event->passkey.params.numcmp))
            break;
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
        sm_glue_on_enc_change(event->enc_change.conn_handle, event->enc_change.status);
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
        sm_glue_on_repeat_pairing(event->repeat_pairing.conn_handle);
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
    /* Raw override (TASK-646): a FAP installed its own payload (e.g. the DCT
     * FC73 session advertisement). Use it verbatim instead of the companion
     * fields; nimble_glue_adv_clear restores the companion advertisement. */
    if(glue.has_custom_adv) {
        int rc = ble_gap_adv_set_data(glue.custom_adv, glue.custom_adv_len);
        if(rc != 0) {
            FURI_LOG_E(TAG, "adv_set_data(raw) failed: %d", rc);
            return;
        }
        ble_gap_adv_rsp_set_data(glue.custom_rsp, glue.custom_rsp_len);
        struct ble_gap_adv_params advp;
        memset(&advp, 0, sizeof(advp));
        advp.conn_mode = BLE_GAP_CONN_MODE_UND;
        advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
        rc = ble_gap_adv_start(glue.addr_type, NULL, BLE_HS_FOREVER, &advp, gap_event, NULL);
        if(rc != 0) {
            FURI_LOG_E(TAG, "adv_start(raw) failed: %d", rc);
            return;
        }
        glue.advertising = true;
        FURI_LOG_I(
            TAG, "Advertising started (raw %u/%u bytes)", glue.custom_adv_len, glue.custom_rsp_len);
        return;
    }

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

/* Start an active scan as observer/central. Only legal on HCILayer once no
 * peripheral link is up (KNOW-618). Results arrive as BLE_GAP_EVENT_DISC. */
static void start_scan(void) {
    struct ble_gap_disc_params dp;
    memset(&dp, 0, sizeof(dp));
    dp.passive = 0; /* active scan: also pull scan responses (names) */
    dp.filter_duplicates = 0; /* count every advertisement, including repeats */
    dp.itvl = 0; /* controller default */
    dp.window = 0;
    int rc = ble_gap_disc(glue.addr_type, BLE_HS_FOREVER, &dp, gap_event, NULL);
    if(rc != 0) {
        /* A non-zero rc here means the controller rejected central scanning —
         * the gating fact for the Flipper-as-central (modal time-share) case. */
        FURI_LOG_E(TAG, "ble_gap_disc failed: %d (central scan not available?)", rc);
        glue.scanning = false;
        return;
    }
    glue.scanning = true;
    FURI_LOG_I(TAG, "Central scan started");
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
    for(size_t i = 0; i < COUNT_OF(glue.conn_handles); i++)
        glue.conn_handles[i] = BLE_HS_CONN_HANDLE_NONE;
    glue.mode = mode;
    dyn_gatt_init();

    nimble_port_init();
    ble_npl_event_init(&gatt_rebuild_event, gatt_rebuild_event_fn, NULL);
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

    /* L2CAP CoC server (TASK-615): the DCT channel, registered alongside the GATT
     * services so a peer can open a CoC on a peripheral link. Non-fatal if it
     * fails — the companion GATT still runs. */
    if(coc_server_start() != 0) {
        FURI_LOG_W(TAG, "CoC server registration failed");
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

bool nimble_glue_is_scanning(void) {
    return glue.scanning;
}

bool nimble_glue_adv_set_raw(
    const uint8_t* adv,
    uint8_t adv_len,
    const uint8_t* rsp,
    uint8_t rsp_len) {
    if(adv_len > sizeof(glue.custom_adv) || rsp_len > sizeof(glue.custom_rsp)) return false;
    if((adv_len && !adv) || (rsp_len && !rsp)) return false;
    if(adv_len) memcpy(glue.custom_adv, adv, adv_len);
    glue.custom_adv_len = adv_len;
    if(rsp_len) memcpy(glue.custom_rsp, rsp, rsp_len);
    glue.custom_rsp_len = rsp_len;
    glue.has_custom_adv = true;
    /* Re-advertise with the new payload (maybe_advertise respects the central
     * suspend and the link budget). */
    if(glue.advertising) {
        ble_gap_adv_stop();
        glue.advertising = false;
    }
    maybe_advertise();
    return true;
}

void nimble_glue_adv_clear(void) {
    if(!glue.has_custom_adv) return;
    glue.has_custom_adv = false;
    glue.custom_adv_len = 0;
    glue.custom_rsp_len = 0;
    if(glue.advertising) {
        ble_gap_adv_stop();
        glue.advertising = false;
    }
    maybe_advertise(); /* back to the companion advertisement */
}

bool nimble_glue_central_probe_start(void) {
    if(!glue.synced) return false;
    if(glue.central_probe) return false; /* already probing */
    glue.central_probe = true;
    glue.scan_count = 0;
    /* Suspend the companion: stop advertising and drop the bound peripheral link.
     * The scan begins from the DISCONNECT handler once conn_count reaches 0, or
     * immediately when nothing is connected. */
    if(glue.advertising) {
        ble_gap_adv_stop();
        glue.advertising = false;
    }
    if(glue.conn_count > 0 && glue.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(glue.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    if(glue.conn_count == 0) {
        start_scan();
    }
    return true;
}

void nimble_glue_central_probe_stop(void) {
    if(!glue.central_probe) return;
    if(glue.scanning) {
        ble_gap_disc_cancel();
        glue.scanning = false;
    }
    glue.central_probe = false;
    maybe_advertise(); /* restore the companion */
}

/* End the modal central session and restore the companion. */
static void central_finish(void) {
    central.active = false;
    central.state = CENTRAL_IDLE;
    central.conn_handle = BLE_HS_CONN_HANDLE_NONE;
    central.cb = NULL;
    central.ctx = NULL;
    glue.central_probe = false; /* let the companion advertise again */
    glue.scanning = false;
    maybe_advertise();
    FURI_LOG_I(TAG, "Central session ended, companion restored");
    /* A GATT rebuild requested during the central session was deferred. */
    if(glue.gatt_rebuild_pending) gatt_rebuild_post();
}

/* ---- GATT table rebuild (TASK-632) ------------------------------------------
 * NimBLE only accepts service changes while ble_gatts_mutable(): no links and no
 * GAP procedure. The rebuild runs on the host thread through an NPL event:
 * stop advertising, drop every peripheral link, and once the last one is gone
 * reset the table, re-register the built-in services in their boot order (so
 * their handles, and bonded peers' cached handles and CCCDs, do not move), then
 * the dynamic services, start the table and send Service Changed. A central
 * session defers the rebuild until it ends. */

static void gatt_rebuild_now(void) {
    int rc = ble_gatts_reset();
    if(rc != 0) {
        FURI_LOG_E(TAG, "GATT rebuild: reset failed %d", rc);
        glue.gatt_rebuild_pending = false;
        glue.gatt_rebuilding = false;
        maybe_advertise();
        return;
    }
    /* NimBLE 1.10.0's ble_gatts_reset() leaves the resource counters that
     * ble_gatts_count_cfg() accumulates, so every rebuild would re-count the
     * whole table on top and grow the attribute/service/CCCD pools until the
     * heap runs out. Upstream fixed this after 1.10.0 (af4baa41, "reset
     * resource counts in ble_gatts_reset()"); until the pinned tag includes it,
     * clear them here. Harmless on versions that already do. */
    ble_hs_max_attrs = 0;
    ble_hs_max_services = 0;
    ble_hs_max_client_configs = 0;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    if(nimble_mode_has_serial(glue.mode)) serial_gatt_register();
    if(nimble_mode_has_hid(glue.mode)) hid_gatt_register();
    dyn_gatt_register_all();
    rc = ble_gatts_start();
    if(rc == 0) {
        dyn_gatt_after_start();
        ble_svc_gatt_changed(0x0001, 0xFFFF);
        FURI_LOG_I(TAG, "GATT table rebuilt");
    } else {
        FURI_LOG_E(TAG, "GATT rebuild: start failed %d", rc);
    }
    glue.gatt_rebuilding = false;
    /* Definitions changed while we rebuilt: go again. */
    glue.gatt_rebuild_pending = dyn_gatt_dirty();
    if(glue.gatt_rebuild_pending) {
        gatt_rebuild_post();
    } else {
        maybe_advertise();
    }
}

static void gatt_rebuild_event_fn(struct ble_npl_event* ev) {
    UNUSED(ev);
    glue.gatt_rebuild_queued = false;
    if(!glue.gatt_rebuild_pending || !glue.synced) return;
    if(central.active) {
        FURI_LOG_I(TAG, "GATT rebuild deferred until the central session ends");
        return;
    }
    if(!glue.gatt_rebuilding) {
        glue.gatt_rebuilding = true;
        if(glue.advertising) {
            ble_gap_adv_stop();
            glue.advertising = false;
        }
        for(size_t i = 0; i < COUNT_OF(glue.conn_handles); i++) {
            if(glue.conn_handles[i] != BLE_HS_CONN_HANDLE_NONE)
                ble_gap_terminate(glue.conn_handles[i], BLE_ERR_REM_USER_CONN_TERM);
        }
        if(glue.conn_count)
            FURI_LOG_I(TAG, "GATT rebuild: dropping %u link(s)", glue.conn_count);
    }
    if(glue.conn_count == 0) gatt_rebuild_now();
}

static void gatt_rebuild_post(void) {
    if(glue.gatt_rebuild_queued) return;
    glue.gatt_rebuild_queued = true;
    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &gatt_rebuild_event);
}

bool nimble_glue_gatt_rebuild_request(void) {
    if(!glue.started) return false;
    glue.gatt_rebuild_pending = true;
    gatt_rebuild_post();
    return true;
}

/* GAP events for the modal central link. Kept separate from gap_event so the
 * central connection never binds the companion Serial/HID services. Lifecycle is
 * reported to central.cb; the caller drives the link (GATT client / CoC). */
static int central_gap_event(struct ble_gap_event* event, void* arg) {
    UNUSED(arg);
    switch(event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if(event->connect.status == 0) {
            central.conn_handle = event->connect.conn_handle;
            central.state = CENTRAL_RUNNING;
            fixedcid_on_connect(event->connect.conn_handle);
            FURI_LOG_I(TAG, "Central link up handle=%u", central.conn_handle);
            if(central.cb) central.cb(NimbleCentralConnected, central.conn_handle, 0, central.ctx);
        } else {
            FURI_LOG_W(TAG, "Central connect failed: %d", event->connect.status);
            NimbleCentralCb cb = central.cb;
            void* ctx = central.ctx;
            central_finish();
            if(cb) cb(NimbleCentralFailed, 0, event->connect.status, ctx);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT: {
        FURI_LOG_I(TAG, "Central link down, reason %d", event->disconnect.reason);
        NimbleCentralCb cb = central.cb;
        void* ctx = central.ctx;
        uint16_t h = central.conn_handle;
        int reason = event->disconnect.reason;
        central_finish();
        if(cb) cb(NimbleCentralDisconnected, h, reason, ctx);
        return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        /* Forward notifications/indications on the central link to the GATT
         * client core so ble_gatt_client_* subscribers receive them. */
        static uint8_t nbuf[512];
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if(len > sizeof(nbuf)) len = sizeof(nbuf);
        if(len && os_mbuf_copydata(event->notify_rx.om, 0, len, nbuf) == 0) {
            gattc_api_on_notify(
                event->notify_rx.conn_handle, event->notify_rx.attr_handle, nbuf, len);
        }
        return 0;
    }

    /* Pairing on the central link is only ever driven by a security consumer
     * (TASK-665); the companion's passkey-display path is peripheral-only. */
    case BLE_GAP_EVENT_PASSKEY_ACTION:
        sm_glue_on_passkey_action(
            event->passkey.conn_handle,
            event->passkey.params.action,
            event->passkey.params.numcmp);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        sm_glue_on_enc_change(event->enc_change.conn_handle, event->enc_change.status);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        sm_glue_on_repeat_pairing(event->repeat_pairing.conn_handle);
        struct ble_gap_conn_desc desc;
        if(ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_gap_unpair(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    default:
        return 0;
    }
}

bool nimble_glue_central_start(const char* name, NimbleCentralCb cb, void* ctx) {
    if(!glue.synced || central.active) return false;
    if(glue.gatt_rebuild_pending || glue.gatt_rebuilding) return false; /* table in flux */
    memset(&central, 0, sizeof(central));
    central.active = true;
    central.state = CENTRAL_SCANNING;
    central.conn_handle = BLE_HS_CONN_HANDLE_NONE;
    central.cb = cb;
    central.ctx = ctx;
    strncpy(central.name, name ? name : "", sizeof(central.name) - 1);
    /* Suspend the companion; the scan starts from the DISCONNECT handler once the
     * last peripheral link is gone (or immediately if nothing is connected). */
    glue.central_probe = true;
    glue.scan_count = 0;
    if(glue.advertising) {
        ble_gap_adv_stop();
        glue.advertising = false;
    }
    if(glue.conn_count > 0 && glue.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(glue.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    if(glue.conn_count == 0) {
        start_scan();
    }
    FURI_LOG_I(TAG, "Central session started, scanning for '%s'", central.name);
    return true;
}

void nimble_glue_central_stop(void) {
    if(!central.active) return;
    if(central.state == CENTRAL_RUNNING && central.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        /* Drop the central link; central_gap_event's DISCONNECT runs central_finish. */
        ble_gap_terminate(central.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    } else {
        if(glue.scanning) ble_gap_disc_cancel();
        central_finish();
    }
}

bool nimble_glue_central_is_active(void) {
    return central.active;
}

uint16_t nimble_glue_central_conn_handle(void) {
    return central.conn_handle;
}

/* --- DCT helper: a central session that opens a CoC client on connect -------- */

static uint16_t dct_psm;

static void dct_central_cb(NimbleCentralEventKind kind, uint16_t conn, int status, void* ctx) {
    UNUSED(status);
    UNUSED(ctx);
    if(kind == NimbleCentralConnected) {
        FURI_LOG_I(TAG, "DCT: opening CoC PSM 0x%04X on handle %u", dct_psm, conn);
        int rc = coc_client_connect(conn, dct_psm);
        if(rc != 0) {
            FURI_LOG_E(TAG, "coc_client_connect rc=%d", rc);
            ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
}

bool nimble_glue_dct_connect(uint16_t psm) {
    dct_psm = psm;
    return nimble_glue_central_start(DCT_PEER_NAME, dct_central_cb, NULL);
}

void nimble_glue_dct_stop(void) {
    nimble_glue_central_stop();
}

bool nimble_glue_dct_is_active(void) {
    return nimble_glue_central_is_active();
}

uint32_t nimble_glue_dct_rx_bytes(void) {
    return coc_rx_bytes();
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

int nimble_glue_link_terminate(uint16_t conn_handle, uint8_t reason) {
    if(conn_handle == BLE_HS_CONN_HANDLE_NONE) return BLE_HS_ENOTCONN;
    int rc = ble_gap_terminate(conn_handle, reason ? reason : BLE_ERR_REM_USER_CONN_TERM);
    FURI_LOG_I(TAG, "Terminate link %u reason 0x%02X rc=%d", conn_handle, reason, rc);
    return rc;
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
