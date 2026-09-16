#include "bt_i.h"
#include "bt_keys_storage.h"

#include <core/check.h>
#include <furi_hal_bt.h>
#include <furi_hal_bt_hci.h>
#include <services/battery_service.h>
#include <notification/notification_messages.h>
#include <gui/elements.h>
#include <assets_icons.h>
#include <profiles/serial_profile.h>
#include <gatt_host_shim.h>
#include <ble_dispatch.h>
#include <nimble_glue.h>

#define TAG "BtSrv"

#define BT_RPC_EVENT_BUFF_SENT    (1UL << 0)
#define BT_RPC_EVENT_DISCONNECTED (1UL << 1)
#define BT_RPC_EVENT_ALL          (BT_RPC_EVENT_BUFF_SENT | BT_RPC_EVENT_DISCONNECTED)

#define ICON_SPACER 2

static void bt_draw_statusbar_callback(Canvas* canvas, void* context) {
    furi_assert(context);

    Bt* bt = context;
    uint8_t draw_offset = 0;
    if(bt->beacon_active) {
        canvas_draw_icon(canvas, 0, 0, &I_BLE_beacon_7x8);
        draw_offset += icon_get_width(&I_BLE_beacon_7x8) + ICON_SPACER;
    }
    if(bt->status == BtStatusAdvertising) {
        canvas_draw_icon(canvas, draw_offset, 0, &I_Bluetooth_Idle_5x8);
    } else if(bt->status == BtStatusConnected) {
        canvas_draw_icon(canvas, draw_offset, 0, &I_Bluetooth_Connected_16x8);
    }
}

static ViewPort* bt_statusbar_view_port_alloc(Bt* bt) {
    ViewPort* statusbar_view_port = view_port_alloc();
    view_port_set_width(statusbar_view_port, 5);
    view_port_draw_callback_set(statusbar_view_port, bt_draw_statusbar_callback, bt);
    view_port_enabled_set(statusbar_view_port, false);
    return statusbar_view_port;
}

static void bt_pin_code_view_port_draw_callback(Canvas* canvas, void* context) {
    furi_assert(context);
    Bt* bt = context;
    char pin_code_info[24];
    canvas_draw_icon(canvas, 0, 0, &I_BLE_Pairing_128x64);
    snprintf(pin_code_info, sizeof(pin_code_info), "Pairing code\n%06lu", bt->pin_code);
    elements_multiline_text_aligned(canvas, 64, 4, AlignCenter, AlignTop, pin_code_info);
    elements_button_left(canvas, "Quit");
}

static void bt_pin_code_view_port_input_callback(InputEvent* event, void* context) {
    furi_assert(context);
    Bt* bt = context;
    if(event->type == InputTypeShort) {
        if(event->key == InputKeyLeft || event->key == InputKeyBack) {
            view_port_enabled_set(bt->pin_code_view_port, false);
        }
    }
}

static void bt_storage_callback(const void* message, void* context) {
    furi_assert(context);
    Bt* bt = context;
    const StorageEvent* event = message;

    if(event->type == StorageEventTypeCardMount) {
        const BtMessage msg = {
            .type = BtMessageTypeReloadKeysSettings,
        };

        furi_check(
            furi_message_queue_put(bt->message_queue, &msg, FuriWaitForever) == FuriStatusOk);
    }
}

static ViewPort* bt_pin_code_view_port_alloc(Bt* bt) {
    ViewPort* view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, bt_pin_code_view_port_draw_callback, bt);
    view_port_input_callback_set(view_port, bt_pin_code_view_port_input_callback, bt);
    view_port_enabled_set(view_port, false);
    return view_port;
}

static void bt_pin_code_show(Bt* bt, uint32_t pin_code) {
    bt->pin_code = pin_code;
    if(!bt->pin_code_view_port) {
        // Pin code view port
        bt->pin_code_view_port = bt_pin_code_view_port_alloc(bt);
        gui_add_view_port(bt->gui, bt->pin_code_view_port, GuiLayerFullscreen);
    }
    notification_message(bt->notification, &sequence_display_backlight_on);
    if(bt->suppress_pin_screen) return;

    gui_view_port_send_to_front(bt->gui, bt->pin_code_view_port);
    view_port_enabled_set(bt->pin_code_view_port, true);
}

static void bt_pin_code_hide(Bt* bt) {
    bt->pin_code = 0;
    if(bt->pin_code_view_port && view_port_is_enabled(bt->pin_code_view_port)) {
        view_port_enabled_set(bt->pin_code_view_port, false);
    }
}

static bool bt_pin_code_verify_event_handler(Bt* bt, uint32_t pin) {
    furi_assert(bt);
    bt->pin_code = pin;
    notification_message(bt->notification, &sequence_display_backlight_on);
    if(bt->suppress_pin_screen) return true;

    FuriString* pin_str;
    if(!bt->dialog_message) {
        bt->dialog_message = dialog_message_alloc();
    }
    dialog_message_set_icon(bt->dialog_message, &I_BLE_Pairing_128x64, 0, 0);
    pin_str = furi_string_alloc_printf("Verify code\n%06lu", pin);
    dialog_message_set_text(
        bt->dialog_message, furi_string_get_cstr(pin_str), 64, 4, AlignCenter, AlignTop);
    dialog_message_set_buttons(bt->dialog_message, "Cancel", "OK", NULL);
    DialogMessageButton button = dialog_message_show(bt->dialogs, bt->dialog_message);
    furi_string_free(pin_str);
    return button == DialogMessageButtonCenter;
}

static void bt_battery_level_changed_callback(const void* _event, void* context) {
    furi_assert(_event);
    furi_assert(context);

    Bt* bt = context;
    BtMessage message = {};
    const PowerEvent* event = _event;
    bool is_charging = false;
    switch(event->type) {
    case PowerEventTypeBatteryLevelChanged:
        message.type = BtMessageTypeUpdateBatteryLevel;
        message.data.battery_level = event->data.battery_level;
        furi_check(
            furi_message_queue_put(bt->message_queue, &message, FuriWaitForever) == FuriStatusOk);
        break;
    case PowerEventTypeStartCharging:
        is_charging = true;
        /* fallthrough */
    case PowerEventTypeFullyCharged:
    case PowerEventTypeStopCharging:
        message.type = BtMessageTypeUpdatePowerState;
        message.data.power_state_charging = is_charging;
        furi_check(
            furi_message_queue_put(bt->message_queue, &message, FuriWaitForever) == FuriStatusOk);
        break;
    }
}

Bt* bt_alloc(void) {
    Bt* bt = malloc(sizeof(Bt));
    // Init default maximum packet size
    bt->max_packet_size = BLE_PROFILE_SERIAL_PACKET_SIZE_MAX;
    bt->current_profile = NULL;
    bt->profile_suspended = false;
    // Resident NimBLE host state (set true only if bt_nimble_bringup succeeds).
    // Bt is malloc'd, not zeroed, so initialise explicitly.
    bt->nimble_active = false;
    bt->nimble_timer = NULL;
    bt->nimble_last_status = BtStatusUnavailable;
    bt->nimble_pin_shown = false;
    bt->nimble_profile_started = false;
    // Keys storage
    bt->keys_storage = bt_keys_storage_alloc(BT_KEYS_STORAGE_PATH);
    // Alloc queue
    bt->message_queue = furi_message_queue_alloc(8, sizeof(BtMessage));

    // Setup statusbar view port
    bt->statusbar_view_port = bt_statusbar_view_port_alloc(bt);
    // Notification
    bt->notification = furi_record_open(RECORD_NOTIFICATION);
    // Gui
    bt->gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(bt->gui, bt->statusbar_view_port, GuiLayerStatusBarLeft);

    // Dialogs
    bt->dialogs = furi_record_open(RECORD_DIALOGS);

    // Power
    bt->power = furi_record_open(RECORD_POWER);
    FuriPubSub* power_pubsub = power_get_pubsub(bt->power);
    furi_pubsub_subscribe(power_pubsub, bt_battery_level_changed_callback, bt);

    // RPC
    bt->rpc = furi_record_open(RECORD_RPC);
    bt->rpc_event = furi_event_flag_alloc();

    // API evnent
    bt->api_event = furi_event_flag_alloc();

    bt->pin = 0;

    return bt;
}

// Called from GAP thread from Serial service
static uint16_t bt_serial_event_callback(SerialServiceEvent event, void* context) {
    furi_assert(context);
    Bt* bt = context;
    uint16_t ret = 0;

    if(event.event == SerialServiceEventTypeDataReceived) {
        size_t bytes_processed =
            rpc_session_feed(bt->rpc_session, event.data.buffer, event.data.size, 1000);
        if(bytes_processed != event.data.size) {
            FURI_LOG_E(
                TAG, "Only %zu of %u bytes processed by RPC", bytes_processed, event.data.size);
        }
        ret = rpc_session_get_available_size(bt->rpc_session);
    } else if(event.event == SerialServiceEventTypeDataSent) {
        furi_event_flag_set(bt->rpc_event, BT_RPC_EVENT_BUFF_SENT);
    } else if(event.event == SerialServiceEventTypesBleResetRequest) {
        FURI_LOG_I(TAG, "BLE restart request received");
        BtMessage message = {
            .type = BtMessageTypeSetProfile,
            .data.profile.params = NULL,
            .data.profile.template = ble_profile_serial,
        };
        furi_check(
            furi_message_queue_put(bt->message_queue, &message, FuriWaitForever) == FuriStatusOk);
    }
    return ret;
}

// Called from RPC thread
static void bt_rpc_send_bytes_callback(void* context, uint8_t* bytes, size_t bytes_len) {
    furi_assert(context);
    Bt* bt = context;

    if(furi_event_flag_get(bt->rpc_event) & BT_RPC_EVENT_DISCONNECTED) {
        // Early stop from sending if we're already disconnected
        return;
    }
    furi_event_flag_clear(bt->rpc_event, BT_RPC_EVENT_ALL & (~BT_RPC_EVENT_DISCONNECTED));
    size_t bytes_sent = 0;
    while(bytes_sent < bytes_len) {
        size_t bytes_remain = bytes_len - bytes_sent;
        if(bytes_remain > bt->max_packet_size) {
            ble_profile_serial_tx(bt->current_profile, &bytes[bytes_sent], bt->max_packet_size);
            bytes_sent += bt->max_packet_size;
        } else {
            ble_profile_serial_tx(bt->current_profile, &bytes[bytes_sent], bytes_remain);
            bytes_sent += bytes_remain;
        }
        // We want BT_RPC_EVENT_DISCONNECTED to stick, so don't clear
        uint32_t event_flag = furi_event_flag_wait(
            bt->rpc_event, BT_RPC_EVENT_ALL, FuriFlagWaitAny | FuriFlagNoClear, FuriWaitForever);
        if(event_flag & BT_RPC_EVENT_DISCONNECTED) {
            break;
        } else {
            // If we didn't get BT_RPC_EVENT_DISCONNECTED, then clear everything else
            furi_event_flag_clear(bt->rpc_event, BT_RPC_EVENT_ALL & (~BT_RPC_EVENT_DISCONNECTED));
        }
    }
}

static void bt_serial_buffer_is_empty_callback(void* context) {
    furi_assert(context);
    Bt* bt = context;
    furi_check(furi_hal_bt_check_profile_type(bt->current_profile, ble_profile_serial));
    ble_profile_serial_notify_buffer_is_empty(bt->current_profile);
}

// Called from GAP thread
static bool bt_on_gap_event_callback(GapEvent event, void* context) {
    furi_assert(context);
    Bt* bt = context;
    bool ret = false;
    bt->pin = 0;
    bool do_update_status = false;
    bool current_profile_is_serial =
        furi_hal_bt_check_profile_type(bt->current_profile, ble_profile_serial);

    if(event.type == GapEventTypeConnected) {
        // Update status bar
        bt->status = BtStatusConnected;
        do_update_status = true;
        bt_open_rpc_connection(bt);
        // Update battery level
        PowerInfo info;
        power_get_info(bt->power, &info);
        BtMessage message = {.type = BtMessageTypeUpdateStatus};
        message.type = BtMessageTypeUpdateBatteryLevel;
        message.data.battery_level = info.charge;
        furi_check(
            furi_message_queue_put(bt->message_queue, &message, FuriWaitForever) == FuriStatusOk);
        ret = true;
    } else if(event.type == GapEventTypeDisconnected) {
        if(current_profile_is_serial && bt->rpc_session) {
            FURI_LOG_I(TAG, "Close RPC connection");
            ble_profile_serial_set_rpc_active(
                bt->current_profile, FuriHalBtSerialRpcStatusNotActive);
            furi_event_flag_set(bt->rpc_event, BT_RPC_EVENT_DISCONNECTED);
            rpc_session_close(bt->rpc_session);
            ble_profile_serial_set_event_callback(bt->current_profile, 0, NULL, NULL);
            bt->rpc_session = NULL;
        }
        ret = true;
    } else if(event.type == GapEventTypeStartAdvertising) {
        bt->status = BtStatusAdvertising;
        do_update_status = true;
        ret = true;
    } else if(event.type == GapEventTypeStopAdvertising) {
        bt->status = BtStatusOff;
        do_update_status = true;
        ret = true;
    } else if(event.type == GapEventTypePinCodeShow) {
        bt->pin = event.data.pin_code;
        BtMessage message = {
            .type = BtMessageTypePinCodeShow, .data.pin_code = event.data.pin_code};
        furi_check(
            furi_message_queue_put(bt->message_queue, &message, FuriWaitForever) == FuriStatusOk);
        ret = true;
    } else if(event.type == GapEventTypePinCodeVerify) {
        bt->pin = event.data.pin_code;
        ret = bt_pin_code_verify_event_handler(bt, event.data.pin_code);
    } else if(event.type == GapEventTypeUpdateMTU) {
        bt->max_packet_size = event.data.max_packet_size;
        ret = true;
    } else if(event.type == GapEventTypeBeaconStart) {
        bt->beacon_active = true;
        do_update_status = true;
        ret = true;
    } else if(event.type == GapEventTypeBeaconStop) {
        bt->beacon_active = false;
        do_update_status = true;
        ret = true;
    }

    if(do_update_status) {
        BtMessage message = {.type = BtMessageTypeUpdateStatus};
        furi_check(
            furi_message_queue_put(bt->message_queue, &message, FuriWaitForever) == FuriStatusOk);
    }
    return ret;
}

static void bt_on_key_storage_change_callback(uint8_t* addr, uint16_t size, void* context) {
    furi_assert(context);
    Bt* bt = context;
    BtMessage message = {
        .type = BtMessageTypeKeysStorageUpdated,
        .data.key_storage_data.start_address = addr,
        .data.key_storage_data.size = size};
    furi_check(
        furi_message_queue_put(bt->message_queue, &message, FuriWaitForever) == FuriStatusOk);
}

static void bt_statusbar_update(Bt* bt) {
    uint8_t active_icon_width = 0;
    if(bt->beacon_active) {
        active_icon_width = icon_get_width(&I_BLE_beacon_7x8) + ICON_SPACER;
    }
    if(bt->status == BtStatusAdvertising) {
        active_icon_width += icon_get_width(&I_Bluetooth_Idle_5x8);
    } else if(bt->status == BtStatusConnected) {
        active_icon_width += icon_get_width(&I_Bluetooth_Connected_16x8);
    }

    if(active_icon_width > 0) {
        view_port_set_width(bt->statusbar_view_port, active_icon_width);
        view_port_enabled_set(bt->statusbar_view_port, true);
    } else {
        view_port_enabled_set(bt->statusbar_view_port, false);
    }
}

static void bt_show_warning(Bt* bt, const char* text) {
    if(!bt->dialog_message) {
        bt->dialog_message = dialog_message_alloc();
    }
    dialog_message_set_text(bt->dialog_message, text, 64, 28, AlignCenter, AlignCenter);
    dialog_message_set_buttons(bt->dialog_message, "Quit", NULL, NULL);
    dialog_message_show(bt->dialogs, bt->dialog_message);
}

void bt_open_rpc_connection(Bt* bt) {
    if(!bt->rpc_session && bt->status == BtStatusConnected) {
        // Clear BT_RPC_EVENT_DISCONNECTED because it might be set from previous session
        furi_event_flag_clear(bt->rpc_event, BT_RPC_EVENT_DISCONNECTED);
        if(furi_hal_bt_check_profile_type(bt->current_profile, ble_profile_serial)) {
            // Open RPC session
            bt->rpc_session = rpc_session_open(bt->rpc, RpcOwnerBle);
            if(bt->rpc_session) {
                FURI_LOG_I(TAG, "Open RPC connection");
                rpc_session_set_send_bytes_callback(bt->rpc_session, bt_rpc_send_bytes_callback);
                rpc_session_set_buffer_is_empty_callback(
                    bt->rpc_session, bt_serial_buffer_is_empty_callback);
                rpc_session_set_context(bt->rpc_session, bt);
                ble_profile_serial_set_event_callback(
                    bt->current_profile, RPC_BUFFER_SIZE, bt_serial_event_callback, bt);
                ble_profile_serial_set_rpc_active(
                    bt->current_profile, FuriHalBtSerialRpcStatusActive);
            } else {
                FURI_LOG_W(TAG, "RPC is busy, failed to open new session");
            }
        }
    }
}

void bt_close_rpc_connection(Bt* bt) {
    if(furi_hal_bt_check_profile_type(bt->current_profile, ble_profile_serial) &&
       bt->rpc_session) {
        FURI_LOG_I(TAG, "Close RPC connection");
        furi_event_flag_set(bt->rpc_event, BT_RPC_EVENT_DISCONNECTED);
        rpc_session_close(bt->rpc_session);
        ble_profile_serial_set_event_callback(bt->current_profile, 0, NULL, NULL);
        bt->rpc_session = NULL;
    }
}

/* NimBLE host: stop the app profile instance started by bt_nimble_change_profile,
 * mirroring furi_hal_bt_reinit (current_profile->config->stop). The serial
 * sentinel is not an instance and needs no stop. */
static void bt_nimble_stop_profile(Bt* bt) {
    if(bt->nimble_profile_started && bt->current_profile) {
        FURI_LOG_I(TAG, "NimBLE: stopping app profile");
        bt->current_profile->config->stop(bt->current_profile);
    }
    bt->nimble_profile_started = false;
    bt->current_profile = NULL;
}

/* Backward compatibility (KNOW-609, KNOW-613, TASK-612): HID FAPs link their own
 * copy of lib/ble_profile, so the only firmware code they reach is bt_profile_*
 * and the exported ble_gatt_* primitives. With the GATT host shim installed at
 * bring-up, we run the requested template's start() exactly like the stock
 * furi_hal_bt_start_app would: the FAP's ble_profile_hid_start builds its
 * BleProfileHid over the shim (no CPU2 traffic), and its later ble_profile_hid_*
 * senders pass their own `profile->config == ble_profile_hid` check and push
 * reports through ble_gatt_characteristic_update -> shim -> NimBLE. Never call
 * furi_hal_bt_change_app here: it would reinit CPU2 and break NimBLE's
 * controller ownership. The serial profile is served natively by NimBLE, so
 * ble_profile_serial gets a sentinel handle instead of a started instance. */
static void bt_nimble_change_profile(Bt* bt, BtMessage* message) {
    const FuriHalBleProfileTemplate* template = message->data.profile.template;
    FuriHalBleProfileBase* instance = NULL;

    bt_nimble_stop_profile(bt);

    if(template == ble_profile_serial || !template) {
        static FuriHalBleProfileBase nimble_serial_profile;
        nimble_serial_profile.config = ble_profile_serial;
        instance = &nimble_serial_profile;
    } else if(ble_gatt_host_shim_active() && template->start) {
        instance = template->start(message->data.profile.params);
        bt->nimble_profile_started = (instance != NULL);
        FURI_LOG_D(TAG, "NimBLE: app profile start -> %p", (void*)instance);
    } else {
        FURI_LOG_E(TAG, "NimBLE: GATT host shim not installed; profile unavailable");
    }

    bt->current_profile = instance;
    if(message->profile_instance) *message->profile_instance = instance;
    if(message->result) *message->result = instance != NULL;
}

static void bt_change_profile(Bt* bt, BtMessage* message) {
    if(bt->nimble_active) {
        bt_nimble_change_profile(bt, message);
        return;
    }

    if(furi_hal_bt_is_gatt_gap_supported()) {
        bt_settings_load(&bt->bt_settings);

        bt_close_rpc_connection(bt);

        bt_keys_storage_load(bt->keys_storage);

        bt->current_profile = furi_hal_bt_change_app(
            message->data.profile.template,
            message->data.profile.params,
            bt_keys_storage_get_root_keys(bt->keys_storage),
            bt_on_gap_event_callback,
            bt);
        if(bt->current_profile) {
            FURI_LOG_I(TAG, "Bt App started");
            if(bt->bt_settings.enabled) {
                furi_hal_bt_start_advertising();
            }
            furi_hal_bt_set_key_storage_change_callback(bt_on_key_storage_change_callback, bt);
        } else {
            FURI_LOG_E(TAG, "Failed to start Bt App");
        }
        if(message->profile_instance) {
            *message->profile_instance = bt->current_profile;
        }
        if(message->result) {
            *message->result = bt->current_profile != NULL;
        }

    } else {
        bt_show_warning(bt, "Radio stack doesn't support this app");
        if(message->result) {
            *message->result = false;
        }
        if(message->profile_instance) {
            *message->profile_instance = NULL;
        }
    }
}

static void bt_close_connection(Bt* bt);

static void bt_suspend_profile(Bt* bt, BtMessage* message) {
    bool result = !bt->profile_suspended;
    if(result) {
        bt_close_connection(bt);
        if(bt->nimble_active) {
            bt_nimble_stop_profile(bt);
        }
        /* furi_hal_bt_enter_ll_only() owns and frees the HAL profile next. */
        bt->current_profile = NULL;
        bt->profile_suspended = true;
    }
    if(message->result) *message->result = result;
}

static void bt_resume_default_profile(Bt* bt, BtMessage* message) {
    bool result = false;
    if(bt->profile_suspended) {
        BtMessage profile_message = {
            .data.profile.params = NULL,
            .data.profile.template = ble_profile_serial,
            .result = &result,
        };
        bt_change_profile(bt, &profile_message);
        if(result) bt->profile_suspended = false;
    }
    if(message->result) *message->result = result;
}

static void bt_close_connection(Bt* bt) {
    if(bt->nimble_active) {
        /* On the NimBLE host, drop the link through the glue; never touch the
         * stock CPU2 advertising path. */
        nimble_glue_disconnect();
        return;
    }
    bt_close_rpc_connection(bt);
    furi_hal_bt_stop_advertising();
}

static void bt_apply_settings(Bt* bt) {
    if(bt->bt_settings.enabled) {
        furi_hal_bt_start_advertising();
    } else {
        furi_hal_bt_stop_advertising();
    }
}

static void bt_load_keys(Bt* bt) {
    if(!furi_hal_bt_is_gatt_gap_supported()) {
        bt_show_warning(bt, "Unsupported radio stack");
        bt->status = BtStatusUnavailable;
        return;

    } else if(bt_keys_storage_is_changed(bt->keys_storage)) {
        FURI_LOG_I(TAG, "Loading new keys");

        bt_close_rpc_connection(bt);
        bt_keys_storage_load(bt->keys_storage);

        bt->current_profile = NULL;
    } else {
        FURI_LOG_I(TAG, "Keys unchanged");
    }
}

static void bt_start_application(Bt* bt) {
    if(!bt->current_profile) {
        bt->current_profile = furi_hal_bt_change_app(
            ble_profile_serial,
            NULL,
            bt_keys_storage_get_root_keys(bt->keys_storage),
            bt_on_gap_event_callback,
            bt);

        if(!bt->current_profile) {
            FURI_LOG_E(TAG, "BLE App start failed");
            bt->status = BtStatusUnavailable;
        }
    }
}

static void bt_load_settings(Bt* bt) {
    bt_settings_load(&bt->bt_settings);
    bt_apply_settings(bt);
}

static void bt_handle_get_settings(Bt* bt, BtMessage* message) {
    *message->data.settings = bt->bt_settings;
}

static void bt_handle_set_settings(Bt* bt, BtMessage* message) {
    bt->bt_settings = *message->data.csettings;
    bt_apply_settings(bt);
    bt_settings_save(&bt->bt_settings);
}

static void bt_handle_reload_keys_settings(Bt* bt) {
    bt_load_keys(bt);
    bt_start_application(bt);
    bt_load_settings(bt);
}

static void bt_init_keys_settings(Bt* bt) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    furi_pubsub_subscribe(storage_get_pubsub(storage), bt_storage_callback, bt);

    if(storage_sd_status(storage) != FSE_OK) {
        FURI_LOG_D(TAG, "SD Card not ready, skipping settings");

        // Just start the BLE serial application without loading the keys or settings
        bt_start_application(bt);
        return;
    }

    bt_handle_reload_keys_settings(bt);
}

/* ---- Firmware-resident NimBLE host (HCILayer radio) ----------------------
 * On a BLE HCI Layer radio the CPU2 has no host, so the stock profile path
 * (furi_hal_bt_change_app etc.) cannot run. Instead the bt service brings up the
 * vendored NimBLE host on CPU1 over the raw HCI transport. NimBLE owns the
 * Serial Service GATT server and the RPC bridge (lib/nimble/glue); this service
 * only mirrors the host state into the status bar and PIN screen. KNOW-597. */

// Runs on the bt service thread (timer dispatcher). Mirrors NimBLE host state
// into bt->status and the PIN screen by posting the same messages the stock GAP
// callback would.
static void bt_nimble_poll_callback(void* context) {
    furi_assert(context);
    Bt* bt = context;

    BtStatus status;
    if(nimble_glue_is_connected()) {
        status = BtStatusConnected;
    } else if(nimble_glue_is_advertising()) {
        status = BtStatusAdvertising;
    } else {
        status = BtStatusOff;
    }

    if(status != bt->nimble_last_status) {
        bt->nimble_last_status = status;
        bt->status = status;
        const BtMessage message = {.type = BtMessageTypeUpdateStatus};
        furi_message_queue_put(bt->message_queue, &message, 0);
    }

    // Legacy passkey pairing shows a 6-digit code (DISPLAY_ONLY). Surface it on
    // the PIN screen while pairing, and hide it once pairing ends.
    if(nimble_glue_is_pairing()) {
        if(!bt->nimble_pin_shown) {
            bt->nimble_pin_shown = true;
            const BtMessage message = {
                .type = BtMessageTypePinCodeShow, .data.pin_code = nimble_glue_passkey()};
            furi_message_queue_put(bt->message_queue, &message, 0);
        }
    } else if(bt->nimble_pin_shown) {
        bt->nimble_pin_shown = false;
        // UpdateStatus hides the PIN screen in the message loop.
        const BtMessage message = {.type = BtMessageTypeUpdateStatus};
        furi_message_queue_put(bt->message_queue, &message, 0);
    }
}

// GATT host shim (TASK-612 / KNOW-613): the ble_gatt_* primitives that the
// FAP-embedded lib/ble_profile copies call are redirected here instead of the
// CPU2 ACI. HID input reports (0x2A4D with an input Report Reference) and the
// battery level (0x2A19) are mapped onto the NimBLE GATT server; everything else
// (report map, HID info, device info strings) is already served statically by
// NimBLE and is dropped. Runs on the updating app's thread; the glue only does
// a shallow copy + event post, the NimBLE host thread sends.
#define BT_UUID16_HID_REPORT        (0x2A4D)
#define BT_UUID16_BATTERY_LEVEL     (0x2A19)
#define BT_HID_REPORT_TYPE_INPUT    (0x01)

static void bt_nimble_gatt_update_callback(
    uint16_t char_uuid16,
    uint16_t report_ref,
    const uint8_t* data,
    uint16_t len,
    void* context) {
    UNUSED(context);
    if(char_uuid16 == BT_UUID16_HID_REPORT) {
        if((report_ref >> 8) == BT_HID_REPORT_TYPE_INPUT) {
            nimble_glue_hid_input_report((uint8_t)(report_ref & 0xFF), data, len);
        }
    } else if(char_uuid16 == BT_UUID16_BATTERY_LEVEL) {
        if(data && len >= 1) nimble_glue_hid_battery_level(data[0]);
    }
}

static const BleGattHostShim bt_nimble_gatt_shim = {
    .on_update = bt_nimble_gatt_update_callback,
    .context = NULL,
};

// Acquire the raw HCI controller and start the NimBLE host. C2 stays alive on an
// HCILayer radio even though furi_hal_bt_start_radio_stack() returns false, so
// the controller can be acquired here. Returns true if the host started.
static bool bt_nimble_bringup(Bt* bt) {
    if(furi_hal_bt_hci_get_abi() != FURI_HAL_BT_HCI_ABI) {
        FURI_LOG_E(TAG, "Raw HCI transport ABI mismatch; NimBLE unavailable");
        return false;
    }
    if(!furi_hal_bt_hci_acquire(FURI_HAL_BT_HCI_ABI)) {
        FURI_LOG_E(TAG, "Raw HCI controller acquire failed");
        return false;
    }
    /* Selectable BLE mode (KNOW-606/TASK-607) from the persisted setting; an
     * out-of-range value falls back to the combined serial + HID peripheral. */
    NimbleMode mode = (NimbleMode)bt->bt_settings.ble_mode;
    if(mode >= NimbleModeCount) mode = NimbleModePeripheralCombined;
    if(!nimble_glue_start(mode)) {
        FURI_LOG_E(TAG, "NimBLE host start failed (mode %d)", mode);
        furi_hal_bt_hci_release();
        return false;
    }

    /* Redirect the exported ble_gatt_* primitives away from the (absent) CPU2
     * GATT server so app profile templates can start over NimBLE (KNOW-613).
     * Installed in every mode: a HID app on a serial-only host then starts
     * cleanly and its reports are dropped by the glue, instead of the app
     * issuing ACI commands into a controller NimBLE owns. */
    ble_gatt_host_shim_set(&bt_nimble_gatt_shim);

    /* Start the BLE dispatch thread now so FAP callbacks (CoC, GATT client, GATT
     * server events) are delivered off the NimBLE host thread from the first
     * event on (TASK-631). The furi_ble adapters also start it lazily. */
    ble_dispatch_init();

    bt->nimble_active = true;
    bt->nimble_profile_started = false;
    bt->nimble_last_status = BtStatusOff;
    bt->nimble_pin_shown = false;
    bt->nimble_timer =
        furi_timer_alloc(bt_nimble_poll_callback, FuriTimerTypePeriodic, bt);
    furi_timer_start(bt->nimble_timer, furi_ms_to_ticks(400));

    FURI_LOG_I(TAG, "NimBLE host started on the HCI Layer radio (mode %d)", mode);
    return true;
}

int32_t bt_srv(void* p) {
    UNUSED(p);
    Bt* bt = bt_alloc();

    if(furi_hal_rtc_get_boot_mode() != FuriHalRtcBootModeNormal) {
        FURI_LOG_W(TAG, "Skipping start in special boot mode");
        ble_glue_wait_for_c2_start(FURI_HAL_BT_C2_START_TIMEOUT);
        furi_record_create(RECORD_BT, bt);

        furi_thread_suspend(furi_thread_get_current_id());
        return 0;
    }

    // start_radio_stack() brings up C2 but returns false on an HCILayer radio
    // (its StackType is unsupported by the stock host); it deliberately leaves
    // SHCI running, so the raw HCI controller is still usable afterwards.
    bool radio_ok = furi_hal_bt_start_radio_stack();
    bool use_nimble = !(radio_ok && furi_hal_bt_is_gatt_gap_supported());

    if(!use_nimble) {
        bt_init_keys_settings(bt);
        furi_hal_bt_set_key_storage_change_callback(bt_on_key_storage_change_callback, bt);
    } else if(!radio_ok) {
        FURI_LOG_W(TAG, "Stock radio stack unavailable; using NimBLE host over raw HCI");
    }

    // Create the record before the (slower) NimBLE bring-up so consumers that
    // block on furi_record_open(RECORD_BT) — e.g. desktop — are not stalled.
    furi_record_create(RECORD_BT, bt);

    if(use_nimble) {
        /* Load settings so bring-up sees the persisted BLE mode (and enabled). */
        bt_settings_load(&bt->bt_settings);
#ifdef BT_DIAG_RAW_HCI
        /* Diagnostic (TASK-615): force Raw-HCI mode so the resident host leaves
         * the controller free for a raw-HCI FAP (Tailcat HCI bridge + Bumble),
         * to test the DCT L2CAP CoC flow on the HCILayer radio. */
        bt->bt_settings.ble_mode = NimbleModeRawHci;
#endif
        if(bt->bt_settings.ble_mode == NimbleModeRawHci) {
            /* Raw-HCI mode (KNOW-606 slot): do not acquire the controller. A
             * raw-HCI consumer FAP (Tailcat) takes it over USB. The resident
             * NimBLE host and the companion stay down while this mode is set. */
            FURI_LOG_I(
                TAG, "Raw-HCI mode: resident NimBLE host not started; controller left free");
            bt->nimble_active = false;
        } else if(!bt_nimble_bringup(bt)) {
            FURI_LOG_E(TAG, "NimBLE host bring-up failed; BLE unavailable");
            bt->status = BtStatusUnavailable;
        }
    }

    BtMessage message;

    while(1) {
        furi_check(
            furi_message_queue_get(bt->message_queue, &message, FuriWaitForever) == FuriStatusOk);
        FURI_LOG_D(
            TAG,
            "call %d, lock 0x%p, result 0x%p",
            message.type,
            (void*)message.lock,
            (void*)message.result);
        if(message.type == BtMessageTypeSuspendProfile) {
            bt_suspend_profile(bt, &message);
        } else if(message.type == BtMessageTypeResumeDefaultProfile) {
            bt_resume_default_profile(bt, &message);
        } else if(message.type == BtMessageTypeUpdateStatus && !bt->profile_suspended) {
            // Update view ports
            bt_statusbar_update(bt);
            bt_pin_code_hide(bt);
            if(bt->status_changed_cb) {
                bt->status_changed_cb(bt->status, bt->status_changed_ctx);
            }
        } else if(message.type == BtMessageTypeUpdateBatteryLevel && !bt->profile_suspended) {
            if(bt->nimble_active) {
                // NimBLE serves the Battery Service itself
                nimble_glue_hid_battery_level(message.data.battery_level);
            } else {
                // Stock CPU2 battery service
                furi_hal_bt_update_battery_level(message.data.battery_level);
            }
        } else if(
            message.type == BtMessageTypeUpdatePowerState && !bt->profile_suspended &&
            !bt->nimble_active) {
            furi_hal_bt_update_power_state(message.data.power_state_charging);
        } else if(message.type == BtMessageTypePinCodeShow && !bt->profile_suspended) {
            // Display PIN code
            bt_pin_code_show(bt, message.data.pin_code);
        } else if(message.type == BtMessageTypeKeysStorageUpdated && !bt->profile_suspended) {
            bt_keys_storage_update(
                bt->keys_storage,
                message.data.key_storage_data.start_address,
                message.data.key_storage_data.size);
        } else if(message.type == BtMessageTypeSetProfile && !bt->profile_suspended) {
            bt_change_profile(bt, &message);
        } else if(message.type == BtMessageTypeDisconnect && !bt->profile_suspended) {
            if(bt->nimble_active) {
                nimble_glue_disconnect();
            } else {
                bt_close_connection(bt);
            }
        } else if(message.type == BtMessageTypeForgetBondedDevices) {
            if(bt->nimble_active) {
                nimble_glue_forget_bonds();
            } else {
                bt_keys_storage_delete(bt->keys_storage);
            }
        } else if(message.type == BtMessageTypeGetSettings) {
            bt_handle_get_settings(bt, &message);
        } else if(message.type == BtMessageTypeSetSettings && !bt->profile_suspended) {
            bt_handle_set_settings(bt, &message);
        } else if(message.type == BtMessageTypeReloadKeysSettings && !bt->profile_suspended) {
            bt_handle_reload_keys_settings(bt);
        }

        if(message.lock) api_lock_unlock(message.lock);
    }

    return 0;
}
