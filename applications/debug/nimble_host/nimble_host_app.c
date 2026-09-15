/*
 * On-device NimBLE host experiment (TASK-575), milestones 1 and 2.
 *
 * The FAP suspends the stock BT service, acquires the raw HCI controller
 * through furi_hal_bt_hci, then brings up the vendored NimBLE host on CPU1 (all
 * NimBLE code lives behind nimble_glue.h in the -w private lib). When NimBLE
 * finishes controller startup it syncs, and the screen shows the identity
 * address read back from the controller plus a passive-scan advert count. That
 * round trip proves the host, the NPL-over-FURI port, and the HCI transport
 * work end to end.
 *
 * The controller may be BLE Full flipped to LL_ONLY (radio_stack_type 1) or a
 * BLE HCI Layer controller (radio_stack_type 2); NimBLE only needs a raw HCI
 * controller and both provide one. The acquire/release stack-type handling
 * lives in the installed firmware's furi_hal_bt_hci, not in this FAP.
 *
 * Milestone 2 teardown is graceful: nimble_glue_stop frees every NPL object so
 * no callout timer fires into freed FAP memory, then the controller is released
 * without a reboot. The stock GATT/GAP profile is resumed only on a radio that
 * can host it (see the exit path below).
 */

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_bt_hci.h>
#include <gui/gui.h>
#include <bt/bt_service/bt.h>

#include "nimble_glue.h"

#define TAG "NimbleHost"

typedef struct {
    FuriMessageQueue* input;
} NimbleHostApp;

static void draw_callback(Canvas* canvas, void* context) {
    UNUSED(context);
    char line[48];
    uint8_t addr[6];
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 1, 12, "NimBLE Serial / B1");
    canvas_set_font(canvas, FontSecondary);
    if(nimble_glue_faulted()) {
        canvas_draw_str(canvas, 1, 28, "HCI FAULT - exit & retry");
    } else if(nimble_glue_is_pairing()) {
        canvas_set_font(canvas, FontPrimary);
        snprintf(line, sizeof(line), "Pair: %06lu", nimble_glue_passkey());
        canvas_draw_str(canvas, 1, 30, line);
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 1, 44, "Enter on phone");
    } else if(nimble_glue_is_connected()) {
        canvas_draw_str(canvas, 1, 24, nimble_glue_is_bonded() ? "Connected (bonded)" : "Connected");
        snprintf(line, sizeof(line), "RX bytes: %lu", nimble_glue_rx_bytes());
        canvas_draw_str(canvas, 1, 36, line);
    } else if(nimble_glue_get_addr(addr)) {
        canvas_draw_str(
            canvas, 1, 24, nimble_glue_is_advertising() ? "Advertising" : "Synced");
        snprintf(
            line,
            sizeof(line),
            "Me %02X:%02X:%02X:%02X:%02X:%02X",
            addr[5],
            addr[4],
            addr[3],
            addr[2],
            addr[1],
            addr[0]);
        canvas_draw_str(canvas, 1, 36, line);
        canvas_draw_str(canvas, 1, 48, "Connect from phone app");
    } else {
        canvas_draw_str(canvas, 1, 28, "Starting host...");
    }
    canvas_draw_str(canvas, 1, 62, "Back: stop & exit");
}

static void input_callback(InputEvent* event, void* context) {
    NimbleHostApp* app = context;
    furi_message_queue_put(app->input, event, 0);
}

int32_t nimble_host_app(void* context) {
    UNUSED(context);

    /*
     * Only manage the stock bt service on a radio it can actually host: BLE Full
     * (LL_HOST <-> LL_ONLY). On a BLE HCI Layer radio the bt service already
     * gave up at boot (BLE unavailable, no profile, not driving the controller),
     * and its suspend/resume is a one-way trap: bt_resume_default_profile only
     * clears profile_suspended when bt_change_profile succeeds, which it never
     * does without GATT/GAP support. Suspending there would leave the service
     * permanently suspended, so the next launch's bt_profile_suspend returns
     * false and the app bails. The proven tailcat_hci FAP never touches the bt
     * service on this radio; do the same and acquire the controller directly.
     */
    bool manage_bt = furi_hal_bt_is_gatt_gap_supported();
    Bt* bt = furi_record_open(RECORD_BT);
    if(manage_bt && !bt_profile_suspend(bt)) {
        FURI_LOG_E(TAG, "Failed to suspend BT service");
        furi_record_close(RECORD_BT);
        return -1;
    }

    if(furi_hal_bt_hci_get_abi() != FURI_HAL_BT_HCI_ABI ||
       !furi_hal_bt_hci_acquire(FURI_HAL_BT_HCI_ABI)) {
        FURI_LOG_E(TAG, "Raw HCI controller acquire failed");
        if(manage_bt) bt_profile_resume_default(bt);
        furi_record_close(RECORD_BT);
        furi_hal_power_reset();
        return -1;
    }

    NimbleHostApp* app = malloc(sizeof(NimbleHostApp));
    app->input = furi_message_queue_alloc(8, sizeof(InputEvent));

    if(!nimble_glue_start()) {
        FURI_LOG_E(TAG, "Failed to start NimBLE host");
    }

    Gui* gui = furi_record_open(RECORD_GUI);
    ViewPort* viewport = view_port_alloc();
    view_port_draw_callback_set(viewport, draw_callback, app);
    view_port_input_callback_set(viewport, input_callback, app);
    gui_add_view_port(gui, viewport, GuiLayerFullscreen);

    InputEvent event;
    bool running = true;
    while(running) {
        if(furi_message_queue_get(app->input, &event, 100) == FuriStatusOk) {
            if(event.key == InputKeyBack && event.type == InputTypeShort) {
                running = false;
            }
        }
        view_port_update(viewport);
    }

    gui_remove_view_port(gui, viewport);
    view_port_free(viewport);
    furi_record_close(RECORD_GUI);

    /*
     * Graceful teardown: stop the host, free all NPL objects (so no callout
     * timer fires into freed FAP memory), then release the controller. On a
     * BLE Full radio also resume the stock profile; on an HCI Layer radio we
     * never suspended it (manage_bt is false), so there is nothing to resume and
     * nothing to reboot for. See the manage_bt comment at startup.
     */
    nimble_glue_stop();
    bool released = furi_hal_bt_hci_release();

    bool resumed = true;
    if(manage_bt) {
        resumed = released && bt_profile_resume_default(bt);
    }

    furi_message_queue_free(app->input);
    free(app);
    furi_record_close(RECORD_BT);
    if(!released || !resumed) {
        /* Controller could not be cleanly reset/restored; reboot to recover. */
        furi_hal_power_reset();
    }
    return 0;
}
