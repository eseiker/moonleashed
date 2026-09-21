/*
 * furi_ble session broker (TASK-631 foundation + TASK-633 central role). See
 * furi_ble_session.h. Lifecycle events from the NimBLE host thread are posted
 * into a FuriMessageQueue the FAP drains from its own loop, so the host thread
 * never runs FAP code directly.
 */

#include "furi_ble_session.h"

#include <nimble_glue.h>
#include <furi.h>

#define TAG "FuriBleSession"

#define FURI_BLE_SESSION_QUEUE_DEPTH 8
#define FURI_BLE_CONN_NONE           0xFFFF

struct FuriBleSession {
    FuriBleRole role;
    FuriMessageQueue* queue;
    volatile uint16_t conn_handle;
};

/* Runs on the NimBLE host thread: translate and enqueue, never call the FAP. */
static void session_central_cb(NimbleCentralEventKind kind, uint16_t conn, int status, void* ctx) {
    FuriBleSession* s = ctx;
    if(!s) return;
    FuriBleEvent ev = {.conn_handle = conn, .status = status};
    switch(kind) {
    case NimbleCentralConnected:
        s->conn_handle = conn;
        ev.type = FuriBleEventCentralConnected;
        break;
    case NimbleCentralDisconnected:
        s->conn_handle = FURI_BLE_CONN_NONE;
        ev.type = FuriBleEventCentralDisconnected;
        break;
    case NimbleCentralFailed:
        s->conn_handle = FURI_BLE_CONN_NONE;
        ev.type = FuriBleEventCentralFailed;
        break;
    default:
        return;
    }
    furi_message_queue_put(s->queue, &ev, 0);
}

FuriBleSession* furi_ble_session_alloc(const FuriBleSessionConfig* config) {
    if(!config) return NULL;
    if(config->role != FuriBleRoleCentral) {
        FURI_LOG_E(TAG, "unsupported role %d", config->role);
        return NULL;
    }
    /* Arbitration: only one central session at a time. */
    if(nimble_glue_central_is_active()) {
        FURI_LOG_W(TAG, "a central session is already active");
        return NULL;
    }

    FuriBleSession* s = malloc(sizeof(FuriBleSession));
    s->role = config->role;
    s->conn_handle = FURI_BLE_CONN_NONE;
    s->queue = furi_message_queue_alloc(FURI_BLE_SESSION_QUEUE_DEPTH, sizeof(FuriBleEvent));

    if(!nimble_glue_central_start(config->central_name, session_central_cb, s)) {
        FURI_LOG_E(TAG, "central_start failed");
        furi_message_queue_free(s->queue);
        free(s);
        return NULL;
    }
    return s;
}

bool furi_ble_session_get_event(FuriBleSession* session, FuriBleEvent* event, uint32_t timeout_ms) {
    if(!session || !event) return false;
    return furi_message_queue_get(session->queue, event, timeout_ms) == FuriStatusOk;
}

uint16_t furi_ble_session_conn_handle(FuriBleSession* session) {
    return session ? session->conn_handle : FURI_BLE_CONN_NONE;
}

void furi_ble_session_free(FuriBleSession* session) {
    if(!session) return;
    if(session->role == FuriBleRoleCentral) {
        nimble_glue_central_stop(); /* drop the central link, advertise again */
    }
    furi_message_queue_free(session->queue);
    free(session);
}
