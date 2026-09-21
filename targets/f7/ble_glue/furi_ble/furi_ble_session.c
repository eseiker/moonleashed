#include "furi_ble_session.h"

#include <nimble_glue.h>
#include <furi.h>

#define QUEUE_DEPTH 8
#define CONN_NONE   0xFFFF

struct FuriBleSession {
    FuriBleRole role;
    FuriMessageQueue* queue;
    volatile uint16_t conn_handle;
};

// Host thread: only translate and enqueue
static void session_central_cb(NimbleCentralEventKind kind, uint16_t conn, int status, void* ctx) {
    FuriBleSession* session = ctx;
    FuriBleEvent event = {.conn_handle = conn, .status = status};
    switch(kind) {
    case NimbleCentralConnected:
        session->conn_handle = conn;
        event.type = FuriBleEventCentralConnected;
        break;
    case NimbleCentralDisconnected:
        session->conn_handle = CONN_NONE;
        event.type = FuriBleEventCentralDisconnected;
        break;
    default:
        session->conn_handle = CONN_NONE;
        event.type = FuriBleEventCentralFailed;
        break;
    }
    furi_message_queue_put(session->queue, &event, 0);
}

FuriBleSession* furi_ble_session_alloc(const FuriBleSessionConfig* config) {
    furi_check(config);
    if(config->role != FuriBleRoleCentral) return NULL;

    FuriBleSession* session = malloc(sizeof(FuriBleSession));
    session->role = config->role;
    session->conn_handle = CONN_NONE;
    session->queue = furi_message_queue_alloc(QUEUE_DEPTH, sizeof(FuriBleEvent));
    if(!nimble_glue_central_start(config->central_name, session_central_cb, session)) {
        furi_message_queue_free(session->queue);
        free(session);
        return NULL;
    }
    return session;
}

bool furi_ble_session_get_event(FuriBleSession* session, FuriBleEvent* event, uint32_t timeout_ms) {
    furi_check(session);
    furi_check(event);
    return furi_message_queue_get(session->queue, event, timeout_ms) == FuriStatusOk;
}

uint16_t furi_ble_session_conn_handle(FuriBleSession* session) {
    furi_check(session);
    return session->conn_handle;
}

void furi_ble_session_free(FuriBleSession* session) {
    furi_check(session);
    nimble_glue_central_stop();
    furi_message_queue_free(session->queue);
    free(session);
}
