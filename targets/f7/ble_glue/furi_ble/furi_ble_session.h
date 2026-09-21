#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** A BLE role an app holds while it runs. Events arrive on a queue the app
 *  drains from its own thread, never on the BLE host thread. */
typedef struct FuriBleSession FuriBleSession;

typedef enum {
    /** Scan for a peer by advertised name and connect as central. The
     *  companion's link stays up; advertising pauses until the session is freed.
     *  One central session runs at a time. */
    FuriBleRoleCentral,
} FuriBleRole;

typedef struct {
    FuriBleRole role;
    /** Central role: the advertised name of the peer to connect to, at most
     *  29 bytes. */
    const char* central_name;
} FuriBleSessionConfig;

typedef enum {
    FuriBleEventCentralConnected, /**< conn_handle is valid */
    FuriBleEventCentralDisconnected, /**< status is the disconnect reason */
    FuriBleEventCentralFailed, /**< status is the host error */
} FuriBleEventType;

typedef struct {
    FuriBleEventType type;
    uint16_t conn_handle;
    int32_t status;
} FuriBleEvent;

/** Start a session. Returns NULL if BLE is not ready, the name is too long, or
 *  another central session is running. A session freed while connected keeps
 *  running until its link has dropped. */
FuriBleSession* furi_ble_session_alloc(const FuriBleSessionConfig* config);

/** Wait up to timeout_ms for the next event. Returns false on timeout. */
bool furi_ble_session_get_event(FuriBleSession* session, FuriBleEvent* event, uint32_t timeout_ms);

/** The central link's connection handle, 0xFFFF while not connected. */
uint16_t furi_ble_session_conn_handle(FuriBleSession* session);

/** End the session: drop its link and advertise again. */
void furi_ble_session_free(FuriBleSession* session);

#ifdef __cplusplus
}
#endif
