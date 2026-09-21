#pragma once

/*
 * furi_ble session broker (TASK-631 foundation + TASK-633 central role).
 *
 * A FAP scopes its BLE capability with a FuriBleSession. Lifecycle events are
 * delivered off the NimBLE host thread through a FuriMessageQueue the FAP drains
 * from its own loop (furi_ble_session_get_event), so an unloaded or blocked FAP
 * never stalls or corrupts the resident host. Freeing the session tears down its
 * role (for the central role: drop the link and advertise again).
 *
 * This first cut covers the central role. The GATT-client and L2CAP-CoC
 * data APIs (ble_gatt_client_* / ble_l2cap_coc_*) still deliver their own events
 * on the host thread; routing them through this queue is the remaining broker
 * work.
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct FuriBleSession FuriBleSession;

typedef enum {
    /* Central: scan for a named peer and connect as central. The companion's
     * link stays up; advertising pauses until the session is freed (TASK-818).
     * One central session runs at a time. */
    FuriBleRoleCentral,
} FuriBleRole;

typedef struct {
    FuriBleRole role;
    /* Central role: the advertised name of the peer to connect to. */
    const char* central_name;
} FuriBleSessionConfig;

typedef enum {
    FuriBleEventCentralConnected, /* conn_handle valid */
    FuriBleEventCentralDisconnected, /* status = disconnect reason */
    FuriBleEventCentralFailed, /* status = connect failure code */
} FuriBleEventType;

typedef struct {
    FuriBleEventType type;
    uint16_t conn_handle;
    int32_t status;
} FuriBleEvent;

/* Allocate a session and start its role. Returns NULL if the host is not ready
 * or the role conflicts with an active session (arbitration). */
FuriBleSession* furi_ble_session_alloc(const FuriBleSessionConfig* config);

/* Wait up to timeout_ms for the next session event. Returns true and fills
 * `event` when one arrives, false on timeout. Call from the FAP's own loop. */
bool furi_ble_session_get_event(FuriBleSession* session, FuriBleEvent* event, uint32_t timeout_ms);

/* The current central connection handle (valid after CentralConnected), for use
 * with ble_gatt_client_* / ble_l2cap_coc_*. 0xFFFF when not connected. */
uint16_t furi_ble_session_conn_handle(FuriBleSession* session);

/* Tear the session down and advertise again. */
void furi_ble_session_free(FuriBleSession* session);

#ifdef __cplusplus
}
#endif
