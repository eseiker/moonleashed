/*
 * Moon-Firmware-compatible ble_l2cap_coc_* API (KNOW-636), backed by the
 * resident NimBLE host instead of the ST ACI stack. This firmware-side adapter
 * forwards to the neutral multi-channel CoC core in the NimBLE glue
 * (coc_api_*, lib/nimble/glue/coc_glue.c) and maps its events onto the
 * BleL2capCocEvent the FAP-facing API defines. A FAP written for Moon-Firmware's
 * ble_l2cap_coc_* recompiles against this unchanged (TASK-621).
 *
 * The registered callback currently runs on the NimBLE host thread (the neutral
 * core emits synchronously). Off-thread delivery is the furi_ble broker
 * foundation (TASK-631); the signatures here do not change when that lands.
 */

#include "l2cap_coc.h"

#include <coc_glue.h>
#include <furi.h>
#include <string.h>

#define TAG "BleL2capCoC"

#define L2CAP_COC_MAX_CONNECTIONS 2

typedef struct {
    uint16_t connection_handle;
    bool active;
    BleL2capCocCallback callback;
    void* context;
} L2capCocConnection;

static L2capCocConnection coc_connections[L2CAP_COC_MAX_CONNECTIONS];
static bool coc_started;

/* Exact match first, then the default slot (handle=0) so a peripheral app can
 * register one callback before the connection handle is known. Mirrors
 * Moon-Firmware's routing. */
static L2capCocConnection* coc_find_connection(uint16_t connection_handle) {
    for(int i = 0; i < L2CAP_COC_MAX_CONNECTIONS; i++) {
        if(coc_connections[i].active &&
           coc_connections[i].connection_handle == connection_handle) {
            return &coc_connections[i];
        }
    }
    if(connection_handle != 0) {
        for(int i = 0; i < L2CAP_COC_MAX_CONNECTIONS; i++) {
            if(coc_connections[i].active && coc_connections[i].connection_handle == 0) {
                return &coc_connections[i];
            }
        }
    }
    return NULL;
}

static L2capCocConnection* coc_alloc_connection(uint16_t connection_handle) {
    L2capCocConnection* existing = coc_find_connection(connection_handle);
    if(existing) return existing;
    for(int i = 0; i < L2CAP_COC_MAX_CONNECTIONS; i++) {
        if(!coc_connections[i].active) {
            memset(&coc_connections[i], 0, sizeof(L2capCocConnection));
            coc_connections[i].connection_handle = connection_handle;
            coc_connections[i].active = true;
            return &coc_connections[i];
        }
    }
    FURI_LOG_E(TAG, "No free CoC connection slots (max %d)", L2CAP_COC_MAX_CONNECTIONS);
    return NULL;
}

static void coc_free_connection(uint16_t connection_handle) {
    for(int i = 0; i < L2CAP_COC_MAX_CONNECTIONS; i++) {
        if(coc_connections[i].active &&
           coc_connections[i].connection_handle == connection_handle) {
            coc_connections[i].active = false;
        }
    }
}

/* Translate a neutral core event into the FAP-facing BleL2capCocEvent and route
 * it to the callback registered for that connection. */
static void coc_dispatch(const CocApiEvent* ev, void* context) {
    UNUSED(context);
    L2capCocConnection* conn = coc_find_connection(ev->conn_handle);
    if(!conn || !conn->callback) return;

    BleL2capCocEvent out = {
        .channel_index = ev->channel_index,
        .connection_handle = ev->conn_handle,
    };
    switch(ev->type) {
    case CocApiConnected:
        out.type = BleL2capCocEventConnected;
        out.connected.peer_mtu = ev->peer_mtu;
        break;
    case CocApiDisconnected:
        out.type = BleL2capCocEventDisconnected;
        break;
    case CocApiData:
        out.type = BleL2capCocEventDataReceived;
        out.data.data = ev->data;
        out.data.data_len = ev->data_len;
        break;
    case CocApiTxUnstalled:
        out.type = BleL2capCocEventTxDone;
        break;
    case CocApiError:
        out.type = BleL2capCocEventError;
        out.error.code = ev->error_code;
        break;
    default:
        return;
    }
    conn->callback(&out, conn->context);
}

void ble_l2cap_coc_init(void) {
    memset(coc_connections, 0, sizeof(coc_connections));
    coc_api_init(coc_dispatch, NULL);
    coc_started = true;
    FURI_LOG_I(TAG, "L2CAP CoC initialized (NimBLE-backed)");
}

void ble_l2cap_coc_deinit(void) {
    coc_api_deinit();
    memset(coc_connections, 0, sizeof(coc_connections));
    coc_started = false;
}

void ble_l2cap_coc_set_callback(
    uint16_t connection_handle,
    BleL2capCocCallback callback,
    void* context) {
    if(callback) {
        L2capCocConnection* conn = coc_alloc_connection(connection_handle);
        if(conn) {
            conn->callback = callback;
            conn->context = context;
        }
    } else {
        coc_free_connection(connection_handle);
    }
}

bool ble_l2cap_coc_connect(
    uint16_t conn_handle,
    uint16_t spsm,
    uint16_t mtu,
    uint16_t mps,
    uint16_t initial_credits) {
    UNUSED(mps); /* NimBLE picks the MPS; credits are managed automatically */
    UNUSED(initial_credits);
    return coc_api_connect(conn_handle, spsm, mtu);
}

bool ble_l2cap_coc_accept(
    uint16_t conn_handle,
    uint16_t mtu,
    uint16_t mps,
    uint16_t initial_credits,
    uint16_t result) {
    /* The NimBLE server auto-accepts incoming channels in the core, so this is a
     * compatibility no-op: true when the caller wanted to accept (result 0). A
     * post-hoc reject is not supported on this backend. */
    UNUSED(conn_handle);
    UNUSED(mtu);
    UNUSED(mps);
    UNUSED(initial_credits);
    return result == 0x0000;
}

bool ble_l2cap_coc_send(uint8_t channel_index, const uint8_t* data, uint16_t data_len) {
    if(data_len > 252) data_len = 252;
    return coc_api_send(channel_index, data, data_len);
}

bool ble_l2cap_coc_flow_control(uint8_t channel_index, uint16_t credits) {
    UNUSED(credits); /* NimBLE re-arms with its own credit count */
    return coc_api_grant(channel_index);
}

bool ble_l2cap_coc_disconnect(uint8_t channel_index) {
    return coc_api_disconnect(channel_index);
}

bool ble_l2cap_coc_listen(uint16_t spsm, uint16_t mtu) {
    if(!coc_started) ble_l2cap_coc_init();
    return coc_api_listen(spsm, mtu);
}
