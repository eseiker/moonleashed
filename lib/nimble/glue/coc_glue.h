/*
 * L2CAP CoC / multi-role track for the firmware NimBLE host (TASK-615, fork 2).
 *
 * Goal: keep the companion peripheral link (Serial Service RPC) up while the
 * host also acts as a central and opens an L2CAP LE Credit-Based CoC — the
 * tailcat/DCT role (KNOW-516), delivered as native NimBLE multi-role instead of
 * raw HCI. This header currently exposes only the controller capability probe;
 * the CoC data path is added once the probe confirms the controller allows the
 * needed role combinations.
 */

#ifndef COC_GLUE_H_
#define COC_GLUE_H_

/* Query and log the controller's multi-role capability: LE Read Supported States
 * (which role/state combinations may run at once) and LE Read Buffer Size. Call
 * once from the host sync callback, after the controller is up. The result tells
 * whether scanning/initiating a second link while a peripheral connection is up
 * is legal — the gating fact for running a CoC central alongside the companion.
 */
void coc_probe_capabilities(void);

/* Register the L2CAP CoC echo server on the test PSM. Call once in
 * nimble_glue_start after the host services are registered. Returns 0 on
 * success. Proves NimBLE CoC on the HCILayer radio before the DCT logic lands. */
int coc_server_start(void);

/* Open an L2CAP CoC as the client on an already-established central link
 * (TASK-615, Milestone 2 / DCT). conn_handle is the central connection; psm is
 * the peer's CoC PSM. On connect the Flipper sends an opening payload so the
 * round trip is observable. Returns 0 on success. */
int coc_client_connect(uint16_t conn_handle, uint16_t psm);

/* True while a peer has an open CoC channel. */
bool coc_is_connected(void);

/* Bytes received on the CoC since it connected. */
uint32_t coc_rx_bytes(void);

/* --- Neutral multi-channel CoC core (TASK-621) ----------------------------- *
 *
 * A transport-neutral, plain-C L2CAP CoC manager the firmware furi_ble layer
 * builds the Moon-Firmware-compatible ble_l2cap_coc_* API on (KNOW-636). It owns
 * the NimBLE channels, assigns a small uint8_t channel index to each, and reports
 * events through one registered dispatcher. It is independent of the echo/DCT
 * test helpers above; both can run at once on different PSMs.
 *
 * These functions are safe to include from the firmware (no NimBLE types leak
 * through this header).
 */

typedef enum {
    CocApiConnected, /* a channel opened: index + conn_handle + peer_mtu valid */
    CocApiDisconnected, /* a channel closed: index + conn_handle valid */
    CocApiData, /* SDU received: index + conn_handle + data/data_len valid */
    CocApiTxUnstalled, /* a stalled send can resume: index + conn_handle valid */
    CocApiError, /* connect failed: conn_handle + error_code valid */
} CocApiEventType;

typedef struct {
    CocApiEventType type;
    uint8_t channel_index;
    uint16_t conn_handle;
    const uint8_t* data; /* CocApiData only; valid only during the callback */
    uint16_t data_len;
    uint16_t peer_mtu; /* CocApiConnected only */
    uint16_t error_code; /* CocApiError only */
} CocApiEvent;

typedef void (*CocApiCallback)(const CocApiEvent* event, void* context);

/* Initialize the core and set the single event dispatcher (idempotent). */
void coc_api_init(CocApiCallback dispatch, void* context);

/* Tear the core down: close nothing on the controller, just drop state. */
void coc_api_deinit(void);

/* Listen for incoming CoC connections on a PSM (server role). Incoming channels
 * are auto-accepted and reported as CocApiConnected. Returns true on success. */
bool coc_api_listen(uint16_t psm, uint16_t mtu);

/* Open a CoC as the client on an established link (central/DCT). Returns true if
 * the request was issued; CocApiConnected or CocApiError follows. */
bool coc_api_connect(uint16_t conn_handle, uint16_t psm, uint16_t mtu);

/* Send data on a channel. Returns true if sent or queued. */
bool coc_api_send(uint8_t channel_index, const uint8_t* data, uint16_t len);

/* Re-arm reception on a channel, replenishing the peer's credits (flow control).
 * Reception is auto-re-armed after each CocApiData, so this is usually optional. */
bool coc_api_grant(uint8_t channel_index);

/* Disconnect a channel. Returns true if the request was issued. */
bool coc_api_disconnect(uint8_t channel_index);

#endif /* COC_GLUE_H_ */
