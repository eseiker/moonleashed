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

#endif /* COC_GLUE_H_ */
