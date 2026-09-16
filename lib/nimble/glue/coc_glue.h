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

#endif /* COC_GLUE_H_ */
