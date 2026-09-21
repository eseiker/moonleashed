#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_L2CAP_COC_MPS_MAX         248
#define BLE_L2CAP_COC_MTU_DEFAULT     2048
#define BLE_L2CAP_COC_CREDITS_DEFAULT 10

typedef enum {
    BleL2capCocEventConnected,
    BleL2capCocEventDisconnected,
    BleL2capCocEventDataReceived,
    BleL2capCocEventCreditsReceived,
    BleL2capCocEventTxDone,
    BleL2capCocEventError,
} BleL2capCocEventType;

typedef struct {
    BleL2capCocEventType type;
    uint8_t channel_index;
    uint16_t connection_handle;
    union {
        struct {
            uint16_t peer_mtu;
            uint16_t peer_mps; // 0: not reported
            uint16_t initial_credits; // 0: the host manages credits
        } connected;
        struct {
            const uint8_t* data;
            uint16_t data_len;
        } data;
        struct {
            uint16_t credits;
        } credits;
        struct {
            uint16_t code;
        } error;
    };
} BleL2capCocEvent;

/** Runs on the BLE dispatch thread. Event data is valid during the call.
 *  CreditsReceived is never sent: the host manages credits. */
typedef void (*BleL2capCocCallback)(BleL2capCocEvent* event, void* context);

/* Requests return true once queued; a failure arrives as an Error event. Our
 * MTU is at most BLE_L2CAP_COC_MTU_DEFAULT. */

void ble_l2cap_coc_init(void);

/** Stop listening on every PSM. After this returns, no callback runs again. */
void ble_l2cap_coc_deinit(void);

/** Set the callback for a connection, or NULL to remove it. Handle 0 is the
 *  default for connections that have no callback of their own. */
void ble_l2cap_coc_set_callback(
    uint16_t connection_handle,
    BleL2capCocCallback callback,
    void* context);

/** Open a channel on a link. mps and initial_credits are ignored: the host
 *  picks them. Connected or Error follows. */
bool ble_l2cap_coc_connect(
    uint16_t conn_handle,
    uint16_t spsm,
    uint16_t mtu,
    uint16_t mps,
    uint16_t initial_credits);

/** Incoming channels on a listened PSM are accepted automatically; this returns
 *  whether result asked for acceptance. */
bool ble_l2cap_coc_accept(
    uint16_t conn_handle,
    uint16_t mtu,
    uint16_t mps,
    uint16_t initial_credits,
    uint16_t result);

/** Send one SDU of up to the peer's MTU. */
bool ble_l2cap_coc_send(uint8_t channel_index, const uint8_t* data, uint16_t data_len);

/** Hand the host a fresh receive buffer. Reception re-arms after each SDU on
 *  its own, so this is rarely needed. */
bool ble_l2cap_coc_flow_control(uint8_t channel_index, uint16_t credits);

bool ble_l2cap_coc_disconnect(uint8_t channel_index);

/** Moonleashed: accept channels on a PSM (server role). Unlike the other
 *  calls, this and deinit update the host's server list from the caller's
 *  thread, under the host lock. */
bool ble_l2cap_coc_listen(uint16_t spsm, uint16_t mtu);

#ifdef __cplusplus
}
#endif
