/* NimBLE H4 transport over furi_hal_bt_hci. Frames from the controller are
 * handed to the host straight from the IPCC interrupt, as upstream's UART
 * transport does from its RX interrupt: no queue and no reader thread. */

#include <furi.h>
#include <furi_hal_bt_hci.h>

#include "nimble/ble.h"
#include "nimble/hci_common.h"
#include "nimble/nimble_npl.h"
#include "nimble/transport.h"
#include "host/ble_hs.h"
#include "os/os_mbuf.h"

#include "nimble_glue.h"

#define H4_CMD 0x01
#define H4_ACL 0x02
#define H4_EVT 0x04

#define HCI_SEND_TIMEOUT_MS 2000

static bool ready; /* pools initialised */
static volatile uint32_t dropped_evt;
static volatile uint32_t dropped_adv;
static volatile uint32_t dropped_acl;

int ble_transport_to_ll_cmd_impl(void* buf) {
    const uint8_t* cmd = buf;
    size_t length = 3U + cmd[2];
    uint8_t frame[1 + FURI_HAL_BT_HCI_FRAME_MAX];
    int rc = 0;

    if(length + 1 > sizeof(frame)) {
        rc = BLE_ERR_MEM_CAPACITY;
    } else {
        frame[0] = H4_CMD;
        memcpy(frame + 1, cmd, length);
        if(!furi_hal_bt_hci_send(frame, length + 1, HCI_SEND_TIMEOUT_MS)) {
            rc = BLE_ERR_MEM_CAPACITY;
        }
    }

    ble_transport_free(buf);
    return rc;
}

int ble_transport_to_ll_acl_impl(struct os_mbuf* om) {
    uint8_t frame[1 + FURI_HAL_BT_HCI_FRAME_MAX];
    uint16_t length = os_mbuf_len(om);
    int rc = 0;

    if((size_t)length + 1 > sizeof(frame)) {
        rc = BLE_ERR_MEM_CAPACITY;
    } else {
        frame[0] = H4_ACL;
        os_mbuf_copydata(om, 0, length, frame + 1);
        if(!furi_hal_bt_hci_send(frame, length + 1, HCI_SEND_TIMEOUT_MS)) {
            rc = BLE_ERR_MEM_CAPACITY;
        }
    }

    os_mbuf_free_chain(om);
    return rc;
}

int ble_transport_to_ll_iso_impl(struct os_mbuf* om) {
    os_mbuf_free_chain(om);
    return BLE_ERR_UNSUPPORTED;
}

// IPCC interrupt. Advertising reports may be dropped; anything else falls
// back to the discardable pool.
static void rx_evt(const uint8_t* body, size_t length) {
    bool discardable =
        body[0] == BLE_HCI_EVCODE_LE_META && length > 2 &&
        (body[2] == BLE_HCI_LE_SUBEV_ADV_RPT || body[2] == BLE_HCI_LE_SUBEV_EXT_ADV_RPT);
    uint8_t* evt =
        length <= MYNEWT_VAL(BLE_TRANSPORT_EVT_SIZE) ? ble_transport_alloc_evt(discardable) : NULL;
    if(!evt) {
        if(discardable) {
            dropped_adv++;
        } else {
            // A lost event leaves the host out of step; a reset re-syncs it
            dropped_evt++;
            ble_hs_sched_reset(BLE_HS_ECONTROLLER);
        }
        return;
    }
    memcpy(evt, body, length);
    if(ble_transport_to_hs_evt(evt) != 0) dropped_evt++;
}

static void rx_acl(const uint8_t* body, size_t length) {
    struct os_mbuf* om = ble_transport_alloc_acl_from_ll();
    if(!om || os_mbuf_append(om, body, length) != 0) {
        if(om) os_mbuf_free_chain(om);
        dropped_acl++;
        return;
    }
    ble_transport_to_hs_acl(om);
}

static void rx_frame(const uint8_t* frame, size_t length, void* context) {
    UNUSED(context);
    if(!ready) {
        dropped_evt++;
        return;
    }
    if(frame[0] == H4_EVT) {
        rx_evt(frame + 1, length - 1);
    } else if(frame[0] == H4_ACL) {
        rx_acl(frame + 1, length - 1);
    }
}

void nimble_transport_furi_attach(void) {
    furi_hal_bt_hci_set_rx_callback(rx_frame, NULL);
}

void nimble_transport_furi_drops(uint32_t* evt, uint32_t* adv, uint32_t* acl) {
    *evt = dropped_evt;
    *adv = dropped_adv;
    *acl = dropped_acl;
}

void ble_transport_ll_init(void) {
    ready = true;
}
