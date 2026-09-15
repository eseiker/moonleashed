/*
 * NimBLE HCI transport bound to the Flipper BLE Full controller in LL_ONLY.
 *
 * NimBLE's host calls the public ble_transport_to_ll_* functions (inline
 * wrappers in nimble/transport/monitor.h when the monitor is off), which land
 * on the *_impl functions defined here. Each impl prepends the H4 indicator
 * byte and hands the frame to furi_hal_bt_hci_send. A reader thread pulls whole
 * H4 frames from furi_hal_bt_hci_receive, strips the indicator, and pushes them
 * up with ble_transport_to_hs_evt / ble_transport_to_hs_acl.
 *
 * The controller must already be acquired (furi_hal_bt_hci_acquire) before
 * ble_transport_ll_init runs; the FAP entry point does that.
 *
 * This file lives in the vendored lib so it is built with -w alongside NimBLE,
 * which keeps -Wundef in the firmware build from tripping on NimBLE headers.
 */

#include <furi.h>
#include <furi_hal_bt_hci.h>

#include "nimble/ble.h"
#include "nimble/nimble_npl.h"
#include "nimble/transport.h"
#include "os/os_mbuf.h"

#include "nimble_glue.h"

#define TAG "NimbleHci"

#define H4_CMD 0x01
#define H4_ACL 0x02
#define H4_EVT 0x04

/* Milliseconds a blocking HCI command send waits for a mailbox slot. */
#define HCI_SEND_TIMEOUT_MS 2000

typedef struct {
    FuriThread* reader;
    volatile bool stop;
    volatile bool fault;
} NimbleTransport;

static NimbleTransport transport;

/* Host -> controller: HCI command. buf is a raw command packet. */
int ble_transport_to_ll_cmd_impl(void* buf) {
    const uint8_t* cmd = buf;
    /* opcode(2) + param length(1) + params */
    size_t length = 3U + cmd[2];
    uint8_t frame[1 + FURI_HAL_BT_HCI_FRAME_MAX];
    int rc = 0;

    if(length + 1 > sizeof(frame)) {
        rc = BLE_ERR_MEM_CAPACITY;
    } else {
        frame[0] = H4_CMD;
        memcpy(frame + 1, cmd, length);
        if(!furi_hal_bt_hci_send(frame, length + 1, HCI_SEND_TIMEOUT_MS)) {
            transport.fault = true;
            rc = BLE_ERR_MEM_CAPACITY;
        }
    }

    ble_transport_free(buf);
    return rc;
}

/* Host -> controller: ACL data as an mbuf chain. */
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
            transport.fault = true;
            rc = BLE_ERR_MEM_CAPACITY;
        }
    }

    os_mbuf_free_chain(om);
    return rc;
}

/* ISO is disabled in syscfg; refuse and release the buffer if ever called. */
int ble_transport_to_ll_iso_impl(struct os_mbuf* om) {
    os_mbuf_free_chain(om);
    return BLE_ERR_UNSUPPORTED;
}

static void reader_deliver_evt(const uint8_t* body, size_t length) {
    uint8_t* evt = ble_transport_alloc_evt(0);
    if(!evt) {
        FURI_LOG_W(TAG, "No event buffer; dropping %u bytes", (unsigned)length);
        return;
    }
    memcpy(evt, body, length);
    if(ble_transport_to_hs_evt(evt) != 0) {
        FURI_LOG_W(TAG, "Host rejected event");
    }
}

static void reader_deliver_acl(const uint8_t* body, size_t length) {
    struct os_mbuf* om = ble_transport_alloc_acl_from_ll();
    if(!om) {
        FURI_LOG_W(TAG, "No ACL buffer; dropping %u bytes", (unsigned)length);
        return;
    }
    if(os_mbuf_append(om, body, length) != 0) {
        os_mbuf_free_chain(om);
        return;
    }
    ble_transport_to_hs_acl(om);
}

static int32_t reader_thread(void* context) {
    NimbleTransport* t = context;
    uint8_t frame[FURI_HAL_BT_HCI_FRAME_MAX];

    while(!t->stop) {
        int32_t n = furi_hal_bt_hci_receive(frame, sizeof(frame), 100);
        if(n < 0) {
            FURI_LOG_E(TAG, "HCI transport fault");
            t->fault = true;
            break;
        }
        if(n == 0) {
            continue; /* timeout */
        }
        if(frame[0] == H4_EVT) {
            reader_deliver_evt(frame + 1, n - 1);
        } else if(frame[0] == H4_ACL) {
            reader_deliver_acl(frame + 1, n - 1);
        } else {
            FURI_LOG_W(TAG, "Unexpected H4 type 0x%02X", frame[0]);
        }
    }
    return 0;
}

/* Called from nimble_port_init after the host side is initialized. */
void ble_transport_ll_init(void) {
    transport.stop = false;
    transport.fault = false;
    transport.reader = furi_thread_alloc_ex("NimbleHciRx", 2048, reader_thread, &transport);
    furi_thread_start(transport.reader);
}

void nimble_transport_furi_stop(void) {
    if(!transport.reader) {
        return;
    }
    transport.stop = true;
    furi_thread_join(transport.reader);
    furi_thread_free(transport.reader);
    transport.reader = NULL;
}

bool nimble_transport_furi_faulted(void) {
    return transport.fault;
}
