/* NimBLE H4 transport over furi_hal_bt_hci. */

#include <furi.h>
#include <furi_hal_bt_hci.h>

#include "nimble/ble.h"
#include "nimble/nimble_npl.h"
#include "nimble/transport.h"
#include "os/os_mbuf.h"

#define TAG "NimbleHci"

#define H4_CMD 0x01
#define H4_ACL 0x02
#define H4_EVT 0x04

#define HCI_SEND_TIMEOUT_MS 2000

static FuriThread* reader;

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

static void reader_deliver_evt(const uint8_t* body, size_t length) {
    if(length > MYNEWT_VAL(BLE_TRANSPORT_EVT_SIZE)) {
        FURI_LOG_W(TAG, "Event too long; dropping %u bytes", (unsigned)length);
        return;
    }
    /* Dropping an event can desync the host: wait for a buffer. */
    uint8_t* evt;
    for(uint32_t tries = 0; !(evt = ble_transport_alloc_evt(0)); tries++) {
        if(tries == 1000) {
            FURI_LOG_E(TAG, "No event buffer; dropping %u bytes", (unsigned)length);
            return;
        }
        furi_delay_ms(1);
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
    UNUSED(context);
    uint8_t frame[FURI_HAL_BT_HCI_FRAME_MAX];

    bool faulted = false;
    for(;;) {
        int32_t n = furi_hal_bt_hci_receive(frame, sizeof(frame), 100);
        if(n < 0) {
            /* Faulted or released; events resume once the bridge is acquired again. */
            if(!faulted) FURI_LOG_E(TAG, "HCI transport fault");
            faulted = true;
            furi_delay_ms(100);
            continue;
        }
        faulted = false;
        if(n == 0) {
            continue;
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

void ble_transport_ll_init(void) {
    reader = furi_thread_alloc_ex("NimbleHciRx", 2048, reader_thread, NULL);
    furi_thread_start(reader);
}
