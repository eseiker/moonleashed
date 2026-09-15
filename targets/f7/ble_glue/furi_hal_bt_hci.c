#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_bt_hci.h>
#include "ble_glue.h"
#include "ble_app.h"
#include "../furi_hal/furi_hal_bt_i.h"
#include <interface/patterns/ble_thread/tl/tl.h>
#include <interface/patterns/ble_thread/shci/shci.h>

#define TAG "BtHci"

typedef struct {
    uint16_t length;
    uint8_t bytes[FURI_HAL_BT_HCI_FRAME_MAX];
} HciFrame;

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t command_buffer;
PLACE_IN_SECTION("MB_MEM2")
ALIGN(4) static uint8_t acl_buffer[sizeof(TL_PacketHeader_t) + FURI_HAL_BT_HCI_FRAME_MAX];

static FuriMessageQueue* rx_queue;
static FuriSemaphore* command_free;
static FuriSemaphore* acl_free;
static bool initialized;
static bool hci_layer_started;
static volatile bool owned;
static volatile bool hci_layer_owned;
static volatile bool fault;
static volatile bool acl_pending;
static volatile uint16_t pending_opcode;
static volatile uint8_t command_status;

static uint16_t read_le16(const uint8_t* p) {
    return p[0] | ((uint16_t)p[1] << 8);
}

static void acl_ack(void) {
    /* A reset can deliver the previous session's ACK through this callback. */
    if(!owned || !acl_pending) return;
    acl_pending = false;
    if(furi_semaphore_release(acl_free) != FuriStatusOk) fault = true;
}

static bool reset_semaphore(FuriSemaphore* semaphore) {
    while(furi_semaphore_acquire(semaphore, 0) == FuriStatusOk) {
    }
    return furi_semaphore_release(semaphore) == FuriStatusOk;
}

static void controller_event(TL_EvtPacket_t* packet) {
    const uint8_t* bytes = (const uint8_t*)&packet->evtserial;
    HciFrame frame;
    bool response = bytes[0] == 4 && (bytes[1] == 0x0e || bytes[1] == 0x0f);
    size_t length = 0;
    if(bytes[0] == 4)
        length = 3U + bytes[2];
    else if(bytes[0] == 2)
        length = 5U + read_le16(bytes + 3);

    /* LL-host startup may briefly deliver through the previous raw callback. */
    if(!owned) {
        if(!response) TL_MM_EvtDone(packet);
        return;
    }

    if(!length || length > sizeof(frame.bytes)) {
        fault = true;
        if(!response) TL_MM_EvtDone(packet);
        return;
    }

    /* Command replies use dedicated shared buffers, not the event pool. */
    if(response) {
        bool complete = bytes[1] == 0x0e;
        size_t minimum = 7;
        uint16_t opcode = length >= minimum ? read_le16(bytes + (complete ? 4 : 5)) : 0;
        if(!opcode) {
            fault = true;
            return;
        }
        if(!pending_opcode || opcode != pending_opcode) {
            FURI_LOG_W(
                TAG, "Drop stale command response %04X (pending %04X)", opcode, pending_opcode);
            return;
        }
    }

    frame.length = length;
    memcpy(frame.bytes, bytes, length);
    if(furi_message_queue_put(rx_queue, &frame, 0) != FuriStatusOk) fault = true;

    if(response) {
        bool complete = bytes[1] == 0x0e;
        command_status = bytes[complete ? 6 : 3];
        pending_opcode = 0;
        if(furi_semaphore_release(command_free) != FuriStatusOk) fault = true;
    } else {
        TL_MM_EvtDone(packet);
    }
}

uint32_t furi_hal_bt_hci_get_abi(void) {
    return FURI_HAL_BT_HCI_ABI;
}

bool furi_hal_bt_hci_acquire(uint32_t abi) {
    if(abi != FURI_HAL_BT_HCI_ABI || owned || !ble_glue_is_alive()) return false;
    const BleGlueC2Info* info = ble_glue_get_c2_info();
    bool hci_layer = info->StackType == INFO_STACK_TYPE_BLE_HCI;
    bool full_stack = info->StackType == INFO_STACK_TYPE_BLE_FULL;
    if(info->mode != BleGlueC2ModeStack || (!hci_layer && !full_stack) ||
       info->VersionMajor != 1 || info->VersionMinor != 20 || info->VersionSub != 0)
        return false;

    if(!initialized) {
        rx_queue = furi_message_queue_alloc(32, sizeof(HciFrame));
        command_free = furi_semaphore_alloc(1, 1);
        acl_free = furi_semaphore_alloc(1, 1);
        initialized = true;
    }

    if(full_stack && !furi_hal_bt_enter_ll_only()) return false;

    furi_message_queue_reset(rx_queue);
    acl_pending = false;
    pending_opcode = 0;
    command_status = 0xff;
    fault = false;
    if(!reset_semaphore(command_free) || !reset_semaphore(acl_free)) {
        if(full_stack) furi_hal_bt_leave_ll_only();
        fault = true;
        return false;
    }

    if(full_stack || !hci_layer_started) {
        TL_BLE_InitConf_t config = {
            .IoBusEvtCallBack = controller_event,
            .IoBusAclDataTxAck = acl_ack,
            .p_cmdbuffer = (uint8_t*)&command_buffer,
            .p_AclDataBuffer = acl_buffer,
        };
        TL_BLE_Init(&config);
    }

    furi_hal_power_insomnia_enter();

    if((full_stack || !hci_layer_started) && !ble_app_start_ll_only()) {
        if(full_stack) furi_hal_bt_leave_ll_only();
        furi_hal_power_insomnia_exit();
        fault = true;
        return false;
    }
    if(hci_layer) hci_layer_started = true;

    /* Drain callbacks queued before the raw-controller init completed. */
    furi_delay_tick(1);
    hci_layer_owned = hci_layer;
    owned = true;
    return true;
}

bool furi_hal_bt_hci_send(const uint8_t* frame, size_t length, uint32_t timeout_ms) {
    if(!owned || fault || !frame || length < 4 || length > FURI_HAL_BT_HCI_FRAME_MAX) return false;
    if(frame[0] == 1 && length == 4U + frame[3]) {
        /* This command has no response; the current mailbox contract cannot recycle it. */
        if(read_le16(frame + 1) == 0x0c35 || read_le16(frame + 1) == 0) return false;
        if(furi_semaphore_acquire(command_free, timeout_ms) != FuriStatusOk) return false;
        pending_opcode = read_le16(frame + 1);
        command_status = 0xff;
        memcpy(&command_buffer.cmdserial, frame, length);
        TL_BLE_SendCmd(NULL, 0);
        return true;
    }
    if(frame[0] == 2 && length >= 5 && length == 5U + read_le16(frame + 3)) {
        if(furi_semaphore_acquire(acl_free, timeout_ms) != FuriStatusOk) return false;
        memcpy(acl_buffer + sizeof(TL_PacketHeader_t), frame, length);
        acl_pending = true;
        TL_BLE_SendAclData(NULL, 0);
        return true;
    }
    return false;
}

int32_t furi_hal_bt_hci_receive(uint8_t* frame, size_t capacity, uint32_t timeout_ms) {
    if(!owned || fault || !frame || capacity < FURI_HAL_BT_HCI_FRAME_MAX) return -1;
    HciFrame next;
    if(furi_message_queue_get(rx_queue, &next, timeout_ms) != FuriStatusOk) return fault ? -1 : 0;
    memcpy(frame, next.bytes, next.length);
    return next.length;
}

bool furi_hal_bt_hci_release(void) {
    if(!owned) return false;
    const uint8_t reset[] = {1, 3, 12, 0};
    bool acl_idle = furi_semaphore_acquire(acl_free, 1000) == FuriStatusOk;
    if(acl_idle) furi_semaphore_release(acl_free);
    bool ok = acl_idle && furi_hal_bt_hci_send(reset, sizeof(reset), 1000);
    if(ok) {
        ok = furi_semaphore_acquire(command_free, 1000) == FuriStatusOk;
        if(ok) {
            ok = command_status == 0 && !fault;
            furi_semaphore_release(command_free);
        }
    }
    owned = false;
    bool restored = hci_layer_owned || furi_hal_bt_leave_ll_only();
    hci_layer_owned = false;
    ok = ok && restored;
    if(!ok) fault = true;
    furi_hal_power_insomnia_exit();
    return ok;
}
