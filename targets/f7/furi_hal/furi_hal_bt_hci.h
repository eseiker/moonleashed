#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FURI_HAL_BT_HCI_FRAME_MAX 260U

#ifdef __cplusplus
extern "C" {
#endif

/* Interrupt context, frame starts with the H4 type. Set before acquire; with
 * a callback in place no receive queue exists and furi_hal_bt_hci_receive
 * returns -1. */
typedef void (*FuriHalBtHciRxCallback)(const uint8_t* frame, size_t length, void* context);
void furi_hal_bt_hci_set_rx_callback(FuriHalBtHciRxCallback callback, void* context);

bool furi_hal_bt_hci_acquire(void);
bool furi_hal_bt_hci_send(const uint8_t* frame, size_t length, uint32_t timeout_ms);
/* Returns frame length, 0 on timeout, -1 on fault. */
int32_t furi_hal_bt_hci_receive(uint8_t* frame, size_t capacity, uint32_t timeout_ms);
/* Resets the controller. Failure requires a reboot. */
bool furi_hal_bt_hci_release(void);

#ifdef __cplusplus
}
#endif
