/*
 * Controller capability probe for the CoC / multi-role track (TASK-615).
 * See coc_glue.h. Runs standard HCI reads through the host's command channel
 * (the same path nimble_glue.c uses for the ST vendor address command) and logs
 * the raw results for offline decode against the Core spec LE States table.
 */

#include <furi.h>

#include "nimble/ble.h"
#include "nimble/hci_common.h"

#include "coc_glue.h"

#define TAG "CocProbe"

/* Private host HCI command channel (ble_hs_hci_priv.h). Declared here like
 * nimble_glue.c declares ble_hs_id_set_pub, so the glue can issue a standard
 * (non-vendor) HCI command and read its return parameters. */
extern int ble_hs_hci_cmd_tx(
    uint16_t opcode,
    const void* cmd,
    uint8_t cmd_len,
    void* rsp,
    uint8_t rsp_len);

void coc_probe_capabilities(void) {
    /* LE Read Supported States (OGF 0x08, OCF 0x001C): a 64-bit mask of the
     * role/state combinations the controller allows simultaneously. Bits for
     * "Scanning + Connection (Peripheral)" and "Initiating + Connection
     * (Peripheral)" say whether a CoC central can run while the companion
     * peripheral link is up. Logged as two 32-bit halves (newlib-nano printf has
     * no %llX). */
    struct ble_hci_le_rd_supp_states_rp states = {0};
    int rc = ble_hs_hci_cmd_tx(
        BLE_HCI_OP(BLE_HCI_OGF_LE, BLE_HCI_OCF_LE_RD_SUPP_STATES),
        NULL,
        0,
        &states,
        sizeof(states));
    if(rc == 0) {
        uint32_t hi = (uint32_t)(states.states >> 32);
        uint32_t lo = (uint32_t)(states.states & 0xFFFFFFFFu);
        FURI_LOG_I(TAG, "LE supported states: 0x%08lX%08lX", hi, lo);
    } else {
        FURI_LOG_E(TAG, "LE Read Supported States failed: %d", rc);
    }

    /* LE Read Buffer Size (OCF 0x0002): ACL payload length and packet count the
     * controller can buffer — a capacity hint for holding two links at once. */
    struct ble_hci_le_rd_buf_size_rp buf = {0};
    rc = ble_hs_hci_cmd_tx(
        BLE_HCI_OP(BLE_HCI_OGF_LE, BLE_HCI_OCF_LE_RD_BUF_SIZE),
        NULL,
        0,
        &buf,
        sizeof(buf));
    if(rc == 0) {
        FURI_LOG_I(
            TAG, "LE buffer size: pkt_len=%u num_pkts=%u", buf.data_len, buf.data_packets);
    } else {
        FURI_LOG_E(TAG, "LE Read Buffer Size failed: %d", rc);
    }
}
