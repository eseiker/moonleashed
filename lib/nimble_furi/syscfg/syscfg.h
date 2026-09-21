#pragma once

/* Overrides on top of upstream's generated config, which guards every value with #ifndef. */

/* The companion link plus one central link. */
#define MYNEWT_VAL_BLE_MAX_CONNECTIONS (2)
#define MYNEWT_VAL_BLE_WHITELIST       (0)
#define MYNEWT_VAL_BLE_HCI_VS          (1)

/* NimBLE adds COC_MAX_NUM to the channel pool itself. Two receive buffers per
 * channel: l2cap_coc.c sizes its pool for them. */
#define MYNEWT_VAL_BLE_L2CAP_COC_MAX_NUM        (2)
#define MYNEWT_VAL_BLE_L2CAP_COC_SDU_BUFF_COUNT (2)

#define MYNEWT_VAL_BLE_ATT_SVR_READ_MULT    (0)
#define MYNEWT_VAL_BLE_ATT_SVR_SIGNED_WRITE (0)

/* Pair like the stock stack: PIN on screen, bonded, SC allowed. */
#define MYNEWT_VAL_BLE_SM_BONDING        (1)
#define MYNEWT_VAL_BLE_SM_MITM           (1)
#define MYNEWT_VAL_BLE_SM_IO_CAP         (BLE_HS_IO_DISPLAY_ONLY)
#define MYNEWT_VAL_BLE_SM_OUR_KEY_DIST   (3)
#define MYNEWT_VAL_BLE_SM_THEIR_KEY_DIST (3)
#define MYNEWT_VAL_BLE_SM_SC             (1)

/* msys_pool.c registers the pool from SRAM2 instead. */
#define MYNEWT_VAL_MSYS_1_BLOCK_COUNT (0)

/* The HCILayer controller has no ISO. */
#define MYNEWT_VAL_BLE_TRANSPORT_ISO_COUNT         (0)
#define MYNEWT_VAL_BLE_TRANSPORT_ISO_FROM_HS_COUNT (0)
#define MYNEWT_VAL_BLE_TRANSPORT_ISO_FROM_LL_COUNT (0)

#include "../../nimble/porting/examples/linux/include/syscfg/syscfg.h"
