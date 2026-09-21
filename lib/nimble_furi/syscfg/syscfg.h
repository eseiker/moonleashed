#pragma once

/* Overrides on top of upstream's generated config, which guards every value with #ifndef. */

#define MYNEWT_VAL_BLE_ROLE_CENTRAL  (0)
#define MYNEWT_VAL_BLE_ROLE_OBSERVER (0)
#define MYNEWT_VAL_BLE_WHITELIST     (0)
#define MYNEWT_VAL_BLE_HCI_VS        (1)

#define MYNEWT_VAL_BLE_ATT_SVR_READ_MULT    (0)
#define MYNEWT_VAL_BLE_ATT_SVR_SIGNED_WRITE (0)

/* Pair like the stock stack: PIN on screen, bonded. */
#define MYNEWT_VAL_BLE_SM_BONDING        (1)
#define MYNEWT_VAL_BLE_SM_MITM           (1)
#define MYNEWT_VAL_BLE_SM_IO_CAP         (BLE_HS_IO_DISPLAY_ONLY)
#define MYNEWT_VAL_BLE_SM_OUR_KEY_DIST   (3)
#define MYNEWT_VAL_BLE_SM_THEIR_KEY_DIST (3)

/* msys_pool.c registers the pool from SRAM2 instead. */
#define MYNEWT_VAL_MSYS_1_BLOCK_COUNT (0)

/* The HCILayer controller has no ISO. */
#define MYNEWT_VAL_BLE_TRANSPORT_ISO_COUNT         (0)
#define MYNEWT_VAL_BLE_TRANSPORT_ISO_FROM_HS_COUNT (0)
#define MYNEWT_VAL_BLE_TRANSPORT_ISO_FROM_LL_COUNT (0)

#include "../../nimble/porting/examples/linux/include/syscfg/syscfg.h"
