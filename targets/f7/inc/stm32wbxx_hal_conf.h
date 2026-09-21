#pragma once

/* The firmware uses LL drivers only, except the PKA HAL driver: PKA has no LL
 * API for elliptic-curve multiplication, which BLE Secure Connections needs.
 * lib/nimble_furi/glue/sm_alg_pka.c provides HAL_GetTick. */

#define HAL_PKA_MODULE_ENABLED
#define USE_RTOS                       0U
#define USE_HAL_PKA_REGISTER_CALLBACKS 0U

#include "stm32_assert.h"
#include "stm32wbxx_hal_pka.h"
