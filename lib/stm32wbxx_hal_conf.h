/*
 * Minimal ST HAL configuration for this firmware (TASK-698).
 *
 * The firmware is LL-only: lib/stm32wb.scons builds nothing but *_ll_*.c. One
 * HAL driver is an exception, stm32wbxx_hal_pka.c, because the PKA accelerator
 * has no LL API for elliptic-curve scalar multiplication and the resident
 * NimBLE host needs P-256 for Secure Connections.
 *
 * Only PKA is enabled here. Adding another module means adding its source to
 * lib/stm32wb.scons as well, and checking what else it drags in.
 *
 * HAL_GetTick is the only HAL core function the PKA driver calls; it is
 * provided in lib/nimble_furi/glue/sm_alg_pka.c on top of FURI's tick.
 */

#ifndef STM32WBxx_HAL_CONF_H
#define STM32WBxx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_PKA_MODULE_ENABLED

/* Clock values. The HAL core uses these; PKA itself does not. */
#if !defined(HSE_VALUE)
#define HSE_VALUE (32000000UL)
#endif
#if !defined(HSI_VALUE)
#define HSI_VALUE (16000000UL)
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE (32768UL)
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE (32000UL)
#endif
#if !defined(MSI_VALUE)
#define MSI_VALUE (4000000UL)
#endif
#if !defined(EXTERNAL_SAI1_CLOCK_VALUE)
#define EXTERNAL_SAI1_CLOCK_VALUE (2097000UL)
#endif
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT (100UL)
#endif
#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT (5000UL)
#endif

#define VDD_VALUE                (3300UL)
#define TICK_INT_PRIORITY        ((1UL << __NVIC_PRIO_BITS) - 1UL)
#define USE_RTOS                 0U
#define PREFETCH_ENABLE          0U
#define INSTRUCTION_CACHE_ENABLE  1U
#define DATA_CACHE_ENABLE         1U

#define USE_HAL_PKA_REGISTER_CALLBACKS 0U

/* assert_param comes from targets/f7/inc/stm32_assert.h, which the LL drivers
 * already use; do not define another one here. */
#include "stm32_assert.h"

#include "stm32wbxx_hal_pka.h"

#ifdef __cplusplus
}
#endif

#endif /* STM32WBxx_HAL_CONF_H */
