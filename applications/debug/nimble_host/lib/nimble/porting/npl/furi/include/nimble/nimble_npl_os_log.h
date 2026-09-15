/*
 * OS-specific NPL logging hook for the FURI port.
 *
 * nimble/nimble_npl_log.h expands BLE_NPL_LOG_IMPL(level) for each level to
 * generate a per-module logging function. NimBLE's internal NPL logging is not
 * routed anywhere on Flipper, so these expand to empty inline functions; this
 * keeps vprintf and its formatting code out of the FAP.
 */

#ifndef _NIMBLE_NPL_OS_LOG_H_
#define _NIMBLE_NPL_OS_LOG_H_

#define BLE_NPL_LOG_IMPL(lvl)                                                            \
    static inline void _BLE_NPL_LOG_CAT(                                                 \
        BLE_NPL_LOG_MODULE, _BLE_NPL_LOG_CAT(_, lvl))(const char* fmt, ...) {            \
        (void)fmt;                                                                       \
    }

#endif /* _NIMBLE_NPL_OS_LOG_H_ */
