/* NPL logging is compiled out. */

#ifndef _NIMBLE_NPL_OS_LOG_H_
#define _NIMBLE_NPL_OS_LOG_H_

#define BLE_NPL_LOG_IMPL(lvl)                                                          \
    static inline void _BLE_NPL_LOG_CAT(BLE_NPL_LOG_MODULE, _BLE_NPL_LOG_CAT(_, lvl))( \
        const char* fmt, ...) {                                                        \
        (void)fmt;                                                                     \
    }

#endif /* _NIMBLE_NPL_OS_LOG_H_ */
