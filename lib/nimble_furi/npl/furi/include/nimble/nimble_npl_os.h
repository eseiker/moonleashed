/* NimBLE NPL types for FURI. Handles are void* so this header needs no furi.h. */

#ifndef _NIMBLE_NPL_OS_H_
#define _NIMBLE_NPL_OS_H_

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_NPL_OS_ALIGNMENT 4
#define BLE_NPL_TIME_FOREVER 0xFFFFFFFFU

typedef uint32_t ble_npl_time_t;
typedef int32_t ble_npl_stime_t;

struct ble_npl_event {
    bool queued;
    ble_npl_event_fn* fn;
    void* arg;
};

struct ble_npl_eventq {
    void* q; /* FuriMessageQueue* of (struct ble_npl_event*) */
};

struct ble_npl_mutex {
    void* handle; /* FuriMutex* (recursive) */
};

struct ble_npl_sem {
    void* handle; /* FuriSemaphore* (counting) */
};

struct ble_npl_callout {
    void* handle; /* FuriTimer* */
    struct ble_npl_eventq* evq;
    struct ble_npl_event ev;
};

#ifdef __cplusplus
}
#endif

#endif /* _NIMBLE_NPL_OS_H_ */
