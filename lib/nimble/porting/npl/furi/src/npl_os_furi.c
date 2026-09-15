/*
 * NimBLE porting layer implemented on the Flipper FURI kernel.
 *
 * Every ble_npl_* entry point declared in nimble/nimble_npl.h is defined here
 * as an external function. Event queues map to FuriMessageQueue, mutexes and
 * semaphores to the recursive/counting FURI variants, and callouts to
 * single-shot FuriTimer instances. Critical sections toggle PRIMASK directly so
 * this port needs no FURI critical-section symbol.
 *
 * The NimBLE host runs entirely in thread context on Flipper (its own
 * nimble_port_run thread plus the HCI reader thread), so the FreeRTOS port's
 * from-ISR paths are not reproduced; a call from interrupt context asserts.
 */

#include <furi.h>
#include "nimble/nimble_npl.h"

/* Our FOREVER constant must match FURI's blocking sentinel. */
_Static_assert(BLE_NPL_TIME_FOREVER == FuriWaitForever, "npl forever mismatch");

static volatile uint32_t s_critical_nesting = 0;

static inline bool npl_in_isr(void) {
    return furi_kernel_is_irq_or_masked();
}

/*
 * Object registry. NimBLE never frees the OS objects it allocates, so the port
 * records every FuriTimer/queue/mutex/semaphore it hands out and frees them all
 * in npl_furi_shutdown when the FAP tears down. Freeing the timers is what lets
 * the FAP exit without a reboot: a live FuriTimer would otherwise fire its
 * callback into freed FAP memory after unload.
 */
typedef enum {
    NPL_OBJ_TIMER,
    NPL_OBJ_QUEUE,
    NPL_OBJ_MUTEX,
    NPL_OBJ_SEM,
} NplObjKind;

#define NPL_OBJ_MAX 192

static struct {
    NplObjKind kind;
    void* handle;
} s_objs[NPL_OBJ_MAX];
static size_t s_obj_count;

static void npl_register(NplObjKind kind, void* handle) {
    uint32_t ctx = ble_npl_hw_enter_critical();
    if(s_obj_count < NPL_OBJ_MAX) {
        s_objs[s_obj_count].kind = kind;
        s_objs[s_obj_count].handle = handle;
        s_obj_count++;
    }
    ble_npl_hw_exit_critical(ctx);
}

void npl_furi_shutdown(void) {
    for(size_t i = 0; i < s_obj_count; i++) {
        void* h = s_objs[i].handle;
        if(!h) continue;
        switch(s_objs[i].kind) {
        case NPL_OBJ_TIMER:
            furi_timer_stop(h);
            furi_timer_free(h);
            break;
        case NPL_OBJ_QUEUE:
            furi_message_queue_free(h);
            break;
        case NPL_OBJ_MUTEX:
            furi_mutex_free(h);
            break;
        case NPL_OBJ_SEM:
            furi_semaphore_free(h);
            break;
        }
    }
    s_obj_count = 0;
}

/*
 * Generic
 */

bool ble_npl_os_started(void) {
    return true;
}

void* ble_npl_get_current_task_id(void) {
    return furi_thread_get_current_id();
}

/*
 * Event queue
 */

void ble_npl_eventq_init(struct ble_npl_eventq* evq) {
    evq->q = furi_message_queue_alloc(32, sizeof(struct ble_npl_event*));
    npl_register(NPL_OBJ_QUEUE, evq->q);
}

bool ble_npl_eventq_is_empty(struct ble_npl_eventq* evq) {
    return furi_message_queue_get_count(evq->q) == 0;
}

struct ble_npl_event* ble_npl_eventq_get(struct ble_npl_eventq* evq, ble_npl_time_t tmo) {
    struct ble_npl_event* ev = NULL;
    if(furi_message_queue_get(evq->q, &ev, tmo) != FuriStatusOk) {
        return NULL;
    }
    if(ev) {
        ev->queued = false;
    }
    return ev;
}

void ble_npl_eventq_put(struct ble_npl_eventq* evq, struct ble_npl_event* ev) {
    if(ev->queued) {
        return;
    }
    ev->queued = true;
    furi_message_queue_put(evq->q, &ev, FuriWaitForever);
}

void ble_npl_eventq_remove(struct ble_npl_eventq* evq, struct ble_npl_event* ev) {
    if(!ev->queued) {
        return;
    }
    /*
     * FURI, like FreeRTOS, cannot pull one element from the middle of a queue,
     * so drain every queued event and put back all but the target. The queue is
     * short and this runs rarely, so the cost is acceptable.
     */
    uint32_t ctx = ble_npl_hw_enter_critical();
    uint32_t count = furi_message_queue_get_count(evq->q);
    for(uint32_t i = 0; i < count; i++) {
        struct ble_npl_event* tmp = NULL;
        if(furi_message_queue_get(evq->q, &tmp, 0) != FuriStatusOk) {
            break;
        }
        if(tmp == ev) {
            continue;
        }
        furi_message_queue_put(evq->q, &tmp, 0);
    }
    ble_npl_hw_exit_critical(ctx);
    ev->queued = false;
}

void ble_npl_event_init(struct ble_npl_event* ev, ble_npl_event_fn* fn, void* arg) {
    memset(ev, 0, sizeof(*ev));
    ev->fn = fn;
    ev->arg = arg;
}

bool ble_npl_event_is_queued(struct ble_npl_event* ev) {
    return ev->queued;
}

void* ble_npl_event_get_arg(struct ble_npl_event* ev) {
    return ev->arg;
}

void ble_npl_event_set_arg(struct ble_npl_event* ev, void* arg) {
    ev->arg = arg;
}

void ble_npl_event_run(struct ble_npl_event* ev) {
    ev->fn(ev);
}

/*
 * Mutexes
 */

ble_npl_error_t ble_npl_mutex_init(struct ble_npl_mutex* mu) {
    if(!mu) {
        return BLE_NPL_INVALID_PARAM;
    }
    mu->handle = furi_mutex_alloc(FuriMutexTypeRecursive);
    npl_register(NPL_OBJ_MUTEX, mu->handle);
    return mu->handle ? BLE_NPL_OK : BLE_NPL_ERROR;
}

ble_npl_error_t ble_npl_mutex_pend(struct ble_npl_mutex* mu, ble_npl_time_t timeout) {
    if(!mu || !mu->handle) {
        return BLE_NPL_INVALID_PARAM;
    }
    furi_check(!npl_in_isr());
    return furi_mutex_acquire(mu->handle, timeout) == FuriStatusOk ? BLE_NPL_OK : BLE_NPL_TIMEOUT;
}

ble_npl_error_t ble_npl_mutex_release(struct ble_npl_mutex* mu) {
    if(!mu || !mu->handle) {
        return BLE_NPL_INVALID_PARAM;
    }
    furi_check(!npl_in_isr());
    return furi_mutex_release(mu->handle) == FuriStatusOk ? BLE_NPL_OK : BLE_NPL_BAD_MUTEX;
}

/*
 * Semaphores
 */

ble_npl_error_t ble_npl_sem_init(struct ble_npl_sem* sem, uint16_t tokens) {
    if(!sem) {
        return BLE_NPL_INVALID_PARAM;
    }
    sem->handle = furi_semaphore_alloc(128, tokens);
    npl_register(NPL_OBJ_SEM, sem->handle);
    return sem->handle ? BLE_NPL_OK : BLE_NPL_ERROR;
}

ble_npl_error_t ble_npl_sem_pend(struct ble_npl_sem* sem, ble_npl_time_t timeout) {
    if(!sem || !sem->handle) {
        return BLE_NPL_INVALID_PARAM;
    }
    return furi_semaphore_acquire(sem->handle, timeout) == FuriStatusOk ? BLE_NPL_OK :
                                                                          BLE_NPL_TIMEOUT;
}

ble_npl_error_t ble_npl_sem_release(struct ble_npl_sem* sem) {
    if(!sem || !sem->handle) {
        return BLE_NPL_INVALID_PARAM;
    }
    return furi_semaphore_release(sem->handle) == FuriStatusOk ? BLE_NPL_OK : BLE_NPL_ERROR;
}

uint16_t ble_npl_sem_get_count(struct ble_npl_sem* sem) {
    return furi_semaphore_get_count(sem->handle);
}

/*
 * Callouts
 */

static void npl_callout_cb(void* context) {
    struct ble_npl_callout* co = context;
    if(co->evq) {
        ble_npl_eventq_put(co->evq, &co->ev);
    } else {
        co->ev.fn(&co->ev);
    }
}

void ble_npl_callout_init(
    struct ble_npl_callout* co,
    struct ble_npl_eventq* evq,
    ble_npl_event_fn* ev_cb,
    void* ev_arg) {
    memset(co, 0, sizeof(*co));
    co->handle = furi_timer_alloc(npl_callout_cb, FuriTimerTypeOnce, co);
    npl_register(NPL_OBJ_TIMER, co->handle);
    co->evq = evq;
    ble_npl_event_init(&co->ev, ev_cb, ev_arg);
}

ble_npl_error_t ble_npl_callout_reset(struct ble_npl_callout* co, ble_npl_time_t ticks) {
    if(ticks == 0) {
        ticks = 1;
    }
    furi_timer_stop(co->handle);
    furi_timer_start(co->handle, ticks);
    return BLE_NPL_OK;
}

void ble_npl_callout_stop(struct ble_npl_callout* co) {
    furi_timer_stop(co->handle);
}

bool ble_npl_callout_is_active(struct ble_npl_callout* co) {
    return furi_timer_is_running(co->handle);
}

ble_npl_time_t ble_npl_callout_get_ticks(struct ble_npl_callout* co) {
    return furi_timer_get_expire_time(co->handle);
}

ble_npl_time_t ble_npl_callout_remaining_ticks(struct ble_npl_callout* co, ble_npl_time_t now) {
    ble_npl_time_t exp = furi_timer_get_expire_time(co->handle);
    return exp > now ? exp - now : 0;
}

void ble_npl_callout_set_arg(struct ble_npl_callout* co, void* arg) {
    co->ev.arg = arg;
}

/*
 * Time
 */

ble_npl_time_t ble_npl_time_get(void) {
    return furi_get_tick();
}

ble_npl_error_t ble_npl_time_ms_to_ticks(uint32_t ms, ble_npl_time_t* out_ticks) {
    uint64_t ticks = ((uint64_t)ms * furi_kernel_get_tick_frequency()) / 1000;
    if(ticks > UINT32_MAX) {
        return BLE_NPL_EINVAL;
    }
    *out_ticks = (ble_npl_time_t)ticks;
    return BLE_NPL_OK;
}

ble_npl_error_t ble_npl_time_ticks_to_ms(ble_npl_time_t ticks, uint32_t* out_ms) {
    uint64_t ms = ((uint64_t)ticks * 1000) / furi_kernel_get_tick_frequency();
    if(ms > UINT32_MAX) {
        return BLE_NPL_EINVAL;
    }
    *out_ms = (uint32_t)ms;
    return BLE_NPL_OK;
}

ble_npl_time_t ble_npl_time_ms_to_ticks32(uint32_t ms) {
    return ((uint64_t)ms * furi_kernel_get_tick_frequency()) / 1000;
}

uint32_t ble_npl_time_ticks_to_ms32(ble_npl_time_t ticks) {
    return ((uint64_t)ticks * 1000) / furi_kernel_get_tick_frequency();
}

void ble_npl_time_delay(ble_npl_time_t ticks) {
    furi_delay_tick(ticks);
}

/*
 * Hardware critical sections (Cortex-M PRIMASK)
 */

uint32_t ble_npl_hw_enter_critical(void) {
    uint32_t primask;
    __asm volatile("mrs %0, primask" : "=r"(primask));
    __asm volatile("cpsid i" ::: "memory");
    s_critical_nesting++;
    return primask;
}

void ble_npl_hw_exit_critical(uint32_t ctx) {
    if(s_critical_nesting > 0) {
        s_critical_nesting--;
    }
    __asm volatile("msr primask, %0" ::"r"(ctx) : "memory");
}

bool ble_npl_hw_is_in_critical(void) {
    return s_critical_nesting > 0;
}
