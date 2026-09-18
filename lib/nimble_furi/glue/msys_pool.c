/*
 * NimBLE's mbuf pool, allocated from the SRAM2 the radio core does not use
 * (TASK-785).
 *
 * Every ACL packet, ATT PDU and L2CAP SDU passes through msys, and its memory
 * is held for the life of the host. Upstream keeps it in a static array,
 * os_msys_1_data, which cost 7,008 bytes of the firmware's own RAM.
 *
 * furi_hal_memory hands out the part of SRAM2A/SRAM2B that the SBRSA and
 * SNBRSA option bytes leave to this core, and nothing else claims it: on the
 * HCILayer radio `free` reports about 12.5 KB free there. SRAM1 and SRAM2 are
 * contiguous, so the block behaves like any other RAM. The pool never frees, so
 * the block is allocated once and reused when the host restarts.
 *
 * MSYS_1_BLOCK_COUNT is 0 in our syscfg, which removes the upstream array and
 * leaves upstream's os_msys_init registering nothing. nimble_glue_start calls
 * msys_pool_init straight after nimble_port_init, because os_msys_init resets
 * the pool list.
 */

#include <furi.h>
#include <core/memmgr.h>

#include "os/os.h"
#include "mem/mem.h"

#include "msys_pool.h"

#define TAG "MsysPool"

/* Upstream's sizing for msys_1, which our syscfg no longer instantiates. */
#define MSYS_BLOCK_COUNT 24
#define MSYS_BLOCK_SIZE  OS_ALIGN(MYNEWT_VAL(MSYS_1_BLOCK_SIZE), 4)

static os_membuf_t* s_data;
static struct os_mempool s_mempool;
static struct os_mbuf_pool s_mbuf_pool;

bool msys_pool_init(void) {
    if(!s_data) {
        size_t size = OS_MEMPOOL_SIZE(MSYS_BLOCK_COUNT, MSYS_BLOCK_SIZE) * sizeof(os_membuf_t);
        /* Falls back to the ordinary heap when SRAM2 is full or unavailable. */
        s_data = memmgr_alloc_from_pool(size);
        if(!s_data) {
            FURI_LOG_E(TAG, "msys alloc failed (%u bytes)", (unsigned)size);
            return false;
        }
    }

    int rc = mem_init_mbuf_pool(
        s_data, &s_mempool, &s_mbuf_pool, MSYS_BLOCK_COUNT, MSYS_BLOCK_SIZE, "msys_1");
    if(rc != 0) {
        FURI_LOG_E(TAG, "msys pool init failed: %d", rc);
        return false;
    }
    return os_msys_register(&s_mbuf_pool) == 0;
}
