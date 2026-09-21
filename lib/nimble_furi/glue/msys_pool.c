/* NimBLE's msys pool, from SRAM2 while it has room, instead of a static array. */

#include <furi.h>
#include <core/memmgr.h>

#include "os/os.h"
#include "mem/mem.h"

#include "msys_pool.h"

#define TAG "MsysPool"

#define MSYS_BLOCK_COUNT 24
#define MSYS_BLOCK_SIZE  OS_ALIGN(MYNEWT_VAL(MSYS_1_BLOCK_SIZE), 4)

static os_membuf_t* s_data;
static struct os_mempool s_mempool;
static struct os_mbuf_pool s_mbuf_pool;

bool msys_pool_init(void) {
    if(!s_data) {
        size_t size = OS_MEMPOOL_SIZE(MSYS_BLOCK_COUNT, MSYS_BLOCK_SIZE) * sizeof(os_membuf_t);
        /* Falls back to the heap; never NULL. */
        s_data = memmgr_alloc_from_pool(size);
    }

    int rc = mem_init_mbuf_pool(
        s_data, &s_mempool, &s_mbuf_pool, MSYS_BLOCK_COUNT, MSYS_BLOCK_SIZE, "msys_1");
    if(rc != 0) {
        FURI_LOG_E(TAG, "msys pool init failed: %d", rc);
        return false;
    }
    return os_msys_register(&s_mbuf_pool) == 0;
}
