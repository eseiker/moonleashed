#include "elf_file_xip.h"
#include <furi_hal_flash.h>
#include <furi.h>
#include <toolbox/crc32_calc.h>

#define TAG "XIP"

static bool xip_region_in_use = false;

void xip_region_init(XipRegion* region) {
    furi_check(region);
    memset(region, 0, sizeof(XipRegion));

    if(xip_region_in_use) {
        FURI_LOG_W(TAG, "XIP region already in use by another app, skipping");
        return;
    }

    size_t free_start = furi_hal_flash_get_free_page_start_address();
    size_t free_end = (size_t)furi_hal_flash_get_free_end_address();

    if(free_end <= free_start) {
        FURI_LOG_W(TAG, "No free flash available");
        return;
    }

    size_t free_size = free_end - free_start;
    size_t region_size = MIN(free_size, (size_t)XIP_REGION_MAX_SIZE);

    if(region_size < XIP_REGION_MIN_SIZE) {
        FURI_LOG_W(
            TAG,
            "Flash too fragmented for XIP: %zu available, %u minimum",
            region_size,
            XIP_REGION_MIN_SIZE);
        return;
    }

    region->base_addr = (uint32_t)free_start;
    region->end_addr = region->base_addr + region_size;
    /* Reserve space for cache header at start of region */
    region->data_start = region->base_addr + XIP_CACHE_HEADER_SIZE;
    region->next_free = region->data_start;
    region->active = true;
    region->cache_valid = false;
    xip_region_in_use = true;

    FURI_LOG_I(
        TAG,
        "Region initialized: 0x%08lX - 0x%08lX (%zu KB, header at base)",
        region->base_addr,
        region->end_addr,
        region_size / 1024);
}

void xip_region_release(XipRegion* region) {
    furi_check(region);
    if(region->active) {
        xip_region_in_use = false;
        region->active = false;
        FURI_LOG_D(TAG, "Region released");
    }
}

bool xip_cache_validate(
    XipRegion* region,
    File* fd,
    uint16_t api_version_major,
    uint16_t api_version_minor) {
    furi_check(region);
    furi_check(fd);

    if(!region->active) return false;

    /* Read cache header directly from flash (memory-mapped on STM32) */
    const XipCacheHeader* header = (const XipCacheHeader*)(region->base_addr);

    /* Tier 1: Quick checks (instant, no I/O) */
    if(header->magic != XIP_CACHE_MAGIC) {
        FURI_LOG_I(TAG, "Cache miss: no valid header (magic=0x%08lX)", header->magic);
        return false;
    }

    uint32_t current_api = ((uint32_t)api_version_major << 16) | api_version_minor;
    if(header->api_version != current_api) {
        FURI_LOG_I(
            TAG,
            "Cache miss: API version mismatch (cached=%08lX, current=%08lX)",
            header->api_version,
            current_api);
        return false;
    }

    uint64_t file_size = storage_file_size(fd);
    if(header->file_size != (uint32_t)file_size) {
        FURI_LOG_I(
            TAG,
            "Cache miss: file size mismatch (cached=%lu, current=%llu)",
            header->file_size,
            file_size);
        return false;
    }

    if(header->section_count == 0 || header->section_count > XIP_CACHE_MAX_SECTIONS) {
        FURI_LOG_I(TAG, "Cache miss: invalid section count %lu", header->section_count);
        return false;
    }

    /* Tier 2: CRC check (requires reading entire FAP file, ~50-80ms) */
    uint32_t tick_start = furi_get_tick();
    uint32_t file_crc = crc32_calc_file(fd, NULL, NULL);
    storage_file_seek(fd, 0, true); /* Reset position after full-file read */
    uint32_t crc_ms = furi_get_tick() - tick_start;

    if(header->file_crc32 != file_crc) {
        FURI_LOG_I(
            TAG,
            "Cache miss: CRC mismatch (cached=%08lX, computed=%08lX, %lums)",
            header->file_crc32,
            file_crc,
            crc_ms);
        return false;
    }

    FURI_LOG_I(
        TAG,
        "Cache HIT: %lu sections, CRC %08lX verified in %lums",
        header->section_count,
        file_crc,
        crc_ms);

    region->cache_valid = true;
    return true;
}

/** Write a pre-built cache header to the XIP flash region.
 *  The header area (first 256 bytes) must already be erased.
 */
bool xip_cache_commit_header(XipRegion* region, const XipCacheHeader* header) {
    furi_check(region);
    furi_check(header);

    if(!region->active) return false;

    /* Write header to the reserved area at region base.
     * This area was erased as part of xip_region_erase (it's the first page). */
    furi_hal_flash_write_block(
        region->base_addr, (const uint8_t*)header, sizeof(XipCacheHeader));

    FURI_LOG_I(
        TAG,
        "Cache header written: %lu sections, CRC %08lX",
        header->section_count,
        header->file_crc32);

    return true;
}

/** Get the cached header from flash (read-only, memory-mapped). */
const XipCacheHeader* xip_cache_get_header(const XipRegion* region) {
    if(!region->active) return NULL;
    return (const XipCacheHeader*)(region->base_addr);
}

uint32_t xip_region_alloc(XipRegion* region, size_t size, size_t alignment) {
    furi_check(region);
    if(!region->active || size == 0) return 0;

    /* Flash writes require 8-byte alignment minimum */
    if(alignment < 8) alignment = 8;

    /* Align the bump pointer */
    uint32_t aligned = (region->next_free + alignment - 1) & ~(alignment - 1);

    if(aligned + size > region->end_addr) {
        FURI_LOG_E(
            TAG,
            "Alloc failed: need %zu at 0x%08lX, end 0x%08lX",
            size,
            aligned,
            region->end_addr);
        return 0;
    }

    region->next_free = aligned + size;

    FURI_LOG_D(TAG, "Alloc %zu bytes at 0x%08lX", size, aligned);
    return aligned;
}

bool xip_region_erase(XipRegion* region) {
    furi_check(region);
    if(!region->active) return false;

    /* Erase from base_addr (including header area) to next_free */
    if(region->next_free == region->data_start) return true; /* nothing allocated */

    size_t page_size = furi_hal_flash_get_page_size();
    int16_t first_page = furi_hal_flash_get_page_number(region->base_addr);
    int16_t last_page = furi_hal_flash_get_page_number(region->next_free - 1);

    furi_check(first_page >= 0);
    furi_check(last_page >= 0);

    FURI_LOG_I(
        TAG, "Erasing pages %d - %d (%zu KB)", first_page, last_page,
        ((last_page - first_page + 1) * page_size) / 1024);

    for(int16_t page = first_page; page <= last_page; page++) {
        furi_hal_flash_erase(page);
        /* Yield every 4 pages to let BLE stack process events. */
        if((page - first_page) % 4 == 3) {
            furi_delay_tick(1);
        }
    }

    return true;
}

bool xip_region_commit(
    XipRegion* region,
    uint32_t flash_addr,
    const void* ram_data,
    size_t size) {
    furi_check(region);
    furi_check(ram_data);

    if(!region->active) return false;
    if(size == 0) return true;

    /* Bounds check */
    if(flash_addr < region->base_addr || (flash_addr + size) > region->end_addr) {
        FURI_LOG_E(TAG, "Commit out of bounds: 0x%08lX + %zu", flash_addr, size);
        return false;
    }

    /* Alignment check */
    if(flash_addr & 0x7) {
        FURI_LOG_E(TAG, "Commit address not 8-byte aligned: 0x%08lX", flash_addr);
        return false;
    }

    /* Use bulk write for speed — single begin/end cycle */
    furi_hal_flash_write_block(flash_addr, (const uint8_t*)ram_data, size);

    FURI_LOG_D(TAG, "Committed %zu bytes to 0x%08lX", size, flash_addr);
    return true;
}

size_t xip_region_used(const XipRegion* region) {
    furi_check(region);
    if(!region->active) return 0;
    return region->next_free - region->data_start;
}

/* ==== Multi-tenant manager (TASK-573) ======================================
 * A process-wide owner of the shared XIP region and its on-flash directory.
 * The single-tenant functions above are still used by the current loader path;
 * the manager is wired in by the elf_file.c integration milestone.
 */

#define XIP_MGR_MAX_PINNED (XIP_MAX_TENANTS + 2)

static struct {
    bool attached;
    uint32_t base; /**< region start (directory page) */
    uint32_t end; /**< region end */
    uint32_t page_size;
    uint32_t total_pages; /**< pages in the whole region, directory included */
    XipDirectory dir; /**< RAM working copy of the flash directory */
    struct {
        uint32_t addr;
        uint32_t pages;
    } pinned[XIP_MGR_MAX_PINNED];
    size_t pinned_count;
} xip_mgr;

static void xip_manager_format(void) {
    memset(&xip_mgr.dir, 0, sizeof(xip_mgr.dir));
    xip_mgr.dir.magic = XIP_DIR_MAGIC;
    xip_mgr.dir.format_version = XIP_DIR_FORMAT_VERSION;
    xip_mgr.dir.lru_next = 1;
}

bool xip_manager_ensure(void) {
    if(xip_mgr.attached) return true;

    size_t free_start = furi_hal_flash_get_free_page_start_address();
    size_t free_end = (size_t)furi_hal_flash_get_free_end_address();
    if(free_end <= free_start) return false;

    size_t region_size = MIN((size_t)(free_end - free_start), (size_t)XIP_REGION_MAX_SIZE);
    if(region_size < XIP_REGION_MIN_SIZE) return false;

    xip_mgr.base = (uint32_t)free_start;
    xip_mgr.page_size = furi_hal_flash_get_page_size();
    /* Whole pages only; a partial tail page is unusable for a page-aligned block. */
    xip_mgr.total_pages = region_size / xip_mgr.page_size;
    xip_mgr.end = xip_mgr.base + xip_mgr.total_pages * xip_mgr.page_size;
    xip_mgr.pinned_count = 0;

    /* Load the directory from flash (memory-mapped), or format a fresh one. */
    const XipDirectory* flash_dir = (const XipDirectory*)xip_mgr.base;
    if(flash_dir->magic == XIP_DIR_MAGIC &&
       flash_dir->format_version == XIP_DIR_FORMAT_VERSION) {
        memcpy(&xip_mgr.dir, flash_dir, sizeof(xip_mgr.dir));
        FURI_LOG_I(TAG, "Directory loaded from flash");
    } else {
        xip_manager_format();
        FURI_LOG_I(TAG, "No valid directory; starting empty");
    }

    xip_mgr.attached = true;
    FURI_LOG_I(
        TAG,
        "Manager attached: 0x%08lX - 0x%08lX (%lu pages)",
        xip_mgr.base,
        xip_mgr.end,
        xip_mgr.total_pages);
    return true;
}

bool xip_manager_active(void) {
    return xip_mgr.attached;
}

uint32_t xip_manager_region_base(void) {
    return xip_mgr.base;
}

uint32_t xip_manager_region_end(void) {
    return xip_mgr.end;
}

int xip_manager_find(uint32_t file_crc32, uint32_t file_size, uint32_t api_version) {
    if(!xip_mgr.attached) return -1;
    for(int i = 0; i < XIP_MAX_TENANTS; i++) {
        const XipTenantEntry* t = &xip_mgr.dir.tenants[i];
        if(t->valid && t->file_crc32 == file_crc32 && t->file_size == file_size &&
           t->api_version == api_version) {
            return i;
        }
    }
    return -1;
}

const XipTenantEntry* xip_manager_tenant(int index) {
    if(!xip_mgr.attached || index < 0 || index >= XIP_MAX_TENANTS) return NULL;
    return &xip_mgr.dir.tenants[index];
}

void xip_manager_touch(int index) {
    if(!xip_mgr.attached || index < 0 || index >= XIP_MAX_TENANTS) return;
    xip_mgr.dir.tenants[index].lru = xip_mgr.dir.lru_next++;
}

/* First page (page 0) is the directory; page indices below are region-relative. */
static uint32_t xip_page_of(uint32_t addr) {
    return (addr - xip_mgr.base) / xip_mgr.page_size;
}

static bool xip_page_pinned(uint32_t page) {
    for(size_t i = 0; i < xip_mgr.pinned_count; i++) {
        uint32_t p0 = xip_page_of(xip_mgr.pinned[i].addr);
        if(page >= p0 && page < p0 + xip_mgr.pinned[i].pages) return true;
    }
    return false;
}

/* Mark which region pages are occupied by valid tenants. page_used[0] (the
 * directory) is always set. */
static void xip_mark_used(bool* page_used) {
    memset(page_used, 0, xip_mgr.total_pages * sizeof(bool));
    page_used[0] = true;
    for(int i = 0; i < XIP_MAX_TENANTS; i++) {
        const XipTenantEntry* t = &xip_mgr.dir.tenants[i];
        if(!t->valid) continue;
        uint32_t p0 = xip_page_of(t->block_addr);
        for(uint32_t p = p0; p < p0 + t->block_pages && p < xip_mgr.total_pages; p++) {
            page_used[p] = true;
        }
    }
}

/* Find the first run of `pages` contiguous free pages. Returns the start page,
 * or 0 (never valid, page 0 is the directory) if none. */
static uint32_t xip_find_free_run(const bool* page_used, uint32_t pages) {
    uint32_t run = 0;
    for(uint32_t p = 1; p < xip_mgr.total_pages; p++) {
        if(page_used[p]) {
            run = 0;
            continue;
        }
        if(++run == pages) return p - pages + 1;
    }
    return 0;
}

/* Evict the least-recently-used valid, non-pinned tenant. Returns false if none
 * can be evicted. */
static bool xip_evict_one(void) {
    int victim = -1;
    uint32_t best_lru = UINT32_MAX;
    for(int i = 0; i < XIP_MAX_TENANTS; i++) {
        const XipTenantEntry* t = &xip_mgr.dir.tenants[i];
        if(!t->valid) continue;
        if(xip_page_pinned(xip_page_of(t->block_addr))) continue;
        if(t->lru < best_lru) {
            best_lru = t->lru;
            victim = i;
        }
    }
    if(victim < 0) return false;
    FURI_LOG_I(TAG, "Evicting tenant %d (lru %lu)", victim, best_lru);
    xip_mgr.dir.tenants[victim].valid = 0;
    return true;
}

uint32_t xip_manager_alloc_block(uint32_t pages) {
    if(!xip_mgr.attached || pages == 0) return 0;
    if(pages > xip_mgr.total_pages - 1) return 0; /* cannot fit even alone */

    bool* page_used = malloc(xip_mgr.total_pages * sizeof(bool));
    if(!page_used) return 0;

    uint32_t start_page = 0;
    while(1) {
        xip_mark_used(page_used);
        start_page = xip_find_free_run(page_used, pages);
        if(start_page != 0) break;
        if(!xip_evict_one()) break; /* nothing left to free */
    }
    free(page_used);

    if(start_page == 0) {
        FURI_LOG_E(TAG, "No room for %lu pages even after eviction", pages);
        return 0;
    }
    return xip_mgr.base + start_page * xip_mgr.page_size;
}

int xip_manager_put_tenant(const XipTenantEntry* entry) {
    if(!xip_mgr.attached || !entry) return -1;
    for(int i = 0; i < XIP_MAX_TENANTS; i++) {
        if(!xip_mgr.dir.tenants[i].valid) {
            xip_mgr.dir.tenants[i] = *entry;
            xip_mgr.dir.tenants[i].valid = 1;
            return i;
        }
    }
    return -1;
}

bool xip_manager_commit(void) {
    if(!xip_mgr.attached) return false;
    int16_t page = furi_hal_flash_get_page_number(xip_mgr.base);
    furi_check(page >= 0);
    furi_hal_flash_erase(page);
    furi_hal_flash_write_block(xip_mgr.base, (const uint8_t*)&xip_mgr.dir, sizeof(xip_mgr.dir));
    FURI_LOG_I(TAG, "Directory committed");
    return true;
}

void xip_manager_pin(uint32_t block_addr, uint32_t block_pages) {
    if(!xip_mgr.attached || block_pages == 0) return;
    if(xip_mgr.pinned_count >= XIP_MGR_MAX_PINNED) {
        FURI_LOG_E(TAG, "Pin table full");
        return;
    }
    xip_mgr.pinned[xip_mgr.pinned_count].addr = block_addr;
    xip_mgr.pinned[xip_mgr.pinned_count].pages = block_pages;
    xip_mgr.pinned_count++;
}

void xip_manager_unpin(uint32_t block_addr, uint32_t block_pages) {
    for(size_t i = 0; i < xip_mgr.pinned_count; i++) {
        if(xip_mgr.pinned[i].addr == block_addr && xip_mgr.pinned[i].pages == block_pages) {
            xip_mgr.pinned[i] = xip_mgr.pinned[xip_mgr.pinned_count - 1];
            xip_mgr.pinned_count--;
            return;
        }
    }
}
