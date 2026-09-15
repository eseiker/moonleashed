#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <storage/storage.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum (ceiling) size of the XIP flash region. The region is sized
 *  dynamically from whatever free flash remains after the firmware image
 *  and BLE stack security boundary; this is the upper bound above which
 *  we don't grow. 300KB = 75 pages of 4KB. */
#define XIP_REGION_MAX_SIZE (300 * 1024)

/** Minimum viable XIP region. Below this, XIP stays inactive and all
 *  apps fall back to RAM-only loading. 64KB fits a typical small FAP
 *  while leaving flash that fragmented to be clearly a bug to
 *  investigate rather than silently continue. */
#define XIP_REGION_MIN_SIZE (64 * 1024)

/** Cache header magic value ("XIPC") */
#define XIP_CACHE_MAGIC 0x58495043

/** Maximum cached sections */
#define XIP_CACHE_MAX_SECTIONS 8

/** Size reserved at start of XIP region for cache header (8-byte aligned) */
#define XIP_CACHE_HEADER_SIZE 256

/** Per-section cache entry */
typedef struct {
    uint32_t flash_offset; /**< Offset from XIP base where section data starts */
    uint32_t size; /**< Section size in bytes */
    char name[16]; /**< Section name (".text", ".rodata", etc.) */
} XipCacheSectionEntry;

/** Cache header stored at the start of the XIP flash region.
 *  Allows skipping erase/write when re-launching the same app.
 */
typedef struct {
    uint32_t magic; /**< XIP_CACHE_MAGIC */
    uint32_t file_size; /**< FAP file size (quick validation) */
    uint32_t file_crc32; /**< CRC32 of FAP file (definitive validation) */
    uint32_t api_version; /**< Firmware API version (major << 16 | minor) */
    uint32_t section_count; /**< Number of cached sections */
    uint32_t ram_addr_hash; /**< Hash of RAM section exec_addrs at cache time;
                                 if RAM sections land at different addresses on
                                 next launch the cache must be invalidated because
                                 XIP code contains relocated pointers to those addrs */
    XipCacheSectionEntry sections[XIP_CACHE_MAX_SECTIONS];
} XipCacheHeader;

/* ---- Multi-tenant directory (TASK-573) ------------------------------------
 * The region's first page holds a directory of tenants: several app images
 * coexist in flash at once, each in its own page-aligned block. Launching an
 * app that is already a tenant is a cache hit with no flash write; a miss
 * allocates a new block, evicting least-recently-used tenants if the region is
 * full. A tenant is never evicted while its app executes (it is "pinned").
 */

/** Directory magic value ("XIPD"). */
#define XIP_DIR_MAGIC 0x58495044u

/** Directory format version; bump to invalidate every cached tenant at once. */
#define XIP_DIR_FORMAT_VERSION 1u

/** Maximum tenants held at once. The whole directory must fit one flash page. */
#define XIP_MAX_TENANTS 6

/** One tenant: an app image cached in the region. */
typedef struct {
    uint32_t valid; /**< 1 if this slot holds a tenant */
    uint32_t file_size; /**< FAP file size (quick identity check) */
    uint32_t file_crc32; /**< CRC32 of the FAP file (definitive identity) */
    uint32_t api_version; /**< Firmware API version (major << 16 | minor) */
    uint32_t block_addr; /**< Page-aligned flash address of this image */
    uint32_t block_pages; /**< Image size in flash pages */
    uint32_t ram_addr_hash; /**< Hash of RAM section addresses when cached */
    uint32_t section_count; /**< Number of cached sections */
    uint32_t lru; /**< Higher is more recently used */
    XipCacheSectionEntry sections[XIP_CACHE_MAX_SECTIONS]; /**< Offsets from block_addr */
} XipTenantEntry;

/** Directory stored in the region's first flash page. */
typedef struct {
    uint32_t magic; /**< XIP_DIR_MAGIC */
    uint32_t format_version; /**< XIP_DIR_FORMAT_VERSION */
    uint32_t lru_next; /**< Next LRU value to assign */
    uint32_t reserved; /**< Padding / future use */
    XipTenantEntry tenants[XIP_MAX_TENANTS];
} XipDirectory;

/** XIP flash region bump allocator.
 *  Manages a region of internal flash used for execute-in-place loading
 *  of FAP .text and .rodata sections.
 */
typedef struct {
    uint32_t base_addr; /**< Flash start address (page-aligned). In the multi-tenant
                             path this is the tenant's block base, not the region base. */
    uint32_t end_addr; /**< Flash end address (block end in the multi-tenant path) */
    uint32_t data_start; /**< Where section data begins (block base; no in-block header) */
    uint32_t next_free; /**< Next available address (bump pointer) */
    uint32_t block_pages; /**< Tenant block size in flash pages (multi-tenant) */
    int tenant_index; /**< Directory slot of this tenant, or -1 until committed */
    uint32_t cached_ram_hash; /**< ram_addr_hash from the matched tenant (cache hit) */
    bool active; /**< Whether XIP is available */
    bool cache_valid; /**< True if cached XIP data matches current app */
    bool needs_rerelocation; /**< Cache hit but RAM addrs changed — patch in place */
} XipRegion;

/* ---- Multi-tenant manager API (TASK-573) ----------------------------------
 * A process-wide manager owns the shared region and its directory. The old
 * single-tenant functions below still exist during the migration.
 */

/** Discover the flash region and load its directory, formatting an empty one in
 *  RAM if flash holds no valid directory. Idempotent. Returns true if a region
 *  of at least XIP_REGION_MIN_SIZE is available. */
bool xip_manager_ensure(void);

/** True once xip_manager_ensure() has found a usable region. */
bool xip_manager_active(void);

/** Region geometry, valid after xip_manager_ensure() succeeds. */
uint32_t xip_manager_region_base(void);
uint32_t xip_manager_region_end(void);

/** Find a tenant matching this identity. Returns its index, or -1. */
int xip_manager_find(uint32_t file_crc32, uint32_t file_size, uint32_t api_version);

/** Read-only pointer to a tenant entry (in the RAM directory copy). */
const XipTenantEntry* xip_manager_tenant(int index);

/** Mark a tenant used now (bumps its LRU). */
void xip_manager_touch(int index);

/** Allocate a page-aligned block of `pages` pages, evicting least-recently-used
 *  non-pinned tenants if needed. Returns the block's flash address, or 0. The
 *  RAM directory is updated (evictions applied); the caller then fills a tenant
 *  slot and calls xip_manager_commit(). */
uint32_t xip_manager_alloc_block(uint32_t pages);

/** Store a tenant entry into a free directory slot in RAM. Returns the slot
 *  index, or -1 if the directory is full. */
int xip_manager_put_tenant(const XipTenantEntry* entry);

/** Update a tenant's cached ram_addr_hash in the RAM directory (after an
 *  in-place re-relocation). Caller commits the directory afterwards. */
void xip_manager_update_hash(int index, uint32_t ram_addr_hash);

/** Invalidate a tenant slot in the RAM directory (frees its pages for reuse). */
void xip_manager_invalidate(int index);

/** Write the RAM directory back to its flash page (erase + write). */
bool xip_manager_commit(void);

/** Pin / unpin a block range so eviction never touches a running app. */
void xip_manager_pin(uint32_t block_addr, uint32_t block_pages);
void xip_manager_unpin(uint32_t block_addr, uint32_t block_pages);

/** Initialize XIP region from free flash.
 *  Reserves space for the cache header at the start.
 *  Queries the free flash area and checks if at least XIP_REGION_MAX_SIZE
 *  bytes are available. Sets region->active = true on success.
 *
 *  @param region   XIP region to initialize
 */
void xip_region_init(XipRegion* region);

/** Validate the XIP cache against the current FAP file.
 *  Reads the cache header from flash and compares file size, CRC, and API version.
 *
 *  @param region           XIP region (must be initialized)
 *  @param fd               open file handle to the FAP file
 *  @param api_version_major firmware API major version
 *  @param api_version_minor firmware API minor version
 *  @return                 true if cache is valid (can skip erase/write)
 */
bool xip_cache_validate(
    XipRegion* region,
    File* fd,
    uint16_t api_version_major,
    uint16_t api_version_minor);

/** Write a pre-built cache header to the XIP flash region.
 *  The header area must have been erased as part of the XIP erase cycle.
 *
 *  @param region   XIP region
 *  @param header   fully-populated cache header to write
 *  @return         true on success
 */
bool xip_cache_commit_header(XipRegion* region, const XipCacheHeader* header);

/** Get the cached header from flash (read-only, memory-mapped).
 *
 *  @param region   XIP region
 *  @return         pointer to flash-resident header, or NULL if not active
 */
const XipCacheHeader* xip_cache_get_header(const XipRegion* region);

/** Allocate address space from the XIP region (bump allocator).
 *  Does NOT erase or write flash — just advances the pointer.
 *  Alignment is rounded up to 8 (flash write block size) minimum.
 *
 *  @param region       XIP region
 *  @param size         bytes to allocate
 *  @param alignment    required alignment (will be clamped to >= 8)
 *  @return             flash address, or 0 on failure
 */
uint32_t xip_region_alloc(XipRegion* region, size_t size, size_t alignment);

/** Erase only the flash pages that have been allocated.
 *  Must be called after all xip_region_alloc() calls and before
 *  xip_region_commit() calls.
 *
 *  @param region   XIP region
 *  @return         true on success
 */
bool xip_region_erase(XipRegion* region);

/** Write RAM staging buffer to pre-erased flash.
 *
 *  @param region       XIP region (for bounds checking)
 *  @param flash_addr   destination address in flash (must be 8-byte aligned)
 *  @param ram_data     source data in RAM
 *  @param size         number of bytes to write
 *  @return             true on success
 */
bool xip_region_commit(XipRegion* region, uint32_t flash_addr, const void* ram_data, size_t size);

/** Release the XIP region so another app can use it.
 *  Called when the app using XIP is freed.
 *
 *  @param region   XIP region to release
 */
void xip_region_release(XipRegion* region);

/** Get total bytes allocated so far.
 *
 *  @param region   XIP region
 *  @return         bytes used
 */
size_t xip_region_used(const XipRegion* region);

#ifdef __cplusplus
}
#endif
