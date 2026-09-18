/**
 * @file furi_hal_memory.h
 * Memory HAL API
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Init memory pool manager
 */
void furi_hal_memory_init(void);

/**
 * @brief Allocate memory from separate memory pool. That memory can't be freed.
 * 
 * @param size 
 * @return void* 
 */
void* furi_hal_memory_alloc(size_t size);

/**
 * @brief Get free memory pool size
 * 
 * @return size_t 
 */
size_t furi_hal_memory_get_free(void);

/**
 * @brief Get max free block size from memory pool
 * 
 * @return size_t 
 */
size_t furi_hal_memory_max_pool_block(void);

/** Get free space in one SRAM2 region
 *
 * @param      region  0 for SRAM2A, 1 for SRAM2B
 *
 * @return     free bytes in that region, 0 if the pool is unavailable
 */
size_t furi_hal_memory_region_free(uint8_t region);

#ifdef __cplusplus
}
#endif
