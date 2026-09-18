/*
 * NimBLE's mbuf pool in the SRAM2 the radio core leaves free (TASK-785).
 * See msys_pool.c.
 */

#ifndef MSYS_POOL_H_
#define MSYS_POOL_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Allocate (once) and register the msys pool. Call after nimble_port_init,
 * which resets the pool list. Returns false if the memory could not be had. */
bool msys_pool_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MSYS_POOL_H_ */
