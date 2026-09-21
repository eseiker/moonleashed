#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Call after nimble_port_init, which resets the pool list. */
bool msys_pool_init(void);

#ifdef __cplusplus
}
#endif
