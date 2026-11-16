/* NimBLE glue functions exported to CLI plugins (e.g. cli_bt.fal).
 *
 * Upstream ships each CLI command as a separate .fal plugin, so the few glue
 * functions bt_cli.c calls must resolve against the firmware API table. Keep
 * this subset small: only symbols an external CLI plugin needs belong here.
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* True once the NimBLE host has started on an acquired controller. */
bool nimble_glue_is_started(void);

/* Crypto self-tests against the NIST P-256 and RFC 4493 vectors. */
bool sm_alg_pka_selftest(void);
bool sm_cmac_selftest(void);

#ifdef __cplusplus
}
#endif
