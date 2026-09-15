/*
 * Bond persistence for the NimBLE host FAP (TASK-589, follow-up #3).
 *
 * NimBLE keeps bonds in the RAM store (ble_store_config.c), which is lost when
 * the FAP exits, forcing a re-pair every launch. These helpers persist the RAM
 * store to a file on the Flipper's storage and restore it on start, so a phone
 * stays bonded across FAP runs. They use only the public ble_store iterate/write
 * API plus the Flipper Storage record, so the vendored store is unmodified.
 *
 * Only glue .c files include this header.
 */

#ifndef SERIAL_STORE_H_
#define SERIAL_STORE_H_

/* Load persisted bonds from storage into the (already-initialised) RAM store.
 * Call after ble_store_config_init() and before advertising. */
void serial_store_load(void);

/* Write the current RAM store (our-sec, peer-sec, CCCDs) to storage. Call when
 * a bond is established or changed. */
void serial_store_save(void);

/* Delete the persisted bond file (bt_forget_bonded_devices). */
void serial_store_forget(void);

#endif /* SERIAL_STORE_H_ */
