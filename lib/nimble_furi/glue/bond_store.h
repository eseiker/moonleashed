#pragma once

/* NimBLE keeps bonds in RAM; these persist them to the SD card. */
void bond_store_load(void);
void bond_store_save(void);
void bond_store_forget(void);
