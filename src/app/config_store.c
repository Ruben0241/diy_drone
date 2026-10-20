/*
 * config_store.c - Einstellungen im letzten 64-KB-Block des Blackbox-Flash
 *
 * Der Block liegt hinter dem Logbereich und wird von "erase" nicht beruehrt.
 * Format: siehe params_pack() (Text "name=wert" mit CRC32).
 */
#include <string.h>

#include "app_shared.h"
#include "flash_w25q.h"
#include "params.h"

#define BLOCK_SIZE  65536u
#define BLOB_MAX    4096u

static uint8_t blob[BLOB_MAX];

uint32_t config_store_addr(void)
{
    const uint32_t cap = flash_capacity();
    return cap >= 2u * BLOCK_SIZE ? cap - BLOCK_SIZE : 0u;
}

int config_store_load(flight_config_t *cfg)
{
    const uint32_t addr = config_store_addr();
    if (!addr || !flash_read(addr, blob, sizeof(blob))) return -1;
    return params_unpack(cfg, blob, (int)sizeof(blob));
}

bool config_store_save(const flight_config_t *cfg)
{
    const uint32_t addr = config_store_addr();
    if (!addr) return false;
    memset(blob, 0xFF, sizeof(blob));
    const int n = params_pack(cfg, blob, (int)sizeof(blob));
    if (n < 0) return false;
    if (!flash_erase_block64k(addr)) return false;
    for (uint32_t off = 0; off < (uint32_t)n; off += 256u) {
        if (!flash_program_page(addr + off, &blob[off])) return false;
    }
    /* zur Kontrolle zuruecklesen */
    flight_config_t check;
    flight_default_config(&check);
    return config_store_load(&check) == params_count();
}
