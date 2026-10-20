/*
 * flash_w25q.h - SPI-NOR-Flash (Blackbox-Speicher, W25Q128 oder kompatibel) an SPI2
 *
 * Standardbefehle, die praktisch alle SPI-NOR-Flashs verstehen:
 *   0x9F JEDEC-ID, 0x06 Schreibfreigabe, 0x05 Status, 0x03 Lesen,
 *   0x02 Seite schreiben (256 Byte), 0xD8 64-KB-Block loeschen, 0xC7 alles loeschen
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

bool     flash_init(void);          /* false = kein Flash gefunden */
uint32_t flash_jedec_id(void);      /* z. B. 0xEF4018 = Winbond W25Q128 */
uint32_t flash_capacity(void);      /* Byte, 0 = unbekannt */

bool flash_read(uint32_t addr, uint8_t *buf, uint32_t len);
bool flash_program_page(uint32_t addr, const uint8_t *buf);  /* 256 Byte, Seitenanfang */
bool flash_erase_block64k(uint32_t addr);
bool flash_erase_chip(void);

/* Wird waehrend langer Wartezeiten (Loeschen) aufgerufen, z. B. vTaskDelay(1) */
void flash_set_wait_hook(void (*hook)(void));
