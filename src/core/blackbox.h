/*
 * blackbox.h - Aufzeichnung der Flugdaten in einen SPI-NOR-Flash
 *
 * Die Daten werden ab Adresse 0 lueckenlos in ganzen Seiten (256 Byte)
 * geschrieben. Beim Start wird das Ende der Daten per binaerer Suche
 * gefunden (erste Seite, die mit 0xFF beginnt). Geloescht wird nur auf
 * Befehl (USB), nie waehrend des Flugs.
 *
 * Der Zugriff auf den Flash laeuft ueber Funktionszeiger. So laesst sich das
 * Modul auf dem PC mit einem simulierten Flash im RAM testen.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "logrec.h"

#define BB_PAGE_SIZE   256u
#define BB_BLOCK_SIZE  65536u

typedef struct {
    bool (*read)(uint32_t addr, uint8_t *buf, uint32_t len);
    bool (*program_page)(uint32_t addr, const uint8_t *buf);   /* genau 256 Byte */
    bool (*erase_block)(uint32_t addr);                        /* 64 KB */
    bool (*erase_chip)(void);
    uint32_t capacity;                                         /* Byte */
} bb_flash_ops_t;

typedef struct {
    const bb_flash_ops_t *ops;
    uint32_t write_addr;          /* naechste freie Seite */
    uint8_t  page[BB_PAGE_SIZE];
    uint32_t page_fill;           /* Byte im Seitenpuffer */
    bool     full;
    bool     ok;                  /* Flash erkannt */
    uint32_t records_written;
    uint32_t records_dropped;
    uint32_t write_errors;
} blackbox_t;

bool     bb_init(blackbox_t *bb, const bb_flash_ops_t *ops);
/* Einen 32-Byte-Datensatz anhaengen (Seite wird geschrieben, sobald voll) */
bool     bb_append(blackbox_t *bb, const uint8_t rec[LOGREC_SIZE]);
/* Rest der Seite mit Fuellsaetzen auffuellen und schreiben (beim Disarmen) */
void     bb_flush(blackbox_t *bb);
uint32_t bb_used_bytes(const blackbox_t *bb);
/* Geschriebenen Bereich loeschen (ganze 64-KB-Bloecke), progress: 0..100 */
bool     bb_erase_used(blackbox_t *bb, void (*progress)(uint32_t percent));
bool     bb_erase_all(blackbox_t *bb);
