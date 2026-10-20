/*
 * params.h - Einstellungen per Name lesen/setzen und als Text speichern
 *
 * Jede Einstellung hat einen Namen (wie in Betaflight, wo moeglich), einen
 * erlaubten Bereich und zeigt auf ein Feld in flight_config_t.
 * Gespeichert wird als Text "name=wert\n". Das ist robust gegen Aenderungen
 * an der Struktur: unbekannte Namen werden ignoriert, fehlende behalten den
 * Standardwert.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "flight.h"

typedef struct {
    const char *name;
    uint16_t    offset;     /* Byte-Offset in flight_config_t */
    float       min, max;
    uint8_t     decimals;   /* fuer die Anzeige */
    bool        reinit;     /* Aenderung braucht Neuinitialisierung der Filter/Regler */
} param_def_t;

int                params_count(void);
const param_def_t *params_def(int i);
int                params_find(const char *name);
float              params_get(const flight_config_t *cfg, int i);
/* false, wenn Name unbekannt, Zahl ungueltig oder ausserhalb des Bereichs */
bool               params_set(flight_config_t *cfg, const char *name, const char *value);

/* Zahl ohne strtof (newlib strtof braucht malloc) */
bool               params_parse_float(const char *s, float *out);
/* "name = wert" in buf, Rueckgabe Laenge */
int                params_format(const flight_config_t *cfg, int i, char *buf, int cap);

/* Als Text speichern/laden. Rueckgabe: Laenge bzw. Anzahl gesetzter Werte (-1 = Fehler) */
int                params_serialize(const flight_config_t *cfg, char *buf, int cap);
int                params_deserialize(flight_config_t *cfg, const char *text, int len);

uint32_t           params_crc32(const void *data, size_t len);

/* Speicherblock: [Magic "DFC1"][Laenge][CRC32 des Texts][Text]. Rueckgabe Laenge / -1 */
#define PARAMS_MAGIC     0x31434644u
#define PARAMS_HEADER    12
int                params_pack(const flight_config_t *cfg, uint8_t *blob, int cap);
/* Rueckgabe: Anzahl gesetzter Werte, -1 = kein gueltiger Block (cfg unveraendert) */
int                params_unpack(flight_config_t *cfg, const uint8_t *blob, int len);
