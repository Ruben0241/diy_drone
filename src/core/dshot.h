/*
 * dshot.h - DShot-Kodierung (Begleitarbeit Kap. 2.5)
 *
 * Frame (16 Bit, MSB zuerst): [11 Bit Gas][1 Bit Telemetrie][4 Bit CRC]
 *   0 = Motor aus, 1..47 = Befehle, 48..2047 = Gas
 *   CRC = (v ^ (v >> 4) ^ (v >> 8)) & 0x0F,  v = (Gas << 1) | Telemetrie
 *
 * Fuer die Ausgabe mit Timer + DMA wird jedes Bit in einen Vergleichswert
 * (High-Zeit in Timer-Ticks) uebersetzt. Ein zusaetzlicher 0-Wert am Ende
 * sorgt dafuer, dass die Leitung nach dem Frame low bleibt.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define DSHOT_BITS          16u
#define DSHOT_BUFFER_SLOTS  (DSHOT_BITS + 1u)
#define DSHOT_MIN_THROTTLE  48u
#define DSHOT_MAX_THROTTLE  2047u

uint16_t dshot_make_frame(uint16_t value, bool telemetry);
/* Motorwert 0 ... 1 -> DShot-Wert 48 ... 2047 (mit Leerlauf "idle" 0 ... 1) */
uint16_t dshot_from_output(float out, float idle);

/* Schreibt die Vergleichswerte eines Frames.
 * stride: Abstand zwischen zwei Bits im Puffer (4 bei Burst-DMA fuer 4 Motoren). */
void dshot_fill_buffer(uint32_t *buf, uint32_t stride, uint16_t frame,
                       uint32_t ticks_zero, uint32_t ticks_one);
