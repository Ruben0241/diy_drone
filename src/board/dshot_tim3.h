/*
 * dshot_tim3.h - DShot-Ausgabe auf 4 Motoren mit TIM3 und DMA-Burst
 *
 * Alle vier Motorpins liegen auf TIM3 (CH1..CH4). Bei jedem Update-Ereignis
 * des Timers schreibt der DMA-Controller (DMA1 Stream 2, Kanal 5 = TIM3_UP)
 * ueber das DMAR-Register vier Vergleichswerte nacheinander in CCR1..CCR4.
 * Damit werden alle vier Frames gleichzeitig ausgegeben, ohne dass die CPU
 * pro Bit eingreifen muss (Begleitarbeit Kap. 2.5).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum { DSHOT150 = 150, DSHOT300 = 300, DSHOT600 = 600 } dshot_rate_t;

void dshot_init(dshot_rate_t rate);
/* values: 0 = aus, 48..2047 = Gas; Reihenfolge M1..M4 */
bool dshot_write(const uint16_t values[4]);
/* Notaus: Ausgaenge sofort auf low (z. B. im Fehler-Handler) */
void dshot_emergency_stop(void);
