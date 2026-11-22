/*
 * peripherals.h - LEDs, Summer, Akkuspannung
 */
#pragma once

#include <stdbool.h>

void  indicators_init(void);
void  led_set(int led, bool on);
void  beeper_set(bool on);

void  vbat_init(void);
/* blockierend, ca. 20 us; scale wie Betaflight vbat_scale (Standard 110) */
float vbat_read_volts(float scale);
