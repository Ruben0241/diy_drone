/*
 * peripherals.h - LEDs, Summer, Akkuspannung
 */
#pragma once

#include <stdbool.h>

void  indicators_init(void);
void  led_set(int led, bool on);
void  beeper_set(bool on);

void  vbat_init(void);
float vbat_read_volts(void);    /* blockierend, ca. 10 us */
