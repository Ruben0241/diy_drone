/*
 * util.h - kleine Hilfsfunktionen (hardwareunabhaengig)
 */
#pragma once

#include <stdint.h>

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f

static inline float constrainf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float maxf(float a, float b) { return a > b ? a : b; }
static inline float minf(float a, float b) { return a < b ? a : b; }

/* Achsen-Indizes, ueberall gleich verwendet */
enum { AXIS_ROLL = 0, AXIS_PITCH = 1, AXIS_YAW = 2, AXIS_COUNT = 3 };
