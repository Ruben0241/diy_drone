/*
 * rates.h - Stickkurve: Stickausschlag -> Soll-Drehrate
 *
 * Gleiche Formel wie Betaflight "Actual Rates" (src/main/fc/rc.c, applyActualRates),
 * damit Build A und Build B exakt gleich eingestellt werden koennen:
 *
 *   e(x) = |x| * (x^5 * expo + x * (1 - expo))
 *   rate = x * center + max(0, max_rate - center) * e(x)
 *
 *   x: Stick -1..1, center: Empfindlichkeit in der Mitte [dps], max_rate [dps], expo 0..1
 * Linear (center = max_rate, expo beliebig): rate = x * max_rate
 */
#pragma once

float rates_apply(float stick, float center_dps, float max_dps, float expo);
