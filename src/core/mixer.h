/*
 * mixer.h - Quad-X-Mischer (Begleitarbeit Kap. 2.1, Gl. 2.1)
 *
 * Motorreihenfolge und Vorzeichen wie Betaflight "QUADX":
 *   Motor 1 = hinten rechts, 2 = vorne rechts, 3 = hinten links, 4 = vorne links
 *   u_i = T + r_i * roll + p_i * pitch + y_i * yaw
 *
 * Wie in Betaflight wird der Yaw-Anteil bei normaler Propellerrichtung
 * ("props in", yaw_motors_reversed = OFF) negiert.
 *
 * Desaturierung ("Airmode"): Reicht der Motorbereich nicht, werden die
 * Regleranteile verkleinert und der Gaswert verschoben, damit die Lage-
 * regelung immer Vorrang vor dem Gas hat.
 */
#pragma once

#include <stdbool.h>

#define MIXER_MOTOR_COUNT 4

typedef struct {
    bool yaw_motors_reversed;
} mixer_config_t;

/* throttle: 0 ... 1, roll/pitch/yaw: PID-Summe / PID_MIXER_SCALING (ca. -0.5 ... 0.5)
 * out: Motorwerte 0 ... 1
 * Rueckgabe: Spannweite der Regleranteile (max - min) vor dem Verkleinern.
 *            > 1 bedeutet Saettigung (fuer Anti-Windup und Runaway-Erkennung). */
float mixer_mix(const mixer_config_t *cfg, float throttle, float roll, float pitch, float yaw,
               float out[MIXER_MOTOR_COUNT]);
