/*
 * pid.h - Drehraten-PID-Regler (eine Achse)
 *
 * Die Verstaerkungen werden in denselben Zahlen angegeben wie in Betaflight
 * (z. B. P=45, I=80, D=30). Intern werden sie mit denselben Skalierungs-
 * faktoren wie in Betaflight umgerechnet (PTERM_SCALE usw.), damit die Werte
 * zwischen Build A und Build B direkt vergleichbar sind.
 *
 * Ausgang: "PID-Summe" in Betaflight-Einheiten (1000 = voller Motorbereich).
 * Fuer den Mischer wird durch PID_MIXER_SCALING geteilt.
 *
 * Diskrete Form (vgl. Begleitarbeit Kap. 2.2, Gl. 2.3):
 *   P = Kp * e
 *   I = I + Ki * e * dt            (begrenzt, Anti-Windup)
 *   D = Kd * (-(y_k - y_{k-1}) / dt), D auf den Istwert, PT1-gefiltert
 */
#pragma once

#include <stdbool.h>
#include "pt1.h"

/* Skalierung wie Betaflight (src/main/flight/pid.h) */
#define PID_PTERM_SCALE     0.032029f
#define PID_ITERM_SCALE     0.244381f
#define PID_DTERM_SCALE     0.000529f
#define PID_MIXER_SCALING   1000.0f

typedef struct {
    float kp, ki, kd;        /* bereits skaliert */
    float i_limit;           /* Betrag-Grenze fuer I (PID-Einheiten) */
    float sum_limit;         /* Betrag-Grenze fuer P+I+D */
    float dt;                /* Abtastzeit [s] */

    float i_term;
    float prev_measurement;
    bool  has_prev;
    pt1_t dterm_lpf;

    /* letzte Werte, fuer Logging */
    float p, i, d, out;
} pid_axis_t;

void  pid_init(pid_axis_t *pid, float p_gain, float i_gain, float d_gain,
               float dt, float dterm_lpf_hz, float i_limit, float sum_limit);
float pid_update(pid_axis_t *pid, float setpoint, float measurement, bool freeze_i);
void  pid_reset(pid_axis_t *pid);
