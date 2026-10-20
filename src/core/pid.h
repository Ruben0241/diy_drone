/*
 * pid.h - Drehraten-Regler einer Achse (P, I, D, Feedforward)
 *
 * Verstaerkungen in Betaflight-Zahlen (z. B. P=45, I=80, D=30, F=120), intern mit
 * denselben Skalierungsfaktoren wie Betaflight (src/main/flight/pid.h). So sind die
 * Werte von Build A und Build B direkt vergleichbar.
 *
 * Ausgang in Betaflight-Einheiten (1000 = voller Motorbereich).
 *
 *   e = Soll - Ist
 *   P = Kp * e
 *   I = I + Ki * e * dt * relax * windup     (begrenzt auf +-i_limit)
 *   D = -Kd * d(Ist)/dt * tpa                 (D auf den Istwert, zweistufig gefiltert)
 *   F = Kf * d(Soll)/dt                       (Feedforward, gefiltert)
 *
 * I-Term-Relax (wie Betaflight, Typ "Setpoint"): Waehrend schneller Stickbewegungen
 * wird das Aufintegrieren gebremst, sonst ueberschwingt die Drohne nach Flips:
 *   hp = |Soll - PT1(Soll)|,  relax = max(0, 1 - hp / 40 dps)
 */
#pragma once

#include <stdbool.h>
#include "biquad.h"
#include "pt1.h"

#define PID_PTERM_SCALE        0.032029f
#define PID_ITERM_SCALE        0.244381f
#define PID_DTERM_SCALE        0.000529f
#define PID_FEEDFORWARD_SCALE  0.013754f
#define PID_MIXER_SCALING      1000.0f
#define ITERM_RELAX_THRESHOLD  40.0f      /* dps, wie Betaflight */

typedef struct {
    float p_gain, i_gain, d_gain, f_gain;  /* Betaflight-Zahlen */
    float i_limit;                         /* PID-Einheiten (Betaflight: 400) */
    float sum_limit;                       /* PID-Einheiten (Betaflight: 500 / 400 Yaw) */
    float dt;
    int   dterm_lpf1_type; float dterm_lpf1_hz;
    int   dterm_lpf2_type; float dterm_lpf2_hz;
    float ff_lpf_hz;                       /* Glaettung des Feedforward (0 = aus) */
    float p_lpf_hz;                        /* PT1 auf den P-Anteil (Betaflight nur Yaw: 100 Hz; 0 = aus) */
    bool  iterm_relax;
    float iterm_relax_hz;                  /* Betaflight: 15 Hz */
} pid_params_t;

typedef struct {
    float i_factor;    /* 0..1: Anti-Windup bei Motorsaettigung (1 = normal) */
    float d_factor;    /* 0..1: TPA (Throttle PID Attenuation) */
    bool  stabilise;   /* false: Ausgang 0 und I zuruecksetzen (am Boden, Gas unten) */
} pid_runtime_t;

typedef struct {
    float kp, ki, kd, kf;
    float i_limit, sum_limit, dt;
    bool  iterm_relax;

    float i_term;
    float prev_measurement, prev_setpoint;
    bool  has_prev;
    lpf_stage_t dterm1, dterm2;
    pt1_t ff_lpf, relax_lpf, p_lpf;
    bool  p_lpf_on;

    /* letzte Werte (Logging, Anzeige, Runaway-Erkennung) */
    float p, i, d, f;
    float sum_raw;     /* P+I+D+F vor der Begrenzung */
    float out;         /* begrenzt */
} pid_axis_t;

void  pid_init(pid_axis_t *pid, const pid_params_t *prm);
float pid_update(pid_axis_t *pid, float setpoint, float measurement, const pid_runtime_t *rt);
void  pid_reset(pid_axis_t *pid);
