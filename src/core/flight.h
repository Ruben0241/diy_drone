/*
 * flight.h - Flugsteuerung ohne Hardwarebezug
 *
 * Ein Aufruf von flight_step() entspricht einem Regelzyklus:
 *   Gyro (Vorzeichen, Kalibrierung, Filter) -> Lageschaetzung (Mahony) ->
 *   Sticks (Totband, RC-Glaettung, Rates) -> Arming/Failsafe ->
 *   Sollwerte (Rate- oder Angle-Mode) -> PID je Achse -> Mischer -> DShot-Werte
 *
 * Weil hier keine Register vorkommen, laesst sich das Modul auf dem PC testen
 * und in einer Physik-Simulation betreiben (tests/, tools/sim/).
 *
 * Alle Einstellungen sind float, damit sie ueber USB gesetzt und gespeichert
 * werden koennen (params.c). Ja/Nein-Werte: 0 oder 1.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "biquad.h"
#include "crsf.h"
#include "mahony.h"
#include "mixer.h"
#include "pid.h"
#include "pt1.h"
#include "util.h"

/* Kanalbelegung (EdgeTX/ELRS Standard "AETR") */
enum { CH_ROLL = 0, CH_PITCH = 1, CH_THROTTLE = 2, CH_YAW = 3, CH_ARM = 4, CH_MODE = 5 };

typedef struct {
    float loop_hz;                 /* Regelfrequenz */

    /* PID (Betaflight-Zahlen) */
    float pid_p[AXIS_COUNT], pid_i[AXIS_COUNT], pid_d[AXIS_COUNT], pid_f[AXIS_COUNT];
    float i_limit;                 /* Betaflight: 400 */
    float sum_limit[AXIS_COUNT];   /* Betaflight: 500 / 500 / 400 */
    float iterm_relax;             /* 0 aus, 1 Roll/Pitch, 2 Roll/Pitch/Yaw */
    float iterm_relax_hz;
    float iterm_windup;            /* Saettigungsgrenze 0..1 fuer Anti-Windup (1 = aus) */
    float tpa_rate;                /* 0..1: D-Abschwaechung bei Vollgas (0 = aus) */
    float tpa_breakpoint;          /* Gas 0..1, ab dem TPA wirkt */

    /* Filter (Typ: 0 aus, 1 PT1, 2 Biquad) */
    float gyro_lpf1_type, gyro_lpf1_hz, gyro_lpf2_type, gyro_lpf2_hz;
    float gyro_notch_hz, gyro_notch_cutoff_hz;            /* 0 = aus */
    float dterm_lpf1_type, dterm_lpf1_hz, dterm_lpf2_type, dterm_lpf2_hz;
    float acc_lpf_hz;
    float rc_smoothing_hz;         /* PT1 auf die Sticks (0 = aus) */
    float ff_smoothing_hz;

    /* Stickkurve (Betaflight "Actual Rates") */
    float rate_center[AXIS_COUNT], rate_max[AXIS_COUNT], rate_expo[AXIS_COUNT];
    float stick_deadband;          /* 0 ... 1 */

    /* Angle-Mode */
    float angle_max_deg;           /* Betaflight angle_limit: 60 */
    float angle_kp;                /* Soll-Rate [dps] pro Grad Fehler (Betaflight angle_p_gain 50 -> 5) */

    /* Lageschaetzung */
    float mahony_kp, mahony_ki;

    /* Motoren, Arming, Sicherheit */
    float motor_idle;              /* 0 ... 1, Leerlauf (Betaflight dshot_idle_value 550 -> 0.055) */
    float arm_throttle_max;
    float arm_angle_max_deg;
    float low_throttle;            /* darunter: Gas "unten" */
    float airmode_start;           /* Gas, ab dem die Regelung auch bei Gas 0 aktiv bleibt */
    float runaway_prevention;      /* 1 = Schutz vor "durchgehender" Drohne beim Start */

    /* Ausrichtung und Korrekturen (Bodentest) */
    float gyro_sign[AXIS_COUNT];
    float stick_sign[AXIS_COUNT];
    float yaw_motors_reversed;
    float acc_bias[3];             /* ACC-Kalibrierung [g] */

    float gyro_cal_samples;
    float gyro_cal_max_dev_dps;
} flight_config_t;

typedef enum { MODE_RATE = 0, MODE_ANGLE = 1 } flight_mode_t;

typedef struct {
    float    gyro_dps[AXIS_COUNT];   /* bereits ausgerichtet (Koerpersystem) */
    float    acc_g[3];
    uint16_t channels[CRSF_NUM_CHANNELS];
    bool     rc_valid;               /* aktuelles Paket innerhalb des Timeouts */
    bool     arm_inhibit;            /* z. B. waehrend Log-Download/Loeschen ueber USB */
    bool     motor_test;             /* Motortest (nur disarmed, ohne Propeller!) */
    float    motor_test_value[4];    /* 0 ... 1 */
} flight_input_t;

typedef struct {
    flight_config_t cfg;
    pid_axis_t      pid[AXIS_COUNT];
    lpf_stage_t     gyro_lpf1[AXIS_COUNT], gyro_lpf2[AXIS_COUNT];
    biquad_t        gyro_notch[AXIS_COUNT];
    bool            notch_on;
    pt1_t           acc_lpf[3];
    pt1_t           rc_lpf[AXIS_COUNT];
    mahony_t        ahrs;
    mixer_config_t  mixer;

    /* Gyro-Kalibrierung */
    bool     calibrated;
    uint32_t cal_count;
    float    cal_sum[AXIS_COUNT];
    float    cal_min[AXIS_COUNT], cal_max[AXIS_COUNT];
    float    gyro_bias[AXIS_COUNT];

    /* ACC-Kalibrierung */
    bool     acc_cal_running, acc_cal_done;
    uint32_t acc_cal_count;
    float    acc_cal_sum[3];

    /* Zustand */
    bool          armed;
    bool          failsafe;
    bool          arm_switch_seen_off;
    bool          airmode_active;
    bool          runaway_armed;        /* Runaway-Schutz noch aktiv (vor dem Abheben) */
    float         runaway_time_s, takeoff_time_s;
    bool          runaway_triggered;
    float         mix_range;            /* Saettigung des letzten Zyklus */
    flight_mode_t mode;

    /* Ergebnisse dieses Zyklus (auch fuers Logging) */
    float    gyro_f[AXIS_COUNT];
    float    acc_f[3];
    float    setpoint[AXIS_COUNT];
    float    attitude_deg[3];        /* roll, pitch, yaw */
    float    throttle;
    float    motor[MIXER_MOTOR_COUNT];
    uint16_t dshot[MIXER_MOTOR_COUNT];

    /* Ereignisse (fuer Summer/LED), werden jeden Zyklus neu gesetzt */
    bool     event_armed, event_disarmed, event_failsafe, event_runaway;
} flight_t;

void flight_default_config(flight_config_t *cfg);
void flight_init(flight_t *f, const flight_config_t *cfg);
/* Neue Einstellungen uebernehmen, Kalibrierung und Lage bleiben erhalten (nur disarmed) */
bool flight_apply_config(flight_t *f, const flight_config_t *cfg);
void flight_step(flight_t *f, const flight_input_t *in);
/* Sofort disarmen (z. B. Gyro-Ausfall). Neues Armen erst nach Schalter aus -> ein. */
void flight_force_disarm(flight_t *f);
/* ACC-Kalibrierung starten: Drohne waehrend ca. 0.5 s still und waagrecht halten */
void flight_start_acc_calibration(flight_t *f);
