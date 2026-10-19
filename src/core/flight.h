/*
 * flight.h - Flugsteuerung ohne Hardwarebezug
 *
 * Ein Aufruf von flight_step() entspricht einem Regelzyklus:
 *   Gyro-Kalibrierung -> Filter -> Lageschaetzung -> Arming/Failsafe ->
 *   Sollwerte (Rate- oder Angle-Mode) -> PID -> Mischer -> DShot-Werte
 *
 * Weil hier keine Register vorkommen, laesst sich das Modul auf dem PC
 * testen (siehe tests/).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "pt1.h"
#include "mahony.h"
#include "mixer.h"
#include "crsf.h"
#include "util.h"

/* Kanalbelegung (EdgeTX/ELRS Standard "AETR") */
enum { CH_ROLL = 0, CH_PITCH = 1, CH_THROTTLE = 2, CH_YAW = 3, CH_ARM = 4, CH_MODE = 5 };

typedef struct {
    float loop_hz;                 /* Regelfrequenz */
    float pid_p[AXIS_COUNT];       /* Betaflight-Zahlen */
    float pid_i[AXIS_COUNT];
    float pid_d[AXIS_COUNT];
    float i_limit;                 /* PID-Einheiten (Betaflight: 400) */
    float sum_limit[AXIS_COUNT];   /* Betaflight: 500 / 500 / 400 */
    float gyro_lpf_hz;
    float dterm_lpf_hz;
    float max_rate_dps[AXIS_COUNT];/* lineare Kennlinie: Vollausschlag = max_rate */
    float stick_deadband;          /* 0 ... 1 */
    float angle_max_deg;           /* Angle-Mode: maximaler Winkel */
    float angle_kp;                /* Angle-Mode: Soll-Rate [dps] pro Grad Fehler */
    float mahony_kp, mahony_ki;
    float motor_idle;              /* 0 ... 1, Leerlauf bei Gas 0 (Betaflight: 0.055) */
    float arm_throttle_max;        /* Arming nur unter diesem Gas */
    float arm_angle_max_deg;       /* Arming nur, wenn flacher als dieser Winkel */
    float iterm_reset_throttle;    /* darunter wird I zurueckgesetzt (am Boden) */
    float gyro_sign[AXIS_COUNT];   /* +1/-1: Vorzeichen-Korrektur, Bodentest! */
    float stick_sign[AXIS_COUNT];  /* +1/-1 */
    bool  yaw_motors_reversed;
    uint32_t gyro_cal_samples;     /* Anzahl Werte fuer die Kalibrierung */
    float gyro_cal_max_dev_dps;    /* erlaubte Bewegung waehrend Kalibrierung */
} flight_config_t;

typedef enum { MODE_RATE = 0, MODE_ANGLE = 1 } flight_mode_t;

typedef struct {
    float    gyro_dps[AXIS_COUNT];   /* bereits ausgerichtet (Koerpersystem) */
    float    acc_g[3];
    uint16_t channels[CRSF_NUM_CHANNELS];
    bool     rc_valid;               /* aktuelles Paket innerhalb des Timeouts */
} flight_input_t;

typedef struct {
    flight_config_t cfg;
    pid_axis_t      pid[AXIS_COUNT];
    pt1_t           gyro_lpf[AXIS_COUNT];
    mahony_t        ahrs;
    mixer_config_t  mixer;

    /* Kalibrierung */
    bool     calibrated;
    uint32_t cal_count;
    float    cal_sum[AXIS_COUNT];
    float    cal_min[AXIS_COUNT], cal_max[AXIS_COUNT];
    float    gyro_bias[AXIS_COUNT];

    /* Zustand */
    bool          armed;
    bool          failsafe;
    bool          arm_switch_seen_off;
    flight_mode_t mode;

    /* Ergebnisse dieses Zyklus (auch fuers Logging) */
    float    gyro_f[AXIS_COUNT];
    float    setpoint[AXIS_COUNT];
    float    attitude_deg[3];        /* roll, pitch, yaw */
    float    throttle;
    float    motor[MIXER_MOTOR_COUNT];
    uint16_t dshot[MIXER_MOTOR_COUNT];

    /* Ereignisse (fuer Summer/LED), werden jeden Zyklus neu gesetzt */
    bool     event_armed, event_disarmed, event_failsafe;
} flight_t;

void flight_default_config(flight_config_t *cfg);
void flight_init(flight_t *f, const flight_config_t *cfg);
void flight_step(flight_t *f, const flight_input_t *in);
