#include "flight.h"
#include "dshot.h"
#include <math.h>
#include <string.h>

void flight_default_config(flight_config_t *c)
{
    memset(c, 0, sizeof(*c));
    c->loop_hz = 1000.0f;

    /* Betaflight-Standardwerte (src/main/flight/pid.h, PID_*_DEFAULT) */
    c->pid_p[AXIS_ROLL] = 45; c->pid_i[AXIS_ROLL] = 80; c->pid_d[AXIS_ROLL] = 30;
    c->pid_p[AXIS_PITCH] = 47; c->pid_i[AXIS_PITCH] = 84; c->pid_d[AXIS_PITCH] = 34;
    c->pid_p[AXIS_YAW] = 45; c->pid_i[AXIS_YAW] = 80; c->pid_d[AXIS_YAW] = 0;
    c->i_limit = 400.0f;
    c->sum_limit[AXIS_ROLL] = 500.0f;
    c->sum_limit[AXIS_PITCH] = 500.0f;
    c->sum_limit[AXIS_YAW] = 400.0f;

    /* Bei 1 kHz liegt die Nyquist-Frequenz bei 500 Hz */
    c->gyro_lpf_hz = 150.0f;
    c->dterm_lpf_hz = 75.0f;

    c->max_rate_dps[AXIS_ROLL] = 400.0f;
    c->max_rate_dps[AXIS_PITCH] = 400.0f;
    c->max_rate_dps[AXIS_YAW] = 400.0f;
    c->stick_deadband = 0.01f;

    c->angle_max_deg = 45.0f;
    c->angle_kp = 5.0f;

    c->mahony_kp = 0.25f;   /* wie Betaflight imu_dcm_kp = 2500 */
    c->mahony_ki = 0.0f;

    c->motor_idle = 0.055f;
    c->arm_throttle_max = 0.05f;
    c->arm_angle_max_deg = 25.0f;
    c->iterm_reset_throttle = 0.05f;

    for (int a = 0; a < AXIS_COUNT; a++) {
        c->gyro_sign[a] = 1.0f;
        c->stick_sign[a] = 1.0f;
    }
    c->yaw_motors_reversed = false;

    c->gyro_cal_samples = 1000;       /* 1 s bei 1 kHz */
    c->gyro_cal_max_dev_dps = 4.0f;
}

static void cal_restart(flight_t *f)
{
    f->cal_count = 0;
    for (int a = 0; a < AXIS_COUNT; a++) {
        f->cal_sum[a] = 0.0f;
        f->cal_min[a] = 1e9f;
        f->cal_max[a] = -1e9f;
    }
}

void flight_init(flight_t *f, const flight_config_t *cfg)
{
    memset(f, 0, sizeof(*f));
    f->cfg = *cfg;
    const float dt = 1.0f / cfg->loop_hz;

    for (int a = 0; a < AXIS_COUNT; a++) {
        pid_init(&f->pid[a], cfg->pid_p[a], cfg->pid_i[a], cfg->pid_d[a], dt,
                 cfg->dterm_lpf_hz, cfg->i_limit, cfg->sum_limit[a]);
        pt1_init(&f->gyro_lpf[a], cfg->gyro_lpf_hz, dt);
    }
    mahony_init(&f->ahrs, cfg->mahony_kp, cfg->mahony_ki);
    f->mixer.yaw_motors_reversed = cfg->yaw_motors_reversed;
    cal_restart(f);
}

static void gyro_calibration(flight_t *f, const float raw[AXIS_COUNT])
{
    for (int a = 0; a < AXIS_COUNT; a++) {
        f->cal_sum[a] += raw[a];
        f->cal_min[a] = minf(f->cal_min[a], raw[a]);
        f->cal_max[a] = maxf(f->cal_max[a], raw[a]);
        if (f->cal_max[a] - f->cal_min[a] > 2.0f * f->cfg.gyro_cal_max_dev_dps) {
            cal_restart(f);   /* Drohne wurde bewegt: von vorne beginnen */
            return;
        }
    }
    if (++f->cal_count >= f->cfg.gyro_cal_samples) {
        for (int a = 0; a < AXIS_COUNT; a++) {
            f->gyro_bias[a] = f->cal_sum[a] / (float)f->cal_count;
        }
        f->calibrated = true;
    }
}

static float apply_deadband(float x, float db)
{
    if (fabsf(x) <= db) return 0.0f;
    return (x > 0.0f) ? (x - db) / (1.0f - db) : (x + db) / (1.0f - db);
}

static void update_arming(flight_t *f, const flight_input_t *in, float throttle)
{
    const bool arm_switch = in->rc_valid && crsf_to_unipolar(in->channels[CH_ARM]) > 0.5f;

    /* Failsafe: keine gueltigen Pakete -> sofort Motoren aus */
    if (!in->rc_valid) {
        if (!f->failsafe) {
            f->event_failsafe = true;
        }
        f->failsafe = true;
        f->arm_switch_seen_off = false;   /* nach Verbindungsverlust Schalter neu betaetigen */
        if (f->armed) {
            f->armed = false;
            f->event_disarmed = true;
        }
        return;
    }
    f->failsafe = false;

    if (!arm_switch) {
        f->arm_switch_seen_off = true;
        if (f->armed) {
            f->armed = false;
            f->event_disarmed = true;
        }
        return;
    }

    if (!f->armed) {
        const bool level = fabsf(f->attitude_deg[0]) < f->cfg.arm_angle_max_deg &&
                           fabsf(f->attitude_deg[1]) < f->cfg.arm_angle_max_deg;
        if (f->arm_switch_seen_off && f->calibrated && level &&
            throttle < f->cfg.arm_throttle_max) {
            f->armed = true;
            f->event_armed = true;
            for (int a = 0; a < AXIS_COUNT; a++) {
                pid_reset(&f->pid[a]);
            }
        }
    }
}

void flight_step(flight_t *f, const flight_input_t *in)
{
    const flight_config_t *c = &f->cfg;
    const float dt = 1.0f / c->loop_hz;
    f->event_armed = f->event_disarmed = f->event_failsafe = false;

    /* 1. Gyro: Vorzeichen, Kalibrierung, Tiefpass */
    float raw[AXIS_COUNT];
    for (int a = 0; a < AXIS_COUNT; a++) {
        raw[a] = in->gyro_dps[a] * c->gyro_sign[a];
    }
    if (!f->calibrated) {
        gyro_calibration(f, raw);
    }
    for (int a = 0; a < AXIS_COUNT; a++) {
        f->gyro_f[a] = pt1_apply(&f->gyro_lpf[a], raw[a] - f->gyro_bias[a]);
    }

    /* 2. Lageschaetzung (Beschleunigung nur nahe 1 g verwenden) */
    const float acc_norm = sqrtf(in->acc_g[0] * in->acc_g[0] + in->acc_g[1] * in->acc_g[1] +
                                 in->acc_g[2] * in->acc_g[2]);
    const bool use_acc = acc_norm > 0.8f && acc_norm < 1.2f;
    mahony_update(&f->ahrs, f->gyro_f[0] * DEG2RAD, f->gyro_f[1] * DEG2RAD, f->gyro_f[2] * DEG2RAD,
                  in->acc_g[0], in->acc_g[1], in->acc_g[2], use_acc || !f->calibrated, dt);
    mahony_get_euler_deg(&f->ahrs, &f->attitude_deg[0], &f->attitude_deg[1], &f->attitude_deg[2]);

    /* 3. Sticks */
    float stick[AXIS_COUNT];
    stick[AXIS_ROLL]  = crsf_to_bipolar(in->channels[CH_ROLL]);
    stick[AXIS_PITCH] = crsf_to_bipolar(in->channels[CH_PITCH]);
    stick[AXIS_YAW]   = crsf_to_bipolar(in->channels[CH_YAW]);
    for (int a = 0; a < AXIS_COUNT; a++) {
        stick[a] = apply_deadband(stick[a], c->stick_deadband) * c->stick_sign[a];
    }
    const float throttle = in->rc_valid ? crsf_to_unipolar(in->channels[CH_THROTTLE]) : 0.0f;
    f->throttle = throttle;
    f->mode = (in->rc_valid && crsf_to_unipolar(in->channels[CH_MODE]) > 0.5f) ? MODE_ANGLE : MODE_RATE;

    /* 4. Arming / Failsafe */
    update_arming(f, in, throttle);

    /* 5. Sollwerte */
    if (f->mode == MODE_ANGLE) {
        /* aeusserer Regelkreis: Winkelfehler -> Soll-Drehrate (Kaskade, Kap. 2.2) */
        for (int a = 0; a < 2; a++) {
            const float target = stick[a] * c->angle_max_deg;
            f->setpoint[a] = constrainf(c->angle_kp * (target - f->attitude_deg[a]),
                                        -c->max_rate_dps[a], c->max_rate_dps[a]);
        }
        f->setpoint[AXIS_YAW] = stick[AXIS_YAW] * c->max_rate_dps[AXIS_YAW];
    } else {
        for (int a = 0; a < AXIS_COUNT; a++) {
            f->setpoint[a] = stick[a] * c->max_rate_dps[a];
        }
    }

    /* 6. PID (innerer Regelkreis, Drehrate) */
    float pid_out[AXIS_COUNT];
    const bool on_ground = throttle < c->iterm_reset_throttle;
    for (int a = 0; a < AXIS_COUNT; a++) {
        pid_out[a] = pid_update(&f->pid[a], f->setpoint[a], f->gyro_f[a], !f->armed);
        if (!f->armed || on_ground) {
            f->pid[a].i_term = 0.0f;   /* kein Aufintegrieren am Boden */
        }
    }

    /* 7. Mischer und DShot-Werte */
    if (f->armed) {
        mixer_mix(&f->mixer, throttle,
                  pid_out[AXIS_ROLL] / PID_MIXER_SCALING,
                  pid_out[AXIS_PITCH] / PID_MIXER_SCALING,
                  pid_out[AXIS_YAW] / PID_MIXER_SCALING, f->motor);
        for (int i = 0; i < MIXER_MOTOR_COUNT; i++) {
            f->dshot[i] = dshot_from_output(f->motor[i], c->motor_idle);
        }
    } else {
        for (int i = 0; i < MIXER_MOTOR_COUNT; i++) {
            f->motor[i] = 0.0f;
            f->dshot[i] = 0;   /* DShot 0 = Motor aus */
        }
    }
}
