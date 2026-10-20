#include "flight.h"
#include "dshot.h"
#include "rates.h"
#include <math.h>
#include <string.h>

void flight_default_config(flight_config_t *c)
{
    memset(c, 0, sizeof(*c));
    c->loop_hz = 1000.0f;

    /* Betaflight-Standardwerte (src/main/flight/pid.h, PID_*_DEFAULT) - ohne Feedforward,
     * damit im Vergleich nur der PID-Regler wirkt (Kap. Methodik). Betaflight: F = 120/125/120 */
    c->pid_p[AXIS_ROLL] = 45; c->pid_i[AXIS_ROLL] = 80; c->pid_d[AXIS_ROLL] = 30;
    c->pid_p[AXIS_PITCH] = 47; c->pid_i[AXIS_PITCH] = 84; c->pid_d[AXIS_PITCH] = 34;
    c->pid_p[AXIS_YAW] = 45; c->pid_i[AXIS_YAW] = 80; c->pid_d[AXIS_YAW] = 0;
    c->i_limit = 400.0f;
    c->sum_limit[AXIS_ROLL] = 500.0f;
    c->sum_limit[AXIS_PITCH] = 500.0f;
    c->sum_limit[AXIS_YAW] = 400.0f;
    c->iterm_relax = 0.0f;          /* Betaflight-Standard: RP; fuer den Vergleich aus */
    c->iterm_relax_hz = 15.0f;
    c->iterm_windup = 80.0f;        /* Betaflight 2025.12/master: I-Grenze 80 % von 500 bzw. 400 */
    c->iterm_mix_windup = 0.85f;    /* zusaetzlich: I bremsen, wenn der Mischer saettigt (wie PX4) */
    c->yaw_lowpass_hz = 100.0f;
    c->tpa_rate = 0.0f;             /* Betaflight-Standard: 0.65 ab 0.35; fuer den Vergleich aus */
    c->tpa_breakpoint = 0.35f;

    /* Filter: Regelung mit 1 kHz, Nyquist-Frequenz 500 Hz.
     * D-Term wie Betaflight (PT1 75 Hz + PT1 150 Hz). */
    c->gyro_lpf1_type = FILTER_PT1;  c->gyro_lpf1_hz = 150.0f;
    c->gyro_lpf2_type = FILTER_OFF;  c->gyro_lpf2_hz = 0.0f;
    c->gyro_notch_hz = 0.0f;         c->gyro_notch_cutoff_hz = 0.0f;
    c->dterm_lpf1_type = FILTER_PT1; c->dterm_lpf1_hz = 75.0f;
    c->dterm_lpf2_type = FILTER_PT1; c->dterm_lpf2_hz = 150.0f;
    c->acc_lpf_hz = 25.0f;           /* Betaflight acc_lpf_hz = 25 */
    c->rc_smoothing_hz = 40.0f;
    c->ff_smoothing_hz = 30.0f;

    /* Linear 400 dps (Kap. 3.4.1); in Betaflight gleich einstellen */
    for (int a = 0; a < AXIS_COUNT; a++) {
        c->rate_center[a] = 400.0f;
        c->rate_max[a] = 400.0f;
        c->rate_expo[a] = 0.0f;
    }
    c->stick_deadband = 0.01f;

    c->angle_max_deg = 60.0f;
    c->angle_kp = 5.0f;

    c->mahony_kp = 0.25f;           /* Betaflight imu_dcm_kp = 2500 -> 0.25 */
    c->mahony_ki = 0.0f;

    c->motor_idle = 0.055f;
    c->arm_throttle_max = 0.05f;
    c->arm_angle_max_deg = 25.0f;
    c->low_throttle = 0.05f;
    c->airmode_start = 0.25f;       /* Betaflight airmode_start_throttle_percent = 25 */
    c->runaway_prevention = 1.0f;

    for (int a = 0; a < AXIS_COUNT; a++) {
        c->gyro_sign[a] = 1.0f;
        c->stick_sign[a] = 1.0f;
    }
    /* Koerpersystem z nach oben: positive Gierrate = gegen den Uhrzeigersinn (Nase links).
     * Yaw-Stick rechts soll nach rechts drehen -> Vorzeichen umkehren (wie Betaflight rc.c) */
    c->stick_sign[AXIS_YAW] = -1.0f;
    c->yaw_motors_reversed = 0.0f;

    c->gyro_cal_samples = 1000.0f;  /* 1 s bei 1 kHz */
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

/* Filter, Regler und Mischer aus der Konfiguration aufbauen */
static void setup_from_config(flight_t *f)
{
    const flight_config_t *c = &f->cfg;
    const float fs = c->loop_hz, dt = 1.0f / c->loop_hz;

    for (int a = 0; a < AXIS_COUNT; a++) {
        const pid_params_t prm = {
            .p_gain = c->pid_p[a], .i_gain = c->pid_i[a], .d_gain = c->pid_d[a], .f_gain = c->pid_f[a],
            .i_limit = fminf(c->i_limit, 0.01f * c->iterm_windup * c->sum_limit[a]),
            .sum_limit = c->sum_limit[a], .dt = dt,
            .p_lpf_hz = (a == AXIS_YAW) ? c->yaw_lowpass_hz : 0.0f,
            .dterm_lpf1_type = (int)c->dterm_lpf1_type, .dterm_lpf1_hz = c->dterm_lpf1_hz,
            .dterm_lpf2_type = (int)c->dterm_lpf2_type, .dterm_lpf2_hz = c->dterm_lpf2_hz,
            .ff_lpf_hz = c->ff_smoothing_hz,
            .iterm_relax = (a < AXIS_YAW) ? (c->iterm_relax >= 1.0f) : (c->iterm_relax >= 2.0f),
            .iterm_relax_hz = c->iterm_relax_hz,
        };
        pid_init(&f->pid[a], &prm);
        lpf_stage_init(&f->gyro_lpf1[a], (int)c->gyro_lpf1_type, c->gyro_lpf1_hz, fs);
        lpf_stage_init(&f->gyro_lpf2[a], (int)c->gyro_lpf2_type, c->gyro_lpf2_hz, fs);
        pt1_init(&f->rc_lpf[a], c->rc_smoothing_hz > 0.0f ? c->rc_smoothing_hz : 0.45f * fs, dt);
    }
    f->notch_on = c->gyro_notch_hz > 0.0f && c->gyro_notch_cutoff_hz > 0.0f &&
                  c->gyro_notch_cutoff_hz < c->gyro_notch_hz && c->gyro_notch_hz < 0.5f * fs;
    if (f->notch_on) {
        const float q = notch_q_from_cutoff(c->gyro_notch_hz, c->gyro_notch_cutoff_hz);
        for (int a = 0; a < AXIS_COUNT; a++) biquad_init_notch(&f->gyro_notch[a], c->gyro_notch_hz, fs, q);
    }
    for (int i = 0; i < 3; i++) {
        pt1_init(&f->acc_lpf[i], c->acc_lpf_hz > 0.0f ? c->acc_lpf_hz : 0.45f * fs, dt);
        pt1_reset(&f->acc_lpf[i], i == 2 ? 1.0f : 0.0f);
    }
    f->ahrs.kp = c->mahony_kp;
    f->ahrs.ki = c->mahony_ki;
    f->mixer.yaw_motors_reversed = c->yaw_motors_reversed >= 0.5f;
}

void flight_init(flight_t *f, const flight_config_t *cfg)
{
    memset(f, 0, sizeof(*f));
    f->cfg = *cfg;
    mahony_init(&f->ahrs, cfg->mahony_kp, cfg->mahony_ki);
    setup_from_config(f);
    cal_restart(f);
}

bool flight_apply_config(flight_t *f, const flight_config_t *cfg)
{
    if (f->armed) return false;
    f->cfg = *cfg;
    setup_from_config(f);
    return true;
}

void flight_start_acc_calibration(flight_t *f)
{
    f->acc_cal_running = true;
    f->acc_cal_done = false;
    f->acc_cal_count = 0;
    f->acc_cal_sum[0] = f->acc_cal_sum[1] = f->acc_cal_sum[2] = 0.0f;
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
    if ((float)++f->cal_count >= f->cfg.gyro_cal_samples) {
        for (int a = 0; a < AXIS_COUNT; a++) {
            f->gyro_bias[a] = f->cal_sum[a] / (float)f->cal_count;
        }
        f->calibrated = true;
    }
}

static void acc_calibration(flight_t *f, const float acc_raw[3])
{
    /* nur bei Stillstand: sonst neu beginnen */
    for (int a = 0; a < AXIS_COUNT; a++) {
        if (fabsf(f->gyro_f[a]) > 5.0f) { flight_start_acc_calibration(f); return; }
    }
    for (int i = 0; i < 3; i++) f->acc_cal_sum[i] += acc_raw[i];
    if (++f->acc_cal_count >= 500) {
        const float n = (float)f->acc_cal_count;
        /* waagrecht: erwartet (0, 0, 1 g) */
        f->cfg.acc_bias[0] = f->acc_cal_sum[0] / n;
        f->cfg.acc_bias[1] = f->acc_cal_sum[1] / n;
        f->cfg.acc_bias[2] = f->acc_cal_sum[2] / n - 1.0f;
        f->acc_cal_running = false;
        f->acc_cal_done = true;
        mahony_init(&f->ahrs, f->cfg.mahony_kp, f->cfg.mahony_ki);
    }
}

static float apply_deadband(float x, float db)
{
    if (fabsf(x) <= db) return 0.0f;
    return (x > 0.0f) ? (x - db) / (1.0f - db) : (x + db) / (1.0f - db);
}

static void disarm(flight_t *f)
{
    if (f->armed) f->event_disarmed = true;
    f->armed = false;
    f->airmode_active = false;
}

static void update_arming(flight_t *f, const flight_input_t *in, float throttle)
{
    const bool arm_switch = in->rc_valid && crsf_to_unipolar(in->channels[CH_ARM]) > 0.5f;

    /* Failsafe: keine gueltigen Pakete -> sofort Motoren aus */
    if (!in->rc_valid) {
        if (!f->failsafe) f->event_failsafe = true;
        f->failsafe = true;
        f->arm_switch_seen_off = false;   /* nach Verbindungsverlust Schalter neu betaetigen */
        disarm(f);
        return;
    }
    f->failsafe = false;

    if (!arm_switch) {
        f->arm_switch_seen_off = true;
        f->runaway_triggered = false;
        disarm(f);
        return;
    }

    if (!f->armed) {
        const bool level = fabsf(f->attitude_deg[0]) < f->cfg.arm_angle_max_deg &&
                           fabsf(f->attitude_deg[1]) < f->cfg.arm_angle_max_deg;
        if (f->arm_switch_seen_off && f->calibrated && level && !in->arm_inhibit &&
            !f->acc_cal_running && throttle < f->cfg.arm_throttle_max) {
            f->armed = true;
            f->event_armed = true;
            f->airmode_active = false;
            f->runaway_armed = f->cfg.runaway_prevention >= 0.5f;
            f->runaway_time_s = 0.0f;
            f->takeoff_time_s = 0.0f;
            for (int a = 0; a < AXIS_COUNT; a++) pid_reset(&f->pid[a]);
        }
    }
}

void flight_force_disarm(flight_t *f)
{
    disarm(f);
    f->arm_switch_seen_off = false;
    for (int i = 0; i < MIXER_MOTOR_COUNT; i++) {
        f->motor[i] = 0.0f;
        f->dshot[i] = 0;
    }
}

/* Runaway-Schutz wie Betaflight (core.c): Vor dem Abheben bedeutet eine dauerhaft grosse
 * PID-Summe bei gleichzeitiger Drehung fast immer einen Fehler (z. B. falsches Vorzeichen,
 * falsche Motorreihenfolge). Dann sofort disarmen. Nach 0.5 s mit Gas > 25 % ist der Schutz aus. */
static void runaway_check(flight_t *f, float dt)
{
    if (!f->runaway_armed) return;
    if (f->throttle > 0.25f) {
        f->takeoff_time_s += dt;
        if (f->takeoff_time_s > 0.5f) { f->runaway_armed = false; return; }
    }
    bool big_sum = false, rotating = false;
    for (int a = 0; a < AXIS_COUNT; a++) {
        if (fabsf(f->pid[a].sum_raw) >= 600.0f) big_sum = true;
        const float lim = (a == AXIS_YAW) ? 50.0f : 15.0f;
        if (fabsf(f->gyro_f[a]) > lim) rotating = true;
    }
    if (big_sum && rotating) {
        f->runaway_time_s += dt;
        if (f->runaway_time_s >= 0.075f) {
            f->runaway_triggered = true;
            f->event_runaway = true;
            flight_force_disarm(f);
        }
    } else {
        f->runaway_time_s = 0.0f;
    }
}

void flight_step(flight_t *f, const flight_input_t *in)
{
    const flight_config_t *c = &f->cfg;
    const float dt = 1.0f / c->loop_hz;
    f->event_armed = f->event_disarmed = f->event_failsafe = f->event_runaway = false;

    /* 1. Gyro: Vorzeichen, Kalibrierung, Filter */
    float raw[AXIS_COUNT];
    for (int a = 0; a < AXIS_COUNT; a++) {
        raw[a] = in->gyro_dps[a] * c->gyro_sign[a];
    }
    if (!f->calibrated) {
        gyro_calibration(f, raw);
    }
    for (int a = 0; a < AXIS_COUNT; a++) {
        float g = raw[a] - f->gyro_bias[a];
        if (f->notch_on) g = biquad_apply(&f->gyro_notch[a], g);
        g = lpf_stage_apply(&f->gyro_lpf1[a], g);
        f->gyro_f[a] = lpf_stage_apply(&f->gyro_lpf2[a], g);
    }

    /* 2. Beschleunigung und Lageschaetzung (Beschleunigung nur nahe 1 g verwenden) */
    for (int i = 0; i < 3; i++) {
        f->acc_f[i] = pt1_apply(&f->acc_lpf[i], in->acc_g[i] - c->acc_bias[i]);
    }
    if (f->acc_cal_running && !f->armed) {
        acc_calibration(f, in->acc_g);
    }
    const float acc_norm = sqrtf(f->acc_f[0] * f->acc_f[0] + f->acc_f[1] * f->acc_f[1] +
                                 f->acc_f[2] * f->acc_f[2]);
    const bool use_acc = acc_norm > 0.8f && acc_norm < 1.2f;
    mahony_update(&f->ahrs, f->gyro_f[0] * DEG2RAD, f->gyro_f[1] * DEG2RAD, f->gyro_f[2] * DEG2RAD,
                  f->acc_f[0], f->acc_f[1], f->acc_f[2], use_acc || !f->calibrated, dt);
    mahony_get_euler_deg(&f->ahrs, &f->attitude_deg[0], &f->attitude_deg[1], &f->attitude_deg[2]);

    /* 3. Sticks: Totband, Vorzeichen, RC-Glaettung */
    float stick[AXIS_COUNT];
    stick[AXIS_ROLL]  = crsf_to_bipolar(in->channels[CH_ROLL]);
    stick[AXIS_PITCH] = crsf_to_bipolar(in->channels[CH_PITCH]);
    stick[AXIS_YAW]   = crsf_to_bipolar(in->channels[CH_YAW]);
    for (int a = 0; a < AXIS_COUNT; a++) {
        const float s = in->rc_valid ? apply_deadband(stick[a], c->stick_deadband) * c->stick_sign[a] : 0.0f;
        stick[a] = pt1_apply(&f->rc_lpf[a], s);
    }
    const float throttle = in->rc_valid ? crsf_to_unipolar(in->channels[CH_THROTTLE]) : 0.0f;
    f->throttle = throttle;
    f->mode = (in->rc_valid && crsf_to_unipolar(in->channels[CH_MODE]) > 0.5f) ? MODE_ANGLE : MODE_RATE;

    /* 4. Arming / Failsafe */
    update_arming(f, in, throttle);
    if (f->armed && throttle >= c->airmode_start) f->airmode_active = true;

    /* 5. Sollwerte */
    if (f->mode == MODE_ANGLE) {
        /* aeusserer Regelkreis: Winkelfehler -> Soll-Drehrate (Kaskade) */
        for (int a = 0; a < 2; a++) {
            const float target = stick[a] * c->angle_max_deg;
            f->setpoint[a] = constrainf(c->angle_kp * (target - f->attitude_deg[a]),
                                        -c->rate_max[a], c->rate_max[a]);
        }
        f->setpoint[AXIS_YAW] = rates_apply(stick[AXIS_YAW], c->rate_center[AXIS_YAW],
                                            c->rate_max[AXIS_YAW], c->rate_expo[AXIS_YAW]);
    } else {
        for (int a = 0; a < AXIS_COUNT; a++) {
            f->setpoint[a] = rates_apply(stick[a], c->rate_center[a], c->rate_max[a], c->rate_expo[a]);
        }
    }

    /* 6. PID (innerer Regelkreis, Drehrate) */
    pid_runtime_t rt;
    rt.stabilise = f->armed && (f->airmode_active || throttle >= c->low_throttle);
    /* Anti-Windup: I nicht weiter aufbauen, wenn die Motoren schon am Anschlag sind */
    rt.i_factor = 1.0f;
    if (c->iterm_mix_windup < 1.0f) {
        rt.i_factor = constrainf((1.0f - f->mix_range) / (1.0f - c->iterm_mix_windup), 0.0f, 1.0f);
    }
    /* TPA: D bei viel Gas abschwaechen (Betaflight tpa_mode = D) */
    rt.d_factor = 1.0f;
    if (c->tpa_rate > 0.0f && throttle > c->tpa_breakpoint && c->tpa_breakpoint < 1.0f) {
        rt.d_factor = 1.0f - c->tpa_rate * (throttle - c->tpa_breakpoint) / (1.0f - c->tpa_breakpoint);
    }
    float pid_out[AXIS_COUNT];
    for (int a = 0; a < AXIS_COUNT; a++) {
        pid_out[a] = pid_update(&f->pid[a], f->setpoint[a], f->gyro_f[a], &rt);
    }
    if (f->armed) runaway_check(f, dt);

    /* 7. Mischer und DShot-Werte */
    if (f->armed) {
        f->mix_range = mixer_mix(&f->mixer, throttle,
                                 pid_out[AXIS_ROLL] / PID_MIXER_SCALING,
                                 pid_out[AXIS_PITCH] / PID_MIXER_SCALING,
                                 pid_out[AXIS_YAW] / PID_MIXER_SCALING, f->motor);
        for (int i = 0; i < MIXER_MOTOR_COUNT; i++) {
            f->dshot[i] = dshot_from_output(f->motor[i], c->motor_idle);
        }
    } else {
        f->mix_range = 0.0f;
        for (int i = 0; i < MIXER_MOTOR_COUNT; i++) {
            const bool test = in->motor_test && !in->arm_inhibit;
            const float v = test ? constrainf(in->motor_test_value[i], 0.0f, 1.0f) : 0.0f;
            f->motor[i] = v;
            f->dshot[i] = (v > 0.0f) ? dshot_from_output(v, 0.0f) : 0;   /* DShot 0 = Motor aus */
        }
    }
}
