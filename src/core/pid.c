#include "pid.h"
#include "util.h"
#include <math.h>

void pid_init(pid_axis_t *pid, const pid_params_t *prm)
{
    const float fs = 1.0f / prm->dt;
    pid->kp = prm->p_gain * PID_PTERM_SCALE;
    pid->ki = prm->i_gain * PID_ITERM_SCALE;
    pid->kd = prm->d_gain * PID_DTERM_SCALE;
    pid->kf = prm->f_gain * PID_FEEDFORWARD_SCALE;
    pid->i_limit = prm->i_limit;
    pid->sum_limit = prm->sum_limit;
    pid->dt = prm->dt;
    pid->iterm_relax = prm->iterm_relax;
    lpf_stage_init(&pid->dterm1, prm->dterm_lpf1_type, prm->dterm_lpf1_hz, fs);
    lpf_stage_init(&pid->dterm2, prm->dterm_lpf2_type, prm->dterm_lpf2_hz, fs);
    pt1_init(&pid->ff_lpf, prm->ff_lpf_hz > 0.0f ? prm->ff_lpf_hz : 0.45f * fs, prm->dt);
    pt1_init(&pid->relax_lpf, prm->iterm_relax_hz > 0.0f ? prm->iterm_relax_hz : 15.0f, prm->dt);
    pid_reset(pid);
}

void pid_reset(pid_axis_t *pid)
{
    pid->i_term = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_setpoint = 0.0f;
    pid->has_prev = false;
    pid->p = pid->i = pid->d = pid->f = pid->sum_raw = pid->out = 0.0f;
    pt1_reset(&pid->ff_lpf, 0.0f);
}

float pid_update(pid_axis_t *pid, float setpoint, float measurement, const pid_runtime_t *rt)
{
    const float error = setpoint - measurement;

    /* Ableitungen (erster Aufruf: 0) */
    float d_meas = 0.0f, d_sp = 0.0f;
    if (pid->has_prev) {
        d_meas = (measurement - pid->prev_measurement) / pid->dt;
        d_sp = (setpoint - pid->prev_setpoint) / pid->dt;
    }
    pid->prev_measurement = measurement;
    pid->prev_setpoint = setpoint;
    pid->has_prev = true;

    /* Filter laufen immer mit, damit beim Einschalten kein Sprung entsteht */
    const float d_filt = lpf_stage_apply(&pid->dterm2, lpf_stage_apply(&pid->dterm1, -d_meas));
    const float ff_filt = pt1_apply(&pid->ff_lpf, d_sp);
    const float sp_lp = pt1_apply(&pid->relax_lpf, setpoint);

    if (!rt->stabilise) {
        pid->i_term = 0.0f;
        pid->p = pid->i = pid->d = pid->f = pid->sum_raw = pid->out = 0.0f;
        return 0.0f;
    }

    /* P */
    pid->p = pid->kp * error;

    /* I mit I-Term-Relax und Anti-Windup */
    float relax = 1.0f;
    if (pid->iterm_relax) {
        const float hp = fabsf(setpoint - sp_lp);
        relax = constrainf(1.0f - hp / ITERM_RELAX_THRESHOLD, 0.0f, 1.0f);
    }
    pid->i_term += pid->ki * error * pid->dt * relax * constrainf(rt->i_factor, 0.0f, 1.0f);
    pid->i_term = constrainf(pid->i_term, -pid->i_limit, pid->i_limit);
    pid->i = pid->i_term;

    /* D auf den Istwert (kein "D-Kick" bei Stickspruengen), mit TPA */
    pid->d = pid->kd * d_filt * rt->d_factor;

    /* Feedforward: reagiert auf die Stickbewegung, nicht auf den Fehler */
    pid->f = pid->kf * ff_filt;

    pid->sum_raw = pid->p + pid->i + pid->d + pid->f;
    pid->out = constrainf(pid->sum_raw, -pid->sum_limit, pid->sum_limit);
    return pid->out;
}
