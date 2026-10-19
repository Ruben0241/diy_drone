#include "pid.h"
#include "util.h"

void pid_init(pid_axis_t *pid, float p_gain, float i_gain, float d_gain,
              float dt, float dterm_lpf_hz, float i_limit, float sum_limit)
{
    pid->kp = p_gain * PID_PTERM_SCALE;
    pid->ki = i_gain * PID_ITERM_SCALE;
    pid->kd = d_gain * PID_DTERM_SCALE;
    pid->dt = dt;
    pid->i_limit = i_limit;
    pid->sum_limit = sum_limit;
    pt1_init(&pid->dterm_lpf, dterm_lpf_hz, dt);
    pid_reset(pid);
}

void pid_reset(pid_axis_t *pid)
{
    pid->i_term = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->has_prev = false;
    pt1_reset(&pid->dterm_lpf, 0.0f);
    pid->p = pid->i = pid->d = pid->out = 0.0f;
}

float pid_update(pid_axis_t *pid, float setpoint, float measurement, bool freeze_i)
{
    const float error = setpoint - measurement;

    /* P-Anteil */
    pid->p = pid->kp * error;

    /* I-Anteil mit Begrenzung (Anti-Windup) */
    if (!freeze_i) {
        pid->i_term += pid->ki * error * pid->dt;
        pid->i_term = constrainf(pid->i_term, -pid->i_limit, pid->i_limit);
    }
    pid->i = pid->i_term;

    /* D-Anteil auf den Istwert (kein "D-Kick" bei Stickspruengen), gefiltert */
    float d_raw = 0.0f;
    if (pid->has_prev) {
        d_raw = -(measurement - pid->prev_measurement) / pid->dt;
    }
    pid->prev_measurement = measurement;
    pid->has_prev = true;
    pid->d = pid->kd * pt1_apply(&pid->dterm_lpf, d_raw);

    pid->out = constrainf(pid->p + pid->i + pid->d, -pid->sum_limit, pid->sum_limit);
    return pid->out;
}
