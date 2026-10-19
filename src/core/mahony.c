#include "mahony.h"
#include "util.h"
#include <math.h>

void mahony_init(mahony_t *m, float kp, float ki)
{
    m->q0 = 1.0f; m->q1 = m->q2 = m->q3 = 0.0f;
    m->ix = m->iy = m->iz = 0.0f;
    m->kp = kp;
    m->ki = ki;
}

void mahony_update(mahony_t *m, float gx, float gy, float gz,
                   float ax, float ay, float az, bool use_acc, float dt)
{
    float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;

    const float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (use_acc && norm > 1e-6f) {
        ax /= norm; ay /= norm; az /= norm;

        /* geschaetzte Richtung der Schwerkraft im Koerpersystem (3. Zeile von R) */
        const float vx = 2.0f * (q1 * q3 - q0 * q2);
        const float vy = 2.0f * (q0 * q1 + q2 * q3);
        const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* Fehler e = a x v_hat */
        const float ex = ay * vz - az * vy;
        const float ey = az * vx - ax * vz;
        const float ez = ax * vy - ay * vx;

        if (m->ki > 0.0f) {
            m->ix += m->ki * ex * dt;
            m->iy += m->ki * ey * dt;
            m->iz += m->ki * ez * dt;
            gx += m->ix; gy += m->iy; gz += m->iz;
        }
        gx += m->kp * ex;
        gy += m->kp * ey;
        gz += m->kp * ez;
    }

    /* Quaternion integrieren: q_dot = 0.5 * q (x) [0, w] */
    const float hdt = 0.5f * dt;
    const float nq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * hdt;
    const float nq1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * hdt;
    const float nq2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * hdt;
    const float nq3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * hdt;

    const float qn = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
    m->q0 = nq0 / qn; m->q1 = nq1 / qn; m->q2 = nq2 / qn; m->q3 = nq3 / qn;
}

void mahony_get_euler_deg(const mahony_t *m, float *roll, float *pitch, float *yaw)
{
    const float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;
    *roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
    *pitch = asinf(constrainf(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * RAD2DEG;
    *yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;
}
