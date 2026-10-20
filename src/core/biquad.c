#include "biquad.h"
#include <math.h>

#define PI_F 3.14159265358979f

static void set_coeffs(biquad_t *f, float b0, float b1, float b2, float a0, float a1, float a2)
{
    f->b0 = b0 / a0; f->b1 = b1 / a0; f->b2 = b2 / a0;
    f->a1 = a1 / a0; f->a2 = a2 / a0;
    biquad_reset(f);
}

void biquad_init_lpf(biquad_t *f, float fc_hz, float fs_hz, float q)
{
    const float w0 = 2.0f * PI_F * fc_hz / fs_hz;
    const float c = cosf(w0), alpha = sinf(w0) / (2.0f * q);
    set_coeffs(f, (1.0f - c) * 0.5f, 1.0f - c, (1.0f - c) * 0.5f, 1.0f + alpha, -2.0f * c, 1.0f - alpha);
}

void biquad_init_notch(biquad_t *f, float fc_hz, float fs_hz, float q)
{
    const float w0 = 2.0f * PI_F * fc_hz / fs_hz;
    const float c = cosf(w0), alpha = sinf(w0) / (2.0f * q);
    set_coeffs(f, 1.0f, -2.0f * c, 1.0f, 1.0f + alpha, -2.0f * c, 1.0f - alpha);
}

void biquad_reset(biquad_t *f)
{
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

float biquad_apply(biquad_t *f, float x)
{
    /* Direktform I */
    const float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
    f->x2 = f->x1; f->x1 = x;
    f->y2 = f->y1; f->y1 = y;
    return y;
}

float notch_q_from_cutoff(float center_hz, float cutoff_hz)
{
    /* Bandbreite B = f_c - f_u und f_c^2 = f_u * f_o  ->  Q = f_c * f_u / (f_c^2 - f_u^2) */
    return center_hz * cutoff_hz / (center_hz * center_hz - cutoff_hz * cutoff_hz);
}

void lpf_stage_init(lpf_stage_t *s, int type, float fc_hz, float fs_hz)
{
    /* Aus, falls ungueltig oder oberhalb der Nyquist-Frequenz */
    if (fc_hz <= 0.0f || fc_hz >= 0.5f * fs_hz) type = FILTER_OFF;
    s->type = (filter_type_t)type;
    if (type == FILTER_PT1) pt1_init(&s->pt1, fc_hz, 1.0f / fs_hz);
    else if (type == FILTER_BIQUAD) biquad_init_lpf(&s->bq, fc_hz, fs_hz, 0.70710678f);
}

float lpf_stage_apply(lpf_stage_t *s, float x)
{
    switch (s->type) {
    case FILTER_PT1:    return pt1_apply(&s->pt1, x);
    case FILTER_BIQUAD: return biquad_apply(&s->bq, x);
    default:            return x;
    }
}

void lpf_stage_reset(lpf_stage_t *s)
{
    if (s->type == FILTER_PT1) pt1_reset(&s->pt1, 0.0f);
    else if (s->type == FILTER_BIQUAD) biquad_reset(&s->bq);
}
