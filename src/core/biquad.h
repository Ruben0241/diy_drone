/*
 * biquad.h - Filter 2. Ordnung (Tiefpass, Kerbfilter) und waehlbare Filterstufe
 *
 * Koeffizienten nach dem "Audio EQ Cookbook" (R. Bristow-Johnson, W3C Note 2021):
 *   w0 = 2*pi*fc/fs, alpha = sin(w0)/(2Q)
 *   Tiefpass: b0 = (1-cos w0)/2, b1 = 1-cos w0, b2 = b0
 *   Notch:    b0 = 1, b1 = -2 cos w0, b2 = 1
 *   a0 = 1+alpha, a1 = -2 cos w0, a2 = 1-alpha   (alles durch a0 geteilt)
 *
 * Eine Filterstufe (lpf_stage_t) ist wahlweise aus, PT1 oder Biquad (Butterworth, Q = 1/sqrt 2).
 */
#pragma once

#include "pt1.h"

typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquad_t;

void  biquad_init_lpf(biquad_t *f, float fc_hz, float fs_hz, float q);
void  biquad_init_notch(biquad_t *f, float fc_hz, float fs_hz, float q);
float biquad_apply(biquad_t *f, float x);
void  biquad_reset(biquad_t *f);
/* Guete eines Kerbfilters aus Mitten- und unterer Grenzfrequenz (-3 dB) */
float notch_q_from_cutoff(float center_hz, float cutoff_hz);

typedef enum { FILTER_OFF = 0, FILTER_PT1 = 1, FILTER_BIQUAD = 2 } filter_type_t;

typedef struct {
    filter_type_t type;
    pt1_t pt1;
    biquad_t bq;
} lpf_stage_t;

void  lpf_stage_init(lpf_stage_t *s, int type, float fc_hz, float fs_hz);
float lpf_stage_apply(lpf_stage_t *s, float x);
void  lpf_stage_reset(lpf_stage_t *s);
