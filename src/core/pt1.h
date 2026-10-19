/*
 * pt1.h - Tiefpassfilter 1. Ordnung (PT1)
 *
 *   y_k = y_{k-1} + a * (x_k - y_{k-1}),   a = dt / (RC + dt),  RC = 1 / (2*pi*fc)
 */
#pragma once

typedef struct {
    float state;
    float k;
} pt1_t;

void  pt1_init(pt1_t *f, float cutoff_hz, float dt_s);
float pt1_apply(pt1_t *f, float x);
void  pt1_reset(pt1_t *f, float value);
