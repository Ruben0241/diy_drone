#include "pt1.h"

void pt1_init(pt1_t *f, float cutoff_hz, float dt_s)
{
    const float rc = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
    f->k = dt_s / (rc + dt_s);
    f->state = 0.0f;
}

float pt1_apply(pt1_t *f, float x)
{
    f->state += f->k * (x - f->state);
    return f->state;
}

void pt1_reset(pt1_t *f, float value)
{
    f->state = value;
}
