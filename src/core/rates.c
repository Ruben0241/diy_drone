#include "rates.h"
#include <math.h>

float rates_apply(float x, float center, float max_rate, float expo)
{
    const float ax = fabsf(x);
    const float x5 = x * x * x * x * x;
    const float e = ax * (x5 * expo + x * (1.0f - expo));
    const float movement = (max_rate > center) ? (max_rate - center) : 0.0f;
    return x * center + movement * e;
}
