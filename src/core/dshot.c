#include "dshot.h"
#include "util.h"

uint16_t dshot_make_frame(uint16_t value, bool telemetry)
{
    if (value > DSHOT_MAX_THROTTLE) {
        value = DSHOT_MAX_THROTTLE;
    }
    const uint16_t v = (uint16_t)((value << 1) | (telemetry ? 1u : 0u));
    const uint16_t crc = (uint16_t)((v ^ (v >> 4) ^ (v >> 8)) & 0x0F);
    return (uint16_t)((v << 4) | crc);
}

uint16_t dshot_from_output(float out, float idle)
{
    out = constrainf(out, 0.0f, 1.0f);
    const float scaled = idle + (1.0f - idle) * out;
    const float range = (float)(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE);
    return (uint16_t)(DSHOT_MIN_THROTTLE + (uint16_t)(scaled * range + 0.5f));
}

void dshot_fill_buffer(uint32_t *buf, uint32_t stride, uint16_t frame,
                       uint32_t ticks_zero, uint32_t ticks_one)
{
    for (uint32_t i = 0; i < DSHOT_BITS; i++) {
        const bool bit = (frame & (0x8000u >> i)) != 0;
        buf[i * stride] = bit ? ticks_one : ticks_zero;
    }
    buf[DSHOT_BITS * stride] = 0;  /* Leitung danach low */
}
