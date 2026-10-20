#include "mixer.h"
#include "util.h"

/* {roll, pitch, yaw} pro Motor, Werte aus Betaflight mixer_init.c (mixerQuadX) */
static const float mix_table[MIXER_MOTOR_COUNT][3] = {
    { -1.0f,  1.0f, -1.0f },   /* M1 hinten rechts */
    { -1.0f, -1.0f,  1.0f },   /* M2 vorne rechts  */
    {  1.0f,  1.0f,  1.0f },   /* M3 hinten links  */
    {  1.0f, -1.0f, -1.0f },   /* M4 vorne links   */
};

float mixer_mix(const mixer_config_t *cfg, float throttle, float roll, float pitch, float yaw,
               float out[MIXER_MOTOR_COUNT])
{
    if (!cfg->yaw_motors_reversed) {
        yaw = -yaw;
    }

    float mix[MIXER_MOTOR_COUNT];
    float mix_min = 0.0f, mix_max = 0.0f;
    for (int i = 0; i < MIXER_MOTOR_COUNT; i++) {
        mix[i] = mix_table[i][0] * roll + mix_table[i][1] * pitch + mix_table[i][2] * yaw;
        if (i == 0 || mix[i] < mix_min) mix_min = mix[i];
        if (i == 0 || mix[i] > mix_max) mix_max = mix[i];
    }

    /* Regleranteile passen nicht in den Motorbereich: verkleinern */
    const float range = mix_max - mix_min;
    if (range > 1.0f) {
        for (int i = 0; i < MIXER_MOTOR_COUNT; i++) {
            mix[i] /= range;
        }
        mix_min /= range;
        mix_max /= range;
    }

    /* Gas so verschieben, dass kein Motor unter 0 oder ueber 1 kommt */
    throttle = constrainf(throttle, -mix_min, 1.0f - mix_max);

    for (int i = 0; i < MIXER_MOTOR_COUNT; i++) {
        out[i] = constrainf(throttle + mix[i], 0.0f, 1.0f);
    }
    return range;
}
