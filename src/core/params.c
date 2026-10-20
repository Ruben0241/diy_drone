#include "params.h"
#include <string.h>

#define F(field) ((uint16_t)offsetof(flight_config_t, field))

/* Namen wie in Betaflight, wo es ein Gegenstueck gibt */
static const param_def_t defs[] = {
    { "p_roll",  F(pid_p[0]), 0, 250, 0, true },
    { "i_roll",  F(pid_i[0]), 0, 250, 0, true },
    { "d_roll",  F(pid_d[0]), 0, 250, 0, true },
    { "f_roll",  F(pid_f[0]), 0, 2000, 0, true },
    { "p_pitch", F(pid_p[1]), 0, 250, 0, true },
    { "i_pitch", F(pid_i[1]), 0, 250, 0, true },
    { "d_pitch", F(pid_d[1]), 0, 250, 0, true },
    { "f_pitch", F(pid_f[1]), 0, 2000, 0, true },
    { "p_yaw",   F(pid_p[2]), 0, 250, 0, true },
    { "i_yaw",   F(pid_i[2]), 0, 250, 0, true },
    { "d_yaw",   F(pid_d[2]), 0, 250, 0, true },
    { "f_yaw",   F(pid_f[2]), 0, 2000, 0, true },
    { "iterm_limit",     F(i_limit), 0, 1000, 0, true },
    { "pidsum_limit",    F(sum_limit[0]), 100, 1000, 0, true },
    { "pidsum_limit_pitch", F(sum_limit[1]), 100, 1000, 0, true },
    { "pidsum_limit_yaw",   F(sum_limit[2]), 100, 1000, 0, true },
    { "iterm_relax",     F(iterm_relax), 0, 2, 0, true },
    { "iterm_relax_cutoff", F(iterm_relax_hz), 1, 50, 0, true },
    { "iterm_windup",    F(iterm_windup), 20, 100, 0, true },
    { "iterm_mix_windup", F(iterm_mix_windup), 0.3f, 1, 2, false },
    { "yaw_lowpass_hz",  F(yaw_lowpass_hz), 0, 500, 0, true },
    { "tpa_rate",        F(tpa_rate), 0, 1, 2, false },
    { "tpa_breakpoint",  F(tpa_breakpoint), 0, 1, 2, false },

    { "gyro_lpf1_type",  F(gyro_lpf1_type), 0, 2, 0, true },
    { "gyro_lpf1_hz",    F(gyro_lpf1_hz), 0, 500, 0, true },
    { "gyro_lpf2_type",  F(gyro_lpf2_type), 0, 2, 0, true },
    { "gyro_lpf2_hz",    F(gyro_lpf2_hz), 0, 500, 0, true },
    { "gyro_notch_hz",   F(gyro_notch_hz), 0, 500, 0, true },
    { "gyro_notch_cutoff", F(gyro_notch_cutoff_hz), 0, 500, 0, true },
    { "dterm_lpf1_type", F(dterm_lpf1_type), 0, 2, 0, true },
    { "dterm_lpf1_hz",   F(dterm_lpf1_hz), 0, 500, 0, true },
    { "dterm_lpf2_type", F(dterm_lpf2_type), 0, 2, 0, true },
    { "dterm_lpf2_hz",   F(dterm_lpf2_hz), 0, 500, 0, true },
    { "acc_lpf_hz",      F(acc_lpf_hz), 0, 200, 0, true },
    { "rc_smoothing_hz", F(rc_smoothing_hz), 0, 200, 0, true },
    { "ff_smoothing_hz", F(ff_smoothing_hz), 0, 200, 0, true },

    { "roll_center",  F(rate_center[0]), 0, 2000, 0, false },
    { "roll_max",     F(rate_max[0]), 0, 2000, 0, false },
    { "roll_expo",    F(rate_expo[0]), 0, 1, 2, false },
    { "pitch_center", F(rate_center[1]), 0, 2000, 0, false },
    { "pitch_max",    F(rate_max[1]), 0, 2000, 0, false },
    { "pitch_expo",   F(rate_expo[1]), 0, 1, 2, false },
    { "yaw_center",   F(rate_center[2]), 0, 2000, 0, false },
    { "yaw_max",      F(rate_max[2]), 0, 2000, 0, false },
    { "yaw_expo",     F(rate_expo[2]), 0, 1, 2, false },
    { "deadband",     F(stick_deadband), 0, 0.2f, 3, false },

    { "angle_limit",  F(angle_max_deg), 10, 90, 0, false },
    { "angle_p",      F(angle_kp), 0, 20, 2, false },
    { "imu_kp",       F(mahony_kp), 0, 10, 3, true },
    { "imu_ki",       F(mahony_ki), 0, 1, 3, true },

    { "motor_idle",   F(motor_idle), 0, 0.2f, 3, false },
    { "arm_throttle_max", F(arm_throttle_max), 0, 0.3f, 2, false },
    { "arm_angle_max",    F(arm_angle_max_deg), 5, 180, 0, false },
    { "low_throttle",     F(low_throttle), 0, 0.3f, 2, false },
    { "airmode_start",    F(airmode_start), 0, 1, 2, false },
    { "runaway_prevention", F(runaway_prevention), 0, 1, 0, false },

    { "gyro_sign_roll",  F(gyro_sign[0]), -1, 1, 0, false },
    { "gyro_sign_pitch", F(gyro_sign[1]), -1, 1, 0, false },
    { "gyro_sign_yaw",   F(gyro_sign[2]), -1, 1, 0, false },
    { "stick_sign_roll",  F(stick_sign[0]), -1, 1, 0, false },
    { "stick_sign_pitch", F(stick_sign[1]), -1, 1, 0, false },
    { "stick_sign_yaw",   F(stick_sign[2]), -1, 1, 0, false },
    { "yaw_motors_reversed", F(yaw_motors_reversed), 0, 1, 0, true },
    { "acc_bias_x", F(acc_bias[0]), -0.5f, 0.5f, 4, false },
    { "acc_bias_y", F(acc_bias[1]), -0.5f, 0.5f, 4, false },
    { "acc_bias_z", F(acc_bias[2]), -0.5f, 0.5f, 4, false },
};

#define N_DEFS ((int)(sizeof(defs) / sizeof(defs[0])))

int params_count(void) { return N_DEFS; }
const param_def_t *params_def(int i) { return (i >= 0 && i < N_DEFS) ? &defs[i] : NULL; }

int params_find(const char *name)
{
    for (int i = 0; i < N_DEFS; i++) {
        if (strcmp(defs[i].name, name) == 0) return i;
    }
    return -1;
}

static float *field(flight_config_t *cfg, int i) { return (float *)((uint8_t *)cfg + defs[i].offset); }
float params_get(const flight_config_t *cfg, int i)
{
    return *(const float *)((const uint8_t *)cfg + defs[i].offset);
}

bool params_parse_float(const char *s, float *out)
{
    while (*s == ' ' || *s == '\t') s++;
    bool neg = false;
    if (*s == '-' || *s == '+') { neg = (*s == '-'); s++; }
    if (!((*s >= '0' && *s <= '9') || *s == '.')) return false;
    double v = 0.0, scale = 1.0;
    bool digits = false, frac = false;
    for (; *s; s++) {
        if (*s >= '0' && *s <= '9') {
            digits = true;
            if (frac) { scale *= 0.1; v += (*s - '0') * scale; }
            else v = v * 10.0 + (*s - '0');
        } else if (*s == '.' && !frac) {
            frac = true;
        } else if (*s == ' ' || *s == '\r' || *s == '\n' || *s == '\t') {
            break;
        } else {
            return false;
        }
    }
    if (!digits) return false;
    while (*s == ' ' || *s == '\r' || *s == '\n' || *s == '\t') s++;
    if (*s) return false;
    *out = (float)(neg ? -v : v);
    return true;
}

bool params_set(flight_config_t *cfg, const char *name, const char *value)
{
    const int i = params_find(name);
    float v;
    if (i < 0 || !params_parse_float(value, &v)) return false;
    if (v < defs[i].min || v > defs[i].max) return false;
    if (strncmp(name, "gyro_sign", 9) == 0 || strncmp(name, "stick_sign", 10) == 0) {
        if (v != 1.0f && v != -1.0f) return false;
    }
    *field(cfg, i) = v;
    return true;
}

/* Kleine Hilfen ohne printf (newlib-nano-printf kann kein float) */
static int put_str(char *b, int pos, int cap, const char *s)
{
    while (*s && pos < cap - 1) b[pos++] = *s++;
    b[pos < cap ? pos : cap - 1] = 0;
    return pos;
}

static int put_uint(char *b, int pos, int cap, uint32_t v)
{
    char t[11]; int n = 0;
    do { t[n++] = (char)('0' + v % 10u); v /= 10u; } while (v && n < 10);
    while (n-- && pos < cap - 1) b[pos++] = t[n];
    b[pos < cap ? pos : cap - 1] = 0;
    return pos;
}

static int put_fixed(char *b, int pos, int cap, float v, int dec)
{
    uint32_t scale = 1;
    for (int k = 0; k < dec; k++) scale *= 10u;
    if (v < 0.0f) { pos = put_str(b, pos, cap, "-"); v = -v; }
    const uint32_t x = (uint32_t)(v * (float)scale + 0.5f);
    pos = put_uint(b, pos, cap, x / scale);
    if (dec > 0) {
        pos = put_str(b, pos, cap, ".");
        uint32_t frac = x % scale;
        for (uint32_t d = scale / 10u; d > 0; d /= 10u) {
            const char c[2] = { (char)('0' + (frac / d) % 10u), 0 };
            pos = put_str(b, pos, cap, c);
        }
    }
    return pos;
}

int params_format(const flight_config_t *cfg, int i, char *buf, int cap)
{
    if (cap < 2 || i < 0 || i >= N_DEFS) return 0;
    int pos = put_str(buf, 0, cap, defs[i].name);
    pos = put_str(buf, pos, cap, " = ");
    return put_fixed(buf, pos, cap, params_get(cfg, i), defs[i].decimals);
}

int params_serialize(const flight_config_t *cfg, char *buf, int cap)
{
    int pos = 0;
    for (int i = 0; i < N_DEFS; i++) {
        pos = put_str(buf, pos, cap, defs[i].name);
        pos = put_str(buf, pos, cap, "=");
        /* immer 4 Nachkommastellen, damit nichts verloren geht */
        pos = put_fixed(buf, pos, cap, params_get(cfg, i), 4);
        pos = put_str(buf, pos, cap, "\n");
        if (pos >= cap - 1) return -1;
    }
    return pos;
}

int params_deserialize(flight_config_t *cfg, const char *text, int len)
{
    int set = 0, start = 0;
    char line[64];
    for (int k = 0; k <= len; k++) {
        if (k == len || text[k] == '\n') {
            const int n = k - start;
            if (n > 0 && n < (int)sizeof(line)) {
                memcpy(line, &text[start], (size_t)n);
                line[n] = 0;
                char *eq = strchr(line, '=');
                if (eq) {
                    *eq = 0;
                    if (params_set(cfg, line, eq + 1)) set++;
                }
            }
            start = k + 1;
        }
    }
    return set;
}

uint32_t params_crc32(const void *data, size_t len)
{
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
    }
    return ~crc;
}

static void w32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint32_t r32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

int params_pack(const flight_config_t *cfg, uint8_t *blob, int cap)
{
    if (cap <= PARAMS_HEADER) return -1;
    const int n = params_serialize(cfg, (char *)blob + PARAMS_HEADER, cap - PARAMS_HEADER);
    if (n < 0) return -1;
    w32(blob, PARAMS_MAGIC);
    w32(blob + 4, (uint32_t)n);
    w32(blob + 8, params_crc32(blob + PARAMS_HEADER, (size_t)n));
    return PARAMS_HEADER + n;
}

int params_unpack(flight_config_t *cfg, const uint8_t *blob, int len)
{
    if (len < PARAMS_HEADER || r32(blob) != PARAMS_MAGIC) return -1;
    const uint32_t n = r32(blob + 4);
    if (n == 0 || n > (uint32_t)(len - PARAMS_HEADER)) return -1;
    if (params_crc32(blob + PARAMS_HEADER, n) != r32(blob + 8)) return -1;
    return params_deserialize(cfg, (const char *)blob + PARAMS_HEADER, (int)n);
}
