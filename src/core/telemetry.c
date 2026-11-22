#include "telemetry.h"

static int put_str(char *b, int pos, int cap, const char *s)
{
    while (*s && pos < cap - 1) b[pos++] = *s++;
    b[pos < cap ? pos : cap - 1] = 0;
    return pos;
}

static int put_int(char *b, int pos, int cap, int32_t v)
{
    char t[12];
    int n = 0;
    uint32_t u = (v < 0) ? (uint32_t)(-(v + 1)) + 1u : (uint32_t)v;
    do { t[n++] = (char)('0' + u % 10u); u /= 10u; } while (u && n < 11);
    if (v < 0) t[n++] = '-';
    while (n-- && pos < cap - 1) b[pos++] = t[n];
    b[pos < cap ? pos : cap - 1] = 0;
    return pos;
}

static int32_t scaled(float v, float scale)
{
    const float x = v * scale;
    if (!(x == x)) return 0;                      /* NaN */
    if (x > 2.0e9f) return 2000000000;
    if (x < -2.0e9f) return -2000000000;
    return (int32_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
}

static int field(char *b, int pos, int cap, int32_t v)
{
    pos = put_str(b, pos, cap, ",");
    return put_int(b, pos, cap, v);
}

int tlm_format_header(char *buf, int cap)
{
    if (cap < 1) return 0;
    buf[0] = 0;
    return put_str(buf, put_str(buf, 0, cap, "$H,"), cap, TLM_FIELDS);
}

int tlm_format(const tlm_t *t, char *buf, int cap)
{
    if (cap < 1) return 0;
    buf[0] = 0;
    int p = put_str(buf, 0, cap, "$T,");
    p = put_int(buf, p, cap, (int32_t)t->t_ms);
    p = field(buf, p, cap, t->armed);
    p = field(buf, p, cap, t->failsafe);
    p = field(buf, p, cap, t->mode);
    p = field(buf, p, cap, t->airmode);
    p = field(buf, p, cap, t->calibrated);
    p = field(buf, p, cap, t->motor_test);
    p = field(buf, p, cap, t->arm_block);
    for (int a = 0; a < 3; a++) p = field(buf, p, cap, scaled(t->att_deg[a], 10.0f));
    for (int a = 0; a < 3; a++) p = field(buf, p, cap, scaled(t->gyro_dps[a], 10.0f));
    for (int a = 0; a < 3; a++) p = field(buf, p, cap, scaled(t->acc_g[a], 1000.0f));
    for (int a = 0; a < 3; a++) p = field(buf, p, cap, scaled(t->setpoint_dps[a], 10.0f));
    p = field(buf, p, cap, scaled(t->throttle, 1000.0f));
    for (int m = 0; m < 4; m++) p = field(buf, p, cap, t->motor[m]);
    for (int c = 0; c < 8; c++) p = field(buf, p, cap, t->rc[c]);
    p = field(buf, p, cap, t->link_quality);
    p = field(buf, p, cap, scaled(t->vbat_v, 1000.0f));
    p = field(buf, p, cap, (int32_t)t->loop_avg_x10);
    p = field(buf, p, cap, (int32_t)t->loop_max_us);
    p = field(buf, p, cap, t->rx_uart);
    p = field(buf, p, cap, (int32_t)t->rx_frames);
    p = field(buf, p, cap, (int32_t)t->rx_crc_errors);
    for (int m = 0; m < 4; m++) p = field(buf, p, cap, t->out[m]);
    return p;
}

int tlm_battery_cells(float v)
{
    /* geladen max. 4.2-4.35 V, leer ca. 3.3 V: v / 4.3 aufgerundet trennt 3S (bis 12.9 V)
     * von 4S (ab 13.2 V) usw. Am besten im Ruhezustand (beim Einstecken) bestimmen. */
    if (v < 5.0f) return 0;
    int n = (int)(v / 4.3f);
    if ((float)n * 4.3f < v) n++;
    return n < 2 ? 2 : n > 6 ? 6 : n;
}

int tlm_battery_percent_cells(float v, int n)
{
    if (n == 0) return 0;
    const float x = (v / (float)n - 3.3f) / (4.2f - 3.3f) * 100.0f;
    return x <= 0.0f ? 0 : x >= 100.0f ? 100 : (int)(x + 0.5f);
}

int tlm_battery_percent(float v)
{
    return tlm_battery_percent_cells(v, tlm_battery_cells(v));
}
