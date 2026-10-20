#include "logrec.h"
#include "crsf.h"
#include <string.h>

uint8_t logrec_crc8(const uint8_t *data, uint32_t len)
{
    return crsf_crc8(data, len);   /* gleiche CRC8 (Polynom 0xD5) wie CRSF */
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static int32_t round_clamp(float x, int32_t lo, int32_t hi)
{
    const float r = (x >= 0.0f) ? x + 0.5f : x - 0.5f;
    if (r <= (float)lo) return lo;
    if (r >= (float)hi) return hi;
    return (int32_t)r;
}

static void finish(uint8_t out[LOGREC_SIZE])
{
    out[LOGREC_SIZE - 1] = logrec_crc8(out, LOGREC_SIZE - 1);
}

void logrec_encode_data(const logrec_data_t *d, uint8_t out[LOGREC_SIZE])
{
    memset(out, 0, LOGREC_SIZE);
    out[0] = LOGREC_TYPE_DATA;
    out[1] = d->flags;
    put_u32(&out[2], d->t_us);
    for (int a = 0; a < 3; a++) {
        put_u16(&out[6 + 2 * a], (uint16_t)(int16_t)round_clamp(d->setpoint_dps[a] * 10.0f, -32768, 32767));
        put_u16(&out[12 + 2 * a], (uint16_t)(int16_t)round_clamp(d->gyro_dps[a] * 10.0f, -32768, 32767));
    }
    for (int m = 0; m < 4; m++) {
        put_u16(&out[18 + 2 * m], d->motor[m]);
    }
    put_u16(&out[26], (uint16_t)round_clamp(d->throttle * 10000.0f, 0, 10000));
    put_u16(&out[28], (uint16_t)round_clamp(d->vbat_v * 1000.0f, 0, 65535));
    out[30] = d->link_quality;
    finish(out);
}

void logrec_encode_header(const logrec_header_t *h, uint8_t out[LOGREC_SIZE])
{
    memset(out, 0, LOGREC_SIZE);
    out[0] = LOGREC_TYPE_HEADER;
    out[1] = LOGREC_VERSION;
    put_u32(&out[2], h->git_hash);
    put_u16(&out[6], h->loop_hz);
    put_u16(&out[8], h->log_hz);
    for (int a = 0; a < 3; a++) {
        for (int k = 0; k < 3; k++) {
            out[10 + 3 * a + k] = (uint8_t)round_clamp(h->pid[a][k], 0, 255);
        }
    }
    put_u16(&out[19], h->max_rate_dps);
    out[21] = h->gyro_lpf_hz;
    out[22] = h->dterm_lpf_hz;
    finish(out);
}

void logrec_encode_pad(uint8_t out[LOGREC_SIZE])
{
    memset(out, 0, LOGREC_SIZE);
    out[0] = LOGREC_TYPE_PAD;
    finish(out);
}

bool logrec_crc_ok(const uint8_t rec[LOGREC_SIZE])
{
    return logrec_crc8(rec, LOGREC_SIZE - 1) == rec[LOGREC_SIZE - 1];
}
