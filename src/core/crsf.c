#include "crsf.h"
#include "util.h"

uint8_t crsf_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

void crsf_parser_init(crsf_parser_t *p)
{
    p->pos = 0;
    p->frames_ok = 0;
    p->crc_errors = 0;
}

static void unpack_channels(const uint8_t *payload, uint16_t ch[CRSF_NUM_CHANNELS])
{
    /* 16 x 11 Bit, LSB zuerst, lueckenlos hintereinander */
    uint32_t bitbuf = 0;
    unsigned bits = 0;
    unsigned idx = 0;
    for (unsigned i = 0; i < 22 && idx < CRSF_NUM_CHANNELS; i++) {
        bitbuf |= (uint32_t)payload[i] << bits;
        bits += 8;
        while (bits >= 11 && idx < CRSF_NUM_CHANNELS) {
            ch[idx++] = (uint16_t)(bitbuf & 0x7FF);
            bitbuf >>= 11;
            bits -= 11;
        }
    }
}

crsf_result_t crsf_parser_feed(crsf_parser_t *p, uint8_t byte,
                               uint16_t channels[CRSF_NUM_CHANNELS],
                               crsf_link_stats_t *stats)
{
    /* Byte 0: Adresse bzw. Sync-Byte. Der Empfaenger sendet an den FC 0xC8;
     * 0xEE/0xEA kommen bei manchen Sendern/Empfaengern ebenfalls vor. */
    if (p->pos == 0 && byte != 0xC8 && byte != 0xEE && byte != 0xEA) {
        return CRSF_NONE;
    }
    p->buf[p->pos++] = byte;

    if (p->pos == 2) {
        const uint8_t len = p->buf[1];
        if (len < 2 || len > CRSF_FRAME_SIZE_MAX - 2) {
            /* ungueltige Laenge: um ein Byte verschieben und neu synchronisieren */
            p->buf[0] = p->buf[1];
            p->pos = 1;
        }
        return CRSF_NONE;
    }
    if (p->pos < 2) {
        return CRSF_NONE;
    }

    const uint8_t frame_len = (uint8_t)(p->buf[1] + 2);  /* Adresse + Laenge + Rest */
    if (p->pos < frame_len) {
        return CRSF_NONE;
    }

    /* Paket vollstaendig */
    p->pos = 0;
    const uint8_t type = p->buf[2];
    const uint8_t crc = crsf_crc8(&p->buf[2], (uint32_t)(p->buf[1] - 1));
    if (crc != p->buf[frame_len - 1]) {
        p->crc_errors++;
        return CRSF_NONE;
    }
    p->frames_ok++;

    if (type == CRSF_FRAMETYPE_RC_CHANNELS && p->buf[1] == 24) {
        unpack_channels(&p->buf[3], channels);
        return CRSF_RC_CHANNELS;
    }
    if (type == CRSF_FRAMETYPE_LINK_STATS && p->buf[1] >= 12 && stats) {
        stats->uplink_rssi_1 = p->buf[3];
        stats->uplink_link_quality = p->buf[5];
        stats->uplink_snr = (int8_t)p->buf[6];
        return CRSF_LINK_STATS;
    }
    return CRSF_OTHER;
}

float crsf_to_bipolar(uint16_t v)
{
    const float x = ((float)v - (float)CRSF_CHANNEL_MID) /
                    (0.5f * (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN));
    return constrainf(x, -1.0f, 1.0f);
}

float crsf_to_unipolar(uint16_t v)
{
    const float x = ((float)v - (float)CRSF_CHANNEL_MIN) /
                    (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN);
    return constrainf(x, 0.0f, 1.0f);
}
