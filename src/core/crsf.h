/*
 * crsf.h - Parser fuer das CRSF-Protokoll (ExpressLRS-Empfaenger -> FC)
 *
 * Paketaufbau (TBS CRSF Rev. 07):
 *   [Adresse][Laenge][Typ][Nutzdaten ...][CRC8]
 *   Laenge = Anzahl Bytes von Typ bis und mit CRC
 *   CRC8 (Polynom 0xD5) ueber Typ + Nutzdaten
 *
 * Typ 0x16 = RC-Kanaele: 16 Kanaele a 11 Bit, LSB zuerst, 22 Byte.
 *   172 = 988 us, 992 = 1500 us, 1811 = 2012 us
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CRSF_BAUDRATE              420000u
#define CRSF_FRAME_SIZE_MAX        64u
#define CRSF_NUM_CHANNELS          16u
#define CRSF_FRAMETYPE_LINK_STATS  0x14u
#define CRSF_FRAMETYPE_RC_CHANNELS 0x16u

#define CRSF_CHANNEL_MIN  172u
#define CRSF_CHANNEL_MID  992u
#define CRSF_CHANNEL_MAX  1811u

typedef struct {
    uint8_t  buf[CRSF_FRAME_SIZE_MAX];
    uint8_t  pos;
    uint32_t frames_ok;
    uint32_t crc_errors;
} crsf_parser_t;

typedef enum {
    CRSF_NONE = 0,     /* noch kein vollstaendiges Paket */
    CRSF_RC_CHANNELS,  /* neue Kanalwerte in channels[] */
    CRSF_LINK_STATS,   /* Link-Statistik empfangen */
    CRSF_OTHER         /* gueltiges Paket anderer Art */
} crsf_result_t;

typedef struct {
    uint8_t uplink_rssi_1;      /* -dBm */
    uint8_t uplink_link_quality;/* % */
    int8_t  uplink_snr;
} crsf_link_stats_t;

uint8_t       crsf_crc8(const uint8_t *data, uint32_t len);
void          crsf_parser_init(crsf_parser_t *p);
/* Ein Byte einspeisen. Ist ein Paket fertig, wird es ausgewertet. */
crsf_result_t crsf_parser_feed(crsf_parser_t *p, uint8_t byte,
                               uint16_t channels[CRSF_NUM_CHANNELS],
                               crsf_link_stats_t *stats);
/* Kanalwert -> -1 ... +1 (Mitte = 0) bzw. 0 ... 1 */
float crsf_to_bipolar(uint16_t v);
float crsf_to_unipolar(uint16_t v);
