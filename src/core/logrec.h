/*
 * logrec.h - Binaerformat der Flugdaten (Build B)
 *
 * Jeder Datensatz ist 32 Byte lang (Little Endian), damit genau 8 Datensaetze
 * in eine Flash-Seite von 256 Byte passen und nie ueber eine Seitengrenze gehen.
 * Byte 31 ist eine CRC8 (Polynom 0xD5) ueber Byte 0..30.
 *
 * Datensatz (Typ 0xA5), einmal pro Regelzyklus:
 *   0      Typ 0xA5
 *   1      Flags: Bit0 armed, Bit1 Failsafe, Bit2 Angle-Mode
 *   2..5   Zeit [us], uint32
 *   6..11  Soll-Drehrate roll, pitch, yaw [0.1 dps], int16
 *   12..17 Ist-Drehrate (gefiltert) roll, pitch, yaw [0.1 dps], int16
 *   18..25 Motorbefehle M1..M4 [DShot-Wert], uint16
 *   26..27 Gas [0..10000], uint16
 *   28..29 Akkuspannung [mV], uint16
 *   30     Link-Qualitaet [%], uint8
 *   31     CRC8
 *
 * Kopf (Typ 0x5A), einmal beim Armen (Beginn eines Flugs):
 *   0      Typ 0x5A
 *   1      Formatversion (1)
 *   2..5   Git-Commit der Firmware (erste 8 Hex-Ziffern), uint32
 *   6..7   Regelfrequenz [Hz], uint16
 *   8..9   Logfrequenz [Hz], uint16
 *   10..18 PID-Verstaerkungen P,I,D roll, P,I,D pitch, P,I,D yaw (Betaflight-Zahlen), uint8
 *   19..20 maximale Drehrate [dps], uint16
 *   21     Gyro-Tiefpass [Hz], uint8
 *   22     D-Tiefpass [Hz], uint8
 *   23..30 reserviert (0)
 *   31     CRC8
 *
 * Fuellsatz (Typ 0x00): fuellt beim Disarmen den Rest der Flash-Seite.
 * 0xFF am Satzanfang = geloeschter Flash = Ende der Daten.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define LOGREC_SIZE         32u
#define LOGREC_TYPE_DATA    0xA5u
#define LOGREC_TYPE_HEADER  0x5Au
#define LOGREC_TYPE_PAD     0x00u
#define LOGREC_TYPE_ERASED  0xFFu
#define LOGREC_VERSION      1u

#define LOGREC_FLAG_ARMED    0x01u
#define LOGREC_FLAG_FAILSAFE 0x02u
#define LOGREC_FLAG_ANGLE    0x04u

typedef struct {
    uint32_t t_us;
    float    setpoint_dps[3];
    float    gyro_dps[3];
    uint16_t motor[4];
    float    throttle;        /* 0 ... 1 */
    float    vbat_v;
    uint8_t  link_quality;
    uint8_t  flags;
} logrec_data_t;

typedef struct {
    uint32_t git_hash;
    uint16_t loop_hz;
    uint16_t log_hz;
    float    pid[3][3];       /* [Achse][P, I, D] */
    uint16_t max_rate_dps;
    uint8_t  gyro_lpf_hz;
    uint8_t  dterm_lpf_hz;
} logrec_header_t;

uint8_t logrec_crc8(const uint8_t *data, uint32_t len);
void    logrec_encode_data(const logrec_data_t *d, uint8_t out[LOGREC_SIZE]);
void    logrec_encode_header(const logrec_header_t *h, uint8_t out[LOGREC_SIZE]);
void    logrec_encode_pad(uint8_t out[LOGREC_SIZE]);
bool    logrec_crc_ok(const uint8_t rec[LOGREC_SIZE]);
