/*
 * test_core.c - Tests der hardwareunabhaengigen Module auf dem PC
 *
 *   cmake -S tests -B build-tests && cmake --build build-tests && ./build-tests/test_core
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "crsf.h"
#include "dshot.h"
#include "flight.h"
#include "mahony.h"
#include "mixer.h"
#include "pid.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond) do { checks++; if (!(cond)) { failures++; \
    printf("  FEHLER %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b, tol) do { checks++; if (fabs((double)(a) - (double)(b)) > (tol)) { failures++; \
    printf("  FEHLER %s:%d: %s = %g, erwartet %g\n", __FILE__, __LINE__, #a, (double)(a), (double)(b)); } } while (0)

/* ---------- CRSF ---------- */

static void test_crsf(void)
{
    printf("CRSF\n");
    /* Beispielpaket aus dem CRSF-Wiki: alle 16 Kanaele = 992 (1500 us) */
    const uint8_t frame[] = {
        0xC8, 0x18, 0x16, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C,
        0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0xAD };

    CHECK(crsf_crc8(&frame[2], sizeof(frame) - 3) == 0xAD);

    crsf_parser_t p;
    crsf_parser_init(&p);
    uint16_t ch[CRSF_NUM_CHANNELS] = {0};
    crsf_result_t r = CRSF_NONE;

    /* Muell davor darf nicht stoeren */
    const uint8_t junk[] = { 0x00, 0x13, 0x77 };
    for (unsigned i = 0; i < sizeof(junk); i++) crsf_parser_feed(&p, junk[i], ch, NULL);

    for (unsigned i = 0; i < sizeof(frame); i++) {
        r = crsf_parser_feed(&p, frame[i], ch, NULL);
    }
    CHECK(r == CRSF_RC_CHANNELS);
    for (int i = 0; i < 16; i++) CHECK(ch[i] == 992);

    /* Fehlerhafte Pruefsumme wird verworfen */
    uint8_t bad[sizeof(frame)];
    memcpy(bad, frame, sizeof(frame));
    bad[5] ^= 0x01;
    for (unsigned i = 0; i < sizeof(bad); i++) r = crsf_parser_feed(&p, bad[i], ch, NULL);
    CHECK(r == CRSF_NONE);
    CHECK(p.crc_errors == 1);

    CHECK_NEAR(crsf_to_bipolar(CRSF_CHANNEL_MID), 0.0, 1e-6);
    CHECK_NEAR(crsf_to_bipolar(CRSF_CHANNEL_MAX), 1.0, 2e-3);
    CHECK_NEAR(crsf_to_bipolar(CRSF_CHANNEL_MIN), -1.0, 2e-3);
    CHECK_NEAR(crsf_to_unipolar(CRSF_CHANNEL_MIN), 0.0, 1e-6);
}

/* ---------- DShot ---------- */

static void test_dshot(void)
{
    printf("DShot\n");
    /* Gas 0, ohne Telemetrie: v = 0 -> CRC 0 -> Frame 0 */
    CHECK(dshot_make_frame(0, false) == 0x0000);
    /* Gas 1046 (0b10000010110), keine Telemetrie:
       v = 0x82C, CRC = (0x82C ^ 0x82 ^ 0x8) & 0xF = 0x6 -> Frame 0x82C6 */
    CHECK(dshot_make_frame(1046, false) == 0x82C6);
    /* CRC-Eigenschaft: XOR aller vier Nibbles ist 0 */
    for (uint16_t v = 48; v < 2048; v += 37) {
        const uint16_t fr = dshot_make_frame(v, (v & 1) != 0);
        CHECK(((fr ^ (fr >> 4) ^ (fr >> 8) ^ (fr >> 12)) & 0xF) == 0);
        CHECK((fr >> 5) == v);
    }
    CHECK(dshot_from_output(0.0f, 0.0f) == 48);
    CHECK(dshot_from_output(1.0f, 0.0f) == 2047);
    CHECK(dshot_from_output(0.0f, 0.055f) > 48);

    uint32_t buf[DSHOT_BUFFER_SLOTS * 4];
    memset(buf, 0xFF, sizeof(buf));
    dshot_fill_buffer(&buf[1], 4, 0x8001, 10, 20);
    CHECK(buf[1] == 20);              /* erstes Bit = MSB = 1 */
    CHECK(buf[1 + 4] == 10);
    CHECK(buf[1 + 15 * 4] == 20);     /* letztes Bit = 1 */
    CHECK(buf[1 + 16 * 4] == 0);      /* Abschluss */
    CHECK(buf[0] == 0xFFFFFFFFu);     /* andere Motoren unberuehrt */
}

/* ---------- PID ---------- */

static void test_pid(void)
{
    printf("PID\n");
    pid_axis_t pid;
    pid_init(&pid, 45, 0, 0, 0.001f, 75, 400, 500);
    /* nur P: 100 dps Fehler -> 45 * 0.032029 * 100 */
    CHECK_NEAR(pid_update(&pid, 100, 0, false), 45 * PID_PTERM_SCALE * 100, 1e-3);

    pid_init(&pid, 0, 80, 0, 0.001f, 75, 400, 500);
    for (int i = 0; i < 1000; i++) pid_update(&pid, 10, 0, false);
    /* 1 s bei 10 dps: I = 80 * 0.244381 * 10 * 1 */
    CHECK_NEAR(pid.i_term, 80 * PID_ITERM_SCALE * 10, 0.5);

    /* Anti-Windup: I bleibt begrenzt */
    for (int i = 0; i < 100000; i++) pid_update(&pid, 500, 0, false);
    CHECK_NEAR(pid.i_term, 400, 1e-3);

    /* D auf den Istwert: steigender Istwert -> negativer D-Anteil */
    pid_init(&pid, 0, 0, 30, 0.001f, 1000, 400, 500);
    pid_update(&pid, 0, 0, false);
    float out = 0;
    for (int i = 1; i < 50; i++) out = pid_update(&pid, 0, (float)i, false);
    CHECK(out < 0.0f);
}

/* ---------- Mischer ---------- */

static void test_mixer(void)
{
    printf("Mischer\n");
    mixer_config_t cfg = { .yaw_motors_reversed = false };
    float m[4];

    mixer_mix(&cfg, 0.5f, 0, 0, 0, m);
    for (int i = 0; i < 4; i++) CHECK_NEAR(m[i], 0.5, 1e-6);

    /* positives Roll (nach rechts rollen) -> linke Motoren (3, 4) staerker */
    mixer_mix(&cfg, 0.5f, 0.1f, 0, 0, m);
    CHECK(m[2] > m[0] && m[3] > m[1]);

    /* positives Pitch (Nase runter) -> hintere Motoren (1, 3) staerker */
    mixer_mix(&cfg, 0.5f, 0, 0.1f, 0, m);
    CHECK(m[0] > m[1] && m[2] > m[3]);

    /* Desaturierung: Regelanteil bleibt erhalten, auch bei Gas 0 */
    mixer_mix(&cfg, 0.0f, 0.2f, 0, 0, m);
    CHECK(m[2] - m[0] > 0.39f);
    for (int i = 0; i < 4; i++) CHECK(m[i] >= 0.0f && m[i] <= 1.0f);

    mixer_mix(&cfg, 1.0f, 0.8f, 0.8f, 0.8f, m);
    for (int i = 0; i < 4; i++) CHECK(m[i] >= 0.0f && m[i] <= 1.0f);
}

/* ---------- Mahony ---------- */

static void test_mahony(void)
{
    printf("Mahony\n");
    mahony_t m;
    float r, p, y;

    /* Stillstand, waagrecht: bleibt bei 0 */
    mahony_init(&m, 0.25f, 0);
    for (int i = 0; i < 2000; i++) mahony_update(&m, 0, 0, 0, 0, 0, 1, true, 0.001f);
    mahony_get_euler_deg(&m, &r, &p, &y);
    CHECK_NEAR(r, 0, 0.01); CHECK_NEAR(p, 0, 0.01);

    /* Rechts um 30 Grad geneigt (y zeigt teilweise nach oben) -> roll = +30 */
    mahony_init(&m, 2.0f, 0);
    for (int i = 0; i < 20000; i++)
        mahony_update(&m, 0, 0, 0, 0, sinf(30 * DEG2RAD), cosf(30 * DEG2RAD), true, 0.001f);
    mahony_get_euler_deg(&m, &r, &p, &y);
    CHECK_NEAR(r, 30, 0.5);

    /* Nase 20 Grad nach unten (x zeigt teilweise nach unten) -> pitch = +20 */
    mahony_init(&m, 2.0f, 0);
    for (int i = 0; i < 20000; i++)
        mahony_update(&m, 0, 0, 0, -sinf(20 * DEG2RAD), 0, cosf(20 * DEG2RAD), true, 0.001f);
    mahony_get_euler_deg(&m, &r, &p, &y);
    CHECK_NEAR(p, 20, 0.5);

    /* Nur Gyro: +90 dps Rollrate fuer 0.5 s -> roll = +45 */
    mahony_init(&m, 0, 0);
    for (int i = 0; i < 500; i++) mahony_update(&m, 90 * DEG2RAD, 0, 0, 0, 0, 1, false, 0.001f);
    mahony_get_euler_deg(&m, &r, &p, &y);
    CHECK_NEAR(r, 45, 0.1);

    /* Nur Gyro: +60 dps Nickrate fuer 0.5 s -> pitch = +30 */
    mahony_init(&m, 0, 0);
    for (int i = 0; i < 500; i++) mahony_update(&m, 0, 60 * DEG2RAD, 0, 0, 0, 1, false, 0.001f);
    mahony_get_euler_deg(&m, &r, &p, &y);
    CHECK_NEAR(p, 30, 0.1);
}

/* ---------- Flugsteuerung: Arming, Failsafe, Regelrichtung ---------- */

static void set_sticks(flight_input_t *in, uint16_t thr, uint16_t arm)
{
    for (int i = 0; i < 16; i++) in->channels[i] = CRSF_CHANNEL_MID;
    in->channels[CH_THROTTLE] = thr;
    in->channels[CH_ARM] = arm;
    in->channels[CH_MODE] = CRSF_CHANNEL_MIN;
}

static void run(flight_t *f, flight_input_t *in, int n)
{
    for (int i = 0; i < n; i++) flight_step(f, in);
}

static void test_flight(void)
{
    printf("Flugsteuerung\n");
    flight_config_t cfg;
    flight_default_config(&cfg);
    flight_t f;
    flight_init(&f, &cfg);

    flight_input_t in;
    memset(&in, 0, sizeof(in));
    in.acc_g[2] = 1.0f;
    in.rc_valid = true;

    /* Arm-Schalter beim Einschalten schon an: darf NICHT armen */
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
    run(&f, &in, 1500);
    CHECK(f.calibrated);
    CHECK(!f.armed);
    for (int i = 0; i < 4; i++) CHECK(f.dshot[i] == 0);

    /* Schalter aus, wieder ein: armt */
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MIN);
    run(&f, &in, 10);
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
    run(&f, &in, 10);
    CHECK(f.armed);
    for (int i = 0; i < 4; i++) CHECK(f.dshot[i] >= 48);

    /* Regelrichtung: Drohne rollt nach rechts (positive Rate), Sollwert 0
       -> Regler muss rechte Motoren (1, 2) staerker machen */
    set_sticks(&in, 1200, CRSF_CHANNEL_MAX);
    in.gyro_dps[AXIS_ROLL] = 100.0f;
    run(&f, &in, 20);
    CHECK(f.motor[0] > f.motor[2] && f.motor[1] > f.motor[3]);
    in.gyro_dps[AXIS_ROLL] = 0.0f;

    /* Nase geht nach unten (positive Nickrate) -> vordere Motoren (2, 4) staerker */
    pid_reset(&f.pid[AXIS_PITCH]);
    in.gyro_dps[AXIS_PITCH] = 100.0f;
    run(&f, &in, 20);
    CHECK(f.motor[1] > f.motor[0] && f.motor[3] > f.motor[2]);
    in.gyro_dps[AXIS_PITCH] = 0.0f;

    /* Failsafe: keine Pakete -> sofort aus */
    in.rc_valid = false;
    run(&f, &in, 1);
    CHECK(!f.armed && f.failsafe);
    for (int i = 0; i < 4; i++) CHECK(f.dshot[i] == 0);

    /* Verbindung zurueck, Schalter noch an: bleibt aus */
    in.rc_valid = true;
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
    run(&f, &in, 10);
    CHECK(!f.armed);

    /* Mit Gas oben darf nicht gearmt werden */
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MIN);
    run(&f, &in, 5);
    set_sticks(&in, 1200, CRSF_CHANNEL_MAX);
    run(&f, &in, 5);
    CHECK(!f.armed);

    /* Kalibrierung startet neu, wenn die Drohne bewegt wird */
    flight_init(&f, &cfg);
    for (int i = 0; i < 1500; i++) {
        in.gyro_dps[AXIS_ROLL] = (i % 100 < 50) ? 20.0f : -20.0f;
        flight_step(&f, &in);
    }
    CHECK(!f.calibrated);
}

int main(void)
{
    test_crsf();
    test_dshot();
    test_pid();
    test_mixer();
    test_mahony();
    test_flight();
    printf("\n%d Pruefungen, %d Fehler\n", checks, failures);
    return failures ? 1 : 0;
}
