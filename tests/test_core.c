/*
 * test_core.c - Tests der hardwareunabhaengigen Module auf dem PC
 *
 *   cmake -S tests -B build-tests && cmake --build build-tests && ./build-tests/test_core
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "blackbox.h"
#include "crsf.h"
#include "logrec.h"
#include "dshot.h"
#include "flight.h"
#include "mahony.h"
#include "mixer.h"
#include "params.h"
#include "pid.h"
#include "rates.h"

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

static pid_params_t pid_prm(float p, float i, float d, float f)
{
    pid_params_t prm = {
        .p_gain = p, .i_gain = i, .d_gain = d, .f_gain = f,
        .i_limit = 400, .sum_limit = 500, .dt = 0.001f,
        .dterm_lpf1_type = FILTER_OFF, .dterm_lpf2_type = FILTER_OFF,
        .ff_lpf_hz = 0, .iterm_relax = false, .iterm_relax_hz = 15,
    };
    return prm;
}

static const pid_runtime_t RT_NORMAL = { .i_factor = 1.0f, .d_factor = 1.0f, .stabilise = true };

static void test_pid(void)
{
    {   /* ungueltiger Messwert: Ausgang 0, I bleibt endlich */
        pid_params_t q = { .p_gain = 45, .i_gain = 80, .d_gain = 30, .i_limit = 400, .sum_limit = 500,
                           .dt = 0.001f };
        pid_axis_t a;
        pid_runtime_t r = { .i_factor = 1, .d_factor = 1, .stabilise = true };
        pid_init(&a, &q);
        pid_update(&a, 100, 0, &r);
        CHECK(pid_update(&a, 100, NAN, &r) == 0.0f);
        CHECK(isfinite(a.i_term));
        pid_update(&a, 100, 0, &r);
        CHECK(isfinite(a.d) && a.out > 0.0f);   /* erholt sich wieder (Filter zurueckgesetzt) */
    }

    printf("PID\n");
    pid_axis_t pid;
    pid_params_t prm = pid_prm(45, 0, 0, 0);
    pid_init(&pid, &prm);
    /* nur P: 100 dps Fehler -> 45 * 0.032029 * 100 */
    CHECK_NEAR(pid_update(&pid, 100, 0, &RT_NORMAL), 45 * PID_PTERM_SCALE * 100, 1e-3);

    prm = pid_prm(0, 80, 0, 0);
    pid_init(&pid, &prm);
    for (int i = 0; i < 1000; i++) pid_update(&pid, 10, 0, &RT_NORMAL);
    /* 1 s bei 10 dps: I = 80 * 0.244381 * 10 * 1 */
    CHECK_NEAR(pid.i_term, 80 * PID_ITERM_SCALE * 10, 0.5);

    /* I bleibt begrenzt */
    for (int i = 0; i < 100000; i++) pid_update(&pid, 500, 0, &RT_NORMAL);
    CHECK_NEAR(pid.i_term, 400, 1e-3);

    /* Anti-Windup ueber i_factor: I waechst nicht weiter */
    pid_reset(&pid);
    pid_runtime_t rt = RT_NORMAL;
    rt.i_factor = 0.0f;
    for (int i = 0; i < 1000; i++) pid_update(&pid, 50, 0, &rt);
    CHECK_NEAR(pid.i_term, 0, 1e-6);

    /* stabilise = false: Ausgang 0, I zurueckgesetzt */
    for (int i = 0; i < 100; i++) pid_update(&pid, 50, 0, &RT_NORMAL);
    CHECK(pid.i_term > 0.0f);
    rt = RT_NORMAL; rt.stabilise = false;
    CHECK_NEAR(pid_update(&pid, 50, 0, &rt), 0, 1e-9);
    CHECK_NEAR(pid.i_term, 0, 1e-9);

    /* D auf den Istwert: steigender Istwert -> negativer D-Anteil, Sollsprung -> kein D */
    prm = pid_prm(0, 0, 30, 0);
    pid_init(&pid, &prm);
    pid_update(&pid, 0, 0, &RT_NORMAL);
    float out = 0;
    for (int i = 1; i < 50; i++) out = pid_update(&pid, 0, (float)i, &RT_NORMAL);
    CHECK(out < 0.0f);
    pid_reset(&pid);
    pid_update(&pid, 0, 0, &RT_NORMAL);
    CHECK_NEAR(pid_update(&pid, 300, 0, &RT_NORMAL), 0, 1e-6);
    /* D = Kd * 1 dps/ms = 30 * 0.000529 * 1000; TPA halbiert */
    pid_reset(&pid);
    pid_update(&pid, 0, 0, &RT_NORMAL);
    CHECK_NEAR(pid_update(&pid, 0, -1, &RT_NORMAL), 30 * PID_DTERM_SCALE * 1000, 1e-3);
    rt = RT_NORMAL; rt.d_factor = 0.5f;
    CHECK_NEAR(pid_update(&pid, 0, -2, &rt), 0.5 * 30 * PID_DTERM_SCALE * 1000, 1e-3);

    /* Feedforward: Rampe 0.1 dps/ms -> F = Kf * 100 dps/s */
    prm = pid_prm(0, 0, 0, 100);
    pid_init(&pid, &prm);
    for (int i = 0; i < 20; i++) out = pid_update(&pid, 0.1f * (float)i, 0.1f * (float)i, &RT_NORMAL);
    CHECK_NEAR(out, 100 * PID_FEEDFORWARD_SCALE * 100, 1e-2);

    /* I-Term-Relax: bei schnellem Sollsprung baut sich kaum I auf */
    pid_params_t a = pid_prm(0, 80, 0, 0), b = a;
    b.iterm_relax = true;
    pid_axis_t pa, pb;
    pid_init(&pa, &a); pid_init(&pb, &b);
    for (int i = 0; i < 30; i++) { pid_update(&pa, 400, 0, &RT_NORMAL); pid_update(&pb, 400, 0, &RT_NORMAL); }
    CHECK(pb.i_term < 0.2f * pa.i_term);
    /* im Dauerzustand wirkt I wieder normal */
    for (int i = 0; i < 2000; i++) { pid_update(&pb, 10, 0, &RT_NORMAL); }
    CHECK(pb.i_term > 30.0f);

    /* Begrenzung der Summe; sum_raw behaelt den echten Wert */
    prm = pid_prm(200, 0, 0, 0);
    pid_init(&pid, &prm);
    CHECK_NEAR(pid_update(&pid, 1000, 0, &RT_NORMAL), 500, 1e-3);
    CHECK(pid.sum_raw > 500.0f);
}

/* ---------- Filter und Rates ---------- */

static float sine_gain(void (*init)(biquad_t *, float, float, float), float fc, float q, float f_test)
{
    biquad_t bq;
    init(&bq, fc, 1000.0f, q);
    float peak = 0;
    for (int i = 0; i < 4000; i++) {
        const float y = biquad_apply(&bq, sinf(2.0f * 3.14159265f * f_test * (float)i / 1000.0f));
        if (i > 3000 && fabsf(y) > peak) peak = fabsf(y);
    }
    return peak;
}

static void test_filters_rates(void)
{
    printf("Filter und Rates\n");
    /* Butterworth-Tiefpass 100 Hz: bei 100 Hz -3 dB (0.707), bei 10 Hz ~1, bei 400 Hz klein */
    CHECK_NEAR(sine_gain(biquad_init_lpf, 100, 0.7071f, 100), 0.707, 0.02);
    CHECK_NEAR(sine_gain(biquad_init_lpf, 100, 0.7071f, 10), 1.0, 0.01);
    CHECK(sine_gain(biquad_init_lpf, 100, 0.7071f, 400) < 0.08f);
    /* Notch 200 Hz: bei 200 Hz fast 0, weit weg ~1 */
    const float q = notch_q_from_cutoff(200, 160);
    CHECK_NEAR(q, 200.0 * 160.0 / (200.0 * 200.0 - 160.0 * 160.0), 1e-4);
    CHECK(sine_gain(biquad_init_notch, 200, q, 200) < 0.02f);
    CHECK_NEAR(sine_gain(biquad_init_notch, 200, q, 20), 1.0, 0.02);
    /* PT1: Gleichanteil wird 1, bei fc Amplitude ~0.707 (nur fuer fc << fs genau) */
    lpf_stage_t st;
    lpf_stage_init(&st, FILTER_PT1, 20, 1000);
    float y = 0;
    for (int i = 0; i < 2000; i++) y = lpf_stage_apply(&st, 1.0f);
    CHECK_NEAR(y, 1.0, 1e-4);
    lpf_stage_init(&st, FILTER_BIQUAD, 600, 1000);   /* ueber Nyquist -> aus */
    CHECK(st.type == FILTER_OFF);

    /* Rates wie Betaflight "Actual": linear bei center = max */
    CHECK_NEAR(rates_apply(0.5f, 400, 400, 0.5f), 200, 1e-3);
    CHECK_NEAR(rates_apply(-1.0f, 400, 400, 0), -400, 1e-3);
    /* Betaflight-Standard (70 / 670, expo 0): Vollausschlag 670, halber Ausschlag 35 + 600*0.25 */
    CHECK_NEAR(rates_apply(1.0f, 70, 670, 0), 670, 1e-3);
    CHECK_NEAR(rates_apply(0.5f, 70, 670, 0), 35 + 600 * 0.25, 1e-3);
    CHECK_NEAR(rates_apply(-0.5f, 70, 670, 0), -(35 + 600 * 0.25), 1e-3);
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

    /* Yaw-Stick rechts -> Soll-Gierrate negativ (im Uhrzeigersinn). Dafuer muessen die
       gegen den Uhrzeigersinn drehenden Propeller (Betaflight "props in": M2, M3) schneller werden */
    pid_reset(&f.pid[AXIS_YAW]);
    set_sticks(&in, 1200, CRSF_CHANNEL_MAX);
    in.channels[CH_YAW] = 1500;
    run(&f, &in, 20);
    CHECK(f.setpoint[AXIS_YAW] < -100.0f);
    CHECK(f.motor[1] > f.motor[0] && f.motor[2] > f.motor[3]);
    /* Roll-Stick rechts -> Soll positiv, linke Motoren (M3, M4) schneller */
    pid_reset(&f.pid[AXIS_YAW]);
    set_sticks(&in, 1200, CRSF_CHANNEL_MAX);
    in.channels[CH_ROLL] = 1500;
    run(&f, &in, 20);
    CHECK(f.setpoint[AXIS_ROLL] > 100.0f);
    CHECK(f.motor[2] > f.motor[0] && f.motor[3] > f.motor[1]);
    /* Pitch-Stick vorne -> Soll positiv (Nase runter), hintere Motoren (M1, M3) schneller */
    pid_reset(&f.pid[AXIS_ROLL]);
    set_sticks(&in, 1200, CRSF_CHANNEL_MAX);
    in.channels[CH_PITCH] = 1500;
    run(&f, &in, 20);
    CHECK(f.setpoint[AXIS_PITCH] > 100.0f);
    CHECK(f.motor[0] > f.motor[1] && f.motor[2] > f.motor[3]);
    set_sticks(&in, 1200, CRSF_CHANNEL_MAX);
    for (int a = 0; a < 3; a++) pid_reset(&f.pid[a]);

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

    /* Arm-Sperre (USB-Download): Schalterwechsel armt nicht */
    flight_init(&f, &cfg);
    memset(&in, 0, sizeof(in));
    in.acc_g[2] = 1.0f;
    in.rc_valid = true;
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MIN);
    run(&f, &in, 1500);
    in.arm_inhibit = true;
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
    run(&f, &in, 10);
    CHECK(!f.armed);

    /* Zwangs-Disarm (Gyro-Ausfall): bleibt aus, bis der Schalter neu betaetigt wird */
    in.arm_inhibit = false;
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MIN);
    run(&f, &in, 5);
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
    run(&f, &in, 5);
    CHECK(f.armed);
    flight_force_disarm(&f);
    CHECK(!f.armed && f.dshot[0] == 0);
    run(&f, &in, 50);
    CHECK(!f.armed);
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MIN);
    run(&f, &in, 5);
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
    run(&f, &in, 5);
    CHECK(f.armed);
}

/* ---------- Neue Funktionen der Flugsteuerung ---------- */

static void arm_now(flight_t *f, flight_input_t *in, uint16_t thr)
{
    set_sticks(in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MIN);
    run(f, in, 5);
    set_sticks(in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
    run(f, in, 5);
    in->channels[CH_THROTTLE] = thr;
}

static void test_flight_features(void)
{
    printf("Flugsteuerung: Airmode, Runaway, Motortest, ACC, Konfiguration\n");
    flight_config_t cfg;
    flight_default_config(&cfg);
    flight_t f;
    flight_input_t in;
    memset(&in, 0, sizeof(in));
    in.acc_g[2] = 1.0f;
    in.rc_valid = true;

    /* I-Grenzen wie Betaflight: 80 % von 500 (Roll/Pitch) bzw. 400 (Yaw); Yaw-P mit 100-Hz-Tiefpass */
    flight_init(&f, &cfg);
    CHECK_NEAR(f.pid[AXIS_ROLL].i_limit, 400.0f, 1e-3);
    CHECK_NEAR(f.pid[AXIS_YAW].i_limit, 320.0f, 1e-3);
    CHECK(f.pid[AXIS_YAW].p_lpf_on && !f.pid[AXIS_ROLL].p_lpf_on);
    {
        pid_runtime_t r = { .i_factor = 1, .d_factor = 1, .stabilise = true };
        const float p_yaw = pid_update(&f.pid[AXIS_YAW], 100.0f, 0.0f, &r);   /* erster Schritt gedaempft */
        CHECK(f.pid[AXIS_YAW].p > 0.0f && f.pid[AXIS_YAW].p < 0.5f * f.pid[AXIS_YAW].kp * 100.0f);
        (void)p_yaw;
    }

    /* Gas unten, Airmode noch nicht aktiv: keine Regelung (Motoren nur Leerlauf) */
    flight_init(&f, &cfg);
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MIN);
    run(&f, &in, 1100);
    arm_now(&f, &in, CRSF_CHANNEL_MIN);
    CHECK(f.armed && !f.airmode_active);
    in.gyro_dps[AXIS_ROLL] = 50.0f;
    run(&f, &in, 20);
    CHECK(f.dshot[0] == f.dshot[2]);
    /* nach einmal Gas > 25 % bleibt die Regelung auch bei Gas 0 aktiv (Flips) */
    in.channels[CH_THROTTLE] = 900;     /* ca. 44 % */
    run(&f, &in, 5);
    CHECK(f.airmode_active);
    in.channels[CH_THROTTLE] = CRSF_CHANNEL_MIN;
    run(&f, &in, 20);
    CHECK(f.dshot[0] > f.dshot[2]);
    in.gyro_dps[AXIS_ROLL] = 0.0f;

    /* Runaway-Schutz: vor dem Abheben dreht die Drohne weg, Regler am Anschlag -> disarm */
    flight_init(&f, &cfg);
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MIN);
    run(&f, &in, 1100);
    arm_now(&f, &in, 400);              /* ca. 14 %: Regelung aktiv, aber noch nicht abgehoben */
    in.gyro_dps[AXIS_ROLL] = 500.0f;    /* P allein: 45 * 0.032 * 500 = 720 > 600 */
    run(&f, &in, 300);
    CHECK(!f.armed && f.runaway_triggered);
    in.gyro_dps[AXIS_ROLL] = 0.0f;
    /* ohne Schutz bleibt sie armed */
    cfg.runaway_prevention = 0.0f;
    flight_init(&f, &cfg);
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MIN);
    run(&f, &in, 1100);
    arm_now(&f, &in, 400);
    in.gyro_dps[AXIS_ROLL] = 500.0f;
    run(&f, &in, 300);
    CHECK(f.armed);
    in.gyro_dps[AXIS_ROLL] = 0.0f;
    cfg.runaway_prevention = 1.0f;

    /* Anti-Windup: Bei Saettigung (mix_range > 1) waechst I nicht weiter */
    flight_init(&f, &cfg);
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MIN);
    run(&f, &in, 1100);
    arm_now(&f, &in, 1200);
    cfg.runaway_prevention = 0.0f;
    f.cfg.runaway_prevention = 0.0f;
    f.runaway_armed = false;
    in.gyro_dps[AXIS_ROLL] = 2000.0f;
    run(&f, &in, 50);
    CHECK(f.mix_range >= 1.0f);
    const float i_before = f.pid[AXIS_ROLL].i_term;
    run(&f, &in, 50);
    CHECK_NEAR(f.pid[AXIS_ROLL].i_term, i_before, 1.0);
    in.gyro_dps[AXIS_ROLL] = 0.0f;
    cfg.runaway_prevention = 1.0f;

    /* Motortest: nur disarmed, nur angeforderter Motor, gesperrt bei arm_inhibit */
    flight_init(&f, &cfg);
    set_sticks(&in, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MIN);
    in.motor_test = true;
    in.motor_test_value[0] = 0; in.motor_test_value[1] = 0.2f;
    in.motor_test_value[2] = 0; in.motor_test_value[3] = 0;
    run(&f, &in, 2);
    CHECK(f.dshot[1] > 48 && f.dshot[0] == 0 && f.dshot[2] == 0);
    in.arm_inhibit = true;
    run(&f, &in, 1);
    CHECK(f.dshot[1] == 0);
    in.arm_inhibit = false;
    in.motor_test = false;

    /* ACC-Kalibrierung */
    flight_init(&f, &cfg);
    in.acc_g[0] = 0.02f; in.acc_g[1] = -0.03f; in.acc_g[2] = 1.05f;
    run(&f, &in, 1100);
    flight_start_acc_calibration(&f);
    run(&f, &in, 600);
    CHECK(f.acc_cal_done);
    CHECK_NEAR(f.cfg.acc_bias[0], 0.02, 1e-4);
    CHECK_NEAR(f.cfg.acc_bias[1], -0.03, 1e-4);
    CHECK_NEAR(f.cfg.acc_bias[2], 0.05, 1e-4);
    run(&f, &in, 3000);
    CHECK(fabsf(f.attitude_deg[0]) < 0.5f && fabsf(f.attitude_deg[1]) < 0.5f);
    in.acc_g[0] = 0; in.acc_g[1] = 0; in.acc_g[2] = 1.0f;

    /* Neue Konfiguration uebernehmen: Kalibrierung bleibt, armed -> abgelehnt */
    flight_config_t c2 = cfg;
    c2.pid_p[AXIS_ROLL] = 60;
    CHECK(flight_apply_config(&f, &c2));
    CHECK(f.calibrated);
    CHECK_NEAR(f.pid[AXIS_ROLL].kp, 60 * PID_PTERM_SCALE, 1e-6);
    arm_now(&f, &in, CRSF_CHANNEL_MIN);
    CHECK(f.armed);
    CHECK(!flight_apply_config(&f, &cfg));
}

/* ---------- Einstellungen ---------- */

static void test_params(void)
{
    printf("Einstellungen\n");
    float v;
    CHECK(params_parse_float("12.5", &v) && v == 12.5f);
    CHECK(params_parse_float("-0.25", &v) && v == -0.25f);
    CHECK(params_parse_float(" 7 ", &v) && v == 7.0f);
    CHECK(!params_parse_float("abc", &v));
    CHECK(!params_parse_float("1.2.3", &v));
    CHECK(!params_parse_float("", &v));

    flight_config_t a, b;
    flight_default_config(&a);
    CHECK(params_set(&a, "p_roll", "52"));
    CHECK(a.pid_p[AXIS_ROLL] == 52.0f);
    CHECK(params_set(&a, "gyro_sign_yaw", "-1"));
    CHECK(!params_set(&a, "gyro_sign_yaw", "0.5"));     /* nur +1/-1 */
    CHECK(!params_set(&a, "p_roll", "999"));             /* ausserhalb */
    CHECK(!params_set(&a, "gibts_nicht", "1"));
    CHECK(params_set(&a, "roll_expo", "0.35"));
    CHECK(params_set(&a, "acc_bias_x", "0.0123"));

    char line[64];
    params_format(&a, params_find("p_roll"), line, sizeof(line));
    CHECK(strcmp(line, "p_roll = 52") == 0);

    /* Text speichern und laden ergibt dieselben Werte */
    char buf[4096];
    const int n = params_serialize(&a, buf, sizeof(buf));
    CHECK(n > 0);
    flight_default_config(&b);
    CHECK(params_deserialize(&b, buf, n) == params_count());
    for (int i = 0; i < params_count(); i++) {
        CHECK_NEAR(params_get(&a, i), params_get(&b, i), 1e-4);
    }
    /* unbekannte und kaputte Zeilen werden ignoriert */
    const char *txt = "p_roll=33\nfoo=1\nkaputt\ni_roll=xx\nd_roll=20";
    flight_default_config(&b);
    CHECK(params_deserialize(&b, txt, (int)strlen(txt)) == 2);
    CHECK(b.pid_p[AXIS_ROLL] == 33.0f && b.pid_d[AXIS_ROLL] == 20.0f && b.pid_i[AXIS_ROLL] == 80.0f);
    /* alle Standardwerte liegen im erlaubten Bereich */
    flight_default_config(&b);
    for (int i = 0; i < params_count(); i++) {
        const param_def_t *d = params_def(i);
        const float x = params_get(&b, i);
        if (x < d->min || x > d->max) { printf("  Standardwert ausserhalb: %s\n", d->name); failures++; }
        checks++;
    }
    CHECK(params_crc32("123456789", 9) == 0xCBF43926u);   /* Pruefwert CRC-32 */

    /* Speicherblock: packen, entpacken, Beschaedigung erkennen */
    static uint8_t blob[4096];
    flight_default_config(&a);
    a.pid_d[AXIS_PITCH] = 41;
    const int bl = params_pack(&a, blob, sizeof(blob));
    CHECK(bl > PARAMS_HEADER);
    flight_default_config(&b);
    CHECK(params_unpack(&b, blob, bl) == params_count());
    CHECK(b.pid_d[AXIS_PITCH] == 41.0f);
    blob[PARAMS_HEADER + 5] ^= 0x20;
    flight_default_config(&b);
    CHECK(params_unpack(&b, blob, bl) == -1);
    CHECK(b.pid_d[AXIS_PITCH] == 34.0f);
    memset(blob, 0xFF, 64);                          /* geloeschter Flash */
    CHECK(params_unpack(&b, blob, 64) == -1);
}

/* ---------- Logformat ---------- */

static void test_logrec(const char *sample_path)
{
    printf("Logformat\n");
    logrec_data_t d;
    memset(&d, 0, sizeof(d));
    d.t_us = 0x01020304u;
    d.flags = LOGREC_FLAG_ARMED | LOGREC_FLAG_ANGLE;
    d.setpoint_dps[0] = 123.45f; d.setpoint_dps[1] = -200.0f; d.setpoint_dps[2] = 5000.0f;
    d.gyro_dps[0] = -0.04f; d.gyro_dps[1] = 1.0f; d.gyro_dps[2] = -3276.8f;
    d.motor[0] = 48; d.motor[1] = 1000; d.motor[2] = 2047; d.motor[3] = 0;
    d.throttle = 0.4321f; d.vbat_v = 15.12f; d.link_quality = 99;

    uint8_t r[LOGREC_SIZE];
    logrec_encode_data(&d, r);
    CHECK(r[0] == LOGREC_TYPE_DATA);
    CHECK(r[1] == 0x05);
    CHECK(r[2] == 0x04 && r[5] == 0x01);
    CHECK((int16_t)(r[6] | (r[7] << 8)) == 1235);     /* 123.45 dps -> 1235 */
    CHECK((int16_t)(r[8] | (r[9] << 8)) == -2000);
    CHECK((int16_t)(r[10] | (r[11] << 8)) == 32767);  /* begrenzt */
    CHECK((int16_t)(r[12] | (r[13] << 8)) == 0);
    CHECK((int16_t)(r[16] | (r[17] << 8)) == -32768);
    CHECK((r[22] | (r[23] << 8)) == 2047);
    CHECK((r[26] | (r[27] << 8)) == 4321);
    CHECK((r[28] | (r[29] << 8)) == 15120);
    CHECK(r[30] == 99);
    CHECK(logrec_crc_ok(r));
    r[10] ^= 1;
    CHECK(!logrec_crc_ok(r));

    logrec_header_t h = { .git_hash = 0xDEADBEEFu, .loop_hz = 1000, .log_hz = 1000,
        .pid = { {45, 80, 30}, {47, 84, 34}, {45, 80, 0} },
        .max_rate_dps = 400, .gyro_lpf_hz = 150, .dterm_lpf_hz = 75,
        .ff = { 120, 125, 0 }, .flags = 0x05 };
    logrec_encode_header(&h, r);
    CHECK(r[0] == LOGREC_TYPE_HEADER && r[1] == LOGREC_VERSION);
    CHECK(r[2] == 0xEF && r[5] == 0xDE);
    CHECK(r[10] == 45 && r[13] == 47 && r[18] == 0);
    CHECK(r[23] == 120 && r[25] == 125 && r[29] == 0x05);
    CHECK(logrec_crc_ok(r));

    /* Beispieldatei fuer den Python-Test (Kopf + 20 Datensaetze + Fuellung) */
    if (sample_path) {
        FILE *fp = fopen(sample_path, "wb");
        if (fp) {
            logrec_encode_header(&h, r);
            fwrite(r, 1, LOGREC_SIZE, fp);
            for (int i = 0; i < 20; i++) {
                d.t_us = 1000000u + (uint32_t)i * 1000u;
                d.setpoint_dps[0] = (float)i * 10.0f;
                d.gyro_dps[0] = (float)i * 9.5f;
                d.setpoint_dps[2] = 0.0f; d.gyro_dps[2] = 0.0f;
                logrec_encode_data(&d, r);
                fwrite(r, 1, LOGREC_SIZE, fp);
            }
            logrec_encode_pad(r);
            for (int i = 0; i < 3; i++) fwrite(r, 1, LOGREC_SIZE, fp);
            fclose(fp);
        }
    }
}

/* ---------- Blackbox mit simuliertem Flash ---------- */

#define SIM_CAP (4u * BB_BLOCK_SIZE)
static uint8_t sim[SIM_CAP];
static int sim_programs, sim_bad_program;

static bool sim_read(uint32_t a, uint8_t *b, uint32_t n) { memcpy(b, &sim[a], n); return a + n <= SIM_CAP; }
static bool sim_prog(uint32_t a, const uint8_t *b)
{
    sim_programs++;
    for (uint32_t i = 0; i < BB_PAGE_SIZE; i++) {
        if ((sim[a + i] & b[i]) != b[i]) sim_bad_program++;  /* NOR: nur 1 -> 0 */
        sim[a + i] &= b[i];
    }
    return true;
}
static bool sim_erase_block(uint32_t a) { memset(&sim[a], 0xFF, BB_BLOCK_SIZE); return true; }
static bool sim_erase_chip(void) { memset(sim, 0xFF, SIM_CAP); return true; }
static const bb_flash_ops_t sim_ops = { sim_read, sim_prog, sim_erase_block, sim_erase_chip, SIM_CAP };

static void test_blackbox(void)
{
    printf("Blackbox\n");
    blackbox_t bb;
    uint8_t rec[LOGREC_SIZE];
    logrec_data_t d;
    memset(&d, 0, sizeof(d));

    memset(sim, 0xFF, sizeof(sim));
    CHECK(bb_init(&bb, &sim_ops));
    CHECK(bb_used_bytes(&bb) == 0 && !bb.full);

    /* 20 Datensaetze = 2.5 Seiten; nach flush 3 Seiten */
    for (int i = 0; i < 20; i++) { d.t_us = (uint32_t)i; logrec_encode_data(&d, rec); bb_append(&bb, rec); }
    CHECK(bb_used_bytes(&bb) == 2 * BB_PAGE_SIZE);
    bb_flush(&bb);
    CHECK(bb_used_bytes(&bb) == 3 * BB_PAGE_SIZE);
    CHECK(sim[2 * BB_PAGE_SIZE + 4 * LOGREC_SIZE] == LOGREC_TYPE_PAD);
    bb_flush(&bb);                                   /* leerer Puffer: nichts schreiben */
    CHECK(bb_used_bytes(&bb) == 3 * BB_PAGE_SIZE);

    /* Neustart: Ende wird wiedergefunden */
    blackbox_t bb2;
    CHECK(bb_init(&bb2, &sim_ops));
    CHECK(bb_used_bytes(&bb2) == 3 * BB_PAGE_SIZE);

    /* teilweise beschriebene Seite (Stromausfall) wird uebersprungen */
    sim[3 * BB_PAGE_SIZE + 100] = 0x12;
    CHECK(bb_init(&bb2, &sim_ops));
    CHECK(bb_used_bytes(&bb2) == 4 * BB_PAGE_SIZE);

    /* Flash fuellen: danach voll, weitere Saetze werden verworfen */
    for (uint32_t i = 0; i < SIM_CAP / LOGREC_SIZE; i++) { logrec_encode_data(&d, rec); bb_append(&bb2, rec); }
    CHECK(bb2.full);
    CHECK(bb2.records_dropped > 0);
    CHECK(bb_used_bytes(&bb2) <= SIM_CAP);
    CHECK(sim_bad_program == 0);

    /* Loeschen des benutzten Bereichs */
    CHECK(bb_erase_used(&bb2, NULL));
    CHECK(bb_used_bytes(&bb2) == 0 && !bb2.full);
    CHECK(bb_init(&bb2, &sim_ops));
    CHECK(bb_used_bytes(&bb2) == 0);

    /* Voller Flash wird beim Start erkannt */
    memset(sim, 0xA5, sizeof(sim));
    CHECK(bb_init(&bb2, &sim_ops));
    CHECK(bb2.full);
}

int main(int argc, char **argv)
{
    test_crsf();
    test_dshot();
    test_pid();
    test_filters_rates();
    test_mixer();
    test_mahony();
    test_flight();
    test_flight_features();
    test_params();
    test_logrec(argc > 1 ? argv[1] : NULL);
    test_blackbox();
    printf("\n%d Pruefungen, %d Fehler\n", checks, failures);
    return failures ? 1 : 0;
}
