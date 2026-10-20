/*
 * main.c - Eigene Flugsteuerung fuer den GEPRC TAKER F722 (Build B)
 *
 * Aufteilung in FreeRTOS-Tasks (Begleitarbeit Kap. 2.4, Tabelle 6):
 *
 *   Task       Prioritaet  Takt                Aufgabe
 *   control    4 (hoch)    Gyro Data-Ready     IMU lesen, Lage, PID, Mischer, DShot
 *   rx         3           1 kHz               CRSF-Bytes auswerten, Failsafe-Zeit
 *   log        2           Queue               Flugdaten in den 16-MB-Flash (nur armed)
 *   usbd       2           USB-Ereignisse      TinyUSB
 *   cli        1           Eingabe             Befehle ueber USB (info, dump, erase, ...)
 *   status     1 (tief)    10 Hz               LEDs, Summer, Akkuspannung
 *
 * SICHERHEIT: Erste Tests nur OHNE Propeller. Siehe docs/bodentest.md.
 */
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "board.h"
#include "dshot_tim3.h"
#include "flash_w25q.h"
#include "icm42688.h"
#include "peripherals.h"
#include "rx_uart.h"
#include "system.h"

#include "app_shared.h"
#include "blackbox.h"
#include "crsf.h"
#include "flight.h"
#include "logrec.h"

/* ---------- Einstellungen ---------- */

#define LOOP_ODR            ICM_ODR_1K
#define LOOP_HZ             1000u
#define LOG_DIVIDER         1u       /* 1 = jeder Zyklus (1 kHz), 2 = 500 Hz ... */
#define DSHOT_RATE          DSHOT300
#define RC_TIMEOUT_MS       250u     /* laenger keine Pakete -> Failsafe */
#define GYRO_TIMEOUT_MS     5u       /* kein Data-Ready -> Motoren aus */
#define WATCHDOG_MS         100u
#define VBAT_WARN_PER_CELL  3.5f

/* ---------- gemeinsame Daten (app_shared.h) ---------- */

flight_t           g_flight;
volatile uint16_t  g_rc_channels[CRSF_NUM_CHANNELS];
volatile uint8_t   g_link_quality;
volatile uint32_t  g_rc_frames, g_rc_crc_errors;
volatile float     g_vbat_volts;
volatile bool      g_gyro_ok;
volatile uint32_t  g_loop_overruns;
volatile uint32_t  g_log_queue_drops;
blackbox_t         g_blackbox;
SemaphoreHandle_t  g_flash_mutex;
volatile bool      g_cli_busy;

/* ---------- lokale Daten ---------- */

typedef struct {
    uint32_t t_us;
    float    setpoint[3];
    float    gyro[3];
    uint16_t motor[4];
    float    throttle;
    uint8_t  flags;
} log_sample_t;

typedef struct {
    uint16_t   channels[CRSF_NUM_CHANNELS];
    TickType_t last_frame_tick;
    bool       ever_received;
} rc_shared_t;

static rc_shared_t   rc_shared;
static TaskHandle_t  control_task_handle;
static QueueHandle_t log_queue;
static volatile bool beep_request;

/* ---------- Data-Ready-Interrupt -> Regel-Task wecken ---------- */

static void on_gyro_drdy(void)
{
    BaseType_t woken = pdFALSE;
    if (control_task_handle) {
        vTaskNotifyGiveFromISR(control_task_handle, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

/* ---------- Regel-Task ---------- */

static void control_task(void *arg)
{
    (void)arg;
    flight_config_t cfg;
    flight_default_config(&cfg);
    cfg.loop_hz = (float)LOOP_HZ;
    /* Vorzeichen-Korrekturen nach dem Bodentest hier eintragen, z. B.:
     * cfg.gyro_sign[AXIS_YAW] = -1.0f; */
    flight_init(&g_flight, &cfg);

    flight_input_t in;
    memset(&in, 0, sizeof(in));
    imu_sample_t imu;
    uint32_t log_count = 0;

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(GYRO_TIMEOUT_MS)) == 0) {
            /* Gyro meldet sich nicht: Motoren aus, nicht weiterregeln */
            g_gyro_ok = false;
            flight_force_disarm(&g_flight);
            dshot_write(g_flight.dshot);
            if (g_flight.event_disarmed) beep_request = true;
            continue;
        }
        g_gyro_ok = true;
        const uint32_t t0 = micros();

        icm42688_read(&imu);
        memcpy(in.gyro_dps, imu.gyro_dps, sizeof(in.gyro_dps));
        memcpy(in.acc_g, imu.acc_g, sizeof(in.acc_g));

        taskENTER_CRITICAL();
        memcpy(in.channels, rc_shared.channels, sizeof(in.channels));
        const TickType_t age = xTaskGetTickCount() - rc_shared.last_frame_tick;
        in.rc_valid = rc_shared.ever_received && age < pdMS_TO_TICKS(RC_TIMEOUT_MS);
        taskEXIT_CRITICAL();
        in.arm_inhibit = g_cli_busy;

        flight_step(&g_flight, &in);
        if (!dshot_write(g_flight.dshot)) {
            g_loop_overruns++;
        }

        if (g_flight.event_armed || g_flight.event_disarmed || g_flight.event_failsafe) {
            beep_request = true;
        }

        /* Messdaten fuers Logging (nicht blockierend) */
        if (++log_count >= LOG_DIVIDER) {
            log_count = 0;
            log_sample_t s;
            s.t_us = t0;
            s.throttle = g_flight.throttle;
            for (int a = 0; a < 3; a++) {
                s.setpoint[a] = g_flight.setpoint[a];
                s.gyro[a] = g_flight.gyro_f[a];
            }
            memcpy(s.motor, g_flight.dshot, sizeof(s.motor));
            s.flags = (uint8_t)((g_flight.armed ? LOGREC_FLAG_ARMED : 0u) |
                                (g_flight.failsafe ? LOGREC_FLAG_FAILSAFE : 0u) |
                                (g_flight.mode == MODE_ANGLE ? LOGREC_FLAG_ANGLE : 0u));
            if (xQueueSend(log_queue, &s, 0) != pdTRUE) {
                g_log_queue_drops++;
            }
        }

        watchdog_kick();
    }
}

/* ---------- Empfaenger-Task ---------- */

static void rx_task(void *arg)
{
    (void)arg;
    crsf_parser_t parser;
    crsf_parser_init(&parser);
    uint16_t ch[CRSF_NUM_CHANNELS];
    crsf_link_stats_t stats;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        uint8_t b;
        while (rx_uart_read(&b)) {
            const crsf_result_t r = crsf_parser_feed(&parser, b, ch, &stats);
            if (r == CRSF_RC_CHANNELS) {
                taskENTER_CRITICAL();
                memcpy(rc_shared.channels, ch, sizeof(ch));
                rc_shared.last_frame_tick = xTaskGetTickCount();
                rc_shared.ever_received = true;
                taskEXIT_CRITICAL();
                for (unsigned i = 0; i < CRSF_NUM_CHANNELS; i++) g_rc_channels[i] = ch[i];
            } else if (r == CRSF_LINK_STATS) {
                g_link_quality = stats.uplink_link_quality;
            }
        }
        g_rc_frames = parser.frames_ok;
        g_rc_crc_errors = parser.crc_errors;
        vTaskDelayUntil(&wake, 1);
    }
}

/* ---------- Logging-Task: Flugdaten in den Flash ---------- */

static bool ops_read(uint32_t a, uint8_t *b, uint32_t n) { return flash_read(a, b, n); }
static bool ops_prog(uint32_t a, const uint8_t *b) { return flash_program_page(a, b); }
static bool ops_erase_block(uint32_t a) { return flash_erase_block64k(a); }
static bool ops_erase_chip(void) { return flash_erase_chip(); }
static bb_flash_ops_t flash_ops = { ops_read, ops_prog, ops_erase_block, ops_erase_chip, 0 };

static void write_header(void)
{
    const flight_config_t *c = &g_flight.cfg;
    logrec_header_t h;
    memset(&h, 0, sizeof(h));
    h.git_hash = FW_GIT_HASH;
    h.loop_hz = (uint16_t)LOOP_HZ;
    h.log_hz = (uint16_t)(LOOP_HZ / LOG_DIVIDER);
    for (int a = 0; a < 3; a++) {
        h.pid[a][0] = c->pid_p[a];
        h.pid[a][1] = c->pid_i[a];
        h.pid[a][2] = c->pid_d[a];
    }
    h.max_rate_dps = (uint16_t)c->max_rate_dps[AXIS_ROLL];
    h.gyro_lpf_hz = (uint8_t)c->gyro_lpf_hz;
    h.dterm_lpf_hz = (uint8_t)c->dterm_lpf_hz;
    uint8_t rec[LOGREC_SIZE];
    logrec_encode_header(&h, rec);
    bb_append(&g_blackbox, rec);
}

static void log_task(void *arg)
{
    (void)arg;
    log_sample_t s;
    bool was_armed = false;
    uint8_t rec[LOGREC_SIZE];

    for (;;) {
        if (xQueueReceive(log_queue, &s, portMAX_DELAY) != pdTRUE) continue;
        const bool armed = (s.flags & LOGREC_FLAG_ARMED) != 0;
        if (!armed && !was_armed) continue;      /* am Boden nichts aufzeichnen */

        logrec_data_t d;
        d.t_us = s.t_us;
        memcpy(d.setpoint_dps, s.setpoint, sizeof(d.setpoint_dps));
        memcpy(d.gyro_dps, s.gyro, sizeof(d.gyro_dps));
        memcpy(d.motor, s.motor, sizeof(d.motor));
        d.throttle = s.throttle;
        d.vbat_v = g_vbat_volts;
        d.link_quality = g_link_quality;
        d.flags = s.flags;
        logrec_encode_data(&d, rec);

        xSemaphoreTake(g_flash_mutex, portMAX_DELAY);
        if (armed && !was_armed) write_header();  /* Beginn eines Flugs */
        bb_append(&g_blackbox, rec);
        if (!armed) bb_flush(&g_blackbox);       /* Ende eines Flugs: Seite abschliessen */
        xSemaphoreGive(g_flash_mutex);
        was_armed = armed;
    }
}

/* ---------- Status-Task ---------- */

static void beep(uint32_t ms)
{
    beeper_set(true);
    vTaskDelay(pdMS_TO_TICKS(ms));
    beeper_set(false);
}

static void beeps(int n)
{
    for (int i = 0; i < n; i++) {
        beep(60);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

static void status_task(void *arg)
{
    (void)arg;
    vbat_init();
    if (!system_clock_is_hse()) beeps(2);    /* Quarz nicht angelaufen */
    if (!g_blackbox.ok) beeps(4);            /* kein Flash gefunden */

    uint32_t tick = 0;
    bool full_warned = false;
    for (;;) {
        tick++;
        g_vbat_volts = vbat_read_volts();

        /* LED 0: blinkt langsam = bereit, schnell = nicht kalibriert / kein Empfaenger,
         * dauernd an = armed */
        bool led;
        if (g_flight.armed) led = true;
        else if (!g_flight.calibrated || g_flight.failsafe || !g_gyro_ok) led = (tick % 2) == 0;
        else led = (tick % 10) < 5;
        led_set(0, led);
        led_set(1, g_flight.mode == MODE_ANGLE);

        if (beep_request) {
            beep_request = false;
            beep(80);
        }
        /* Logspeicher voll: beim Armen dreimal piepsen */
        if (g_flight.armed && g_blackbox.full && !full_warned) {
            beeps(3);
            full_warned = true;
        }
        if (!g_flight.armed) full_warned = false;

        /* Akkuwarnung (Zellenzahl grob aus der Spannung geschaetzt) */
        const float v = g_vbat_volts;
        const int cells = (v > 4.4f * 5) ? 6 : (v > 4.4f * 4) ? 5 : (v > 4.4f * 3) ? 4 : 3;
        if (v > 5.0f && v < VBAT_WARN_PER_CELL * (float)cells && (tick % 20) == 0) {
            beep(200);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------- Fehlerbehandlung: Motoren immer zuerst aus ---------- */

void fc_fatal_error(const char *file, int line)
{
    (void)file; (void)line;
    __disable_irq();
    dshot_emergency_stop();
    for (;;) {
        beeper_set(true);
        delay_ms(50);
        beeper_set(false);
        delay_ms(450);
    }
}

void HardFault_Handler(void)       { fc_fatal_error("HardFault", 0); }
void MemManage_Handler(void)       { fc_fatal_error("MemManage", 0); }
void BusFault_Handler(void)        { fc_fatal_error("BusFault", 0); }
void UsageFault_Handler(void)      { fc_fatal_error("UsageFault", 0); }
void vApplicationMallocFailedHook(void) { fc_fatal_error("malloc", 0); }
void vApplicationStackOverflowHook(TaskHandle_t t, char *name)
{
    (void)t; (void)name;
    fc_fatal_error("stack", 0);
}

/* ---------- Start ---------- */

int main(void)
{
    system_clock_init();
    system_timer_init();
    NVIC_SetPriorityGrouping(0);   /* alle Bits fuer Praeemption (FreeRTOS-Vorgabe) */

    indicators_init();
    beeper_set(true);
    delay_ms(100);
    beeper_set(false);

    /* Motoren zuerst: der ESC soll sofort "0" sehen */
    dshot_init(DSHOT_RATE);
    rx_uart_init(CRSF_BAUDRATE);

    /* Blackbox-Flash: Ende der bisherigen Daten suchen (wenige ms) */
    if (flash_init()) {
        flash_ops.capacity = flash_capacity();
        bb_init(&g_blackbox, &flash_ops);
    }

    icm42688_drdy_callback = on_gyro_drdy;
    if (!icm42688_init(LOOP_ODR)) {
        fc_fatal_error("gyro not found", 0);
    }

    log_queue = xQueueCreate(64, sizeof(log_sample_t));
    g_flash_mutex = xSemaphoreCreateMutex();
    configASSERT(log_queue != NULL && g_flash_mutex != NULL);

    xTaskCreate(control_task, "control", 1024, NULL, 4, &control_task_handle);
    xTaskCreate(rx_task,      "rx",      512,  NULL, 3, NULL);
    xTaskCreate(log_task,     "log",     768,  NULL, 2, NULL);
    xTaskCreate(status_task,  "status",  512,  NULL, 1, NULL);
    usb_cli_start();

    watchdog_init(WATCHDOG_MS);
    vTaskStartScheduler();
    fc_fatal_error("scheduler", 0);
    return 0;
}
