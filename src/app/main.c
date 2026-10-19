/*
 * main.c - Eigene Flugsteuerung fuer den GEPRC TAKER F722 (Build B)
 *
 * Aufteilung in FreeRTOS-Tasks (Begleitarbeit Kap. 2.4, Tabelle 6):
 *
 *   Task       Prioritaet  Takt                Aufgabe
 *   control    4 (hoch)    Gyro Data-Ready     IMU lesen, Lage, PID, Mischer, DShot
 *   rx         3           1 kHz               CRSF-Bytes auswerten, Failsafe-Zeit
 *   log        2           Queue               Messdaten sammeln (Flash folgt)
 *   status     1 (tief)    10 Hz               LEDs, Summer, Akkuspannung
 *
 * SICHERHEIT: Erste Tests nur OHNE Propeller. Siehe README.
 */
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "board.h"
#include "dshot_tim3.h"
#include "icm42688.h"
#include "peripherals.h"
#include "rx_uart.h"
#include "system.h"

#include "crsf.h"
#include "flight.h"

/* ---------- Einstellungen ---------- */

#define LOOP_ODR            ICM_ODR_1K
#define LOOP_HZ             1000.0f
#define DSHOT_RATE          DSHOT300
#define RC_TIMEOUT_MS       250u     /* laenger keine Pakete -> Failsafe */
#define GYRO_TIMEOUT_MS     5u       /* kein Data-Ready -> Motoren aus */
#define WATCHDOG_MS         100u
#define VBAT_WARN_PER_CELL  3.5f

/* ---------- gemeinsame Daten ---------- */

typedef struct {
    uint16_t   channels[CRSF_NUM_CHANNELS];
    TickType_t last_frame_tick;
    bool       ever_received;
} rc_shared_t;

typedef struct {
    uint32_t t_us;
    float    setpoint[3];
    float    gyro[3];
    float    pid_p[3], pid_i[3], pid_d[3];
    uint16_t motor[4];
    float    throttle;
    uint8_t  flags;      /* Bit0 armed, Bit1 failsafe, Bit2 angle */
} log_sample_t;

static rc_shared_t   rc_shared;
static flight_t      flight;
static TaskHandle_t  control_task_handle;
static QueueHandle_t log_queue;
static volatile bool gyro_ok;
static volatile bool beep_request;
static volatile uint32_t loop_overruns;
static volatile float vbat_volts;

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
    cfg.loop_hz = LOOP_HZ;
    /* Vorzeichen-Korrekturen nach dem Bodentest hier eintragen, z. B.:
     * cfg.gyro_sign[AXIS_YAW] = -1.0f; */
    flight_init(&flight, &cfg);

    flight_input_t in;
    memset(&in, 0, sizeof(in));
    imu_sample_t imu;
    const uint16_t motors_off[4] = { 0, 0, 0, 0 };

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(GYRO_TIMEOUT_MS)) == 0) {
            /* Gyro meldet sich nicht: Motoren aus, nicht weiterregeln */
            gyro_ok = false;
            flight.armed = false;
            dshot_write(motors_off);
            continue;
        }
        gyro_ok = true;
        const uint32_t t0 = micros();

        icm42688_read(&imu);
        memcpy(in.gyro_dps, imu.gyro_dps, sizeof(in.gyro_dps));
        memcpy(in.acc_g, imu.acc_g, sizeof(in.acc_g));

        taskENTER_CRITICAL();
        memcpy(in.channels, rc_shared.channels, sizeof(in.channels));
        const TickType_t age = xTaskGetTickCount() - rc_shared.last_frame_tick;
        in.rc_valid = rc_shared.ever_received && age < pdMS_TO_TICKS(RC_TIMEOUT_MS);
        taskEXIT_CRITICAL();

        flight_step(&flight, &in);
        if (!dshot_write(flight.dshot)) {
            loop_overruns++;
        }

        if (flight.event_armed || flight.event_disarmed || flight.event_failsafe) {
            beep_request = true;
        }

        /* Messdaten fuers Logging (nicht blockierend) */
        log_sample_t s;
        s.t_us = t0;
        s.throttle = flight.throttle;
        for (int a = 0; a < 3; a++) {
            s.setpoint[a] = flight.setpoint[a];
            s.gyro[a] = flight.gyro_f[a];
            s.pid_p[a] = flight.pid[a].p;
            s.pid_i[a] = flight.pid[a].i;
            s.pid_d[a] = flight.pid[a].d;
        }
        memcpy(s.motor, flight.dshot, sizeof(s.motor));
        s.flags = (uint8_t)((flight.armed ? 1u : 0u) | (flight.failsafe ? 2u : 0u) |
                            (flight.mode == MODE_ANGLE ? 4u : 0u));
        (void)xQueueSend(log_queue, &s, 0);

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
            if (crsf_parser_feed(&parser, b, ch, &stats) == CRSF_RC_CHANNELS) {
                taskENTER_CRITICAL();
                memcpy(rc_shared.channels, ch, sizeof(ch));
                rc_shared.last_frame_tick = xTaskGetTickCount();
                rc_shared.ever_received = true;
                taskEXIT_CRITICAL();
            }
        }
        vTaskDelayUntil(&wake, 1);
    }
}

/* ---------- Logging-Task ---------- */

static void log_task(void *arg)
{
    (void)arg;
    log_sample_t s;
    uint32_t count = 0;
    for (;;) {
        if (xQueueReceive(log_queue, &s, portMAX_DELAY) == pdTRUE) {
            count++;
            /* TODO: Datensaetze in den W25Q128-Flash (SPI2) schreiben und
             * spaeter als CSV auslesen (Kap. 3.4.2). */
        }
    }
}

/* ---------- Status-Task ---------- */

static void beep(uint32_t ms)
{
    beeper_set(true);
    vTaskDelay(pdMS_TO_TICKS(ms));
    beeper_set(false);
}

static void status_task(void *arg)
{
    (void)arg;
    vbat_init();
    uint32_t tick = 0;
    for (;;) {
        tick++;
        vbat_volts = vbat_read_volts();

        /* LED 0: blinkt langsam = bereit, schnell = nicht kalibriert / kein Empfaenger,
         * dauernd an = armed */
        bool led;
        if (flight.armed) led = true;
        else if (!flight.calibrated || flight.failsafe || !gyro_ok) led = (tick % 2) == 0;
        else led = (tick % 10) < 5;
        led_set(0, led);
        led_set(1, flight.mode == MODE_ANGLE);

        if (beep_request) {
            beep_request = false;
            beep(80);
        }
        /* Akkuwarnung (Zellenzahl grob aus der Spannung geschaetzt) */
        const float v = vbat_volts;
        const int cells = (v > 4.4f * 5) ? 6 : (v > 4.4f * 4) ? 5 : (v > 4.4f * 3) ? 4 : 3;
        if (v > 5.0f && v < VBAT_WARN_PER_CELL * cells && (tick % 20) == 0) {
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

    icm42688_drdy_callback = on_gyro_drdy;
    if (!icm42688_init(LOOP_ODR)) {
        fc_fatal_error("gyro not found", 0);
    }

    log_queue = xQueueCreate(64, sizeof(log_sample_t));
    configASSERT(log_queue != NULL);

    xTaskCreate(control_task, "control", 1024, NULL, 4, &control_task_handle);
    xTaskCreate(rx_task,      "rx",      512,  NULL, 3, NULL);
    xTaskCreate(log_task,     "log",     512,  NULL, 2, NULL);
    xTaskCreate(status_task,  "status",  512,  NULL, 1, NULL);

    watchdog_init(WATCHDOG_MS);
    vTaskStartScheduler();
    fc_fatal_error("scheduler", 0);
    return 0;
}
