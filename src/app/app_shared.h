/*
 * app_shared.h - Zustand, den mehrere Tasks lesen (Regelung, Logging, USB, Status)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "blackbox.h"
#include "flight.h"

#include "fw_version.h"   /* FW_GIT_HASH, beim Build erzeugt */

/* Flugsteuerung: wird nur vom Regel-Task geschrieben, andere Tasks lesen
 * fuer Anzeigezwecke (kleine Inkonsistenzen sind dort unkritisch). */
extern flight_t           g_flight;
extern volatile uint16_t  g_rc_channels[CRSF_NUM_CHANNELS];
extern volatile uint8_t   g_link_quality;
extern volatile uint32_t  g_rc_frames, g_rc_crc_errors;
extern volatile float     g_vbat_volts;
extern volatile bool      g_gyro_ok;
extern volatile uint32_t  g_loop_overruns;
extern volatile uint32_t  g_log_queue_drops;

/* Blackbox: Zugriff nur mit g_flash_mutex */
extern blackbox_t         g_blackbox;
extern SemaphoreHandle_t  g_flash_mutex;

/* true, solange ueber USB geloescht oder heruntergeladen wird -> nicht armen */
extern volatile bool      g_cli_busy;

/* Neue Einstellungen fuer den Regel-Task (wird nur disarmed uebernommen) */
extern flight_config_t    g_cfg_new;
extern volatile bool      g_cfg_new_pending;
extern volatile bool      g_cfg_from_flash;        /* beim Start gespeicherte Werte geladen */

/* Motortest (nur disarmed, ohne Propeller) */
extern volatile float     g_motor_test_value[4];
extern volatile uint32_t  g_motor_test_until_ms;
extern volatile bool      g_acc_cal_request;

/* Zeitmessung der Regelschleife [us] */
extern volatile uint32_t  g_loop_us_max, g_loop_us_avg_x10;
extern volatile uint32_t  g_cycle_us_min, g_cycle_us_max;

void     usb_cli_start(void);
uint32_t config_store_addr(void);
int      config_store_load(flight_config_t *cfg);
bool     config_store_save(const flight_config_t *cfg);
