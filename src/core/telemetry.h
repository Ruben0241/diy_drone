/*
 * telemetry.h - Live-Daten fuer den Configurator (USB, Befehl "stream <hz>")
 *
 * Zuerst eine Kopfzeile mit den Feldnamen, danach pro Abtastung eine Zeile:
 *   $H,t_ms,armed,failsafe,...      (Feldnamen, TLM_FIELDS)
 *   $T,123456,0,0,...               (ganze Zahlen, Einheiten siehe unten)
 * Der Configurator liest die Felder ueber die Namen, neue Felder koennen also
 * angehaengt werden, ohne dass alte Versionen falsch lesen.
 *
 * Einheiten: Lage, Gyro, Sollwert in 0.1 Grad bzw. 0.1 Grad/s, Beschleunigung in mg,
 * Gas 0..1000, Motoren als DShot-Wert (0 = aus, 48..2047), Kanaele roh (CRSF 172..1811),
 * Akku in mV, Rechenzeit in 0.1 us.
 */
#pragma once

#include <stdint.h>

#define TLM_FIELDS "t_ms,armed,failsafe,mode,airmode,calibrated,motor_test,arm_block," \
    "att_r,att_p,att_y,gyro_r,gyro_p,gyro_y,acc_x,acc_y,acc_z,sp_r,sp_p,sp_y,thr," \
    "m1,m2,m3,m4,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,lq,vbat_mv,loop_avg_x10,loop_max_us," \
    "rx_uart,rx_frames,rx_crc_err,out1,out2,out3,out4"

typedef struct {
    uint32_t t_ms;
    uint8_t  armed, failsafe, mode, airmode, calibrated, motor_test;
    uint16_t arm_block;
    float    att_deg[3], gyro_dps[3], acc_g[3], setpoint_dps[3];
    float    throttle;              /* 0 ... 1 */
    uint16_t motor[4];              /* M1..M4 (logisch, wie im Log) */
    uint16_t rc[8];
    uint8_t  link_quality;
    float    vbat_v;
    uint32_t loop_avg_x10, loop_max_us;
    uint8_t  rx_uart;               /* 0 = noch keiner gefunden */
    uint32_t rx_frames, rx_crc_errors;
    uint16_t out[4];                /* Ausgaenge 1..4 nach motorN_out */
} tlm_t;

/* Kopfzeile "$H,..." bzw. Datenzeile "$T,...", jeweils ohne Zeilenende. Rueckgabe Laenge. */
int tlm_format_header(char *buf, int cap);
int tlm_format(const tlm_t *t, char *buf, int cap);

/* Akku: Zellenzahl grob aus der Spannung (LiPo 3.3 ... 4.4 V pro Zelle), 0 = kein Akku (< 5 V) */
int   tlm_battery_cells(float volts);
/* Rest in % linear zwischen 3.3 V und 4.2 V pro Zelle (unter Last nur grob) */
int   tlm_battery_percent(float volts);
int   tlm_battery_percent_cells(float volts, int cells);
