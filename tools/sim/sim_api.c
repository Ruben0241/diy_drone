/*
 * sim_api.c - Schnittstelle zwischen dem Firmware-Kern und der Python-Simulation
 *
 * Wird als gemeinsame Bibliothek (libfc_core.so) gebaut. Python ruft fc_step()
 * im Takt der Regelschleife auf und uebergibt die simulierten Sensorwerte.
 * Es laeuft derselbe Code wie auf dem Flightcontroller (src/core).
 */
#include <string.h>
#include "dshot.h"
#include "flight.h"
#include "params.h"

static flight_config_t cfg;
static flight_t fl;

void fc_init(void)
{
    flight_default_config(&cfg);
    flight_init(&fl, &cfg);
}

/* Einstellung setzen (vor fc_reset), Rueckgabe 1 = ok */
int fc_set(const char *name, const char *value)
{
    return params_set(&cfg, name, value) ? 1 : 0;
}

void fc_reset(void)
{
    flight_init(&fl, &cfg);
}

/* Ein Regelzyklus. motors: Motorbefehl 0..1 wie er per DShot zum ESC geht.
 * Rueckgabe: Bit0 armed, Bit1 Runaway ausgeloest, Bit2 Airmode aktiv, Bit3 kalibriert */
int fc_step(const float gyro_dps[3], const float acc_g[3], const unsigned short ch[16], int rc_valid,
            float motors[4], float setpoint[3], float attitude[3], float pid_sum[3])
{
    flight_input_t in;
    memset(&in, 0, sizeof(in));
    for (int i = 0; i < 3; i++) { in.gyro_dps[i] = gyro_dps[i]; in.acc_g[i] = acc_g[i]; }
    for (int i = 0; i < 16; i++) in.channels[i] = ch[i];
    in.rc_valid = rc_valid != 0;
    flight_step(&fl, &in);
    for (int m = 0; m < 4; m++) {
        const uint16_t d = fl.dshot[m];
        motors[m] = (d < DSHOT_MIN_THROTTLE) ? 0.0f
                  : (float)(d - DSHOT_MIN_THROTTLE) / (float)(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE);
    }
    for (int a = 0; a < 3; a++) {
        setpoint[a] = fl.setpoint[a];
        attitude[a] = fl.attitude_deg[a];
        pid_sum[a] = fl.pid[a].out;
    }
    return (fl.armed ? 1 : 0) | (fl.runaway_triggered ? 2 : 0) | (fl.airmode_active ? 4 : 0) |
           (fl.calibrated ? 8 : 0);
}

float fc_gyro_filtered(int axis) { return fl.gyro_f[axis]; }
