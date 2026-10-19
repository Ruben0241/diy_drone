/*
 * mahony.h - Lageschaetzung mit dem Komplementaerfilter nach Mahony et al. (2008)
 *
 * Koordinaten (Koerpersystem, wie Betaflight nach der Sensor-Ausrichtung):
 *   x nach vorne, y nach links, z nach oben (rechtshaendig)
 *   positive Rollrate  = rechte Seite nach unten
 *   positive Nickrate  = Nase nach unten
 * Der Beschleunigungssensor misst im Stillstand +1 g auf z.
 *
 * Gleichungen (Begleitarbeit Kap. 2.3, Gl. 2.4 und 2.5):
 *   e      = a x v_hat
 *   w_korr = w_gyro + Kp*e + Ki*integral(e)
 */
#pragma once

#include <stdbool.h>

typedef struct {
    float q0, q1, q2, q3;     /* Quaternion */
    float ix, iy, iz;         /* Integral des Fehlers */
    float kp, ki;
} mahony_t;

void mahony_init(mahony_t *m, float kp, float ki);
/* gyro in rad/s, acc in beliebiger Einheit (wird normiert), dt in s.
 * use_acc = false: nur Gyro integrieren (z. B. bei starker Beschleunigung). */
void mahony_update(mahony_t *m, float gx, float gy, float gz,
                   float ax, float ay, float az, bool use_acc, float dt);
/* Winkel in Grad: roll > 0 = rechts unten, pitch > 0 = Nase unten */
void mahony_get_euler_deg(const mahony_t *m, float *roll, float *pitch, float *yaw);
