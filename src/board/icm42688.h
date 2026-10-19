/*
 * icm42688.h - Treiber fuer die IMU TDK InvenSense ICM-42688-P an SPI3
 *
 * Messbereiche: Gyro +-2000 dps (16.4 LSB/dps), Beschleunigung +-16 g (2048 LSB/g)
 * Der Data-Ready-Interrupt (INT1 an PC3) taktet die Regelschleife.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum { ICM_ODR_8K = 3, ICM_ODR_4K = 4, ICM_ODR_2K = 5, ICM_ODR_1K = 6 } icm_odr_t;

typedef struct {
    float gyro_dps[3];   /* Koerpersystem (nach Ausrichtung CW180) */
    float acc_g[3];
    float temp_c;
} imu_sample_t;

bool icm42688_init(icm_odr_t odr);       /* false = Sensor nicht gefunden */
void icm42688_read(imu_sample_t *s);
uint8_t icm42688_whoami(void);

/* Wird im Data-Ready-Interrupt aufgerufen (in main.c gesetzt) */
extern void (*icm42688_drdy_callback)(void);
