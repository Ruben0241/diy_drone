/*
 * board.h - Pinbelegung GEPRC TAKER F722 (GEP-F722-HD V2), Betaflight-Target GEPRCF722
 *
 * Quelle der Pinbelegung: betaflight/config, configs/GEPR/GEPRCF722/config.h
 * VOR DEM FLASHEN pruefen: Betaflight-CLI "version" muss GEPRCF722 anzeigen.
 *
 *   Gyro ICM-42688-P   SPI3 (SCK PC10, MISO PC11, MOSI PC12), CS PA15, INT PC3
 *                      Ausrichtung CW180 (x und y umgekehrt)
 *   Gyro 2 (optional)  SPI3, CS PD2  -> wird immer abgewaehlt (high)
 *   Empfaenger (CRSF)  UART2: TX PA2, RX PA3  [pruefen: an welchem UART haengt der RP3?]
 *   Motoren (DShot)    M1 PB4 TIM3_CH1, M2 PB5 TIM3_CH2, M3 PB0 TIM3_CH3, M4 PB1 TIM3_CH4
 *                      DMA-Burst ueber TIM3_UP: DMA1 Stream 2, Kanal 5
 *   Blackbox-Flash     W25Q128 an SPI2 (SCK PB13, MISO PB14, MOSI PB15), CS PB12
 *   OSD MAX7456        SPI1, CS PB2 (nicht verwendet, abgewaehlt)
 *   Akkuspannung       PC0 = ADC1 Kanal 10
 *   USB                PA11/PA12 (OTG_FS), VBUS-Erkennung abgeschaltet
 *   LEDs               PA13, PA14 (dadurch kein SWD, Flashen nur per DFU)
 *   Summer             PC13 (invertiert)
 */
#pragma once

#include "stm32f7xx.h"

#define HSE_MHZ         8u           /* Quarz; Betaflight nimmt auf dem F7 ebenfalls 8 MHz an */
#define SYSCLK_HZ       216000000u
#define APB1_HZ          54000000u
#define APB2_HZ         108000000u
#define APB1_TIMER_HZ   108000000u   /* Timer-Takt = 2 x APB1, da Vorteiler != 1 */

/* Gyro */
#define GYRO_SPI        SPI3
#define GYRO_CS_PORT    GPIOA
#define GYRO_CS_PIN     15
#define GYRO2_CS_PORT   GPIOD
#define GYRO2_CS_PIN    2
#define GYRO_INT_PORT   GPIOC
#define GYRO_INT_PIN    3

/* Empfaenger */
#define RX_USART        USART2
#define RX_USART_IRQn   USART2_IRQn

/* Abzuwaehlende Chips auf anderen SPI-Bussen */
#define FLASH_CS_PORT   GPIOB
#define FLASH_CS_PIN    12
#define OSD_CS_PORT     GPIOB
#define OSD_CS_PIN      2

/* LEDs und Summer */
#define LED0_PORT       GPIOA
#define LED0_PIN        13
#define LED1_PORT       GPIOA
#define LED1_PIN        14
#define LED_ACTIVE_LOW  1        /* [pruefen] */
#define BEEPER_PORT     GPIOC
#define BEEPER_PIN      13
#define BEEPER_ACTIVE_HIGH 1     /* Betaflight: BEEPER_INVERTED */

/* Akkuspannung: U = ADC / 4095 * 3.3 V * VBAT_SCALE / 10
 * VBAT_SCALE wie Betaflight "vbat_scale" [pruefen: Wert aus diff all uebernehmen] */
#define VBAT_ADC_CHANNEL 10
#define VBAT_SCALE       110.0f
