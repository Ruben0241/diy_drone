/*
 * system.h - Takt, Zeitmessung, GPIO-Hilfsfunktionen, Watchdog
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "board.h"

typedef enum { GPIO_IN = 0, GPIO_OUT = 1, GPIO_AF = 2, GPIO_ANALOG = 3 } gpio_mode_t;
typedef enum { GPIO_NOPULL = 0, GPIO_PULLUP = 1, GPIO_PULLDOWN = 2 } gpio_pull_t;

void     system_clock_init(void);       /* HSI 16 MHz -> PLL -> 216 MHz */
void     system_timer_init(void);       /* DWT-Zykluszaehler fuer Mikrosekunden */
uint32_t micros(void);
void     delay_us(uint32_t us);
void     delay_ms(uint32_t ms);

void gpio_clock_enable(GPIO_TypeDef *port);
void gpio_config(GPIO_TypeDef *port, uint32_t pin, gpio_mode_t mode, uint32_t af,
                 gpio_pull_t pull, bool high_speed);

static inline void gpio_write(GPIO_TypeDef *port, uint32_t pin, bool high)
{
    port->BSRR = high ? (1u << pin) : (1u << (pin + 16u));
}
static inline bool gpio_read(GPIO_TypeDef *port, uint32_t pin)
{
    return (port->IDR & (1u << pin)) != 0;
}

void watchdog_init(uint32_t timeout_ms);
void watchdog_kick(void);
