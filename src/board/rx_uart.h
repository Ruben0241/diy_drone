/*
 * rx_uart.h - Empfang der CRSF-Bytes ueber USART2 (Interrupt + Ringpuffer)
 *
 * Der Interrupt legt jedes Byte in einen Ringpuffer; der Empfaenger-Task
 * liest ihn regelmaessig aus und fuettert den CRSF-Parser.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void     rx_uart_init(uint32_t baud);
bool     rx_uart_read(uint8_t *byte);    /* false = Puffer leer */
uint32_t rx_uart_overruns(void);
