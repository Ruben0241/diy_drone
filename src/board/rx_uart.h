/*
 * rx_uart.h - Empfang der CRSF-Bytes (Interrupt + Ringpuffer) mit automatischer Suche
 *
 * Auf dem GEPRC F722 sind UART 1, 2, 3, 4 und 6 herausgefuehrt. Welcher davon am
 * ELRS-Empfaenger haengt, haengt von der Verkabelung ab (Betaflight: Ports-Tab).
 * Mit rx_uart = 0 werden alle fuenf nur empfangend geoeffnet; der Empfaenger-Task
 * rastet auf dem ersten UART ein, der gueltige CRSF-Pakete liefert (rx_uart_lock).
 * Erst danach wird dort auch gesendet (Telemetrie) und die anderen werden abgeschaltet.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define RX_UART_PORTS 5   /* Index 0..4 = UART 1, 2, 3, 4, 6 */

/* uart: 0 = alle (Suche), sonst 1, 2, 3, 4 oder 6 */
void     rx_uart_init(int uart, uint32_t baud);
int      rx_uart_number(int index);          /* Index -> UART-Nummer (1, 2, 3, 4, 6) */
bool     rx_uart_active(int index);
bool     rx_uart_read(int index, uint8_t *byte);   /* false = Puffer leer */
/* Nur noch diesen UART verwenden, Sender einschalten */
void     rx_uart_lock(int index);
/* Telemetrie senden (nur am eingerasteten UART, nicht blockierend). false = Puffer voll */
bool     rx_uart_write(const uint8_t *data, uint32_t len);
uint32_t rx_uart_overruns(void);
