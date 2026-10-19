#include "rx_uart.h"
#include "board.h"
#include "system.h"

#define RING_SIZE 256u   /* Zweierpotenz */

static volatile uint8_t  ring[RING_SIZE];
static volatile uint32_t head;   /* schreibt nur der Interrupt */
static volatile uint32_t tail;   /* schreibt nur der Task */
static volatile uint32_t overruns;

void rx_uart_init(uint32_t baud)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;

    gpio_config(GPIOA, 2, GPIO_AF, 7, GPIO_PULLUP, true);   /* TX2 (spaeter Telemetrie) */
    gpio_config(GPIOA, 3, GPIO_AF, 7, GPIO_PULLUP, true);   /* RX2 */

    RX_USART->CR1 = 0;
    RX_USART->BRR = (APB1_HZ + baud / 2u) / baud;          /* 16-fach Oversampling */
    RX_USART->CR2 = 0;                                      /* 1 Stoppbit, nicht invertiert */
    RX_USART->CR3 = 0;
    RX_USART->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_SetPriority(RX_USART_IRQn, 7);
    NVIC_EnableIRQ(RX_USART_IRQn);
}

void USART2_IRQHandler(void)
{
    const uint32_t isr = RX_USART->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        RX_USART->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;
        overruns++;
    }
    if (isr & USART_ISR_RXNE) {
        const uint8_t b = (uint8_t)RX_USART->RDR;
        const uint32_t next = (head + 1u) & (RING_SIZE - 1u);
        if (next != tail) {
            ring[head] = b;
            head = next;
        } else {
            overruns++;   /* Puffer voll: Byte verwerfen */
        }
    }
}

bool rx_uart_read(uint8_t *byte)
{
    if (tail == head) return false;
    *byte = ring[tail];
    tail = (tail + 1u) & (RING_SIZE - 1u);
    return true;
}

uint32_t rx_uart_overruns(void)
{
    return overruns;
}
