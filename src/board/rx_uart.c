#include "rx_uart.h"
#include "board.h"
#include "system.h"

#define RING_SIZE 256u   /* Zweierpotenz */
#define TX_SIZE   64u    /* Zweierpotenz */

typedef struct {
    int            number;          /* 1, 2, 3, 4, 6 */
    USART_TypeDef *regs;
    IRQn_Type      irq;
    volatile uint32_t *rcc_enr;
    uint32_t       rcc_bit;
    uint32_t       clock_hz;
    GPIO_TypeDef  *tx_port, *rx_port;
    uint8_t        tx_pin, rx_pin, af;
} uart_hw_t;

/* Pins aus betaflight/config GEPRCF722 (UARTn_TX_PIN / UARTn_RX_PIN), AF laut Datenblatt */
static const uart_hw_t hw[RX_UART_PORTS] = {
    { 1, USART1, USART1_IRQn, &RCC->APB2ENR, RCC_APB2ENR_USART1EN, APB2_HZ, GPIOA, GPIOA,  9, 10, 7 },
    { 2, USART2, USART2_IRQn, &RCC->APB1ENR, RCC_APB1ENR_USART2EN, APB1_HZ, GPIOA, GPIOA,  2,  3, 7 },
    { 3, USART3, USART3_IRQn, &RCC->APB1ENR, RCC_APB1ENR_USART3EN, APB1_HZ, GPIOB, GPIOB, 10, 11, 7 },
    { 4, UART4,  UART4_IRQn,  &RCC->APB1ENR, RCC_APB1ENR_UART4EN,  APB1_HZ, GPIOA, GPIOA,  0,  1, 8 },
    { 6, USART6, USART6_IRQn, &RCC->APB2ENR, RCC_APB2ENR_USART6EN, APB2_HZ, GPIOC, GPIOC,  6,  7, 8 },
};

typedef struct {
    volatile uint8_t  ring[RING_SIZE];
    volatile uint32_t head;     /* schreibt nur der Interrupt */
    volatile uint32_t tail;     /* schreibt nur der Task */
    bool              active;
} rx_port_t;

static rx_port_t ports[RX_UART_PORTS];
static volatile uint32_t overruns;
static int locked = -1;

static volatile uint8_t  tx_ring[TX_SIZE];
static volatile uint32_t tx_head, tx_tail;

int rx_uart_number(int index) { return (index >= 0 && index < RX_UART_PORTS) ? hw[index].number : 0; }
bool rx_uart_active(int index) { return index >= 0 && index < RX_UART_PORTS && ports[index].active; }

static void open_port(int i, uint32_t baud, bool with_tx)
{
    const uart_hw_t *h = &hw[i];
    *h->rcc_enr |= h->rcc_bit;
    (void)*h->rcc_enr;
    gpio_config(h->rx_port, h->rx_pin, GPIO_AF, h->af, GPIO_PULLUP, true);
    if (with_tx) gpio_config(h->tx_port, h->tx_pin, GPIO_AF, h->af, GPIO_PULLUP, true);

    USART_TypeDef *u = h->regs;
    u->CR1 = 0;
    u->BRR = (h->clock_hz + baud / 2u) / baud;    /* 16-fach Oversampling */
    u->CR2 = 0;                                   /* 1 Stoppbit, nicht invertiert */
    u->CR3 = USART_CR3_OVRDIS;                    /* Ueberlauf blockiert den Empfang nicht */
    u->CR1 = USART_CR1_RE | (with_tx ? USART_CR1_TE : 0u) | USART_CR1_RXNEIE | USART_CR1_UE;
    ports[i].head = ports[i].tail = 0;
    ports[i].active = true;

    NVIC_SetPriority(h->irq, 7);
    NVIC_EnableIRQ(h->irq);
}

static void close_port(int i)
{
    NVIC_DisableIRQ(hw[i].irq);
    hw[i].regs->CR1 = 0;
    ports[i].active = false;
    gpio_config(hw[i].rx_port, hw[i].rx_pin, GPIO_IN, 0, GPIO_NOPULL, false);
}

void rx_uart_init(int uart, uint32_t baud)
{
    locked = -1;
    for (int i = 0; i < RX_UART_PORTS; i++) {
        if (uart == 0) {
            open_port(i, baud, false);            /* Suche: nur empfangen */
        } else if (hw[i].number == uart) {
            open_port(i, baud, true);
            locked = i;
        }
    }
}

void rx_uart_lock(int index)
{
    if (index < 0 || index >= RX_UART_PORTS || locked == index) return;
    for (int i = 0; i < RX_UART_PORTS; i++) {
        if (i != index && ports[i].active) close_port(i);
    }
    /* Sender dazuschalten (fuer die Telemetrie) */
    const uart_hw_t *h = &hw[index];
    gpio_config(h->tx_port, h->tx_pin, GPIO_AF, h->af, GPIO_PULLUP, true);
    h->regs->CR1 |= USART_CR1_TE;
    locked = index;
}

bool rx_uart_read(int index, uint8_t *byte)
{
    rx_port_t *p = &ports[index];
    if (!p->active || p->tail == p->head) return false;
    *byte = p->ring[p->tail];
    p->tail = (p->tail + 1u) & (RING_SIZE - 1u);
    return true;
}

bool rx_uart_write(const uint8_t *data, uint32_t len)
{
    if (locked < 0) return false;
    const uint32_t free_bytes = (tx_tail - tx_head - 1u) & (TX_SIZE - 1u);
    if (len > free_bytes) return false;
    for (uint32_t k = 0; k < len; k++) {
        tx_ring[tx_head] = data[k];
        tx_head = (tx_head + 1u) & (TX_SIZE - 1u);
    }
    hw[locked].regs->CR1 |= USART_CR1_TXEIE;      /* Interrupt holt die Bytes ab */
    return true;
}

uint32_t rx_uart_overruns(void)
{
    return overruns;
}

static void irq(int i)
{
    USART_TypeDef *u = hw[i].regs;
    const uint32_t isr = u->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;
        overruns++;
    }
    if (isr & USART_ISR_RXNE) {
        rx_port_t *p = &ports[i];
        const uint8_t b = (uint8_t)u->RDR;
        const uint32_t next = (p->head + 1u) & (RING_SIZE - 1u);
        if (next != p->tail) {
            p->ring[p->head] = b;
            p->head = next;
        } else {
            overruns++;   /* Puffer voll: Byte verwerfen */
        }
    }
    if ((u->CR1 & USART_CR1_TXEIE) && (isr & USART_ISR_TXE)) {
        if (i == locked && tx_tail != tx_head) {
            u->TDR = tx_ring[tx_tail];
            tx_tail = (tx_tail + 1u) & (TX_SIZE - 1u);
        } else {
            u->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

void USART1_IRQHandler(void) { irq(0); }
void USART2_IRQHandler(void) { irq(1); }
void USART3_IRQHandler(void) { irq(2); }
void UART4_IRQHandler(void)  { irq(3); }
void USART6_IRQHandler(void) { irq(4); }
