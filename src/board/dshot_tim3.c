#include "dshot_tim3.h"
#include "board.h"
#include "system.h"
#include "dshot.h"

#define MOTORS 4u
#define DMA_STREAM       DMA1_Stream2
#define DMA_CHANNEL      5u
#define DMA_FLAGS_MASK   (DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | \
                          DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2)

/* Puffer: pro Bit-Slot vier Werte (CCR1..CCR4) hintereinander */
static uint32_t dma_buffer[DSHOT_BUFFER_SLOTS * MOTORS];
static uint32_t ticks_zero, ticks_one;
static bool initialized;

void dshot_init(dshot_rate_t rate)
{
    /* Bitdauer in Timer-Ticks: 108 MHz / 300 kHz = 360 */
    const uint32_t period = APB1_TIMER_HZ / ((uint32_t)rate * 1000u);
    ticks_zero = (period * 3u) / 8u;   /* 37.5 % High-Zeit -> "0" */
    ticks_one  = (period * 3u) / 4u;   /* 75 % High-Zeit   -> "1" */

    /* Pins: M1 PB4, M2 PB5, M3 PB0, M4 PB1, alle AF2 = TIM3 */
    gpio_config(GPIOB, 4, GPIO_AF, 2, GPIO_PULLDOWN, true);
    gpio_config(GPIOB, 5, GPIO_AF, 2, GPIO_PULLDOWN, true);
    gpio_config(GPIOB, 0, GPIO_AF, 2, GPIO_PULLDOWN, true);
    gpio_config(GPIOB, 1, GPIO_AF, 2, GPIO_PULLDOWN, true);

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;

    TIM3->CR1 = 0;
    TIM3->PSC = 0;
    TIM3->ARR = period - 1u;
    /* PWM-Modus 1 mit Preload auf allen vier Kanaelen */
    TIM3->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM3->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (6u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = 0;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    /* DMA-Burst: Startadresse CCR1 (Offset 0x34 / 4 = 13), 4 Transfers pro Update */
    TIM3->DCR = (3u << TIM_DCR_DBL_Pos) | (13u << TIM_DCR_DBA_Pos);
    TIM3->DIER = TIM_DIER_UDE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    /* DMA1 Stream 2: Speicher -> TIM3->DMAR, 32 Bit, Adresse im Speicher erhoehen */
    DMA_STREAM->CR = 0;
    while (DMA_STREAM->CR & DMA_SxCR_EN) { }
    DMA_STREAM->PAR = (uint32_t)&TIM3->DMAR;
    DMA_STREAM->CR = (DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) |
                     (2u << DMA_SxCR_PL_Pos) |          /* hohe Prioritaet */
                     (2u << DMA_SxCR_MSIZE_Pos) |       /* 32 Bit */
                     (2u << DMA_SxCR_PSIZE_Pos) |       /* 32 Bit */
                     DMA_SxCR_MINC |
                     (1u << DMA_SxCR_DIR_Pos);          /* Speicher -> Peripherie */
    DMA_STREAM->FCR = 0;                                /* Direct Mode */

    initialized = true;
}

bool dshot_write(const uint16_t values[4])
{
    if (!initialized) return false;

    /* Laeuft noch eine Uebertragung? Dann diesen Zyklus auslassen. */
    if (DMA_STREAM->CR & DMA_SxCR_EN) return false;

    for (uint32_t m = 0; m < MOTORS; m++) {
        const uint16_t frame = dshot_make_frame(values[m], false);
        dshot_fill_buffer(&dma_buffer[m], MOTORS, frame, ticks_zero, ticks_one);
    }

    DMA1->LIFCR = DMA_FLAGS_MASK;
    DMA_STREAM->M0AR = (uint32_t)dma_buffer;
    DMA_STREAM->NDTR = DSHOT_BUFFER_SLOTS * MOTORS;
    DMA_STREAM->CR |= DMA_SxCR_EN;
    return true;
}

void dshot_emergency_stop(void)
{
    DMA_STREAM->CR &= ~DMA_SxCR_EN;
    TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = 0;
    TIM3->CCER = 0;
    /* Pins als Ausgang low: ESC bekommt kein Signal mehr und stoppt */
    gpio_config(GPIOB, 4, GPIO_OUT, 0, GPIO_NOPULL, false); gpio_write(GPIOB, 4, false);
    gpio_config(GPIOB, 5, GPIO_OUT, 0, GPIO_NOPULL, false); gpio_write(GPIOB, 5, false);
    gpio_config(GPIOB, 0, GPIO_OUT, 0, GPIO_NOPULL, false); gpio_write(GPIOB, 0, false);
    gpio_config(GPIOB, 1, GPIO_OUT, 0, GPIO_NOPULL, false); gpio_write(GPIOB, 1, false);
    initialized = false;
}
