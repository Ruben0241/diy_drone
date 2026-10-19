#include "peripherals.h"
#include "board.h"
#include "system.h"

void indicators_init(void)
{
    gpio_config(LED0_PORT, LED0_PIN, GPIO_OUT, 0, GPIO_NOPULL, false);
    gpio_config(LED1_PORT, LED1_PIN, GPIO_OUT, 0, GPIO_NOPULL, false);
    gpio_config(BEEPER_PORT, BEEPER_PIN, GPIO_OUT, 0, GPIO_NOPULL, false);
    led_set(0, false);
    led_set(1, false);
    beeper_set(false);

    /* nicht verwendete Chips auf den anderen SPI-Bussen abwaehlen */
    gpio_config(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_OUT, 0, GPIO_NOPULL, false);
    gpio_write(FLASH_CS_PORT, FLASH_CS_PIN, true);
    gpio_config(OSD_CS_PORT, OSD_CS_PIN, GPIO_OUT, 0, GPIO_NOPULL, false);
    gpio_write(OSD_CS_PORT, OSD_CS_PIN, true);
}

void led_set(int led, bool on)
{
    const bool level = LED_ACTIVE_LOW ? !on : on;
    if (led == 0) gpio_write(LED0_PORT, LED0_PIN, level);
    else          gpio_write(LED1_PORT, LED1_PIN, level);
}

void beeper_set(bool on)
{
    gpio_write(BEEPER_PORT, BEEPER_PIN, BEEPER_ACTIVE_HIGH ? on : !on);
}

void vbat_init(void)
{
    gpio_config(GPIOC, 0, GPIO_ANALOG, 0, GPIO_NOPULL, false);
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;

    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | (1u << ADC_CCR_ADCPRE_Pos);   /* 108 / 4 = 27 MHz */
    ADC1->CR1 = 0;                                   /* 12 Bit */
    ADC1->CR2 = 0;
    ADC1->SMPR1 = (7u << ADC_SMPR1_SMP10_Pos);       /* lange Abtastzeit (480 Zyklen) */
    ADC1->SQR1 = 0;                                  /* eine Wandlung */
    ADC1->SQR3 = VBAT_ADC_CHANNEL;
    ADC1->CR2 |= ADC_CR2_ADON;
    delay_us(10);
}

float vbat_read_volts(void)
{
    ADC1->SR = 0;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    uint32_t timeout = 100000;
    while (!(ADC1->SR & ADC_SR_EOC) && --timeout) { }
    const float raw = (float)(ADC1->DR & 0xFFF);
    return raw / 4095.0f * 3.3f * VBAT_SCALE / 10.0f;
}
