#include "system.h"

uint32_t SystemCoreClock = 16000000u;   /* wird nach der Takt-Initialisierung gesetzt */

/* Wird vom Startup-Code vor main() aufgerufen */
void SystemInit(void)
{
    SCB->CPACR |= (3u << (10u * 2u)) | (3u << (11u * 2u));   /* FPU freischalten */
    SCB->VTOR = FLASH_BASE;                                  /* Vektortabelle im Flash */
    __DSB();
    __ISB();
}

void system_clock_init(void)
{
    /* HSI einschalten (16 MHz intern). Der Quarz des Boards wird nicht benoetigt;
     * fuer UART/SPI/DShot reicht die Genauigkeit des HSI (ca. 1 %). */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    /* Spannungsregler auf Scale 1 fuer 216 MHz */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_0 | PWR_CR1_VOS_1;

    /* Flash: 7 Wartezyklen, ART-Beschleuniger und Prefetch */
    FLASH->ACR = FLASH_ACR_LATENCY_7WS | FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;

    /* PLL: 16 MHz / M8 = 2 MHz, * N216 = 432 MHz, / P2 = 216 MHz, / Q9 = 48 MHz */
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }
    RCC->PLLCFGR = (8u << RCC_PLLCFGR_PLLM_Pos) |
                   (216u << RCC_PLLCFGR_PLLN_Pos) |
                   (0u << RCC_PLLCFGR_PLLP_Pos) |      /* 0 = Teiler 2 */
                   RCC_PLLCFGR_PLLSRC_HSI |
                   (9u << RCC_PLLCFGR_PLLQ_Pos);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    /* Over-Drive fuer 216 MHz */
    PWR->CR1 |= PWR_CR1_ODEN;
    while (!(PWR->CSR1 & PWR_CSR1_ODRDY)) { }
    PWR->CR1 |= PWR_CR1_ODSWEN;
    while (!(PWR->CSR1 & PWR_CSR1_ODSWRDY)) { }

    /* Busse: AHB 216 MHz, APB1 54 MHz (/4), APB2 108 MHz (/2) */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
                RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    /* Systemtakt auf PLL umschalten */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }

    SystemCoreClock = SYSCLK_HZ;

    /* Instruktions-Cache einschalten. Der Daten-Cache bleibt aus,
     * damit DMA-Puffer ohne Cache-Pflege funktionieren. */
    SCB_EnableICache();
}

void system_timer_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;     /* Cortex-M7: DWT entsperren */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t micros(void)
{
    return DWT->CYCCNT / (SYSCLK_HZ / 1000000u);
}

void delay_us(uint32_t us)
{
    const uint32_t start = DWT->CYCCNT;
    const uint32_t ticks = us * (SYSCLK_HZ / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) { }
}

void delay_ms(uint32_t ms)
{
    while (ms--) delay_us(1000);
}

void gpio_clock_enable(GPIO_TypeDef *port)
{
    if (port == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (port == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if (port == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if (port == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    (void)RCC->AHB1ENR;
}

void gpio_config(GPIO_TypeDef *port, uint32_t pin, gpio_mode_t mode, uint32_t af,
                 gpio_pull_t pull, bool high_speed)
{
    gpio_clock_enable(port);
    if (mode == GPIO_AF) {
        volatile uint32_t *afr = &port->AFR[pin >> 3];
        const uint32_t shift = (pin & 7u) * 4u;
        *afr = (*afr & ~(0xFu << shift)) | ((af & 0xFu) << shift);
    }
    port->OSPEEDR = (port->OSPEEDR & ~(3u << (pin * 2u))) | ((high_speed ? 3u : 0u) << (pin * 2u));
    port->OTYPER &= ~(1u << pin);   /* Push-Pull */
    port->PUPDR = (port->PUPDR & ~(3u << (pin * 2u))) | ((uint32_t)pull << (pin * 2u));
    port->MODER = (port->MODER & ~(3u << (pin * 2u))) | ((uint32_t)mode << (pin * 2u));
}

void watchdog_init(uint32_t timeout_ms)
{
    /* LSI ~32 kHz, Teiler 32 -> 1 kHz, also 1 Zaehlerschritt pro ms */
    IWDG->KR = 0xCCCC;          /* starten */
    IWDG->KR = 0x5555;          /* Register freigeben */
    IWDG->PR = 3;               /* Teiler 32 */
    IWDG->RLR = timeout_ms > 4095 ? 4095 : timeout_ms;
    while (IWDG->SR) { }
    IWDG->KR = 0xAAAA;
}

void watchdog_kick(void)
{
    IWDG->KR = 0xAAAA;
}
