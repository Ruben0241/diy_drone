#include "system.h"

uint32_t SystemCoreClock = 16000000u;   /* wird nach der Takt-Initialisierung gesetzt */

static bool clock_from_hse;

/* Wunsch "in den Bootloader starten". Liegt in .noinit und ueberlebt deshalb
 * einen Software-Reset. Zwei Woerter, damit zufaelliger RAM-Inhalt nach dem
 * Einschalten praktisch nie passt. */
#define BOOT_MAGIC 0xDF00B007u
__attribute__((section(".noinit"))) static volatile uint32_t boot_request[2];

/* STM32F72x/73x: System-Bootloader (DFU) ab 0x1FF00000 (ST AN2606) */
#define SYSTEM_MEMORY_BASE 0x1FF00000u

/* Wird vom Startup-Code vor main() aufgerufen, noch vor .data/.bss */
void SystemInit(void)
{
    if (boot_request[0] == BOOT_MAGIC && boot_request[1] == ~BOOT_MAGIC) {
        boot_request[0] = 0;
        boot_request[1] = 0;
        /* Peripherie ist nach dem Reset unveraendert: direkt springen */
        const uint32_t *sysmem = (const uint32_t *)SYSTEM_MEMORY_BASE;
        __set_MSP(sysmem[0]);
        ((void (*)(void))sysmem[1])();
        for (;;) { }
    }
    SCB->CPACR |= (3u << (10u * 2u)) | (3u << (11u * 2u));   /* FPU freischalten */
    SCB->VTOR = FLASH_BASE;                                  /* Vektortabelle im Flash */
    __DSB();
    __ISB();
}

void system_reboot(bool to_bootloader)
{
    if (to_bootloader) {
        boot_request[0] = BOOT_MAGIC;
        boot_request[1] = ~BOOT_MAGIC;
    }
    __DSB();
    NVIC_SystemReset();
}

bool system_clock_is_hse(void)
{
    return clock_from_hse;
}

/* Quarz starten; false, wenn er nicht innerhalb von ca. 50 ms anlaeuft */
static bool start_hse(void)
{
    RCC->CR |= RCC_CR_HSEON;
    for (uint32_t i = 0; i < 400000u; i++) {      /* ca. 50 ms bei 16 MHz */
        if (RCC->CR & RCC_CR_HSERDY) return true;
    }
    RCC->CR &= ~RCC_CR_HSEON;
    return false;
}

void system_clock_init(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    /* Spannungsregler auf Scale 1 fuer 216 MHz */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_0 | PWR_CR1_VOS_1;

    /* Flash: 7 Wartezyklen, ART-Beschleuniger und Prefetch */
    FLASH->ACR = FLASH_ACR_LATENCY_7WS | FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;

    /* PLL wie Betaflight auf dem F7 (Quarz 8 MHz):
     *   8 MHz / M8 = 1 MHz, * N432 = 432 MHz, / P2 = 216 MHz (CPU), / Q9 = 48 MHz (USB)
     * Laeuft der Quarz nicht an, wird der interne HSI (16 MHz, ca. 1 %) verwendet:
     *   16 MHz / M16 = 1 MHz, sonst gleich. Fliegen geht, USB ist dann unzuverlaessig. */
    clock_from_hse = start_hse();
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }
    RCC->PLLCFGR = ((clock_from_hse ? HSE_MHZ : 16u) << RCC_PLLCFGR_PLLM_Pos) |
                   (432u << RCC_PLLCFGR_PLLN_Pos) |
                   (0u << RCC_PLLCFGR_PLLP_Pos) |      /* 0 = Teiler 2 */
                   (clock_from_hse ? RCC_PLLCFGR_PLLSRC_HSE : RCC_PLLCFGR_PLLSRC_HSI) |
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

    /* USB-Takt (48 MHz) aus PLL-Q, nicht aus PLLSAI */
    RCC->DCKCFGR2 &= ~RCC_DCKCFGR2_CK48MSEL;

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

/* Der 32-Bit-Zykluszaehler laeuft bei 216 MHz nach ~19.9 s ueber. Darum wird er
 * hier auf 64 Bit erweitert; micros() laeuft so erst nach ~71 min ueber.
 * Voraussetzung: Aufruf mindestens alle 19 s (der Regel-Task ruft 1000x/s auf). */
uint32_t micros(void)
{
    static uint32_t last_cycles;
    static uint64_t high_cycles;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now = DWT->CYCCNT;
    if (now < last_cycles) {
        high_cycles += (1ull << 32);
    }
    last_cycles = now;
    const uint64_t total = high_cycles + now;
    __set_PRIMASK(primask);
    return (uint32_t)(total / (SYSCLK_HZ / 1000000u));
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
