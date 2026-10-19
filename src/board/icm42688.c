#include "icm42688.h"
#include "board.h"
#include "system.h"

/* Register (Bank 0, sofern nicht anders vermerkt) */
#define REG_DEVICE_CONFIG      0x11
#define REG_INT_CONFIG         0x14
#define REG_TEMP_DATA1         0x1D   /* danach ACCEL X..Z, GYRO X..Z (je 2 Byte, big endian) */
#define REG_INTF_CONFIG1       0x4D
#define REG_PWR_MGMT0          0x4E
#define REG_GYRO_CONFIG0       0x4F
#define REG_ACCEL_CONFIG0      0x50
#define REG_GYRO_ACCEL_CONFIG0 0x52
#define REG_INT_CONFIG0        0x63
#define REG_INT_CONFIG1        0x64
#define REG_INT_SOURCE0        0x65
#define REG_WHO_AM_I           0x75
#define REG_BANK_SEL           0x76
/* Bank 1 */
#define REG_GYRO_CONFIG_STATIC3 0x0C
#define REG_GYRO_CONFIG_STATIC4 0x0D
#define REG_GYRO_CONFIG_STATIC5 0x0E
/* Bank 2 */
#define REG_ACCEL_CONFIG_STATIC2 0x03
#define REG_ACCEL_CONFIG_STATIC3 0x04
#define REG_ACCEL_CONFIG_STATIC4 0x05

#define WHO_AM_I_ICM42688P     0x47
#define GYRO_LSB_PER_DPS       16.4f
#define ACC_LSB_PER_G          2048.0f

void (*icm42688_drdy_callback)(void) = 0;

/* ---------- SPI3 ---------- */

static inline void cs_low(void)  { gpio_write(GYRO_CS_PORT, GYRO_CS_PIN, false); }
static inline void cs_high(void) { gpio_write(GYRO_CS_PORT, GYRO_CS_PIN, true); }

static uint8_t spi_xfer(uint8_t out)
{
    while (!(GYRO_SPI->SR & SPI_SR_TXE)) { }
    *(volatile uint8_t *)&GYRO_SPI->DR = out;     /* 8-Bit-Zugriff, sonst 16 Bit */
    while (!(GYRO_SPI->SR & SPI_SR_RXNE)) { }
    return *(volatile uint8_t *)&GYRO_SPI->DR;
}

static void spi_set_prescaler(uint32_t br_bits)
{
    GYRO_SPI->CR1 &= ~SPI_CR1_SPE;
    GYRO_SPI->CR1 = (GYRO_SPI->CR1 & ~SPI_CR1_BR) | (br_bits << SPI_CR1_BR_Pos);
    GYRO_SPI->CR1 |= SPI_CR1_SPE;
}

static void spi_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    (void)RCC->APB1ENR;

    gpio_config(GPIOC, 10, GPIO_AF, 6, GPIO_NOPULL, true);   /* SCK  */
    gpio_config(GPIOC, 11, GPIO_AF, 6, GPIO_PULLUP, true);   /* MISO */
    gpio_config(GPIOC, 12, GPIO_AF, 6, GPIO_NOPULL, true);   /* MOSI */

    gpio_config(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_OUT, 0, GPIO_NOPULL, true);
    cs_high();
    /* moeglichen zweiten Gyro auf demselben Bus abwaehlen */
    gpio_config(GYRO2_CS_PORT, GYRO2_CS_PIN, GPIO_OUT, 0, GPIO_NOPULL, false);
    gpio_write(GYRO2_CS_PORT, GYRO2_CS_PIN, true);

    /* Master, SPI-Mode 3 (CPOL=1, CPHA=1), Software-CS, 8 Bit */
    GYRO_SPI->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA |
                    (6u << SPI_CR1_BR_Pos);                    /* 54 MHz / 128 = 422 kHz */
    GYRO_SPI->CR2 = (7u << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;   /* 8 Bit, RXNE bei 8 Bit */
    GYRO_SPI->CR1 |= SPI_CR1_SPE;
}

static void write_reg(uint8_t reg, uint8_t value)
{
    cs_low();
    spi_xfer(reg & 0x7F);
    spi_xfer(value);
    cs_high();
    delay_us(10);
}

static uint8_t read_reg(uint8_t reg)
{
    cs_low();
    spi_xfer(reg | 0x80);
    const uint8_t v = spi_xfer(0);
    cs_high();
    return v;
}

static void read_burst(uint8_t reg, uint8_t *buf, uint32_t len)
{
    cs_low();
    spi_xfer(reg | 0x80);
    for (uint32_t i = 0; i < len; i++) buf[i] = spi_xfer(0);
    cs_high();
}

static void set_bank(uint8_t bank)
{
    write_reg(REG_BANK_SEL, bank & 7u);
}

/* ---------- Data-Ready-Interrupt (PC3 -> EXTI3) ---------- */

static void drdy_irq_init(void)
{
    gpio_config(GYRO_INT_PORT, GYRO_INT_PIN, GPIO_IN, 0, GPIO_PULLDOWN, false);
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI3) | SYSCFG_EXTICR1_EXTI3_PC;
    EXTI->RTSR |= (1u << 3);
    EXTI->FTSR &= ~(1u << 3);
    EXTI->PR = (1u << 3);
    EXTI->IMR |= (1u << 3);
    NVIC_SetPriority(EXTI3_IRQn, 6);    /* >= configMAX_SYSCALL_INTERRUPT_PRIORITY (5) */
    NVIC_EnableIRQ(EXTI3_IRQn);
}

void EXTI3_IRQHandler(void)
{
    EXTI->PR = (1u << 3);
    if (icm42688_drdy_callback) icm42688_drdy_callback();
}

/* ---------- Initialisierung ---------- */

uint8_t icm42688_whoami(void)
{
    return read_reg(REG_WHO_AM_I);
}

bool icm42688_init(icm_odr_t odr)
{
    spi_init();
    delay_ms(10);

    set_bank(0);
    write_reg(REG_DEVICE_CONFIG, 0x01);      /* Soft-Reset */
    delay_ms(5);

    bool found = false;
    for (int attempt = 0; attempt < 10 && !found; attempt++) {
        found = (icm42688_whoami() == WHO_AM_I_ICM42688P);
        if (!found) delay_ms(10);
    }
    if (!found) return false;

    /* Zum Konfigurieren Gyro und Beschleunigung aus */
    set_bank(0);
    write_reg(REG_PWR_MGMT0, 0x00);

    /* Anti-Alias-Filter ca. 258 Hz (Datenblatt Kap. 5.3: DELT=6, DELTSQR=36, BITSHIFT=10) */
    set_bank(1);
    write_reg(REG_GYRO_CONFIG_STATIC3, 6);
    write_reg(REG_GYRO_CONFIG_STATIC4, 36 & 0xFF);
    write_reg(REG_GYRO_CONFIG_STATIC5, (uint8_t)((36 >> 8) | (10 << 4)));
    set_bank(2);
    write_reg(REG_ACCEL_CONFIG_STATIC2, 6 << 1);
    write_reg(REG_ACCEL_CONFIG_STATIC3, 36 & 0xFF);
    write_reg(REG_ACCEL_CONFIG_STATIC4, (uint8_t)((36 >> 8) | (10 << 4)));

    set_bank(0);
    /* UI-Filter: niedrige Latenz */
    write_reg(REG_GYRO_ACCEL_CONFIG0, (15u << 4) | 15u);
    /* INT1: Puls, Push-Pull, aktiv high; Data-Ready auf INT1 */
    write_reg(REG_INT_CONFIG, 0x03);
    write_reg(REG_INT_CONFIG0, 0x00);
    write_reg(REG_INT_SOURCE0, 1u << 3);
    uint8_t v = read_reg(REG_INT_CONFIG1);
    v &= (uint8_t)~(1u << 4);                 /* INT_ASYNC_RESET = 0 (Datenblatt) */
    v |= (uint8_t)((1u << 6) | (1u << 5));    /* kurzer Puls, De-Assert aus */
    write_reg(REG_INT_CONFIG1, v);
    /* AFSR aus, verhindert Aussetzer im Gyrosignal */
    v = read_reg(REG_INTF_CONFIG1);
    v = (uint8_t)((v & ~0xC0u) | 0x40u);
    write_reg(REG_INTF_CONFIG1, v);

    /* Gyro und Beschleunigung im Low-Noise-Modus einschalten */
    write_reg(REG_PWR_MGMT0, 0x0F);
    delay_ms(1);
    /* Messbereich (FS_SEL = 0: 2000 dps / 16 g) und Ausgaberate */
    write_reg(REG_GYRO_CONFIG0, (uint8_t)((0u << 5) | ((uint8_t)odr & 0x0F)));
    delay_ms(15);
    write_reg(REG_ACCEL_CONFIG0, (uint8_t)((0u << 5) | ((uint8_t)odr & 0x0F)));
    delay_ms(15);

    /* Ab jetzt schnell auslesen: 54 MHz / 4 = 13.5 MHz (max. 24 MHz) */
    spi_set_prescaler(1);

    drdy_irq_init();
    return true;
}

static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

void icm42688_read(imu_sample_t *s)
{
    uint8_t b[14];
    read_burst(REG_TEMP_DATA1, b, sizeof(b));

    const float ax = be16(&b[2]) / ACC_LSB_PER_G;
    const float ay = be16(&b[4]) / ACC_LSB_PER_G;
    const float az = be16(&b[6]) / ACC_LSB_PER_G;
    const float gx = be16(&b[8]) / GYRO_LSB_PER_DPS;
    const float gy = be16(&b[10]) / GYRO_LSB_PER_DPS;
    const float gz = be16(&b[12]) / GYRO_LSB_PER_DPS;

    /* Ausrichtung CW180 (Betaflight GYRO_1_ALIGN): x -> -x, y -> -y, z -> z */
    s->acc_g[0] = -ax;   s->acc_g[1] = -ay;   s->acc_g[2] = az;
    s->gyro_dps[0] = -gx; s->gyro_dps[1] = -gy; s->gyro_dps[2] = gz;
    s->temp_c = be16(&b[0]) / 132.48f + 25.0f;
}
