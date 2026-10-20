#include "flash_w25q.h"
#include "board.h"
#include "spi.h"
#include "system.h"

#define FLASH_SPI         SPI2

#define CMD_WRITE_ENABLE  0x06
#define CMD_READ_STATUS1  0x05
#define CMD_READ_DATA     0x03
#define CMD_PAGE_PROGRAM  0x02
#define CMD_BLOCK_ERASE64 0xD8
#define CMD_CHIP_ERASE    0xC7
#define CMD_JEDEC_ID      0x9F
#define CMD_RELEASE_PD    0xAB
#define STATUS_BUSY       0x01

#define PAGE_SIZE         256u
#define MAX_CAPACITY      (16u * 1024u * 1024u)   /* 3-Byte-Adressen */

static uint32_t jedec_id;
static uint32_t capacity;
static void (*wait_hook)(void);

static inline void cs_low(void)  { gpio_write(FLASH_CS_PORT, FLASH_CS_PIN, false); }
static inline void cs_high(void) { gpio_write(FLASH_CS_PORT, FLASH_CS_PIN, true); }

static void send_addr(uint8_t cmd, uint32_t addr)
{
    const uint8_t b[4] = { cmd, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    spi_write(FLASH_SPI, b, 4);
}

static uint8_t read_status(void)
{
    cs_low();
    spi_xfer(FLASH_SPI, CMD_READ_STATUS1);
    const uint8_t s = spi_xfer(FLASH_SPI, 0xFF);
    cs_high();
    return s;
}

static void write_enable(void)
{
    cs_low();
    spi_xfer(FLASH_SPI, CMD_WRITE_ENABLE);
    cs_high();
}

/* Warten bis der Flash fertig ist. long_op: Hook aufrufen (andere Tasks laufen lassen) */
static bool wait_ready(uint32_t timeout_ms, bool long_op)
{
    const uint32_t start = micros();
    while (read_status() & STATUS_BUSY) {
        if ((micros() - start) > timeout_ms * 1000u) return false;
        if (long_op && wait_hook) wait_hook();
    }
    return true;
}

void flash_set_wait_hook(void (*hook)(void))
{
    wait_hook = hook;
}

bool flash_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    (void)RCC->APB1ENR;

    gpio_config(GPIOB, 13, GPIO_AF, 5, GPIO_NOPULL, true);   /* SCK  */
    gpio_config(GPIOB, 14, GPIO_AF, 5, GPIO_PULLUP, true);   /* MISO */
    gpio_config(GPIOB, 15, GPIO_AF, 5, GPIO_NOPULL, true);   /* MOSI */
    gpio_config(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_OUT, 0, GPIO_NOPULL, true);
    cs_high();

    /* Mode 0, 54 MHz / 2 = 27 MHz (Lesebefehl 0x03 erlaubt bis 50 MHz) */
    spi_init(FLASH_SPI, 0, false);

    /* Falls der Flash im Stromsparmodus ist: aufwecken */
    cs_low();
    spi_xfer(FLASH_SPI, CMD_RELEASE_PD);
    cs_high();
    delay_us(50);

    cs_low();
    spi_xfer(FLASH_SPI, CMD_JEDEC_ID);
    uint8_t id[3];
    spi_read(FLASH_SPI, id, 3);
    cs_high();
    jedec_id = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];

    if (id[0] == 0x00 || id[0] == 0xFF || id[2] < 0x10 || id[2] > 0x20) {
        capacity = 0;
        return false;
    }
    capacity = 1u << id[2];               /* z. B. 0x18 -> 16 MB */
    if (capacity > MAX_CAPACITY) capacity = MAX_CAPACITY;
    return wait_ready(10, false);
}

uint32_t flash_jedec_id(void) { return jedec_id; }
uint32_t flash_capacity(void) { return capacity; }

bool flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (!capacity || addr + len > capacity) return false;
    if (!wait_ready(10, false)) return false;
    cs_low();
    send_addr(CMD_READ_DATA, addr);
    spi_read(FLASH_SPI, buf, len);
    cs_high();
    return true;
}

bool flash_program_page(uint32_t addr, const uint8_t *buf)
{
    if (!capacity || (addr % PAGE_SIZE) != 0 || addr + PAGE_SIZE > capacity) return false;
    if (!wait_ready(10, false)) return false;
    write_enable();
    cs_low();
    send_addr(CMD_PAGE_PROGRAM, addr);
    spi_write(FLASH_SPI, buf, PAGE_SIZE);
    cs_high();
    return wait_ready(5, false);          /* typ. 0.7 ms, max. 3 ms */
}

bool flash_erase_block64k(uint32_t addr)
{
    if (!capacity || (addr % 65536u) != 0 || addr >= capacity) return false;
    if (!wait_ready(10, false)) return false;
    write_enable();
    cs_low();
    send_addr(CMD_BLOCK_ERASE64, addr);
    cs_high();
    return wait_ready(3000, true);        /* typ. 150 ms, max. 2 s */
}

bool flash_erase_chip(void)
{
    if (!capacity) return false;
    if (!wait_ready(10, false)) return false;
    write_enable();
    cs_low();
    spi_xfer(FLASH_SPI, CMD_CHIP_ERASE);
    cs_high();
    return wait_ready(400000, true);      /* W25Q128: typ. 40 s, max. 200 s */
}
