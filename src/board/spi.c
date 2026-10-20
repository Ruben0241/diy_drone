#include "spi.h"

void spi_init(SPI_TypeDef *spi, uint32_t br, bool mode3)
{
    spi->CR1 = 0;
    spi->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (br << SPI_CR1_BR_Pos) |
               (mode3 ? (SPI_CR1_CPOL | SPI_CR1_CPHA) : 0u);
    spi->CR2 = (7u << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;   /* 8 Bit, RXNE bei 8 Bit */
    spi->CR1 |= SPI_CR1_SPE;
}

void spi_set_prescaler(SPI_TypeDef *spi, uint32_t br)
{
    spi->CR1 &= ~SPI_CR1_SPE;
    spi->CR1 = (spi->CR1 & ~SPI_CR1_BR) | (br << SPI_CR1_BR_Pos);
    spi->CR1 |= SPI_CR1_SPE;
}

uint8_t spi_xfer(SPI_TypeDef *spi, uint8_t out)
{
    while (!(spi->SR & SPI_SR_TXE)) { }
    *(volatile uint8_t *)&spi->DR = out;     /* 8-Bit-Zugriff, sonst 16 Bit */
    while (!(spi->SR & SPI_SR_RXNE)) { }
    return *(volatile uint8_t *)&spi->DR;
}

void spi_read(SPI_TypeDef *spi, uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) buf[i] = spi_xfer(spi, 0xFF);
}

void spi_write(SPI_TypeDef *spi, const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) (void)spi_xfer(spi, buf[i]);
}
