/*
 * spi.h - einfacher SPI-Master (Polling, 8 Bit) fuer SPI2 (Flash) und SPI3 (Gyro)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "board.h"

/* br: Teiler-Bits 0..7 -> APB / 2^(br+1); mode3: CPOL=1, CPHA=1 */
void    spi_init(SPI_TypeDef *spi, uint32_t br, bool mode3);
void    spi_set_prescaler(SPI_TypeDef *spi, uint32_t br);
uint8_t spi_xfer(SPI_TypeDef *spi, uint8_t out);
void    spi_read(SPI_TypeDef *spi, uint8_t *buf, uint32_t len);
void    spi_write(SPI_TypeDef *spi, const uint8_t *buf, uint32_t len);
