/*
 * ws2812.c
 *
 *  Created on: Feb 24, 2016
 *      Author: n0ll
 */

#include "ws2812.h"
#define SPI_DEV HSPI

void ICACHE_FLASH_ATTR ws2812_init(void)
{
    DBG_LOG("Enter");

    //init SPI bus
    spi_init_gpio(SPI_DEV, SPI_CLK_USE_DIV);
    spi_clock(SPI_DEV, 5, 5); // 3.2Mhz @ 80 Mhz
    spi_tx_byte_order(SPI_DEV, SPI_BYTE_ORDER_HIGH_TO_LOW);
    spi_rx_byte_order(SPI_DEV, SPI_BYTE_ORDER_HIGH_TO_LOW);

    SET_PERI_REG_MASK(SPI_USER(SPI_DEV), SPI_CS_SETUP|SPI_CS_HOLD);
    CLEAR_PERI_REG_MASK(SPI_USER(SPI_DEV), SPI_FLASH_MODE);

    DBG_LOG("Exit");
}

void ws2812_push(void *data, size_t size)
{
    uint8   *_d = data;
    uint32  b;
    uint8 bits[] = {
        0x88, // 0
        0x8E, // 1
        0xE8, // 2
        0xEE, // 3
    };

//    DBG_LOG("Enter");

    while (size--) {
        b = (
                bits[(*_d) & 0x3] |
                bits[((*_d) >> 2) & 0x3] << 8 |
                bits[((*_d) >> 4) & 0x3] << 16 |
                bits[((*_d) >> 6) & 0x3] << 24
        );

        spi_tx32(SPI_DEV, b);
        _d ++;
    }

//    DBG_LOG("Exit");
}
