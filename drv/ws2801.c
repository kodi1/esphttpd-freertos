/*
 * ws2801.c
 *
 *  Created on: Feb 24, 2016
 *      Author: n0ll
 */

#include "ws2801.h"
#define SPI_DEV HSPI

void ICACHE_FLASH_ATTR ws2801_init(void)
{
    DBG_LOG("Enter");

    //init SPI bus
    spi_init_gpio(SPI_DEV, SPI_CLK_USE_DIV);
    spi_clock(SPI_DEV, 4, 10); // 2MHz @ 80MHz
    spi_tx_byte_order(SPI_DEV, SPI_BYTE_ORDER_HIGH_TO_LOW);
    spi_rx_byte_order(SPI_DEV, SPI_BYTE_ORDER_HIGH_TO_LOW);

    SET_PERI_REG_MASK(SPI_USER(SPI_DEV), SPI_CS_SETUP|SPI_CS_HOLD);
    CLEAR_PERI_REG_MASK(SPI_USER(SPI_DEV), SPI_FLASH_MODE);

    DBG_LOG("Exit");
}

void ws2801_push(void *data, size_t size)
{
    uint32  *_d = data;
    DBG_LOG("Enter");

    if (size & 0x3) {
        DBG_LOG("Data size is not multiple of 4");
        return;
    }

    while (size) {
        spi_tx32(SPI_DEV, *_d);

        _d ++;
        size -= 4;
    }

    DBG_LOG("Exit");
}
