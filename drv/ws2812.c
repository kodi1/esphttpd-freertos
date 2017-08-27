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
    spi_clock(SPI_DEV, 4, 5); // 4Mhz @ 80 Mhz
    spi_tx_byte_order(SPI_DEV, SPI_BYTE_ORDER_HIGH_TO_LOW);
    spi_rx_byte_order(SPI_DEV, SPI_BYTE_ORDER_HIGH_TO_LOW);

    SET_PERI_REG_MASK(SPI_USER(SPI_DEV), SPI_CS_SETUP|SPI_CS_HOLD);
    CLEAR_PERI_REG_MASK(SPI_USER(SPI_DEV), SPI_FLASH_MODE);

    DBG_LOG("Exit");
}

void IRAM_ATTR ws2812_push(void *data, size_t size)
{
    void *nmi_isr;
    uint8   *_d = data;
    uint32  b;
    uint8 bits[] = {
        0x88, // 0
        0x8E, // 1
        0xE8, // 2
        0xEE, // 3
    };

//    DBG_LOG("Enter");
    __asm__ __volatile__ (
          "j function_entry\n"

          ".align 128\n"
          "vecbase_mod:\n"
          "nop\n"

          ".align 16\n"
          "debug_exception_mod:\n"
          "nop\n"

          ".align 16\n"
          "nmi_exception_mod:\n"
          "rfi 3\n"

          "function_entry:\n"
          "rsr.vecbase %0\n"
          "movi a2, vecbase_mod\n"
          "wsr.vecbase a2\n"

          : "=r" (nmi_isr)
          :
          : "a2", "memory"
    );

    while (size--) {
        b = (
                bits[(*_d) & 0x3] |
                bits[((*_d) >> 2) & 0x3] << 8 |
                bits[((*_d) >> 4) & 0x3] << 16 |
                bits[((*_d) >> 6) & 0x3] << 24
        );
        spi_tx32(SPI_DEV, b);
        _d++;
    }

    __asm__ __volatile__ (
          "wsr.vecbase %0\n"            // restore original vecbase
          :
          : "r" (nmi_isr)
          : "memory"
    );

    wDev_MacTim1Arm(1);

//    DBG_LOG("Exit");
}
