/*
 * ws2812.h
 *
 *  Created on: Feb 24, 2016
 *      Author: n0ll
 */

#ifndef WS2812_H_
#define WS2812_H_

#include <c_types.h>
#include "spi.h"
#include "debug.h"

typedef struct {
    uint8   g;
    uint8   r;
    uint8   b;
} pixel_t;

void ICACHE_FLASH_ATTR ws2812_init(void);
void ICACHE_FLASH_ATTR ws2812_push(void *data, size_t size);

#endif /* WS2812_H_ */
