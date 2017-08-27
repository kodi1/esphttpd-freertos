/*
 * ws2801.h
 *
 *  Created on: Feb 24, 2016
 *      Author: n0ll
 */

#ifndef WS2801_H_
#define WS2801_H_

#include <c_types.h>
#include "spi.h"
#include "debug.h"

void ICACHE_FLASH_ATTR ws2801_init(void);
void ICACHE_FLASH_ATTR ws2801_push(void *data, size_t size);

#endif /* WS2801_H_ */
