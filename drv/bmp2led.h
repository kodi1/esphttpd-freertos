/*
 * bmp2led.h
 *
 *  Created on: Mar 19, 2016
 *      Author: n0ll
 */

#ifndef DRV_BMP2LED_H_
#define DRV_BMP2LED_H_

#include <c_types.h>

#include "bmp.h"
#include "debug.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "httpd.h"
#include "ws2812.h"

#define RGB_BMP_W   (32)
#define RGB_BMP_H   (8)

#define MAX_BMP2LED_DATA    (4)
#define BMP2LED_WAIT_TIME   (20)    // milliseconds
#define BMP2LED_POWER_DOWN  (6)   // seconds
#define _IDLE_CNT           ((BMP2LED_POWER_DOWN * 1000) / BMP2LED_WAIT_TIME)
#define _ms(a)              ((a) / portTICK_RATE_MS)

void ICACHE_FLASH_ATTR init_rgb2led_exec();
int ICACHE_FLASH_ATTR cgi_bmp2led(HttpdConnData *connData);

typedef enum {
    BMP_DATA,
    RAW_DATA,
} data_type_t;

typedef struct {
    data_type_t type;
    void        *data;
} data_pixel_t;

#endif /* DRV_BMP2LED_H_ */
