/*
 * led_exec.h
 *
 *  Created on: Feb 28, 2016
 *      Author: n0ll
 */

#ifndef DRV_LED_EXEC_H_
#define DRV_LED_EXEC_H_

#include "c_types.h"
#include "led_pwm.h"
#include "debug.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "httpd.h"

#define MAX_CMD_LEN         (4)
#define ms_ticks(a)         ((a) / portTICK_RATE_MS)

typedef enum {
    EXEC_LED_NOP,
    EXEC_LED_SET_R,
    EXEC_LED_SET_G,
    EXEC_LED_SET_B,
} led_cmd_id_t;

typedef struct {
    uint32      step;
    uint32      time;
    uint32      max;
    uint32      min;
} led_cmd_t;

typedef struct {
    led_cmd_id_t    id;
    led_cmd_t       exc;
} led_exec_data_t;

void ICACHE_FLASH_ATTR init_led_exec();
int ICACHE_FLASH_ATTR cgi_rgb(HttpdConnData *connData);

#endif /* DRV_LED_EXEC_H_ */
