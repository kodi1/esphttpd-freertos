/*
 * led_pwm.h
 *
 *  Created on: Feb 8, 2016
 *      Author: xxx
 */

#ifndef LED_PWM_H_
#define LED_PWM_H_

#include <stdio.h>
#include <c_types.h>
#include <esp8266.h>
#include <pwm.h>
#include "debug.h"

#define PWM_0_OUT_IO_MUX    PERIPHS_IO_MUX_GPIO2_U
#define PWM_0_OUT_IO_FUNC   FUNC_GPIO2
#define PWM_0_OUT_IO_NUM    2
#define PWM_1_OUT_IO_MUX    PERIPHS_IO_MUX_GPIO4_U
#define PWM_1_OUT_IO_FUNC   FUNC_GPIO4
#define PWM_1_OUT_IO_NUM    4
#define PWM_2_OUT_IO_MUX    PERIPHS_IO_MUX_GPIO5_U
#define PWM_2_OUT_IO_FUNC   FUNC_GPIO5
#define PWM_2_OUT_IO_NUM    5

#define CLIP(a,b)       (a < b) ? a : b
#define PWM_INVERT(a)   ((~a) & 0x3ff)
#define LED_CNT(a)      (sizeof(*(a))/sizeof((a)->raw[0]))

typedef struct {
    uint32  r;
    uint32  g;
    uint32  b;
} color_t;

typedef struct {
    union {
        uint32  raw[0];
        color_t color;
    };
} led_t;

typedef struct {
    uint32  period;   //us
    led_t   led;
} led_pwm_t;

void ICACHE_FLASH_ATTR led_pwm_init (uint32 time);
void ICACHE_FLASH_ATTR led_pwm_set (led_t *led);
void ICACHE_FLASH_ATTR led_pwm_get_cfg(led_pwm_t *p);

#endif /* LED_PWM_H_ */
