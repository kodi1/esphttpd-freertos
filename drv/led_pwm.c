/*
 * led_pwm.c
 *
 *  Created on: Feb 8, 2016
 *      Author: xxx
 */
#include "led_pwm.h"

static led_pwm_t pwm;

void ICACHE_FLASH_ATTR led_pwm_init (uint32 time)
{
    uint8 i;

    DBG_LOG("PWM period (us): %d", time);

    pwm.period = time;
    if (!pwm.period) {
        pwm.period = 5000;    // us
    }

    for (i= 0; i < LED_CNT(&pwm.led); i++) {
        pwm.led.raw[i] = 0;
    }

    uint32 iocfg[LED_CNT(&pwm.led)][3] = {
        {PWM_0_OUT_IO_MUX,PWM_0_OUT_IO_FUNC,PWM_0_OUT_IO_NUM},
        {PWM_1_OUT_IO_MUX,PWM_1_OUT_IO_FUNC,PWM_1_OUT_IO_NUM},
        {PWM_2_OUT_IO_MUX,PWM_2_OUT_IO_FUNC,PWM_2_OUT_IO_NUM},
    };

    pwm_init(pwm.period,  pwm.led.raw, LED_CNT(&pwm.led), iocfg);

    led_pwm_set(&pwm.led);
}

void ICACHE_FLASH_ATTR led_pwm_set (led_t *led)
{
    uint8   i;

    for (i= 0; i < LED_CNT(led); i++) {
        led->raw[i] = CLIP(led->raw[i], 1023);
        DBG_LOG("Pwm ch: %d value %d", i, led->raw[i]);
        pwm_set_duty(PWM_INVERT(led->raw[i]), i);
    }

    pwm.led = *led;
    pwm_start();
}

void ICACHE_FLASH_ATTR led_pwm_get_cfg(led_pwm_t *p)
{
    *p = pwm;
}
