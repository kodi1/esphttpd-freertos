/*
 * led_exec.c
 *
 *  Created on: Feb 28, 2016
 *      Author: n0ll
 */

#include "led_exec.h"

#define COMAND_WAIT_TIME    (10)        // milliseconds
#define FADE_TIME_STEP      (200)       // milliseconds
#define MAX_IDLE_TIME       (1800)      // seconds
#define _IDLE_TIME (MAX_IDLE_TIME * (1000 / COMAND_WAIT_TIME))

#define _MAX(a,b)       (a > b) ? a : b
#define _MIN(a,b)       (a < b) ? a : b
#define _CLIP(a,b,c)    _MIN(_MAX(a, b), c)

typedef struct {
    void        *lock;
    char        *name;
    led_cmd_t   ctrl;
    uint32      val;
    uint8       sleep;
} led_exec_t;

static struct {
    led_exec_t  red;
    led_exec_t  green;
    led_exec_t  blue;
    uint32_t    idle_time;
    uint8       sleep;
} cmd_data = {
    .idle_time = 0,
    .sleep = 0,
    .red = {
        .name = "Red",
        .val = 1023,
        .sleep = 0,
        .ctrl = {
            .step = 10,
            .time = 100,
            .max = 950,
            .min = 3,
        },
    },
    .green = {
        .name = "Green",
        .val = 1023,
        .sleep = 0,
        .ctrl = {
            .step = 10,
            .time = 900,
            .max = 950,
            .min = 3,
        },
    },
    .blue = {
        .name = "Blue",
        .val = 1023,
        .sleep = 0,
        .ctrl = {
            .step = 10,
            .time = 300,
            .max = 950,
            .min = 3,
        },
    },
};

static xQueueHandle        queue;

int ICACHE_FLASH_ATTR cgi_rgb(HttpdConnData *connData)
{
    char tmp[32];
    DBG_LOG("Enter");

    if (connData->conn==NULL) {
        //Connection aborted. Clean up.
        return HTTPD_CGI_DONE;
    }

    led_exec_data_t *led_data = malloc(sizeof(*led_data));
    if (!led_data) {
        DBG_LOG("Can't allocate mem")
        return HTTPD_CGI_DONE;
    }

    DBG_LOG("Args: %", connData->getArgs);

    if(0 >= httpdFindArg(connData->getArgs, "id", tmp, sizeof(tmp))) {
        DBG_LOG("Error can't find id value");
        goto EXIT_1;
    }
    led_data->id = atoi(tmp);

    if(0 >= httpdFindArg(connData->getArgs, "max", tmp, sizeof(tmp))) {
        DBG_LOG("Error can't find max value");
        goto EXIT_1;
    }
    led_data->exc.max = atoi(tmp);

    if(0 >= httpdFindArg(connData->getArgs, "min", tmp, sizeof(tmp))) {
        DBG_LOG("Error can't find min value");
        goto EXIT_1;
    }
    led_data->exc.min = atoi(tmp);

    if(0 >= httpdFindArg(connData->getArgs, "step", tmp, sizeof(tmp))) {
        DBG_LOG("Error can't find step value");
        goto EXIT_1;
    }
    led_data->exc.step = atoi(tmp);

    if(0 >= httpdFindArg(connData->getArgs, "time", tmp, sizeof(tmp))) {
        DBG_LOG("Error can't find time value");
        goto EXIT_1;
    }
    led_data->exc.time = atoi(tmp);

    DBG_LOG("ID: %d Min: %d Max: d Step: %d Time: %d",
            led_data->id,
            led_data->exc.min,
            led_data->exc.max,
            led_data->exc.step,
            led_data->exc.time
            );

    xQueueSendToBack(*(void **)(connData->cgiArg), &led_data, portMAX_DELAY);

    httpdStartResponse(connData, 200);
    httpdHeader(connData, "Content-Type", "text/plain");
    httpdHeader(connData, "Content-Length", "4");
    httpdEndHeaders(connData);

    httpdSend(connData, "ok !", -1);
    DBG_LOG("Exit");
    return HTTPD_CGI_DONE;

EXIT_1:
    free(led_data);
    DBG_LOG("Exit error");

    httpdStartResponse(connData, 200);
    httpdHeader(connData, "Content-Type", "text/plain");
    httpdHeader(connData, "Content-Length", "7");
    httpdEndHeaders(connData);

    httpdSend(connData, "error !", -1);
    return HTTPD_CGI_DONE;
}

static void ICACHE_FLASH_ATTR led_task (void *arg)
{
    uint8           up_down = 0;
    led_exec_t      *d      = arg;
    uint32          t;

    while (1) {
        t = FADE_TIME_STEP;
        xSemaphoreTake(d->lock, portMAX_DELAY);
        if (!d->sleep) {
            t = d->ctrl.time;
            if (d->ctrl.max != d->ctrl.min) {
                d->val += up_down ? -d->ctrl.step : d->ctrl.step;
                d->val = _CLIP(d->ctrl.min, (sint32)d->val, d->ctrl.max);
                if (d->val >= d->ctrl.max) {
                    up_down = 1;
                }
                if (d->val <= d->ctrl.min) {
                    up_down = 0;
                }
            } else {
                d->val = d->ctrl.min;
            }
        } else {
            if(d->val) {
                d->val --;
            }
        }

        xSemaphoreGive(d->lock);
        DBG_LOG("%s: %d %d", d->name, d->val, up_down);
        vTaskDelay(ms_ticks(t));
    }
}

static void ICACHE_FLASH_ATTR sleep_cmd(uint8 sleep)
{
    DBG_LOG("Enter");

    xSemaphoreTake(cmd_data.red.lock, portMAX_DELAY);
    cmd_data.red.sleep = sleep;
    xSemaphoreGive(cmd_data.red.lock);

    xSemaphoreTake(cmd_data.green.lock, portMAX_DELAY);
    cmd_data.green.sleep =sleep;
    xSemaphoreGive(cmd_data.green.lock);

    xSemaphoreTake(cmd_data.blue.lock, portMAX_DELAY);
    cmd_data.blue.sleep = sleep;
    xSemaphoreGive(cmd_data.blue.lock);

    DBG_LOG("Exit");
}

static void ICACHE_FLASH_ATTR set_rgb(void)
{
    static led_t led;
    uint8   change = 0;

    DBG_LOG("Enter");

    xSemaphoreTake(cmd_data.red.lock, portMAX_DELAY);
    if (led.color.r != cmd_data.red.val) {
        led.color.r = cmd_data.red.val;
        change = 1;
    }
    xSemaphoreGive(cmd_data.red.lock);

    xSemaphoreTake(cmd_data.green.lock, portMAX_DELAY);
    if (led.color.g != cmd_data.green.val) {
        led.color.g = cmd_data.green.val;
        change = 1;
    }
    xSemaphoreGive(cmd_data.green.lock);

    xSemaphoreTake(cmd_data.blue.lock, portMAX_DELAY);
    if (led.color.b != cmd_data.blue.val) {
        led.color.b = cmd_data.blue.val;
        change = 1;
    }
    xSemaphoreGive(cmd_data.blue.lock);

    if (change) {
        led_pwm_set (&led);
    }
    DBG_LOG("Exit");
}

static void ICACHE_FLASH_ATTR apply_ctrl (led_exec_data_t *rcv)
{
    DBG_LOG("Enter");

    switch (rcv->id) {
    case EXEC_LED_SET_R:
        xSemaphoreTake(cmd_data.red.lock, portMAX_DELAY);
        cmd_data.red.ctrl = rcv->exc;
        xSemaphoreGive(cmd_data.red.lock);
        break;
    case EXEC_LED_SET_G:
        xSemaphoreTake(cmd_data.green.lock, portMAX_DELAY);
        cmd_data.green.ctrl = rcv->exc;
        xSemaphoreGive(cmd_data.green.lock);
        break;
    case EXEC_LED_SET_B:
        xSemaphoreTake(cmd_data.blue.lock, portMAX_DELAY);
        cmd_data.blue.ctrl = rcv->exc;
        xSemaphoreGive(cmd_data.blue.lock);
        break;
    default:
        DBG_LOG("Unknown id %d", rcv->id);
        break;
    }

    DBG_LOG("Exit");
}

static void ICACHE_FLASH_ATTR led_ctrl (void *arg)
{
//  wifi_set_sleep_type(NONE_SLEEP_T);
    led_pwm_init(0);

    while (1) {
        led_exec_data_t *rcv  = NULL;
        DBG_LOG("Wait for cmd ...");
        if(xQueueReceive(queue, &rcv, ms_ticks(COMAND_WAIT_TIME))) {
            cmd_data.idle_time = 0;

            apply_ctrl(rcv);
             if(cmd_data.sleep) {
                cmd_data.sleep = 0;
                sleep_cmd(cmd_data.sleep);
            }
        }

        if (!cmd_data.sleep) {
            if(_IDLE_TIME > cmd_data.idle_time) {
                cmd_data.idle_time ++;
            } else {
                DBG_LOG("Idle time reached");
                cmd_data.sleep = 1;
                sleep_cmd(cmd_data.sleep);
            }
        }

        set_rgb();

        if (rcv) {
            free(rcv);
        }
    }
}

void ICACHE_FLASH_ATTR init_led_exec()
{
    queue = xQueueCreate(MAX_CMD_LEN, sizeof(void *));
    if (!queue) {
        DBG_LOG("Cant create cmd_queue");
        return;
    }

    xTaskCreate(&led_ctrl, "led_ctrl", 512, NULL, 2, NULL);

    cmd_data.red.lock = xSemaphoreCreateMutex();
    cmd_data.green.lock = xSemaphoreCreateMutex();
    cmd_data.blue.lock = xSemaphoreCreateMutex();

    xTaskCreate(&led_task, "red", 512, &cmd_data.red, 1, NULL);
    xTaskCreate(&led_task, "green", 512, &cmd_data.green, 1, NULL);
    xTaskCreate(&led_task, "blue", 512, &cmd_data.blue, 1, NULL);
}
