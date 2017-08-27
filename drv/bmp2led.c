/*
 * bmp2led.c
 *
 *  Created on: Mar 19, 2016
 *      Author: n0ll
 */

#include <esp8266.h>
#include "bmp2led.h"

struct create_data {
    void    *q_data;
};

pixel_t rgb_data[RGB_BMP_W * RGB_BMP_H];

static xSemaphoreHandle    sem;
static xQueueHandle        q;

static int8 ICACHE_FLASH_ATTR bmp_data_push (void *d, uint32 len)
{
    int8        ret    = 0;
    bmp_data_t  *img    = d;
    void        *data   = NULL;

    DBG_LOG("Enter");

    if (!q || !img) {
        DBG_LOG("Null pointer");
        ret = -1;
        goto EXIT_1;
    }

    if (img->header.type != 0x4d42) {
        DBG_LOG("Wrong data type 0x%x", img->header.type);
        ret = -2;
        goto EXIT_1;
    }

    if (len != img->header.size) {
        uint8 *c = d;
        DBG_LOG("Data size error %d != %d", img->header.size, len);
        ret = -3;
        goto EXIT_1;
    }

    if ((img->info.height != RGB_BMP_H) || (img->info.width != RGB_BMP_W)) {
        DBG_LOG("Wrong size image %d x %d", img->info.width, img->info.height);
        ret = -4;
        goto EXIT_1;
    }

//    if (img->info.imagesize != sizeof(rgb_data)) {
//        DBG_LOG("Wrong data size %d", img->info.imagesize);
//        ret = -5;
//        goto EXIT_1;
//    }

    data = malloc(sizeof(rgb_data));
    if (!data) {
        DBG_LOG("Can't allocate memory size %d", sizeof(rgb_data));
        ret = -6;
        goto EXIT_1;
    }

    memcpy(data, img->bytes + img->header.offset, sizeof(rgb_data));

    if (pdTRUE != xQueueSendToBack(q, &data, portMAX_DELAY)) {
        DBG_LOG("Can't push data to queue");
        ret = -7;
        free(data);
        goto EXIT_1;
    }

EXIT_1:
    DBG_LOG("Exit %s", ret ? "Err" : "Ok");
    return ret;
}

static void push_data(void)
{
    static portBASE_TYPE wake = pdFALSE;
    ws2812_push(&rgb_data, sizeof(rgb_data));
    xSemaphoreGiveFromISR(sem, &wake);
}

static void ICACHE_FLASH_ATTR bmp_task (void *arg)
{
    uint32 pw_down_cnt      = 0;
    int8  w, h;
    pixel_t *_rgb;

    vSemaphoreCreateBinary(sem);
    DBG_LOG("Task create. Led buff size %d", sizeof(rgb_data));

    hw_timer_init(0);
    hw_timer_set_func(&push_data);

    memset(rgb_data, 0x0f, sizeof(rgb_data));
    ws2812_init();
    hw_timer_arm(50);

    while (1) {
        bmp_pixel_t *rcv  = NULL;
        if(pdFALSE == xQueueReceive(q, &rcv, _ms(BMP2LED_WAIT_TIME))) {
            if (_IDLE_CNT > pw_down_cnt++) {
                continue;
            }
            DBG_LOG("Shutdown Leds");
            memset(rgb_data, 0x00, sizeof(rgb_data));
        } else {
            xSemaphoreTake(sem, portMAX_DELAY);
            _rgb = rgb_data;
            for (w = 0; w < RGB_BMP_W; w += 2) {
                for (h = RGB_BMP_H - 1; h >= 0; h--) {
                    _rgb->r = rcv[(h * RGB_BMP_W) + w].r;
                    _rgb->g = rcv[(h * RGB_BMP_W) + w].g;
                    _rgb->b = rcv[(h * RGB_BMP_W) + w].b;
                    _rgb++;
                }
                _rgb += RGB_BMP_H;
            }

            _rgb = rgb_data + RGB_BMP_H;
            for (w = 1; w < RGB_BMP_W; w += 2) {
                for (h = 0; h < RGB_BMP_H; h++) {
                    _rgb->r = rcv[(h * RGB_BMP_W) + w].r;
                    _rgb->g = rcv[(h * RGB_BMP_W) + w].g;
                    _rgb->b = rcv[(h * RGB_BMP_W) + w].b;
                    _rgb++;
                }
                _rgb += RGB_BMP_H;
            }
        }

        hw_timer_arm(50);

        pw_down_cnt = 0;
        if (rcv) {
            free(rcv);
        }

        DBG_LOG("Free mem: %d", system_get_free_heap_size());
        DBG_LOG("Wait for data ...");
    }
}

static void bmp_task_tst (void *arg)
{
    uint8 cnt = 0;
    uint32 i;

    DBG_LOG("Enter");
    ws2812_init();

    while (1) {
        if (cnt & 1) {
            memset(rgb_data, 0x03, sizeof(rgb_data));
        } else {
            memset(rgb_data, 0x00, sizeof(rgb_data));
        }

        taskENTER_CRITICAL();
        ws2812_push(rgb_data, sizeof(rgb_data));
        taskEXIT_CRITICAL();

        vTaskDelay(_ms(300));
        cnt++;
    }
}
void ICACHE_FLASH_ATTR init_rgb2led_exec(void)
{
    static struct create_data c;

    DBG_LOG("Enter");

    q = xQueueCreate(MAX_BMP2LED_DATA, sizeof(void *));
    if (!q) {
        DBG_LOG("Cant create cmd_queue");
        return;
    }

    xTaskCreate(&bmp_task, "bmp2led", 512, NULL, 8, NULL);
//    xTaskCreate(&bmp_task_tst, "bmp2led_tst", 256, NULL, 8, NULL);
    DBG_LOG("Exit");
}

int ICACHE_FLASH_ATTR cgi_bmp2led(HttpdConnData *connData)
{
    DBG_LOG("Enter");

    if (connData->conn == NULL) {
        //Connection aborted. Clean up.
        return HTTPD_CGI_DONE;
    }

    bmp_data_push(connData->post->buff, connData->post->buffLen);

    httpdStartResponse(connData, 200);
    httpdHeader(connData, "Content-Type", "text/plain");
    httpdHeader(connData, "Content-Length", "4");
    httpdEndHeaders(connData);

    httpdSend(connData, "ok !", -1);
    DBG_LOG("Exit");
    return HTTPD_CGI_DONE;
}
