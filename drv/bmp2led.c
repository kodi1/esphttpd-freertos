/*
 * bmp2led.c
 *
 *  Created on: Mar 19, 2016
 *      Author: n0ll
 */

#include <esp8266.h>
#include "bmp2led.h"
#include "lwip/lwip/sockets.h"

pixel_t rgb_data[RGB_BMP_W * RGB_BMP_H];
#define MAX_DATA_SIZE (sizeof(rgb_data))
static char rx_buff[MAX_DATA_SIZE];

static xSemaphoreHandle    sem;
static xQueueHandle        q;

static int8 ICACHE_FLASH_ATTR bmp_data_push (void *d, uint32 len)
{
    int8            ret    = 0;
    bmp_data_t      *img    = d;
    data_pixel_t    *msg;

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

    msg = malloc(sizeof(data_pixel_t));
    if (!msg) {
        DBG_LOG("Can't allocate memory size %d", sizeof(data_pixel_t));
        ret = -5;
        goto EXIT_1;
    }

    msg->type = BMP_DATA;
    msg->data = malloc(sizeof(rgb_data));
    if (!msg->data) {
        DBG_LOG("Can't allocate memory size %d", sizeof(rgb_data));
        ret = -6;
        goto EXIT_1;
    }

    memcpy(msg->data, img->bytes + img->header.offset, sizeof(rgb_data));
    if (pdTRUE != xQueueSendToBack(q, &msg, portMAX_DELAY)) {
        DBG_LOG("Can't push data to queue");
        ret = -7;
        free(msg->data);
        free(msg);
    }

EXIT_1:
    DBG_LOG("Exit %s", ret ? "Err" : "Ok");
    return ret;
}

static int8 ICACHE_FLASH_ATTR raw_data_push (void *d)
{
    int8            ret    = 0;
    data_pixel_t    *msg;

    DBG_LOG("Enter");

    if (!q || !d) {
        DBG_LOG("Null pointer");
        ret = -1;
        goto EXIT_1;
    }

    msg = malloc(sizeof(data_pixel_t));
    if (!msg) {
        DBG_LOG("Can't allocate memory size %d", sizeof(data_pixel_t));
        ret = -5;
        goto EXIT_1;
    }

    msg->type = RAW_DATA;
    msg->data = malloc(sizeof(rgb_data));
    if (!msg->data) {
        DBG_LOG("Can't allocate memory size %d", sizeof(rgb_data));
        ret = -6;
        goto EXIT_1;
    }

    memcpy(msg->data, d, sizeof(rgb_data));
    if (pdTRUE != xQueueSendToBack(q, &msg, portMAX_DELAY)) {
        DBG_LOG("Can't push data to queue");
        ret = -7;
        free(msg->data);
        free(msg);
    }

EXIT_1:
    DBG_LOG("Exit %s", ret ? "Err" : "Ok");
    return ret;
}

static void push_data(void)
{
    static portBASE_TYPE wake = pdFALSE;
    vPortEnterCritical();
    ws2812_push(&rgb_data, sizeof(rgb_data));
    vPortExitCritical();
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
    hw_timer_arm(5);

    while (1) {
        data_pixel_t *rcv_data  = NULL;
        if(pdFALSE == xQueueReceive(q, &rcv_data, _ms(BMP2LED_WAIT_TIME))) {
            if (_IDLE_CNT > pw_down_cnt++) {
                continue;
            }
            DBG_LOG("Shutdown Leds");
            xSemaphoreTake(sem, portMAX_DELAY);
            memset(rgb_data, 0x00, sizeof(rgb_data));
        } else {
            xSemaphoreTake(sem, portMAX_DELAY);
            if (BMP_DATA == rcv_data->type) {
                _rgb = rgb_data;
                bmp_pixel_t *rcv = rcv_data->data;
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
            }  else {
                memcpy(rgb_data, rcv_data->data, sizeof(rgb_data));
            }
        }

        hw_timer_arm(5);

        pw_down_cnt = 0;
        if (rcv_data && rcv_data->data) {
            free(rcv_data->data);
            free(rcv_data);
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

        ws2812_push(rgb_data, sizeof(rgb_data));

        vTaskDelay(_ms(300));
        cnt++;
    }
}

static void ICACHE_FLASH_ATTR udp_raw_task (void *arg)
{
#ifdef PRINT_TIME_DBG
    uint32  time = 0;
#endif
    struct sockaddr_in      _local;
    struct sockaddr_in      _remote;
    int _socket, recv_len;
    size_t _rlen = sizeof(_remote);
    void *out = NULL;

    DBG_LOG("Enter port:  55555");

    // zero out the structure
    memset(&_local, 0, sizeof(_local));
    _local.sin_family = AF_INET;
    _local.sin_addr.s_addr = INADDR_ANY;
    _local.sin_port = htons(55555);
    _local.sin_len = sizeof(_local);

    //create a UDP socket
    while (1) {
        _socket=socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (_socket == -1) {
            DBG_LOG("Failed to create socket");
            vTaskDelay(1000/portTICK_RATE_MS);
            continue;
        }
        break;
    };
    DBG_LOG("Socket: %d", _socket);

    //bind socket to port
    while (1) {
        if(bind(_socket , (struct sockaddr *)&_local, sizeof(_local)) == -1) {
            DBG_LOG("Failed bind socket to port");
            vTaskDelay(1000/portTICK_RATE_MS);
            continue;
        }
        break;
    };
    DBG_LOG("Bind to port: %d", _local.sin_port);

    //keep listening for data
    while(1)
    {
        DBG_LOG("%d Waiting for data...");
#ifdef PRINT_TIME_DBG
        DBG_LOG("Proc time: %d us", system_get_time() - time);
        DBG_LOG("Free mem: %d", system_get_free_heap_size());
#endif

        //try to receive some data, this is a blocking call
        recv_len = recvfrom(_socket, rx_buff, MAX_DATA_SIZE, 0,  (struct sockaddr *)&_remote, &_rlen);
        if (-1 == recv_len) {
            DBG_LOG("Receive data error exit");
            break;
        }

        DBG_LOG("Received packet from %s:%d", inet_ntoa(_remote.sin_addr), ntohs(_remote.sin_port));

#ifdef PRINT_TIME_DBG
        time = system_get_time();
#endif
        if (MAX_DATA_SIZE >= recv_len) {
            raw_data_push(rx_buff);
        } else {
            DBG_LOG("Len: %d from %d", recv_len, MAX_DATA_SIZE);
        }

        if (out) {
            //now reply the client with the same data
            DBG_LOG("Reply %s Len %d", out, strlen(out));
            int ret = sendto(_socket, out, strlen(out), 0,  (struct sockaddr *)&_remote, sizeof(_remote));
            free(out);  // json reply message
            out = NULL;

            if (-1 == ret) {
                DBG_LOG("Send data error exit");
                break;
            }
        }
    }
    close(_socket);
}

void ICACHE_FLASH_ATTR init_rgb2led_exec(void)
{
    DBG_LOG("Enter");

    q = xQueueCreate(MAX_BMP2LED_DATA, sizeof(void *));
    if (!q) {
        DBG_LOG("Cant create cmd_queue");
        return;
    }

    xTaskCreate(&bmp_task, "bmp2led", 512, NULL, 8, NULL);
    xTaskCreate(&udp_raw_task, "raw2led", 512, NULL, 8, NULL);
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
