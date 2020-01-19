/*
 * sensors.c
 *
 *  Created on: Apr 30, 2017
 *      Author: n0ll
 */

#include <esp8266.h>
#include "httpd.h"
#include "httpdespfs.h"
#include "../user/io.h"
#include "espfs.h"
#include "captdns.h"
#include "webpages-espfs.h"
#include "debug.h"
#include "gpio.h"
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "lwip/lwip/sockets.h"
#include "lwip/lwip/netdb.h"

#include "json/cJSON.h"

// GPOI 4, 5, 2 - led
// MTDO, MTDI

#if !defined HATOKEN || !defined HAHOST
#error 'HATOKEN and HAHOST not set'
#endif
#ifndef HAPORT
#define HAPORT 8123
#endif

#define _GPIO_FUNC(a,b) (a##b)
#define GPIO_FUNC(a) _GPIO_FUNC(FUNC_GPIO,a)
#define _STR(x) #x
#define STR(x) _STR(x)

#define DIVD_POWER  4
#define DHT_POWER   5
#define DHT_DATA    12
#define SLEEP_EN    13
#define LED_PIN     2

#define MUX_DIVD_POWER PERIPHS_IO_MUX_GPIO4_U
#define MUX_DHT_POWER  PERIPHS_IO_MUX_GPIO5_U
#define MUX_DHT_DATA   PERIPHS_IO_MUX_MTDI_U
#define MUX_SLEEP_EN   PERIPHS_IO_MUX_MTCK_U

#define SLEEP_SECONDS   (1000 * 1000 * 600)
#define _ms(a)          ((a) / portTICK_RATE_MS)
#define _ADC            (3400.0 / 830.0)
#define ADC_TO_V(r)     (((_ADC * r * 1000) / 1024.0) - 77)

#define clip(a, b, c)   ((a > b) ? a : ((b > c) ? c : b))
#define percent(a, b, c) (((b - a) * 100) / (c - a))

#define MAX_BATT (4000)
#define MIN_BATT (2900)

#define MAXLINE (1024)
char _temp[MAXLINE + 1];

static xSemaphoreHandle xconnect = NULL;
static xSemaphoreHandle xdetect = NULL;
static xQueueHandle xdataq = NULL;
static uint32_t addr = 0;
static uint8_t channel = 0;

volatile uint32_t sens_detected = 0;

typedef struct
{
    int16_t     temperature;
    uint16_t    humidity;
    int16_t     temp_idx;
    uint32_t    vbatt;
} dht_data_t;

//http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
static ICACHE_FLASH_ATTR int16_t heat_idx (int16_t t, uint16_t h)
{
#define C_TO_F(x) ((x * 1.8) + 32)
#define F_TO_C(x) ((x - 32.0) / 1.8)
    float idx;
    float _t = C_TO_F(t / 10.0);
    float _h = h / 10.0;

    if (80.0 < _t) {
        idx = -42.379 +
                (2.04901523 * _t) +
                (10.14333127 * _h) -
                (.22475541 * _t * _h) -
                (.00683783 * _t * _t) -
                (.05481717 *  _h * _h) +
                (.00122874 * _t * _t * _h) +
                (.00085282 * _t * _h * _h) -
                (.00000199 * _t * _t * _h * _h);

        if( (13.0 > _h) &&
            (112.0 > _t)
            ) {
            idx -= (((13 - _h) / 4) * sqrt((17 - fabs(_t - 95))/17));
        }

        if ( (85.0 < _h) &&
             (87.0 > _t)
            ) {
            idx += (((_h - 85) / 10) * ((87 - _t) / 5));
        }

    } else {
        idx = 0.5 * (_t + 61.0 + ((_t - 68.0) * 1.2) + (_h * 0.094));
    }

    return (int16_t)(F_TO_C(idx) * 10);
}

//https://github.com/nekromant/esp8266-frankenstein/blob/master/src/cmd_dht22.c
static ICACHE_FLASH_ATTR dht_data_t dht_read(void)
{
#define DHT_MAXTIMINGS  40
#define DHT_READTIMEOUT 250
#define DHT_BREAKTIME   20
    dht_data_t ret = {0};
    uint8 data[5] = {0};

    vTaskDelay(_ms(500));           // Wait sensor
    GPIO_OUTPUT_SET(DHT_DATA, 0);   // low for 20ms
    vTaskDelay(_ms(20));            // MCU start condition

    portENTER_CRITICAL();

    GPIO_DIS_OUTPUT(DHT_DATA);
    os_delay_us(40);

    uint8 i = 0;
    uint8 bits = 0;
    bool laststate = 1;
    do{
        uint8 counter = 0;
        while (GPIO_INPUT_GET(DHT_DATA) == laststate)
        {
            os_delay_us(1);
            if (++counter > DHT_READTIMEOUT){
                portEXIT_CRITICAL();
                xSemaphoreGive(xdetect);
                DBG_LOG("DHT reading timeout on bit %d", bits);
                return ret;
            }
        }

        laststate = GPIO_INPUT_GET(DHT_DATA);

        // skip first 3 bits
        if ((i >= 4) && (i%2 == 0)) {
            data[bits/8] <<= 1;
            if (counter > DHT_BREAKTIME)
                data[bits/8] |= 1;
            if(++bits >= 40) break;
        }
    }while(++i);

    portEXIT_CRITICAL();

    sens_detected = 1;
    DBG_LOG("DHT detected")
    xSemaphoreGive(xdetect);

    if (bits >= 40) {
        uint8 checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
        if (data[4] == checksum) {
            ret.humidity = data[0] * 256 + data[1];
            ret.temperature = (data[2] & 0x7f) * 256 + data[3];
            if(data[2] & 0x80) ret.temperature = -ret.temperature;

            ret.temp_idx = heat_idx(ret.temperature, ret.humidity);
            return ret;
        }
        DBG_LOG("Checksum mismatch. Expected %d got %d. Data: %02x %02x %02x %02x",
                data[4], checksum, data[0], data[1], data[2], data[3]);
    } else {
        DBG_LOG("Want to read 40 bits, got: %d", bits);
    }
    return ret;
}

static void ICACHE_FLASH_ATTR init_gpio(void)
{
    PIN_FUNC_SELECT(MUX_DIVD_POWER, GPIO_FUNC(DIVD_POWER));
    PIN_FUNC_SELECT(MUX_DHT_POWER , GPIO_FUNC(DHT_POWER));
    PIN_FUNC_SELECT(MUX_DHT_DATA  , GPIO_FUNC(DHT_DATA));
    PIN_FUNC_SELECT(MUX_SLEEP_EN  , GPIO_FUNC(SLEEP_EN));

    PIN_PULLUP_EN(MUX_SLEEP_EN);
    GPIO_AS_INPUT(SLEEP_EN);

    PIN_PULLUP_EN(MUX_DHT_DATA);
    GPIO_AS_INPUT(DHT_DATA);

    // disable on board led
    GPIO_OUTPUT_SET(LED_PIN, 1);

    return;
}

static inline uint32_t dont_sleep(void)
{
    return !sens_detected;
}

static void ICACHE_FLASH_ATTR goto_sleep(void)
{
    if (dont_sleep()) {
        DBG_LOG("Sleep disabled");
        goto exit_1;
    }

    DBG_LOG("Get some sleep");
    system_deep_sleep(SLEEP_SECONDS);
exit_1:
    vTaskDelay(_ms(2000));
}

static void wifi_cb(System_Event_t *e)
{
    switch (e->event_id) {
        case EVENT_STAMODE_CONNECTED:
            channel = e->event_info.connected.channel;
        case EVENT_STAMODE_GOT_IP:
            addr = e->event_info.got_ip.ip.addr;
            xSemaphoreGive(xconnect);
            break;
        case EVENT_STAMODE_DHCP_TIMEOUT:
            DBG_LOG("Timeout DHCP");
            goto_sleep();
            break;
        default:
            DBG_LOG("EventId %d", e->event_id);
    }
}

static inline void* ICACHE_FLASH_ATTR _send(struct sockaddr_in *_in,
                                        char        *data,
                                        uint32_t    size)
{
    uint32_t n;
    int     fd;
    void    *d = NULL;

    DBG_LOG("Data out: %s", data);

    fd = socket(AF_INET, SOCK_STREAM, 0);

    if (-1 == fd) {
        DBG_LOG("Error fd");
        goto exit_1;
    }

    if(0 != connect(fd, (struct sockaddr *) _in, sizeof(*_in))) {
        DBG_LOG("Error connect");
        goto exit_2;
    }

    write(fd, data, strlen(data));
    while ((n = read(fd, data, size)) > 0) {
        data[n] = '\0';
        d = data;
        DBG_LOG("Data in: %s", d);
    }

exit_2:
    close(fd);
exit_1:
    return d;
}

static inline ICACHE_FLASH_ATTR void _ha_post (
                                                 uint8 const    *name,
                                                 dht_data_t     *res,
                                                 void           *servin)
{
    char saddr[16];
    void *_data_payload;
    void *_payload;
    void *jspayload = cJSON_CreateObject();
    void *jsattr = cJSON_CreateObject();

    float batt_p = clip(MIN_BATT, res->vbatt, MAX_BATT);
    batt_p = percent(MIN_BATT, batt_p, MAX_BATT);

    cJSON_AddItemToObject(jspayload, "topic", cJSON_CreateString(name));
    cJSON_AddItemToObject(jspayload, "retain", cJSON_CreateBool(0));

    cJSON_AddItemToObject(jsattr, "temperature", cJSON_CreateNumber(res->temperature / 10.0));
    cJSON_AddItemToObject(jsattr, "batt_v", cJSON_CreateNumber(res->vbatt));
    cJSON_AddItemToObject(jsattr, "batt_p", cJSON_CreateNumber(batt_p));
    cJSON_AddItemToObject(jsattr, "temp_idx", cJSON_CreateNumber(res->temp_idx / 10.0));
    cJSON_AddItemToObject(jsattr, "humidity", cJSON_CreateNumber(res->humidity / 10.0));
    cJSON_AddItemToObject(jsattr, "rssi", cJSON_CreateNumber(wifi_station_get_rssi()));
    cJSON_AddItemToObject(jsattr, "addr", cJSON_CreateString(ipaddr_ntoa_r((void *)&addr, saddr, sizeof(saddr))));
    cJSON_AddItemToObject(jsattr, "channel", cJSON_CreateNumber(channel));

    cJSON_Minify(jsattr);
    _payload = cJSON_PrintUnformatted(jsattr);
    cJSON_Delete(jsattr);

    cJSON_AddItemToObject(jspayload, "payload", cJSON_CreateString(_payload));

    cJSON_Minify(jspayload);
    _data_payload = cJSON_PrintUnformatted(jspayload);
    cJSON_Delete(jspayload);

    snprintf(_temp,
                MAXLINE,
                "POST /api/services/mqtt/publish HTTP/1.0\r\n"
                "Authorization: Bearer %s\r\n"
                "Content-Type: application/json\r\n"
                "Content-length: %d\r\n\r\n"
                "%s",
                STR(HATOKEN),
                strlen(_data_payload),
                _data_payload
            );

    _send(servin, _temp, MAXLINE);
    free(_data_payload);
    free(_payload);
}

static void ICACHE_FLASH_ATTR task_send (void *arg)
{
    char **pptr;
    char str[16];
    struct hostent *hptr;
    dht_data_t res;
    struct sockaddr_in servin;

    if(xSemaphoreTake(xconnect, _ms(10000))) {
        DBG_LOG("Have IP %d", addr);
    } else {
        DBG_LOG("Timeout IP");
        goto exit_1;
    }

    do {
        if ((hptr = gethostbyname(STR(HAHOST))) == NULL)
        {
            DBG_LOG("gethostbyname error for host: %s",
                    STR(HAHOST)
                    );
            goto exit_1;
        }

        DBG_LOG("hostname: %s", hptr->h_name);

        if (
            (hptr->h_addrtype == AF_INET) &&
            (pptr = hptr->h_addr_list) != NULL)
        {
            DBG_LOG("address: %s\n",
                    inet_ntoa_r(
                                *pptr[0],
                                str,
                                sizeof(str)
                                )
                    );
        } else {
            DBG_LOG("Error call inet_ntop");
            goto exit_1;
        }

        memset(&servin, 0x00, sizeof(servin));
        servin.sin_family = AF_INET;
        servin.sin_port = htons(HAPORT);
        inet_aton(str, &servin.sin_addr);

        if (0 == xQueueReceive(xdataq, &res, _ms(5000))) {
            DBG_LOG("Queue error");
            goto exit_1;
        }

        _ha_post(arg, &res, &servin);

    } while (dont_sleep());

exit_1:
    do {
        goto_sleep();
    } while (dont_sleep());
    vTaskDelete(NULL);
}

static void ICACHE_FLASH_ATTR task_sens(void *arg)
{
    dht_data_t res;

    do {
        init_gpio();

        GPIO_OUTPUT_SET(DHT_POWER, 0);  // enable dht power
        res = dht_read();
        GPIO_OUTPUT_SET(DHT_POWER, 1);

        GPIO_OUTPUT_SET(DIVD_POWER, 0); // enable battery divider power
        res.vbatt = (uint32_t)ADC_TO_V(system_adc_read());
        GPIO_OUTPUT_SET(DIVD_POWER, 1);

        xQueueSend(xdataq, &res, _ms(10));
    } while (dont_sleep());

    vTaskDelete(NULL);
}

uint32 ICACHE_FLASH_ATTR init_sensor_dht22(uint8_t *name)
{
    vSemaphoreCreateBinary(xconnect);
    xSemaphoreTake(xconnect, portMAX_DELAY);

    vSemaphoreCreateBinary(xdetect);
    xSemaphoreTake(xdetect, portMAX_DELAY);

    xdataq = xQueueCreate(1, sizeof(dht_data_t));

    wifi_set_event_handler_cb(&wifi_cb);

    xTaskCreate(&task_sens, "sense", 256, NULL, 8, NULL);

    xSemaphoreTake(xdetect, portMAX_DELAY);

    if (sens_detected) {
        xTaskCreate(&task_send, "send", 512, name, 3, NULL);
    }

    return sens_detected;
}
