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

#include "spi.h"
#define SPI_DEV HSPI

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

#define DIVD_POWER  2
#define BME_POWER   5
#define BME_CS      4
//#define SLEEP_EN    15

#define MUX_DIVD_POWER  PERIPHS_IO_MUX_GPIO2_U
#define MUX_BME_POWER   PERIPHS_IO_MUX_GPIO5_U
#define MUX_BME_CS      PERIPHS_IO_MUX_GPIO4_U
//#define MUX_SLEEP_EN   PERIPHS_IO_MUX_MTDO_U

#define SLEEP_SECONDS   (1000 * 1000 * 600)
#define _ms(a)          ((a) / portTICK_RATE_MS)
#define _ADC            (3400.0 / 830.0)
#define ADC_TO_V(r)     (((_ADC * r * 1000) / 1024.0) - 77)

#define MAXLINE 1024
char _temp[MAXLINE + 1];

static xSemaphoreHandle xconnect = NULL;
static xSemaphoreHandle xdetect = NULL;
static xQueueHandle xdataq = NULL;
static uint32_t addr = 0;

volatile uint32_t sens_detected = 0;

typedef struct
{
    float       temperature;
    float       humidity;
    float       pressure;
    uint32_t    vbatt;
} bme_data_t;

#define BME_REG_ADDR_ID     (0xD0)
#define BME_REG_ADDR_ADC    (0xF7)
#define BME_REG_ADDR_STATUS (0xF3)
#define BME_REG_ADDR_CTRLH  (0xF2)
#define BME_REG_ADDR_CTRL   (0xF4)
#define BME_REG_ADDR_CONF   (0xF5)
#define BME_REG_ADDR_T1     (0x88)
#define BME_REG_ADDR_T2     (0x8A)
#define BME_REG_ADDR_T3     (0x8C)
#define BME_REG_ADDR_P1     (0x8E)
#define BME_REG_ADDR_P2     (0x90)
#define BME_REG_ADDR_P3     (0x92)
#define BME_REG_ADDR_P4     (0x94)
#define BME_REG_ADDR_P5     (0x96)
#define BME_REG_ADDR_P6     (0x98)
#define BME_REG_ADDR_P7     (0x9A)
#define BME_REG_ADDR_P8     (0x9C)
#define BME_REG_ADDR_P9     (0x9E)
#define BME_REG_ADDR_H1     (0xA1)
#define BME_REG_ADDR_H2     (0xE1)
#define BME_REG_ADDR_H3     (0xE3)
#define BME_REG_ADDR_H4     (0xE4)
#define BME_REG_ADDR_H_45   (0xE5)
#define BME_REG_ADDR_H5     (0xE6)
#define BME_REG_ADDR_H6     (0xE7)
#define BME_REG_ADDR_26     (0xE1)

typedef struct
{
    uint16_t    dig_t1;
    int16_t     dig_t2;
    int16_t     dig_t3;
    uint16_t    dig_p1;
    int16_t     dig_p2;
    int16_t     dig_p3;
    int16_t     dig_p4;
    int16_t     dig_p5;
    int16_t     dig_p6;
    int16_t     dig_p7;
    int16_t     dig_p8;
    int16_t     dig_p9;
    uint8_t     dig_h1;
    int16_t     dig_h2;
    uint8_t     dig_h3;
    int16_t     dig_h4;
    int16_t     dig_h5;
    int8_t      dig_h6;
} bme_calib_data_t;

typedef union {
    struct {
        uint8 press_msb;
        uint8 press_lsb;
        uint8 press_xlsb;
        uint8 temp_msb;
        uint8 temp_lsb;
        uint8 temp_xlsb;
        uint8 hum_msb;
        uint8 hum_lsb;
    };
    uint8 raw_data[0];
} bme_adc_data_t;


static inline uint8 bme_write_byte(uint8 addr, uint8 wr)
{
    uint8 b;
    GPIO_OUTPUT_SET(BME_CS, 0);
    b = spi_transaction(SPI_DEV, 1, 0, 7, addr, 8, wr, 0, 0);
    os_delay_us(400);
    GPIO_OUTPUT_SET(BME_CS, 1);
    return b;
}

static inline int32 bme_read_bytes(uint8 addr, uint8 size, uint8 *data)
{
    int32 ret = -1;
    if (!size || !data) {
        DBG_LOG("Wrong data");
        return ret;
    }

    GPIO_OUTPUT_SET(BME_CS, 0);

    *data = spi_transaction(SPI_DEV, 1, 1, 7, addr, 0, 0, 8, 0);
    data++;
    size--;

    while (size--) {
        *data = spi_rx8(SPI_DEV);
        data++;
    }

    GPIO_OUTPUT_SET(BME_CS, 1);
    return ret;
}
static inline uint16 bme_read_2bytes(uint8 addr)
{
    uint16 ret;
    bme_read_bytes(addr, sizeof(ret), (void *)&ret);
    return ret;
}

static inline uint8 bme_read_byte(uint8 addr)
{
    uint8 ret;
    bme_read_bytes(addr, sizeof(ret), &ret);
    return ret;
}

static inline void bme_config(void)
{
    uint8 tmp;

    tmp = bme_read_byte(BME_REG_ADDR_CTRL);
    tmp |= ((3 << 5) | (3 << 2));               // oversample x 4
    bme_write_byte(BME_REG_ADDR_CTRL, tmp);

    tmp = bme_read_byte(BME_REG_ADDR_CTRLH);
    tmp |= 3 << 0;                              // oversample x 4
    bme_write_byte(BME_REG_ADDR_CTRLH, tmp);

    tmp = bme_read_byte(BME_REG_ADDR_CONF);
    tmp |= 2 << 2;                              // IIR coeff 4
    bme_write_byte(BME_REG_ADDR_CONF, tmp);
}

static inline uint32_t bme_check_if_measure(void)
{
    uint8 tmp = bme_read_byte(BME_REG_ADDR_STATUS);
    return (tmp >> 3) & 0x01;
}

static inline void bme_measure(uint8 force)
{
    uint8 tmp;
    tmp = bme_read_byte(BME_REG_ADDR_CTRL);

    tmp |= 1 << 0;                              // force mode
    if (!force) {
        tmp |= 1 << 1;                          // normal mode
    }

    bme_write_byte(BME_REG_ADDR_CTRL, tmp);
}

static inline uint32_t dont_sleep(void)
{
    return !sens_detected;
}

static ICACHE_FLASH_ATTR void init_gpio(void)
{
    PIN_FUNC_SELECT(MUX_DIVD_POWER, GPIO_FUNC(DIVD_POWER));
    PIN_FUNC_SELECT(MUX_BME_POWER , GPIO_FUNC(BME_POWER));
    PIN_FUNC_SELECT(MUX_BME_CS , GPIO_FUNC(BME_CS));

    //init SPI bus
    spi_init_gpio(SPI_DEV, SPI_CLK_USE_DIV);
    spi_mode(SPI_DEV, 0, 0);
    spi_clock(SPI_DEV, 4, 2); // 10Mhz @ 80 Mhz
    spi_tx_byte_order(SPI_DEV, SPI_BYTE_ORDER_HIGH_TO_LOW);
    spi_rx_byte_order(SPI_DEV, SPI_BYTE_ORDER_HIGH_TO_LOW);

    SET_PERI_REG_MASK(SPI_USER(SPI_DEV), SPI_CS_SETUP|SPI_CS_HOLD);
    CLEAR_PERI_REG_MASK(SPI_USER(SPI_DEV), SPI_FLASH_MODE);

    return;
}

static ICACHE_FLASH_ATTR void bme_calc(bme_adc_data_t *_d,
                                        bme_calib_data_t *_c,
                                        bme_data_t *_res)
{
    int32 adc_t, adc_p, adc_h;
    int32 t_fine;

    adc_t = _d->temp_xlsb;
    adc_t |= _d->temp_lsb << 8;
    adc_t |= _d->temp_msb << 16;
    adc_t >>= 4;

    adc_p = _d->press_xlsb;
    adc_p |= _d->press_lsb << 8;
    adc_p |= _d->press_msb << 16;
    adc_p >>= 4;

    adc_h = _d->hum_lsb | (_d->hum_msb << 8);

//    DBG_LOG("T: 0x%x P: 0x%x H: 0x%x", adc_t, adc_p, adc_h);

    {
        int32 var1, var2;
        var1 = ((((adc_t>>3) - ((int32_t)_c->dig_t1 <<1))) *
                ((int32_t)_c->dig_t2)) >> 11;

        var2 = (((((adc_t>>4) - ((int32_t)_c->dig_t1)) *
                  ((adc_t>>4) - ((int32_t)_c->dig_t1))) >> 12) *
                ((int32_t)_c->dig_t3)) >> 14;

        t_fine = var1 + var2;
        _res->temperature = ((t_fine * 5 + 128) >> 8) / 100.0;
    }

    {
        int64_t var1, var2, p;
        var1 = ((int64_t)t_fine) - 128000;
        var2 = var1 * var1 * (int64_t)_c->dig_p6;
        var2 = var2 + ((var1*(int64_t)_c->dig_p5)<<17);
        var2 = var2 + (((int64_t)_c->dig_p4)<<35);
        var1 = ((var1 * var1 * (int64_t)_c->dig_p3)>>8) +
               ((var1 * (int64_t)_c->dig_p2)<<12);
        var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_c->dig_p1)>>33;

        if (var1 != 0) {
            p = 1048576 - adc_p;
            p = (((p<<31) - var2)*3125) / var1;
            var1 = (((int64_t)_c->dig_p9) * (p>>13) * (p>>13)) >> 25;
            var2 = (((int64_t)_c->dig_p8) * p) >> 19;

            p = ((p + var1 + var2) >> 8) + (((int64_t)_c->dig_p7)<<4);

            _res->pressure = p / 25600.0;
        }
    }

    {
        int32_t v_x1_u32r;

        v_x1_u32r = (t_fine - ((int32_t)76800));

        v_x1_u32r = (((((adc_h << 14) - (((int32_t)_c->dig_h4) << 20) -
                        (((int32_t)_c->dig_h5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                     (((((((v_x1_u32r * ((int32_t)_c->dig_h6)) >> 10) *
                          (((v_x1_u32r * ((int32_t)_c->dig_h3)) >> 11) + ((int32_t)32768))) >> 10) +
                        ((int32_t)2097152)) * ((int32_t)_c->dig_h2) + 8192) >> 14));

        v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                                   ((int32_t)_c->dig_h1)) >> 4));

        v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
        v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;

        _res->humidity = (v_x1_u32r>>12) / 1024.0;
    }
}

static ICACHE_FLASH_ATTR bme_data_t bme_read(void)
{
    bme_adc_data_t data;
    bme_calib_data_t calib;
    bme_data_t ret = {0};

    init_gpio();

    GPIO_OUTPUT_SET(DIVD_POWER, 0);
    ret.vbatt = (uint32_t)ADC_TO_V(system_adc_read());
    GPIO_OUTPUT_SET(DIVD_POWER, 1);

    // bme power on select spi - slect
    GPIO_OUTPUT_SET(BME_CS, 0);
    vTaskDelay(_ms(1));
    GPIO_OUTPUT_SET(BME_POWER, 0);
    vTaskDelay(_ms(1));
    GPIO_OUTPUT_SET(BME_CS, 1);
    vTaskDelay(_ms(10));

    if (0x60 != bme_read_byte(BME_REG_ADDR_ID)) {
        xSemaphoreGive(xdetect);
        DBG_LOG("BME detect fail");
        goto exit_1;
    }

    sens_detected = 1;
    xSemaphoreGive(xdetect);

    bme_config();
    bme_measure(0);

    calib.dig_t1 = bme_read_2bytes(BME_REG_ADDR_T1);
    calib.dig_t2 = bme_read_2bytes(BME_REG_ADDR_T2);
    calib.dig_t3 = bme_read_2bytes(BME_REG_ADDR_T3);

    calib.dig_p1 = bme_read_2bytes(BME_REG_ADDR_P1);
    calib.dig_p2 = bme_read_2bytes(BME_REG_ADDR_P2);
    calib.dig_p3 = bme_read_2bytes(BME_REG_ADDR_P3);
    calib.dig_p4 = bme_read_2bytes(BME_REG_ADDR_P4);
    calib.dig_p5 = bme_read_2bytes(BME_REG_ADDR_P5);
    calib.dig_p6 = bme_read_2bytes(BME_REG_ADDR_P6);
    calib.dig_p7 = bme_read_2bytes(BME_REG_ADDR_P7);
    calib.dig_p8 = bme_read_2bytes(BME_REG_ADDR_P8);
    calib.dig_p9 = bme_read_2bytes(BME_REG_ADDR_P9);

    calib.dig_h1 = bme_read_byte(BME_REG_ADDR_H1);
    calib.dig_h2 = bme_read_2bytes(BME_REG_ADDR_H2);
    calib.dig_h3 = bme_read_byte(BME_REG_ADDR_H3);

    {
        uint8 h45 = bme_read_byte(BME_REG_ADDR_H_45);
        calib.dig_h4 = bme_read_byte(BME_REG_ADDR_H4);
        calib.dig_h4 = calib.dig_h4 << 4 | (h45 & 0x0f);

        calib.dig_h5 = bme_read_byte(BME_REG_ADDR_H5);
        calib.dig_h5 = calib.dig_h5 << 4 | (h45 >> 4);
    }

    calib.dig_h6 = bme_read_byte(BME_REG_ADDR_H6);

    vTaskDelay(_ms(500));

    bme_read_bytes(BME_REG_ADDR_ADC,
                    sizeof(data),
                    data.raw_data);

    bme_calc(&data, &calib, &ret);

//    DBG_LOG("Temp: %d Press: %d Hum: %d",
//                (uint32)(ret.temperature * 100),
//                (uint32)(ret.pressure * 100),
//                (uint32)(ret.humidity * 100));
exit_1:
    GPIO_OUTPUT_SET(BME_POWER, 1);

    if (dont_sleep()) {
        vTaskDelay(_ms(1000));
    }
    return ret;
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
                                                 uint8 const    *type,
                                                 uint8 const    *units,
                                                 uint8 const    *icon,
                                                 float const    result,
                                                 void           *servin)
{
    void *_data_payload;
    void *jspayload = cJSON_CreateObject();
    void *jsattr = cJSON_CreateObject();

    double res = res > 0 ?
            (int32)((result + 0.005) * 100) / 100.0 :
            (int32)((result - 0.005) * 100) / 100.0;

    cJSON_AddItemToObject(jsattr, "attribution", cJSON_CreateString(name));
    cJSON_AddItemToObject(jsattr, "unit_of_measurement", cJSON_CreateString(units));
    cJSON_AddItemToObject(jsattr, "icon", cJSON_CreateString(icon));

    snprintf(_temp,
                    MAXLINE,
                    "%s_%s",
                    type,
                    name);

    cJSON_AddItemToObject(jsattr, "friendly_name", cJSON_CreateString(_temp));

    cJSON_AddItemToObject(jspayload, "attributes", jsattr);
    cJSON_AddItemToObject(jspayload, "state", cJSON_CreateNumber(res));
    cJSON_Minify(jspayload);

    _data_payload = cJSON_PrintUnformatted(jspayload);
    cJSON_Delete(jspayload);

    snprintf(_temp,
                MAXLINE,
                "POST /api/states/sensor.%s_%s HTTP/1.0\r\n"
                "Authorization: Bearer %s\r\n"
                "Content-Type: application/json\r\n"
                "Content-length: %d\r\n\r\n"
                "%s",
                name,
                type,
                STR(HATOKEN),
                strlen(_data_payload),
                _data_payload
            );

    _send(servin, _temp, MAXLINE);
    free(_data_payload);
}

static void ICACHE_FLASH_ATTR task_send (void *arg)
{
    char **pptr;
    char str[16];
    struct hostent *hptr;
    bme_data_t res;
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

        _ha_post(
                    arg,
                    "battery",
                    "V",
                    "mdi:gauge",
                    (res.vbatt / 1000.0),
                    &servin
                );

        _ha_post(
                    arg,
                    "temperature",
                    "°C",
                    "mdi:thermometer",
                    res.temperature,
                    &servin
                );

        _ha_post(
                    arg,
                    "pressure",
                    "hPa",
                    "mdi:gauge",
                    res.pressure,
                    &servin
                );

        _ha_post(
                    arg,
                    "humidity",
                    "%",
                    "mdi:water-percent",
                    res.humidity,
                    &servin
                );
    } while (dont_sleep());

exit_1:
    do {
        goto_sleep();
    } while (dont_sleep());
    vTaskDelete(NULL);
}

static void ICACHE_FLASH_ATTR task_sens(void *arg)
{
    bme_data_t res;

    do {
        res = bme_read();
        xQueueSend(xdataq, &res, _ms(10));
    } while (dont_sleep());

    vTaskDelete(NULL);
}

uint32 ICACHE_FLASH_ATTR init_sensor_bme280(uint8_t *name)
{
    vSemaphoreCreateBinary(xconnect);
    xSemaphoreTake(xconnect, portMAX_DELAY);

    vSemaphoreCreateBinary(xdetect);
    xSemaphoreTake(xdetect, portMAX_DELAY);

    xdataq = xQueueCreate(1, sizeof(bme_data_t));

    wifi_set_event_handler_cb(&wifi_cb);

    xTaskCreate(&task_sens, "sense", 256, NULL, 3, NULL);

    xSemaphoreTake(xdetect, portMAX_DELAY);

    if (sens_detected) {
        xTaskCreate(&task_send, "send", 512, name, 3, NULL);
    }

    return sens_detected;
}
