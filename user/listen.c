/*
 * listen.c
 *
 *  Created on: Feb 12, 2016
 *      Author: n0ll
 */

#include "listen.h"
#include "led_pwm.h"
#include "debug.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/lwip/sockets.h"
#include "json/cJSON.h"

static char rx_buff[MAX_DATA_SIZE];
static char *id2str[] = {
    "None",
    "Discovery",
    "Board",
    "Pwm_period",
    "Led_get",
    "Led_set",
    "Rel_get",
    "Rel_set",
    "Test",
};

struct create_params {
    uint16  port;
};

static uint32 ICACHE_FLASH_ATTR make_sum(void *data, size_t size)
{
    uint8 *p = data;
    uint32 i = 0;
    uint8 sum = 0;

    while (i++ < size) {
        sum ^= *p++;
    }

    return sum;
}

static void ICACHE_FLASH_ATTR discovery_send(void *data)
{
    uint8  r[6], i;
    void *arr;

    DBG_LOG("Enter");

    arr = cJSON_CreateArray();
    wifi_get_ip_info(STATION_IF, (void *)r);
    for (i= 0; i< 4; i++) {
        cJSON_AddItemToArray(arr, cJSON_CreateNumber(r[i]));
    }
    cJSON_AddItemToObject(data, "ip", arr);

    arr = cJSON_CreateArray();
    wifi_get_macaddr(STATION_IF, r);
    for (i= 0; i< 6; i++) {
        cJSON_AddItemToArray(arr, cJSON_CreateNumber(r[i]));
    }
    cJSON_AddItemToObject(data, "mac", arr);

    DBG_LOG("Exit");
}

static void ICACHE_FLASH_ATTR board_send(void *data)
{
//    cJSON_AddItemReferenceToObject(data, "vcc", cJSON_CreateNumber(system_get_vdd33()));
    cJSON_AddItemToObject(data, "vcc",  cJSON_CreateNumber(readvdd33()));
    cJSON_AddItemToObject(data, "chip_id", cJSON_CreateNumber(system_get_chip_id()));
    cJSON_AddItemToObject(data, "rssi", cJSON_CreateNumber(wifi_station_get_rssi()));
//    cJSON_AddItemToObject(data, "time", cJSON_CreateNumber(system_get_time()));
    cJSON_AddItemToObject(data, "free", cJSON_CreateNumber(system_get_free_heap_size()));
}

static void ICACHE_FLASH_ATTR led_get(void *data)
{
    led_pwm_t pwm;
    void *led = cJSON_CreateObject();
    DBG_LOG("Exit");

    led_pwm_get_cfg(&pwm);

    cJSON_AddItemToObject(data, "period", cJSON_CreateNumber(pwm.period));

    cJSON_AddItemToObject(led, "r", cJSON_CreateNumber(pwm.led.color.r));
    cJSON_AddItemToObject(led, "g", cJSON_CreateNumber(pwm.led.color.g));
    cJSON_AddItemToObject(led, "b", cJSON_CreateNumber(pwm.led.color.b));

    cJSON_AddItemToObject(data, "led", led);

    DBG_LOG("Enter");
}

static void ICACHE_FLASH_ATTR led_set(void *data)
{
    led_t led;
    DBG_LOG("Enter");

    led.color.r = cJSON_GetObjectItem(cJSON_GetObjectItem(data, "led"), "r")->valueint;
    led.color.g = cJSON_GetObjectItem(cJSON_GetObjectItem(data, "led"), "b")->valueint;
    led.color.b = cJSON_GetObjectItem(cJSON_GetObjectItem(data, "led"), "g")->valueint;

    led_pwm_set(&led);

    DBG_LOG("Exit");
}

static void ICACHE_FLASH_ATTR led_pwm_set_period(void *data)
{
    DBG_LOG("Enter");

    led_pwm_init(cJSON_GetObjectItem(data, "period")->valueint);

    DBG_LOG("Exit");
}

static ICACHE_FLASH_ATTR proc_data (void *in, void **out)
{
    DBG_LOG("Enter");

    void    *js_in, *js_out, *payload_in, *payload_out;
    uint8   id;

    js_in = cJSON_Parse(in);
    if (!js_in) {
        DBG_LOG("Error json parse:\n%s", in);
        goto EXIT_1;
    }

#ifdef DEBUG_MSG
    char *s = cJSON_Print(js_in);
    DBG_LOG("Received: %s", s);
    free(s);
#endif

    payload_in = cJSON_GetObjectItem(js_in, "payload");
    if (!payload_in) {
        DBG_LOG("No payload found");
        goto EXIT_2;
    }

    {
        void    *tmp = cJSON_PrintUnformatted(payload_in);
        if(cJSON_GetObjectItem(js_in, "sum")->valueint != make_sum(tmp, strlen(tmp))) {
            DBG_LOG("Wrong sum");
            free(tmp);
            goto EXIT_2;
        }
        free(tmp);
    }

    js_out = cJSON_CreateObject();
    if (!js_out) {
        DBG_LOG("Error json obj");
        goto EXIT_2;
    }

    payload_out = cJSON_CreateObject();
    if (!payload_out) {
        DBG_LOG("Error json obj");
        goto EXIT_3;
    }

    id = cJSON_GetObjectItem(payload_in, "cmd_id")->valueint;
    DBG_LOG("ID: %s", id2str[id]);
    switch (id) {
    case ID_DISCOVERY:
        discovery_send(payload_out);
        break;
    case ID_BOARD_GET:
        board_send(payload_out);
        break;
    case ID_PWM_SET_PERIOD:
        led_pwm_set_period(payload_in);
        break;
    case ID_LED_GET:
        led_get(payload_out);
        break;
    case ID_LED_SET:
        led_set(payload_in);
        break;
    case ID_REL_GET:
    case ID_REL_SET:
    default:
        DBG_LOG("Wrong ID %d:", id);
        break;
    }

    cJSON_AddItemToObject(payload_out, "cnt", cJSON_CreateNumber(cJSON_GetObjectItem(payload_in, "cnt")->valueint));
    cJSON_AddItemToObject(payload_out, "cmd_id", cJSON_CreateNumber(cJSON_GetObjectItem(payload_in, "cmd_id")->valueint));
    cJSON_Minify(payload_out);

    cJSON_AddItemToObject(js_out, "payload", payload_out);
    {
        void *tmp = cJSON_PrintUnformatted(payload_out);
        cJSON_AddItemToObject(js_out, "sum", cJSON_CreateNumber(make_sum(tmp, strlen(tmp))));
        free(tmp);
    }

    cJSON_Minify(js_out);
    *out = cJSON_PrintUnformatted(js_out);

EXIT_3:
    cJSON_Delete(js_out);
EXIT_2:
    cJSON_Delete(js_in);
EXIT_1:
    DBG_LOG("Exit");
}
#define PRINT_TIME_DBG
static void ICACHE_FLASH_ATTR listen_task (void *arg)
{
#ifdef PRINT_TIME_DBG
    uint32  time = 0;
#endif
    struct sockaddr_in      _local;
    struct sockaddr_in      _remote;
    struct create_params    *c = arg;
    int _socket, recv_len;
    size_t _rlen = sizeof(_remote);
    void *out = NULL;

    DBG_LOG("Enter port: %d %p", c->port, c);

    // zero out the structure
    memset(&_local, 0, sizeof(_local));
    _local.sin_family = AF_INET;
    _local.sin_addr.s_addr = INADDR_ANY;
    _local.sin_port = htons(c->port);
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
        if (MAX_DATA_SIZE > recv_len) {
            proc_data(rx_buff, &out);
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

xTaskHandle ICACHE_FLASH_ATTR init_listen(uint16 port) {
    xTaskHandle hndl;
    static struct create_params c;

    led_pwm_init(0);

    c.port = port;

    DBG_LOG("Port: %d", c.port);
    xTaskCreate(&listen_task, "listen", 512, &c, 3, &hndl);
    return hndl;
}
