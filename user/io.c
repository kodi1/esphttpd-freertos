
/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Jeroen Domburg <jeroen@spritesmods.com> wrote this file. As long as you retain 
 * this notice you can do whatever you want with this stuff. If we meet some day, 
 * and you think this stuff is worth it, you can buy me a beer in return. 
 * ----------------------------------------------------------------------------
 */


#include <esp8266.h>

#define LEDGPIO 2
#define BTNGPIO 0

static os_timer_t resetBtntimer;

void ICACHE_FLASH_ATTR ioLed(int ena) {
	//gpio_output_set is overkill. ToDo: use better mactos
	if (ena) {
		gpio_output_set((1<<LEDGPIO), 0, (1<<LEDGPIO), 0);
	} else {
		gpio_output_set(0, (1<<LEDGPIO), (1<<LEDGPIO), 0);
	}
}

static void ICACHE_FLASH_ATTR resetBtnTimerCb(void *arg) {
	static int resetCnt=0;
	if ((GPIO_REG_READ(GPIO_IN_ADDRESS)&(1<<BTNGPIO))==0) {
		resetCnt++;
	} else {
		if (resetCnt>=6) { //3 sec pressed
			wifi_station_disconnect();
			wifi_set_opmode(0x3); //reset to AP+STA mode
			printf("Reset to AP mode. Restarting system...\n");
			system_restart();
		}
		resetCnt=0;
	}
}

#ifdef FREERTOS
static void ICACHE_FLASH_ATTR wifi_rst (void *arg)
{
    do {
        resetBtnTimerCb(NULL);
        vTaskDelay(500/portTICK_RATE_MS);
    } while (1);
}
#endif

void ioInit() {
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
	gpio_output_set(0, 0, (1<<LEDGPIO), (1<<BTNGPIO));
#ifdef FREERTOS
	xTaskCreate(&wifi_rst, "wifireset", 128, NULL, 1, NULL);
#else
    os_timer_disarm(&resetBtntimer);
    os_timer_setfn(&resetBtntimer, resetBtnTimerCb, NULL);
    os_timer_arm(&resetBtntimer, 500, 1);
#endif
}

