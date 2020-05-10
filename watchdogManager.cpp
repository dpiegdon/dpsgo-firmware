
#include "dpsgo.h"

TaskHandle_t watchdogManager = NULL;
void watchdogManagerTask(void * ignored)
{
	(void)ignored;
	while(1) {
		nrf_gpio_pin_toggle(pin_fault_led);
		vTaskDelay(configTICK_RATE_HZ/4);
	}
}

