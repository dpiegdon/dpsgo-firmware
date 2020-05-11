
#include "dpsgo.h"

#include <nrfx_wdt.h>

void wdtEventHandler(void)
{
	// FIXME check which tasks did not trigger and send a corresponding message,
	// or store the information somewhere in the UICR?
	nrf_gpio_pin_set(pin_fault_led);
}

TaskHandle_t watchdogManager = NULL;
void watchdogManagerTask(void * ignored)
{
	(void)ignored;

	nrfx_wdt_config_t wdtConfig;
	wdtConfig.behaviour = NRF_WDT_BEHAVIOUR_RUN_SLEEP_HALT;
	wdtConfig.reload_value = 2000; /* milliseconds */
	wdtConfig.interrupt_priority = NRFX_WDT_CONFIG_IRQ_PRIORITY;

	uint32_t err = nrfx_wdt_init(&wdtConfig, wdtEventHandler);
	if(err != NRFX_SUCCESS)
		while(1) { }

	nrfx_wdt_channel_id wdtChannel;
	nrfx_wdt_channel_alloc(&wdtChannel);

	// FIXME block for START event

	nrf_gpio_pin_toggle(pin_fault_led);
	nrfx_wdt_enable();

	while(1) {
		nrfx_wdt_channel_feed(wdtChannel);
		// FIXME block for event group and only continue if all threads passed.
		nrf_gpio_pin_toggle(pin_fault_led);
		vTaskDelay(configTICK_RATE_HZ/4);
	}
}

