
#include <nrfx_clock.h>
#include <nrf_gpio.h>
#include <nrf_temp.h>
#include <nrf_nvmc.h>
#include <stdint.h>
#include <stdio.h>
#include <strings.h>

#include <FreeRTOS.h>
#include <task.h>

#include <embedded_drivers/nrfx/tracing_nrf52840.h>
#include <embedded_drivers/tracing_arm_cm4.h>

#include "dpsgo.h"

namespace { //anonymous

	static void clockEventHandler(nrfx_clock_evt_type_t event)
	{
		(void) event;
	}

}

/* hardware configuration / muxing */

void setupUICR(void)
{
	unsigned changes = 0;

	// make sure we use NFC pins as standard IO
	if((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) != (UICR_NFCPINS_PROTECT_Disabled << UICR_NFCPINS_PROTECT_Pos)) {
		nrf_nvmc_write_word((uint32_t)&NRF_UICR->NFCPINS,
				(0xffffffff & ~UICR_NFCPINS_PROTECT_Msk)
				| (UICR_NFCPINS_PROTECT_Disabled << UICR_NFCPINS_PROTECT_Pos));
		++changes;
	}

	// set GPIO reference voltage to 3.3V
	if((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) != (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos)) {
		nrf_nvmc_write_word((uint32_t)&NRF_UICR->REGOUT0,
				(0xffffffff & ~UICR_REGOUT0_VOUT_Msk)
				| (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos));
		++changes;
	}

	// we need to reset if we changed the UICR. but first show a quick message to the user.
	if(changes) {
		for(int k = 0; k < 4; ++k) {
			nrf_gpio_pin_toggle(pin_fault_led);
			for(uint32_t i = 0; i < 0x50000; ++i)
				__asm__ __volatile__("nop":::);
		}
		NVIC_SystemReset();
	}
}

static void init_hardware(void)
{
	nrfx_gpiote_init();
	nrf_gpio_cfg_output(pin_fault_led);

	setupUICR();

	nrf_temp_init();

	// setup RTC for FreeRTOS.
	// either systick or RTC can be used as sys-tick.
	// RTC consumes less power if tickless idle is chosen as well.
	nrfx_clock_init(clockEventHandler);
	nrf_clock_lf_src_set((nrf_clock_lfclk_t)NRFX_CLOCK_CONFIG_LF_SRC);
	nrfx_clock_lfclk_start();


	// enable all power rails
	nrf_gpio_cfg_output(pin_pwr_nen_10v);
	nrf_gpio_pin_clear(pin_pwr_nen_10v);
	nrf_gpio_cfg_output(pin_pwr_en_1v2);
	nrf_gpio_pin_set(pin_pwr_en_1v2);
	nrf_gpio_cfg_output(pin_pwr_en_12v);
	nrf_gpio_pin_set(pin_pwr_en_12v);

	nrf_gpio_cfg_input(pin_pwr_good, NRF_GPIO_PIN_NOPULL);
}

int main(void)
{
	BaseType_t ret;

#ifdef TRACE_SWO
	nrf_mux_swo();
	// configure SWO pin to 2MBaud UART ITM trace.
	arm_cm4_enable_swo_itm_tracing(true, 15,
					false, false, 0,
					false,
					0x00000003);
#endif // TRACE_SWO

	init_hardware();

	ret = xTaskCreate(watchdogManagerTask, "watchdog", 128, NULL, 1, &watchdogManager);
	assert(ret == pdPASS);

	ret = xTaskCreate(logicManagerTask, "logicManager", 512, NULL, 4, &logicManager);
	assert(ret == pdPASS);

	ret = xTaskCreate(spiManagerTask, "spiManager", 512, NULL, 3, &spiManager);
	assert(ret == pdPASS);

	ret = xTaskCreate(uartManagerTask, "uartManager", 512, NULL, 3, &uartManager);
	assert(ret == pdPASS);

	ret = xTaskCreate(i2cManagerTask, "i2cManager", 512, NULL, 2, &i2cManager);
	assert(ret == pdPASS);


	vTaskStartScheduler();


	// scheduler should never return, so indicate fault if it did
	__disable_irq();
	for(int k = 0; k < 128; ++k) {
		nrf_gpio_pin_toggle(pin_fault_led);
		for(uint32_t i = 0; i < 0x50000; ++i)
			__asm__ __volatile__("nop":::);
	}
	NVIC_SystemReset();
}

