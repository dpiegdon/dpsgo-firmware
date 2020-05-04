
#include <nrfx_clock.h>
#include <nrf_gpio.h>
#include <nrfx_gpiote.h>
#include <nrfx_twim.h>
#include <nrf_802154.h>
#include <nrf_temp.h>
#include "nrf_nvmc.h"
#include <stdint.h>
#include <stdio.h>
#include <strings.h>

#include <FreeRTOS.h>
#include <task.h>

#include "bsp.h"
#include "syscalls.h"
#include "embedded_drivers/nrfx/glue.h"
#include "embedded_drivers/nrfx/uarte.h"
#include "embedded_drivers/ssd1306_i2c_display.h"
#include "embedded_drivers/font_tama_mini02.h"

namespace { //anonymous

	void msleep_implementation(void * ctx, unsigned msecs)
	{
		(void) ctx;
		vTaskDelay((msecs * configTICK_RATE_HZ) / 1000);
	}

	static void clockEventHandler(nrfx_clock_evt_type_t event)
	{
		(void) event;
	}

}

/* hardware configuration / muxing */

static inline void nop(void)
{
	__asm__ __volatile__("nop":::);
}

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

	if(changes) {
		while(1) {
			// loop until reset and indicate state on fault LED
			nrf_gpio_pin_toggle(pin_fault_led);
			for(uint32_t i = 0; i < 0x50000; ++i)
				nop();
		}
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


	// disable all power rails
	nrf_gpio_cfg_output(pin_pwr_nen_10v);
	nrf_gpio_pin_set(pin_pwr_nen_10v);
	nrf_gpio_cfg_output(pin_pwr_en_1v2);
	nrf_gpio_pin_clear(pin_pwr_en_1v2);
	nrf_gpio_cfg_output(pin_pwr_en_12v);
	nrf_gpio_pin_clear(pin_pwr_en_12v);

	nrf_gpio_cfg_input(pin_pwr_good, NRF_GPIO_PIN_NOPULL);
}

static void ledFlasherTask(void * led)
{
	vTaskDelay(16 * (0xf & (unsigned) led));
	while(1) {
		//nrf_gpio_pin_toggle(*(const uint32_t *)led);
		vTaskDelay(configTICK_RATE_HZ/3);
	}
}

static void gpsToDisplayTask(void * ignored)
{
	(void)ignored;

	Uarte uart(pin_gps_rxd, pin_gps_txd, pin_gps_cts, pin_gps_rts);

	nrfx_twim_t twim_instance;

	embedded_drivers::nrfx_init_twim(&twim_instance, pin_i2c_scl, pin_i2c_sda,
			(nrf_twim_frequency_t)104857600, // magic value for 400 KHz
			NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
			NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT);

	embedded_drivers::Ssd1306I2cDisplay disp(NULL, msleep_implementation,
			&twim_instance,
			embedded_drivers::nrfx_twim_tx_implementation,
			embedded_drivers::nrfx_twim_rx_implementation,
			embedded_drivers::font_tama_mini02::dataptr,
			embedded_drivers::font_tama_mini02::width,
			embedded_drivers::font_tama_mini02::height);

	stdio_via_ssd1306(&disp);

	printf("ok.");

	while(1) {
		int ret;
		char c;
		ret = uart.Read(&c, sizeof(c));
		nrf_gpio_pin_toggle(pin_fault_led);
		if(ret == NRFX_SUCCESS) {
			disp.PutChar(c);
		} else {
			int errors = uart.GetErrors();
			printf("error: ");
			if(errors & NRF_UARTE_ERROR_OVERRUN_MASK)
				printf("overrun ");
			if(errors & NRF_UARTE_ERROR_PARITY_MASK)
				printf("parity ");
			if(errors & NRF_UARTE_ERROR_FRAMING_MASK)
				printf("framing ");
			if(errors & NRF_UARTE_ERROR_BREAK_MASK)
				printf("break ");
			printf("\x02\r\n");
		}
	}
}

int main(void)
{
	TaskHandle_t ledFlasher = NULL;
	TaskHandle_t gpsToDisplay = NULL;

	BaseType_t ret;

	init_hardware();

	ret = xTaskCreate(ledFlasherTask, "led", 128, (void*)&pin_fault_led, 3, &ledFlasher);
	assert(ret == pdPASS);

	ret = xTaskCreate(gpsToDisplayTask, "gps2display", 512, NULL, 3, &gpsToDisplay);
	assert(ret == pdPASS);

	vTaskStartScheduler();
}

