
#include <nrfx_clock.h>
#include <nrf_gpio.h>
#include <nrfx_gpiote.h>
#include <nrfx_uarte.h>
#include <nrf_802154.h>
#include <nrf_temp.h>
#include <stdint.h>
#include <stdio.h>
#include <strings.h>

#include <FreeRTOS.h>
#include <task.h>

const uint32_t led_pin = NRF_GPIO_PIN_MAP(0,8);

static void clockEventHandler(nrfx_clock_evt_type_t event)
{
	(void) event;
}

#define MAX_MESSAGE_SIZE 17
#define CHANNEL          23
// chan23 center is 2465MHz

/* hardware configuration / muxing */

static const uint32_t led = NRF_GPIO_PIN_MAP(1,13);

static const nrfx_gpiote_out_config_t led_config = {
	.init_state = GPIOTE_CONFIG_OUTINIT_Low,
	.task_pin = false
};

const nrfx_uarte_t uart0 = NRFX_UARTE_INSTANCE(0);

static const nrfx_uarte_config_t uart_config = {
	.pseltxd = NRF_GPIO_PIN_MAP(1,10),
	.pselrxd = NRF_GPIO_PIN_MAP(1,11),
	.pselcts = NRF_UARTE_PSEL_DISCONNECTED,
	.pselrts = NRF_UARTE_PSEL_DISCONNECTED,
	.hwfc = NRFX_UART_DEFAULT_CONFIG_HWFC,
	.parity = NRFX_UART_DEFAULT_CONFIG_PARITY,
	.baudrate = NRFX_UART_DEFAULT_CONFIG_BAUDRATE,
	.interrupt_priority = NRFX_UART_DEFAULT_CONFIG_IRQ_PRIORITY
};

/* global state variables */

static volatile bool m_tx_done;
static volatile bool m_tx_failed;
static volatile nrf_802154_tx_error_t m_tx_errorcode;

static void init_hardware(void)
{
	nrf_temp_init();

	// setup RTC for FreeRTOS.
	// either systick or RTC can be used as sys-tick.
	// RTC consumes less power if tickless idle is chosen as well.
	nrfx_clock_init(clockEventHandler);
	nrf_clock_lf_src_set((nrf_clock_lfclk_t)NRFX_CLOCK_CONFIG_LF_SRC);
	nrfx_clock_lfclk_start();

	nrf_gpio_cfg_output(led_pin);
}

static void ledFlasherTask(void * led)
{
	vTaskDelay(16 * (0xf & (unsigned) led));
	while(1) {
		nrf_gpio_pin_toggle(*(const uint32_t *)led);
		vTaskDelay(configTICK_RATE_HZ/2);
	}
}

int main(void)
{
	TaskHandle_t ledFlasher = NULL;

	BaseType_t ret;

	init_hardware();

	ret = xTaskCreate(ledFlasherTask, "led", 128, (void*)&led_pin, 3, &ledFlasher);
	assert(ret == pdPASS);

	vTaskStartScheduler();
}

