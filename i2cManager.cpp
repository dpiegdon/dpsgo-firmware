
#include "dpsgo.h"

#include "embedded_drivers/ssd1306_i2c_display.h"
#include "embedded_drivers/font_tama_mini02.h"
#include "embedded_drivers/nrfx/glue.h"

#include <nrfx_twim.h>

namespace { //anonymous

	void msleep_implementation(void * ctx, unsigned msecs)
	{
		(void) ctx;
		vTaskDelay((msecs * configTICK_RATE_HZ) / 1000);
	}
}

StreamBufferHandle_t console = NULL;

TaskHandle_t i2cManager = NULL;
void i2cManagerTask(void * ignored)
{
	(void)ignored;

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

	console = xStreamBufferCreate(256, 1);

	printf("console ready.\r\n");

	while(1) {
		char buf[64];
		size_t len;
		len = xStreamBufferReceive(console, buf, sizeof(buf), pdMS_TO_TICKS(1000));
		disp.Write(buf, len);
	};
}

