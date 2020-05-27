
#include "dpsgo.h"

#include "embedded_drivers/ssd1306_i2c_display.h"
#include "embedded_drivers/si5351_i2c_clockgen.h"
#include "embedded_drivers/font_tama_mini02.h"
#include "embedded_drivers/nrfx/glue.h"

#include <nrfx_twim.h>
#include <cstring>

using namespace embedded_drivers;

namespace { //anonymous

	void msleep_implementation(void * ctx, unsigned msecs)
	{
		(void) ctx;
		vTaskDelay((msecs * configTICK_RATE_HZ) / 1000);
	}

	bool init_si5351(Si5351I2cClockgenerator & clockgen, Ssd1306I2cDisplay & display)
	{
		display.Puts("Si5351 wait for SysInit\r\n");
		while(!clockgen.SysInitCompleted()) {
			//display.PutChar('.');
			vTaskDelay(configTICK_RATE_HZ/10);
		}
		display.Puts("Si5351 ready\r\n");

		if(!clockgen.PowerDown())
			return false;
		display.Puts("Si5351 powered down\r\n");

		if(!clockgen.EnableInterrupts(false, false, false, false))
			return false;
		display.Puts("Si5351 INT off\r\n");

		if(!clockgen.EnableFanout(true, true, true))
			return false;
		display.Puts("Si5351 fanout ok\r\n");

		if(!clockgen.PllSetInputSource(Si5351I2cClockgenerator::PLLSRC_CLKIN,
					Si5351I2cClockgenerator::PLLSRC_CLKIN, 1))
			return false;
		display.Puts("Si5351 pll source ok\r\n");

		/*
		if(!clockgen.PllSetMultisynth(Si5351I2cClockgenerator::PLL_A,
					60, 0, 1))
			return false;
		display.Puts("Si5351 PLL-A @600MHz\r\n");
		// FIXME set integer mode for PLL_A
		*/

		if(!clockgen.PllSetMultisynth(Si5351I2cClockgenerator::PLL_B,
					80, 0, 1))
			return false;
		display.Puts("Si5351 PLL-B @800MHz\r\n");
		// FIXME set integer mode for PLL_B

		// setup CLK0 to be 800/10 = 80MHz
		if(!clockgen.ClockSetMultisynth(Si5351I2cClockgenerator::CLOCK_0,
					10, 0, 1, 1, 0))
			return false;
		display.Puts("Si5351 clk0 MS set\r\n");
		// FIXME set MultiSynth0 to integer mode?

		if(!clockgen.ClockSetControl(Si5351I2cClockgenerator::CLOCK_0,
					true, false, Si5351I2cClockgenerator::PLL_B,
					false, Si5351I2cClockgenerator::CLKSRC_CLKIN, 0b11))
			return false;
		display.Puts("Si5351 clk0 ctrl ok\r\n");

		if(!clockgen.OebPinEnable(0xff))
			return false;
		display.Puts("Si5351 oeb pin disabled\r\n");

		if(!clockgen.ResetPll(true, true))
			return false;
		display.Puts("Si5351 PLL Reset ok\r\n");

		if(!clockgen.OutputEnable(0xfe))
			return false;
		display.Puts("Si5351 clk0 output on\r\n");

		return true;
	}
}

StreamBufferHandle_t console = NULL;

TaskHandle_t i2cManager = NULL;
void i2cManagerTask(void * ignored)
{
	(void)ignored;

	nrfx_twim_t twim_instance;

	twim_instance.p_twim       = NRF_TWIM0;
	twim_instance.drv_inst_idx = NRFX_TWIM0_INST_IDX;

	nrfx_init_twim(&twim_instance, pin_i2c_scl, pin_i2c_sda,
			(nrf_twim_frequency_t)104857600, // magic value for 400 KHz
			NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
			NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT);

	Ssd1306I2cDisplay display(NULL, msleep_implementation,
			&twim_instance,
			nrfx_twim_tx_implementation,
			nrfx_twim_rx_implementation,
			font_tama_mini02::dataptr,
			font_tama_mini02::width,
			font_tama_mini02::height);

	Si5351I2cClockgenerator clockgen(
			&twim_instance,
			nrfx_twim_tx_implementation,
			nrfx_twim_rx_implementation
			);

	console = xStreamBufferCreate(256, 1);

	display.Puts("console ready.\r\n");
	fflush(stdout);

	if(!init_si5351(clockgen, display))
		display.Puts("init_si5351() failed\r\n");

	while(1) {
		char buf[64];
		size_t len;
		len = xStreamBufferReceive(console, buf, sizeof(buf), pdMS_TO_TICKS(1));
		display.Write(buf, len);

		uint8_t val;
		clockgen.I2cRead(Si5351I2cClockgenerator::DeviceStatus, val);
		//printf("0x%2x\r\n", val);
	};
}

