
#include "dpsgo.h"

#include "embedded_drivers/ad5761r_spi_dac.h"
#include "embedded_drivers/nrfx/glue.h"

#include <nrfx_spim.h>

using namespace embedded_drivers;



TaskHandle_t spiManager = NULL;
void spiManagerTask(void * ignored)
{
	(void)ignored;

	nrfx_spim_t spim_instance;

	spim_instance.p_reg = NRF_SPIM3;
	spim_instance.drv_inst_idx = NRFX_SPIM3_INST_IDX;

	nrfx_init_spim(&spim_instance,
			pin_spi_clk,
			pin_spi_sdi,	// SDI is slave side, so this is MOSI
			pin_spi_sdo,
			NRFX_SPIM_PIN_NOT_USED,
			false,
			NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
			0xff,
			NRF_SPIM_FREQ_4M,
			NRF_SPIM_MODE_1,
			NRF_SPIM_BIT_ORDER_MSB_FIRST);

	// prepage FPGA
	// take FPGA down, then put it into SPI slave mode
	nrf_gpio_cfg_output(pin_fpga_nreset);
	nrf_gpio_pin_clear(pin_fpga_nreset);
	/*
	nrf_gpio_cfg_output(pin_spi_ncs_fpga);
	nrf_gpio_pin_clear(pin_spi_ncs_fpga);
	vTaskDelay(portTICK_PERIOD_MS*1);
	nrf_gpio_pin_set(pin_fpga_nreset);
	vTaskDelay(portTICK_PERIOD_MS*1);
	nrf_gpio_pin_set(pin_spi_ncs_fpga);
	vTaskDelay(portTICK_PERIOD_MS*1);
	*/

	// prepare DAC
	nrf_gpio_cfg_output(pin_spi_ncs_dac);
	nrf_gpio_pin_set(pin_spi_ncs_dac);
	struct spi_context_with_cs ad5761rSpiCtx;
	ad5761rSpiCtx.spim_instance = (void*)&spim_instance;
	ad5761rSpiCtx.cs_active_low = true;
	ad5761rSpiCtx.cs_pin = pin_spi_ncs_dac;
	Ad5761rSpiDac ad5761r((void*)&ad5761rSpiCtx, nrfx_spim_xfer_manual_cs_implementation);

	ad5761r.WriteControlRegister(0b00, false, false, true, true, 0b00, 0b001);

	uint16_t i = 0x8000;
	while(1) {
		vTaskDelay(500);
		//i += 655;
		if(!ad5761r.WriteInputRegAndUpdate(i))
			printf("ad5761 write failed.\r\n");
		int32_t ret = ad5761r.ReadInputReg();
		printf("ad5761 read: 0x%08lx\r\n", ret);
	};
}

