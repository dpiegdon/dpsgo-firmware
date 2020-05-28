
#include "dpsgo.h"

#include "embedded_drivers/ad5761r_spi_dac.h"
#include "embedded_drivers/nrfx/glue.h"

#include <nrfx_spim.h>
#include <cstring>
#include <strings.h>

using namespace embedded_drivers;


bool fpga_initialize(nrfx_spim_t & spim_instance)
{
	// upload FPGA image
	bool success = false;

	nrfx_init_spim(&spim_instance,
			pin_spi_clk,
			pin_spi_sdi,	// SDI is slave side, so this is MOSI
			pin_spi_sdo,
			NRFX_SPIM_PIN_NOT_USED,
			false,
			NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
			0xff,
			NRF_SPIM_FREQ_1M,
			NRF_SPIM_MODE_3,
			NRF_SPIM_BIT_ORDER_MSB_FIRST);

	// prepage FPGA
	// take FPGA down, then put it into SPI slave mode
	nrf_gpio_cfg_output(pin_fpga_nreset);
	nrf_gpio_pin_clear(pin_fpga_nreset);
	vTaskDelay(pdMS_TO_TICKS(1));

	nrf_gpio_cfg_output(pin_spi_ncs_fpga);
	nrf_gpio_pin_clear(pin_spi_ncs_fpga);
	nrf_gpio_cfg_input(pin_fpga_cdone, NRF_GPIO_PIN_NOPULL);
	vTaskDelay(pdMS_TO_TICKS(1));

	nrf_gpio_pin_set(pin_fpga_nreset);
	vTaskDelay(pdMS_TO_TICKS(1));

	success = nrfx_spim_xfer_implementation(
			(void*)&spim_instance,
			FPGA_BITSTREAM, FPGA_BITSTREAM_SIZE,
			NULL, 0);
	if(!success)
		goto fail;

	uint8_t tailbuf[8];
	bzero(tailbuf, sizeof(tailbuf));
	success = nrfx_spim_xfer_implementation(
			(void*)&spim_instance,
			tailbuf, sizeof(tailbuf),
			NULL, 0);

	if(!success)
		goto fail;

	vTaskDelay(pdMS_TO_TICKS(1));

	success = (0 != nrf_gpio_pin_read(pin_fpga_cdone));

	vTaskDelay(pdMS_TO_TICKS(1));

fail:
	nrf_gpio_pin_set(pin_spi_ncs_fpga);
	vTaskDelay(pdMS_TO_TICKS(1));

	nrfx_spim_uninit(&spim_instance);
	return success;
}

static uint8_t c = 0x3f;

void fpga_transfer(nrfx_spim_t & spim_instance)
{
	// spim_instance is assumed to be initialized in mode 1.
	struct spi_context_with_cs ctx{ (void*)&spim_instance, true, pin_spi_ncs_fpga };
	uint8_t tx_buf[5]{ 0,0,0,0,c };
	uint8_t rx_buf[5];

	nrfx_spim_xfer_manual_cs_implementation((void*)&ctx, tx_buf, sizeof(tx_buf),
						rx_buf, sizeof(rx_buf));

	if(rx_buf[0] & ((1<<5)|(1<<7)))
		c -= 1;
	if(rx_buf[0] & ((1<<4)|(1<<6)))
		c += 1;
}


TaskHandle_t spiManager = NULL;
void spiManagerTask(void * ignored)
{
	(void)ignored;

	// init SPI bus for use with DAC and FPGA instance
	nrfx_spim_t spim_instance;

	spim_instance.p_reg = NRF_SPIM3;
	spim_instance.drv_inst_idx = NRFX_SPIM3_INST_IDX;

	while(!fpga_initialize(spim_instance))
		printf("failed to initialize FPGA\r\n");

	// prepare for use with DAC and FPGA client
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
		vTaskDelay(100);

		if(!ad5761r.WriteInputRegAndUpdate(i))
			printf("ad5761 write failed.\r\n");
		int32_t ret = ad5761r.ReadInputReg();
		if(ret != i)
			printf("ad5761 readback bad result: expected 0x%04x, got 0x%04lx\r\n",
					i, ret);

		vTaskDelay(1);

		fpga_transfer(spim_instance);
	};
}

