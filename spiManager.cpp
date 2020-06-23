
#include "dpsgo.h"

#include "embedded_drivers/ad5761r_spi_dac.h"
#include "embedded_drivers/nrfx/glue.h"

#include <algorithm>
#include <cstring>
#include <strings.h>
#include <nrfx_spim.h>
#include <machine/endian.h>
#include <event_groups.h>

using namespace embedded_drivers;

static EventGroupHandle_t spimEvents;
#define SPIM_EVENT_DONE (1 << 0)
#define SPIM_EVENTS     ( SPIM_EVENT_DONE )

namespace /* anon */ {

	static uint16_t downcount = 1;
	uint16_t dac_out = 0x8000;

	void spimEventHandler(nrfx_spim_evt_t const * event, void * context)
	{
		(void)context;
		EventBits_t events = 0;

		switch(event->type) {
			case NRFX_SPIM_EVENT_DONE:
				events |= SPIM_EVENT_DONE;
				break;
		}

		BaseType_t xTrue = pdTRUE;
		xEventGroupSetBitsFromISR(spimEvents, events, &xTrue);
	}

	bool blocking_spim_xfer_implementation(void * spi_context,
			uint8_t const * tx_buf,
			size_t tx_size,
			uint8_t * rx_buf,
			size_t rx_size)
	{
		nrfx_spim_t * spim_instance = (nrfx_spim_t *)spi_context;

		nrfx_spim_xfer_desc_t xfer;

		xfer.p_tx_buffer = tx_buf;
		xfer.tx_length = tx_size;
		xfer.p_rx_buffer = rx_buf;
		xfer.rx_length = rx_size;

		if(NRFX_SUCCESS != nrfx_spim_xfer(spim_instance, &xfer, 0))
			return false;

		EventBits_t events;
		while(0 == (events = xEventGroupWaitBits(spimEvents, SPIM_EVENTS, true, false, pdMS_TO_TICKS(1000))))
			/* wait */ ;
		return events == SPIM_EVENT_DONE;
	}

	bool blocking_spim_xfer_manual_cs_implementation(void * spi_context_with_cs,
			uint8_t const * tx_buf,
			size_t tx_size,
			uint8_t * rx_buf,
			size_t rx_size)
	{
		struct spi_context_with_cs * ctx = (struct spi_context_with_cs *)spi_context_with_cs;
		if(ctx->cs_active_low)
			nrf_gpio_pin_clear(ctx->cs_pin);
		else
			nrf_gpio_pin_set(ctx->cs_pin);
		bool ret = blocking_spim_xfer_implementation(ctx->spim_instance, tx_buf, tx_size, rx_buf, rx_size);
		if(ctx->cs_active_low)
			nrf_gpio_pin_set(ctx->cs_pin);
		else
			nrf_gpio_pin_clear(ctx->cs_pin);
		return ret;
	}


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
				NRF_SPIM_BIT_ORDER_MSB_FIRST,
				spimEventHandler, NULL);

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

		success = blocking_spim_xfer_implementation(
				(void*)&spim_instance,
				FPGA_BITSTREAM, FPGA_BITSTREAM_SIZE,
				NULL, 0);
		if(!success)
			goto fail;

		uint8_t tailbuf[8];
		bzero(tailbuf, sizeof(tailbuf));
		success = blocking_spim_xfer_implementation(
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

	struct FpgaSpiBuffer {
		uint16_t downcount;
		uint8_t upcount_high;
		uint32_t upcount;
	} __attribute__((packed));

	static int up = 0;
	static int down = 0;

	void fpga_transfer(nrfx_spim_t & spim_instance)
	{
		// spim_instance is assumed to be initialized in mode 1.
		struct spi_context_with_cs ctx{ (void*)&spim_instance, true, pin_spi_ncs_fpga };

		struct FpgaSpiBuffer tx_buf{.downcount=__htons(downcount), .upcount_high=0, .upcount=0};
		struct FpgaSpiBuffer rx_buf{};

		if(!blocking_spim_xfer_manual_cs_implementation((void*)&ctx, (uint8_t*)&tx_buf, sizeof(tx_buf),
							(uint8_t*)&rx_buf, sizeof(rx_buf)))
			return;

		uint64_t upcount = (((uint64_t)rx_buf.upcount_high) << 32) + __ntohl(rx_buf.upcount);

		if(upcount != 0) {
			printf("\fcounter %llu\r\n", upcount);

			int delta = 0x500 / (1+std::min(up, down));
			if(delta < 1)
				delta = 1;

			printf("up %u down %u delta %u\r\n", up, down, delta);

			if(upcount < downcount * internal_reference_frequency) {
				if(dac_out <= 0xffff-delta) {
					dac_out += delta;
					if(up < 1024)
						up++;
				}
				printf("DAC up: %u\r\n", dac_out);
			} else if(upcount > downcount * internal_reference_frequency) {
				if(dac_out >= 0+delta) {
					dac_out -= delta;
					if(down < 1024)
						down++;
				}
				printf("DAC down: %u\r\n", dac_out);
			} else {
				if(downcount < 300)
					downcount += 1;
			}
		}
	}

} // end of namespace anon

TaskHandle_t spiManager = NULL;
void spiManagerTask(void * ignored)
{
	(void)ignored;

	spimEvents = xEventGroupCreate();

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
			NRF_SPIM_FREQ_2M,
			NRF_SPIM_MODE_2,
			NRF_SPIM_BIT_ORDER_MSB_FIRST,
			spimEventHandler, NULL);

	// prepare DAC
	nrf_gpio_cfg_output(pin_spi_ncs_dac);
	nrf_gpio_pin_set(pin_spi_ncs_dac);
	struct spi_context_with_cs ad5761rSpiCtx;
	ad5761rSpiCtx.spim_instance = (void*)&spim_instance;
	ad5761rSpiCtx.cs_active_low = true;
	ad5761rSpiCtx.cs_pin = pin_spi_ncs_dac;
	Ad5761rSpiDac ad5761r((void*)&ad5761rSpiCtx, blocking_spim_xfer_manual_cs_implementation);

	ad5761r.WriteControlRegister(0b00, false, false, true, true, 0b00, 0b001);

	while(1) {
		for(int i = 0; i < 1+downcount; ++i)
			vTaskDelay(1100);

		if(!ad5761r.WriteInputRegAndUpdate(dac_out))
			printf("ad5761 write failed.\r\n");
#if 0
		else
			printf("ad5761 set to 0x%04x.\r\n", dac_out);
#endif
		int32_t ret = ad5761r.ReadInputReg();
		if(ret != dac_out)
			printf("ad5761 readback bad result: expected 0x%04x, got 0x%04lx\r\n",
					dac_out, ret);

		vTaskDelay(1);

		fpga_transfer(spim_instance);
	};
}

