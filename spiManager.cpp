
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

namespace /* anon */ {
	/* events for that the task will want to block */
	static EventGroupHandle_t spimEvents;
#	define SPIM_EVENT_DONE (1 << 0)
#	define SPIM_EVENTS     ( SPIM_EVENT_DONE )

	/* SPI event handler that allows us to block tasks */
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

	/* blocking implementation of SPI transfers,
	 * using above event handler to unblock task once transfer finishes */
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

	/* manual chipselect-wrapper for blocking SPI transfer.
	 * the nrfx driver does not support multiple chipselects, so we use GPIOs to toggle these. */
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


	/* upload FPGA bitstream and start up FPGA */
	bool fpga_initialize(nrfx_spim_t & spim_instance)
	{
		bool success = false;

		// init SPI driver for specific use
		nrfx_init_spim(&spim_instance,
				pin_spi_clk,
				pin_spi_sdi, // SDI is slave side, so this is MOSI
				pin_spi_sdo,
				NRFX_SPIM_PIN_NOT_USED,
				false,
				NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
				0xff,
				NRF_SPIM_FREQ_1M,
				NRF_SPIM_MODE_3,
				NRF_SPIM_BIT_ORDER_MSB_FIRST,
				spimEventHandler, NULL);

		// reset configuration
		nrf_gpio_cfg_output(pin_fpga_nreset);
		nrf_gpio_pin_clear(pin_fpga_nreset);
		vTaskDelay(pdMS_TO_TICKS(1));

		// put FPGA it into SPI slave mode when leaving reset
		nrf_gpio_cfg_output(pin_spi_ncs_fpga);
		nrf_gpio_pin_clear(pin_spi_ncs_fpga);
		nrf_gpio_cfg_input(pin_fpga_cdone, NRF_GPIO_PIN_NOPULL);
		vTaskDelay(pdMS_TO_TICKS(1));

		// leave reset
		nrf_gpio_pin_set(pin_fpga_nreset);
		vTaskDelay(pdMS_TO_TICKS(1));

		// upload bitstream via SPI
		success = blocking_spim_xfer_implementation(
				(void*)&spim_instance,
				FPGA_BITSTREAM, FPGA_BITSTREAM_SIZE,
				NULL, 0);
		if(!success)
			goto finish;

		// send tailing clocks after bitstream
		uint8_t tailbuf[8];
		bzero(tailbuf, sizeof(tailbuf));
		success = blocking_spim_xfer_implementation(
				(void*)&spim_instance,
				tailbuf, sizeof(tailbuf),
				NULL, 0);
		if(!success)
			goto finish;

		// wait for device to settle
		vTaskDelay(pdMS_TO_TICKS(1));

		// check if configuration was successful
		success = (0 != nrf_gpio_pin_read(pin_fpga_cdone));
		vTaskDelay(pdMS_TO_TICKS(1));

	finish:
		// release chipselect
		nrf_gpio_pin_set(pin_spi_ncs_fpga);
		vTaskDelay(pdMS_TO_TICKS(1));

		// release SPI driver
		nrfx_spim_uninit(&spim_instance);
		return success;
	}

	/* SPI transfer payload format containing all the counters,
	 * as expected by used FPGA bitstream */
	struct CounterSpiTransfer {
	private: /* actual packed payload corresponding to transfer */
		uint16_t downcount;	// counter to be decreased on each GPS clock
		uint8_t upcount_high;	// counter to be increased on each system block, highest 8 bits
		uint32_t upcount;	// counter to be increased on each system block, lower 32 bits

	public: /* only methods */
		CounterSpiTransfer(uint16_t newDowncount=0)
			: downcount(0), upcount_high(0), upcount(0)
		{ setDownCount(newDowncount); }

		void setDownCount(uint16_t newDowncount)
		{ this->downcount = __htons(newDowncount); }

		uint64_t getUpCount(void)
		{ return (((uint64_t)this->upcount_high) << 32) + __ntohl(this->upcount); }
	} __attribute__((packed));

	/* stateful evaluation of counter feedback from FPGA.
	 * this calculates and stores the output to be set via the DAC later on. */
	static int up = 1;
	static int down = 1;
	static bool ignore_next = true;
	static uint16_t downcount = 1;
	uint16_t dac_out = 0x8000;

	void fpga_transfer(nrfx_spim_t & spim_instance)
	{
		// spim_instance is assumed to be initialized in mode 1.
		struct spi_context_with_cs ctx{ (void*)&spim_instance, true, pin_spi_ncs_fpga };

		struct CounterSpiTransfer tx_buf(downcount);
		struct CounterSpiTransfer rx_buf;

		if(!blocking_spim_xfer_manual_cs_implementation((void*)&ctx, (uint8_t*)&tx_buf, sizeof(tx_buf),
							(uint8_t*)&rx_buf, sizeof(rx_buf)))
			return;

		if(ignore_next) {
			ignore_next = false;
		} else {

			uint64_t upcount = rx_buf.getUpCount();

			if(upcount != 0) {
				printf("%s %llu/%d\r\n", ignore_next ? "IGN" : "CTR", upcount, downcount);

				int div = 2 << downcount;
				if((div <= 0) || (div > 0x2000))
					div = 0x2000;
				else
					div = std::max(div, std::min(up, down));
				unsigned delta = 0x2000 / div;
				if(delta < 1)
					delta = 1;

				uint64_t expected_upcount = downcount*internal_reference_frequency;
				if(upcount < expected_upcount) {
					if(dac_out <= 0xffff-delta) {
						dac_out += delta;
						if(up < 100)
							up++;
					}
					printf(" \\DAC up %d => %u\r\n", delta, dac_out);
				} else if(upcount > expected_upcount) {
					if(dac_out >= 0+delta) {
						dac_out -= delta;
						if(down < 100)
							down++;
					}
					printf(" \\DAC down %d => %u\r\n", delta, dac_out);
				} else {
					if(downcount < (60*60*3)) {
						downcount += 1;
						ignore_next = true;
					}
				}

				double precision = 1.0 + ::abs(expected_upcount - upcount);
				precision /= upcount;
				printf(" \\PCN %.1E\r\n", precision);
			}
		}
	}

} // end of namespace anon

TaskHandle_t spiManager = NULL;
void spiManagerTask(void * ignored)
{
	(void)ignored;

	/* create methodology for blocking the task to wait for events */
	spimEvents = xEventGroupCreate();

	/* define SPI bus instance for use with DAC and FPGA */
	nrfx_spim_t spim_instance;
	spim_instance.p_reg = NRF_SPIM3;
	spim_instance.drv_inst_idx = NRFX_SPIM3_INST_IDX;

	/* disable all chipselects */
	nrf_gpio_cfg_output(pin_spi_ncs_dac);
	nrf_gpio_pin_set(pin_spi_ncs_dac);
	nrf_gpio_cfg_output(pin_spi_ncs_fpga);
	nrf_gpio_pin_set(pin_spi_ncs_fpga);

	/* start FPGA */
	while(!fpga_initialize(spim_instance))
		printf("failed to initialize FPGA\r\n");

	/* prepare SPI bus instance for use with DAC and FPGA.
	 * we have to re-do this as the FPGA bitstream upload needs a different
	 * SPI mode config than the DAC and the now-installed FPGA bitstream need. */
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

	/* setup DAC driver */
	struct spi_context_with_cs ad5761rSpiCtx;
	ad5761rSpiCtx.spim_instance = (void*)&spim_instance;
	ad5761rSpiCtx.cs_active_low = true;
	ad5761rSpiCtx.cs_pin = pin_spi_ncs_dac;
	Ad5761rSpiDac ad5761r((void*)&ad5761rSpiCtx, blocking_spim_xfer_manual_cs_implementation);

	/* configure DAC for our circumstances */
	ad5761r.WriteControlRegister(0b00, false, false, true, true, 0b00, 0b001);

	while(1) {
		if(!ignore_next)
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

		vTaskDelay(pdMS_TO_TICKS(100));

		fpga_transfer(spim_instance);
	};
}

