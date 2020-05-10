
#include "dpsgo.h"

#include "embedded_drivers/nrfx/uarte.h"

TaskHandle_t uartManager = NULL;
void uartManagerTask(void * ignored)
{
	(void)ignored;

	Uarte uart(pin_gps_rxd, pin_gps_txd, pin_gps_cts, pin_gps_rts);

	while(1) { };
	/*
	while(1) {
		int ret;
		char c;
		ret = uart.Read(&c, sizeof(c));
		if(ret == NRFX_SUCCESS) {
			//disp.PutChar(c);
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
	*/
}

