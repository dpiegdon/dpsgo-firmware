
#include <unistd.h>

#include "dpsgo.h"

#include <event_groups.h>

#define BUFFER_SIZE 32
static uint8_t * hot_buffer;
static uint8_t * cold_buffer;
static volatile size_t hot_received;
static size_t cold_received;

static EventGroupHandle_t uarteEvents;
#define UARTE_EVENT_RX  (1 << 0)
#define UARTE_EVENT_TX  (1 << 1)
#define UARTE_EVENT_ERR (1 << 2)
#define UARTE_EVENTS    ( UARTE_EVENT_RX | UARTE_EVENT_TX | UARTE_EVENT_ERR )

void uarteEventHandler(nrfx_uarte_event_t const * event, void * context)
{
	(void)context;
	EventBits_t events = 0;

	switch(event->type) {
		case NRFX_UARTE_EVT_ERROR:
			events |= UARTE_EVENT_ERR;
			hot_received = snprintf((char*)hot_buffer, BUFFER_SIZE,
					"UARTE err 0x%08lx\r\n", event->data.error.error_mask);
			break;
		case NRFX_UARTE_EVT_TX_DONE:
			events |= UARTE_EVENT_TX;
			break;
		case NRFX_UARTE_EVT_RX_DONE:
			events |= UARTE_EVENT_RX;
			hot_received = event->data.rxtx.bytes;
			break;
	}

	BaseType_t xTrue = pdTRUE;
	xEventGroupSetBitsFromISR(uarteEvents, events, &xTrue);
}

void setupUart(nrfx_uarte_t & instance)
{
	nrfx_uarte_config_t config{};

	instance.p_reg = NRFX_CONCAT_2(NRF_UARTE, 0);
	instance.drv_inst_idx = NRFX_CONCAT_3(NRFX_UARTE, 0, _INST_IDX);

	config.p_context = NULL;

	config.hwfc     = static_cast<nrf_uarte_hwfc_t>(0); // no hardware flow control
	config.parity   = static_cast<nrf_uarte_parity_t>(0); // no parity
	config.baudrate = static_cast<nrf_uarte_baudrate_t>(2576384); // 9600 baud
	config.interrupt_priority = 7; // low priority

	config.pseltxd = pin_gps_rxd;
	config.pselrxd = pin_gps_txd;
	config.pselcts = pin_gps_cts;
	config.pselrts = pin_gps_rts;

	assert(NRFX_SUCCESS == nrfx_uarte_init(&instance, &config, uarteEventHandler));
}

TaskHandle_t uartManager = NULL;
void uartManagerTask(void * ignored)
{
	(void)ignored;

	nrfx_uarte_t uarteInstance;

	uarteEvents = xEventGroupCreate();

	setupUart(uarteInstance);

	// FIXME wait for GO signal here.

	uint8_t buffer0[BUFFER_SIZE];
	uint8_t buffer1[BUFFER_SIZE];

	hot_buffer = buffer0;
	hot_received = 0;
	cold_buffer = buffer1;
	cold_received = 0;

	while(1) {
		// receive in non-blocking way
		hot_received = 0;
		assert(NRFX_SUCCESS == nrfx_uarte_rx(&uarteInstance, hot_buffer, BUFFER_SIZE));

		// and transmit previously received
		if(cold_received) {
			// FIXME some blocking way would be better,
			// or dissect into GPS messages (lines)
			write(STDOUT_FILENO, cold_buffer, cold_received);
		}

		EventBits_t events;
		while(0 == (events = xEventGroupWaitBits(uarteEvents, UARTE_EVENTS, true, false, pdMS_TO_TICKS(1000))))
			/* wait */ ;

		uint8_t * tmp;
		tmp = cold_buffer;
		cold_buffer = hot_buffer;
		cold_received = hot_received;
		hot_buffer = tmp;
	}
}

