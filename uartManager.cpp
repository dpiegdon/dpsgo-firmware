
#include <string>
#include <algorithm>
#include <unistd.h>

#include "dpsgo.h"

#include <event_groups.h>

#define BUFFER_SIZE 32
#define NMEA_MAX_SIZE 85
static uint8_t * hot_buffer;
static uint8_t * cold_buffer;
static volatile size_t hot_received;
static size_t cold_received;

static EventGroupHandle_t uarteEvents;
#define UARTE_EVENT_RX  (1 << 0)
#define UARTE_EVENT_TX  (1 << 1)
#define UARTE_EVENT_ERR (1 << 2)
#define UARTE_EVENTS    ( UARTE_EVENT_RX | UARTE_EVENT_TX | UARTE_EVENT_ERR )

namespace { /* anon */
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

	void setupUarte(nrfx_uarte_t & instance)
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

	int hexDigit2int(int digit)
	{
		if(digit <= '9')
			return digit - '0';
		else if(digit >= 'a')
			return digit - 'a' + 10;
		else
			return digit - 'A' + 10;
	}

	std::string nmeaStripFrontToken(std::string & payload)
	{
		size_t n = payload.find_first_of(',');
		std::string ret = payload.substr(0, n);
		payload = payload.substr(n+1, std::string::npos);
		return ret;
	}

	void handleNmeaFixData(std::string & payload)
	{ // GPGGA
		std::string tok;
		size_t n = std::count(payload.begin(), payload.end(), ',');
		if(n < 9)
			return;

		/*
		// UTC time
		tok = nmeaStripFrontToken(payload);
		printf("TIME UTC %s\r\n", tok.c_str());

		// latitude
		tok = nmeaStripFrontToken(payload);
		printf("LAT %s  ", tok.c_str());

		// longitude
		tok = nmeaStripFrontToken(payload);
		printf("LONG %s\r\n", tok.c_str());

		// fix quality
		tok = nmeaStripFrontToken(payload);
		printf("FIXQ %s  ", tok.c_str());

		// number of sats tracked
		tok = nmeaStripFrontToken(payload);
		printf("SATS %s  ", tok.c_str());

		// horizontal dilution
		tok = nmeaStripFrontToken(payload);
		printf("DILU %s\r\n", tok.c_str());

		// altitude [m]
		tok = nmeaStripFrontToken(payload);
		printf("ALT %s  ", tok.c_str());

		// geoid height
		tok = nmeaStripFrontToken(payload);
		printf("GEOH %s\r\n", tok.c_str());

		// time since DGPS update [s]
		tok = nmeaStripFrontToken(payload);
		printf("DGPST %ss  ", tok.c_str());

		// DPGS station id
		tok = nmeaStripFrontToken(payload);
		printf("ID %s\r\n\n", tok.c_str());
		*/
	}

	void handleNmeaLatLong(std::string & payload)
	{ // GPGLL
	}

	void handleNmeaOverallSatReception(std::string & payload)
	{ // GPGSA
	}

	void handleNmeaDetailedSatData(std::string & payload)
	{ // GPGSV
	}

	void handleNmeaMinimumRecommendedData(std::string & payload)
	{ // GPRMC
	}

	void handleNmeaMessage(std::string msg)
	/* expects message without leading '$'. */
	{
		size_t i;
		size_t len;
		// strip whitespace tail
		while(msg.back() == '\n' || msg.back() == '\r')
			msg.pop_back();

		// strip and verify (optional) checksum tail
		i = msg.find_last_of('*');
		if(std::string::npos != i) {
			len = msg.length();
			if(3 != len - i)
				return;

			uint8_t checksum = 16 * hexDigit2int(msg[i+1]) + hexDigit2int(msg[i+2]);
			msg = msg.substr(0, i);

			uint8_t selfsummed = 0;
			for(auto it: msg)
				selfsummed ^= it;

			if(checksum != selfsummed)
				return;
		}

		i = msg.find_first_of(',');
		std::string msgtype = msg.substr(0, i);
		msg = msg.substr(i+1, std::string::npos);

		if("GPGGA" == msgtype)
			handleNmeaFixData(msg);
		else if("GPGLL" == msgtype)
			handleNmeaLatLong(msg);
		else if("GPGSA" == msgtype)
			handleNmeaOverallSatReception(msg);
		else if("GPGSV" == msgtype)
			handleNmeaDetailedSatData(msg);
		else if("GPRMC" == msgtype)
			handleNmeaMinimumRecommendedData(msg);
		// else message irrelevant
	}

} // namespace anon

TaskHandle_t uartManager = NULL;
void uartManagerTask(void * ignored)
{
	(void)ignored;

	nrfx_uarte_t uarteInstance;

	uarteEvents = xEventGroupCreate();

	setupUarte(uarteInstance);

	// FIXME wait for GO signal here.

	uint8_t buffer0[BUFFER_SIZE];
	uint8_t buffer1[BUFFER_SIZE];

	hot_buffer = buffer0;
	hot_received = 0;
	cold_buffer = buffer1;
	cold_received = 0;

	std::string nmeaBuffer;
	nmeaBuffer.reserve(BUFFER_SIZE + NMEA_MAX_SIZE);

	while(1) {
		// receive in non-blocking way
		hot_received = 0;
		assert(NRFX_SUCCESS == nrfx_uarte_rx(&uarteInstance, hot_buffer, BUFFER_SIZE));

		// and transmit previously received
		if(cold_received) {
			nmeaBuffer.append((char*)cold_buffer, cold_received);
			if(nmeaBuffer.length() > (BUFFER_SIZE + NMEA_MAX_SIZE)) {
				nmeaBuffer.clear(); // drop corrupted messages
			} else {
				size_t next;
				while(std::string::npos != (next = nmeaBuffer.find_first_of('$', 1))) {
					if('$' == nmeaBuffer[0])
						handleNmeaMessage(nmeaBuffer.substr(1, next-1));
					nmeaBuffer = nmeaBuffer.substr(next, std::string::npos);
				}
			}
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

