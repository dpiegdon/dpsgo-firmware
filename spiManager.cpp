
#include "dpsgo.h"

TaskHandle_t spiManager = NULL;
void spiManagerTask(void * ignored)
{
	(void)ignored;

	while(1) {
		vTaskDelay(configTICK_RATE_HZ/4);
	};
}

