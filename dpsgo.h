
#pragma once

#include <FreeRTOS.h>
#include <task.h>

#include "bsp.h"

extern TaskHandle_t watchdogManager;
void watchdogManagerTask(void * ignored);

extern TaskHandle_t uartManager;
void uartManagerTask(void * ignored);

extern TaskHandle_t spiManager;
void spiManagerTask(void * ignored);

extern TaskHandle_t i2cManager;
void i2cManagerTask(void * ignored);

extern TaskHandle_t logicManager;
void logicManagerTask(void * ignored);

