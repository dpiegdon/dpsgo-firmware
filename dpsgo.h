
#pragma once

#include <FreeRTOS.h>
#include <task.h>
//#include <stream_buffer.h>

#include "bsp.h"

// symbols from fpga_bitstream.o:
extern const char _binary____firmware_fpga_top_bin_end;
extern const char _binary____firmware_fpga_top_bin_start;
#define FPGA_BITSTREAM (&_binary____firmware_fpga_top_bin_start)
#define FPGA_BITSTREAM_SIZE ((&_binary____firmware_fpga_top_bin_end) - FPGA_BITSTREAM)


extern TaskHandle_t watchdogManager;
void watchdogManagerTask(void * ignored);

extern TaskHandle_t logicManager;
void logicManagerTask(void * ignored);

extern TaskHandle_t spiManager;
void spiManagerTask(void * ignored);

extern TaskHandle_t i2cManager;
void i2cManagerTask(void * ignored);

extern TaskHandle_t uartManager;
void uartManagerTask(void * ignored);

