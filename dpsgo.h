
#pragma once

#include <FreeRTOS.h>
#include <task.h>
#include <stream_buffer.h>

#include "bsp.h"



/* the watchdogManager checks that the system continues to operator properly,
 * or tries to recover by resetting it. */

extern TaskHandle_t watchdogManager;
void watchdogManagerTask(void * ignored);



/* the logicManager contains the controlling logic of the system. */

extern TaskHandle_t logicManager;
void logicManagerTask(void * ignored);



/* the spiManager interfaces to all devices on the SPI bus:
 * the FPGA and the DAC.
 * it is also sole owner of these devices. */

extern TaskHandle_t spiManager;
void spiManagerTask(void * ignored);

// symbols from fpga_bitstream.o:
extern const char _binary____firmware_fpga_top_bin_end;
extern const char _binary____firmware_fpga_top_bin_start;
#define FPGA_BITSTREAM (&_binary____firmware_fpga_top_bin_start)
#define FPGA_BITSTREAM_SIZE ((&_binary____firmware_fpga_top_bin_end) - FPGA_BITSTREAM)



/* the i2cManager interfaces to all devices on the I2C bus:
 * the frequency synthesizer, the display and the temperatur sensors.
 * as the owner of the display, it is also owner of the console.
 * it is also sole owner of these devices. */

extern TaskHandle_t i2cManager;
void i2cManagerTask(void * ignored);

// https://www.freertos.org/RTOS-stream-message-buffers.html
// => only write to console if in critical section.
extern StreamBufferHandle_t console;



/* the uartManager interfaces to all devices on the UART:
 * the GPS receiver.
 * it is also sole owner of these devices. */

extern TaskHandle_t uartManager;
void uartManagerTask(void * ignored);

