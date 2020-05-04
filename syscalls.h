#pragma once

#include "embedded_drivers/ssd1306_i2c_display.h"
#include "embedded_drivers/nrfx/uarte.h"

// use UARTE for all stdio
void stdio_via_uarte(Uarte * uarte);

// use SSD1306 for all stdio
void stdio_via_ssd1306(embedded_drivers::Ssd1306I2cDisplay * new_display);

