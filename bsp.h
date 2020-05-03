
#pragma once

#include "nrfx_gpiote.h"
#include "nrfx_uarte.h"

const nrfx_gpiote_pin_t pin_fault_led = NRF_GPIO_PIN_MAP(0, 8);

const nrfx_gpiote_pin_t pin_pwr_en_1v2 = NRF_GPIO_PIN_MAP(0, 9);
const nrfx_gpiote_pin_t pin_pwr_en_12v = NRF_GPIO_PIN_MAP(0, 10);
const nrfx_gpiote_pin_t pin_pwr_nen_10v = NRF_GPIO_PIN_MAP(0, 17);
const nrfx_gpiote_pin_t pin_pwr_good = NRF_GPIO_PIN_MAP(1, 6);

const nrfx_gpiote_pin_t pin_i2c_scl = NRF_GPIO_PIN_MAP(0, 5);
const nrfx_gpiote_pin_t pin_i2c_sda = NRF_GPIO_PIN_MAP(0, 6);

const nrfx_gpiote_pin_t pin_spi_sdo = NRF_GPIO_PIN_MAP(0, 29);
const nrfx_gpiote_pin_t pin_spi_clk = NRF_GPIO_PIN_MAP(1, 10);
const nrfx_gpiote_pin_t pin_spi_sdi = NRF_GPIO_PIN_MAP(1, 13);
const nrfx_gpiote_pin_t pin_spi_ncs_fpga = NRF_GPIO_PIN_MAP(0, 28);
const nrfx_gpiote_pin_t pin_spi_ncs_dac = NRF_GPIO_PIN_MAP(1, 11);

const nrfx_gpiote_pin_t pin_fpga_nreset = NRF_GPIO_PIN_MAP(0, 30);
const nrfx_gpiote_pin_t pin_fpga_cdone = NRF_GPIO_PIN_MAP(0, 31);
const nrfx_gpiote_pin_t pin_fpga_wdt = NRF_GPIO_PIN_MAP(0, 26);
const nrfx_gpiote_pin_t pin_fpga_int = NRF_GPIO_PIN_MAP(0, 15);

const nrfx_gpiote_pin_t pin_dac_value = NRF_GPIO_PIN_MAP(0, 3);
const nrfx_gpiote_pin_t pin_dac_nalert = NRF_GPIO_PIN_MAP(0, 22);
const nrfx_gpiote_pin_t pin_dac_nclear = NRF_GPIO_PIN_MAP(0, 24);
const nrfx_gpiote_pin_t pin_dac_nreset = NRF_GPIO_PIN_MAP(1, 2);
const nrfx_gpiote_pin_t pin_dac_nload = NRF_GPIO_PIN_MAP(1, 4);

const nrfx_gpiote_pin_t pin_gps_pulse = NRF_GPIO_PIN_MAP(1, 9);
const nrfx_gpiote_pin_t pin_gps_txd = NRF_GPIO_PIN_MAP(0, 4);
const nrfx_gpiote_pin_t pin_gps_nreset = NRF_GPIO_PIN_MAP(0, 7);
const nrfx_gpiote_pin_t pin_gps_rxd = NRF_GPIO_PIN_MAP(0, 12);

const nrfx_gpiote_pin_t pin_pll_nen = NRF_GPIO_PIN_MAP(0, 13);
const nrfx_gpiote_pin_t pin_pll_int = NRF_GPIO_PIN_MAP(0, 20);

const nrfx_gpiote_pin_t pin_ocxo_vref = NRF_GPIO_PIN_MAP(0, 2);
