#include <Arduino.h>
#include <Wire.h>

#include "MTL2Board.h"

void MTL2Board::begin() {
    NRF52Board::begin();
    btn_prev_state = HIGH;

    pinMode(PIN_VBAT_READ, INPUT);

    #if defined(PIN_BOARD_SDA) && defined(PIN_BOARD_SCL)
      Wire.setPins(PIN_BOARD_SDA, PIN_BOARD_SCL);
    #endif

    Wire.begin();

    nrf_gpio_cfg_default(SX126X_POWER_EN);
    nrf_gpio_cfg_output(SX126X_POWER_EN);
    nrf_gpio_pin_write(SX126X_POWER_EN, 1);
    delay(10);   // give sx1262 some time to power up
}