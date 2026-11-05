#include "variant.h"
#include "wiring_constants.h"
#include "wiring_digital.h"

const uint32_t g_ADigitalPinMap[] = {
  0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,  // P0.00-P0.31
  32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47   // P1.00-P1.15
};

void initVariant()
{
  nrf_gpio_cfg_default(PIN_USER_BTN);
  nrf_gpio_cfg_default(JOYSTICK_LEFT);
  nrf_gpio_cfg_default(JOYSTICK_RIGHT);
  nrf_gpio_cfg_default(PIN_USER_POWER);

  nrf_gpio_cfg_input(PIN_USER_BTN, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(JOYSTICK_LEFT, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(JOYSTICK_RIGHT, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(PIN_USER_POWER, NRF_GPIO_PIN_PULLUP);
}
