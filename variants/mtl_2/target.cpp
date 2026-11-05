#include <Arduino.h>
#include "target.h"
#include <helpers/ArduinoHelpers.h>

#if defined(MTL_2_REPEATER)
  #include "../examples/simple_repeater/MyMesh.h"
#else
  #include "../examples/companion_radio/MyMesh.h"
#endif

extern MyMesh the_mesh;

MTL2Board board;

RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);

VolatileRTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);
#if ENV_INCLUDE_GPS
  #include <helpers/sensors/MicroNMEALocationProvider.h>
  MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1, &rtc_clock);
  EnvironmentSensorManager sensors = EnvironmentSensorManager(nmea);
#else
  EnvironmentSensorManager sensors;
#endif

#ifdef DISPLAY_CLASS
  DISPLAY_CLASS display;
  MomentaryButton user_btn(PIN_USER_BTN, 1000, true, true);
  #if !defined(MTL_2_REPEATER)
    MomentaryButton joystick_left(JOYSTICK_LEFT, 1000, true, false, false);
    MomentaryButton joystick_right(JOYSTICK_RIGHT, 1000, true, false, false);
    MomentaryButton back_btn(PIN_BACK_BTN, 1000, true, false, true);
  #endif
#endif

void power_update() {
#if defined(PIN_USER_POWER)
  static unsigned long next_power_chck = 0;
  static uint8_t _lastSwitchPower = 0xFF;

  if (millis() > next_power_chck) {
    auto* node_prefs = the_mesh.getNodePrefs();
    uint8_t newPower = digitalRead(PIN_USER_POWER) == HIGH ? USER_POWER_MAX : USER_POWER_MIN;
    if (newPower != _lastSwitchPower && digitalRead(P_LORA_BUSY) == LOW) {
      _lastSwitchPower = newPower;
      radio_driver.setTxPower(newPower);
      node_prefs->tx_power_dbm = newPower;
    }
    next_power_chck = millis() + 300;
  }
#endif
}

extern "C" void __real_loop(void);
extern "C" void __wrap_loop(void) {
    __real_loop();
    power_update();
}

bool radio_init() {
  rtc_clock.begin(Wire);

  return radio.std_init(&SPI);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);  // create new random identity
}
