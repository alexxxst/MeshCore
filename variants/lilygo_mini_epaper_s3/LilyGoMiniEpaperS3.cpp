#include "LilyGoMiniEpaperS3.h"
#include <Arduino.h>
#include <target.h>

uint16_t LilyGoMiniEpaperS3::getBattMilliVolts() {
  analogReadResolution(12);

  uint32_t raw = 0;
  for (int i = 0; i < 8; i++) {
    raw += analogReadMilliVolts(PIN_VBAT_READ);
  }
  raw = raw / 8;

  return 2 * raw;
}

void LilyGoMiniEpaperS3::begin() {
  ESP32Board::begin();
  pinMode(PIN_USER_BTN, INPUT_PULLUP);

#ifdef UI_HAS_JOYSTICK
  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT, INPUT_PULLUP);
#endif

  pinMode(PIN_DISPLAY_EN, OUTPUT);
  digitalWrite(PIN_DISPLAY_EN, HIGH);
}
