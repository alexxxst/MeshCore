#include <Arduino.h>
#include "LilyGoMiniEpaperS3.h"

uint16_t LilyGoMiniEpaperS3::getBattMilliVolts() {
  analogReadResolution(12);

  uint32_t raw = 0;
  for (int i = 0; i < 8; i++) {
    raw += analogReadMilliVolts(PIN_VBAT_READ);
  }
  raw = raw / 8;

  return (2 * raw);
}

void LilyGoMiniEpaperS3::begin() {
  btn_prev_state = HIGH;

  pinMode(PIN_VBAT_READ, INPUT);

  // Set all button pins to INPUT_PULLUP
  pinMode(PIN_USER_BTN, INPUT_PULLUP);
  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT, INPUT_PULLUP);
}
