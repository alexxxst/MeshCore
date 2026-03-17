#include "LilyGoMiniEpaperS3.h"
#include "PCF85063A.h"

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

static PCF85063A rtc_85063a;
static bool rtc_85063a_success = false;

bool PCF85063A_Clock::i2c_probe(TwoWire& wire, uint8_t addr) {
  wire.beginTransmission(addr);
  uint8_t error = wire.endTransmission();
  return (error == 0);
}

void PCF85063A_Clock::begin(TwoWire& wire) {
  if(i2c_probe(wire, I2C_ADDR)){
    rtc_85063a.begin(wire);
    rtc_85063a_success = true;
    // rtc_85063a.reset();
  }
}
uint32_t PCF85063A_Clock::getCurrentTime() {
  if(rtc_85063a_success){
    return rtc_85063a.now().unixtime();
  }
  return _fallback->getCurrentTime();
}

void PCF85063A_Clock::setCurrentTime(uint32_t time) {
  if (rtc_85063a_success) {
    rtc_85063a.adjust(DateTime(time));
  } else {
    _fallback->setCurrentTime(time);
  }
}
