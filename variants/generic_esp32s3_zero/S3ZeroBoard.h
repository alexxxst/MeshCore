#pragma once

#include <Arduino.h>
#include <helpers/ESP32Board.h>

// Waveshare ESP32-S3-Zero has a single onboard WS2812 on GPIO21.
#ifndef S3ZERO_RGB_LED
  #define S3ZERO_RGB_LED 21
#endif

// WS2812 brightness (0-255). Kept low: this LED is a local diagnostic, not a
// beacon, and the node may be solar powered.
#define S3ZERO_LED_BRIGHTNESS 40

// Boot self-test / reset reason codes, shown once on power-up:
//   2 x green = firmware alive, radio initialised, identity loaded
//   2 x red   = previous boot ended in panic or watchdog reset
//   3 x red   = previous boot ended in brownout (supply sagged)
#define S3ZERO_BLINK_ON_MS   150
#define S3ZERO_BLINK_OFF_MS  150

class S3ZeroBoard : public ESP32Board {
  static void setLed(uint8_t r, uint8_t g, uint8_t b) {
    neopixelWrite(S3ZERO_RGB_LED, r, g, b);
  }

  static void blink(uint8_t r, uint8_t g, uint8_t b, uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
      setLed(r, g, b);
      delay(S3ZERO_BLINK_ON_MS);
      setLed(0, 0, 0);
      delay(S3ZERO_BLINK_OFF_MS);
    }
  }

public:
  // NOTE: ESP32Board::begin() is not virtual (MainBoard has no such hook), it
  // is called on the concrete board object from the example's setup().
  void begin() {
    ESP32Board::begin();

    // Report why the previous boot ended before anything touches the radio.
    switch (getResetReason()) {
      case ESP_RST_BROWNOUT:
        blink(S3ZERO_LED_BRIGHTNESS, 0, 0, 3);
        break;
      case ESP_RST_PANIC:
      case ESP_RST_INT_WDT:
      case ESP_RST_TASK_WDT:
      case ESP_RST_WDT:
        blink(S3ZERO_LED_BRIGHTNESS, 0, 0, 2);
        break;
      default:
        break;   // power-on, software reset, deep-sleep wake: nothing to report
    }
  }

  // Called by the example after radio_init() succeeded and the identity is
  // loaded. Radio init failure halts before this point, so green means "OK".
  void onBootComplete() override {
    ESP32Board::onBootComplete();
    blink(0, S3ZERO_LED_BRIGHTNESS, 0, 2);
  }

  // Red only while the radio is actually transmitting. onAfterTransmit() is
  // called by the dispatcher both on completion and on TX timeout, so the LED
  // always goes dark by itself; no timer or main-loop polling needed.
  void onBeforeTransmit() override {
    setLed(S3ZERO_LED_BRIGHTNESS, 0, 0);
  }

  void onAfterTransmit() override {
    setLed(0, 0, 0);
  }
};
