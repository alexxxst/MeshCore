#pragma once

#include <Arduino.h>
#include <helpers/ESP32Board.h>

class LilyGoMiniEpaperS3 : public ESP32Board {
public:
  LilyGoMiniEpaperS3() { }

  const char* getManufacturerName() const override {
    return "LilyGo Mini E-paper S3";
  }
  void begin();
  uint16_t getBattMilliVolts() override;
};

class PCF85063A_Clock : public mesh::RTCClock {
  mesh::RTCClock* _fallback;
public:
  PCF85063A_Clock(mesh::RTCClock& fallback) : _fallback(&fallback) { }
  static bool i2c_probe(TwoWire &wire, uint8_t addr);
  void begin(TwoWire& wire);
  uint32_t getCurrentTime() override;
  void setCurrentTime(uint32_t time) override;

  void tick() override {
    _fallback->tick();   // is typically VolatileRTCClock, which now needs tick()
  }
};