#pragma once

#include <Arduino.h>
#include <helpers/ESP32Board.h>

class LilyGoMiniEpaperS3 : public ESP32Board {
public:
  LilyGoMiniEpaperS3() { }

  const char* getManufacturerName() const override {
    return "LilyGo Mini Epaper S3";
  }
  void begin();
  uint16_t getBattMilliVolts() override;
};
