#pragma once

#include <MeshCore.h>
#include <Arduino.h>
#include <helpers/NRF52Board.h>

#define P_LORA_NSS 45	    	//P1.13
#define P_LORA_DIO_1 10	  	//P0.10
#define P_LORA_RESET 9	  	//P0.09
#define P_LORA_BUSY  29	  	//P0.29
#define P_LORA_MISO  2	  	//P0.02
#define P_LORA_SCLK  43	  	//P1.11
#define P_LORA_MOSI  47 		//P1.15
#define SX126X_POWER_EN  38 //P1.06
#define SX126X_RXEN 17	    	//P0.17
#define SX126X_TXEN RADIOLIB_NC
#define SX126X_DIO2_AS_RF_SWITCH  true
#define SX126X_DIO3_TCXO_VOLTAGE (1.8f)

#define  PIN_VBAT_READ 31 	// P0.31
#define  ADC_MULTIPLIER   (1.51f) // dependent on voltage divider resistors. TODO: more accurate battery tracking

class MTL2Board : public NRF52BoardDCDC {
protected:
  uint8_t btn_prev_state;
  float adc_mult = ADC_MULTIPLIER;

public:
  MTL2Board() : NRF52Board("MTL_2_OTA") {}
  void begin();

  #define BATTERY_SAMPLES 8

  uint16_t getBattMilliVolts() override {
    analogReadResolution(12);

    uint32_t raw = 0;
    for (int i = 0; i < BATTERY_SAMPLES; i++) {
      raw += analogRead(PIN_VBAT_READ);
    }
    raw = raw / BATTERY_SAMPLES;
    return (adc_mult * raw);
  }

  bool setAdcMultiplier(float multiplier) override {
    if (multiplier == 0.0f) {
      adc_mult = ADC_MULTIPLIER;}
    else {
      adc_mult = multiplier;
    }
    return true;
  }
  float getAdcMultiplier() const override {
    if (adc_mult == 0.0f) {
      return ADC_MULTIPLIER;
    } else {
      return adc_mult;
    }
  }

  const char* getManufacturerName() const override {
    return "MTL_2 DIY";
  }

  int buttonStateChanged() {
    #ifdef BUTTON_PIN
      uint8_t v = digitalRead(BUTTON_PIN);
      if (v != btn_prev_state) {
        btn_prev_state = v;
        return (v == LOW) ? 1 : -1;
      }
    #endif
      return 0;
  }

  void shutdownPeripherals() override {
    NRF52Board::shutdownPeripherals();

    #ifdef SX126X_POWER_EN
      pinMode(SX126X_POWER_EN, OUTPUT);
      digitalWrite(SX126X_POWER_EN, LOW);
    #endif

    #ifdef PIN_USER_BTN
      nrf_gpio_cfg_input(PIN_USER_BTN, NRF_GPIO_PIN_PULLUP);
      nrf_gpio_cfg_sense_set(PIN_USER_BTN, NRF_GPIO_PIN_SENSE_LOW);
    #endif
  }
};
