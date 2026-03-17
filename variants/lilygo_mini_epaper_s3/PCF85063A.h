#pragma once

#include "Arduino.h"
#include "Wire.h"
#include "RTClib.h"

#define I2C_ADDR           0x51

#define RTC_CTRL_1         0x0
#define RTC_CTRL_2         0x01
#define RTC_OFFSET         0x02
#define RTC_RAM_by         0x03

#define RTC_SECOND_ADDR    0x04
#define RTC_MINUTE_ADDR    0x05
#define RTC_HOUR_ADDR      0x06
#define RTC_DAY_ADDR       0x07
#define RTC_WDAY_ADDR      0x08
#define RTC_MONTH_ADDR     0x09
#define RTC_YEAR_ADDR      0x0A // years 0-99; calculate real year = 1970 + RCC reg year

class PCF85063A {
  TwoWire& _wire = Wire;
public:
  PCF85063A();
  enum CountdownSrcClock {
    TIMER_CLOCK_4096HZ = 0,
    TIMER_CLOCK_64HZ = 1,
    TIMER_CLOCK_1HZ = 2,
    TIMER_CLOCK_1PER60HZ = 3
  };

  void begin(TwoWire& wire);
  void setTime(uint8_t hour, uint8_t minute, uint8_t sec);
  void setDate(uint8_t weekday, uint8_t day, uint8_t month, uint8_t yr);
  void readTime();
  DateTime now();
  void adjust(const DateTime &dt);
  void reset();

private:
  uint8_t decToBcd(uint8_t val);
  uint8_t bcdToDec(uint8_t val);
  /* time variables */
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t day;
  uint8_t weekday;
  uint8_t month;
  uint16_t year;
};