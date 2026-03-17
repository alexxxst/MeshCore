#include "PCF85063A.h"

// INIT
PCF85063A::PCF85063A() {}
void PCF85063A::begin(TwoWire &wire) {
  _wire = wire;
}

// PUBLIC
void PCF85063A::setTime(uint8_t hour, uint8_t minute, uint8_t second) {
  _wire.beginTransmission(I2C_ADDR);
  _wire.write(RTC_SECOND_ADDR);
  _wire.write(decToBcd(second));
  _wire.write(decToBcd(minute));
  _wire.write(decToBcd(hour));
  _wire.endTransmission();
}

void PCF85063A::setDate(uint8_t weekday, uint8_t day, uint8_t month, uint8_t yr) {
  year = yr - 1970; // convert to RTC year format 0-99

  _wire.beginTransmission(I2C_ADDR);
  _wire.write(RTC_DAY_ADDR);
  _wire.write(decToBcd(day));
  _wire.write(decToBcd(weekday));
  _wire.write(decToBcd(month));
  _wire.write(decToBcd(year));
  _wire.endTransmission();
}

void PCF85063A::readTime() {
  _wire.beginTransmission(I2C_ADDR);
  _wire.write(RTC_SECOND_ADDR); // datasheet 8.4.
  _wire.endTransmission();

  _wire.requestFrom(I2C_ADDR, 7);

  while (_wire.available()) {
    second = bcdToDec(_wire.read() & 0x7F); // ignore bit 7
    minute = bcdToDec(_wire.read() & 0x7F);
    hour = bcdToDec(_wire.read() & 0x3F); // ignore bits 7 & 6
    day = bcdToDec(_wire.read() & 0x3F);
    weekday = bcdToDec(_wire.read() & 0x07); // ignore bits 7,6,5,4 & 3
    month = bcdToDec(_wire.read() & 0x1F);   // ignore bits 7,6 & 5
    year = bcdToDec(_wire.read()) + 1970;
  }
}

/**************************************************************************/
/*!
    @brief  Get the current date/time
    @return DateTime object containing the current date/time
*/
/**************************************************************************/
DateTime PCF85063A::now() {
  readTime();
  // Serial.printf("%d-%d-%d %d:%d:%d", year, month, day, hour, minute, second);
  // Serial.println();
  return DateTime(year, month, day, hour, minute, second);
}

/**************************************************************************/
/*!
    @brief  Set the date and time
    @param dt DateTime to set
*/
/**************************************************************************/
void PCF85063A::adjust(const DateTime &dt) {
  setDate(0, dt.day(), dt.month(), dt.year());
  setTime(dt.hour(), dt.minute(), dt.second());
}

void PCF85063A::reset() // datasheet 8.2.1.3.
{
  _wire.beginTransmission(I2C_ADDR);
  _wire.write(RTC_CTRL_1);
  _wire.write(0x58);
  _wire.endTransmission();
}

// PRIVATE
uint8_t PCF85063A::decToBcd(uint8_t val) {
  return ((val / 10 * 16) + (val % 10));
}

uint8_t PCF85063A::bcdToDec(uint8_t val) {
  return ((val / 16 * 10) + (val % 16));
}