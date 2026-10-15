#include "UITask.h"

#include <RTClib.h>
#include <Arduino.h>
#include <target.h>

#ifndef AUTO_OFF_MILLIS
  #define AUTO_OFF_MILLIS      15000  // 15 seconds (0 on e-ink, e.g. LilyGo T-Echo)
#endif
#define BOOT_SCREEN_MILLIS   4000   // 4 seconds

// E-ink partial refresh drives the whole panel with a weak waveform, so pixels that
// don't change slowly fade. Periodically repaint the panel (splash frame -> content
// frame) to restore contrast. Lower = crisper but more visible "flash" during redraw.
#ifndef EINK_RESTORE_MILLIS
  #define EINK_RESTORE_MILLIS  60000
#endif

// 'meshcore', 128x13px
static const uint8_t meshcore_logo [] PROGMEM = {
    0x3c, 0x01, 0xe3, 0xff, 0xc7, 0xff, 0x8f, 0x03, 0x87, 0xfe, 0x1f, 0xfe, 0x1f, 0xfe, 0x1f, 0xfe,
    0x3c, 0x03, 0xe3, 0xff, 0xc7, 0xff, 0x8e, 0x03, 0x8f, 0xfe, 0x3f, 0xfe, 0x1f, 0xff, 0x1f, 0xfe,
    0x3e, 0x03, 0xc3, 0xff, 0x8f, 0xff, 0x0e, 0x07, 0x8f, 0xfe, 0x7f, 0xfe, 0x1f, 0xff, 0x1f, 0xfc,
    0x3e, 0x07, 0xc7, 0x80, 0x0e, 0x00, 0x0e, 0x07, 0x9e, 0x00, 0x78, 0x0e, 0x3c, 0x0f, 0x1c, 0x00,
    0x3e, 0x0f, 0xc7, 0x80, 0x1e, 0x00, 0x0e, 0x07, 0x1e, 0x00, 0x70, 0x0e, 0x38, 0x0f, 0x3c, 0x00,
    0x7f, 0x0f, 0xc7, 0xfe, 0x1f, 0xfc, 0x1f, 0xff, 0x1c, 0x00, 0x70, 0x0e, 0x38, 0x0e, 0x3f, 0xf8,
    0x7f, 0x1f, 0xc7, 0xfe, 0x0f, 0xff, 0x1f, 0xff, 0x1c, 0x00, 0xf0, 0x0e, 0x38, 0x0e, 0x3f, 0xf8,
    0x7f, 0x3f, 0xc7, 0xfe, 0x0f, 0xff, 0x1f, 0xff, 0x1c, 0x00, 0xf0, 0x1e, 0x3f, 0xfe, 0x3f, 0xf0,
    0x77, 0x3b, 0x87, 0x00, 0x00, 0x07, 0x1c, 0x0f, 0x3c, 0x00, 0xe0, 0x1c, 0x7f, 0xfc, 0x38, 0x00,
    0x77, 0xfb, 0x8f, 0x00, 0x00, 0x07, 0x1c, 0x0f, 0x3c, 0x00, 0xe0, 0x1c, 0x7f, 0xf8, 0x38, 0x00,
    0x73, 0xf3, 0x8f, 0xff, 0x0f, 0xff, 0x1c, 0x0e, 0x3f, 0xf8, 0xff, 0xfc, 0x70, 0x78, 0x7f, 0xf8,
    0xe3, 0xe3, 0x8f, 0xff, 0x1f, 0xfe, 0x3c, 0x0e, 0x3f, 0xf8, 0xff, 0xfc, 0x70, 0x3c, 0x7f, 0xf8,
    0xe3, 0xe3, 0x8f, 0xff, 0x1f, 0xfc, 0x3c, 0x0e, 0x1f, 0xf8, 0xff, 0xf8, 0x70, 0x3c, 0x7f, 0xf8,
};

void UITask::begin(const char* name, const char* group) {
  _prevBtnState = HIGH;
  _auto_off = millis() + AUTO_OFF_MILLIS;
  _display->turnOn();

  sprintf(_node_name, "%s in %s", name, group);

  // Draw a distinct splash first: e-ink partial refresh only repaints changed
  // pixels, so the real screen must differ from the previous frame to be drawn
  // (same idea as companion_radio "Loading..." / repeater "Please wait...").
  _display->startFrame();
  _display->setTextSize(1);
  _display->drawTextCentered(_display->width() / 2, 28, "Loading...");
  _display->endFrame();

  _next_reset = millis() + EINK_RESTORE_MILLIS;
}

void UITask::renderCurrScreen() {

  // meshcore logo
  _display->setColor(UIColor::corp_blue);
  _display->drawXbm((_display->width() - 128) / 2, 3, meshcore_logo, 128, 13);

  _display->setTextSize(1);
  _display->setColor(UIColor::corp_blue);

  // battery info
  sprintf(_info, "Batt %1.2f v", static_cast<float>(board.getBattMilliVolts()) / 1000.0f);
  const uint16_t infoWidth = _display->getTextWidth(_info);
  _display->setCursor((_display->width() - infoWidth) / 2, 22);
  _display->print(_info);

  // node name
  const uint16_t nameWidth = _display->getTextWidth(_node_name);
  _display->setCursor((_display->width() - nameWidth) / 2, 35);
  _display->print(_node_name);

  // stats
  sprintf(_stats, "Q%d R%d S%d L%d", _quiet, _total_request, _total_sent, _last_msg_count);
  const uint16_t statsWidth = _display->getTextWidth(_stats);
  _display->setCursor((_display->width() - statsWidth) / 2, 48);
  _display->print(_stats);

  // time
  const auto dt = DateTime(_timestamp);
  sprintf(_time, "%02d:%02d:%02d %d/%d/%d UT", dt.hour(), dt.minute(), dt.second(), dt.day(), dt.month(), dt.year());
  const uint16_t timeWidth = _display->getTextWidth(_time);
  _display->setCursor((_display->width() - timeWidth) / 2, 67);
  _display->print(_time);

#if ENV_INCLUDE_GPS == 1
  // gps
  LocationProvider *nmea = sensors.getLocationProvider();
  if (nmea != nullptr) {
    sprintf(_gps, "%s sats: %d", nmea->isValid() ? "GPS fix" : "GPS no fix", nmea->satellitesCount());
  } else {
    sprintf(_gps, "No GPS");
  }
  const uint16_t gpsWidth = _display->getTextWidth(_gps);
  _display->setCursor((_display->width() - gpsWidth) / 2, 85);
  _display->print(_gps);
#endif

}

void UITask::loop(const bool quiet, const unsigned long total_request, const unsigned long total_sent, const unsigned long time, const unsigned long last_msg_count) {

  // user button
#ifdef PIN_USER_BTN
  if (millis() >= _next_read) {
    const int btnState = digitalRead(PIN_USER_BTN);
    if (btnState != _prevBtnState) {
      if (btnState == LOW) {  // pressed?
        if (_display->isOn()) {
          // TODO: any action ?
        } else {
          _display->turnOn();
        }
        _auto_off = millis() + AUTO_OFF_MILLIS;   // extend auto-off timer
      }
      _prevBtnState = btnState;
    }
    _next_read = millis() + 200;  // 5 reads per second
  }
#endif

  // backlite button
#if defined(BACKLIGHT_BTN)
  if (millis() > next_backlight_btn_check) {
    const bool touch_state = digitalRead(PIN_BUTTON2);
#if defined(DISP_BACKLIGHT)
    digitalWrite(DISP_BACKLIGHT, !touch_state);
#elif defined(EXP_PIN_BACKLIGHT)
    expander.digitalWrite(EXP_PIN_BACKLIGHT, !touch_state);
#endif
    next_backlight_btn_check = millis() + 300;
  }
#endif

#if AUTO_OFF_MILLIS > 0
  if (millis() > _auto_off + 110000) {
    _display->turnOn();
    _auto_off = millis() + AUTO_OFF_MILLIS;
  }
#endif

  _quiet = quiet;
  _last_msg_count = last_msg_count;
  _total_request = total_request;
  _timestamp = time;
  _total_sent = total_sent;

  if (_display->isOn()) {
    if (millis() >= _next_refresh) {
      if (_display->isEink() && millis() >= _next_reset) {
        // periodic contrast restore: draw a splash frame first, so the following
        // content frame is a big change and repaints the whole panel (see above)
        _display->startFrame();
        _display->setTextSize(1);
        _display->drawTextCentered(_display->width() / 2, 28, "...");
        _display->endFrame();
        _next_reset = millis() + EINK_RESTORE_MILLIS;
      }
      _display->startFrame();
      renderCurrScreen();
      _display->endFrame();
      // like companion_radio/simple_repeater: don't redraw continuously. For e-ink
      // refresh rarely; endFrame() only pushes to the panel when the content (CRC) changed.
      _next_refresh = millis() + (_display->isEink() ? 10000 : 1000);
    }
#if AUTO_OFF_MILLIS > 0
    if (millis() > _auto_off) {
      _display->turnOff();
    }
#endif
  }

  // power off blue LED
#ifdef LED_BLUE
  if (millis() >= _led_reset) {
    digitalWrite(LED_BLUE, HIGH);
    _led_reset = millis() + 500;
  }
#endif

}