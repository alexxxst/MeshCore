#pragma once

#include <helpers/ui/DisplayDriver.h>

class UITask {
  DisplayDriver* _display;
  unsigned long _next_read = 0, _next_refresh = 0, _led_reset = 0, _gps_sync = 0, _auto_off = 0;
  int _prevBtnState{};
  char _info[32]{};
  char _time[32]{};
  char _gps[32]{};
  char _stats[32]{};
  char _node_name[32]{};
  bool _quiet = false;

  unsigned int next_backlight_btn_check = 0;
  unsigned long _timestamp = 0;
  unsigned long _last_msg_count = 0;
  unsigned long _total_request = 0;
  unsigned long _total_received = 0;
  unsigned long _total_sent = 0;

  void renderCurrScreen();
public:
  explicit UITask(DisplayDriver& display) : _display(&display) { _next_read = _next_refresh = 0; }
  void begin(const char* name, const char* group);
  void loop(bool quiet, unsigned long total_request, unsigned long total_sent, unsigned long time, unsigned long last_msg_count);
};