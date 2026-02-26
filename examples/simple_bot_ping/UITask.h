#pragma once

#include <helpers/ui/DisplayDriver.h>

class UITask {
  DisplayDriver* _display;
  unsigned long _next_read, _next_refresh, _auto_off;
  int _prevBtnState;
  char _info[32];
  char _stats[32];
  char _node_name[32];
  bool _quiet = false;

  unsigned long _last_msg_count = 0;
  unsigned long _total_request = 0;
  unsigned long _total_received = 0;
  unsigned long _total_sent = 0;

  void renderCurrScreen();
public:
  UITask(DisplayDriver& display) : _display(&display) { _next_read = _next_refresh = 0; }
  void begin(const char* name, const char* group);
  void loop(bool quiet, unsigned long total_request, unsigned long total_sent, unsigned long total_received, unsigned long last_msg_count);
};