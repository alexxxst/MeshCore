#pragma once

#include <stdio.h>
#include <string.h>

// Room kept for the "\n+N more (list M)" suffix, so entries never push it out.
#define LIST_REPLY_SUFFIX_RESERVE  24

// Pages a list into a fixed-size CLI reply buffer: entries from 'offset' are
// printed while they fit, then a "+N more (list M)" line names the offset of
// the next page. Used by commands like 'blacklist path list [offset]'.
class ListReplyPager {
  char* _reply;
  char* _dp;
  int _cap;       // usable characters, excluding the terminating NUL
  int _offset, _index, _shown, _more;

public:
  ListReplyPager(char* reply, size_t reply_sz, int offset = 0)
    : _reply(reply), _dp(reply), _cap(reply_sz > 0 ? (int)reply_sz - 1 : 0),
      _offset(offset < 0 ? 0 : offset), _index(0), _shown(0), _more(0) {
    if (reply_sz > 0) *reply = 0;
  }

  void add(const char* text, size_t len) {
    if (_index++ < _offset) return;
    int start = (int)(_dp - _reply) + (_shown > 0 ? 1 : 0);
    if (_more > 0 || start + (int)len + LIST_REPLY_SUFFIX_RESERVE > _cap) {
      _more++;
      return;
    }
    if (_shown > 0) *_dp++ = '\n';
    memcpy(_dp, text, len);
    _dp += len;
    *_dp = 0;
    _shown++;
  }

  void finish() {
    if (_cap <= 0) return;
    if (_shown == 0 && _more == 0) {
      if (_cap >= 5) strcpy(_reply, "-none-");
    } else if (_more > 0) {
      char suffix[32];
      int n = snprintf(suffix, sizeof(suffix), "%s+%d more (list %d)",
                       _shown > 0 ? "\n" : "", _more, _offset + _shown);
      if ((int)(_dp - _reply) + n <= _cap) memcpy(_dp, suffix, n + 1);
    }
  }
};
