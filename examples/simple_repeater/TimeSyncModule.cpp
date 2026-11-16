#include "TimeSyncModule.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <Mesh.h>
#include <Utils.h>

#include "TimeSyncDefaults.h"

#define TIMESYNC_MIN_PREFIX_BYTES   4    // same length the 'neighbors' command shows

// Appends to a CLI reply without ever writing past reply_sz. If the text does
// not fit, the reply is left truncated and NUL-terminated; later appends stop.
static void appendReply(char* reply, size_t reply_sz, size_t& used, const char* fmt, ...) {
  if (used >= reply_sz) return;
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(reply + used, reply_sz - used, fmt, args);
  va_end(args);
  if (n < 0) {
    reply[used] = 0;
    return;
  }
  if ((size_t)n >= reply_sz - used) {
    used = reply_sz;   // vsnprintf has already NUL-terminated the truncated text
    return;
  }
  used += (size_t)n;
}

static File openTimeSyncWrite(FILESYSTEM* fs) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  fs->remove(TIMESYNC_FILE);
  return fs->open(TIMESYNC_FILE, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  return fs->open(TIMESYNC_FILE, "w");
#else
  return fs->open(TIMESYNC_FILE, "w", true);
#endif
}

static int parseTimeSyncKeyArg(const char* s, uint8_t* out) {
  while (*s == ' ') s++;
  char tok[TIMESYNC_KEY_SIZE * 2 + 1];
  int n = 0;
  while (*s && *s != ' ') {
    if (n >= (int)sizeof(tok) - 1 || !mesh::Utils::isHexChar(*s)) return -1;
    tok[n++] = *s++;
  }
  tok[n] = 0;
  while (*s == ' ') s++;
  if (*s != 0 || n == 0 || (n & 1)) return -1;   // trailing junk, empty, or odd number of digits
  if (!mesh::Utils::fromHex(out, n / 2, tok)) return -1;
  return n / 2;
}

void TimeSyncModule::begin(FILESYSTEM* fs, const TimeSyncModuleContext& context) {
  _fs = fs;
  _context = context;
  _has_synced = false;
  _last_millis = 0;
  memset(_last_key, 0, sizeof(_last_key));
  load();
}

void TimeSyncModule::resetSyncStatus() {
  _sources.clear();
  _has_synced = false;
  _last_millis = 0;
  memset(_last_key, 0, sizeof(_last_key));
}

void TimeSyncModule::load() {
  _list.clear();
  _from_defaults = false;

  bool loaded = false;
  if (_fs->exists(TIMESYNC_FILE)) {
#if defined(RP2040_PLATFORM)
    File file = _fs->open(TIMESYNC_FILE, "r");
#else
    File file = _fs->open(TIMESYNC_FILE);
#endif
    if (file) {
      uint8_t buf[TimeSyncList::MAX_SERIALIZED];
      int len = file.read(buf, sizeof(buf));
      file.close();
      if (len == 0) {
        loaded = true;   // a zero-byte file is an explicit empty list: the file wins, sync stays off
      } else if (len > 0) {
        loaded = _list.deserialize(buf, len);
      }
    }
  }

  if (!loaded) {
    for (size_t i = 0; i < TIMESYNC_DEFAULT_KEY_COUNT; i++) {
      _list.add(TIMESYNC_DEFAULT_KEYS[i]);
    }
    _from_defaults = _list.count() > 0;
    MESH_DEBUG_PRINTLN("TimeSync: loaded %d build-time default key(s)", _list.count());
  }
}

bool TimeSyncModule::save() {
  File file = openTimeSyncWrite(_fs);
  if (!file) return false;

  uint8_t buf[TimeSyncList::MAX_SERIALIZED];
  size_t len = _list.serialize(buf);
  bool success = (file.write(buf, len) == len);
  file.close();
  return success;
}

bool TimeSyncModule::onAdvert(const mesh::Identity& id, const mesh::Packet* packet,
                              uint32_t advert_timestamp, bool is_share,
                              uint32_t airtime_ms, uint32_t now_ms, mesh::RTCClock& rtc) {
  if (_list.count() == 0 || !_list.contains(id.pub_key)) return false;
  if (advert_timestamp < TIMESYNC_MIN_EPOCH || advert_timestamp >= TIMESYNC_MAX_EPOCH) return false;
  if (!_sources.accept(id.pub_key, advert_timestamp, now_ms)) return false;

  uint8_t hops = packet->getPathHashCount();
  uint32_t target = TimeSyncList::computeTarget(advert_timestamp, airtime_ms, hops);

  bool strict_zero_hop = hops == 0 && !packet->isRouteFlood() && !is_share;
  uint32_t now = rtc.getCurrentTime();
  if (!TimeSyncList::shouldAdjust(now, target, strict_zero_hop)) return false;

  rtc.setCurrentTime(target);
  _has_synced = true;
  _last_millis = now_ms;
  memcpy(_last_key, id.pub_key, sizeof(_last_key));
  MESH_DEBUG_PRINTLN("TimeSync: clock %lu -> %lu (%s, hops=%u, airtime=%lums)",
                     (unsigned long)now, (unsigned long)target,
                     strict_zero_hop ? "zero-hop" : "forward-only", (unsigned)hops,
                     (unsigned long)airtime_ms);
  return true;
}

void TimeSyncModule::handleCommand(const char* args, char* reply, size_t reply_sz,
                                   uint32_t now_ms, bool serial) {
  if (reply_sz == 0) return;
  reply[0] = 0;
  size_t used = 0;
  while (*args == ' ') args++;

  if (*args == 0 || strcmp(args, "list") == 0) {
    if (_list.count() == 0) {
      appendReply(reply, reply_sz, used, "no trusted repeaters (time sync off)");
    } else {
      appendReply(reply, reply_sz, used, "trusted %d/%d:", _list.count(), TIMESYNC_MAX_TRUSTED);
      for (int i = 0; i < _list.count(); i++) {
        char hex[10];
        mesh::Utils::toHex(hex, _list.get(i), 4);
        appendReply(reply, reply_sz, used, " %s", hex);
      }
      if (_from_defaults) appendReply(reply, reply_sz, used, " | build defaults");
    }
    if (_has_synced) {
      char hex[10];
      mesh::Utils::toHex(hex, _last_key, 4);
      unsigned long secs_ago = (now_ms - _last_millis) / 1000;
      appendReply(reply, reply_sz, used, " | last sync %lus ago from %s", secs_ago, hex);
    }
    if (serial) {   // serial only: also dump the full keys (handy to copy to other repeaters)
      for (int i = 0; i < _list.count(); i++) {
        mesh::Utils::printHex(Serial, _list.get(i), TIMESYNC_KEY_SIZE);
        Serial.printf("\n");
      }
    }
  } else if (memcmp(args, "add ", 4) == 0) {
    uint8_t key[TIMESYNC_KEY_SIZE];
    int n = parseTimeSyncKeyArg(&args[4], key);
    if (n < 0) {
      strcpy(reply, "Err - bad pubkey (hex)");
    } else if (n < TIMESYNC_MIN_PREFIX_BYTES) {
      strcpy(reply, "Err - need at least 8 hex chars");
    } else {
      uint8_t full[TIMESYNC_KEY_SIZE];
      memset(full, 0, sizeof(full));
      bool resolved = false;
      if (n == TIMESYNC_KEY_SIZE) {
        memcpy(full, key, TIMESYNC_KEY_SIZE);
        resolved = true;
      } else {
        // a prefix is only accepted if exactly one neighbour matches it
        int matches = _context.resolve_neighbour
            ? _context.resolve_neighbour(_context.resolver_ctx, key, n, full)
            : 0;
        if (matches == 1) {
          resolved = true;
        } else if (matches > 1) {
          strcpy(reply, "Err - prefix is ambiguous");
        } else {
          strcpy(reply, "Err - no such neighbour, give the full 64-char pubkey");
        }
      }

      if (resolved) {
        if (_context.self_pub_key != nullptr
            && memcmp(full, _context.self_pub_key, TIMESYNC_KEY_SIZE) == 0) {
          strcpy(reply, "Err - that is this node");
        } else {
          TimeSyncList::AddResult res = _list.add(full);
          if (res == TimeSyncList::FULL) {
            snprintf(reply, reply_sz, "Err - list full (%d)", TIMESYNC_MAX_TRUSTED);
          } else if (res == TimeSyncList::ALREADY) {
            strcpy(reply, "OK - already trusted");
          } else if (save()) {
            _from_defaults = false;
            char hex[10];
            mesh::Utils::toHex(hex, full, 4);
            snprintf(reply, reply_sz, "OK - trusted %s (%d/%d)", hex, _list.count(), TIMESYNC_MAX_TRUSTED);
          } else {
            strcpy(reply, "Err - added, but could not be saved");
          }
        }
      }
    }
  } else if (memcmp(args, "remove ", 7) == 0) {
    uint8_t key[TIMESYNC_KEY_SIZE];
    int n = parseTimeSyncKeyArg(&args[7], key);
    if (n < 0) {
      strcpy(reply, "Err - bad pubkey (hex)");
    } else if (n < TIMESYNC_MIN_PREFIX_BYTES) {
      strcpy(reply, "Err - need at least 8 hex chars");
    } else {
      int removed = _list.removeMatching(key, n);
      if (removed == 0) {
        strcpy(reply, "Err - not in the list");
      } else {
        _sources.clear();
        if (save()) {
          _from_defaults = false;
          snprintf(reply, reply_sz, "OK - removed %d", removed);
        } else {
          strcpy(reply, "Err - removed, but could not be saved");
        }
      }
    }
  } else if (strcmp(args, "clear") == 0) {
    _list.clear();
    resetSyncStatus();
    if (save()) {
      _from_defaults = false;
      strcpy(reply, "OK - cleared (time sync off)");
    } else {
      strcpy(reply, "Err - cleared, but could not be saved");
    }
  } else if (strcmp(args, "defaults") == 0) {
    _list.clear();
    for (size_t i = 0; i < TIMESYNC_DEFAULT_KEY_COUNT; i++) {
      _list.add(TIMESYNC_DEFAULT_KEYS[i]);
    }
    int n = _list.count();
    bool removed = !_fs->exists(TIMESYNC_FILE) || _fs->remove(TIMESYNC_FILE);
    _from_defaults = n > 0;
    resetSyncStatus();
    if (!removed) {
      if (n == 0) {
        strcpy(reply, "Err - no defaults, but could not remove /timesync");
      } else {
        snprintf(reply, reply_sz, "Err - %d default key(s) loaded, but could not remove /timesync", n);
      }
    } else if (n == 0) {
      strcpy(reply, "OK - no build-time defaults (time sync off)");
    } else {
      snprintf(reply, reply_sz, "OK - %d default key(s), file removed", n);
    }
  } else {
    strcpy(reply, "Err - timesync [list|add <pubkey>|remove <pubkey>|defaults|clear]");
  }
}
