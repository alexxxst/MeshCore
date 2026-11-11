#pragma once

// Clock synchronisation from adverts of explicitly trusted repeaters.
//
// This header is deliberately free of Arduino / MeshCore dependencies so the
// list handling and the adjust rule can be unit tested on a host machine.

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifndef TIMESYNC_MAX_TRUSTED
  #define TIMESYNC_MAX_TRUSTED     8     // max number of trusted time sources
#endif

#define TIMESYNC_MIN_EPOCH    1793491200UL   // 2026-11-01 00:00 UTC, fixed
#define TIMESYNC_MAX_EPOCH    1888185600UL   // 2029-11-01 00:00 UTC, raise per release
#define TIMESYNC_MIN_STEP_SECS 10
#define TIMESYNC_MAX_COMP_HOPS 16
#define TIMESYNC_MAX_SOURCES   8
#define TIMESYNC_SOURCE_KEY_BYTES 8

#define TIMESYNC_KEY_SIZE          32    // Ed25519 public key (same as PUB_KEY_SIZE)
#define TIMESYNC_FILE              "/timesync"

class TimeSyncList {
public:
  // file layout: { magic, version, count, count * key[32] }
  static const uint8_t FILE_MAGIC   = 0x54;   // 'T'
  static const uint8_t FILE_VERSION = 1;
  static const size_t  HEADER_SIZE  = 3;
  static const size_t  MAX_SERIALIZED = HEADER_SIZE + TIMESYNC_MAX_TRUSTED * TIMESYNC_KEY_SIZE;

  enum AddResult { ADDED, ALREADY, FULL };

  TimeSyncList() { clear(); }

  int count() const { return n; }

  // full public key at idx, or NULL if out of range
  const uint8_t* get(int idx) const { return (idx >= 0 && idx < n) ? keys[idx] : NULL; }

  bool contains(const uint8_t* pub_key) const {
    for (int i = 0; i < n; i++) {
      if (memcmp(keys[i], pub_key, TIMESYNC_KEY_SIZE) == 0) return true;
    }
    return false;
  }

  AddResult add(const uint8_t* pub_key) {
    if (contains(pub_key)) return ALREADY;
    if (n >= TIMESYNC_MAX_TRUSTED) return FULL;
    memcpy(keys[n++], pub_key, TIMESYNC_KEY_SIZE);
    return ADDED;
  }

  // Parses a comma-separated list of full 32-byte keys (64 hex characters
  // each). Spaces around entries and empty entries are ignored. Malformed
  // entries and entries beyond capacity increment *invalid (when supplied);
  // duplicates are dropped silently. The list is replaced even when no entry
  // is valid.
  int setFromCsv(const char* csv, int* invalid = nullptr) {
    TimeSyncList tmp;
    if (invalid) *invalid = 0;
    if (csv) {
      const char* entry = csv;
      while (true) {
        const char* comma = strchr(entry, ',');
        const char* end = comma ? comma : entry + strlen(entry);
        while (entry < end && (*entry == ' ' || *entry == '\t')) entry++;
        while (end > entry && (end[-1] == ' ' || end[-1] == '\t')) end--;
        if (end > entry) {
          uint8_t key[TIMESYNC_KEY_SIZE];
          if (parseHexKey(entry, (size_t)(end - entry), key)) {
            if (tmp.add(key) == FULL && invalid) (*invalid)++;
          } else if (invalid) {
            (*invalid)++;
          }
        }
        if (!comma) break;
        entry = comma + 1;
      }
    }
    *this = tmp;
    return tmp.count();
  }

  // number of trusted keys that start with the given prefix
  int countMatching(const uint8_t* prefix, size_t len) const {
    if (len == 0 || len > TIMESYNC_KEY_SIZE) return 0;
    int c = 0;
    for (int i = 0; i < n; i++) {
      if (memcmp(keys[i], prefix, len) == 0) c++;
    }
    return c;
  }

  // remove every trusted key that starts with the given prefix, returns how many were removed
  int removeMatching(const uint8_t* prefix, size_t len) {
    if (len == 0 || len > TIMESYNC_KEY_SIZE) return 0;
    int removed = 0;
    int dst = 0;
    for (int src = 0; src < n; src++) {
      if (memcmp(keys[src], prefix, len) == 0) {
        removed++;
      } else {
        if (dst != src) memcpy(keys[dst], keys[src], TIMESYNC_KEY_SIZE);
        dst++;
      }
    }
    for (int i = dst; i < n; i++) memset(keys[i], 0, TIMESYNC_KEY_SIZE);
    n = dst;
    return removed;
  }

  void clear() {
    n = 0;
    memset(keys, 0, sizeof(keys));
  }

  // Corrects the advert timestamp for the deterministic LoRa airtime of the
  // source transmission and every hop, capped so a bogus hop count cannot
  // overflow the calculation.
  static uint32_t computeTarget(uint32_t timestamp, uint32_t airtime_ms, uint8_t hops) {
    if (hops > TIMESYNC_MAX_COMP_HOPS) hops = TIMESYNC_MAX_COMP_HOPS;
    uint32_t delay_ms = airtime_ms * (uint32_t)(1 + hops);
    return timestamp + (delay_ms + 500) / 1000;
  }

  // Forward correction is always allowed. Backward correction is only allowed
  // when allow_backward is set; otherwise a flood/relayed/Share advert can
  // never move the clock back. Changes inside the deadband are ignored.
  static bool shouldAdjust(uint32_t now, uint32_t target, bool allow_backward,
                           uint32_t min_step = TIMESYNC_MIN_STEP_SECS) {
    if (target > now) return (target - now) > min_step;
    if (target < now) return allow_backward && (now - target) > min_step;
    return false;
  }

  size_t serialize(uint8_t* out) const {   // 'out' must hold MAX_SERIALIZED bytes
    out[0] = FILE_MAGIC;
    out[1] = FILE_VERSION;
    out[2] = (uint8_t)n;
    size_t len = HEADER_SIZE;
    for (int i = 0; i < n; i++) {
      memcpy(&out[len], keys[i], TIMESYNC_KEY_SIZE);
      len += TIMESYNC_KEY_SIZE;
    }
    return len;
  }

  // replaces the list; leaves it empty and returns false if the data is malformed
  bool deserialize(const uint8_t* in, size_t len) {
    clear();
    if (len < HEADER_SIZE || in[0] != FILE_MAGIC || in[1] != FILE_VERSION) return false;
    size_t count = in[2];
    if (count > TIMESYNC_MAX_TRUSTED || len < HEADER_SIZE + count * TIMESYNC_KEY_SIZE) return false;
    for (size_t i = 0; i < count; i++) {
      add(&in[HEADER_SIZE + i * TIMESYNC_KEY_SIZE]);   // add() also drops duplicates
    }
    return true;
  }

private:
  static int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  }

  static bool parseHexKey(const char* s, size_t len, uint8_t* out) {
    if (len != TIMESYNC_KEY_SIZE * 2) return false;
    for (size_t i = 0; i < TIMESYNC_KEY_SIZE; i++) {
      int hi = hexValue(s[2 * i]);
      int lo = hexValue(s[2 * i + 1]);
      if (hi < 0 || lo < 0) return false;
      out[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
  }

  int n;
  uint8_t keys[TIMESYNC_MAX_TRUSTED][TIMESYNC_KEY_SIZE];
};

// RAM-only replay cache for trusted time sources. Each slot stores the first
// 8 bytes of the public key, the last accepted raw timestamp and the time the
// source was last heard, so an old or repeated advert cannot win twice.
class TimeSyncSources {
public:
  TimeSyncSources() { clear(); }

  void clear() {
    for (int i = 0; i < TIMESYNC_MAX_SOURCES; i++) {
      memset(_sources[i].key, 0, TIMESYNC_SOURCE_KEY_BYTES);
      _sources[i].last_ts = 0;
      _sources[i].heard_ms = 0;
    }
  }

  bool accept(const uint8_t* pub_key, uint32_t timestamp, uint32_t now_ms) {
    int slot = -1;
    for (int i = 0; i < TIMESYNC_MAX_SOURCES; i++) {
      if (_sources[i].last_ts != 0 &&
          memcmp(_sources[i].key, pub_key, TIMESYNC_SOURCE_KEY_BYTES) == 0) {
        slot = i;
        break;
      }
    }

    if (slot >= 0) {
      if (timestamp <= _sources[slot].last_ts) return false;
      _sources[slot].last_ts = timestamp;
      _sources[slot].heard_ms = now_ms;
      return true;
    }

    for (int i = 0; i < TIMESYNC_MAX_SOURCES; i++) {
      if (_sources[i].last_ts == 0) {
        slot = i;
        break;
      }
    }

    if (slot < 0) {
      slot = 0;
      for (int i = 1; i < TIMESYNC_MAX_SOURCES; i++) {
        if ((int32_t)(_sources[i].heard_ms - _sources[slot].heard_ms) < 0) slot = i;
      }
    }

    memcpy(_sources[slot].key, pub_key, TIMESYNC_SOURCE_KEY_BYTES);
    _sources[slot].last_ts = timestamp;
    _sources[slot].heard_ms = now_ms;
    return true;
  }

private:
  struct Source {
    uint8_t key[TIMESYNC_SOURCE_KEY_BYTES];
    uint32_t last_ts;
    uint32_t heard_ms;
  };

  Source _sources[TIMESYNC_MAX_SOURCES];
};
