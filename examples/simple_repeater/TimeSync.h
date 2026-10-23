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

#ifndef TIMESYNC_MIN_STEP_SECS
  #define TIMESYNC_MIN_STEP_SECS   2     // ignore adverts that are less than this far ahead of our clock
#endif

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

  // The clock is only ever moved FORWARD (same rule as the 'time' / 'clock sync' CLI commands),
  // and only when the advert is at least TIMESYNC_MIN_STEP_SECS ahead of us. The advert's timestamp is
  // used as-is (no +1), so two repeaters that trust each other can't push each other's clock ahead.
  static bool shouldAdjust(uint32_t now, uint32_t advert_timestamp) {
    return advert_timestamp > now && (advert_timestamp - now) >= TIMESYNC_MIN_STEP_SECS;
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
  int n;
  uint8_t keys[TIMESYNC_MAX_TRUSTED][TIMESYNC_KEY_SIZE];
};
