#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Persisted filter settings live in /filter_prefs so that they are owned by the
// filter module rather than by NodePrefs. The format is versioned. A newer
// writer may append bytes after the declared payload and an older reader will
// ignore them; changing the meaning of existing payload bytes requires a new
// FILE_VERSION.
#define MAX_ADVERT_JAIL_HOURS   168

struct FilterPrefs {
  static constexpr uint8_t FILE_VERSION = 1;
  static constexpr size_t HEADER_SIZE = 6;       // magic[4] + version + payload_len
  static constexpr size_t PAYLOAD_SIZE = 7;      // jail hours + reserved bytes
  static constexpr size_t MAX_SERIALIZED = HEADER_SIZE + PAYLOAD_SIZE;
  static constexpr size_t MAX_READ = MAX_SERIALIZED + 16;   // room for appended fields

  uint8_t advert_jail_hours = 0;   // 0 = jail off

  void clear() {
    advert_jail_hours = 0;
  }

  size_t serialize(uint8_t* out) const {
    out[0] = 'F'; out[1] = 'L'; out[2] = 'T'; out[3] = '1';
    out[4] = FILE_VERSION;
    out[5] = (uint8_t)PAYLOAD_SIZE;
    out[6] = advert_jail_hours > MAX_ADVERT_JAIL_HOURS ? MAX_ADVERT_JAIL_HOURS
                                                       : advert_jail_hours;
    memset(&out[7], 0, PAYLOAD_SIZE - 1);
    return MAX_SERIALIZED;
  }

  // Replaces the values; leaves the defaults in place and returns false when
  // the buffer is not a valid current-version file.
  bool deserialize(const uint8_t* in, size_t len) {
    clear();
    if (in == nullptr || len < HEADER_SIZE + 1) return false;
    if (in[0] != 'F' || in[1] != 'L' || in[2] != 'T' || in[3] != '1') return false;
    if (in[4] != FILE_VERSION) return false;

    const size_t payload_len = in[5];
    if (payload_len < 1 || HEADER_SIZE + payload_len > len) return false;

    advert_jail_hours = in[6];
    if (advert_jail_hours > MAX_ADVERT_JAIL_HOURS) {
      advert_jail_hours = MAX_ADVERT_JAIL_HOURS;
    }
    return true;
  }
};
