#pragma once

#include <stddef.h>
#include <stdint.h>

// Contents of the F1 file. It stores the radio parameters that were active
// before a timed "set radio" and enough information to resolve the matching
// F2 pending entry after a reboot.
struct RemoteRadioGuardState {
  static constexpr uint8_t VERSION = 1;
  static constexpr size_t SERIALIZED_SIZE = 36;

  bool armed = false;
  bool revert_pending = false;
  uint8_t attempts = 0;
  float freq = 0.0f;
  float bw = 0.0f;
  uint8_t sf = 0;
  uint8_t cr = 0;
  uint32_t pending_timestamp = 0;
  uint32_t pending_seq = 0;
  float target_freq = 0.0f;
  float target_bw = 0.0f;
  uint8_t target_sf = 0;
  uint8_t target_cr = 0;

  void clear();
  size_t serialize(uint8_t* out, size_t cap) const;
  bool deserialize(const uint8_t* data, size_t len);
};
