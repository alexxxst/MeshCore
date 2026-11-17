#include "RemoteRadioGuard.h"

#include <string.h>

namespace {

const uint8_t GUARD_MAGIC[4] = {'R', 'C', 'G', '1'};

uint32_t readLe32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

void writeLe32(uint8_t* p, uint32_t value) {
  p[0] = (uint8_t)(value & 0xFF);
  p[1] = (uint8_t)((value >> 8) & 0xFF);
  p[2] = (uint8_t)((value >> 16) & 0xFF);
  p[3] = (uint8_t)((value >> 24) & 0xFF);
}

uint32_t floatBits(float value) {
  uint32_t bits = 0;
  memcpy(&bits, &value, sizeof(bits));
  return bits;
}

float bitsFloat(uint32_t bits) {
  float value = 0.0f;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace

void RemoteRadioGuardState::clear() {
  armed = false;
  revert_pending = false;
  attempts = 0;
  freq = 0.0f;
  bw = 0.0f;
  sf = 0;
  cr = 0;
  pending_timestamp = 0;
  pending_seq = 0;
  target_freq = 0.0f;
  target_bw = 0.0f;
  target_sf = 0;
  target_cr = 0;
}

size_t RemoteRadioGuardState::serialize(uint8_t* out, size_t cap) const {
  if (out == nullptr || cap < SERIALIZED_SIZE) return 0;

  memset(out, 0, SERIALIZED_SIZE);
  memcpy(out, GUARD_MAGIC, sizeof(GUARD_MAGIC));
  out[4] = VERSION;
  out[5] = armed ? 1 : 0;
  out[6] = attempts;
  out[7] = sf;
  out[8] = cr;
  writeLe32(out + 9, floatBits(freq));
  writeLe32(out + 13, floatBits(bw));
  writeLe32(out + 17, pending_timestamp);
  writeLe32(out + 21, pending_seq);
  writeLe32(out + 25, floatBits(target_freq));
  writeLe32(out + 29, floatBits(target_bw));
  out[33] = target_sf;
  out[34] = target_cr;
  out[35] = revert_pending ? 1 : 0;
  return SERIALIZED_SIZE;
}

bool RemoteRadioGuardState::deserialize(const uint8_t* data, size_t len) {
  if (data == nullptr || len < SERIALIZED_SIZE) return false;
  if (memcmp(data, GUARD_MAGIC, sizeof(GUARD_MAGIC)) != 0) return false;
  if (data[4] != VERSION) return false;

  armed = data[5] != 0;
  attempts = data[6];
  sf = data[7];
  cr = data[8];
  freq = bitsFloat(readLe32(data + 9));
  bw = bitsFloat(readLe32(data + 13));
  pending_timestamp = readLe32(data + 17);
  pending_seq = readLe32(data + 21);
  target_freq = bitsFloat(readLe32(data + 25));
  target_bw = bitsFloat(readLe32(data + 29));
  target_sf = data[33];
  target_cr = data[34];
  revert_pending = data[35] != 0;

  if (freq < 150.0f || freq > 2500.0f || bw < 7.0f || bw > 500.0f ||
      sf < 5 || sf > 12 || cr < 5 || cr > 8) {
    return false;
  }
  if (target_freq < 150.0f || target_freq > 2500.0f ||
      target_bw < 7.0f || target_bw > 500.0f ||
      target_sf < 5 || target_sf > 12 || target_cr < 5 || target_cr > 8) {
    return false;
  }
  return true;
}
