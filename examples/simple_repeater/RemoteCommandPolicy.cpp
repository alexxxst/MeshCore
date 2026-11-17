#include "RemoteCommandPolicy.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

struct PrefixRule {
  const char* text;
  RemotePrefixMode mode;
};

// All banned entries are RAW on purpose. CommonCLI itself uses memcmp()
// prefixes for poweroff, shutdown, advert, start ota, clear stats, gps setloc,
// gps advert, log and get prv.key, so "poweroffblabla" would otherwise reach
// the handler. advert.zerohop is whitelisted before this table.
const PrefixRule BANNED[] = {
  {"poweroff", REMOTE_PREFIX_RAW},
  {"shutdown", REMOTE_PREFIX_RAW},
  {"advert", REMOTE_PREFIX_RAW},
  {"start ota", REMOTE_PREFIX_RAW},
  {"erase", REMOTE_PREFIX_RAW},
  {"clear stats", REMOTE_PREFIX_RAW},
  {"set name", REMOTE_PREFIX_RAW},
  {"set lat", REMOTE_PREFIX_RAW},
  {"set lon", REMOTE_PREFIX_RAW},
  {"set prv.key", REMOTE_PREFIX_RAW},
  {"get prv.key", REMOTE_PREFIX_RAW},
  {"password", REMOTE_PREFIX_RAW},
  {"set owner.info", REMOTE_PREFIX_RAW},
  {"set repeat", REMOTE_PREFIX_RAW},
  {"setperm", REMOTE_PREFIX_RAW},
  {"gps setloc", REMOTE_PREFIX_RAW},
  {"gps advert", REMOTE_PREFIX_RAW},
  {"sensor set", REMOTE_PREFIX_RAW},
  {"set bridge", REMOTE_PREFIX_RAW},
  {"set freq", REMOTE_PREFIX_RAW},
  {"set reboot.interval", REMOTE_PREFIX_RAW},
  {"region load", REMOTE_PREFIX_RAW},
  {"log", REMOTE_PREFIX_RAW},
};

struct RadioBand {
  float min_mhz;
  float max_mhz;
  const char* name;
};

const RadioBand RADIO_BANDS[] = {
  {430.0f, 450.0f, "433"},
  {863.0f, 870.0f, "868"},
  {902.0f, 928.0f, "915"},
  {2400.0f, 2500.0f, "2.4G"},
};

const RadioBand* findBand(float mhz) {
  for (const auto& band : RADIO_BANDS) {
    if (mhz >= band.min_mhz && mhz <= band.max_mhz) return &band;
  }
  return nullptr;
}

void stripLeadingSpaces(char* cmd) {
  char* start = cmd;
  while (*start == ' ') ++start;
  if (start != cmd) memmove(cmd, start, strlen(start) + 1);
}

void stripTrailingSpaces(char* cmd) {
  size_t len = strlen(cmd);
  while (len > 0 && cmd[len - 1] == ' ') cmd[--len] = 0;
}

bool onlySpaces(const char* p) {
  while (*p) {
    if (*p != ' ') return false;
    ++p;
  }
  return true;
}

bool parseFloatPart(const char* part, float& out) {
  char* end = nullptr;
  out = strtof(part, &end);
  if (end == part) return false;
  while (*end == ' ') ++end;
  return *end == 0;
}

bool parseIntPart(const char* part, long& out) {
  char* end = nullptr;
  out = strtol(part, &end, 10);
  if (end == part) return false;
  while (*end == ' ') ++end;
  return *end == 0;
}

bool validRadioParams(float freq, float bw, uint8_t sf, uint8_t cr) {
  return freq >= 150.0f && freq <= 2500.0f &&
         bw >= 7.0f && bw <= 500.0f &&
         sf >= 5 && sf <= 12 &&
         cr >= 5 && cr <= 8;
}

bool parseRadioBody(const char* args, float& freq, float& bw,
                    uint8_t& sf, uint8_t& cr, int* timeout_mins) {
  char buf[160];
  strncpy(buf, args, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;

  char* parts[5];
  int wanted = timeout_mins ? 5 : 4;
  // splitArgs() requires exactly max_parts; use the right array size.
  char* save = nullptr;
  int n = 0;
  for (char* p = strtok_r(buf, ",", &save); p != nullptr;
       p = strtok_r(nullptr, ",", &save)) {
    if (n >= wanted) return false;
    parts[n++] = p;
  }
  if (n != wanted) return false;

  float f = 0, b = 0;
  long s = 0, c = 0;
  if (!parseFloatPart(parts[0], f)) return false;
  if (!parseFloatPart(parts[1], b)) return false;
  if (!parseIntPart(parts[2], s)) return false;
  if (!parseIntPart(parts[3], c)) return false;
  if (s < 5 || s > 12 || c < 5 || c > 8) return false;
  if (!validRadioParams(f, b, (uint8_t)s, (uint8_t)c)) return false;

  if (timeout_mins) {
    long t = 0;
    if (!parseIntPart(parts[4], t) || t <= 0 || t > 1440) return false;
    *timeout_mins = (int)t;
  }

  freq = f;
  bw = b;
  sf = (uint8_t)s;
  cr = (uint8_t)c;
  return true;
}

}  // namespace

bool remoteCmdMatchesPrefix(const char* cmd, const char* prefix,
                            RemotePrefixMode mode) {
  if (cmd == nullptr || prefix == nullptr) return false;
  size_t len = strlen(prefix);
  if (strncmp(cmd, prefix, len) != 0) return false;
  if (mode == REMOTE_PREFIX_RAW) return true;
  return cmd[len] == 0 || cmd[len] == ' ';
}

bool remoteCmdNormalize(char* cmd, size_t cap) {
  if (cmd == nullptr || cap == 0) return false;
  if (strnlen(cmd, cap) == cap) return false;   // missing NUL

  stripLeadingSpaces(cmd);
  // MyMesh::handleCommand() strips one more prefix before dispatching. Strip
  // every prefix here with the same condition, otherwise "aa|bb|poweroff"
  // would reach powerOff() after the second strip.
  while (strlen(cmd) > 4 && cmd[2] == '|') {
    memmove(cmd, cmd + 3, strlen(cmd + 3) + 1);
    stripLeadingSpaces(cmd);
  }

  static const char GUEST_PREFIX[] = "set guest.password";
  if (strncmp(cmd, GUEST_PREFIX, sizeof(GUEST_PREFIX) - 1) == 0) {
    char* arg = cmd + sizeof(GUEST_PREFIX) - 1;
    if (*arg == ' ' && onlySpaces(arg)) {
      // Exactly one space so CommonCLI sees "guest.password " + empty value.
      *arg++ = ' ';
      *arg = 0;
      return true;
    }
  }

  stripTrailingSpaces(cmd);
  return true;
}

bool remoteCmdIsBanned(const char* cmd) {
  if (cmd == nullptr) return false;
  if (remoteCmdMatchesPrefix(cmd, "advert.zerohop", REMOTE_PREFIX_WORD)) {
    return false;
  }
  for (const auto& rule : BANNED) {
    if (remoteCmdMatchesPrefix(cmd, rule.text, rule.mode)) return true;
  }
  return false;
}

bool remoteCmdIsTimedRadio(const char* cmd) {
  return remoteCmdMatchesPrefix(cmd, "set radio", REMOTE_PREFIX_WORD);
}

bool remoteCmdIsTimedTx(const char* cmd) {
  return remoteCmdMatchesPrefix(cmd, "set tx", REMOTE_PREFIX_WORD);
}

bool remoteCmdIsRadioChanging(const char* cmd) {
  return remoteCmdMatchesPrefix(cmd, "set radio", REMOTE_PREFIX_RAW) ||
         remoteCmdMatchesPrefix(cmd, "tempradio", REMOTE_PREFIX_RAW) ||
         remoteCmdMatchesPrefix(cmd, "set tx", REMOTE_PREFIX_RAW) ||
         remoteCmdMatchesPrefix(cmd, "set freq", REMOTE_PREFIX_RAW);
}

bool remoteCmdParseSetRadio(const char* cmd, float& freq, float& bw,
                            uint8_t& sf, uint8_t& cr) {
  if (!remoteCmdMatchesPrefix(cmd, "set radio", REMOTE_PREFIX_WORD)) return false;
  const char* args = cmd + strlen("set radio");
  while (*args == ' ') ++args;
  return parseRadioBody(args, freq, bw, sf, cr, nullptr);
}

bool remoteCmdParseSetTx(const char* cmd, int8_t& dbm) {
  if (!remoteCmdMatchesPrefix(cmd, "set tx", REMOTE_PREFIX_WORD)) return false;
  const char* args = cmd + strlen("set tx");
  while (*args == ' ') ++args;

  char* end = nullptr;
  long value = strtol(args, &end, 10);
  if (end == args) return false;
  while (*end == ' ') ++end;
  if (*end != 0) return false;
  if (value < -9 || value > 22) return false;
  dbm = (int8_t)value;
  return true;
}

bool remoteCmdParseTempRadio(const char* cmd, float& freq, float& bw,
                             uint8_t& sf, uint8_t& cr, int& timeout_mins) {
  if (!remoteCmdMatchesPrefix(cmd, "tempradio", REMOTE_PREFIX_WORD)) return false;
  const char* args = cmd + strlen("tempradio");
  while (*args == ' ') ++args;
  int timeout = 0;
  if (!parseRadioBody(args, freq, bw, sf, cr, &timeout)) return false;
  timeout_mins = timeout;
  return true;
}

bool remoteCmdSameBand(float old_mhz, float new_mhz, float bw_khz) {
  const RadioBand* old_band = findBand(old_mhz);
  const RadioBand* new_band = findBand(new_mhz);
  if (old_band == nullptr || new_band == nullptr || old_band != new_band) return false;

  float half_bw = bw_khz / 2000.0f;
  return new_mhz - half_bw >= new_band->min_mhz &&
         new_mhz + half_bw <= new_band->max_mhz;
}
