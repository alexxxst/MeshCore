#pragma once

#include <stddef.h>
#include <stdint.h>

enum RemotePrefixMode {
  REMOTE_PREFIX_WORD,  // prefix must end at NUL or a space
  REMOTE_PREFIX_RAW,   // any command starting with the prefix
};

bool remoteCmdMatchesPrefix(const char* cmd, const char* prefix,
                            RemotePrefixMode mode);

// Normalizes in place: leading spaces, optional companion-style "xx|" prefix,
// and trailing spaces. The one exception is "set guest.password " with no
// argument, where exactly one trailing space is preserved to mean an empty
// password.
bool remoteCmdNormalize(char* cmd, size_t cap);

bool remoteCmdIsBanned(const char* cmd);
bool remoteCmdIsTimedRadio(const char* cmd);
bool remoteCmdIsTimedTx(const char* cmd);
bool remoteCmdIsRadioChanging(const char* cmd);

bool remoteCmdParseSetRadio(const char* cmd, float& freq, float& bw,
                            uint8_t& sf, uint8_t& cr);
bool remoteCmdParseSetTx(const char* cmd, int8_t& dbm);
bool remoteCmdParseTempRadio(const char* cmd, float& freq, float& bw,
                             uint8_t& sf, uint8_t& cr, int& timeout_mins);

bool remoteCmdSameBand(float old_mhz, float new_mhz, float bw_khz);
