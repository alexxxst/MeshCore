#pragma once

#include <stdint.h>

#define MAX_PATH_PREFIX_LEN   4
#define MAX_BLACKLIST_ENTRIES 30

struct BlacklistEntry {
  uint8_t len;                          // 0 = empty slot
  uint8_t prefix[MAX_PATH_PREFIX_LEN];
};
