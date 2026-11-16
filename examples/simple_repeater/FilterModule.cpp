#include "FilterModule.h"
#include "ListReplyPager.h"

#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <MeshCore.h>
#include <Packet.h>
#include <Utils.h>
#include <helpers/TxtDataHelpers.h>

#define PATH_BLACKLIST_FILE   "/path_bl"
#define CHAN_BLACKLIST_FILE   "/chan_bl"
#define FILTER_PREFS_FILE     "/filter_prefs"

static bool isHexStr(const char* s, int len) {
  for (int i = 0; i < len; i++) {
    if (!mesh::Utils::isHexChar(s[i])) return false;
  }
  return true;
}

void FilterModule::begin(FILESYSTEM* fs) {
  _fs = fs;

  memset(_path_blacklist, 0, sizeof(_path_blacklist));
  memset(_chan_blacklist, 0, sizeof(_chan_blacklist));
  memset(_chan_name_filters, 0, sizeof(_chan_name_filters));
  _num_chan_name_filters = 0;
  memset(_advert_jail, 0, sizeof(_advert_jail));
  _drops_path = _drops_chan = _drops_jail = 0;

  _prefs.clear();
  loadPrefs();

  loadBlacklist(PATH_BLACKLIST_FILE, _path_blacklist);
  loadChanBlacklist(CHAN_BLACKLIST_FILE);
}

/* ------------------------- packet vetoes ---------------------------------- */

// Returns true if any path entry in the packet matches a path-prefix blacklist entry.
// Comparison uses min(entry.len, hash_size) bytes, so shorter entries match any hash size.
bool FilterModule::isPathBlacklisted(const mesh::Packet* packet) const {
  uint8_t hash_size  = packet->getPathHashSize();
  uint8_t hash_count = packet->getPathHashCount();
  const uint8_t* ptr = packet->path;
  for (uint8_t h = 0; h < hash_count; h++, ptr += hash_size) {
    for (int b = 0; b < MAX_BLACKLIST_ENTRIES; b++) {
      if (_path_blacklist[b].len == 0) continue;
      uint8_t cmp_len = (_path_blacklist[b].len < hash_size) ? _path_blacklist[b].len : hash_size;
      if (memcmp(ptr, _path_blacklist[b].prefix, cmp_len) == 0) return true;
    }
  }
  return false;
}

// Returns true when a GRP_TXT/GRP_DATA packet matches a channel-hash blacklist
// entry (hex prefix or #channel_name with decrypt verification).
bool FilterModule::isChanBlacklisted(const mesh::Packet* packet) const {
  return isChanFiltered(packet, _chan_blacklist, _chan_name_filters, _num_chan_name_filters);
}

bool FilterModule::vetoRecvFlood(const mesh::Packet* packet) {
  // Drop packets whose path contains a blacklisted node prefix
  if (isPathBlacklisted(packet)) {
    _drops_path++;
    MESH_DEBUG_PRINTLN("filterRecvFloodPacket: path prefix blacklisted, dropping!");
    return true;
  }

  // Drop group-channel packets whose channel hash is blacklisted
  if (isChanBlacklisted(packet)) {
    _drops_chan++;
    MESH_DEBUG_PRINTLN("filterRecvFloodPacket: channel hash blacklisted, dropping!");
    return true;
  }
  return false;
}

bool FilterModule::vetoForwardDirect(const mesh::Packet* packet) {
  // A direct-routed group packet never reaches the receive hook, so stop it
  // here when this node would relay it.
  if (!isChanBlacklisted(packet)) return false;
  _drops_chan++;
  MESH_DEBUG_PRINTLN("allowPacketForward: channel hash blacklisted, dropping direct!");
  return true;
}

bool FilterModule::vetoForwardAdvert(const mesh::Packet* packet, uint32_t now_ms) {
  if (_prefs.advert_jail_hours == 0 || !packet->isRouteFlood()
      || packet->getPayloadType() != PAYLOAD_TYPE_ADVERT
      || packet->payload_len < PUB_KEY_SIZE + 4) {
    return false;
  }

  uint32_t advert_time;
  memcpy(&advert_time, &packet->payload[PUB_KEY_SIZE], 4);
  if (mesh::checkAdvertJail(_advert_jail, MAX_ADVERT_JAIL_ENTRIES, packet->payload, advert_time,
                            now_ms,
                            (uint32_t)_prefs.advert_jail_hours * 3600UL * 1000UL)) {
    _drops_jail++;
    MESH_DEBUG_PRINTLN("allowPacketForward: advert jail, dropping!");
    return true;
  }
  return false;
}

/* ----------------------- blacklist helpers -------------------------------- */

bool FilterModule::addToBlacklist(BlacklistEntry* list, const uint8_t* prefix, uint8_t len) {
  if (len == 0 || len > MAX_PATH_PREFIX_LEN) return false;
  // if already present, consider success
  for (int i = 0; i < MAX_BLACKLIST_ENTRIES; i++) {
    if (list[i].len == len && memcmp(list[i].prefix, prefix, len) == 0) return true;
  }
  // find an empty slot
  for (int i = 0; i < MAX_BLACKLIST_ENTRIES; i++) {
    if (list[i].len == 0) {
      list[i].len = len;
      memcpy(list[i].prefix, prefix, len);
      return true;
    }
  }
  return false; // list is full
}

bool FilterModule::removeFromBlacklist(BlacklistEntry* list, const uint8_t* prefix, uint8_t len) {
  for (int i = 0; i < MAX_BLACKLIST_ENTRIES; i++) {
    if (list[i].len == len && memcmp(list[i].prefix, prefix, len) == 0) {
      list[i].len = 0; // mark slot as empty
      return true;
    }
  }
  return false;
}

void FilterModule::formatBlacklist(const BlacklistEntry* list, char* reply, size_t reply_sz,
                                   int offset) const {
  ListReplyPager pager(reply, reply_sz, offset);
  for (int i = 0; i < MAX_BLACKLIST_ENTRIES; i++) {
    if (list[i].len == 0) continue;
    char hex[MAX_PATH_PREFIX_LEN * 2 + 1];
    mesh::Utils::toHex(hex, list[i].prefix, list[i].len);
    pager.add(hex, list[i].len * 2);
  }
  pager.finish();
}

void FilterModule::loadBlacklist(const char* fname, BlacklistEntry* list) {
  memset(list, 0, sizeof(BlacklistEntry) * MAX_BLACKLIST_ENTRIES);
#if defined(RP2040_PLATFORM)
  File f = _fs->open(fname, "r");
#else
  File f = _fs->open(fname);
#endif
  if (!f) return;
  int idx = 0;
  char line[MAX_PATH_PREFIX_LEN * 2 + 4];
  int line_len = 0;
  while (f.available() && idx < MAX_BLACKLIST_ENTRIES) {
    int c = f.read();
    if (c < 0) break;
    if (c == '\n' || c == '\r') {
      if (line_len >= 2 && (line_len % 2 == 0) && line[0] != '#') {
        line[line_len] = 0;
        uint8_t prefix[MAX_PATH_PREFIX_LEN];
        int byte_len = line_len / 2;
        if (byte_len <= MAX_PATH_PREFIX_LEN && isHexStr(line, line_len)
            && mesh::Utils::fromHex(prefix, byte_len, line)) {
          list[idx].len = (uint8_t)byte_len;
          memcpy(list[idx].prefix, prefix, byte_len);
          idx++;
        }
      }
      line_len = 0;
    } else if (line_len < (int)(sizeof(line) - 1)) {
      line[line_len++] = (char)c;
    }
  }
  // handle last line with no trailing newline
  if (line_len >= 2 && (line_len % 2 == 0) && line[0] != '#' && idx < MAX_BLACKLIST_ENTRIES) {
    line[line_len] = 0;
    uint8_t prefix[MAX_PATH_PREFIX_LEN];
    int byte_len = line_len / 2;
    if (byte_len <= MAX_PATH_PREFIX_LEN && isHexStr(line, line_len)
        && mesh::Utils::fromHex(prefix, byte_len, line)) {
      list[idx].len = (uint8_t)byte_len;
      memcpy(list[idx].prefix, prefix, byte_len);
    }
  }
  f.close();
}

void FilterModule::saveBlacklist(const char* fname, const BlacklistEntry* list) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  _fs->remove(fname);
  File f = _fs->open(fname, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  File f = _fs->open(fname, "w");
#else
  File f = _fs->open(fname, "w", true);
#endif
  if (!f) return;
  for (int i = 0; i < MAX_BLACKLIST_ENTRIES; i++) {
    if (list[i].len == 0) continue;
    char hex[MAX_PATH_PREFIX_LEN * 2 + 2];
    mesh::Utils::toHex(hex, list[i].prefix, list[i].len);
    f.println(hex);
  }
  f.close();
}

/* ------------------- channel name filter helpers -------------------------- */

void FilterModule::deriveChanNameFilter(ChanNameFilter& entry, const char* name) {
  // Derive channel secret: first 16 bytes of sha256(name), zero-padded to PUB_KEY_SIZE
  // to match GroupChannel.secret layout (MACThenDecrypt reads PUB_KEY_SIZE bytes)
  uint8_t full_hash[32];
  mesh::Utils::sha256(full_hash, 32, (const uint8_t*)name, strlen(name));
  memset(entry.secret, 0, sizeof(entry.secret));
  memcpy(entry.secret, full_hash, CIPHER_KEY_SIZE);
  // Derive channel hash from the secret (using 16-byte key length, matching addChannel)
  mesh::Utils::sha256(entry.hash, sizeof(entry.hash), entry.secret, CIPHER_KEY_SIZE);
  StrHelper::strncpy(entry.name, name, sizeof(entry.name));
}

bool FilterModule::addChanNameFilter(const char* name) {
  // must be a "#name" that fits the fixed-size name buffer
  if (name == NULL || name[0] != '#' || strlen(name) < 2
      || strlen(name) >= sizeof(_chan_name_filters[0].name)) return false;
  // Check for duplicate
  for (int i = 0; i < _num_chan_name_filters; i++) {
    if (strcmp(_chan_name_filters[i].name, name) == 0) return true; // already exists
  }
  if (_num_chan_name_filters >= MAX_CHAN_NAME_FILTERS) return false; // full
  deriveChanNameFilter(_chan_name_filters[_num_chan_name_filters], name);
  _num_chan_name_filters++;
  return true;
}

bool FilterModule::removeChanNameFilter(const char* name) {
  for (int i = 0; i < _num_chan_name_filters; i++) {
    if (strcmp(_chan_name_filters[i].name, name) == 0) {
      // shift remaining entries down
      for (int j = i; j < _num_chan_name_filters - 1; j++) {
        _chan_name_filters[j] = _chan_name_filters[j + 1];
      }
      _num_chan_name_filters--;
      memset(&_chan_name_filters[_num_chan_name_filters], 0, sizeof(ChanNameFilter));
      return true;
    }
  }
  return false;
}

void FilterModule::loadChanBlacklist(const char* fname) {
  // Load both hex prefix entries and #channel_name entries from the same file
  loadBlacklist(fname, _chan_blacklist);  // loads hex entries into _chan_blacklist

  // A channel blacklist hex entry is a single-byte channel hash; drop any legacy/oversized entries
  for (int i = 0; i < MAX_BLACKLIST_ENTRIES; i++) {
    if (_chan_blacklist[i].len != PATH_HASH_SIZE) _chan_blacklist[i].len = 0;
  }

  // Now re-read the file to pick up #channel_name lines
  _num_chan_name_filters = 0;
  memset(_chan_name_filters, 0, sizeof(_chan_name_filters));
#if defined(RP2040_PLATFORM)
  File f = _fs->open(fname, "r");
#else
  File f = _fs->open(fname);
#endif
  if (!f) return;
  char line[40];
  int line_len = 0;
  while (f.available() && _num_chan_name_filters < MAX_CHAN_NAME_FILTERS) {
    int c = f.read();
    if (c < 0) break;
    if (c == '\n' || c == '\r') {
      if (line_len > 1 && line[0] == '#' && line_len < (int)sizeof(_chan_name_filters[0].name)) {
        line[line_len] = 0;
        deriveChanNameFilter(_chan_name_filters[_num_chan_name_filters], line);
        _num_chan_name_filters++;
      }
      line_len = 0;
    } else if (line_len < (int)(sizeof(line) - 1)) {
      line[line_len++] = (char)c;
    }
  }
  // handle last line with no trailing newline
  if (line_len > 1 && line[0] == '#' && line_len < (int)sizeof(_chan_name_filters[0].name)
      && _num_chan_name_filters < MAX_CHAN_NAME_FILTERS) {
    line[line_len] = 0;
    deriveChanNameFilter(_chan_name_filters[_num_chan_name_filters], line);
    _num_chan_name_filters++;
  }
  f.close();
}

void FilterModule::saveChanBlacklist(const char* fname) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  _fs->remove(fname);
  File f = _fs->open(fname, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  File f = _fs->open(fname, "w");
#else
  File f = _fs->open(fname, "w", true);
#endif
  if (!f) return;
  // Write hex prefix entries
  for (int i = 0; i < MAX_BLACKLIST_ENTRIES; i++) {
    if (_chan_blacklist[i].len == 0) continue;
    char hex[MAX_PATH_PREFIX_LEN * 2 + 2];
    mesh::Utils::toHex(hex, _chan_blacklist[i].prefix, _chan_blacklist[i].len);
    f.println(hex);
  }
  // Write #channel_name entries
  for (int i = 0; i < _num_chan_name_filters; i++) {
    f.println(_chan_name_filters[i].name);
  }
  f.close();
}

void FilterModule::formatChanBlacklist(char* reply, size_t reply_sz, int offset) const {
  ListReplyPager pager(reply, reply_sz, offset);
  // Hex prefix entries first, then the #channel_name entries
  for (int i = 0; i < MAX_BLACKLIST_ENTRIES; i++) {
    if (_chan_blacklist[i].len == 0) continue;
    char hex[MAX_PATH_PREFIX_LEN * 2 + 1];
    mesh::Utils::toHex(hex, _chan_blacklist[i].prefix, _chan_blacklist[i].len);
    pager.add(hex, _chan_blacklist[i].len * 2);
  }
  for (int i = 0; i < _num_chan_name_filters; i++) {
    pager.add(_chan_name_filters[i].name, strlen(_chan_name_filters[i].name));
  }
  pager.finish();
}

// Jailed advert senders, most strikes first: '<key prefix>:<strikes>/<threshold>'.
void FilterModule::formatAdvertJail(char* reply, size_t reply_sz, int offset,
                                    uint32_t now_ms) const {
  ListReplyPager pager(reply, reply_sz, offset);
  const AdvertJailEntry* jailed[MAX_ADVERT_JAIL_ENTRIES];
  int count = 0;
  for (int i = 0; i < MAX_ADVERT_JAIL_ENTRIES; i++) {
    if (_advert_jail[i].last_seen != 0 && _advert_jail[i].jailed) {
      jailed[count++] = &_advert_jail[i];
    }
  }
  std::sort(jailed, jailed + count, [now_ms](const AdvertJailEntry* a, const AdvertJailEntry* b) {
    if (a->strikes != b->strikes) return a->strikes > b->strikes;
    return now_ms - a->last_seen < now_ms - b->last_seen;   // most recently heard first
  });

  for (int i = 0; i < count; i++) {
    char line[24];
    char hex[ADVERT_JAIL_KEY_SIZE * 2 + 1];
    mesh::Utils::toHex(hex, jailed[i]->key, sizeof(jailed[i]->key));
    int len = snprintf(line, sizeof(line), "%s:%u/%u", hex, (unsigned)jailed[i]->strikes,
                       (unsigned)ADVERT_JAIL_THRESHOLD);
    pager.add(line, len);
  }
  pager.finish();
}

/* ---------------------------- prefs file ---------------------------------- */

void FilterModule::loadPrefs() {
  _prefs.clear();
  if (!_fs->exists(FILTER_PREFS_FILE)) return;
#if defined(RP2040_PLATFORM)
  File file = _fs->open(FILTER_PREFS_FILE, "r");
#else
  File file = _fs->open(FILTER_PREFS_FILE);
#endif
  if (!file) return;

  uint8_t buf[FilterPrefs::MAX_READ];
  int len = file.read(buf, sizeof(buf));
  file.close();
  if (len > 0) {
    _prefs.deserialize(buf, len);   // a malformed file leaves the defaults in place
  }
}

bool FilterModule::savePrefs() {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  _fs->remove(FILTER_PREFS_FILE);
  File file = _fs->open(FILTER_PREFS_FILE, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  File file = _fs->open(FILTER_PREFS_FILE, "w");
#else
  File file = _fs->open(FILTER_PREFS_FILE, "w", true);
#endif
  if (!file) return false;

  uint8_t buf[FilterPrefs::MAX_SERIALIZED];
  size_t len = _prefs.serialize(buf);
  bool success = (file.write(buf, len) == len);
  file.close();
  return success;
}

/* -------------------------------- CLI ------------------------------------- */

bool FilterModule::handleCommand(char* command, char* reply, size_t reply_sz,
                                 uint32_t now_ms) {
  const size_t cap = reply_sz < MAX_BLACKLIST_REPLY_LEN ? reply_sz : MAX_BLACKLIST_REPLY_LEN;

  if (memcmp(command, "blacklist ", 10) == 0) {
    // Commands:
    //   blacklist path list [offset]
    //   blacklist path add <hex>[,<hex>,...]   (each hex = 2-8 even chars -> 1-4 bytes)
    //   blacklist path rem <hex>[,<hex>,...]
    //   blacklist path clear
    //   blacklist chan list [offset]
    //   blacklist chan add <hash|#name>[,<hash|#name>,...]   (hash = 2 hex chars -> 1 byte)
    //   blacklist chan rem <hash|#name>[,<hash|#name>,...]
    //   blacklist chan clear
    //   blacklist stats
    const char* parts[5];
    int n = mesh::Utils::parseTextParts(command, parts, 5, ' ');

    BlacklistEntry* list = NULL;
    const char* list_file = NULL;
    bool is_chan = false;
    if (n >= 2 && strcmp(parts[1], "path") == 0) {
      list = _path_blacklist;  list_file = PATH_BLACKLIST_FILE;
    } else if (n >= 2 && strcmp(parts[1], "chan") == 0) {
      list = _chan_blacklist;  list_file = CHAN_BLACKLIST_FILE;
      is_chan = true;
    }

    if (n == 2 && strcmp(parts[1], "stats") == 0) {
      snprintf(reply, cap, "path:%lu chan:%lu jail:%lu",
               (unsigned long)_drops_path, (unsigned long)_drops_chan, (unsigned long)_drops_jail);
    } else if (list && n >= 3 && strcmp(parts[2], "list") == 0) {
      int offset = (n >= 4) ? atoi(parts[3]) : 0;
      if (is_chan) {
        formatChanBlacklist(reply, cap, offset);
      } else {
        formatBlacklist(list, reply, cap, offset);
      }
    } else if (list && n >= 3 && strcmp(parts[2], "clear") == 0) {
      memset(list, 0, sizeof(BlacklistEntry) * MAX_BLACKLIST_ENTRIES);
      if (is_chan) {
        _num_chan_name_filters = 0;
        memset(_chan_name_filters, 0, sizeof(_chan_name_filters));
        saveChanBlacklist(list_file);
      } else {
        saveBlacklist(list_file, list);
      }
      strcpy(reply, "OK");
    } else if (list && n >= 4 && (strcmp(parts[2], "add") == 0 || strcmp(parts[2], "rem") == 0)) {
      bool is_add = (parts[2][0] == 'a');
      // parts[3] may be a comma-separated list of hex entries or #channel_name entries.
      // It points into the mutable command buffer, so parse it in place (no copy/truncation).
      const char* tok_parts[MAX_BLACKLIST_ENTRIES];
      int tok_n = mesh::Utils::parseTextParts((char*)parts[3], tok_parts, MAX_BLACKLIST_ENTRIES, ',');

      bool any_ok = false, any_err = false;
      for (int t = 0; t < tok_n; t++) {
        if (is_chan && tok_parts[t][0] == '#') {
          // #channel_name entry
          bool ok = is_add ? addChanNameFilter(tok_parts[t])
                           : removeChanNameFilter(tok_parts[t]);
          if (ok) any_ok = true; else any_err = true;
        } else {
          int hex_str_len = strlen(tok_parts[t]);
          // channel hex entries are a single-byte hash; path entries are 1..MAX_PATH_PREFIX_LEN bytes
          int max_hex_len = is_chan ? (PATH_HASH_SIZE * 2) : (MAX_PATH_PREFIX_LEN * 2);
          if (hex_str_len < 2 || hex_str_len > max_hex_len || (hex_str_len % 2) != 0
              || !isHexStr(tok_parts[t], hex_str_len)) {
            any_err = true; continue;
          }
          uint8_t prefix[MAX_PATH_PREFIX_LEN];
          int byte_len = hex_str_len / 2;
          if (!mesh::Utils::fromHex(prefix, byte_len, tok_parts[t])) {
            any_err = true; continue;
          }
          bool ok = is_add ? addToBlacklist(list, prefix, (uint8_t)byte_len)
                           : removeFromBlacklist(list, prefix, (uint8_t)byte_len);
          if (ok) any_ok = true; else any_err = true;
        }
      }

      // auto-save on any successful mutation
      if (any_ok) {
        if (is_chan) {
          saveChanBlacklist(list_file);
        } else {
          saveBlacklist(list_file, list);
        }
      }

      if (any_ok && !any_err)       strcpy(reply, "OK");
      else if (any_ok && any_err)   strcpy(reply, "OK (partial)");
      else                          strcpy(reply, is_add ? "Err - list full or bad input" : "Err - not found or bad input");
    } else {
      strcpy(reply, "Err - usage: blacklist <path|chan> <list [offset]|add|rem|clear> [hex|#name[,...]] | blacklist stats");
    }
    return true;
  }

  if (memcmp(command, "set advert.jail ", 16) == 0) {
    int hours = atoi(&command[16]);
    if (hours < 0 || hours > MAX_ADVERT_JAIL_HOURS) {
      snprintf(reply, reply_sz, "Err - range is 0-%d hours (0=off)", MAX_ADVERT_JAIL_HOURS);
    } else {
      _prefs.advert_jail_hours = (uint8_t)hours;
      // The module owns this setting: report a failed write instead of pretending
      // the value was saved.
      if (savePrefs()) {
        strcpy(reply, "OK");
      } else {
        strcpy(reply, "Err - could not be saved");
      }
    }
    return true;
  }

  if (strcmp(command, "get advert.jail") == 0) {
    snprintf(reply, reply_sz, "> %d", (int)_prefs.advert_jail_hours);
    return true;
  }

  if (strcmp(command, "jail clear") == 0) {
    memset(_advert_jail, 0, sizeof(_advert_jail));
    strcpy(reply, "OK");
    return true;
  }

  if (strcmp(command, "jail") == 0 || memcmp(command, "jail ", 5) == 0) {
    formatAdvertJail(reply, cap, command[4] ? atoi(&command[5]) : 0, now_ms);
    return true;
  }

  return false;
}
