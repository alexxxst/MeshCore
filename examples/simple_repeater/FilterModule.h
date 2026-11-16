#pragma once

// Owns the path/channel blacklists, channel-name filters, the advert jail and
// their CLI. MyMesh only calls the veto methods from its packet hooks.

#include <stddef.h>
#include <stdint.h>

#include "AdvertJail.h"
#include "Blacklist.h"
#include "ChanFilter.h"
#include "FilterPrefs.h"
#include "FsCompat.h"

namespace mesh {
class Packet;
}

#ifndef MAX_CHAN_NAME_FILTERS
  #define MAX_CHAN_NAME_FILTERS   15
#endif

// Conservative cap for blacklist CLI replies: both serial (160) and over-radio
// (161, minus optional 3-byte 'xx|' prefix) buffers are smaller than MAX_PACKET_PAYLOAD.
#define MAX_BLACKLIST_REPLY_LEN   150

class FilterModule {
public:
  void begin(FILESYSTEM* fs);

  // Receive path: drops a flood packet when its path or channel is blocked.
  bool vetoRecvFlood(const mesh::Packet* packet);

  // Forward path: a direct-routed group packet never reaches vetoRecvFlood().
  bool vetoForwardDirect(const mesh::Packet* packet);

  // Forward path: advert jail, checked after signature verification.
  bool vetoForwardAdvert(const mesh::Packet* packet, uint32_t now_ms);

  void clearStats() { _drops_path = _drops_chan = _drops_jail = 0; }

  // Returns true when the command belongs to this module.
  bool handleCommand(char* command, char* reply, size_t reply_sz, uint32_t now_ms);

private:
  FILESYSTEM* _fs = nullptr;
  FilterPrefs _prefs;
  BlacklistEntry _path_blacklist[MAX_BLACKLIST_ENTRIES] = {};
  BlacklistEntry _chan_blacklist[MAX_BLACKLIST_ENTRIES] = {};
  ChanNameFilter _chan_name_filters[MAX_CHAN_NAME_FILTERS] = {};
  int _num_chan_name_filters = 0;
  AdvertJailEntry _advert_jail[MAX_ADVERT_JAIL_ENTRIES] = {};
  uint32_t _drops_path = 0, _drops_chan = 0, _drops_jail = 0;

  bool isPathBlacklisted(const mesh::Packet* packet) const;
  bool isChanBlacklisted(const mesh::Packet* packet) const;
  bool addToBlacklist(BlacklistEntry* list, const uint8_t* prefix, uint8_t len);
  bool removeFromBlacklist(BlacklistEntry* list, const uint8_t* prefix, uint8_t len);
  void formatBlacklist(const BlacklistEntry* list, char* reply, size_t reply_sz, int offset) const;
  void loadBlacklist(const char* fname, BlacklistEntry* list);
  void saveBlacklist(const char* fname, const BlacklistEntry* list);

  void deriveChanNameFilter(ChanNameFilter& entry, const char* name);
  bool addChanNameFilter(const char* name);
  bool removeChanNameFilter(const char* name);
  void loadChanBlacklist(const char* fname);
  void saveChanBlacklist(const char* fname);
  void formatChanBlacklist(char* reply, size_t reply_sz, int offset) const;
  void formatAdvertJail(char* reply, size_t reply_sz, int offset, uint32_t now_ms) const;

  void loadPrefs();
  bool savePrefs();
};
