#pragma once

// Owns the trusted time-source list, the replay cache and the `timesync` CLI.
// MyMesh supplies the adverts and the platform services; this module never
// includes MyMesh.h or NodePrefs.

#include <stddef.h>
#include <stdint.h>

#include "FsCompat.h"
#include "TimeSync.h"

namespace mesh {
class Identity;
class Packet;
class RTCClock;
}

// Prefix resolution needs the repeater's neighbour table and public key, both
// of which live in MyMesh. The module gets them through this narrow context.
struct TimeSyncModuleContext {
  const uint8_t* self_pub_key = nullptr;
  int (*resolve_neighbour)(void* ctx, const uint8_t* prefix, size_t len,
                           uint8_t* full_key) = nullptr;
  void* resolver_ctx = nullptr;
};

class TimeSyncModule {
public:
  void begin(FILESYSTEM* fs, const TimeSyncModuleContext& context);

  // Called for every signature-verified advert. Returns true when the clock
  // was moved. `is_share` is the transport-codes {0,0} case, `airtime_ms` is
  // the sender's estimated airtime for this packet.
  bool onAdvert(const mesh::Identity& id, const mesh::Packet* packet,
                uint32_t advert_timestamp, bool is_share,
                uint32_t airtime_ms, uint32_t now_ms, mesh::RTCClock& rtc);

  // `args` is everything after the `timesync` command word. `serial` enables
  // the full-key dump that only makes sense on the serial console.
  void handleCommand(const char* args, char* reply, size_t reply_sz,
                     uint32_t now_ms, bool serial);

private:
  FILESYSTEM* _fs = nullptr;
  TimeSyncModuleContext _context;
  TimeSyncList _list;
  TimeSyncSources _sources;
  bool _has_synced = false;
  bool _from_defaults = false;
  unsigned long _last_millis = 0;
  uint8_t _last_key[4] = {};

  void load();
  bool save();
  void resetSyncStatus();
};
