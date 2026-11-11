#pragma once

#include <stdint.h>
#include <string.h>

#define ADVERT_JAIL_KEY_SIZE   8   // public key prefix: long enough that nobody grinds a lookalike key
#define ADVERT_JAIL_THRESHOLD  6   // strikes that put a sender in jail

#ifndef MAX_ADVERT_JAIL_ENTRIES
  #define MAX_ADVERT_JAIL_ENTRIES  32
#endif

struct AdvertJailEntry {
  uint8_t key[ADVERT_JAIL_KEY_SIZE];
  uint32_t first_seen;        // millis() of the first counted advert
  uint32_t last_seen;         // millis() of the latest counted advert, 0 = free slot
  uint32_t last_advert_time;  // the sender's own timestamp on that advert
  uint16_t adverts;           // adverts counted
  uint8_t strikes;            // adverts that came sooner than the interval, capped at the threshold
  uint8_t jailed;
};

namespace mesh {

// Per-sender flood advert limit, ported from MeshSydney's advert.jail.
// An advert that comes sooner than 'interval_ms' after the sender's previous
// one is a strike; ADVERT_JAIL_THRESHOLD strikes put the sender in jail, and
// while jailed its early adverts are dropped. An advert that keeps to the
// interval takes a strike off; below 2 strikes the sender is let out. The same
// advert seen again (same timestamp) is not counted.
//
// Call this when forwarding, not when receiving: only adverts with a valid
// signature get there, and only the first copy of each. Nobody can jail a node
// with forged adverts carrying its key.
//
// \param  table        'size' entries, zero-filled when empty
// \param  pub_key      the sender's public key (the first ADVERT_JAIL_KEY_SIZE bytes are kept)
// \param  advert_time  the timestamp inside the advert, by the sender's clock
// \param  now_ms       a millisecond clock (wrap-around is fine)
// \returns  true if this advert must not be forwarded
inline bool checkAdvertJail(AdvertJailEntry* table, int size, const uint8_t* pub_key, uint32_t advert_time,
                            uint32_t now_ms, uint32_t interval_ms) {
  if (size <= 0) return false;
  if (now_ms == 0) now_ms = 1;   // 0 marks a free slot

  AdvertJailEntry* entry = NULL;
  for (int i = 0; i < size; i++) {
    if (table[i].last_seen != 0 && memcmp(table[i].key, pub_key, ADVERT_JAIL_KEY_SIZE) == 0) {
      entry = &table[i];
      break;
    }
  }

  if (entry == NULL) {   // first advert from this sender: take a free slot, else evict one
    AdvertJailEntry* slot = NULL;
    for (int i = 0; i < size; i++) {
      if (table[i].last_seen == 0) { slot = &table[i]; break; }
    }
    if (slot == NULL) {
      // keep the jailed senders: evict the fewest strikes, then the longest unheard
      slot = &table[0];
      for (int i = 1; i < size; i++) {
        if (table[i].strikes < slot->strikes
            || (table[i].strikes == slot->strikes
                && now_ms - table[i].last_seen > now_ms - slot->last_seen)) {
          slot = &table[i];
        }
      }
    }
    memset(slot, 0, sizeof(*slot));
    memcpy(slot->key, pub_key, ADVERT_JAIL_KEY_SIZE);
    slot->first_seen = slot->last_seen = now_ms;
    slot->last_advert_time = advert_time;
    slot->adverts = 1;
    return false;
  }

  if (advert_time == entry->last_advert_time) {   // the same advert again
    return entry->jailed != 0;
  }

  bool interval_ok = now_ms - entry->last_seen >= interval_ms;
  if (interval_ok && entry->strikes > 0) entry->strikes--;
  if (entry->jailed && entry->strikes < 2) entry->jailed = 0;

  entry->last_seen = now_ms;
  entry->last_advert_time = advert_time;
  if (entry->adverts < 0xFFFF) entry->adverts++;

  if (!interval_ok && entry->strikes < ADVERT_JAIL_THRESHOLD) entry->strikes++;
  if (!entry->jailed && entry->strikes >= ADVERT_JAIL_THRESHOLD) entry->jailed = 1;

  return entry->jailed && !interval_ok;
}

}   // namespace mesh
