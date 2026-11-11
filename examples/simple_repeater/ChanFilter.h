#pragma once

#include <MeshCore.h>
#include <Packet.h>
#include <Utils.h>
#include "Blacklist.h"

// A channel-name blacklist entry: the derived channel secret and its 1-byte hash.
struct ChanNameFilter {
  uint8_t hash[PATH_HASH_SIZE];
  uint8_t secret[PUB_KEY_SIZE];
  char    name[32];
};

// Decrypts 'src' with 'secret'; returns the plaintext length, or <= 0 on failure.
typedef int (*ChanDecryptFn)(const uint8_t* secret, uint8_t* dest, const uint8_t* src, int src_len);

// Returns true when a GRP_TXT/GRP_DATA packet belongs to one of the filtered channels.
// A packet whose ciphertext is not a whole number of blocks cannot be verified, but a
// matching channel hash still says it belongs to a filtered channel, so it is dropped.
inline bool isChanNameFiltered(const mesh::Packet* packet, const ChanNameFilter* filters, int num,
                               ChanDecryptFn decrypt = mesh::Utils::MACThenDecrypt) {
  if (num <= 0 || packet->payload_len <= PATH_HASH_SIZE + CIPHER_MAC_SIZE) return false;

  uint8_t channel_hash = packet->payload[0];
  const uint8_t* mac_and_data = &packet->payload[PATH_HASH_SIZE];
  int enc_len = packet->payload_len - PATH_HASH_SIZE;
  int ct_len = enc_len - CIPHER_MAC_SIZE;
  // A genuine sender always emits whole AES blocks; anything else is malformed.
  bool block_aligned = ct_len > 0 && (ct_len % CIPHER_BLOCK_SIZE) == 0;

  for (int i = 0; i < num; i++) {
    // Quick hash check first
    if (channel_hash != filters[i].hash[0]) continue;
    if (!block_aligned) return true;   // malformed packet carrying a filtered channel hash
    // Try to decrypt to verify this is actually the channel
    uint8_t tmp[MAX_PACKET_PAYLOAD];
    int len = decrypt(filters[i].secret, tmp, mac_and_data, enc_len);
    if (len > 0) return true;   // successfully decrypted, confirmed channel match
  }
  return false;
}

// Returns true when a GRP_TXT/GRP_DATA packet belongs to a blacklisted channel,
// either by its 1-byte hash or by a #channel_name entry verified by decryption.
inline bool isChanFiltered(const mesh::Packet* packet, const BlacklistEntry* hashes,
                           const ChanNameFilter* names, int num_names,
                           ChanDecryptFn decrypt = mesh::Utils::MACThenDecrypt) {
  uint8_t pt = packet->getPayloadType();
  if (pt != PAYLOAD_TYPE_GRP_TXT && pt != PAYLOAD_TYPE_GRP_DATA) return false;
  if (packet->payload_len < PATH_HASH_SIZE) return false;

  for (int b = 0; b < MAX_BLACKLIST_ENTRIES; b++) {
    if (hashes[b].len == 0) continue;
    uint8_t avail = (packet->payload_len < MAX_PATH_PREFIX_LEN) ? (uint8_t)packet->payload_len : MAX_PATH_PREFIX_LEN;
    uint8_t cmp_len = (hashes[b].len < avail) ? hashes[b].len : avail;
    if (memcmp(packet->payload, hashes[b].prefix, cmp_len) == 0) return true;
  }

  return isChanNameFiltered(packet, names, num_names, decrypt);
}
