#pragma once

#include <MeshCore.h>
#include <Packet.h>
#include <Utils.h>

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
