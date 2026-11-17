#pragma once

#include <stddef.h>
#include <stdint.h>

#include "RemoteCommandConfig.h"

// Byte-level reader for the ed25519-signer blob:
//
//   blob = C || S
//   P    = timestamp_le32 || dict_id_le16 || cmdpack(text)
//   S    = Ed25519_sign(P)
//   C    = P XOR SHA512("MC1-mask" || S[:32] || counter_le32)
//
// The functions are split so the masked payload and the dictionary codec can
// be tested without a real Ed25519 implementation. remoteCmdOpen() is the
// production entry point and checks the dictionary id and both public keys.

bool remoteCmdMask(const uint8_t* payload, size_t payload_len,
                   const uint8_t* signature, uint8_t* masked);

bool remoteCmdUnmask(const uint8_t* blob, size_t blob_len,
                     uint8_t* payload, size_t payload_cap, size_t& payload_len);

bool remoteCmdDecodeText(const uint8_t* compressed, size_t compressed_len,
                         char* text, size_t text_cap, size_t& text_len);

bool remoteCmdDecode(const uint8_t* payload, size_t payload_len,
                     char* text, size_t text_cap, size_t& text_len,
                     uint32_t& timestamp, uint16_t& dict_id);

// key1 is tried first, then key2 when non-null.
bool remoteCmdOpen(const uint8_t* blob, size_t blob_len,
                   const uint8_t* key1, const uint8_t* key2,
                   char* text, size_t text_cap, size_t& text_len,
                   uint32_t& timestamp);
