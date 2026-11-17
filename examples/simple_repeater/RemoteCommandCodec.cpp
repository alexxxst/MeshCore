#include "RemoteCommandCodec.h"

#include <string.h>

extern "C" {
#include <sha512.h>
}

#if defined(REMOTE_CMD_NATIVE_TEST)
#include <ed_25519.h>
#else
#include <Mesh.h>
#endif

#include "RemoteCommandDictionary.h"

namespace {

const char MASK_LABEL[] = "MC1-mask";
const uint8_t HEX_CHARS[] = "0123456789abcdef";

uint32_t readLe32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

uint16_t readLe16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

void makeMask(const uint8_t* seed, uint8_t* out, size_t len) {
  size_t pos = 0;
  uint32_t counter = 0;
  while (pos < len) {
    uint8_t hash[64];
    uint8_t counter_le[4] = {
        (uint8_t)(counter & 0xFF),
        (uint8_t)((counter >> 8) & 0xFF),
        (uint8_t)((counter >> 16) & 0xFF),
        (uint8_t)((counter >> 24) & 0xFF),
    };

    sha512_context ctx;
    sha512_init(&ctx);
    sha512_update(&ctx, (const unsigned char*)MASK_LABEL, sizeof(MASK_LABEL) - 1);
    sha512_update(&ctx, seed, REMOTE_CMD_KEY_SIZE);
    sha512_update(&ctx, counter_le, sizeof(counter_le));
    sha512_final(&ctx, hash);

    size_t n = len - pos;
    if (n > sizeof(hash)) n = sizeof(hash);
    memcpy(out + pos, hash, n);
    pos += n;
    ++counter;
  }
}

bool verifyPayload(const uint8_t* signature, const uint8_t* payload,
                   size_t payload_len, const uint8_t* public_key) {
  if (signature == nullptr || payload == nullptr || public_key == nullptr) return false;
#if defined(REMOTE_CMD_NATIVE_TEST)
  return ed25519_verify(signature, payload, payload_len, public_key) == 1;
#else
  mesh::Identity id(public_key);
  return id.verify(signature, payload, (int)payload_len);
#endif
}

}  // namespace

bool remoteCmdMask(const uint8_t* payload, size_t payload_len,
                   const uint8_t* signature, uint8_t* masked) {
  if (payload == nullptr || signature == nullptr || masked == nullptr) return false;
  if (payload_len > REMOTE_CMD_MAX_PAYLOAD) return false;

  uint8_t mask[REMOTE_CMD_MAX_PAYLOAD];
  makeMask(signature, mask, payload_len);
  for (size_t i = 0; i < payload_len; ++i) {
    masked[i] = payload[i] ^ mask[i];
  }
  return true;
}

bool remoteCmdUnmask(const uint8_t* blob, size_t blob_len,
                     uint8_t* payload, size_t payload_cap, size_t& payload_len) {
  payload_len = 0;
  if (blob == nullptr || payload == nullptr) return false;
  if (blob_len < 4 + 2 + REMOTE_CMD_SIGNATURE_SIZE) return false;
  if (blob_len > REMOTE_CMD_MAX_BLOB) return false;

  size_t len = blob_len - REMOTE_CMD_SIGNATURE_SIZE;
  if (len > payload_cap) return false;

  const uint8_t* signature = blob + len;
  uint8_t mask[REMOTE_CMD_MAX_PAYLOAD];
  makeMask(signature, mask, len);
  for (size_t i = 0; i < len; ++i) {
    payload[i] = blob[i] ^ mask[i];
  }
  payload_len = len;
  return true;
}

bool remoteCmdDecodeText(const uint8_t* compressed, size_t compressed_len,
                         char* text, size_t text_cap, size_t& text_len) {
  text_len = 0;
  if (compressed == nullptr || text == nullptr || text_cap == 0) return false;

  size_t pos = 0;
  size_t out = 0;
  while (pos < compressed_len) {
    uint8_t code = compressed[pos++];

    if (code < REMOTE_CMD_DICT_COUNT) {
      const char* token = remoteCmdDictToken(code);
      if (token == nullptr) return false;
      size_t n = strlen(token);
      if (n >= text_cap - out) return false;   // keep room for NUL
      memcpy(text + out, token, n);
      out += n;
    } else if (code == 254) {                  // literal-run
      if (pos >= compressed_len) return false;
      uint8_t n = compressed[pos++];
      if (n > compressed_len - pos) return false;
      if (n >= text_cap - out) return false;
      memcpy(text + out, compressed + pos, n);
      out += n;
      pos += n;
    } else if (code == 255) {                  // hex-run
      if (pos >= compressed_len) return false;
      uint8_t n = compressed[pos++];
      if (n == 0) return false;
      size_t packed = (n + 1) / 2;
      if (packed > compressed_len - pos) return false;
      if (n >= text_cap - out) return false;
      for (size_t k = 0; k < n; ++k) {
        uint8_t byte = compressed[pos + k / 2];
        uint8_t nibble = (k & 1) ? (byte & 0x0F) : (byte >> 4);
        text[out++] = (char)HEX_CHARS[nibble];
      }
      pos += packed;
    } else {
      return false;
    }
  }

  text[out] = 0;
  if (memchr(text, 0, out) != nullptr) return false;   // commands are C strings
  text_len = out;
  return true;
}

bool remoteCmdDecode(const uint8_t* payload, size_t payload_len,
                     char* text, size_t text_cap, size_t& text_len,
                     uint32_t& timestamp, uint16_t& dict_id) {
  text_len = 0;
  timestamp = 0;
  dict_id = 0;
  if (payload == nullptr || payload_len < 6) return false;

  timestamp = readLe32(payload);
  dict_id = readLe16(payload + 4);
  return remoteCmdDecodeText(payload + 6, payload_len - 6, text, text_cap, text_len);
}

bool remoteCmdOpen(const uint8_t* blob, size_t blob_len,
                   const uint8_t* key1, const uint8_t* key2,
                   char* text, size_t text_cap, size_t& text_len,
                   uint32_t& timestamp) {
  text_len = 0;
  timestamp = 0;

  uint8_t payload[REMOTE_CMD_MAX_PAYLOAD];
  size_t payload_len = 0;
  if (!remoteCmdUnmask(blob, blob_len, payload, sizeof(payload), payload_len)) return false;
  if (payload_len < 6) return false;

  timestamp = readLe32(payload);
  uint16_t dict_id = readLe16(payload + 4);
  if (dict_id != REMOTE_CMD_DICT_ID) return false;

  const uint8_t* signature = blob + payload_len;
  bool verified = (key1 != nullptr) &&
                  verifyPayload(signature, payload, payload_len, key1);
  if (!verified && key2 != nullptr) {
    verified = verifyPayload(signature, payload, payload_len, key2);
  }
  if (!verified) return false;

  // Decode only after the packet is authenticated. The caller may hand us a
  // buffer that is also used by a command sequence that is already running.
  if (!remoteCmdDecodeText(payload + 6, payload_len - 6, text, text_cap, text_len)) {
    return false;
  }
  return true;
}
