#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <ed_25519.h>

#include "RemoteCommandCodec.h"
#include "RemoteCommandConfig.h"
#include "RemoteCommandDictionary.h"

namespace {

const uint8_t GOLDEN_BLOB[] = {
  0x22, 0x07, 0xdd, 0x5a, 0x51, 0xff, 0x9f, 0x51, 0xe8, 0xcf, 0xaa, 0xfc,
  0xaf, 0x80, 0x0c, 0x8b, 0xb1, 0x24, 0x04, 0x8e, 0x3e, 0x66, 0x72, 0xf9,
  0xf4, 0x9c, 0xe9, 0x20, 0x12, 0x07, 0x9f, 0xd0, 0xfb, 0xd0, 0xc9, 0x58,
  0x57, 0x5b, 0x05, 0xaa, 0x99, 0x33, 0xce, 0x42, 0x95, 0x4b, 0x37, 0xd0,
  0x01, 0xff, 0xa2, 0x3e, 0xb6, 0xf5, 0x8c, 0xef, 0x9c, 0x20, 0x97, 0x4f,
  0xf1, 0xb8, 0xe4, 0xe7, 0xf5, 0x0d, 0xee, 0xb7, 0x9d, 0xbf, 0xcb, 0xda,
  0x84, 0x8b, 0x0f, 0xb6, 0x50, 0x16, 0x98, 0xc5, 0x0b, 0xb3, 0x62, 0x08,
  0x7a, 0xbf, 0x1e, 0xa4, 0x09
};

const uint8_t GOLDEN_PUB[] = {
  0x73, 0x8d, 0xc9, 0x66, 0x55, 0x75, 0xc3, 0xba, 0x7c, 0xec, 0x96, 0xc6,
  0x79, 0x95, 0x0a, 0x7d, 0x9d, 0x81, 0x98, 0xd3, 0x5d, 0xe9, 0xd3, 0x2f,
  0x5d, 0xdb, 0x30, 0xd7, 0x59, 0x7f, 0xe7, 0xb4
};

const uint8_t GOLDEN_PRIV[] = {
  0x48, 0x98, 0x46, 0x65, 0xe3, 0xab, 0x81, 0x0a, 0xd4, 0xbe, 0x93, 0xb6,
  0x7a, 0x31, 0x3a, 0x4c, 0x68, 0xd5, 0x0a, 0x95, 0xa4, 0x90, 0xdf, 0x22,
  0xb4, 0x93, 0x24, 0x89, 0x25, 0xe0, 0xc5, 0x42, 0xa7, 0x53, 0x7b, 0x59,
  0x81, 0x7c, 0x39, 0x0c, 0x77, 0x93, 0x16, 0xfb, 0x49, 0x40, 0x24, 0x29,
  0xf0, 0x94, 0xe0, 0x55, 0x21, 0xcd, 0x91, 0x22, 0xc5, 0x3e, 0xab, 0xa3,
  0xef, 0xa0, 0xba, 0x15
};

std::vector<uint8_t> buildBlob(uint32_t timestamp, uint16_t dict_id,
                               const std::vector<uint8_t>& compressed) {
  std::vector<uint8_t> payload;
  payload.push_back((uint8_t)(timestamp & 0xFF));
  payload.push_back((uint8_t)((timestamp >> 8) & 0xFF));
  payload.push_back((uint8_t)((timestamp >> 16) & 0xFF));
  payload.push_back((uint8_t)((timestamp >> 24) & 0xFF));
  payload.push_back((uint8_t)(dict_id & 0xFF));
  payload.push_back((uint8_t)(dict_id >> 8));
  payload.insert(payload.end(), compressed.begin(), compressed.end());

  uint8_t signature[REMOTE_CMD_SIGNATURE_SIZE];
  ed25519_sign(signature, payload.data(), payload.size(), GOLDEN_PUB, GOLDEN_PRIV);

  std::vector<uint8_t> masked(payload.size());
  if (!remoteCmdMask(payload.data(), payload.size(), signature, masked.data())) {
    ADD_FAILURE() << "remoteCmdMask failed";
    return {};
  }

  std::vector<uint8_t> blob = masked;
  blob.insert(blob.end(), signature, signature + sizeof(signature));
  return blob;
}

}  // namespace

TEST(RemoteCommandCodec, GoldenBlobUnmasksAndDecodes) {
  uint8_t payload[REMOTE_CMD_MAX_PAYLOAD];
  size_t payload_len = 0;
  ASSERT_TRUE(remoteCmdUnmask(GOLDEN_BLOB, sizeof(GOLDEN_BLOB), payload, sizeof(payload), payload_len));

  char text[REMOTE_CMD_MAX_TEXT];
  size_t text_len = 0;
  uint32_t timestamp = 0;
  uint16_t dict_id = 0;
  ASSERT_TRUE(remoteCmdDecode(payload, payload_len, text, sizeof(text), text_len, timestamp, dict_id));

  EXPECT_EQ(1700000000u, timestamp);
  EXPECT_EQ(0xD55Cu, dict_id);
  EXPECT_EQ("reboot\nset radio 869.618,62.5,8,5\nset guest.password \n", std::string(text, text_len));
}

TEST(RemoteCommandCodec, GoldenSignatureVerifiesWithTheRealEd25519) {
  uint8_t payload[REMOTE_CMD_MAX_PAYLOAD];
  size_t payload_len = 0;
  ASSERT_TRUE(remoteCmdUnmask(GOLDEN_BLOB, sizeof(GOLDEN_BLOB), payload, sizeof(payload), payload_len));

  const uint8_t* signature = GOLDEN_BLOB + payload_len;
  ASSERT_EQ(1, ed25519_verify(signature, payload, payload_len, GOLDEN_PUB));

  payload[0] ^= 0x01;
  EXPECT_EQ(0, ed25519_verify(signature, payload, payload_len, GOLDEN_PUB));
}

TEST(RemoteCommandCodec, OpenChecksDictionaryAndSignature) {
  char text[REMOTE_CMD_MAX_TEXT];
  size_t text_len = 0;
  uint32_t timestamp = 0;

  ASSERT_TRUE(remoteCmdOpen(GOLDEN_BLOB, sizeof(GOLDEN_BLOB), GOLDEN_PUB, nullptr,
                            text, sizeof(text), text_len, timestamp));
  EXPECT_EQ(1700000000u, timestamp);
  EXPECT_EQ("reboot\nset radio 869.618,62.5,8,5\nset guest.password \n", std::string(text, text_len));

  // The mock Identity verifier always accepts, so a different dict_id is what
  // this test can actually exercise on the native target.
  std::vector<uint8_t> compressed = {0};  // token: reboot
  auto wrong_dict = buildBlob(1700000001u, 0x1234u, compressed);
  EXPECT_FALSE(remoteCmdOpen(wrong_dict.data(), wrong_dict.size(), GOLDEN_PUB, nullptr,
                             text, sizeof(text), text_len, timestamp));
}

TEST(RemoteCommandCodec, InvalidPacketsDoNotTouchTheCallerTextBuffer) {
  char text[64];
  size_t text_len = 0;
  uint32_t timestamp = 0;

  std::vector<uint8_t> bad_dict = buildBlob(1700000000u, 0x1234u, {0});
  strcpy(text, "SENTINEL");
  EXPECT_FALSE(remoteCmdOpen(bad_dict.data(), bad_dict.size(), GOLDEN_PUB, nullptr,
                             text, sizeof(text), text_len, timestamp));
  EXPECT_STREQ("SENTINEL", text);

  std::vector<uint8_t> bad_sig(GOLDEN_BLOB, GOLDEN_BLOB + sizeof(GOLDEN_BLOB));
  bad_sig.back() ^= 0x01;
  strcpy(text, "SENTINEL");
  EXPECT_FALSE(remoteCmdOpen(bad_sig.data(), bad_sig.size(), GOLDEN_PUB, nullptr,
                             text, sizeof(text), text_len, timestamp));
  EXPECT_STREQ("SENTINEL", text);
}

TEST(RemoteCommandCodec, FallsBackFromKey1ToKey2) {
  uint8_t wrong_key[REMOTE_CMD_KEY_SIZE];
  memset(wrong_key, 0x11, sizeof(wrong_key));

  char text[REMOTE_CMD_MAX_TEXT];
  size_t text_len = 0;
  uint32_t timestamp = 0;

  EXPECT_FALSE(remoteCmdOpen(GOLDEN_BLOB, sizeof(GOLDEN_BLOB), wrong_key, nullptr,
                             text, sizeof(text), text_len, timestamp));
  ASSERT_TRUE(remoteCmdOpen(GOLDEN_BLOB, sizeof(GOLDEN_BLOB), wrong_key, GOLDEN_PUB,
                            text, sizeof(text), text_len, timestamp));
  EXPECT_EQ(1700000000u, timestamp);
}

TEST(RemoteCommandCodec, DecodesLiteralAndHexRuns) {
  std::vector<uint8_t> compressed;
  compressed.push_back(0);  // reboot
  compressed.push_back(254); compressed.push_back(5);
  const char literal[] = "hello";
  compressed.insert(compressed.end(), literal, literal + 5);
  compressed.push_back(255); compressed.push_back(4);
  compressed.push_back(0x12); compressed.push_back(0xAB);  // "12ab"

  auto blob = buildBlob(42, 0xD55C, compressed);
  char text[REMOTE_CMD_MAX_TEXT];
  size_t text_len = 0;
  uint32_t timestamp = 0;
  ASSERT_TRUE(remoteCmdOpen(blob.data(), blob.size(), GOLDEN_PUB, nullptr,
                            text, sizeof(text), text_len, timestamp));

  EXPECT_EQ(42u, timestamp);
  EXPECT_EQ("reboothello12ab", std::string(text, text_len));
}

TEST(RemoteCommandCodec, RejectsMalformedOrOversizedCompressedText) {
  char text[16];
  size_t text_len = 0;

  std::vector<uint8_t> truncated = {254, 5, 'a'};
  EXPECT_FALSE(remoteCmdDecodeText(truncated.data(), truncated.size(), text, sizeof(text), text_len));

  std::vector<uint8_t> bad_code = {253};
  EXPECT_FALSE(remoteCmdDecodeText(bad_code.data(), bad_code.size(), text, sizeof(text), text_len));

  std::vector<uint8_t> too_long = {254, 20};
  for (int i = 0; i < 20; ++i) too_long.push_back('x');
  EXPECT_FALSE(remoteCmdDecodeText(too_long.data(), too_long.size(), text, sizeof(text), text_len));

  std::vector<uint8_t> with_nul = {254, 3, 'a', 0, 'b'};
  EXPECT_FALSE(remoteCmdDecodeText(with_nul.data(), with_nul.size(), text, sizeof(text), text_len));
}

TEST(RemoteCommandCodec, RejectsBlobsOutsideTheSignerSizeWindow) {
  uint8_t payload[REMOTE_CMD_MAX_PAYLOAD];
  size_t payload_len = 0;
  EXPECT_FALSE(remoteCmdUnmask(GOLDEN_BLOB, 69, payload, sizeof(payload), payload_len));
  EXPECT_FALSE(remoteCmdUnmask(GOLDEN_BLOB, REMOTE_CMD_MAX_BLOB + 1, payload, sizeof(payload), payload_len));
}
