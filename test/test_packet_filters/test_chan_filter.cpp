#include <gtest/gtest.h>
#include <Packet.h>
#include "ChanFilter.h"

using namespace mesh;

static int verify_calls;
static int verify_result;

static int fake_verify(const uint8_t*, uint8_t*, const uint8_t*, int) {
  verify_calls++;
  return verify_result;
}

static ChanNameFilter filter_for(uint8_t hash) {
  ChanNameFilter f;
  memset(&f, 0, sizeof(f));
  f.hash[0] = hash;
  return f;
}

static Packet group_packet(uint8_t channel_hash, int ciphertext_len,
                           uint8_t route = ROUTE_TYPE_FLOOD,
                           uint8_t payload_type = PAYLOAD_TYPE_GRP_TXT) {
  Packet p;
  p.header = route | (payload_type << PH_TYPE_SHIFT);
  p.payload[0] = channel_hash;
  p.payload_len = PATH_HASH_SIZE + CIPHER_MAC_SIZE + ciphertext_len;
  return p;
}

static BlacklistEntry hash_entry(uint8_t channel_hash) {
  BlacklistEntry entry;
  memset(&entry, 0, sizeof(entry));
  entry.len = 1;
  entry.prefix[0] = channel_hash;
  return entry;
}

TEST(ChanNameFilter, AlignedPacketFromFilteredChannelIsDropped) {
  verify_calls = 0;
  verify_result = 1;
  ChanNameFilter filter = filter_for(0xA1);
  Packet pkt = group_packet(0xA1, CIPHER_BLOCK_SIZE);

  EXPECT_TRUE(isChanNameFiltered(&pkt, &filter, 1, fake_verify));
  EXPECT_EQ(1, verify_calls);
}

TEST(ChanNameFilter, AlignedPacketFromOtherChannelIsKept) {
  verify_calls = 0;
  verify_result = 0;
  ChanNameFilter filter = filter_for(0xA1);
  Packet pkt = group_packet(0xA1, CIPHER_BLOCK_SIZE);

  EXPECT_FALSE(isChanNameFiltered(&pkt, &filter, 1, fake_verify));
  EXPECT_EQ(1, verify_calls);   // same hash, so the secret was tried
}

TEST(ChanNameFilter, HashMismatchDoesNotDecrypt) {
  verify_calls = 0;
  verify_result = 1;
  ChanNameFilter filter = filter_for(0xA1);
  Packet pkt = group_packet(0xB2, CIPHER_BLOCK_SIZE);

  EXPECT_FALSE(isChanNameFiltered(&pkt, &filter, 1, fake_verify));
  EXPECT_EQ(0, verify_calls);
}

// A blacklisted channel must not escape the filter by sending ciphertext that
// is not a whole number of AES blocks: it cannot be verified, but the channel
// hash still says it belongs to the filtered channel.
TEST(ChanNameFilter, UnalignedPacketWithFilteredHashIsDropped) {
  verify_calls = 0;
  verify_result = 1;
  ChanNameFilter filter = filter_for(0xA1);
  Packet pkt = group_packet(0xA1, CIPHER_BLOCK_SIZE + 1);

  EXPECT_TRUE(isChanNameFiltered(&pkt, &filter, 1, fake_verify));
  EXPECT_EQ(0, verify_calls);
}

TEST(ChanNameFilter, UnalignedPacketWithOtherHashIsKept) {
  verify_calls = 0;
  verify_result = 1;
  ChanNameFilter filter = filter_for(0xA1);
  Packet pkt = group_packet(0xB2, CIPHER_BLOCK_SIZE + 1);

  EXPECT_FALSE(isChanNameFiltered(&pkt, &filter, 1, fake_verify));
  EXPECT_EQ(0, verify_calls);
}

TEST(ChanNameFilter, NoFiltersKeepsPacket) {
  verify_calls = 0;
  verify_result = 1;
  Packet pkt = group_packet(0xA1, CIPHER_BLOCK_SIZE);

  EXPECT_FALSE(isChanNameFiltered(&pkt, nullptr, 0, fake_verify));
  EXPECT_EQ(0, verify_calls);
}

TEST(ChanBlacklist, HashEntryMatchesGroupText) {
  BlacklistEntry hashes[MAX_BLACKLIST_ENTRIES] = {};
  hashes[0] = hash_entry(0xA1);
  Packet pkt = group_packet(0xA1, CIPHER_BLOCK_SIZE);

  EXPECT_TRUE(isChanFiltered(&pkt, hashes, nullptr, 0, fake_verify));
}

TEST(ChanBlacklist, HashEntryMatchesGroupData) {
  BlacklistEntry hashes[MAX_BLACKLIST_ENTRIES] = {};
  hashes[0] = hash_entry(0xA1);
  Packet pkt = group_packet(0xA1, CIPHER_BLOCK_SIZE, ROUTE_TYPE_FLOOD, PAYLOAD_TYPE_GRP_DATA);

  EXPECT_TRUE(isChanFiltered(&pkt, hashes, nullptr, 0, fake_verify));
}

TEST(ChanBlacklist, HashEntryMatchesDirectRoutedGroupPacket) {
  BlacklistEntry hashes[MAX_BLACKLIST_ENTRIES] = {};
  hashes[0] = hash_entry(0xA1);
  Packet pkt = group_packet(0xA1, CIPHER_BLOCK_SIZE, ROUTE_TYPE_DIRECT);

  EXPECT_TRUE(isChanFiltered(&pkt, hashes, nullptr, 0, fake_verify));
}

TEST(ChanBlacklist, OtherPayloadTypesAreIgnored) {
  BlacklistEntry hashes[MAX_BLACKLIST_ENTRIES] = {};
  hashes[0] = hash_entry(0xA1);
  Packet pkt = group_packet(0xA1, CIPHER_BLOCK_SIZE, ROUTE_TYPE_FLOOD, PAYLOAD_TYPE_ACK);

  EXPECT_FALSE(isChanFiltered(&pkt, hashes, nullptr, 0, fake_verify));
}

TEST(ChanBlacklist, EmptyListsKeepPacket) {
  BlacklistEntry hashes[MAX_BLACKLIST_ENTRIES] = {};
  Packet pkt = group_packet(0xA1, CIPHER_BLOCK_SIZE);

  EXPECT_FALSE(isChanFiltered(&pkt, hashes, nullptr, 0, fake_verify));
}

TEST(ChanBlacklist, NameFilterIsAppliedThroughTheListCheck) {
  verify_calls = 0;
  verify_result = 1;
  BlacklistEntry hashes[MAX_BLACKLIST_ENTRIES] = {};
  ChanNameFilter filter = filter_for(0xA1);
  Packet pkt = group_packet(0xA1, CIPHER_BLOCK_SIZE);

  EXPECT_TRUE(isChanFiltered(&pkt, hashes, &filter, 1, fake_verify));
  EXPECT_EQ(1, verify_calls);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
