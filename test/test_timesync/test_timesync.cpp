#include <gtest/gtest.h>
#include <TimeSync.h>
#include <TimeSyncDefaults.h>

TEST(TimeSyncDefaults, HeaderHasExpectedKeys) {
  EXPECT_EQ(6u, TIMESYNC_DEFAULT_KEY_COUNT);
  EXPECT_LE(TIMESYNC_DEFAULT_KEY_COUNT, (size_t)TIMESYNC_MAX_TRUSTED);
  EXPECT_EQ(0xab, TIMESYNC_DEFAULT_KEYS[0][0]);   // PRI-Meb-SE
  EXPECT_EQ(0x8c, TIMESYNC_DEFAULT_KEYS[5][0]);   // MeshBBS

  TimeSyncList list;
  for (size_t i = 0; i < TIMESYNC_DEFAULT_KEY_COUNT; i++) {
    EXPECT_EQ(TimeSyncList::ADDED, list.add(TIMESYNC_DEFAULT_KEYS[i]));
  }
  EXPECT_EQ(6, list.count());
}

TEST(TimeSyncCorrection, AddsAirtimeForSourceAndEveryHop) {
  EXPECT_EQ(1002u, TimeSyncList::computeTarget(1000, 2000, 0));
  EXPECT_EQ(1006u, TimeSyncList::computeTarget(1000, 2000, 2));
  EXPECT_EQ(1001u, TimeSyncList::computeTarget(1000, 200, 3));   // 800 ms rounds to 1 s
}

TEST(TimeSyncCorrection, CapsHopCount) {
  EXPECT_EQ(TimeSyncList::computeTarget(1000, 2000, TIMESYNC_MAX_COMP_HOPS),
            TimeSyncList::computeTarget(1000, 2000, 200));
}

TEST(TimeSyncCorrection, DeadbandAndDirection) {
  EXPECT_FALSE(TimeSyncList::shouldAdjust(1000, 1010, true));    // exactly 10 s
  EXPECT_TRUE(TimeSyncList::shouldAdjust(1000, 1011, true));
  EXPECT_FALSE(TimeSyncList::shouldAdjust(1000, 989, false));    // backward not allowed
  EXPECT_TRUE(TimeSyncList::shouldAdjust(1000, 989, true));
  EXPECT_FALSE(TimeSyncList::shouldAdjust(1000, 990, true));     // exactly 10 s back
}

TEST(TimeSyncReplay, RejectsRepeatedAndOlderTimestampsPerSource) {
  uint8_t a[32] = {0xA1};
  uint8_t b[32] = {0xB2};
  TimeSyncSources sources;

  EXPECT_TRUE(sources.accept(a, 100, 1000));
  EXPECT_FALSE(sources.accept(a, 100, 2000));
  EXPECT_FALSE(sources.accept(a, 99, 2000));
  EXPECT_TRUE(sources.accept(a, 101, 2000));
  EXPECT_TRUE(sources.accept(b, 50, 2000));   // independent high-water mark
}

TEST(TimeSyncReplay, EvictsOldestHeardSource) {
  TimeSyncSources sources;
  uint8_t key[32] = {0};
  for (int i = 0; i < TIMESYNC_MAX_SOURCES; i++) {
    key[0] = (uint8_t)(0x10 + i);
    ASSERT_TRUE(sources.accept(key, 100, 1000 + i));
  }
  key[0] = 0x99;
  ASSERT_TRUE(sources.accept(key, 100, 2000));   // evicts key 0x10

  key[0] = 0x10;
  EXPECT_TRUE(sources.accept(key, 101, 3000));   // its slot was forgotten
}

TEST(TimeSyncSanity, WindowIsFixedAtTheBottom) {
  EXPECT_EQ(1793491200u, TIMESYNC_MIN_EPOCH);
  EXPECT_GT(TIMESYNC_MAX_EPOCH, TIMESYNC_MIN_EPOCH);
}

TEST(TimeSyncFile, SavedEmptyListDeserializesEmpty) {
  const size_t header = TimeSyncList::HEADER_SIZE;
  TimeSyncList list;
  uint8_t buf[TimeSyncList::MAX_SERIALIZED];
  EXPECT_EQ(header, list.serialize(buf));

  TimeSyncList loaded;
  EXPECT_TRUE(loaded.deserialize(buf, header));
  EXPECT_EQ(0, loaded.count());
}

TEST(TimeSyncFile, TruncatedBufferIsRejected) {
  const size_t header = TimeSyncList::HEADER_SIZE;
  const uint8_t buf[] = { TimeSyncList::FILE_MAGIC, TimeSyncList::FILE_VERSION, 1 };
  TimeSyncList list;
  EXPECT_FALSE(list.deserialize(buf, header - 1));
  EXPECT_FALSE(list.deserialize(buf, header));
  EXPECT_EQ(0, list.count());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
