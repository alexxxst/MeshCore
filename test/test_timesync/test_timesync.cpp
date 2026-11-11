#include <gtest/gtest.h>
#include <stdio.h>
#include <string.h>
#include <TimeSync.h>
#include <TimeSyncDefaults.h>

static void makeKey(char out[65], uint8_t first) {
  static const char* hex = "0123456789abcdef";
  for (int i = 0; i < 64; i += 2) {
    out[i] = hex[(first >> 4) & 0x0F];
    out[i + 1] = hex[first & 0x0F];
  }
  out[64] = 0;
}

TEST(TimeSyncCsv, ParsesFullKeysAndSpaces) {
  char a[65], b[65];
  makeKey(a, 0xA1);
  makeKey(b, 0xB2);
  char csv[140];
  snprintf(csv, sizeof(csv), " %s , %s ", a, b);

  TimeSyncList list;
  int invalid = 0;
  EXPECT_EQ(2, list.setFromCsv(csv, &invalid));
  EXPECT_EQ(0, invalid);
  EXPECT_EQ(2, list.count());
  EXPECT_EQ(0xA1, list.get(0)[0]);
  EXPECT_EQ(0xB2, list.get(1)[0]);
}

TEST(TimeSyncCsv, DropsDuplicatesAndInvalidEntries) {
  char a[65];
  makeKey(a, 0xA1);
  char csv[180];
  snprintf(csv, sizeof(csv), "%s,%s,zz,%s", a, "abcd", a);

  TimeSyncList list;
  int invalid = 0;
  EXPECT_EQ(1, list.setFromCsv(csv, &invalid));
  EXPECT_EQ(2, invalid);
  EXPECT_EQ(1, list.count());
}

TEST(TimeSyncCsv, CapsAtEightKeys) {
  char csv[700] = {0};
  char key[65];
  for (int i = 0; i < 9; i++) {
    makeKey(key, (uint8_t)(0x10 + i));
    if (i) strcat(csv, ",");
    strcat(csv, key);
  }

  TimeSyncList list;
  int invalid = 0;
  EXPECT_EQ(8, list.setFromCsv(csv, &invalid));
  EXPECT_EQ(1, invalid);
  EXPECT_EQ(8, list.count());
}

TEST(TimeSyncCsv, EmptyInputClearsList) {
  char a[65];
  makeKey(a, 0xA1);
  TimeSyncList list;
  ASSERT_EQ(1, list.setFromCsv(a));
  int invalid = 0;
  EXPECT_EQ(0, list.setFromCsv("", &invalid));
  EXPECT_EQ(0, invalid);
  EXPECT_EQ(0, list.count());
}

TEST(TimeSyncCsv, RepositoryDefaultsParse) {
  TimeSyncList list;
  int invalid = 0;
  EXPECT_EQ(6, list.setFromCsv(TIMESYNC_DEFAULT_KEYS, &invalid));
  EXPECT_EQ(0, invalid);
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

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
