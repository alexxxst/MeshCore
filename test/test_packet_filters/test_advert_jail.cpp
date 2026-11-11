#include <gtest/gtest.h>
#include <MeshCore.h>
#include <string.h>
#include "AdvertJail.h"

using namespace mesh;

static const uint32_t HOUR_MS = 3600UL * 1000UL;

static void make_key(uint8_t* key, uint8_t seed) {
  for (int i = 0; i < PUB_KEY_SIZE; i++) key[i] = (uint8_t)(seed + i);
}

// Sends 'n' adverts 'gap_ms' apart, each with a new sender timestamp; returns
// how many were dropped.
static int send_adverts(AdvertJailEntry* table, int size, const uint8_t* key, int n,
                        uint32_t& now, uint32_t& ts, uint32_t gap_ms, uint32_t interval) {
  int dropped = 0;
  for (int i = 0; i < n; i++) {
    now += gap_ms;
    ts += gap_ms / 1000 + 1;
    if (checkAdvertJail(table, size, key, ts, now, interval)) dropped++;
  }
  return dropped;
}

TEST(AdvertJail, SeventhEarlyAdvertIsTheFirstDropped) {
  AdvertJailEntry table[4] = {};
  uint8_t key[PUB_KEY_SIZE];
  make_key(key, 1);
  uint32_t now = 1000, ts = 100;

  // The first advert opens an entry, the next six are strikes; the sixth strike jails.
  EXPECT_EQ(0, send_adverts(table, 4, key, 6, now, ts, 60 * 1000, 12 * HOUR_MS));
  EXPECT_EQ(0, table[0].jailed);
  EXPECT_EQ(1, send_adverts(table, 4, key, 1, now, ts, 60 * 1000, 12 * HOUR_MS));
  EXPECT_EQ(1, table[0].jailed);
  EXPECT_EQ(ADVERT_JAIL_THRESHOLD, table[0].strikes);
  // While jailed, early adverts keep being dropped.
  EXPECT_EQ(3, send_adverts(table, 4, key, 3, now, ts, 60 * 1000, 12 * HOUR_MS));
}

TEST(AdvertJail, SameAdvertAgainIsNotAStrike) {
  AdvertJailEntry table[4] = {};
  uint8_t key[PUB_KEY_SIZE];
  make_key(key, 1);

  EXPECT_FALSE(checkAdvertJail(table, 4, key, 500, 1000, 12 * HOUR_MS));
  for (int i = 0; i < 10; i++) {
    EXPECT_FALSE(checkAdvertJail(table, 4, key, 500, 2000 + i * 1000, 12 * HOUR_MS));
  }

  EXPECT_EQ(0, table[0].strikes);
  EXPECT_EQ(1, table[0].adverts);
}

TEST(AdvertJail, AdvertsThatKeepTheIntervalAreNeverStrikes) {
  AdvertJailEntry table[4] = {};
  uint8_t key[PUB_KEY_SIZE];
  make_key(key, 1);
  uint32_t now = 1000, ts = 100;

  EXPECT_EQ(0, send_adverts(table, 4, key, 20, now, ts, 13 * HOUR_MS, 12 * HOUR_MS));
  EXPECT_EQ(0, table[0].strikes);
}

TEST(AdvertJail, JailedSenderGetsOutAfterWellSpacedAdverts) {
  AdvertJailEntry table[4] = {};
  uint8_t key[PUB_KEY_SIZE];
  make_key(key, 1);
  uint32_t now = 1000, ts = 100;

  send_adverts(table, 4, key, 7, now, ts, 60 * 1000, HOUR_MS);
  ASSERT_EQ(1, table[0].jailed);
  // A jailed sender's on-time adverts are forwarded and take strikes off: 6 -> 5 -> 4 -> 3 -> 2 -> 1.
  EXPECT_EQ(0, send_adverts(table, 4, key, 4, now, ts, 2 * HOUR_MS, HOUR_MS));
  EXPECT_EQ(1, table[0].jailed);
  EXPECT_EQ(0, send_adverts(table, 4, key, 1, now, ts, 2 * HOUR_MS, HOUR_MS));
  EXPECT_EQ(0, table[0].jailed);
  // An early advert right after release is a strike, not a drop.
  EXPECT_EQ(0, send_adverts(table, 4, key, 1, now, ts, 60 * 1000, HOUR_MS));
}

TEST(AdvertJail, SendersAreTrackedSeparately) {
  AdvertJailEntry table[4] = {};
  uint8_t spammer[PUB_KEY_SIZE], other[PUB_KEY_SIZE];
  make_key(spammer, 1);
  make_key(other, 100);
  uint32_t now = 1000, ts = 100;

  send_adverts(table, 4, spammer, 7, now, ts, 60 * 1000, HOUR_MS);

  uint32_t other_now = now, other_ts = 5000;
  EXPECT_EQ(0, send_adverts(table, 4, other, 3, other_now, other_ts, 60 * 1000, HOUR_MS));
}

TEST(AdvertJail, FullTableEvictsTheFewestStrikesAndKeepsTheJailed) {
  AdvertJailEntry table[2] = {};
  uint8_t spammer[PUB_KEY_SIZE], quiet[PUB_KEY_SIZE], newcomer[PUB_KEY_SIZE];
  make_key(spammer, 1);
  make_key(quiet, 50);
  make_key(newcomer, 99);
  uint32_t now = 1000, ts = 100;

  send_adverts(table, 2, spammer, 7, now, ts, 60 * 1000, HOUR_MS);
  send_adverts(table, 2, quiet, 1, now, ts, 60 * 1000, HOUR_MS);
  send_adverts(table, 2, newcomer, 1, now, ts, 60 * 1000, HOUR_MS);

  // The quiet sender (0 strikes) made room; the jailed spammer is still there.
  bool spammer_kept = false, quiet_kept = false;
  for (int i = 0; i < 2; i++) {
    if (memcmp(table[i].key, spammer, ADVERT_JAIL_KEY_SIZE) == 0) spammer_kept = table[i].jailed;
    if (memcmp(table[i].key, quiet, ADVERT_JAIL_KEY_SIZE) == 0) quiet_kept = true;
  }
  EXPECT_TRUE(spammer_kept);
  EXPECT_FALSE(quiet_kept);
}

TEST(AdvertJail, MillisWrapAroundStillMeasuresTheGap) {
  AdvertJailEntry table[2] = {};
  uint8_t key[PUB_KEY_SIZE];
  make_key(key, 1);
  uint32_t now = 0xFFFFFFFFUL - 30 * 1000, ts = 100;

  send_adverts(table, 2, key, 1, now, ts, 0, HOUR_MS);
  // The clock wraps between adverts that are two hours apart: still on time, no strike.
  EXPECT_EQ(0, send_adverts(table, 2, key, 1, now, ts, 2 * HOUR_MS, HOUR_MS));
  EXPECT_EQ(0, table[0].strikes);
}
