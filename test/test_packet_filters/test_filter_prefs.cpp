#include <gtest/gtest.h>

#include <cstring>

#include "FilterPrefs.h"

TEST(FilterPrefs, DefaultsAreJailOff) {
  FilterPrefs prefs;

  EXPECT_EQ(0, prefs.advert_jail_hours);
}

TEST(FilterPrefs, RoundTripsJailHours) {
  FilterPrefs prefs;
  prefs.advert_jail_hours = 24;

  uint8_t buf[FilterPrefs::MAX_SERIALIZED];
  const size_t len = prefs.serialize(buf);
  ASSERT_EQ(FilterPrefs::MAX_SERIALIZED, len);

  FilterPrefs loaded;
  loaded.advert_jail_hours = 7;
  ASSERT_TRUE(loaded.deserialize(buf, len));
  EXPECT_EQ(24, loaded.advert_jail_hours);
}

TEST(FilterPrefs, RejectsBadMagicVersionOrShortBuffer) {
  uint8_t buf[FilterPrefs::MAX_SERIALIZED];
  FilterPrefs prefs;
  const size_t len = prefs.serialize(buf);

  uint8_t bad_magic[FilterPrefs::MAX_SERIALIZED];
  memcpy(bad_magic, buf, len);
  bad_magic[0] = 'X';
  FilterPrefs loaded;
  EXPECT_FALSE(loaded.deserialize(bad_magic, len));

  uint8_t bad_version[FilterPrefs::MAX_SERIALIZED];
  memcpy(bad_version, buf, len);
  bad_version[4] = FilterPrefs::FILE_VERSION + 1;
  EXPECT_FALSE(loaded.deserialize(bad_version, len));

  EXPECT_FALSE(loaded.deserialize(buf, FilterPrefs::HEADER_SIZE));
  EXPECT_FALSE(loaded.deserialize(buf, 0));
  EXPECT_FALSE(loaded.deserialize(nullptr, len));
}

TEST(FilterPrefs, ClampsJailHoursToTheCliMaximum) {
  FilterPrefs prefs;
  prefs.advert_jail_hours = 200;

  uint8_t buf[FilterPrefs::MAX_SERIALIZED];
  const size_t len = prefs.serialize(buf);

  FilterPrefs loaded;
  ASSERT_TRUE(loaded.deserialize(buf, len));
  EXPECT_EQ(168, loaded.advert_jail_hours);
}

TEST(FilterPrefs, IgnoresExtraPayloadBytesFromANewerWriter) {
  uint8_t buf[FilterPrefs::MAX_SERIALIZED + 4];
  FilterPrefs prefs;
  prefs.advert_jail_hours = 12;
  const size_t len = prefs.serialize(buf);

  buf[5] += 4;   // a newer writer appended four bytes
  memset(buf + len, 0xaa, 4);

  FilterPrefs loaded;
  ASSERT_TRUE(loaded.deserialize(buf, len + 4));
  EXPECT_EQ(12, loaded.advert_jail_hours);
}
