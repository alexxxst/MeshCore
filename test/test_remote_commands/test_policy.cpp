#include <gtest/gtest.h>

#include <cstring>

#include "RemoteCommandPolicy.h"

namespace {

bool normalize(const char* input, std::string& out) {
  char buf[256];
  std::strncpy(buf, input, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  if (!remoteCmdNormalize(buf, sizeof(buf))) return false;
  out = buf;
  return true;
}

}  // namespace

TEST(RemoteCommandPolicy, BannedPrefixesAreRaw) {
  EXPECT_TRUE(remoteCmdIsBanned("poweroff"));
  EXPECT_TRUE(remoteCmdIsBanned("poweroffblabla"));
  EXPECT_TRUE(remoteCmdIsBanned("shutdownblabla"));
  EXPECT_TRUE(remoteCmdIsBanned("advert"));
  EXPECT_TRUE(remoteCmdIsBanned("advertblabla"));
  EXPECT_TRUE(remoteCmdIsBanned("start otablabla"));
  EXPECT_TRUE(remoteCmdIsBanned("eraseblabla"));
  EXPECT_TRUE(remoteCmdIsBanned("clear statsblabla"));
  EXPECT_TRUE(remoteCmdIsBanned("gps setlocblabla"));
  EXPECT_TRUE(remoteCmdIsBanned("gps advertblabla"));
  EXPECT_TRUE(remoteCmdIsBanned("logblabla"));
  EXPECT_TRUE(remoteCmdIsBanned("get prv.keyblabla"));
  EXPECT_TRUE(remoteCmdIsBanned("set bridge.enabled off"));
  EXPECT_TRUE(remoteCmdIsBanned("set freq 433"));
  EXPECT_TRUE(remoteCmdIsBanned("set reboot.interval 24"));
  EXPECT_TRUE(remoteCmdIsBanned("region load"));
}

TEST(RemoteCommandPolicy, AdvertZerohopIsTheOnlyException) {
  EXPECT_FALSE(remoteCmdIsBanned("advert.zerohop"));
  EXPECT_FALSE(remoteCmdIsBanned("advert.zerohop extra"));
  EXPECT_TRUE(remoteCmdIsBanned("advert.zerohopX"));
}

TEST(RemoteCommandPolicy, OtherCommandsAreAllowed) {
  EXPECT_FALSE(remoteCmdIsBanned("reboot"));
  EXPECT_FALSE(remoteCmdIsBanned("set radio 869.618,62.5,8,5"));
  EXPECT_FALSE(remoteCmdIsBanned("set tx 14"));
  EXPECT_FALSE(remoteCmdIsBanned("tempradio 869.618,62.5,8,5,10"));
  EXPECT_FALSE(remoteCmdIsBanned("set guest.password "));
  EXPECT_FALSE(remoteCmdIsBanned("get guest.password"));
}

TEST(RemoteCommandPolicy, TimedCommandsUseWordBoundaries) {
  EXPECT_TRUE(remoteCmdIsTimedRadio("set radio 869.618,62.5,8,5"));
  EXPECT_FALSE(remoteCmdIsTimedRadio("set radio.rxgain on"));
  EXPECT_FALSE(remoteCmdIsTimedRadio("set radioblabla"));
  EXPECT_TRUE(remoteCmdIsTimedTx("set tx 14"));
  EXPECT_FALSE(remoteCmdIsTimedTx("set txblabla"));
}

TEST(RemoteCommandPolicy, RadioChangingCoversAllRadioSetters) {
  EXPECT_TRUE(remoteCmdIsRadioChanging("set radio 869.618,62.5,8,5"));
  EXPECT_TRUE(remoteCmdIsRadioChanging("set radio.rxgain on"));
  EXPECT_TRUE(remoteCmdIsRadioChanging("set radio.rxps max"));
  EXPECT_TRUE(remoteCmdIsRadioChanging("set radio.fem.txgain off"));
  EXPECT_TRUE(remoteCmdIsRadioChanging("tempradio 869.618,62.5,8,5,10"));
  EXPECT_TRUE(remoteCmdIsRadioChanging("set tx 14"));
  EXPECT_TRUE(remoteCmdIsRadioChanging("set freq 869"));
  EXPECT_FALSE(remoteCmdIsRadioChanging("set cad on"));
  EXPECT_FALSE(remoteCmdIsRadioChanging("set agc.reset.interval 240"));
}

TEST(RemoteCommandPolicy, NormalizesOptionalCompanionPrefixAndSpaces) {
  std::string out;
  ASSERT_TRUE(normalize("   xx|poweroff", out));
  EXPECT_EQ("poweroff", out);

  ASSERT_TRUE(normalize("  set name foo  ", out));
  EXPECT_EQ("set name foo", out);
}

TEST(RemoteCommandPolicy, StripsEveryCompanionPrefix) {
  std::string out;
  ASSERT_TRUE(normalize("aa|bb|poweroff", out));
  EXPECT_EQ("poweroff", out);
  EXPECT_TRUE(remoteCmdIsBanned(out.c_str()));

  ASSERT_TRUE(normalize("aa|bb|set radio 869.618,62.5,8,5", out));
  EXPECT_EQ("set radio 869.618,62.5,8,5", out);
  EXPECT_TRUE(remoteCmdIsTimedRadio(out.c_str()));

  ASSERT_TRUE(normalize("aa|bb|tempradio 869.618,62.5,8,5,10", out));
  EXPECT_TRUE(remoteCmdIsRadioChanging(out.c_str()));
}

TEST(RemoteCommandPolicy, PreservesOneTrailingSpaceForEmptyGuestPassword) {
  std::string out;
  ASSERT_TRUE(normalize("set guest.password ", out));
  EXPECT_EQ("set guest.password ", out);

  ASSERT_TRUE(normalize("set guest.password    ", out));
  EXPECT_EQ("set guest.password ", out);

  ASSERT_TRUE(normalize("set guest.password secret   ", out));
  EXPECT_EQ("set guest.password secret", out);

  ASSERT_TRUE(normalize("xx|set guest.password ", out));
  EXPECT_EQ("set guest.password ", out);

  ASSERT_TRUE(normalize("set guest.password", out));
  EXPECT_EQ("set guest.password", out);
}

TEST(RemoteCommandPolicy, ParsesRadioAndTxCommands) {
  float freq = 0, bw = 0;
  uint8_t sf = 0, cr = 0;
  ASSERT_TRUE(remoteCmdParseSetRadio("set radio 869.618,62.5,8,5", freq, bw, sf, cr));
  EXPECT_FLOAT_EQ(869.618f, freq);
  EXPECT_FLOAT_EQ(62.5f, bw);
  EXPECT_EQ(8, sf);
  EXPECT_EQ(5, cr);

  EXPECT_FALSE(remoteCmdParseSetRadio("set radio 433,62.5,7", freq, bw, sf, cr));
  EXPECT_FALSE(remoteCmdParseSetRadio("set radio 869.618,62.5,4,5", freq, bw, sf, cr));

  int8_t dbm = 0;
  ASSERT_TRUE(remoteCmdParseSetTx("set tx -9", dbm));
  EXPECT_EQ(-9, dbm);
  ASSERT_TRUE(remoteCmdParseSetTx("set tx 22", dbm));
  EXPECT_EQ(22, dbm);
  EXPECT_FALSE(remoteCmdParseSetTx("set tx -10", dbm));
  EXPECT_FALSE(remoteCmdParseSetTx("set tx 23", dbm));
  EXPECT_FALSE(remoteCmdParseSetTx("set tx nope", dbm));

  int timeout = 0;
  ASSERT_TRUE(remoteCmdParseTempRadio("tempradio 869.618,62.5,8,5,10", freq, bw, sf, cr, timeout));
  EXPECT_FLOAT_EQ(869.618f, freq);
  EXPECT_EQ(10, timeout);
  EXPECT_FALSE(remoteCmdParseTempRadio("tempradio 869.618,62.5,8,5,0", freq, bw, sf, cr, timeout));
  EXPECT_FALSE(remoteCmdParseTempRadio("tempradio 869.618,62.5,8,5,1441", freq, bw, sf, cr, timeout));
  EXPECT_FALSE(remoteCmdParseTempRadio("tempradio 433,62.5,7,5,10x", freq, bw, sf, cr, timeout));
  EXPECT_FALSE(remoteCmdParseTempRadio("tempradio 433,62.5,7,5,999,extra", freq, bw, sf, cr, timeout));
}

TEST(RemoteRadioBands, SameBandOnly) {
  EXPECT_TRUE(remoteCmdSameBand(868.856f, 869.618f, 62.5f));
  EXPECT_TRUE(remoteCmdSameBand(433.0f, 434.0f, 62.5f));
  EXPECT_TRUE(remoteCmdSameBand(915.0f, 923.0f, 62.5f));
  EXPECT_TRUE(remoteCmdSameBand(2400.0f, 2480.0f, 62.5f));

  EXPECT_FALSE(remoteCmdSameBand(868.856f, 433.0f, 62.5f));
  EXPECT_FALSE(remoteCmdSameBand(433.0f, 868.856f, 62.5f));
  EXPECT_FALSE(remoteCmdSameBand(868.856f, 915.0f, 62.5f));
  EXPECT_FALSE(remoteCmdSameBand(433.0f, 470.0f, 62.5f));
  EXPECT_FALSE(remoteCmdSameBand(470.0f, 480.0f, 62.5f));
  EXPECT_FALSE(remoteCmdSameBand(315.0f, 316.0f, 62.5f));
}

TEST(RemoteRadioBands, ChecksOccupiedBandwidthEdges) {
  EXPECT_TRUE(remoteCmdSameBand(869.618f, 869.618f, 62.5f));
  EXPECT_FALSE(remoteCmdSameBand(869.618f, 869.8f, 500.0f));
  EXPECT_TRUE(remoteCmdSameBand(869.618f, 869.8f, 62.5f));
}
