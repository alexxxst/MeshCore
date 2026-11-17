#include <gtest/gtest.h>

#include <cstring>

#include "RemoteCommandDictionary.h"

TEST(RemoteCommandDictionary, PinsTheUpstreamVersion) {
  EXPECT_EQ(0xD55C, REMOTE_CMD_DICT_ID);
  EXPECT_EQ(157, REMOTE_CMD_DICT_COUNT);
}

TEST(RemoteCommandDictionary, ExposesTokensByCode) {
  EXPECT_STREQ("reboot", remoteCmdDictToken(0));
  EXPECT_STREQ("advert.zerohop", remoteCmdDictToken(2));
  EXPECT_STREQ("set radio", remoteCmdDictToken(15));
  EXPECT_STREQ("set tx", remoteCmdDictToken(16));
  EXPECT_STREQ("tempradio", remoteCmdDictToken(17));
  EXPECT_STREQ("set guest.password", remoteCmdDictToken(24));
  EXPECT_STREQ("region remove", remoteCmdDictToken(53));
  EXPECT_STREQ("\r", remoteCmdDictToken(156));
}

TEST(RemoteCommandDictionary, KeepsThePackedBlobContiguous) {
  for (uint8_t i = 0; i < REMOTE_CMD_DICT_COUNT; ++i) {
    const char* token = remoteCmdDictToken(i);
    ASSERT_NE(nullptr, token);
    EXPECT_GT(std::strlen(token), 0u);
    if (i + 1 < REMOTE_CMD_DICT_COUNT) {
      EXPECT_LT(REMOTE_CMD_DICT_OFFSETS[i], REMOTE_CMD_DICT_OFFSETS[i + 1]);
    }
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
