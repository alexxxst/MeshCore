#include <gtest/gtest.h>

#include <cstring>
#include <string>

#include "RemoteCommandLog.h"
#include "RemoteRadioGuard.h"

TEST(RemoteCommandLog, AppendsAndReadsNewestFirst) {
  RemoteCommandLog log;
  log.clear();

  uint32_t seq1 = 0, seq2 = 0;
  ASSERT_TRUE(log.append(100, "reboot", REMOTE_CMD_LOG_NONE, seq1));
  ASSERT_TRUE(log.append(101, "set tx 14", REMOTE_CMD_LOG_PENDING, seq2));

  EXPECT_EQ(2, log.count());
  EXPECT_EQ(101u, log.lastTimestamp());

  RemoteCommandLogEntry entry;
  ASSERT_TRUE(log.getNewest(0, entry));
  EXPECT_EQ(seq2, entry.seq);
  EXPECT_EQ(101u, entry.timestamp);
  EXPECT_EQ(REMOTE_CMD_LOG_PENDING, entry.status);
  EXPECT_STREQ("set tx 14", entry.command);

  ASSERT_TRUE(log.getNewest(1, entry));
  EXPECT_EQ(seq1, entry.seq);
  EXPECT_STREQ("reboot", entry.command);
}

TEST(RemoteCommandLog, KeepsOnlyThirtyEntries) {
  RemoteCommandLog log;
  log.clear();

  for (int i = 0; i < 35; ++i) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "cmd %d", i);
    uint32_t seq = 0;
    ASSERT_TRUE(log.append(1000 + i, cmd, REMOTE_CMD_LOG_NONE, seq));
  }

  EXPECT_EQ(REMOTE_CMD_LOG_ENTRIES, log.count());
  RemoteCommandLogEntry entry;
  ASSERT_TRUE(log.getNewest(0, entry));
  EXPECT_STREQ("cmd 34", entry.command);
  ASSERT_TRUE(log.getNewest(REMOTE_CMD_LOG_ENTRIES - 1, entry));
  EXPECT_STREQ("cmd 5", entry.command);
  EXPECT_FALSE(log.getNewest(REMOTE_CMD_LOG_ENTRIES, entry));
}

TEST(RemoteCommandLog, ProtectsPendingEntryFromRotation) {
  RemoteCommandLog log;
  log.clear();

  uint32_t pending_seq = 0;
  ASSERT_TRUE(log.append(1, "set radio 869.618,62.5,8,5", REMOTE_CMD_LOG_PENDING, pending_seq));

  for (int i = 0; i < REMOTE_CMD_LOG_ENTRIES; ++i) {
    uint32_t seq = 0;
    ASSERT_TRUE(log.append(100 + i, "reboot", REMOTE_CMD_LOG_NONE, seq));
  }

  EXPECT_EQ(REMOTE_CMD_LOG_ENTRIES, log.count());
  RemoteCommandLogEntry entry;
  bool found = false;
  for (int i = 0; i < log.count(); ++i) {
    ASSERT_TRUE(log.getNewest(i, entry));
    if (entry.seq == pending_seq) {
      found = true;
      EXPECT_EQ(REMOTE_CMD_LOG_PENDING, entry.status);
      break;
    }
  }
  EXPECT_TRUE(found);
}

TEST(RemoteCommandLog, AppendRecordPreservesJournalSequence) {
  RemoteCommandLog log;
  log.clear();

  ASSERT_TRUE(log.appendRecord(42, 1700000000u, REMOTE_CMD_LOG_NONE, "reboot", 6));
  RemoteCommandLogEntry entry;
  ASSERT_TRUE(log.getNewest(0, entry));
  EXPECT_EQ(42u, entry.seq);

  uint32_t seq = 0;
  ASSERT_TRUE(log.append(1700000001u, "set tx 10", REMOTE_CMD_LOG_NONE, seq));
  EXPECT_EQ(43u, seq);

  EXPECT_TRUE(log.remove(42));
  EXPECT_EQ(1, log.count());
  EXPECT_FALSE(log.remove(42));
}

TEST(RemoteCommandLog, UpdatesStatusBySequence) {
  RemoteCommandLog log;
  log.clear();

  uint32_t seq = 0;
  ASSERT_TRUE(log.append(7, "set tx 10", REMOTE_CMD_LOG_PENDING, seq));
  ASSERT_TRUE(log.setStatus(seq, REMOTE_CMD_LOG_OK));

  RemoteCommandLogEntry entry;
  ASSERT_TRUE(log.getNewest(0, entry));
  EXPECT_EQ(REMOTE_CMD_LOG_OK, entry.status);
  EXPECT_FALSE(log.setStatus(seq + 1, REMOTE_CMD_LOG_FAIL));
}

TEST(RemoteCommandLog, RoundTripsThroughItsSerializedBuffer) {
  RemoteCommandLog log;
  log.clear();
  uint32_t seq = 0;
  ASSERT_TRUE(log.append(1700000000u, "set guest.password ", REMOTE_CMD_LOG_NONE, seq));
  ASSERT_TRUE(log.append(1700000001u, "reboot", REMOTE_CMD_LOG_FAIL, seq));

  RemoteCommandLog restored;
  ASSERT_TRUE(restored.deserialize(log.data(), log.size()));
  EXPECT_EQ(2, restored.count());
  EXPECT_EQ(1700000001u, restored.lastTimestamp());

  RemoteCommandLogEntry entry;
  ASSERT_TRUE(restored.getNewest(0, entry));
  EXPECT_STREQ("reboot", entry.command);
  EXPECT_EQ(REMOTE_CMD_LOG_FAIL, entry.status);
}

TEST(RemoteCommandLog, DeserializesInPlaceAfterAFileRead) {
  RemoteCommandLog source;
  source.clear();
  uint32_t seq = 0;
  ASSERT_TRUE(source.append(1700000000u, "reboot", REMOTE_CMD_LOG_NONE, seq));

  RemoteCommandLog restored;
  memcpy(restored.buffer(), source.data(), source.size());
  ASSERT_TRUE(restored.deserialize(restored.buffer(), source.size()));
  EXPECT_EQ(1, restored.count());

  RemoteCommandLogEntry entry;
  ASSERT_TRUE(restored.getNewest(0, entry));
  EXPECT_STREQ("reboot", entry.command);
}

TEST(RemoteCommandLog, RejectsBadOrShortBuffers) {
  RemoteCommandLog log;
  log.clear();
  uint32_t seq = 0;
  ASSERT_TRUE(log.append(1, "reboot", REMOTE_CMD_LOG_NONE, seq));

  uint8_t bad[RemoteCommandLog::SERIALIZED_SIZE];
  memcpy(bad, log.data(), log.size());
  bad[0] = 'X';
  EXPECT_FALSE(log.deserialize(bad, log.size()));
  EXPECT_FALSE(log.deserialize(log.data(), RemoteCommandLog::HEADER_SIZE - 1));
}

TEST(RemoteRadioGuard, RoundTripsOldRadioParameters) {
  RemoteRadioGuardState state;
  state.armed = true;
  state.revert_pending = true;
  state.attempts = 2;
  state.freq = 868.856f;
  state.bw = 62.5f;
  state.sf = 7;
  state.cr = 5;
  state.pending_timestamp = 1234;
  state.pending_seq = 99;
  state.target_freq = 869.618f;
  state.target_bw = 125.0f;
  state.target_sf = 9;
  state.target_cr = 6;

  uint8_t buf[RemoteRadioGuardState::SERIALIZED_SIZE];
  ASSERT_EQ(RemoteRadioGuardState::SERIALIZED_SIZE, state.serialize(buf, sizeof(buf)));

  RemoteRadioGuardState restored;
  ASSERT_TRUE(restored.deserialize(buf, sizeof(buf)));
  EXPECT_TRUE(restored.armed);
  EXPECT_TRUE(restored.revert_pending);
  EXPECT_EQ(2, restored.attempts);
  EXPECT_FLOAT_EQ(868.856f, restored.freq);
  EXPECT_FLOAT_EQ(62.5f, restored.bw);
  EXPECT_EQ(7, restored.sf);
  EXPECT_EQ(5, restored.cr);
  EXPECT_EQ(1234u, restored.pending_timestamp);
  EXPECT_EQ(99u, restored.pending_seq);
  EXPECT_FLOAT_EQ(869.618f, restored.target_freq);
  EXPECT_FLOAT_EQ(125.0f, restored.target_bw);
  EXPECT_EQ(9, restored.target_sf);
  EXPECT_EQ(6, restored.target_cr);
}

TEST(RemoteRadioGuard, RejectsBadMagicVersionOrShortBuffer) {
  RemoteRadioGuardState state;
  state.armed = true;
  uint8_t buf[RemoteRadioGuardState::SERIALIZED_SIZE];
  ASSERT_EQ(RemoteRadioGuardState::SERIALIZED_SIZE, state.serialize(buf, sizeof(buf)));

  RemoteRadioGuardState restored;
  buf[0] = 'X';
  EXPECT_FALSE(restored.deserialize(buf, sizeof(buf)));

  state.serialize(buf, sizeof(buf));
  buf[4] = RemoteRadioGuardState::VERSION + 1;
  EXPECT_FALSE(restored.deserialize(buf, sizeof(buf)));

  EXPECT_FALSE(restored.deserialize(buf, RemoteRadioGuardState::SERIALIZED_SIZE - 1));
}
