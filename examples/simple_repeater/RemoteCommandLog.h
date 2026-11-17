#pragma once

#include <stddef.h>
#include <stdint.h>

#include "RemoteCommandConfig.h"

enum RemoteCommandLogStatus : uint8_t {
  REMOTE_CMD_LOG_NONE = 0,
  REMOTE_CMD_LOG_PENDING = 1,
  REMOTE_CMD_LOG_OK = 2,
  REMOTE_CMD_LOG_FAIL = 3,
};

struct RemoteCommandLogEntry {
  uint32_t seq;
  uint32_t timestamp;
  uint8_t status;
  uint8_t len;
  char command[REMOTE_CMD_MAX_LINE + 1];
};

// Bounded log of the last REMOTE_CMD_LOG_ENTRIES commands, stored oldest-first
// in one serialized buffer. A linear array (not a ring) keeps pending entries
// protected from rotation without extra bookkeeping.
class RemoteCommandLog {
public:
  static constexpr size_t ENTRY_SIZE =
      4 + 4 + 1 + 1 + REMOTE_CMD_MAX_LINE;   // seq, ts, status, len, command
  static constexpr size_t HEADER_SIZE = 16;
  static constexpr size_t SERIALIZED_SIZE =
      HEADER_SIZE + REMOTE_CMD_LOG_ENTRIES * ENTRY_SIZE;

  RemoteCommandLog();

  void clear();
  bool deserialize(const uint8_t* data, size_t len);

  bool append(uint32_t timestamp, const char* command, uint8_t status,
              uint32_t& seq_out);
  bool appendRecord(uint32_t seq, uint32_t timestamp, uint8_t status,
                    const char* command, uint8_t len);
  bool remove(uint32_t seq);
  bool setStatus(uint32_t seq, uint8_t status);

  int count() const { return _data[14]; }
  uint32_t lastTimestamp() const { return readLe32(&_data[5]); }
  const uint8_t* data() const { return _data; }
  uint8_t* buffer() { return _data; }
  size_t size() const { return SERIALIZED_SIZE; }

  bool getNewest(int offset, RemoteCommandLogEntry& out) const;

private:
  uint8_t _data[SERIALIZED_SIZE];

  static uint32_t readLe32(const uint8_t* p);
  static void writeLe32(uint8_t* p, uint32_t value);
  uint8_t* entryAt(int idx);
  const uint8_t* entryAt(int idx) const;
  bool isValid() const;
};
