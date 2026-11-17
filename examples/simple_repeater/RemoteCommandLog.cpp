#include "RemoteCommandLog.h"

#include <string.h>

namespace {
const uint8_t LOG_MAGIC[4] = {'R', 'C', 'L', '1'};
const uint8_t LOG_VERSION = 1;
}  // namespace

RemoteCommandLog::RemoteCommandLog() {
  clear();
}

uint32_t RemoteCommandLog::readLe32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

void RemoteCommandLog::writeLe32(uint8_t* p, uint32_t value) {
  p[0] = (uint8_t)(value & 0xFF);
  p[1] = (uint8_t)((value >> 8) & 0xFF);
  p[2] = (uint8_t)((value >> 16) & 0xFF);
  p[3] = (uint8_t)((value >> 24) & 0xFF);
}

uint8_t* RemoteCommandLog::entryAt(int idx) {
  return _data + HEADER_SIZE + (size_t)idx * ENTRY_SIZE;
}

const uint8_t* RemoteCommandLog::entryAt(int idx) const {
  return _data + HEADER_SIZE + (size_t)idx * ENTRY_SIZE;
}

void RemoteCommandLog::clear() {
  memset(_data, 0, sizeof(_data));
  memcpy(_data, LOG_MAGIC, sizeof(LOG_MAGIC));
  _data[4] = LOG_VERSION;
  writeLe32(&_data[9], 1);   // next_seq
}

bool RemoteCommandLog::isValid() const {
  if (memcmp(_data, LOG_MAGIC, sizeof(LOG_MAGIC)) != 0) return false;
  if (_data[4] != LOG_VERSION) return false;
  if (count() > REMOTE_CMD_LOG_ENTRIES) return false;
  for (int i = 0; i < count(); ++i) {
    const uint8_t* e = entryAt(i);
    if (e[8] > REMOTE_CMD_LOG_FAIL) return false;
    if (e[9] > REMOTE_CMD_MAX_LINE) return false;
  }
  return true;
}

bool RemoteCommandLog::deserialize(const uint8_t* data, size_t len) {
  if (data == nullptr || len < HEADER_SIZE) return false;
  if (memcmp(data, LOG_MAGIC, sizeof(LOG_MAGIC)) != 0) return false;
  if (data[4] != LOG_VERSION) return false;

  bool in_place = (data == _data);
  size_t copy = len < SERIALIZED_SIZE ? len : SERIALIZED_SIZE;
  if (!in_place) {
    clear();
    memcpy(_data, data, copy);
  } else if (copy < SERIALIZED_SIZE) {
    memset(_data + copy, 0, SERIALIZED_SIZE - copy);
  }

  if (!isValid()) {
    clear();
    return false;
  }

  if (readLe32(&_data[9]) == 0) writeLe32(&_data[9], 1);

  uint32_t last = readLe32(&_data[5]);
  for (int i = 0; i < count(); ++i) {
    uint32_t ts = readLe32(entryAt(i) + 4);
    if (ts > last) last = ts;
  }
  writeLe32(&_data[5], last);
  return true;
}

bool RemoteCommandLog::append(uint32_t timestamp, const char* command,
                              uint8_t status, uint32_t& seq_out) {
  uint32_t seq = readLe32(&_data[9]);
  if (!appendRecord(seq, timestamp, status, command,
                    command ? (uint8_t)strlen(command) : 0)) {
    return false;
  }
  seq_out = seq;
  return true;
}

bool RemoteCommandLog::appendRecord(uint32_t seq, uint32_t timestamp,
                                    uint8_t status, const char* command,
                                    uint8_t len) {
  if (len > REMOTE_CMD_MAX_LINE) return false;
  if (command == nullptr && len > 0) return false;
  if (status > REMOTE_CMD_LOG_FAIL) return false;

  int n = count();
  int idx = n;
  if (n >= REMOTE_CMD_LOG_ENTRIES) {
    idx = 0;
    while (idx < n && entryAt(idx)[8] == REMOTE_CMD_LOG_PENDING) ++idx;
    if (idx >= n) idx = 0;   // all pending: evict the oldest anyway

    if (idx + 1 < n) {
      memmove(entryAt(idx), entryAt(idx + 1),
              (size_t)(n - idx - 1) * ENTRY_SIZE);
    }
    idx = n - 1;
  }

  uint8_t* e = entryAt(idx);
  writeLe32(e, seq);
  writeLe32(e + 4, timestamp);
  e[8] = status;
  e[9] = len;
  if (len > 0) memcpy(e + 10, command, len);
  if (len < REMOTE_CMD_MAX_LINE) e[10 + len] = 0;

  if (n < REMOTE_CMD_LOG_ENTRIES) _data[14] = (uint8_t)(n + 1);

  uint32_t next = readLe32(&_data[9]);
  if (seq >= next) {
    next = seq + 1;
    if (next == 0) next = 1;
    writeLe32(&_data[9], next);
  }

  uint32_t last = readLe32(&_data[5]);
  if (timestamp > last) writeLe32(&_data[5], timestamp);
  return true;
}

bool RemoteCommandLog::remove(uint32_t seq) {
  int n = count();
  for (int i = 0; i < n; ++i) {
    if (readLe32(entryAt(i)) != seq) continue;
    if (i + 1 < n) {
      memmove(entryAt(i), entryAt(i + 1),
              (size_t)(n - i - 1) * ENTRY_SIZE);
    }
    _data[14] = (uint8_t)(n - 1);
    return true;
  }
  return false;
}

bool RemoteCommandLog::setStatus(uint32_t seq, uint8_t status) {
  if (status > REMOTE_CMD_LOG_FAIL) return false;
  for (int i = 0; i < count(); ++i) {
    uint8_t* e = entryAt(i);
    if (readLe32(e) == seq) {
      e[8] = status;
      return true;
    }
  }
  return false;
}

bool RemoteCommandLog::getNewest(int offset, RemoteCommandLogEntry& out) const {
  int n = count();
  if (offset < 0 || offset >= n) return false;

  const uint8_t* e = entryAt(n - 1 - offset);
  out.seq = readLe32(e);
  out.timestamp = readLe32(e + 4);
  out.status = e[8];
  out.len = e[9];
  if (out.len > REMOTE_CMD_MAX_LINE) out.len = REMOTE_CMD_MAX_LINE;
  memcpy(out.command, e + 10, out.len);
  out.command[out.len] = 0;
  return true;
}
