#include "RemoteCommandModule.h"

#include <Arduino.h>
#include <RTClib.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <Utils.h>

#include "ListReplyPager.h"
#include "RemoteCommandCodec.h"
#include "RemoteCommandKeys.h"
#include "RemoteCommandPolicy.h"

namespace {

const uint32_t QUEUE_FALLBACK_MS = 35000;
const uint32_t QUEUE_SETTLE_MS = 500;
const uint32_t COMMAND_GAP_MS = 1500;
const uint32_t TX_GUARD_DISCOVER_DELAY_MS = 10000;
const uint32_t TX_GUARD_WINDOW_MS = 60000;

File openRead(FILESYSTEM* fs, const char* name) {
#if defined(RP2040_PLATFORM)
  return fs->open(name, "r");
#else
  return fs->open(name);
#endif
}

File openWrite(FILESYSTEM* fs, const char* name) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  fs->remove(name);
  return fs->open(name, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  return fs->open(name, "w");
#else
  return fs->open(name, "w", true);
#endif
}

File openAppend(FILESYSTEM* fs, const char* name) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  return fs->open(name, FILE_O_WRITE);   // Adafruit LittleFS seeks to end
#elif defined(RP2040_PLATFORM)
  return fs->open(name, "a");
#else
  return fs->open(name, "a", true);
#endif
}

const uint8_t JOURNAL_MAGIC[5] = {'R', 'C', 'J', '1', 1};
const uint8_t JOURNAL_COMMAND = 1;
const uint8_t JOURNAL_STATUS = 2;
const uint16_t JOURNAL_COMPACT_AT = 60;

bool containsCI(const char* haystack, const char* needle) {
  if (haystack == nullptr || needle == nullptr) return false;
  size_t n = strlen(needle);
  for (const char* p = haystack; *p; ++p) {
    size_t i = 0;
    while (i < n && p[i] &&
           tolower((unsigned char)p[i]) == tolower((unsigned char)needle[i])) {
      ++i;
    }
    if (i == n) return true;
  }
  return false;
}

bool replyFailed(const char* reply) {
  if (reply == nullptr || reply[0] == 0) return false;
  if ((reply[0] == 'O' || reply[0] == 'o') &&
      (reply[1] == 'K' || reply[1] == 'k')) {
    return false;
  }
  if (reply[0] == '>') return false;
  return containsCI(reply, "err") || containsCI(reply, "unknown") ||
         containsCI(reply, "invalid") || containsCI(reply, "can't") ||
         containsCI(reply, "bad");
}

bool timeReached(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

uint32_t readLe32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

void writeLe32(uint8_t* p, uint32_t value) {
  p[0] = (uint8_t)(value & 0xFF);
  p[1] = (uint8_t)((value >> 8) & 0xFF);
  p[2] = (uint8_t)((value >> 16) & 0xFF);
  p[3] = (uint8_t)((value >> 24) & 0xFF);
}

const char* statusText(uint8_t status) {
  switch (status) {
    case REMOTE_CMD_LOG_PENDING: return "pending";
    case REMOTE_CMD_LOG_OK: return "ok";
    case REMOTE_CMD_LOG_FAIL: return "fail";
    default: return nullptr;
  }
}

}  // namespace

void RemoteCommandModule::begin(FILESYSTEM* fs, const RemoteCommandModuleContext& context) {
  _fs = fs;
  _ctx = context;
  _queue_count = 0;
  _executing = false;
  _cmd_count = 0;
  _cmd_index = 0;
  _tx_guard = TxGuard{};
  _guard.clear();
  _f1_timer_active = false;

  mesh::Utils::sha256(_channel_hash, sizeof(_channel_hash),
                      REMOTE_CMD_CHANNEL_KEY, REMOTE_CMD_CHANNEL_KEY_LEN);
  memset(_channel.hash, 0, sizeof(_channel.hash));
  memcpy(_channel.hash, _channel_hash, PATH_HASH_SIZE);
  memset(_channel.secret, 0, sizeof(_channel.secret));
  memcpy(_channel.secret, REMOTE_CMD_CHANNEL_KEY, REMOTE_CMD_CHANNEL_KEY_LEN);

  loadLog();
  uint32_t log_ts = _log.lastTimestamp();
  loadLastTimestamp();
  if (log_ts > _last_ts) _last_ts = log_ts;
  loadGuard();
  if (_guard.pending_timestamp > _last_ts) _last_ts = _guard.pending_timestamp;
  reconcilePending();

  if (_guard.armed) {
    if (_guard.revert_pending) {
      resolveGuard(false, true);
      if (!_guard.armed) return;
    }

    RemoteCommandRadioParams current = {};
    bool have_current = _ctx.get_radio && _ctx.get_radio(_ctx.ctx, &current);
    bool target_active = have_current &&
        fabsf(current.freq - _guard.target_freq) < 0.001f &&
        fabsf(current.bw - _guard.target_bw) < 0.01f &&
        current.sf == _guard.target_sf && current.cr == _guard.target_cr;

    if (!target_active) {
      // Power was lost between writing F1 and the CLI saving the new prefs.
      // The guard would test the old radio, so fail the command and restore
      // the known-good old parameters instead.
      if (_guard.pending_seq != 0) {
        persistStatus(_guard.pending_seq, REMOTE_CMD_LOG_FAIL);
      }
      bool reverted = true;
      if (have_current) {
        RemoteCommandRadioParams old = {};
        old.freq = _guard.freq;
        old.bw = _guard.bw;
        old.sf = _guard.sf;
        old.cr = _guard.cr;
        old.tx_power = current.tx_power;
        reverted = _ctx.apply_radio_and_save &&
                   _ctx.apply_radio_and_save(_ctx.ctx, old);
      }
      if (!reverted) {
        _guard.revert_pending = true;
        saveGuard();
        if (_ctx.reboot) _ctx.reboot(_ctx.ctx);
        return;
      }
      _guard.clear();
      removeGuard();
    } else if (_guard.attempts >= GUARD_MAX_ATTEMPTS) {
      resolveGuard(false, true);
    } else {
      _guard.attempts++;
      saveGuard();
      startGuardTimers(_ctx.get_millis ? _ctx.get_millis(_ctx.ctx) : millis());
    }
  }
}

bool RemoteCommandModule::ensureLogFile() {
  if (_fs == nullptr) return false;
  if (_fs->exists(F2_FILE)) return true;
  File file = openWrite(_fs, F2_FILE);
  if (!file) return false;
  size_t n = file.write(JOURNAL_MAGIC, sizeof(JOURNAL_MAGIC));
  file.close();
  _journal_records = 0;
  return n == sizeof(JOURNAL_MAGIC);
}

bool RemoteCommandModule::loadLog() {
  _log.clear();
  _journal_records = 0;
  if (_fs == nullptr) return false;
  if (!ensureLogFile()) return false;

  File file = openRead(_fs, F2_FILE);
  if (!file) return false;

  uint8_t magic[sizeof(JOURNAL_MAGIC)];
  if (file.read(magic, sizeof(magic)) != (int)sizeof(magic) ||
      memcmp(magic, JOURNAL_MAGIC, sizeof(magic)) != 0) {
    file.close();
    compactLog();
    return false;
  }

  bool truncated = false;
  uint8_t hdr[11];
  while (true) {
    int got = file.read(hdr, sizeof(hdr));
    if (got == 0) break;
    if (got != (int)sizeof(hdr)) {
      truncated = true;
      break;
    }

    uint8_t type = hdr[0];
    uint32_t seq = readLe32(hdr + 1);
    uint32_t ts = readLe32(hdr + 5);
    uint8_t status = hdr[9];
    uint8_t len = hdr[10];
    if (len > REMOTE_CMD_MAX_LINE) {
      truncated = true;
      break;
    }

    char command[REMOTE_CMD_MAX_LINE + 1];
    command[0] = 0;
    if (len > 0) {
      if (file.read((uint8_t*)command, len) != (int)len) {
        truncated = true;
        break;
      }
      command[len] = 0;
    }

    bool ok = false;
    if (type == JOURNAL_COMMAND) {
      ok = _log.appendRecord(seq, ts, status, command, len);
    } else if (type == JOURNAL_STATUS) {
      ok = _log.setStatus(seq, status);
    } else {
      truncated = true;
      break;
    }
    if (ok) ++_journal_records;
  }
  file.close();

  if (truncated || _journal_records > JOURNAL_COMPACT_AT) compactLog();
  return true;
}

bool RemoteCommandModule::saveLog() {
  return compactLog();
}

bool RemoteCommandModule::compactLog() {
  if (_fs == nullptr) return false;
  File file = openWrite(_fs, F2_FILE);
  if (!file) return false;

  bool ok = file.write(JOURNAL_MAGIC, sizeof(JOURNAL_MAGIC)) == sizeof(JOURNAL_MAGIC);
  RemoteCommandLogEntry entry;
  for (int i = _log.count() - 1; ok && i >= 0; --i) {
    if (!_log.getNewest(i, entry)) break;
    uint8_t hdr[11];
    hdr[0] = JOURNAL_COMMAND;
    writeLe32(hdr + 1, entry.seq);
    writeLe32(hdr + 5, entry.timestamp);
    hdr[9] = entry.status;
    hdr[10] = entry.len;
    ok = file.write(hdr, sizeof(hdr)) == sizeof(hdr);
    if (ok && entry.len > 0) {
      ok = file.write((const uint8_t*)entry.command, entry.len) == entry.len;
    }
  }
  file.close();
  if (ok) _journal_records = (uint16_t)_log.count();
  return ok;
}

bool RemoteCommandModule::appendJournalCommand(uint32_t seq, uint32_t timestamp,
                                               uint8_t status,
                                               const char* command, uint8_t len) {
  if (_fs == nullptr || command == nullptr || len > REMOTE_CMD_MAX_LINE) return false;
  if (!ensureLogFile()) return false;

  File file = openAppend(_fs, F2_FILE);
  if (!file) return false;
  uint8_t hdr[11];
  hdr[0] = JOURNAL_COMMAND;
  writeLe32(hdr + 1, seq);
  writeLe32(hdr + 5, timestamp);
  hdr[9] = status;
  hdr[10] = len;
  bool ok = file.write(hdr, sizeof(hdr)) == sizeof(hdr);
  if (ok && len > 0) ok = file.write((const uint8_t*)command, len) == len;
  file.close();

  if (!ok) return false;
  ++_journal_records;
  if (_journal_records > JOURNAL_COMPACT_AT) compactLog();
  return true;
}

bool RemoteCommandModule::appendJournalStatus(uint32_t seq, uint8_t status) {
  if (_fs == nullptr) return false;
  if (!ensureLogFile()) return false;

  File file = openAppend(_fs, F2_FILE);
  if (!file) return false;
  uint8_t hdr[11];
  hdr[0] = JOURNAL_STATUS;
  writeLe32(hdr + 1, seq);
  writeLe32(hdr + 5, 0);
  hdr[9] = status;
  hdr[10] = 0;
  bool ok = file.write(hdr, sizeof(hdr)) == sizeof(hdr);
  file.close();

  if (!ok) return false;
  ++_journal_records;
  if (_journal_records > JOURNAL_COMPACT_AT) compactLog();
  return true;
}

bool RemoteCommandModule::persistStatus(uint32_t seq, uint8_t status) {
  if (!_log.setStatus(seq, status)) return false;
  return appendJournalStatus(seq, status);
}

bool RemoteCommandModule::loadTimestampFile(const char* name, uint32_t& timestamp) {
  if (_fs == nullptr || !_fs->exists(name)) return false;
  File file = openRead(_fs, name);
  if (!file) return false;
  uint8_t buf[12];
  int len = file.read(buf, sizeof(buf));
  file.close();
  if (len != (int)sizeof(buf)) return false;
  if (memcmp(buf, "RCT1", 4) != 0) return false;
  uint32_t ts = readLe32(buf + 4);
  uint32_t inv = readLe32(buf + 8);
  if ((~ts) != inv) return false;
  timestamp = ts;
  return true;
}

bool RemoteCommandModule::saveTimestampFile(const char* name, uint32_t timestamp) {
  if (_fs == nullptr) return false;
  File file = openWrite(_fs, name);
  if (!file) return false;
  uint8_t buf[12];
  memcpy(buf, "RCT1", 4);
  writeLe32(buf + 4, timestamp);
  writeLe32(buf + 8, ~timestamp);
  size_t n = file.write(buf, sizeof(buf));
  file.close();
  return n == sizeof(buf);
}

bool RemoteCommandModule::loadLastTimestamp() {
  uint32_t t0 = 0, t1 = 0;
  bool ok0 = loadTimestampFile(TS_FILE0, t0);
  bool ok1 = loadTimestampFile(TS_FILE1, t1);
  if (ok0 && (!ok1 || t0 >= t1)) {
    _last_ts = t0;
    _ts_slot = 0;
    return true;
  }
  if (ok1) {
    _last_ts = t1;
    _ts_slot = 1;
    return true;
  }
  _last_ts = 0;
  _ts_slot = 0;
  return false;
}

bool RemoteCommandModule::saveLastTimestamp(uint32_t timestamp) {
  uint8_t next = (uint8_t)(_ts_slot ^ 1);
  const char* name = next == 0 ? TS_FILE0 : TS_FILE1;
  if (saveTimestampFile(name, timestamp)) {
    _ts_slot = next;
    return true;
  }
  name = _ts_slot == 0 ? TS_FILE0 : TS_FILE1;
  if (saveTimestampFile(name, timestamp)) return true;
  return false;
}

bool RemoteCommandModule::loadGuard() {
  _guard.clear();
  if (_fs == nullptr || !_fs->exists(F1_FILE)) return true;
  File file = openRead(_fs, F1_FILE);
  if (!file) return false;
  uint8_t buf[RemoteRadioGuardState::SERIALIZED_SIZE];
  int len = file.read(buf, sizeof(buf));
  file.close();
  return len == (int)sizeof(buf) && _guard.deserialize(buf, sizeof(buf));
}

bool RemoteCommandModule::saveGuard() {
  if (_fs == nullptr) return false;
  File file = openWrite(_fs, F1_FILE);
  if (!file) return false;
  uint8_t buf[RemoteRadioGuardState::SERIALIZED_SIZE];
  size_t len = _guard.serialize(buf, sizeof(buf));
  size_t n = file.write(buf, len);
  file.close();
  return len > 0 && n == len;
}

void RemoteCommandModule::removeGuard() {
  if (_fs != nullptr && _fs->exists(F1_FILE)) _fs->remove(F1_FILE);
}

void RemoteCommandModule::reconcilePending() {
  bool changed = false;
  RemoteCommandLogEntry entry;
  for (int i = 0; i < _log.count(); ++i) {
    if (!_log.getNewest(i, entry)) break;
    if (entry.status != REMOTE_CMD_LOG_PENDING) continue;

    if (remoteCmdMatchesPrefix(entry.command, "set tx", REMOTE_PREFIX_WORD)) {
      int8_t dbm = 0;
      bool ok = remoteCmdParseSetTx(entry.command, dbm) && _ctx.get_tx_power &&
                _ctx.get_tx_power(_ctx.ctx) == dbm;
      _log.setStatus(entry.seq, ok ? REMOTE_CMD_LOG_OK : REMOTE_CMD_LOG_FAIL);
      changed = true;
    } else if (remoteCmdMatchesPrefix(entry.command, "set radio", REMOTE_PREFIX_WORD)) {
      if (!_guard.armed) {
        _log.setStatus(entry.seq, REMOTE_CMD_LOG_FAIL);
        changed = true;
      }
    } else {
      _log.setStatus(entry.seq, REMOTE_CMD_LOG_FAIL);
      changed = true;
    }
  }
  if (changed) saveLog();
}

void RemoteCommandModule::startGuardTimers(uint32_t now) {
  _f1_timer_active = true;
  _f1_discover_at = now + GUARD_DISCOVER_MINUTES * 60UL * 1000UL;
  _f1_deadline = now + GUARD_RADIO_MINUTES * 60UL * 1000UL;
  _f1_baseline_rx = _rx_count;
}

void RemoteCommandModule::resolveGuard(bool ok, bool reboot_after) {
  _f1_timer_active = false;
  if (_guard.pending_seq != 0) {
    persistStatus(_guard.pending_seq, ok ? REMOTE_CMD_LOG_OK : REMOTE_CMD_LOG_FAIL);
  }

  if (ok) {
    _guard.clear();
    removeGuard();
    return;
  }

  _guard.revert_pending = true;
  saveGuard();

  RemoteCommandRadioParams old = {};
  old.freq = _guard.freq;
  old.bw = _guard.bw;
  old.sf = _guard.sf;
  old.cr = _guard.cr;
  old.tx_power = _ctx.get_tx_power ? _ctx.get_tx_power(_ctx.ctx) : 0;
  bool reverted = _ctx.apply_radio_and_save &&
                  _ctx.apply_radio_and_save(_ctx.ctx, old);

  if (!reverted && _guard.attempts < GUARD_MAX_ATTEMPTS) {
    // Keep F1 and try the revert again on the next boot. Reboot so the
    // in-RAM preferences cannot drift from the on-flash state.
    if (reboot_after && _ctx.reboot) _ctx.reboot(_ctx.ctx);
    return;
  }

  _guard.clear();
  removeGuard();
  if (reboot_after && _ctx.reboot) _ctx.reboot(_ctx.ctx);
}

int RemoteCommandModule::searchChannelsByHash(const uint8_t* hash,
                                              mesh::GroupChannel* dest,
                                              int max_matches) const {
  if (hash == nullptr || dest == nullptr || max_matches < 1) return 0;
  if (hash[0] != _channel.hash[0]) return 0;
  dest[0] = _channel;
  return 1;
}

void RemoteCommandModule::onGroupData(const mesh::Packet* packet,
                                      const mesh::GroupChannel& channel,
                                      const uint8_t* data, size_t len) {
  if (packet == nullptr || data == nullptr || len < 3) return;
  if (memcmp(channel.secret, REMOTE_CMD_CHANNEL_KEY, REMOTE_CMD_CHANNEL_KEY_LEN) != 0) return;

  uint16_t data_type = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  if (data_type != REMOTE_CMD_DATA_TYPE) return;

  uint8_t blob_len = data[2];
  if (blob_len > REMOTE_CMD_MAX_BLOB || (size_t)blob_len + 3 > len) return;

  const uint8_t* blob = data + 3;
  size_t text_len = 0;
  uint32_t timestamp = 0;
  if (!remoteCmdOpen(blob, blob_len, REMOTE_CMD_KEY1, REMOTE_CMD_KEY2,
                     _rx_text, sizeof(_rx_text), text_len, timestamp)) {
    return;
  }
  if (timestamp <= _last_ts) return;

  uint8_t count = countCommands(_rx_text);
  if (count == 0) {
    if (saveLastTimestamp(timestamp)) _last_ts = timestamp;
    return;
  }
  if (count > REMOTE_CMD_MAX_COMMANDS) {
    logDebug("remote commands: too many commands");
    return;
  }
  if (_queue_count >= QUEUE_SIZE) {
    logDebug("remote commands: queue full");
    return;
  }

  uint8_t payload[REMOTE_CMD_MAX_PAYLOAD];
  size_t payload_len = 0;
  if (!remoteCmdUnmask(blob, blob_len, payload, sizeof(payload), payload_len)) return;
  size_t compressed_len = payload_len - 6;
  if (compressed_len > REMOTE_CMD_MAX_PAYLOAD) return;
  if (!saveLastTimestamp(timestamp)) {
    logDebug("remote commands: cannot persist replay timestamp");
    return;
  }

  QueueEntry& entry = _queue[_queue_count++];
  entry.timestamp = timestamp;
  entry.compressed_len = (uint8_t)compressed_len;
  memcpy(entry.compressed, payload + 6, compressed_len);
  entry.received_at = _ctx.get_millis ? _ctx.get_millis(_ctx.ctx) : millis();
  entry.forwarded = false;

  uint8_t hash[32];
  packet->calculatePacketHash(hash);
  memcpy(entry.packet_hash, hash, sizeof(entry.packet_hash));

  _last_ts = timestamp;
}

void RemoteCommandModule::onPacketReceived() {
  ++_rx_count;
}

void RemoteCommandModule::onPacketTransmitted(const mesh::Packet* packet) {
  if (packet == nullptr || _queue_count == 0) return;
  uint8_t hash[32];
  packet->calculatePacketHash(hash);
  for (uint8_t i = 0; i < _queue_count; ++i) {
    if (!_queue[i].forwarded &&
        memcmp(_queue[i].packet_hash, hash, sizeof(_queue[i].packet_hash)) == 0) {
      _queue[i].forwarded = true;
    }
  }
}

void RemoteCommandModule::onDiscoverResponse(uint32_t tag) {
  if (!_tx_guard.active || tag == 0) return;
  for (uint8_t i = 0; i < _tx_guard.tag_count; ++i) {
    if (_tx_guard.tags[i] == tag) {
      _tx_guard.response = true;
      return;
    }
  }
}

void RemoteCommandModule::updateTxGuard(uint32_t now) {
  if (!_tx_guard.active) return;
  if (_tx_guard.response) {
    finishTxGuard(true);
    return;
  }

  if (_tx_guard.phase == 0 && timeReached(now, _tx_guard.next_at)) {
    uint32_t tag = 0;
    if (_ctx.send_discover) _ctx.send_discover(_ctx.ctx, &tag);
    if (tag != 0) _tx_guard.tags[_tx_guard.tag_count++] = tag;
    _tx_guard.phase = 1;
    _tx_guard.next_at = now + TX_GUARD_WINDOW_MS;
  } else if (_tx_guard.phase == 1 && timeReached(now, _tx_guard.next_at)) {
    uint32_t tag = 0;
    if (_ctx.send_discover) _ctx.send_discover(_ctx.ctx, &tag);
    if (tag != 0 && _tx_guard.tag_count < 2) _tx_guard.tags[_tx_guard.tag_count++] = tag;
    _tx_guard.phase = 2;
    _tx_guard.next_at = now + TX_GUARD_WINDOW_MS;
  } else if (_tx_guard.phase == 2 && timeReached(now, _tx_guard.next_at)) {
    finishTxGuard(false);
  }
}

void RemoteCommandModule::finishTxGuard(bool ok) {
  if (ok) {
    bool committed = _ctx.commit_tx_power &&
                     _ctx.commit_tx_power(_ctx.ctx, _tx_guard.new_dbm);
    if (!committed) {
      if (_ctx.commit_tx_power) _ctx.commit_tx_power(_ctx.ctx, _tx_guard.old_dbm);
      if (_ctx.apply_tx_power) _ctx.apply_tx_power(_ctx.ctx, _tx_guard.old_dbm);
      persistStatus(_tx_guard.pending_seq, REMOTE_CMD_LOG_FAIL);
    } else {
      persistStatus(_tx_guard.pending_seq, REMOTE_CMD_LOG_OK);
    }
  } else {
    if (_ctx.apply_tx_power) _ctx.apply_tx_power(_ctx.ctx, _tx_guard.old_dbm);
    persistStatus(_tx_guard.pending_seq, REMOTE_CMD_LOG_FAIL);
  }
  _tx_guard = TxGuard{};
}

void RemoteCommandModule::loop(uint32_t now_ms) {
  updateTxGuard(now_ms);

  if (_f1_timer_active) {
    uint32_t delta = _rx_count - _f1_baseline_rx;
    if (_f1_discover_at != 0 && timeReached(now_ms, _f1_discover_at)) {
      if (delta < GUARD_MIN_RX_PACKETS && _ctx.send_discover) {
        uint32_t tag = 0;
        _ctx.send_discover(_ctx.ctx, &tag);
      }
      _f1_discover_at = 0;
    }
    if (timeReached(now_ms, _f1_deadline)) {
      bool ok = delta >= GUARD_MIN_RX_PACKETS;
      resolveGuard(ok, !ok);
      return;
    }
  }

  if (!_executing) startNextPacket(now_ms);

  if (_executing && timeReached(now_ms, _next_cmd_at)) {
    if (_cmd_index >= _cmd_count) {
      finishPacket();
    } else {
      executeNext();
      _next_cmd_at = now_ms + COMMAND_GAP_MS;
    }
  }
}

bool RemoteCommandModule::radioChangeActive() const {
  bool temp = _ctx.is_temp_radio_active && _ctx.is_temp_radio_active(_ctx.ctx);
  return _guard.armed || _tx_guard.active || temp;
}

bool RemoteCommandModule::startNextPacket(uint32_t now) {
  if (_queue_count == 0) return false;

  QueueEntry& entry = _queue[0];
  bool ready = false;
  if (entry.forwarded && _ctx.get_outbound_total &&
      _ctx.get_outbound_total(_ctx.ctx) == 0 &&
      (uint32_t)(now - entry.received_at) >= QUEUE_SETTLE_MS) {
    ready = true;
  } else if ((uint32_t)(now - entry.received_at) >= QUEUE_FALLBACK_MS) {
    ready = true;
  }
  if (!ready) return false;

  if (!decodeQueuedText(entry)) {
    finishPacket();
    return false;
  }

  _executing = true;
  _executing_ts = entry.timestamp;
  _cmd_index = 0;
  _next_cmd_at = now;
  return true;
}

bool RemoteCommandModule::decodeQueuedText(QueueEntry& entry) {
  size_t text_len = 0;
  if (!remoteCmdDecodeText(entry.compressed, entry.compressed_len,
                           _text, sizeof(_text), text_len)) {
    return false;
  }
  _cmd_count = splitCommands(_text);
  return _cmd_count > 0 && _cmd_count <= REMOTE_CMD_MAX_COMMANDS;
}

uint8_t RemoteCommandModule::countCommands(char* text) {
  uint8_t count = 0;
  char* p = text;
  while (*p) {
    char* line = p;
    while (*p && *p != '\n') ++p;
    if (*p == '\n') *p++ = 0;

    size_t len = strlen(line);
    while (len > 0 && line[len - 1] == '\r') line[--len] = 0;
    if (!remoteCmdNormalize(line, REMOTE_CMD_MAX_TEXT)) continue;
    if (line[0] == 0) continue;
    if (count < 255) ++count;
  }
  return count;
}

uint8_t RemoteCommandModule::splitCommands(char* text) {
  uint8_t count = 0;
  char* p = text;
  while (*p) {
    char* line = p;
    while (*p && *p != '\n') ++p;
    if (*p == '\n') *p++ = 0;

    size_t len = strlen(line);
    while (len > 0 && line[len - 1] == '\r') line[--len] = 0;
    if (!remoteCmdNormalize(line, REMOTE_CMD_MAX_TEXT)) continue;
    if (line[0] == 0) continue;

    if (count < REMOTE_CMD_MAX_COMMANDS) _cmds[count] = line;
    if (count < 255) ++count;
  }
  return count;
}

void RemoteCommandModule::finishPacket() {
  if (_queue_count > 0) {
    if (_queue_count > 1) {
      memmove(&_queue[0], &_queue[1], (size_t)(_queue_count - 1) * sizeof(QueueEntry));
    }
    --_queue_count;
  }
  _executing = false;
  _cmd_index = 0;
  _cmd_count = 0;
}

void RemoteCommandModule::executeNext() {
  if (_cmd_index >= _cmd_count) return;
  char* command = _cmds[_cmd_index++];

  if (strlen(command) > REMOTE_CMD_MAX_LINE) {
    char truncated[REMOTE_CMD_MAX_LINE + 1];
    size_t keep = REMOTE_CMD_MAX_LINE - 3;
    memcpy(truncated, command, keep);
    memcpy(truncated + keep, "...", 3);
    truncated[REMOTE_CMD_MAX_LINE] = 0;
    uint32_t seq = 0;
    appendLog(_executing_ts, truncated, REMOTE_CMD_LOG_FAIL, seq);
    return;
  }
  if (remoteCmdIsBanned(command)) {
    uint32_t seq = 0;
    appendLog(_executing_ts, command, REMOTE_CMD_LOG_FAIL, seq);
    return;
  }
  if (radioChangeActive() && remoteCmdIsRadioChanging(command)) {
    uint32_t seq = 0;
    appendLog(_executing_ts, command, REMOTE_CMD_LOG_FAIL, seq);
    return;
  }
  if (remoteCmdIsTimedRadio(command)) {
    handleTimedRadio(command);
    return;
  }
  if (remoteCmdIsTimedTx(command)) {
    handleTimedTx(command);
    return;
  }
  if (remoteCmdMatchesPrefix(command, "tempradio", REMOTE_PREFIX_WORD)) {
    float freq = 0, bw = 0;
    uint8_t sf = 0, cr = 0;
    int timeout = 0;
    uint32_t seq = 0;

    // Policy and CommonCLI parse tempradio differently. If policy cannot
    // validate it, never fall through to the looser CLI parser.
    if (!remoteCmdParseTempRadio(command, freq, bw, sf, cr, timeout)) {
      appendLog(_executing_ts, command, REMOTE_CMD_LOG_FAIL, seq);
      return;
    }

    RemoteCommandRadioParams old = {};
    if (!_ctx.get_radio || !_ctx.get_radio(_ctx.ctx, &old) ||
        !remoteCmdSameBand(old.freq, freq, bw)) {
      appendLog(_executing_ts, command, REMOTE_CMD_LOG_FAIL, seq);
      return;
    }

    char canonical[64];
    snprintf(canonical, sizeof(canonical), "tempradio %.6f,%.3f,%u,%u,%d",
             freq, bw, (unsigned)sf, (unsigned)cr, timeout);
    executeOrdinary(canonical);
    return;
  }

  executeOrdinary(command);
}

bool RemoteCommandModule::appendLog(uint32_t timestamp, const char* command,
                                    uint8_t status, uint32_t& seq) {
  if (!_log.append(timestamp, command, status, seq)) return false;
  uint8_t len = (uint8_t)strlen(command);
  if (!appendJournalCommand(seq, timestamp, status, command, len)) {
    _log.remove(seq);
    return false;
  }
  return true;
}

bool RemoteCommandModule::executeOrdinary(const char* command) {
  uint32_t seq = 0;
  if (!appendLog(_executing_ts, command, REMOTE_CMD_LOG_OK, seq)) return false;

  memset(_reply, 0, sizeof(_reply));
  if (_ctx.execute_command) {
    _ctx.execute_command(_ctx.ctx, const_cast<char*>(command), _reply, sizeof(_reply));
  }
  if (replyFailed(_reply)) {
    persistStatus(seq, REMOTE_CMD_LOG_FAIL);
  }
  return true;
}

bool RemoteCommandModule::handleTimedRadio(const char* command) {
  uint32_t seq = 0;
  float freq = 0, bw = 0;
  uint8_t sf = 0, cr = 0;
  if (!remoteCmdParseSetRadio(command, freq, bw, sf, cr)) {
    appendLog(_executing_ts, command, REMOTE_CMD_LOG_FAIL, seq);
    return false;
  }

  RemoteCommandRadioParams old = {};
  if (!_ctx.get_radio || !_ctx.get_radio(_ctx.ctx, &old)) {
    appendLog(_executing_ts, command, REMOTE_CMD_LOG_FAIL, seq);
    return false;
  }
  if (!remoteCmdSameBand(old.freq, freq, bw)) {
    appendLog(_executing_ts, command, REMOTE_CMD_LOG_FAIL, seq);
    return false;
  }
  if (freq == old.freq && bw == old.bw && sf == old.sf && cr == old.cr) {
    appendLog(_executing_ts, command, REMOTE_CMD_LOG_OK, seq);
    return true;
  }

  if (!appendLog(_executing_ts, command, REMOTE_CMD_LOG_PENDING, seq)) return false;

  _guard.clear();
  _guard.armed = true;
  _guard.attempts = 0;
  _guard.freq = old.freq;
  _guard.bw = old.bw;
  _guard.sf = old.sf;
  _guard.cr = old.cr;
  _guard.pending_timestamp = _executing_ts;
  _guard.pending_seq = seq;
  _guard.target_freq = freq;
  _guard.target_bw = bw;
  _guard.target_sf = sf;
  _guard.target_cr = cr;

  if (!saveGuard()) {
    persistStatus(seq, REMOTE_CMD_LOG_FAIL);
    _guard.clear();
    return false;
  }

  memset(_reply, 0, sizeof(_reply));
  if (_ctx.execute_command) {
    _ctx.execute_command(_ctx.ctx, const_cast<char*>(command), _reply, sizeof(_reply));
  }
  if (replyFailed(_reply)) {
    persistStatus(seq, REMOTE_CMD_LOG_FAIL);
    _guard.clear();
    removeGuard();
    return false;
  }

  // The new parameters are only applied after a reboot. F1 stays armed; the
  // 10-minute window starts in begin() on the next boot.
  return true;
}

bool RemoteCommandModule::handleTimedTx(const char* command) {
  uint32_t seq = 0;
  int8_t dbm = 0;
  if (!remoteCmdParseSetTx(command, dbm) || !_ctx.get_tx_power) {
    appendLog(_executing_ts, command, REMOTE_CMD_LOG_FAIL, seq);
    return false;
  }

  int8_t old = _ctx.get_tx_power(_ctx.ctx);
  if (dbm >= old) {
    if (!appendLog(_executing_ts, command, REMOTE_CMD_LOG_OK, seq)) return false;

    if (_ctx.apply_tx_power) _ctx.apply_tx_power(_ctx.ctx, dbm);
    bool committed = _ctx.commit_tx_power && _ctx.commit_tx_power(_ctx.ctx, dbm);
    if (!committed) {
      if (_ctx.commit_tx_power) _ctx.commit_tx_power(_ctx.ctx, old);
      if (_ctx.apply_tx_power) _ctx.apply_tx_power(_ctx.ctx, old);
      persistStatus(seq, REMOTE_CMD_LOG_FAIL);
      return false;
    }
    return true;
  }

  if (!appendLog(_executing_ts, command, REMOTE_CMD_LOG_PENDING, seq)) return false;

  if (_ctx.apply_tx_power) _ctx.apply_tx_power(_ctx.ctx, dbm);
  _tx_guard = TxGuard{};
  _tx_guard.active = true;
  _tx_guard.old_dbm = old;
  _tx_guard.new_dbm = dbm;
  _tx_guard.pending_seq = seq;
  _tx_guard.phase = 0;
  _tx_guard.next_at = (_ctx.get_millis ? _ctx.get_millis(_ctx.ctx) : millis()) +
                      TX_GUARD_DISCOVER_DELAY_MS;
  return true;
}

bool RemoteCommandModule::handleCommand(char* command, char* reply,
                                        size_t reply_sz, bool serial) {
  (void)serial;
  if (command == nullptr || reply == nullptr || reply_sz == 0) return false;
  if (!remoteCmdMatchesPrefix(command, "cmdlog", REMOTE_PREFIX_WORD)) return false;

  const char* args = command + strlen("cmdlog");
  while (*args == ' ') ++args;
  int offset = 0;
  if (*args != 0) {
    char* end = nullptr;
    long value = strtol(args, &end, 10);
    while (end && *end == ' ') ++end;
    if (end == args || *end != 0 || value < 0) {
      strncpy(reply, "Err - bad offset", reply_sz - 1);
      reply[reply_sz - 1] = 0;
      return true;
    }
    offset = (int)value;
  }

  ListReplyPager pager(reply, reply_sz, offset);
  RemoteCommandLogEntry entry;
  for (int i = 0; i < _log.count(); ++i) {
    if (!_log.getNewest(i, entry)) break;
    DateTime dt(entry.timestamp);

    char command_part[REMOTE_CMD_MAX_LINE + 1];
    size_t max_line = reply_sz > LIST_REPLY_SUFFIX_RESERVE + 40
                          ? reply_sz - LIST_REPLY_SUFFIX_RESERVE - 1
                          : 32;
    size_t prefix = 19 + 3;   // "YYYY-MM-DD HH:MM:SS : "
    const char* status = statusText(entry.status);
    size_t suffix = status ? strlen(status) + 3 : 0;
    size_t command_cap = max_line > prefix + suffix + 3
                             ? max_line - prefix - suffix
                             : 8;
    if (command_cap > REMOTE_CMD_MAX_LINE) command_cap = REMOTE_CMD_MAX_LINE;

    size_t len = entry.len;
    if (len > command_cap) {
      size_t keep = command_cap > 3 ? command_cap - 3 : command_cap;
      memcpy(command_part, entry.command, keep);
      if (command_cap >= 3) memcpy(command_part + keep, "...", 3);
      len = command_cap;
      command_part[len] = 0;
    } else {
      memcpy(command_part, entry.command, len);
      command_part[len] = 0;
    }

    char line[256];
    if (status != nullptr) {
      snprintf(line, sizeof(line), "%04d-%02d-%02d %02d:%02d:%02d : %s : %s",
               dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(),
               dt.second(), command_part, status);
    } else {
      snprintf(line, sizeof(line), "%04d-%02d-%02d %02d:%02d:%02d : %s",
               dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(),
               dt.second(), command_part);
    }
    pager.add(line, strlen(line));
  }
  pager.finish();
  return true;
}

bool RemoteCommandModule::hasPendingWork() const {
  bool temp = _ctx.is_temp_radio_active && _ctx.is_temp_radio_active(_ctx.ctx);
  return _executing || _queue_count > 0 || _tx_guard.active ||
         _f1_timer_active || temp;
}

void RemoteCommandModule::logDebug(const char* line) {
  if (_ctx.log_line) _ctx.log_line(_ctx.ctx, line);
}
