#pragma once

#include <stddef.h>
#include <stdint.h>

#include <Mesh.h>

#include "FsCompat.h"
#include "RemoteCommandConfig.h"
#include "RemoteCommandLog.h"
#include "RemoteRadioGuard.h"

struct RemoteCommandRadioParams {
  float freq;
  float bw;
  uint8_t sf;
  uint8_t cr;
  int8_t tx_power;
};

struct RemoteCommandModuleContext {
  void* ctx = nullptr;
  uint32_t (*get_millis)(void*) = nullptr;
  int (*get_outbound_total)(void*) = nullptr;
  bool (*send_discover)(void*, uint32_t* tag) = nullptr;
  bool (*get_radio)(void*, RemoteCommandRadioParams* out) = nullptr;
  bool (*is_temp_radio_active)(void*) = nullptr;
  bool (*apply_radio_and_save)(void*, const RemoteCommandRadioParams& params) = nullptr;
  void (*reboot)(void*) = nullptr;
  int8_t (*get_tx_power)(void*) = nullptr;
  void (*apply_tx_power)(void*, int8_t dbm) = nullptr;
  bool (*commit_tx_power)(void*, int8_t dbm) = nullptr;
  void (*execute_command)(void*, char* cmd, char* reply, size_t reply_sz) = nullptr;
  void (*log_line)(void*, const char* line) = nullptr;
};

class RemoteCommandModule {
public:
  static constexpr const char* F1_FILE = "/remote_cmd_radio";
  static constexpr const char* F2_FILE = "/remote_cmd_log";
  static constexpr const char* TS_FILE0 = "/remote_cmd_ts0";
  static constexpr const char* TS_FILE1 = "/remote_cmd_ts1";
  static constexpr uint8_t QUEUE_SIZE = 4;
  static constexpr uint8_t GUARD_MIN_RX_PACKETS = 3;
  static constexpr uint16_t GUARD_RADIO_MINUTES = 10;
  static constexpr uint16_t GUARD_DISCOVER_MINUTES = 8;
  static constexpr uint8_t GUARD_MAX_ATTEMPTS = 5;

  void begin(FILESYSTEM* fs, const RemoteCommandModuleContext& context);

  int searchChannelsByHash(const uint8_t* hash, mesh::GroupChannel* dest,
                           int max_matches) const;

  void onGroupData(const mesh::Packet* packet, const mesh::GroupChannel& channel,
                   const uint8_t* data, size_t len);
  void onPacketReceived();
  void onPacketTransmitted(const mesh::Packet* packet);
  void onDiscoverResponse(uint32_t tag);

  void loop(uint32_t now_ms);

  bool handleCommand(char* command, char* reply, size_t reply_sz, bool serial);
  bool hasPendingWork() const;

private:
  struct QueueEntry {
    uint32_t timestamp = 0;
    uint8_t compressed[REMOTE_CMD_MAX_PAYLOAD] = {};
    uint8_t compressed_len = 0;
    uint8_t packet_hash[8] = {};
    uint32_t received_at = 0;
    bool forwarded = false;
  };

  struct TxGuard {
    bool active = false;
    int8_t old_dbm = 0;
    int8_t new_dbm = 0;
    uint32_t pending_seq = 0;
    uint8_t phase = 0;          // 0: waiting 10s, 1: first minute, 2: second minute
    uint32_t next_at = 0;
    uint32_t tags[2] = {};
    uint8_t tag_count = 0;
    bool response = false;
  };

  FILESYSTEM* _fs = nullptr;
  RemoteCommandModuleContext _ctx;

  uint8_t _channel_hash[32] = {};
  mesh::GroupChannel _channel = {};

  RemoteCommandLog _log;
  RemoteRadioGuardState _guard;
  uint32_t _last_ts = 0;
  uint8_t _ts_slot = 0;
  uint16_t _journal_records = 0;

  bool _f1_timer_active = false;
  uint32_t _f1_discover_at = 0;
  uint32_t _f1_deadline = 0;
  uint32_t _f1_baseline_rx = 0;
  uint32_t _rx_count = 0;

  TxGuard _tx_guard;

  QueueEntry _queue[QUEUE_SIZE];
  uint8_t _queue_count = 0;

  bool _executing = false;
  uint32_t _executing_ts = 0;
  uint32_t _next_cmd_at = 0;
  uint8_t _cmd_index = 0;
  uint8_t _cmd_count = 0;
  char* _cmds[REMOTE_CMD_MAX_COMMANDS] = {};
  char _text[REMOTE_CMD_MAX_TEXT] = {};
  char _rx_text[REMOTE_CMD_MAX_TEXT] = {};
  char _reply[160] = {};

  bool loadLog();
  bool saveLog();
  bool ensureLogFile();
  bool appendJournalCommand(uint32_t seq, uint32_t timestamp, uint8_t status,
                            const char* command, uint8_t len);
  bool appendJournalStatus(uint32_t seq, uint8_t status);
  bool persistStatus(uint32_t seq, uint8_t status);
  bool compactLog();
  bool loadLastTimestamp();
  bool saveLastTimestamp(uint32_t timestamp);
  bool loadTimestampFile(const char* name, uint32_t& timestamp);
  bool saveTimestampFile(const char* name, uint32_t timestamp);
  bool loadGuard();
  bool saveGuard();
  void removeGuard();
  void reconcilePending();
  void startGuardTimers(uint32_t now);
  void resolveGuard(bool ok, bool reboot_after);

  bool appendLog(uint32_t timestamp, const char* command, uint8_t status,
                 uint32_t& seq);
  bool executeOrdinary(const char* command);
  bool handleTimedRadio(const char* command);
  bool handleTimedTx(const char* command);

  void updateTxGuard(uint32_t now);
  void finishTxGuard(bool ok);

  bool radioChangeActive() const;
  bool startNextPacket(uint32_t now);
  void finishPacket();
  bool decodeQueuedText(QueueEntry& entry);
  uint8_t countCommands(char* text);
  uint8_t splitCommands(char* text);
  void executeNext();

  void logDebug(const char* line);
};
