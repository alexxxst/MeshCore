#pragma once

#include <Arduino.h> // needed for PlatformIO
#include <Mesh.h>
#include <helpers/CommonCLI.h>

#if defined(NRF52_PLATFORM)
#include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
#include <LittleFS.h>
#elif defined(ESP32)
#include <SPIFFS.h>
#endif

#include "helpers/ArduinoHelpers.h"
#include <helpers/IdentityStore.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/StaticPoolPacketManager.h>
#include <target.h>

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#define FIRMWARE_VER_TEXT   "v1.2.11"
#define FIRMWARE_BUILD_TEXT "2026-04-24"

#define LORA_FREQ           868.856
#define LORA_BW             62.5
#define LORA_SF             7
#define LORA_CR             7
#define LORA_TX_POWER       22

#define PATH_HASH_MODE      1   // bytes
#define MAX_GROUP_CHANNELS  1
#define MAX_CONTACTS        350

#include <helpers/BaseChatMesh.h>

#define SEND_TIMEOUT_BASE_MILLIS        1000
#define FLOOD_SEND_TIMEOUT_FACTOR       16.0f
#define DIRECT_SEND_PERHOP_FACTOR       6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS 250

#define QUIET_LIMIT_SECONDS             5    // seconds to cooldown
#define QUIET_LIMIT_TIME                5    // minutes to check
#define QUIET_LIMIT_COUNT               30   // messages to check
#define QUIET_LIMIT_TIMES               100  // overall limit for timestamps array
#define QUIET_LIMIT_PAUSE               2.0f // seconds to reply
#define OLD_REPEATER_CHECK              5    // hours
#define OLD_REPEATER_TIME               7    // days
#define MESSAGES_TO_REBOOT              1000
#define MAGIC_TIME_1                    1767214800
#define MAGIC_TIME_2                    (MAGIC_TIME_1 + 5 * 365 * 86400)

#define BOT_NAME                        "Mr.Pong🏓"
#define BOT_NAME_PLAIN                  "Mr.Pong"
#define PUBLIC_GROUP_NAME               "#bot"                     // #bot
#define PUBLIC_GROUP_PSK                "61ChvLPk5de/aaV8na2iEQ==" // #bot's channel PSK

/* -------------------------------------------------------------------------------------- */

struct Repeater {
  uint8_t pub_key[PUB_KEY_SIZE]{};
  char name[32]{};
  unsigned int first_count = 0;
  unsigned int total_count = 0;
  unsigned int advert_time = 0;
  unsigned int update_time = 0;
};

struct NodeStats {
  unsigned int total_request = 0;
  unsigned int total_received = 0;
  unsigned int total_sent = 0;
  unsigned int total_thanks = 0;
  unsigned int total_ignores = 0;
  unsigned int total_hops = 0;

  unsigned int max_hops = 0;
  char max_path[185]{};

  unsigned long time_start = 0;

  unsigned int num_repeaters = 0;
  Repeater repeaters[MAX_CONTACTS];
};

// ReSharper disable once CppPolymorphicClassWithNonVirtualPublicDestructor
class MyMesh : public BaseChatMesh {
  FILESYSTEM *_fs{};
  NodePrefs _prefs{};
  NodeStats _stats{};
  ChannelDetails *_public{};

  unsigned long last_repeater_check = 0;
  unsigned long last_msg_sent = 0;
  unsigned long last_msg_rcvd = 0;
  unsigned long last_msg_times[QUIET_LIMIT_TIMES]{};
  unsigned int last_msg_count = 0;
  unsigned int total_sent = 0;

  char command[512 + 10]{};
  uint8_t tmp_buf[256]{};
  uint8_t _buf[256]{};

  bool clock_set = false;
  bool quiet = false;

  char message[256]{};

  void saveStats();
  // ReSharper disable once CppHidingFunction
  void resetStats();
  void loadStats();
  void setClock(uint32_t timestamp) const;
  void importCard(const char *command);

protected:
  static const char *getTypeName(const uint8_t type) {
    if (type == ADV_TYPE_CHAT) return "Chat";
    if (type == ADV_TYPE_REPEATER) return "Repeater";
    if (type == ADV_TYPE_ROOM) return "Room";
    return "??"; // unknown
  }

  static const char *hop_word(const int hop) {
    int n = hop % 100;
    if (n >= 11 && n <= 14) return "хопов";
    n = hop % 10;
    if (n == 1) return "хоп";
    if (n >= 2 && n <= 4) return "хопа";
    return "хопов";
  }

  static void format_uptime(uint32_t seconds, char *buf, const size_t buf_size) {
    const uint32_t d = seconds / 86400;
    seconds %= 86400;
    const uint32_t h = seconds / 3600;
    seconds %= 3600;
    const uint32_t m = seconds / 60;
    if (d > 0)
      snprintf(buf, buf_size, "%uд %uч %uм", d, h, m);
    else if (h > 0)
      snprintf(buf, buf_size, "%uч %uм", h, m);
    else
      snprintf(buf, buf_size, "%uм", m);
  }

  static void format_days(const uint32_t seconds, char *buf, const size_t buf_size) {
    const uint32_t d = seconds / 86400;
    const uint32_t h = seconds / 3600;
    if (d > 0)
      snprintf(buf, buf_size, "%uд", d);
    else
      snprintf(buf, buf_size, "%uч", h);
  }

  static bool checkRepeaterNamePattern(const char *s) {
    if (strlen(s) < 5) return false;
    return (((s[0] >= 'A' && s[0] <= 'Z') || (s[0] >= 'a' && s[0] <= 'z')) &&
            ((s[1] >= 'A' && s[1] <= 'Z') || (s[1] >= 'a' && s[1] <= 'z')) &&
            ((s[2] >= 'A' && s[2] <= 'Z') || (s[2] >= 'a' && s[2] <= 'z')) && (s[3] == '-' || s[3] == '_')) ||
           (strlen(s) > 7 && (s[0] == 'L' || s[0] == 'l') && (s[1] == 'O' || s[1] == 'o') && (s[2] == '-' || s[2] == '_') &&
            ((s[3] >= 'A' && s[3] <= 'Z') || (s[3] >= 'a' && s[3] <= 'z')) &&
            ((s[4] >= 'A' && s[4] <= 'Z') || (s[4] >= 'a' && s[4] <= 'z')) &&
            ((s[5] >= 'A' && s[5] <= 'Z') || (s[5] >= 'a' && s[5] <= 'z')) && (s[6] == '-' || s[6] == '_'));
  }

  void onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t timestamp, const char *text) override;

  float getAirtimeBudgetFactor() const override { return _prefs.airtime_factor; }

  int calcRxDelay(const float score, const uint32_t air_time) const override {
    if (_prefs.rx_delay_base <= 0.0f) return 0;
    return static_cast<int>((pow(_prefs.rx_delay_base, 0.85f - score) - 1.0) * air_time);
  }

  bool allowPacketForward(const mesh::Packet *packet) override { return false; }

  void onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t *path) override;

  void onContactPathUpdated(const ContactInfo &contact) override {
    // not supported
  }

  bool shouldAutoAddContactType(uint8_t type) const override { return true; }
  bool shouldOverwriteWhenFull() const override { return true; }
  bool isAutoAddEnabled() const override { return true; }

  int getAGCResetInterval() const override {
    return static_cast<int>(_prefs.agc_reset_interval) * 4000; // milliseconds
  }

  ContactInfo *processAck(const uint8_t *data) override {
    // not supported
    return nullptr;
  }

  void onMessageRecv(const ContactInfo &from, mesh::Packet *pkt, const uint32_t sender_timestamp, const char *text) override {
    // not supported
  }

  void onCommandDataRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp, const char *text) override {
    // not supported
  }

  void onSignedMessageRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix,
                           const char *text) override {
    // not supported
  }

  uint8_t onContactRequest(const ContactInfo &contact, uint32_t sender_timestamp, const uint8_t *data, uint8_t len,
                           uint8_t *reply) override {
    return 0; // unknown
  }

  void onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len) override {
    // not supported
  }

  uint32_t calcFloodTimeoutMillisFor(const uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + FLOOD_SEND_TIMEOUT_FACTOR * static_cast<float>(pkt_airtime_millis);
  }

  uint32_t calcDirectTimeoutMillisFor(const uint32_t pkt_airtime_millis, const uint8_t path_len) const override {
    const uint8_t path_hash_count = path_len & 63;
    return SEND_TIMEOUT_BASE_MILLIS +
           (static_cast<float>(pkt_airtime_millis) * DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) *
               static_cast<float>(path_hash_count + 1);
  }

  void onSendTimeout() override { Serial.println("   ERROR: timed out, no ACK."); }

#if ENV_INCLUDE_GPS == 1
  void applyGpsPrefs() const {
    sensors.setSettingValue("gps", _prefs.gps_enabled ? "1" : "0");
    char interval_str[12];
    sprintf(interval_str, "%u", _prefs.gps_interval);
    sensors.setSettingValue("gps_interval", interval_str);
  }
#endif

public:
  MyMesh(mesh::Radio &radio, StdRNG &rng, mesh::RTCClock &rtc, SimpleMeshTables &tables);

  bool getQuiet() const { return quiet; }
  unsigned long getTotalReceived() const { return _stats.total_received; }
  unsigned long getTotalRequested() const { return _stats.total_request; }
  unsigned long getTotalSent() const { return _stats.total_sent; }
  unsigned long getTenReceived() const { return last_msg_count; }
  const NodeStats *getStats() const { return &_stats; }
  void begin(FILESYSTEM &fs);
  void sendMessage(const char *message, uint8_t path_hash_size);
  void handleCommand(const char *command);
  // ReSharper disable once CppHidingFunction
  void loop();

  bool addRepeater(const ContactInfo &contact);
  bool updateRepeater(Repeater &repeater, const ContactInfo &contact) const;
  bool removeRepeater(const Repeater &repeater);
  Repeater *searchRepeaterByPubKey(const uint8_t *pub_key, int prefix_len);
  Repeater *searchRepeaterByPubKey(const uint8_t *pub_key);
  Repeater *createRepeater(uint8_t prefix[4]);
  Repeater *allocateRepeaterSlot();
};