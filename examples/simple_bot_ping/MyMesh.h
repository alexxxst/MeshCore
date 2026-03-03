#pragma once

#include <Arduino.h> // needed for PlatformIO
#include <Mesh.h>
#include <helpers/CommonCLI.h>
#include <sys/signal.h>

#if defined(NRF52_PLATFORM)
#include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
#include <LittleFS.h>
#elif defined(ESP32)
#include <SPIFFS.h>
#endif

#include <RTClib.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/IdentityStore.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/StaticPoolPacketManager.h>
#include <target.h>

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#define FIRMWARE_VER_TEXT   "v1.1.0"
#define FIRMWARE_BUILD_TEXT "2026-03-03"

#define LORA_FREQ           868.856
#define LORA_BW             62.5
#define LORA_SF             8
#define LORA_CR             5
#define LORA_TX_POWER       10

#ifndef MAX_CONTACTS
#define MAX_CONTACTS 350
#endif

#include <helpers/BaseChatMesh.h>

#define SEND_TIMEOUT_BASE_MILLIS        500
#define FLOOD_SEND_TIMEOUT_FACTOR       16.0f
#define DIRECT_SEND_PERHOP_FACTOR       6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS 250

#define MAX_GROUP_CHANNELS              1

#define QUIET_LIMIT_SECONDS             5    // seconds to cooldown
#define QUIET_LIMIT_TIME                5    // minutes to check
#define QUIET_LIMIT_COUNT               30   // messages to check
#define QUIET_LIMIT_TIMES               255  // overall limit for timestamps array
#define QUIET_LIMIT_PAUSE               2.0f // seconds to reply

#define BOT_NAME                        "Mr.Pong🏓"
#define BOT_NAME_PLAIN                  "Mr.Pong"
#define PUBLIC_GROUP_NAME               "#bot"                     // #bot
#define PUBLIC_GROUP_PSK                "61ChvLPk5de/aaV8na2iEQ==" // #bot's channel PSK

/* -------------------------------------------------------------------------------------- */

struct NodeStats {
  unsigned int first_repeaters_count[255]{};
  unsigned int all_repeaters_count[255]{};

  unsigned int total_request = 0;
  unsigned int total_received = 0;
  unsigned int total_sent = 0;
  unsigned int total_thanks = 0;
  unsigned int total_ignores = 0;
  unsigned int total_hops = 0;

  unsigned int max_hops = 0;
  char max_path[255]{};

  unsigned long time_start = 0;
};

class MyMesh : public BaseChatMesh, ContactVisitor {
  FILESYSTEM *_fs;
  NodePrefs _prefs;
  NodeStats _stats;
  ChannelDetails *_public;

  unsigned long last_msg_sent = 0;
  unsigned long last_msg_rcvd = 0;
  unsigned long last_msg_times[QUIET_LIMIT_TIMES];
  unsigned int last_msg_count = 0;
  unsigned long time_start = 0;

  char repeaters_names[255][33];

  char command[512 + 10];
  uint8_t tmp_buf[256];
  char hex_buf[512];

  bool clock_set = false;
  bool quiet = false;

  char message[256];

  void loadContacts();
  void saveContacts();
  void saveStats();
  void loadStats();
  void setClock(uint32_t timestamp);

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

  void onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t timestamp,
                            const char *text) override;

  float getAirtimeBudgetFactor() const override { return _prefs.airtime_factor; }

  int calcRxDelay(const float score, const uint32_t air_time) const override {
    if (_prefs.rx_delay_base <= 0.0f) return 0;
    return static_cast<int>((pow(_prefs.rx_delay_base, 0.85f - score) - 1.0) * air_time);
  }

  bool allowPacketForward(const mesh::Packet *packet) override { return true; }

  void onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t *path) override;

  void onContactPathUpdated(const ContactInfo &contact) override {
    Serial.printf("PATH to: %s, path_len=%d\n", contact.name, static_cast<int32_t>(contact.out_path_len));
    saveContacts();
  }

  ContactInfo *processAck(const uint8_t *data) override {
    // not supported
    return nullptr;
  }

  void onMessageRecv(const ContactInfo &from, mesh::Packet *pkt, const uint32_t sender_timestamp,
                     const char *text) override {
    // not supported
  }

  void onCommandDataRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                         const char *text) override {
    // not supported
  }
  void onSignedMessageRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                           const uint8_t *sender_prefix, const char *text) override {
    // not supported
  }

  uint8_t onContactRequest(const ContactInfo &contact, uint32_t sender_timestamp, const uint8_t *data,
                           uint8_t len, uint8_t *reply) override {
    return 0; // unknown
  }

  void onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len) override {
    // not supported
  }

  uint32_t calcFloodTimeoutMillisFor(const uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + FLOOD_SEND_TIMEOUT_FACTOR * static_cast<float>(pkt_airtime_millis);
  }
  uint32_t calcDirectTimeoutMillisFor(const uint32_t pkt_airtime_millis,
                                      const uint8_t path_len) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (static_cast<float>(pkt_airtime_millis) * DIRECT_SEND_PERHOP_FACTOR +
                                       DIRECT_SEND_PERHOP_EXTRA_MILLIS) *
                                          static_cast<float>(path_len + 1);
  }

  void onSendTimeout() override { Serial.println("   ERROR: timed out, no ACK."); }

public:
  MyMesh(mesh::Radio &radio, StdRNG &rng, mesh::RTCClock &rtc, SimpleMeshTables &tables);

  bool getQuiet() const { return quiet; }

  unsigned long getTotalReceived() const { return _stats.total_received; }

  unsigned long getTotalRequested() const { return _stats.total_request; }

  unsigned long getTotalSent() const { return _stats.total_sent; }

  unsigned long getTenReceived() const { return last_msg_count; }

  void onContactVisit(const ContactInfo &contact) override {
    // not supported
  }

  bool shouldOverwriteWhenFull() const override { return true; }

  void begin(FILESYSTEM &fs);
  void sendSelfAdvert(int delay_millis);
  void sendMessage(const char *message);
  void handleCommand(const char *command);
  void loop();
};