#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>
#include <helpers/CommonCLI.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <RTClib.h>
#include <target.h>

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#define FIRMWARE_VER_TEXT     "v1.0.0"
#define FIRMWARE_BUILD_TEXT   "2026-02-25"

#define LORA_FREQ      868.856
#define LORA_BW        62.5
#define LORA_SF        8
#define LORA_CR        5
#define LORA_TX_POWER  10

#include <helpers/BaseChatMesh.h>

#define SEND_TIMEOUT_BASE_MILLIS          500
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   250

#define MAX_GROUP_CHANNELS  1

#define QUIET_LIMIT_SECONDS 5
#define QUIET_LIMIT_TIME    5 // minutes
#define QUIET_LIMIT_COUNT   20
#define QUIET_LIMIT_TIMES   255

#define  BOT_NAME           "Mr.Pong🏓"
#define  PUBLIC_GROUP_NAME  "#test" // #test
#define  PUBLIC_GROUP_PSK   "nNj88ipHMztZHZaiuEi3Pw==" // #test

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

/* -------------------------------------------------------------------------------------- */

class MyMesh : public BaseChatMesh, ContactVisitor {
  FILESYSTEM* _fs;
  NodePrefs _prefs;
  uint32_t expected_ack_crc;
  ChannelDetails* _public;
  unsigned long last_msg_sent;
  unsigned long last_msg_rcvd;
  unsigned long last_msg_times[QUIET_LIMIT_TIMES];
  unsigned long last_msg_count = 0;

  unsigned long total_request = 0;
  unsigned long total_received = 0;
  unsigned long total_sent = 0;

  char command[512+10];
  uint8_t tmp_buf[256];
  char hex_buf[512];
  bool clock_set = false;
  bool quiet = false;

  char message[256];

  void setClock(const uint32_t timestamp) const {
    const uint32_t curr = getRTCClock()->getCurrentTime();
    if (timestamp > curr) {
      getRTCClock()->setCurrentTime(timestamp);
      Serial.println("   (OK - clock set!)");
    } else {
      Serial.println("   (ERR: clock cannot go backwards)");
    }
  }

protected:
  float getAirtimeBudgetFactor() const override {
    return _prefs.airtime_factor;
  }

  int calcRxDelay(float score, uint32_t air_time) const override {
    if (_prefs.rx_delay_base <= 0.0f) return 0;
    return (int)((pow(_prefs.rx_delay_base, 0.85f - score) - 1.0) * air_time);
  }

  bool allowPacketForward(const mesh::Packet* packet) override {
    return true;
  }

  void onDiscoveredContact(ContactInfo& contact, bool is_new, uint8_t path_len, const uint8_t* path) override {
    // not supported
  }

  void onContactPathUpdated(const ContactInfo& contact) override {
    // not supported
  }

  ContactInfo* processAck(const uint8_t *data) override {
    if (memcmp(data, &expected_ack_crc, 4) == 0) {     // got an ACK from recipient
      Serial.printf("   Got ACK! (round trip: %d millis)\n", _ms->getMillis() - last_msg_sent);
      // NOTE: the same ACK can be received multiple times!
      expected_ack_crc = 0;  // reset our expected hash, now that we have received ACK
      return nullptr;  // TODO: really should return ContactInfo pointer
    }
    return nullptr;
  }

  void onMessageRecv(const ContactInfo& from, mesh::Packet* pkt, const uint32_t sender_timestamp, const char *text) override {
    // not supported
  }

  void onCommandDataRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
    // not supported
  }
  void onSignedMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text) override {
    // not supported
  }

  static const char* hop_word(const int hop) {
    int n = hop % 100;
    if (n >= 11 && n <= 14)
      return "хопов";
    n = hop % 10;
    if (n == 1)
      return "хоп";
    if (n >= 2 && n <= 4)
      return "хопа";
    return "хопов";
  }

  void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, const uint32_t timestamp, const char *text) override {
    Serial.printf("   %s\n", text);
    if (!clock_set) {
      setClock(timestamp + 1);
      clock_set = true;
    }

    total_received++;
    last_msg_rcvd = _ms->getMillis();

    int j = 0;
    for (int i = 0; i < last_msg_count; i++) {
      if (last_msg_rcvd - last_msg_times[i] < QUIET_LIMIT_TIME * 60 * 1000) {
        last_msg_times[j++] = last_msg_times[i];
      }
    }
    last_msg_count = j;

    if (last_msg_count < QUIET_LIMIT_TIMES) {
      last_msg_times[last_msg_count++] = last_msg_rcvd;
    }

    quiet = last_msg_count >= QUIET_LIMIT_COUNT;

    char _from[100], _text[200];
    sscanf(text, "%99[^:]: %199s", _from, _text);
    if (strcmp(_text, "ping") == 0 || strcmp(_text, "Ping") == 0 || strcmp(_text, "test") == 0 || strcmp(_text, "Test") == 0 || strcmp(_text, "пинг") == 0 || strcmp(_text, "Пинг") == 0 || strcmp(_text, "тест") == 0 || strcmp(_text, "Тест") == 0) {
      if (pkt->isRouteDirect() || pkt->path_len == 0) {
        sprintf(message, "@[%s] директ c SNR %03.2f dB", _from, pkt->getSNR());
        Serial.printf("%s\n", message);
      } else {
        char _path[3 * pkt->path_len + 1];
        int8_t offset = 0;
        for (size_t i = 0; i < pkt->path_len; i++) {
          offset += snprintf(_path + offset, sizeof(_path) - offset, "%02X,", pkt->path[i]);
        }

        if (offset > 0) {
          _path[offset - 1] = '\0';
        }
        sprintf(message, "@[%s] %d %s: %s", _from, pkt->path_len, hop_word(pkt->path_len), _path);
        Serial.printf("%s\n", message);
      }
      total_request++;
      if (!quiet && _ms->getMillis() - last_msg_sent > QUIET_LIMIT_SECONDS * 1000) { // QUIET_LIMIT_SECONDS sec
        sendMessage(message);
        last_msg_sent = _ms->getMillis();
        total_sent++;
      } else {
        Serial.printf("Quiet please!\n");
      }
    }
  }

  uint8_t onContactRequest(const ContactInfo& contact, uint32_t sender_timestamp, const uint8_t* data, uint8_t len, uint8_t* reply) override {
    return 0;  // unknown
  }

  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override {
    // not supported
  }

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
  }
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (pkt_airtime_millis * DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (path_len + 1);
  }

  void onSendTimeout() override {
    Serial.println("   ERROR: timed out, no ACK.");
  }

public:
  MyMesh(mesh::Radio& radio, StdRNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
     : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
    // defaults
    memset(&_prefs, 0, sizeof(_prefs));
    _prefs.airtime_factor = 2.0;    // one third
    strcpy(_prefs.node_name, BOT_NAME);
    _prefs.freq = LORA_FREQ;
    _prefs.bw = LORA_BW;
    _prefs.sf = LORA_SF;
    _prefs.cr = LORA_CR;
    _prefs.tx_power_dbm = LORA_TX_POWER;
    _prefs.rx_delay_base = 1.0;
    _prefs.node_lat = 0.0;
    _prefs.node_lon = 0.0;
    _prefs.powersaving_enabled = false;

    command[0] = 0;
    clock_set = false;
  }

  bool getQuiet() const {
    return quiet;
  }

  unsigned long getTotalReceived() const {
    return total_received;
  }

  unsigned long getTotalRequested() const {
    return total_request;
  }

  unsigned long getTotalSent() const {
    return total_sent;
  }

  unsigned long getTenReceived() const {
    return last_msg_count;
  }

  void begin(FILESYSTEM& fs) {
    _fs = &fs;

    BaseChatMesh::begin();

  #if defined(NRF52_PLATFORM)
    IdentityStore store(fs, "");
  #elif defined(RP2040_PLATFORM)
    IdentityStore store(fs, "/identity");
    store.begin();
  #else
    IdentityStore store(fs, "/identity");
  #endif
    if (!store.load("_main", self_id, _prefs.node_name, sizeof(_prefs.node_name))) {  // legacy: node_name was from identity file
      // Need way to get some entropy to seed RNG
      Serial.println("Press ENTER to generate key:");
      char c = 0;
      while (c != '\n') {   // wait for ENTER to be pressed
        if (Serial.available()) c = Serial.read();
      }
      ((StdRNG *)getRNG())->begin(millis());

      self_id = mesh::LocalIdentity(getRNG());  // create new random identity
      int count = 0;
      while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
        self_id = mesh::LocalIdentity(getRNG()); count++;
      }
      store.save("_main", self_id);
    }

    _public = addChannel(PUBLIC_GROUP_NAME, PUBLIC_GROUP_PSK); // pre-configure public channel
  }

  void sendSelfAdvert(const int delay_millis) {
    const auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
    if (pkt) {
      sendFlood(pkt, delay_millis);
    }
  }

  void onContactVisit(const ContactInfo& contact) override {
  }

  void sendMessage(const char* message) {
    uint8_t temp[5+MAX_TEXT_LEN+32];
    const uint32_t timestamp = getRTCClock()->getCurrentTime();
    memcpy(temp, &timestamp, 4);   // mostly an extra blob to help make packet_hash unique
    temp[4] = 0;  // attempt and flags

    sprintf((char *) &temp[5], "%s: %s", _prefs.node_name, &message[0]);  // <sender>: <msg>
    temp[5 + MAX_TEXT_LEN] = 0;  // truncate if too long

    const int len = strlen((char *) &temp[5]);
    const auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, _public->channel, temp, 5 + len);
    if (pkt) {
      sendFlood(pkt);
      Serial.println("   Sent.");
    } else {
      Serial.println("   ERROR: unable to send");
    }
  }

  void handleCommand(const char* command) {
    while (*command == ' ') command++;  // skip leading spaces

    if (memcmp(command, "public ", 7) == 0) {
      // send GroupChannel msg
      sendMessage(&command[7]);
    } else if (strcmp(command, "clock") == 0) {    // show current time
      const uint32_t now = getRTCClock()->getCurrentTime();
      const auto dt = DateTime(now);
      Serial.printf(   "%02d:%02d - %d/%d/%d UTC\n", dt.hour(), dt.minute(), dt.day(), dt.month(), dt.year());
    } else if (memcmp(command, "time ", 5) == 0) {  // set time (to epoch seconds)
      const uint32_t secs = _atoi(&command[5]);
      setClock(secs);
      clock_set = true;
    } else if (strcmp(command, "quiet") == 0) {
      quiet = true;
      Serial.println("   (quiet set).");
    } else if (strcmp(command, "advert") == 0) {
      const auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
      if (pkt) {
        sendZeroHop(pkt);
        Serial.println("   (advert sent, zero hop).");
      } else {
        Serial.println("   ERR: unable to send");
      }
    } else if (memcmp(command, "ver", 3) == 0) {
      Serial.println(FIRMWARE_VER_TEXT);
    } else if (memcmp(command, "reboot", 6) == 0) {
      board.reboot();
    } else if (memcmp(command, "shutdown", 8) == 0) {
      display.turnOff();
      radio_driver.powerOff();
      board.powerOff();
    } else if (memcmp(command, "help", 4) == 0) {
      Serial.println("Commands:");
      Serial.println("   clock");
      Serial.println("   time <epoch-seconds>");
      Serial.println("   advert");
      Serial.println("   quiet");
      Serial.println("   reboot");
      Serial.println("   shutdown");
      Serial.println("   ver");
      Serial.println("   public <text>");
    } else {
      Serial.print("   ERROR: unknown command: "); Serial.println(command);
    }
  }

  void loop() {
    BaseChatMesh::loop();

    unsigned int len = strlen(command);
    while (Serial.available() && len < sizeof(command)-1) {
      const char c = Serial.read();
      if (c != '\n') { 
        command[len++] = c;
        command[len] = 0;
      }
      Serial.print(c);
    }
    if (len == sizeof(command)-1) {  // command buffer full
      command[sizeof(command)-1] = '\r';
    }

    if (len > 0 && command[len - 1] == '\r') {  // received complete line
      command[len - 1] = 0;  // replace newline with C string null terminator

      handleCommand(command);
      command[0] = 0;  // reset command buffer
    }
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables);

void halt() {
  while (true);
}

void setup() {
  Serial.begin(115200);

  board.begin();

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_get_rng_seed());

#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  the_mesh.begin(InternalFS);
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  the_mesh.begin(LittleFS);
#elif defined(ESP32)
  SPIFFS.begin(true);
  the_mesh.begin(SPIFFS);
#else
  #error "need to define filesystem"
#endif

#ifdef DISPLAY_CLASS
  ui_task.begin(BOT_NAME, PUBLIC_GROUP_NAME);
#endif

  radio_set_params(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
  radio_set_tx_power(LORA_TX_POWER);
}

void loop() {
  the_mesh.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop(the_mesh.getQuiet(), the_mesh.getTotalRequested(), the_mesh.getTotalSent(), the_mesh.getTotalReceived(), the_mesh.getTenReceived());
#endif
  rtc_clock.tick();
}
