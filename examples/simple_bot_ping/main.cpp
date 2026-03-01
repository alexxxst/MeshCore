#include <Arduino.h>   // needed for PlatformIO
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

#define FIRMWARE_VER_TEXT     "v1.0.4"
#define FIRMWARE_BUILD_TEXT   "2026-03-02"

#define LORA_FREQ      868.856
#define LORA_BW        62.5
#define LORA_SF        8
#define LORA_CR        5
#define LORA_TX_POWER  10

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS         350
#endif

#include <helpers/BaseChatMesh.h>

#define SEND_TIMEOUT_BASE_MILLIS          500
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   250

#define MAX_GROUP_CHANNELS  1

#define QUIET_LIMIT_SECONDS 5     // seconds to cooldown
#define QUIET_LIMIT_TIME    5     // minutes to check
#define QUIET_LIMIT_COUNT   20    // messages to check
#define QUIET_LIMIT_TIMES   255   // overall limit for timestamps array
#define QUIET_LIMIT_PAUSE   0.3f  // seconds to reply

#define  BOT_NAME           "Mr.Pong🏓"
#define  BOT_NAME_PLAIN     "Mr.Pong"
#define  PUBLIC_GROUP_NAME  "#bot" // #bot
#define  PUBLIC_GROUP_PSK   "61ChvLPk5de/aaV8na2iEQ==" // #bot's channel PSK

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
  unsigned long last_msg_sent = 0;
  unsigned long last_msg_rcvd = 0;
  unsigned long last_msg_times[QUIET_LIMIT_TIMES];
  unsigned long last_msg_count = 0;

  char repeaters_names[255][33];
  unsigned long first_repeaters_count[255];
  unsigned long all_repeaters_count[255];

  unsigned long total_request = 0;
  unsigned long total_received = 0;
  unsigned long total_sent = 0;
  unsigned long total_thanks = 0;
  unsigned long total_ignores = 0;
  unsigned long total_hops = 0;

  unsigned long time_start = 0;

  char command[512+10];
  uint8_t tmp_buf[256];
  char hex_buf[512];
  bool clock_set = false;
  bool quiet = false;

  char message[256];

  static const char* getTypeName(const uint8_t type) {
    if (type == ADV_TYPE_CHAT) return "Chat";
    if (type == ADV_TYPE_REPEATER) return "Repeater";
    if (type == ADV_TYPE_ROOM) return "Room";
    return "??";  // unknown
  }

  void loadContacts() {
    if (_fs->exists("/contacts")) {
#if defined(RP2040_PLATFORM)
      File file = _fs->open("/contacts", "r");
#else
      File file = _fs->open("/contacts");
#endif
      if (file) {
        bool full = false;
        while (!full) {
          ContactInfo c;
          uint8_t pub_key[32];
          uint8_t unused;
          uint32_t reserved;

          bool success = (file.read(pub_key, 32) == 32);
          success = success && (file.read(&c.name, 32) == 32);
          success = success && (file.read(&c.type, 1) == 1);
          success = success && (file.read(&c.flags, 1) == 1);
          success = success && (file.read(&unused, 1) == 1);
          success = success && (file.read(&reserved, 4) == 4);
          success = success && (file.read(&c.out_path_len, 1) == 1);
          success = success && (file.read(&c.last_advert_timestamp, 4) == 4);
          success = success && (file.read(c.out_path, 64) == 64);
          c.gps_lat = c.gps_lon = 0;   // not yet supported

          if (!success) break;  // EOF

          c.id = mesh::Identity(pub_key);
          c.lastmod = 0;

          if (c.type == ADV_TYPE_REPEATER) {
            snprintf(repeaters_names[c.id.pub_key[0]], 32, c.name);
          }

          if (!addContact(c)) full = true;
        }
        file.close();
      }
    }
  }

  void saveContacts() {
#if defined(NRF52_PLATFORM)
    _fs->remove("/contacts");
    File file = _fs->open("/contacts", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File file = _fs->open("/contacts", "w");
#else
    File file = _fs->open("/contacts", "w", true);
#endif
    if (file) {
      ContactsIterator iter;
      ContactInfo c;
      constexpr uint8_t unused = 0;
      uint32_t reserved = 0;

      while (iter.hasNext(this, c)) {
        if (c.type == ADV_TYPE_REPEATER) {
          bool success = (file.write(c.id.pub_key, 32) == 32);
          success = success && (file.write(reinterpret_cast<uint8_t *>(&c.name), 32) == 32);
          success = success && (file.write(&c.type, 1) == 1);
          success = success && (file.write(&c.flags, 1) == 1);
          success = success && (file.write(&unused, 1) == 1);
          success = success && (file.write(reinterpret_cast<uint8_t *>(&reserved), 4) == 4);
          success = success && (file.write(reinterpret_cast<uint8_t *>(&c.out_path_len), 1) == 1);
          success = success && (file.write(reinterpret_cast<uint8_t *>(&c.last_advert_timestamp), 4) == 4);
          success = success && (file.write(c.out_path, 64) == 64);

          snprintf(repeaters_names[c.id.pub_key[0]], 32, c.name);

          if (!success) break;  // write failed
        }
      }
      file.close();
    }
  }

  void setClock(const uint32_t timestamp) {
    const uint32_t curr = getRTCClock()->getCurrentTime();
    if (timestamp > curr) {
      getRTCClock()->setCurrentTime(timestamp);
      time_start = timestamp;
      Serial.println("   (OK - clock set!)");
    } else {
      Serial.println("   (ERR: clock cannot go backwards)");
    }
  }

protected:
  float getAirtimeBudgetFactor() const override {
    return _prefs.airtime_factor;
  }

  int calcRxDelay(const float score, const uint32_t air_time) const override {
    if (_prefs.rx_delay_base <= 0.0f) return 0;
    return static_cast<int>((pow(_prefs.rx_delay_base, 0.85f - score) - 1.0) * air_time);
  }

  bool allowPacketForward(const mesh::Packet* packet) override {
    return true;
  }

  void onDiscoveredContact(ContactInfo& contact, bool is_new, uint8_t path_len, const uint8_t* path) override {
    Serial.printf("ADVERT from -> %s\n", contact.name);
    Serial.printf("  type: %s\n", getTypeName(contact.type));
    Serial.print("   public key: "); mesh::Utils::printHex(Serial, contact.id.pub_key, PUB_KEY_SIZE); Serial.println();
    saveContacts();
  }

  void onContactPathUpdated(const ContactInfo& contact) override {
    Serial.printf("PATH to: %s, path_len=%d\n", contact.name, static_cast<int32_t>(contact.out_path_len));
    saveContacts();
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

  static void format_uptime(uint32_t seconds, char *buf, const size_t buf_size)
  {
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

    message[0] = 0;
    char _from[100], _text[200];
    if (sscanf(text, "%99[^:]: %199[^\0]", _from, _text) > 0) {

      // ping
      if (strcmp(_text, "ping") == 0 || strcmp(_text, "Ping") == 0 || strcmp(_text, "test") == 0 || strcmp(_text, "Test") == 0 || strcmp(_text, "пинг") == 0 || strcmp(_text, "Пинг") == 0 || strcmp(_text, "тест") == 0 || strcmp(_text, "Тест") == 0) {
        if (pkt->isRouteDirect() || pkt->path_len == 0) {
          sprintf(message, "@[%s] директ c SNR %03.2f dB", _from, pkt->getSNR());
        } else {
          char _path[3 * pkt->path_len + 1];
          unsigned int offset = 0;
          for (size_t i = 0; i < pkt->path_len; i++) {
            offset += snprintf(_path + offset, sizeof(_path) - offset, "%02X,", pkt->path[i]);
            if (i == 0) {
              // first repeaters count
              first_repeaters_count[pkt->path[i]]++;
            }
            all_repeaters_count[pkt->path[i]]++;
          }

          if (offset > 0) {
            _path[offset - 1] = '\0';
          }
          sprintf(message, "@[%s] %d %s с %s: %s", _from, pkt->path_len, hop_word(pkt->path_len), repeaters_names[pkt->path[0]], _path);
          total_hops = total_hops + pkt->path_len;
        }
      }

      // to bot command
      if (strstr(_text, BOT_NAME_PLAIN) != nullptr) {

        // stats
        if (strstr(_text, "stats") != nullptr || strstr(_text, "статистика") != nullptr || strstr(_text, "Stats") != nullptr || strstr(_text, "Статистика") != nullptr) {
          char uptime[32];
          format_uptime(getRTCClock()->getCurrentTime() - time_start - 1, uptime, sizeof(uptime));
          int reps = 0;
          for (const unsigned long v : all_repeaters_count) {
            if (v > 0) {
              reps++;
            }
          }
          sprintf(message, "Стата:\n uptime: %s\n ответы: %d из %d\n канал: %d, за %dм: %d\n хопы: %d, репы: %d", uptime, total_sent, total_request, total_received, QUIET_LIMIT_TIME, last_msg_count, total_hops, reps);
        }

        // thanks
        if (strstr(_text, "спасибо") != nullptr || strstr(_text, "thank") != nullptr || strstr(_text, "Спасибо") != nullptr || strstr(_text, "Thank") != nullptr) {
          total_thanks++;
          sprintf(message, "@[%s] вот так нихера себе, кто-то сказал спасибо №%d!", _from, total_thanks);
        }

        // weather
        if (strstr(_text, "погода") != nullptr || strstr(_text, "weather") != nullptr || strstr(_text, "Погода") != nullptr || strstr(_text, "Weather") != nullptr) {
          sprintf(message, "@[%s] бля, ну в окно выгляни! Какой смысл от моих датчиков?", _from);
        }

        // repeaters
        if (strstr(_text, "репитеры") != nullptr || strstr(_text, "repeaters") != nullptr || strstr(_text, "Репитеры") != nullptr || strstr(_text, "Repeaters") != nullptr) {

          unsigned long o_max1 = 0, o_max2 = 0, o_max3 = 0;
          int o_idx1 = -1, o_idx2 = -1, o_idx3 = -1;

          for (int i = 0; i < 255; i++) {
            const unsigned long o_val = first_repeaters_count[i];

            if (o_val > o_max1) {
              o_max3 = o_max2; o_idx3 = o_idx2;
              o_max2 = o_max1; o_idx2 = o_idx1;
              o_max1 = o_val;  o_idx1 = i;
            } else if (o_val > o_max2) {
              o_max3 = o_max2; o_idx3 = o_idx2;
              o_max2 = o_val;  o_idx2 = i;
            } else if (o_val > o_max3) {
              o_max3 = o_val;  o_idx3 = i;
            }
          }

          if (o_idx3 > 0) {
            sprintf(message, "Исходящие топ-3 репы:\n%02X %s - %d\n%02X %s - %d\n%02X %s - %d", o_idx1, repeaters_names[o_idx1], o_max1, o_idx2, repeaters_names[o_idx2], o_max2, o_idx3, repeaters_names[o_idx3], o_max3);
          } else {
            sprintf(message, "@[%s] сорян, пока не набрался топ репитеров в этом канале", _from);
          }
        }

      }
    }

    if (message[0] != 0) {
      total_request++;
      Serial.printf("%s\n", message);
      if (!quiet && _ms->getMillis() - last_msg_sent > QUIET_LIMIT_SECONDS * 1000) { // QUIET_LIMIT_SECONDS sec
        // pause for QUIET_LIMIT_PAUSE seconds before reply
        delay(QUIET_LIMIT_PAUSE * 1000);
        sendMessage(message);
        last_msg_sent = _ms->getMillis();
        total_sent++;
      } else {
        Serial.printf("Quiet please!\n");
        total_ignores++;
      }
    }
  }

  uint8_t onContactRequest(const ContactInfo& contact, uint32_t sender_timestamp, const uint8_t* data, uint8_t len, uint8_t* reply) override {
    return 0;  // unknown
  }

  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override {
    // not supported
  }

  uint32_t calcFloodTimeoutMillisFor(const uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + FLOOD_SEND_TIMEOUT_FACTOR * static_cast<float>(pkt_airtime_millis);
  }
  uint32_t calcDirectTimeoutMillisFor(const uint32_t pkt_airtime_millis, const uint8_t path_len) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (static_cast<float>(pkt_airtime_millis) * DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * static_cast<float>(path_len + 1);
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
    message[0] = 0;
    clock_set = false;

    for (size_t i = 0; i < 255; i++) {
      first_repeaters_count[i] = 0;
      all_repeaters_count[i] = 0;
      snprintf(repeaters_names[i], 32, "UnknownRepeater");
    }
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
      ((StdRNG *)getRNG())->begin(static_cast<long>(millis()));

      self_id = mesh::LocalIdentity(getRNG());  // create new random identity
      int count = 0;
      while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
        self_id = mesh::LocalIdentity(getRNG()); count++;
      }
      store.save("_main", self_id);
    }

    loadContacts();
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

    sprintf(reinterpret_cast<char *>(&temp[5]), "%s: %s", _prefs.node_name, &message[0]);  // <sender>: <msg>
    temp[5 + MAX_TEXT_LEN] = 0;  // truncate if too long

    const unsigned int len = strlen(reinterpret_cast<char *>(&temp[5]));
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
    } else if (memcmp(command, "stats", 5) == 0) {
      char uptime[32];
      format_uptime(getRTCClock()->getCurrentTime() - time_start - 1, uptime, sizeof(uptime));
      sprintf(message, "Bot stats:\n uptime: %s\n requests: %d\n replies: %d\n for %dm: %d\n thanks: %d\n ignores: %d\n total: %d\n hops: %d", uptime, total_request, total_sent, QUIET_LIMIT_TIME, last_msg_count, total_thanks, total_ignores, total_received, total_hops);
      Serial.println(message);
    } else if (memcmp(command, "repeaters", 9) == 0) {
        for (int i = 0; i < 255; i++) {
          sprintf(message, "%02X - %s - %d", i, repeaters_names[i], all_repeaters_count[i]);
          Serial.println(message);
        }
    } else if (memcmp(command, "shutdown", 8) == 0) {
      display.turnOff();
      radio_driver.powerOff();
      board.powerOff();
    } else if (memcmp(command, "help", 4) == 0) {
      Serial.println("Commands:");
      Serial.println("   clock");
      Serial.println("   time <epoch-seconds>");
      Serial.println("   advert");
      Serial.println("   stats");
      Serial.println("   repeaters");
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

[[noreturn]] void halt() {
  while (true);
}

void setup() {
  Serial.begin(115200);

  board.begin();

  if (!radio_init()) { halt(); }

  fast_rng.begin(static_cast<long>(radio_get_rng_seed()));

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
  ui_task.begin(BOT_NAME_PLAIN, PUBLIC_GROUP_NAME);
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
