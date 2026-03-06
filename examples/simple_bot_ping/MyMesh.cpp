#include "MyMesh.h"
#include "RTClib.h"

void MyMesh::loadContacts() {
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
        success = success && (file.read(reinterpret_cast<uint8_t *>(&c.name), 32) == 32);
        success = success && (file.read(&c.type, 1) == 1);
        success = success && (file.read(&c.flags, 1) == 1);
        success = success && (file.read(&unused, 1) == 1);
        success = success && (file.read(reinterpret_cast<uint8_t *>(&reserved), 4) == 4);
        success = success && (file.read(&c.out_path_len, 1) == 1);
        success = success && (file.read(reinterpret_cast<uint8_t *>(&c.last_advert_timestamp), 4) == 4);
        success = success && (file.read(c.out_path, 64) == 64);
        c.gps_lat = c.gps_lon = 0; // not yet supported

        if (!success) break; // EOF

        c.id = mesh::Identity(pub_key);
        c.lastmod = 0;

        if (c.type == ADV_TYPE_REPEATER && checkRepeaterNamePattern(c.name)) {
          sprintf(repeaters_names[pub_key[0]], "%s", c.name);
          // sprintf(message, "Loaded %02X %s", pub_key[0], c.name);
          // Serial.println(message);
        }

        if (!addContact(c)) full = true;
      }
      file.close();
    }
  }
}

void MyMesh::saveContacts() {
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
      if (c.type == ADV_TYPE_REPEATER && checkRepeaterNamePattern(c.name)) {
        bool success = (file.write(c.id.pub_key, 32) == 32);
        success = success && (file.write(reinterpret_cast<uint8_t *>(&c.name), 32) == 32);
        success = success && (file.write(&c.type, 1) == 1);
        success = success && (file.write(&c.flags, 1) == 1);
        success = success && (file.write(&unused, 1) == 1);
        success = success && (file.write(reinterpret_cast<uint8_t *>(&reserved), 4) == 4);
        success = success && (file.write(reinterpret_cast<uint8_t *>(&c.out_path_len), 1) == 1);
        success = success && (file.write(reinterpret_cast<uint8_t *>(&c.last_advert_timestamp), 4) == 4);
        success = success && (file.write(c.out_path, 64) == 64);

        if (success) {
          sprintf(repeaters_names[c.id.pub_key[0]], "%s", c.name);
          // sprintf(message, "Saved %02X %s", c.id.pub_key[0], c.name);
          // Serial.println(message);
        }

        if (!success) break; // write failed
      }
    }
    file.close();
  }
}

void MyMesh::saveStats() {
#if defined(NRF52_PLATFORM)
  _fs->remove("/node_stats");
  File file = _fs->open("/node_stats", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  File file = _fs->open("/node_stats", "w");
#else
  File file = _fs->open("/node_stats", "w", true);
#endif
  if (file) {
    file.write(reinterpret_cast<const uint8_t *>(&_stats), sizeof(_stats));
    file.close();
  }
}

void MyMesh::loadStats() {
  if (_fs->exists("/node_stats")) {
#if defined(RP2040_PLATFORM)
    File file = _fs->open("/node_stats", "r");
#else
    File file = _fs->open("/node_stats");
#endif
    if (file) {
      file.read(reinterpret_cast<uint8_t *>(&_stats), sizeof(_stats));
      file.close();
    }
  }
}

void MyMesh::setClock(const uint32_t timestamp) {
  const uint32_t curr = getRTCClock()->getCurrentTime();
  if (timestamp > curr) {
    getRTCClock()->setCurrentTime(timestamp);
    time_start = timestamp;
    Serial.println("   (OK - clock set!)");
  } else {
    Serial.println("   (ERR: clock cannot go backwards)");
  }
}

void MyMesh::onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt,
                                  const uint32_t timestamp, const char *text) {
  Serial.printf("   %s\n", text);
  if (!clock_set) {
    setClock(timestamp + 1);
    clock_set = true;
  }

  if (_stats.time_start == 0 && clock_set) {
    _stats.time_start = timestamp + 1;
  }

  _stats.total_received++;
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
    if (strncasecmp(_text, "ping", 4) == 0 || strncasecmp(_text, "test", 4) == 0 ||
        strncmp(_text, "пинг", 8) == 0 || strncmp(_text, "Пинг", 8) == 0 || strncmp(_text, "тест", 8) == 0 ||
        strncmp(_text, "Тест", 8) == 0) {
      if (pkt->isRouteDirect() || pkt->path_len == 0) {
        sprintf(message, "@[%s] диpeкт c SNR %03.2f dB", _from, pkt->getSNR());
      } else {
        char _path[3 * pkt->path_len + 1];
        unsigned int offset = 0;
        for (size_t i = 0; i < pkt->path_len; i++) {
          offset += snprintf(_path + offset, sizeof(_path) - offset, "%02X,", pkt->path[i]);
          if (i == 0) {
            // first repeaters count
            _stats.first_repeaters_count[pkt->path[i]]++;
          }
          _stats.all_repeaters_count[pkt->path[i]]++;
        }

        if (offset > 0) {
          _path[offset - 1] = '\0';
        }

        if (pkt->path_len >= _stats.max_hops) {
          _stats.max_hops = pkt->path_len;
          sprintf(_stats.max_path, "%s в %d %s: %s", _from, pkt->path_len, hop_word(pkt->path_len), _path);
        }
        sprintf(message, "@[%s] %d %s c %s: %s", _from, pkt->path_len, hop_word(pkt->path_len),
                repeaters_names[pkt->path[0]], _path);
        _stats.total_hops = _stats.total_hops + pkt->path_len;
      }
    }

    // to bot command
    if (strstr(_text, BOT_NAME_PLAIN) != nullptr) {

      // stats
      if (strstr(_text, "stats") != nullptr || strstr(_text, "статистика") != nullptr ||
          strstr(_text, "Stats") != nullptr || strstr(_text, "Статистика") != nullptr) {
        char uptime[32];
        format_uptime(getRTCClock()->getCurrentTime() - _stats.time_start - 1, uptime, sizeof(uptime));
        int reps = 0;
        for (const unsigned int v : _stats.all_repeaters_count) {
          if (v > 0) {
            reps++;
          }
        }
        sprintf(message,
                "Cтaтa зa: %s\n oтвeты: %d из %d\n кaнaл: %d, зa %dм: %d\n хoпы: %d, peпы: %d", uptime,
                _stats.total_sent, _stats.total_request, _stats.total_received, QUIET_LIMIT_TIME,
                last_msg_count, _stats.total_hops, reps);
      }

      // uptime
      if (strstr(_text, "uptime") != nullptr || strstr(_text, "аптайм") != nullptr ||
          strstr(_text, "Uptime") != nullptr || strstr(_text, "Аптайм") != nullptr) {
        char uptime[32];
        format_uptime(getRTCClock()->getCurrentTime() - time_start - 1, uptime, sizeof(uptime));
        sprintf(message, "Uptime %s", uptime);
          }

      // thanks
      if (strstr(_text, "спасибо") != nullptr || strstr(_text, "thank") != nullptr ||
          strstr(_text, "Спасибо") != nullptr || strstr(_text, "Thank") != nullptr) {
        _stats.total_thanks++;
        sprintf(message, "@[%s] вoт тaк нихepa ceбe, ктo-тo cкaзaл cпacибo №%d!", _from, _stats.total_thanks);
      }

      // path max
      if (strstr(_text, "рекорд") != nullptr || strstr(_text, "record") != nullptr ||
          strstr(_text, "Рекорд") != nullptr || strstr(_text, "Record") != nullptr) {
        if (_stats.max_hops > 5) {
          sprintf(message, "%s", _stats.max_path);
        } else {
          sprintf(message, "@[%s] copян, пoкa нe зaфикcиpoвaн длинный пyть в этoм кaнaлe", _from);
        }
      }

      // weather
      if (strstr(_text, "погода") != nullptr || strstr(_text, "weather") != nullptr ||
          strstr(_text, "Погода") != nullptr || strstr(_text, "Weather") != nullptr) {
        sprintf(message, "@[%s] бля, нy в oкнo выгляни! Kaкoй cмыcл oт мoиx дaтчикoв?", _from);
      }

      // бля
      if (strstr(_text, "бля") != nullptr || strstr(_text, "Бля") != nullptr) {
        sprintf(message, "@[%s] oт бля cлышy.", _from);
          }

      // meow
      if (strstr(_text, "мяу") != nullptr || strstr(_text, "meow") != nullptr ||
          strstr(_text, "Мяу") != nullptr || strstr(_text, "Meow") != nullptr) {
        sprintf(message, "@[%s] кc-кc-кc, нy иди cюдa, пoглaжy!", _from);
          }

      // repeaters
      if (strstr(_text, "репитеры") != nullptr || strstr(_text, "repeaters") != nullptr ||
          strstr(_text, "Репитеры") != nullptr || strstr(_text, "Repeaters") != nullptr) {

        unsigned int o_max1 = 0, o_max2 = 0, o_max3 = 0;
        int o_idx1 = -1, o_idx2 = -1, o_idx3 = -1;

        for (int i = 0; i < 255; i++) {
          const unsigned int o_val = _stats.first_repeaters_count[i];

          if (o_val > o_max1) {
            o_max3 = o_max2;
            o_idx3 = o_idx2;
            o_max2 = o_max1;
            o_idx2 = o_idx1;
            o_max1 = o_val;
            o_idx1 = i;
          } else if (o_val > o_max2) {
            o_max3 = o_max2;
            o_idx3 = o_idx2;
            o_max2 = o_val;
            o_idx2 = i;
          } else if (o_val > o_max3) {
            o_max3 = o_val;
            o_idx3 = i;
          }
        }

        if (o_idx3 > 0) {
          sprintf(message, "Иcxoдящиe тoп-3 peпы:\n%02X %s - %d\n%02X %s - %d\n%02X %s - %d", o_idx1,
                  repeaters_names[o_idx1], o_max1, o_idx2, repeaters_names[o_idx2], o_max2, o_idx3,
                  repeaters_names[o_idx3], o_max3);
        } else {
          sprintf(message, "@[%s] copян, пoкa нe нaбpaлcя тoп peпитepoв в этoм кaнaлe", _from);
        }
      }
    }
  }

  if (message[0] != 0) {
    _stats.total_request++;
    sendMessage(message);
  }

  saveStats();
}

void MyMesh::onDiscoveredContact(ContactInfo &contact, const bool is_new, uint8_t path_len, const uint8_t *path) {
  Serial.printf("ADVERT from -> %s\n", contact.name);
  Serial.printf("  type: %s\n", getTypeName(contact.type));
  Serial.print("   public key: ");
  mesh::Utils::printHex(Serial, contact.id.pub_key, PUB_KEY_SIZE);
  Serial.println();
  saveContacts();
  if (contact.type == ADV_TYPE_REPEATER && checkRepeaterNamePattern(contact.name) && strncmp(repeaters_names[contact.id.pub_key[0]], contact.name, 32) != 0) {
    sprintf(message, "Бип бип бип, oбнapyжeн нoвый peпитep: %02X %s", contact.id.pub_key[0], contact.name);
    sendMessage(message);
    saveStats();
  }
}

MyMesh::MyMesh(mesh::Radio &radio, StdRNG &rng, mesh::RTCClock &rtc, SimpleMeshTables &tables)
    // ReSharper disable CppDFAMemoryLeak
    : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables) {
  // defaults
  memset(&_prefs, 0, sizeof(_prefs));
  _prefs.airtime_factor = 1.0;
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
  _prefs.agc_reset_interval = 12;

  command[0] = 0;
  message[0] = 0;
  tmp_buf[0] = 0;
  hex_buf[0] = 0;
  hex_buf[0] = 0;
  clock_set = false;

  for (auto & repeaters_name : repeaters_names) {
    snprintf(repeaters_name, 32, "UnknownRepeater");
  }
}

void MyMesh::begin(FILESYSTEM &fs) {
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
  if (!store.load("_main", self_id, _prefs.node_name,
                  sizeof(_prefs.node_name))) { // legacy: node_name was from identity file
    // Need way to get some entropy to seed RNG
    Serial.println("Press ENTER to generate key:");
    char c = 0;
    while (c != '\n') { // wait for ENTER to be pressed
      if (Serial.available()) c = Serial.read();
    }

    static_cast<StdRNG *>(getRNG())->begin(static_cast<long>(millis())); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)

    self_id = mesh::LocalIdentity(getRNG()); // create new random identity
    int count = 0;
    while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) { // reserved id hashes
      self_id = mesh::LocalIdentity(getRNG());
      count++;
    }
    store.save("_main", self_id);
  }

  loadContacts();
  loadStats();
  _public = addChannel(PUBLIC_GROUP_NAME, PUBLIC_GROUP_PSK); // pre-configure public channel
}

void MyMesh::sendSelfAdvert(const int delay_millis) {
  const auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
  if (pkt) {
    sendFlood(pkt, delay_millis);
  }
}

void MyMesh::sendMessage(const char *message) {
  Serial.printf("%s\n", message);
  if (!quiet && _ms->getMillis() - last_msg_sent > QUIET_LIMIT_SECONDS * 1000) { // QUIET_LIMIT_SECONDS sec
    uint8_t temp[5 + MAX_TEXT_LEN + 32];
    const uint32_t timestamp = getRTCClock()->getCurrentTime();
    memcpy(temp, &timestamp, 4); // mostly an extra blob to help make packet_hash unique
    temp[4] = 0;                 // attempt and flags

    sprintf(reinterpret_cast<char *>(&temp[5]), "%s: %s", _prefs.node_name, &message[0]); // <sender>: <msg>
    temp[5 + MAX_TEXT_LEN] = 0; // truncate if too long

    const unsigned int len = strlen(reinterpret_cast<char *>(&temp[5]));
    const auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, _public->channel, temp, 5 + len);
    if (pkt) {
      sendFlood(pkt, QUIET_LIMIT_PAUSE * 1000);
      Serial.println("   Sent.");
    } else {
      Serial.println("   ERROR: unable to send");
    }

    last_msg_sent = _ms->getMillis();
    _stats.total_sent++;
  } else {
    Serial.printf("Quiet please!\n");
    _stats.total_ignores++;
  }
}

void MyMesh::importCard(const char* command) {
  while (*command == ' ') command++;   // skip leading spaces
  if (memcmp(command, "meshcore://", 11) == 0) {
    command += 11;  // skip the prefix
    char *ep = strchr(command, 0);  // find end of string
    while (ep > command) {
      ep--;
      if (mesh::Utils::isHexChar(*ep)) break;  // found tail end of card
      *ep = 0;  // remove trailing spaces and other junk
    }
    uint8_t len = strlen(command);
    if (len % 2 == 0) {
      len >>= 1;  // halve, for num bytes
      if (mesh::Utils::fromHex(tmp_buf, len, command)) {
        importContact(tmp_buf, len);
        return;
      }
    }
  }
  Serial.println("   error: invalid format");
}

void MyMesh::handleCommand(const char *command) {
  while (*command == ' ')
    command++; // skip leading spaces

  if (memcmp(command, "public ", 7) == 0) {
    // send GroupChannel msg
    sendMessage(&command[7]);
  } else if (strcmp(command, "clock") == 0) { // show current time
    const uint32_t now = getRTCClock()->getCurrentTime();
    const auto dt = DateTime(now);
    Serial.printf("%02d:%02d - %d/%d/%d UTC\n", dt.hour(), dt.minute(), dt.day(), dt.month(), dt.year());
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
  } else if (memcmp(command, "import ", 7) == 0) {
    importCard(&command[7]);
  } else if (memcmp(command, "reboot", 6) == 0) {
    board.reboot();
  } else if (memcmp(command, "stats", 5) == 0) {
    char uptime[32];
    format_uptime(getRTCClock()->getCurrentTime() - _stats.time_start - 1, uptime, sizeof(uptime));
    sprintf(message,
            "Bot stats:\n stat time start: %s\n requests: %d\n replies: %d\n for %dm: %d\n thanks: %d\n ignores: %d\n "
            "total: %d\n hops: %d",
            uptime, _stats.total_request, _stats.total_sent, QUIET_LIMIT_TIME, last_msg_count,
            _stats.total_thanks, _stats.total_ignores, _stats.total_received, _stats.total_hops);
    Serial.println(message);
  } else if (memcmp(command, "stats reset", 11) == 0) {
    _stats = {};
    Serial.println("   (stats reset).");
  } else if (memcmp(command, "repeaters", 9) == 0) {
    for (int i = 0; i < 255; i++) {
      sprintf(message, "%02X - %s - %d", i, repeaters_names[i], _stats.all_repeaters_count[i]);
      Serial.println(message);
    }
  } else if (memcmp(command, "shutdown", 8) == 0) {
    display.turnOff();
    radio_driver.powerOff();
    board.powerOff();
  } else if (memcmp(command, "help", 4) == 0) {
    Serial.println("Commands:");
    Serial.println("   clock");
    Serial.println("   advert");
    Serial.println("   import {biz card}");
    Serial.println("   stats");
    Serial.println("   stats reset");
    Serial.println("   repeaters");
    Serial.println("   quiet");
    Serial.println("   reboot");
    Serial.println("   shutdown");
    Serial.println("   ver");
    Serial.println("   public <text>");
  } else {
    Serial.print("   ERROR: unknown command: ");
    Serial.println(command);
  }
}

void MyMesh::loop() {
  BaseChatMesh::loop();

  unsigned int len = strlen(command);
  while (Serial.available() && len < sizeof(command) - 1) {
    const char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
    }
    Serial.print(c);
  }
  if (len == sizeof(command) - 1) { // command buffer full
    command[sizeof(command) - 1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') { // received complete line
    command[len - 1] = 0;                    // replace newline with C string null terminator

    handleCommand(command);
    command[0] = 0; // reset command buffer
  }
}
