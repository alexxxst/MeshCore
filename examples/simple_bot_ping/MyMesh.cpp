#include "MyMesh.h"

#include "RTClib.h"

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
    size_t wrote =
      file.write(reinterpret_cast<uint8_t *>(&_stats.total_request), sizeof(_stats.total_request));
    wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.total_received), sizeof(_stats.total_received));
    wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.total_sent), sizeof(_stats.total_sent));
    wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.total_thanks), sizeof(_stats.total_thanks));
    wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.total_ignores), sizeof(_stats.total_ignores));
    wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.total_hops), sizeof(_stats.total_hops));
    wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.max_hops), sizeof(_stats.max_hops));
    wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.max_path), sizeof(_stats.max_path));
    wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.time_start), sizeof(_stats.time_start));
    wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.num_repeaters), sizeof(_stats.num_repeaters));
    for (int i = 0; i < _stats.num_repeaters; i++) {
      wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.repeaters[i].pub_key), sizeof(_stats.repeaters[i].pub_key));
      wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.repeaters[i].name), sizeof(_stats.repeaters[i].name));
      wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.repeaters[i].first_count), sizeof(_stats.repeaters[i].first_count));
      wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.repeaters[i].total_count), sizeof(_stats.repeaters[i].total_count));
      wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.repeaters[i].advert_time), sizeof(_stats.repeaters[i].advert_time));
      wrote += file.write(reinterpret_cast<uint8_t *>(&_stats.repeaters[i].update_time), sizeof(_stats.repeaters[i].update_time));
    }
    // Serial.printf("Stats wrote %d from %d bytes", wrote, sizeof(_stats));
    // Serial.println();
    file.flush();
    file.close();
  }
}

void MyMesh::resetStats() {
#if defined(NRF52_PLATFORM)
  _fs->remove("/node_stats");
  File file = _fs->open("/node_stats", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  File file = _fs->open("/node_stats", "w");
#else
  File file = _fs->open("/node_stats", "w", true);
#endif
  if (file) {
    tmp_buf[0] = '\0';
    file.write(tmp_buf, 1);
    file.flush();
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
      size_t read =
        file.read(&_stats.total_request, sizeof(_stats.total_request));
      read += file.read(&_stats.total_received, sizeof(_stats.total_received));
      read += file.read(&_stats.total_sent, sizeof(_stats.total_sent));
      read += file.read(&_stats.total_thanks, sizeof(_stats.total_thanks));
      read += file.read(&_stats.total_ignores, sizeof(_stats.total_ignores));
      read += file.read(&_stats.total_hops, sizeof(_stats.total_hops));
      read += file.read(&_stats.max_hops, sizeof(_stats.max_hops));
      read += file.read(&_stats.max_path, sizeof(_stats.max_path));
      read += file.read(&_stats.time_start, sizeof(_stats.time_start));
      read += file.read(&_stats.num_repeaters, sizeof(_stats.num_repeaters));
      for (int i = 0; i < _stats.num_repeaters; i++) {
        _stats.repeaters[i] = Repeater();
        read += file.read(&_stats.repeaters[i].pub_key, sizeof(_stats.repeaters[i].pub_key));
        read += file.read(&_stats.repeaters[i].name, sizeof(_stats.repeaters[i].name));
        read += file.read(&_stats.repeaters[i].first_count, sizeof(_stats.repeaters[i].first_count));
        read += file.read(&_stats.repeaters[i].total_count, sizeof(_stats.repeaters[i].total_count));
        read += file.read(&_stats.repeaters[i].advert_time, sizeof(_stats.repeaters[i].advert_time));
        read += file.read(&_stats.repeaters[i].update_time, sizeof(_stats.repeaters[i].update_time));
      }
      // Serial.printf("Stats read %d from %d bytes", read, sizeof(_stats));
      // Serial.println();
      file.close();
    }
  }
}

void MyMesh::setClock(const uint32_t timestamp) {
#if ENV_INCLUDE_GPS == 1
  if (_prefs.gps_enabled) {
    LocationProvider * nmea = sensors.getLocationProvider();
    if (nmea != nullptr) {
      nmea->syncTime();
      Serial.println("   (GPS clock!)");
      return;
    }
  }
#endif
  getRTCClock()->setCurrentTime(timestamp);
  time_start = timestamp;
  Serial.println("   (OK - clock set!)");
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
    size_t len = strlen(command);
    if (len % 2 == 0) {
      len >>= 1;  // halve, for num bytes
      if (mesh::Utils::fromHex(tmp_buf, static_cast<int>(len), command)) {
        importContact(tmp_buf, len);
        return;
      }
    }
  }
  Serial.println("   error: invalid format");
}

void MyMesh::onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt,
                                  const uint32_t timestamp, const char *text) {
  Serial.printf("   %s\n", text);
  if (!clock_set) {
    setClock(timestamp + 1);
    clock_set = true;
  }

#ifdef LED_BLUE
  digitalWrite(LED_BLUE, LOW);
#endif

  if (_stats.time_start == 0 && clock_set) {
    _stats.time_start = getRTCClock()->getCurrentTime() + 1;
  }

  _stats.total_received++;
  last_msg_rcvd = _ms->getMillis();

  // will reply with the same path hash size
  const uint8_t path_hash_size = pkt->getPathHashSize();

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

  // check replay attack with 10 minutes
  if (clock_set && abs(getRTCClock()->getCurrentTime() - timestamp) > 600) {
    Serial.println("   Replay message discarded!");
    return;
  }

  quiet = last_msg_count >= QUIET_LIMIT_COUNT;

  message[0] = 0;
  char _from[40], _text[180];
  if (sscanf(text, "%39[^:]: %179[^\0]", _from, _text) > 0) {

    // ping
    if (strncasecmp(_text, "ping", 4) == 0 || strncasecmp(_text, "test", 4) == 0 ||
        strncmp(_text, "пинг", 8) == 0 || strncmp(_text, "Пинг", 8) == 0 || strncmp(_text, "тест", 8) == 0 ||
        strncmp(_text, "Тест", 8) == 0 || strncmp(_text, "Мяя", 6) == 0 || strncmp(_text, "Мяу", 6) == 0) {
      if (pkt->isRouteDirect() || pkt->path_len == 0) {
        sprintf(message, "@[%s] диpeкт c SNR %03.2f dB", _from, pkt->getSNR());
      } else {
        Repeater *first_repeater = nullptr;
        char _path[(path_hash_size * 2 + 1) * pkt->path_len + 1];
        unsigned int offset = 0;
        for (size_t i = 0; i < pkt->path_len; i += path_hash_size) {
          uint8_t prefix[4]{};
          for (size_t _i = 0; _i < path_hash_size; _i++) {
            prefix[_i] = pkt->path[i + _i];
            offset += snprintf(_path + offset, sizeof(_path) - offset, "%02X", pkt->path[i + _i]);
          }
          offset += snprintf(_path + offset, sizeof(_path) - offset, ",");
          Repeater *repeater = searchRepeaterByPubKey(prefix, path_hash_size);
          if (repeater == nullptr) {
            repeater = createRepeater(prefix);
          }
          if (repeater != nullptr) {
            if (i == 0) {
              // first repeaters count
              repeater->first_count++;
              first_repeater = repeater;
            }
            repeater->total_count++;
            repeater->update_time = getRTCClock()->getCurrentTime();
          }
        }

        if (offset > 0) {
          _path[offset - 1] = '\0';
        }

        if (pkt->path_len >= _stats.max_hops) {
          _stats.max_hops = pkt->path_len;
          sprintf(_stats.max_path, "%s в %d %s: %s", _from, pkt->path_len, hop_word(pkt->path_len), _path);
        }
        if (first_repeater != nullptr) {
          sprintf(message, "@[%s] %d %s c %s: %s", _from, pkt->path_len, hop_word(pkt->path_len), first_repeater->name, _path);
        } else {
          sprintf(message, "@[%s] %d %s: %s", _from, pkt->path_len, hop_word(pkt->path_len), _path);
        }
        _stats.total_hops = _stats.total_hops + pkt->path_len;
      }
    }

    // to bot command
    if (strstr(_text, BOT_NAME_PLAIN) != nullptr) {

      // stats
      if (strstr(_text, "stats") != nullptr || strstr(_text, "статистика") != nullptr) {
        char uptime[16];
        format_uptime(getRTCClock()->getCurrentTime() - _stats.time_start - 1, uptime, sizeof(uptime));
        sprintf(message, "Cтaтa зa: %s\n oтвeты: %d из %d\n кaнaл: %d, зa %dм: %d\n хoпы: %d, peпы: %d",
                uptime, _stats.total_sent, _stats.total_request, _stats.total_received, QUIET_LIMIT_TIME,
                last_msg_count, _stats.total_hops, _stats.num_repeaters);
      }

      // uptime
      if (strstr(_text, "uptime") != nullptr || strstr(_text, "аптайм") != nullptr) {
        char uptime[16];
        format_uptime(getRTCClock()->getCurrentTime() - time_start - 1, uptime, sizeof(uptime));
        sprintf(message, "Uptime %s", uptime);
      }

      // thanks
      if (strstr(_text, "спасибо") != nullptr || strstr(_text, "thank") != nullptr) {
        _stats.total_thanks++;
        sprintf(message, "@[%s] 🥹пожалуйста №%d!", _from, _stats.total_thanks);
      }

      // path max
      if (strstr(_text, "рекорд") != nullptr || strstr(_text, "record") != nullptr) {
        if (_stats.max_hops > 5) {
          sprintf(message, "%s", _stats.max_path);
        } else {
          sprintf(message, "@[%s] copян, пoкa нe зaфикcиpoвaн длинный пyть в этoм кaнaлe", _from);
        }
      }

      // weather
      if (strstr(_text, "погода") != nullptr || strstr(_text, "weather") != nullptr) {
        sprintf(message, "@[%s] 🌦️блин, нy в oкнo выгляни! Kaкoй cмыcл oт мoиx дaтчикoв?", _from);
      }

      // бля
      if (strstr(_text, "бля") != nullptr || strstr(_text, "Бля") != nullptr) {
        sprintf(message, "@[%s] 🤨oт бля cлышy.", _from);
      }

      // meow
      if (strstr(_text, "мяу") != nullptr || strstr(_text, "meow") != nullptr ||
          strstr(_text, "мяя") != nullptr || strstr(_text, "мурр") != nullptr) {
        sprintf(message, "@[%s] 😼кc-кc-кc, нy иди cюдa, пoглaжy!", _from);
      }

      // drink
      if (strstr(_text, "выпьем") != nullptr || strstr(_text, "пиво") != nullptr ||
          strstr(_text, "drink") != nullptr || strstr(_text, "beer") != nullptr) {
        sprintf(message, "@[%s] 🍺вpeмя нaкaтить!", _from);
          }

      // help
      if (strstr(_text, "команды") != nullptr || strstr(_text, "помощь") != nullptr) {
        sprintf(message, "Koмaнды: пинг/тест, статистика, аптайм, репитеры, рекорд, старьё");
      }
      if (strstr(_text, "help") != nullptr) {
        sprintf(message, "Commands: ping/test, stats, uptime, repeaters, record, oldies");
      }

      // repeaters
      if (strstr(_text, "репиторы") != nullptr) {
        sprintf(message, "@[%s] 🙄кaкиe eщё peпитOpы!", _from);
      }

      // repeaters
      if (strstr(_text, "репитеры") != nullptr || strstr(_text, "repeaters") != nullptr) {

        unsigned int o_max1 = 0, o_max2 = 0, o_max3 = 0;
        int o_idx1 = -1, o_idx2 = -1, o_idx3 = -1;

        for (int i = 0; i < _stats.num_repeaters; i++) {
          const unsigned int o_val = _stats.repeaters[i].first_count;

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
          char hex1[6]{}, hex2[6]{}, hex3[6]{};
          mesh::Utils::toHex(hex1, _stats.repeaters[o_idx1].pub_key, _prefs.path_hash_mode);
          mesh::Utils::toHex(hex2, _stats.repeaters[o_idx2].pub_key, _prefs.path_hash_mode);
          mesh::Utils::toHex(hex3, _stats.repeaters[o_idx3].pub_key, _prefs.path_hash_mode);

          sprintf(message, "📡Toп иcxoдящиx peп:\n%s %s: %d\n%s %s: %d\n%s %s: %d",
            hex1, _stats.repeaters[o_idx1].name, o_max1,
            hex2, _stats.repeaters[o_idx2].name, o_max2,
            hex3, _stats.repeaters[o_idx3].name, o_max3);
        } else {
          sprintf(message, "@[%s] copян, пoкa нe нaбpaлcя тoп peпитepoв в этoм кaнaлe", _from);
        }
      }

      // garbage
      if (strstr(_text, "старьё") != nullptr || strstr(_text, "oldies") != nullptr) {

        unsigned int o_min1 = 0xFFFFFFFF, o_min2 = 0xFFFFFFFF, o_min3 = 0xFFFFFFFF;
        int o_idx1 = -1, o_idx2 = -1, o_idx3 = -1;

        for (int i = 0; i < _stats.num_repeaters; i++) {
          const unsigned int o_val = _stats.repeaters[i].advert_time;

          if (o_val > _stats.time_start) {
            if (o_val < o_min1) {
              o_min3 = o_min2;
              o_idx3 = o_idx2;
              o_min2 = o_min1;
              o_idx2 = o_idx1;
              o_min1 = o_val;
              o_idx1 = i;
            } else if (o_val < o_min2) {
              o_min3 = o_min2;
              o_idx3 = o_idx2;
              o_min2 = o_val;
              o_idx2 = i;
            } else if (o_val < o_min3) {
              o_min3 = o_val;
              o_idx3 = i;
            }
          }
        }

        if (o_idx3 > 0 && getRTCClock()->getCurrentTime() - o_min3 > 86400) {
          char hex1[6]{}, hex2[6]{}, hex3[6]{};
          mesh::Utils::toHex(hex1, _stats.repeaters[o_idx1].pub_key, _prefs.path_hash_mode);
          mesh::Utils::toHex(hex2, _stats.repeaters[o_idx2].pub_key, _prefs.path_hash_mode);
          mesh::Utils::toHex(hex3, _stats.repeaters[o_idx3].pub_key, _prefs.path_hash_mode);

          char adv1[4]{}, adv2[4]{}, adv3[4]{};
          format_days(getRTCClock()->getCurrentTime() - o_min1, adv1, sizeof(adv1));
          format_days(getRTCClock()->getCurrentTime() - o_min2, adv2, sizeof(adv2));
          format_days(getRTCClock()->getCurrentTime() - o_min3, adv3, sizeof(adv3));

          sprintf(message, "📡Toп peп бeз aдвepтa:\n%s %s: %s\n%s %s: %s\n%s %s: %s",
            hex1, _stats.repeaters[o_idx1].name, adv1,
            hex2, _stats.repeaters[o_idx2].name, adv2,
            hex3, _stats.repeaters[o_idx3].name, adv3);
        } else {
          sprintf(message, "@[%s] copян, пoкa нe нaбpaлcя тoп peпитepoв в этoм кaнaлe", _from);
        }
          }
    }
  }

  if (message[0] != 0) {
    _stats.total_request++;
    sendMessage(message, path_hash_size);
  }

  saveStats();
}

void MyMesh::onDiscoveredContact(ContactInfo &contact, const bool is_new, uint8_t path_len, const uint8_t *path) {
  Serial.printf("ADVERT from -> %s\n", contact.name);
  Serial.printf("  type: %s\n", getTypeName(contact.type));
  Serial.print("   public key: ");
  mesh::Utils::printHex(Serial, contact.id.pub_key, PUB_KEY_SIZE);
  Serial.println();

#ifdef LED_BLUE
  digitalWrite(LED_BLUE, LOW);
#endif

  if (contact.type == ADV_TYPE_REPEATER && checkRepeaterNamePattern(contact.name)) {
    bool to_send = false;
    // full key search first
    Repeater *repeater = searchRepeaterByPubKey(contact.id.pub_key);
    if (repeater == nullptr) {
      // prefix search
      for (int i = 3; i >= _prefs.path_hash_mode; i--) {
        repeater = searchRepeaterByPubKey(contact.id.pub_key, i);
        if (repeater != nullptr) {
          break;
        }
      }
    }
    if (repeater != nullptr) {
      to_send = updateRepeater(*repeater, contact);
    } else {
      to_send = addRepeater(contact);
    }
    if (to_send && clock_set && getRTCClock()->getCurrentTime() - _stats.time_start > 24 * 3600) {
      char hex[6]{};
      mesh::Utils::toHex(hex, contact.id.pub_key, _prefs.path_hash_mode);
      sprintf(message, "📡Бип-бип-бип, oбнapyжeн нoвый peпитep: %s %s", hex, contact.name);
      sendMessage(message, _prefs.path_hash_mode);
    }

    // check old repeaters every hour when no messages
    if (clock_set && !to_send && _ms->getMillis() - last_repeater_check > 3600 * 1000) {
      for (int i = 0; i < _stats.num_repeaters; i++) {
        const unsigned int time = getRTCClock()->getCurrentTime();
        if (time - _stats.repeaters[i].advert_time > OLD_REPEATER_TIME * 2 * 86400 && time - _stats.repeaters[i].update_time > OLD_REPEATER_TIME * 86400) {
          char hex[6]{};
          mesh::Utils::toHex(hex, _stats.repeaters[i].pub_key, _prefs.path_hash_mode);
          sprintf(message, "📡Бип-бип-бип, дaвнo нe видeл peпитep %s %s ...пpoщaй!", hex, _stats.repeaters[i].name);
          sendMessage(message, _prefs.path_hash_mode);
          removeRepeater(_stats.repeaters[i]);
          break;
        }
      }
      last_repeater_check = _ms->getMillis();
    }

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
  _prefs.gps_enabled = 1;
  _prefs.gps_interval = 120;
  _prefs.powersaving_enabled = false;
  _prefs.agc_reset_interval = 4;
  _prefs.path_hash_mode = PATH_HASH_MODE;

  command[0] = 0;
  message[0] = 0;
  tmp_buf[0] = 0;
  clock_set = false;
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
  if (!store.load("_main", self_id, _prefs.node_name, sizeof(_prefs.node_name))) {
    // legacy: node_name was from identity file
    // Need way to get some entropy to seed RNG
    Serial.println("Press ENTER to generate key:");
    char c = 0;
    while (c != '\n') {
      // wait for ENTER to be pressed
      if (Serial.available()) c = Serial.read();
    }

    static_cast<StdRNG *>(getRNG())->begin(static_cast<long>(millis())); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)

    self_id = mesh::LocalIdentity(getRNG()); // create new random identity
    int count = 0;
    while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {
      // reserved id hashes
      self_id = mesh::LocalIdentity(getRNG());
      count++;
    }
    store.save("_main", self_id);
  }

  loadStats();
  _public = addChannel(PUBLIC_GROUP_NAME, PUBLIC_GROUP_PSK); // pre-configure public channel

#if ENV_INCLUDE_GPS == 1
  applyGpsPrefs();
#endif
}

void MyMesh::sendMessage(const char *message, const uint8_t path_hash_size) {
  Serial.printf("%s\n", message);
  if (!quiet && _ms->getMillis() - last_msg_sent > QUIET_LIMIT_SECONDS * 1000) {
    // QUIET_LIMIT_SECONDS sec
    uint8_t temp[5 + MAX_TEXT_LEN + 32];
    const uint32_t timestamp = getRTCClock()->getCurrentTime();
    memcpy(temp, &timestamp, 4); // mostly an extra blob to help make packet_hash unique
    temp[4] = 0;                 // attempt and flags

    sprintf(reinterpret_cast<char *>(&temp[5]), "%s: %s", _prefs.node_name, &message[0]); // <sender>: <msg>
    temp[5 + MAX_TEXT_LEN] = 0; // truncate if too long

    const unsigned int len = strlen(reinterpret_cast<char *>(&temp[5]));
    const auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, _public->channel, temp, 5 + len);
    if (pkt) {
      sendFlood(pkt, QUIET_LIMIT_PAUSE * 1000, path_hash_size);
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

void MyMesh::handleCommand(const char *command) {
  while (*command == ' ')
    command++; // skip leading spaces

  if (memcmp(command, "public ", 7) == 0) {
    // send GroupChannel msg
    sendMessage(&command[7], _prefs.path_hash_mode);
  } else if (strcmp(command, "clock") == 0) {
    // show current time
    const uint32_t now = getRTCClock()->getCurrentTime();
    const auto dt = DateTime(now);
    Serial.printf("%02d:%02d - %d/%d/%d UTC\n", dt.hour(), dt.minute(), dt.day(), dt.month(), dt.year());
  } else if (strcmp(command, "quiet") == 0) {
    quiet = true;
    Serial.println("   (quiet set).");
  } else if (memcmp(command, "time ", 5) == 0) {
    _prefs.gps_enabled = false;
    setClock(strtol(&command[5], nullptr, 10));
    _prefs.gps_enabled = true;
    clock_set = true;
  } else if (memcmp(command, "import ", 7) == 0) {
    importCard(&command[7]);
  } else if (memcmp(command, "ver", 3) == 0) {
    Serial.println(FIRMWARE_VER_TEXT);
  } else if (memcmp(command, "reboot", 6) == 0) {
    board.reboot();
  } else if (memcmp(command, "stats reset", 11) == 0) {
    resetStats();
    Serial.println("   (stats reset).");
    // _fs->format();
    board.reboot();
  } else if (memcmp(command, "stats", 5) == 0) {
    loadStats();
    char uptime[16];
    format_uptime(getRTCClock()->getCurrentTime() - _stats.time_start - 1, uptime, sizeof(uptime));
    sprintf(message,
            "Bot stats:\n stat time start: %s\n requests: %d\n replies: %d\n for %dm: %d\n thanks: %d\n "
            "ignores: %d\n "
            "total: %d\n hops: %d",
            uptime, _stats.total_request, _stats.total_sent, QUIET_LIMIT_TIME, last_msg_count,
            _stats.total_thanks, _stats.total_ignores, _stats.total_received, _stats.total_hops);
    Serial.println(message);
  } else if (memcmp(command, "repeaters", 9) == 0) {
    for (int i = 0; i < _stats.num_repeaters; i++) {
      mesh::Utils::printHex(Serial, _stats.repeaters[i].pub_key, PUB_KEY_SIZE);
      sprintf(message, " - %s - %d", _stats.repeaters[i].name, _stats.repeaters[i].total_count);
      Serial.println(message);
    }
  } else if (memcmp(command, "shutdown", 8) == 0) {
    display.turnOff();
    radio_driver.powerOff();
    board.powerOff();
  } else if (memcmp(command, "help", 4) == 0) {
    Serial.println("Commands:");
    Serial.println("   clock");
    Serial.println("   time {timestamp}");
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
  if (len == sizeof(command) - 1) {
    // command buffer full
    command[sizeof(command) - 1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {
    // received complete line
    command[len - 1] = 0; // replace newline with C string null terminator

    handleCommand(command);
    command[0] = 0; // reset command buffer
  }
}

Repeater* MyMesh::searchRepeaterByPubKey(const uint8_t *pub_key, const int prefix_len) {
  for (int i = 0; i < _stats.num_repeaters; i++) {
    Repeater* repeater = &_stats.repeaters[i];
    if (memcmp(pub_key, repeater->pub_key, prefix_len) == 0) return repeater;
  }
  return nullptr; // not found
}

Repeater* MyMesh::searchRepeaterByPubKey(const uint8_t *pub_key) {
  for (int i = 0; i < _stats.num_repeaters; i++) {
    Repeater* repeater = &_stats.repeaters[i];
    if (memcmp(pub_key, repeater->pub_key, PUB_KEY_SIZE) == 0) return repeater;
  }
  return nullptr; // not found
}

Repeater *MyMesh::createRepeater(uint8_t prefix[4]) {
  Repeater* repeater = allocateRepeaterSlot();
  if (repeater != nullptr) {
    uint8_t pub_key[PUB_KEY_SIZE]{};
    memcpy(pub_key, prefix, 4);
    memcpy(repeater->pub_key, pub_key, PUB_KEY_SIZE);
    strcpy(repeater->name, "UnknownRepeater");
    return repeater; // success
  }
  return nullptr;
}

bool MyMesh::addRepeater(const ContactInfo &contact) {
  Repeater *repeater = allocateRepeaterSlot();
  if (repeater != nullptr) {
    strcpy(repeater->name, contact.name);
    memcpy(repeater->pub_key, contact.id.pub_key, PUB_KEY_SIZE);
    repeater->advert_time = getRTCClock()->getCurrentTime();
    repeater->update_time = getRTCClock()->getCurrentTime();
    return true; // success
  }
  return false;
}

bool MyMesh::updateRepeater(Repeater &repeater, const ContactInfo &contact) const {
  const bool ret = strcmp(repeater.name, contact.name) != 0;
  strcpy(repeater.name, contact.name);
  memcpy(repeater.pub_key, contact.id.pub_key, PUB_KEY_SIZE);
  repeater.advert_time = getRTCClock()->getCurrentTime();
  repeater.update_time = getRTCClock()->getCurrentTime();
  return ret;
}

bool MyMesh::removeRepeater(const Repeater &repeater) {
  int idx = 0;
  while (idx < _stats.num_repeaters && memcmp(_stats.repeaters[idx].pub_key, repeater.pub_key, PUB_KEY_SIZE) != 0) {
    idx++;
  }
  if (idx >= _stats.num_repeaters) return false; // not found

  // remove from contacts array
  _stats.num_repeaters--;
  while (idx < _stats.num_repeaters) {
    _stats.repeaters[idx] = _stats.repeaters[idx + 1];
    idx++;
  }
  return true; // success
}

Repeater *MyMesh::allocateRepeaterSlot() {
  if (_stats.num_repeaters < MAX_CONTACTS) {
    return &_stats.repeaters[_stats.num_repeaters++];
  }
  if (shouldOverwriteWhenFull()) {
    // Find oldest non-favourite contact by oldest lastmod timestamp
    int oldest_idx = -1;
    uint32_t oldest_lastmod = 0xFFFFFFFF;
    for (int i = 0; i < _stats.num_repeaters; i++) {
      if (_stats.repeaters[i].update_time < oldest_lastmod) {
        oldest_lastmod = _stats.repeaters[i].update_time;
        oldest_idx = i;
      }
    }
    if (oldest_idx >= 0) {
      return &_stats.repeaters[oldest_idx];
    }
  }
  return nullptr; // no space, no overwrite or all contacts are all favourites
}
