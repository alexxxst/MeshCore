#include "MyMesh.h"

#include <Arduino.h> // needed for PlatformIO

#ifdef DISPLAY_CLASS
#include "UITask.h"
static UITask ui_task(display);
#endif

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables);

[[noreturn]] void halt() {
  while (true)
    ;
}

void setup() {
  Serial.begin(115200);

  board.begin();

  if (!radio_init()) {
    halt();
  }

  fast_rng.begin(static_cast<long>(radio_get_rng_seed()));
  sensors.begin();

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
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop(the_mesh.getQuiet(), the_mesh.getTotalRequested(), the_mesh.getTotalSent(),
               the_mesh.getRTCClock()->getCurrentTime(), the_mesh.getTenReceived());
#endif
  rtc_clock.tick();
}
