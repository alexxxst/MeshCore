#pragma once

// The firmware's file system types differ per platform. Keep the includes in
// one place so modules can own their files without copying the #if block.

#include <Arduino.h>

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
  using File = fs::File;
#endif

#include <helpers/IdentityStore.h>
