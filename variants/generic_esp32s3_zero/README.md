# Generic ESP32-S3-Zero repeater (SX1262 / SX1268)

Репитер MeshCore на плате **Waveshare ESP32-S3-Zero** (ESP32-S3, 4 МБ flash)
с модулем EBYTE E22:

- `E22-900M30S` / `E22-900M33S` (SX1262) — 868 МГц;
- `E22-400M30S` / `E22-400M33S` (SX1268) — 433 МГц.

Вариант не требует правок общего кода: вся плата описана в `target.h`,
`target.cpp` и `S3ZeroBoard.h`.

## Прошивки

| Окружение | Радио | Частота | BW | SF | CR | Advert |
|---|---|---:|---:|---:|---:|---|
| `Generic_ESP32S3_Zero_SX1262_repeater` | SX1262 | 868.856 МГц | 62.5 кГц | 7 | 7 | `S3Zero Repeater` |
| `Generic_ESP32S3_Zero_SX1268_repeater` | SX1268 | 433.000 МГц | 62.5 кГц | 7 | 5 | `S3Zero 433 Repeater` |

`868.856 / 62.5 / SF7 / CR7` наследуются из `[arduino_base]` в корневом
`platformio.ini`; для 433 МГц вариант переопределяет частоту и `LORA_CR=5`.

## Распиновка

| Узел | GPIO | Примечание |
|---|---:|---|
| LoRa NSS | 7 | |
| LoRa SCK | 8 | |
| LoRa MOSI | 9 | |
| LoRa MISO | 10 | |
| LoRa RESET | 11 | |
| LoRa BUSY | 12 | |
| LoRa DIO1 | 13 | |
| LoRa TXEN | 3 | ручное управление RF-переключателем |
| LoRa RXEN | 4 | `SX126X_DIO2_AS_RF_SWITCH=false` |
| I2C SDA | 6 | для опционального OLED/датчиков |
| I2C SCL | 5 | |
| Кнопка BOOT | 0 | `PIN_USER_BTN` |
| WS2812 | 21 | встроенный RGB-светодиод платы |

Настройки радио SX126x: TCXO 1.8 В, `SX126X_CURRENT_LIMIT=140`,
`SX126X_RX_BOOSTED_GAIN=1`.

> `LORA_TX_POWER=17` подобран под модули **M30**. Для **M33** это перебор:
> ставьте не более 8 дБм, иначе перегружается PA модуля.

## Светодиод

Встроенный WS2812 (GPIO21) работает как локальный индикатор — экрана у платы нет.

| Сигнал | Значение |
|---|---|
| Красный, на время передачи | радио передаёт; гаснет сразу после `onAfterTransmit()` |
| 2 × зелёный при старте | прошивка жива, радио инициализировалось, ключи загружены |
| 3 × красный при старте | прошлая загрузка — brownout (просадка питания) |
| 2 × красный при старте | прошлая загрузка — panic или watchdog |

Логика — в `S3ZeroBoard.h`. Настраивается там же:
`S3ZERO_RGB_LED` (пин), `S3ZERO_LED_BRIGHTNESS` (0–255),
`S3ZERO_BLINK_ON_MS` / `S3ZERO_BLINK_OFF_MS` (тайминги самотеста).

## Сборка и прошивка

```bash
pio run -e Generic_ESP32S3_Zero_SX1262_repeater
pio run -e Generic_ESP32S3_Zero_SX1262_repeater -t upload

pio run -e Generic_ESP32S3_Zero_SX1268_repeater
pio run -e Generic_ESP32S3_Zero_SX1268_repeater -t upload
```

Объединённый образ для прошивки через `esptool`/веб-прошивальщик:

```bash
pio run -e Generic_ESP32S3_Zero_SX1262_repeater -t mergebin
# результат: .pio/build/<env>/firmware-merged.bin
```

Оба окружения входят в `sh build.sh build-repeater-firmwares`.

Плата прошивается по USB-C (native USB, `ARDUINO_USB_CDC_ON_BOOT=1`).

## Что стоит поменять перед установкой

В `variants/generic_esp32s3_zero/platformio.ini`:

- `ADMIN_PASSWORD='"password"'` — на свой пароль;
- `ADVERT_NAME` — на осмысленное имя ноды;
- `LORA_TX_POWER` — под свой модуль (см. предупреждение выше).

## Ограничения

- Батарея не измеряется: `PIN_VBAT_READ` не определён, `getBattMilliVolts()`
  возвращает `0`. Нужен внешний делитель на ADC.
- OLED не включён по умолчанию. При необходимости добавьте
  `-D DISPLAY_CLASS=SSD1306Display` — библиотека `Adafruit SSD1306` уже в
  `lib_deps`, адрес I2C стандартный (`0x3C`).
