# MTL-1 (ESP32-S3 + SX1262 / SX1268)

Вариант платы **MTL-1** на базе ESP32-S3 DevKitC-1 с модулем SX1262 (868 МГц)
или SX1268 (433 МГц).

## Прошивки

| Окружение | Радио | Частота | BW | SF | CR | TX | Примечание |
|---|---|---:|---:|---:|---:|---:|---|
| `MTL_1_repeater` | SX1262 | 868.856 | 62.5 | 7 | 7 | 18 | SSD1306 |
| `MTL_1_SX1268_repeater` | SX1268 | 433.0 | 62.5 | 7 | 5 | 18 | SSD1306 |
| `MTL_1_repeater_bridge_rs232` | SX1262 | 868.856 | 62.5 | 7 | 7 | 18 | RS-232 на GPIO 5/6 |
| `MTL_1_repeater_bridge_espnow` | SX1262 | 868.856 | 62.5 | 7 | 7 | 18 | ESP-NOW |
| `MTL_1_SX1268_repeater_bridge_espnow` | SX1268 | 433.0 | 62.5 | 7 | 5 | 18 | ESP-NOW |
| `MTL_1_room_server` | SX1262 | 868.856 | 62.5 | 7 | 7 | 18 | SSD1306 |
| `MTL_1_terminal_chat` | SX1262 | 868.856 | 62.5 | 7 | 7 | 18 | Terminal chat |
| `MTL_1_companion_radio_usb` | SX1262 | 868.856 | 62.5 | 7 | 7 | 18 | USB |
| `MTL_1_companion_radio_ble` | SX1262 | 868.856 | 62.5 | 7 | 7 | 18 | BLE + Wi-Fi |
| `MTL_1_SX1268_companion_radio_usb` | SX1268 | 433.0 | 62.5 | 7 | 5 | 18 | USB |
| `MTL_1_SX1268_companion_radio_ble` | SX1268 | 433.0 | 62.5 | 7 | 5 | 18 | BLE + Wi-Fi |
| `MTL_1_companion_radio_wifi` | SX1262 | 868.856 | 62.5 | 7 | 7 | 18 | Только Wi-Fi |
| `MTL_1_sensor` | SX1262 | 868.856 | 62.5 | 7 | 7 | 18 | I2C `SDA=33`, `SCL=34` |
| `MTL_1_kiss_modem` | SX1262 | 868.856 | 62.5 | 7 | 7 | 18 | KISS-модем |

Частота, BW, SF и CR для 868-МГц сборок наследуются из глобальной секции
`[arduino_base]` в `platformio.ini`. В `variants/mtl_1/platformio.ini` остаются
только отличия: 433 МГц / CR5 для SX1268 и значение TX power для платы.

`MTL_1_companion_radio_wifi` требует задать `WIFI_SSID` и `WIFI_PWD` в
`variants/mtl_1/platformio.ini` перед сборкой.

## Сборка

```bash
pio run -e MTL_1_repeater
pio run -e MTL_1_companion_radio_ble
```

Через `build.sh` (рекомендуется для готовых артефактов):

```bash
export FIRMWARE_VERSION=v1.18.0
sh build.sh build-firmware MTL_1_repeater
sh build.sh build-firmware MTL_1_companion_radio_ble
```

`build.sh build-companion-firmwares` включает только BLE-варианты
(`MTL_1_companion_radio_ble` и `MTL_1_SX1268_companion_radio_ble`). USB- и
Wi-Fi-варианты собираются через `build-firmware`.

Готовые файлы складываются в `out/` в виде одного zip на цель. Подробнее про
имена артефактов, переменные окружения и наборы сборки — в
[Building Firmware](../../docs/building.md).

## Распиновка

| Узел | Пин ESP32-S3 | Примечание |
|---|---:|---|
| LoRa NSS | 8 | |
| LoRa SCK | 9 | |
| LoRa MISO | 11 | |
| LoRa MOSI | 10 | |
| LoRa BUSY | 13 | |
| LoRa DIO1 | 14 | |
| LoRa RESET | — | `RADIOLIB_NC` |
| LoRa TX LED | 35 | |
| I2C SDA (по умолчанию) | 17 | |
| I2C SCL (по умолчанию) | 18 | |
| I2C SDA (`MTL_1_sensor`) | 33 | |
| I2C SCL (`MTL_1_sensor`) | 34 | |
| GPS RX | 47 | `Serial1` |
| GPS TX | 48 | `Serial1` |
| GPS EN | 26 | |
| User button | 0 | |
| VEXT EN | 36 | Питание периферии |
| Battery ADC | 1 | `ADC_MULTIPLIER=5.42` |
| ADC control | 37 | |

Дисплей: SSD1306 128×64, I2C.

## Радио и питание

- SX1262: `DIO2` используется как RF switch, TCXO 1.8 В, current limit 140 мА,
  boosted RX gain включён.
- SX1268: те же настройки, частота 433.0 МГц, CR 5.
- `ESP32_CPU_FREQ=80`.
- BLE-компаньоны: `BLE_PIN_CODE=123456`, `AUTO_SHUTDOWN_MILLIVOLTS=3400`,
  `OFFLINE_QUEUE_SIZE=256`, включён Wi-Fi-интерфейс.
- `powerOff()` унаследован от `ESP32Board` и уводит плату в deep sleep без
  таймера пробуждения; `enterDeepSleep(secs)` на этой плате просыпается по
  LoRa DIO1, а при передаче `pin_wake_btn` — ещё и по кнопке пользователя.

## Датчики и GPS

Все сборки `MTL_1` наследуют общий `sensor_base`: GPS, AHTX0, BME280, BMP280,
SHTC3, SHT4X, LPS22HB, INA3221, INA219, INA226, INA260, MLX90614, VL53L0X,
BME680 и BMP085. GPS подключён к `Serial1` (`RX=47`, `TX=48`, `EN=26`).

## Ограничения

- `MTL_1_companion_radio_wifi` не соберётся с рабочим Wi-Fi без корректных
  `WIFI_SSID`/`WIFI_PWD`.
- RS-232 bridge использует GPIO 5/6; при подключении внешнего модуля проверьте,
  что эти пины не заняты другой периферией.
- Значение `ADC_MULTIPLIER=5.42` рассчитано для данной платы; при другой
  схеме делителя напряжения его нужно скорректировать.
