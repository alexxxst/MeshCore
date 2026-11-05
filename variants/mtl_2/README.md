# MTL_2 (E22 / Pro Micro nRF52840)

Вариант платы **MTL_2** на базе Pro Micro nRF52840 и модуля EBYTE E22:

- `E22-900M22S` — SX1262, 868 МГц;
- `E22-400M22S` — SX1268, 433 МГц.

Порт основан на [VladelfPv/MeshCoreMTL — variants/mtlmicro](https://github.com/VladelfPv/MeshCoreMTL/tree/main/variants/mtlmicro) (MIT), адаптирован под текущий MeshCore без изменений общего кода.

## Прошивки

| Окружение | Радио | Частота | BW | SF | CR |
|---|---|---:|---:|---:|---:|
| `MTL_2_E22_sx1262_companion_radio_ble` | SX1262 | 868.856 | 62.5 | 7 | 7 |
| `MTL_2_E22_sx1262_repeater` | SX1262 | 868.856 | 62.5 | 7 | 7 |
| `MTL_2_E22_sx1268_companion_radio_ble` | SX1268 | 433.0 | 62.5 | 7 | 5 |
| `MTL_2_E22_sx1268_repeater` | SX1268 | 433.0 | 62.5 | 7 | 5 |

USB-companion и RS232-мост в этот вариант не входят.

## Сборка

```bash
pio run -e MTL_2_E22_sx1262_companion_radio_ble
pio run -e MTL_2_E22_sx1262_repeater
pio run -e MTL_2_E22_sx1268_companion_radio_ble
pio run -e MTL_2_E22_sx1268_repeater
```

Готовые `.uf2`/`.zip` лежат в `.pio/build/<env>/`. Для repeater’ов также добавлены строки в `build.sh`:

```bash
sh build.sh build-repeater-firmwares
```

## Распиновка

| Узел | Пин nRF | Arduino pin | Примечание |
|---|---|---:|---|
| LoRa NSS | P1.13 | 45 | |
| LoRa SCK | P1.11 | 43 | |
| LoRa MISO | P0.02 | 2 | |
| LoRa MOSI | P1.15 | 47 | |
| LoRa BUSY | P0.29 | 29 | |
| LoRa DIO1 | P0.10 | 10 | |
| LoRa RESET | P0.09 | 9 | |
| LoRa RXEN | P0.17 | 17 | TXEN = `RADIOLIB_NC`, DIO2 как RF switch |
| LoRa POWER_EN | P1.06 | 38 | |
| I2C SDA | P1.04 | 36 | |
| I2C SCL | P0.11 | 11 | |
| GPS TX | P0.22 | 22 | `Serial1`, 9600 (через `PIN_GPS_TX`) |
| GPS RX | P0.20 | 20 | `Serial1`, 9600 (через `PIN_GPS_RX`) |
| GPS EN | P0.24 | 24 | |
| Battery | P0.31 | 31 | `ADC_MULTIPLIER = 1.51` |
| User button | P1.00 | 32 | |
| Joystick left | P1.01 | 33 | companion |
| Joystick right | P1.02 | 34 | companion |
| Power switch | P1.07 | 39 | 10/20 dBm |
| Buzzer | P0.06 | 6 | |
| Vibration | P0.08 | 8 | |

Дисплей:

- companion — SH1106 128×64, I2C, reset `-1`;
- repeater — SSD1306 128×64, I2C, reset `-1`.

## Датчики

Включены AHTX0, BME280, BMP280, INA3221, INA219. GPS включён только в companion-прошивках; в repeater `ENV_INCLUDE_GPS` отключён.

## Поведение и ограничения

- `powerOff()` наследуется от `NRF52Board`; в `shutdownPeripherals()` питание LoRa отключается, пробуждение настраивается на `PIN_USER_BTN`.
- DC/DC не включается: `begin()` вызывает `NRF52Board::begin()`, как в upstream.
- Переключатель мощности меняет 10/20 dBm и обновляет `tx_power_dbm` в памяти; в UI значение появляется при следующем обновлении экрана.
- `PIN_SPI_SCK` в `variant.h` оставлен как в upstream (33), но радио использует `P_LORA_SCLK = 43`; на работу не влияет.
- Отдельной back/select-кнопки нет: `PIN_BACK_BTN = -1`, поэтому triple-click `KEY_SELECT` в companion UI недоступен. Если на плате появится третья кнопка, достаточно задать `PIN_BACK_BTN`.
- `USE_LBT` и пороги шума в `variant.h` оставлены как в upstream, в текущем MeshCore не используются.
- `ADC_MULTIPLIER = 1.51` — значение из upstream, требует проверки на конкретной плате.
- Размер прошивки не проверялся как критерий приёмки.
