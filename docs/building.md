# Building Firmware

The repository includes `build.sh`, a wrapper around PlatformIO that builds a
fixed set of firmware targets, names the artifacts with the version and commit,
and packs each target into a single zip.

## Requirements

- [PlatformIO](https://platformio.org/) with the `pio` command available.
- Python 3 (used for the `.uf2` conversion and archive naming).
- `zip` and `git`.

## Quick start

`FIRMWARE_VERSION` must be set; `build.sh` exits if it is missing.

```bash
export FIRMWARE_VERSION=v1.18.0
sh build.sh list
sh build.sh build-firmware RAK_4631_repeater
```

The output is written to `out/`. Every command except `help` and `list` clears
`out/` first.

## Commands

| Command | What it builds |
|---|---|
| `sh build.sh help` | Show usage. |
| `sh build.sh list` | List every PlatformIO environment. |
| `sh build.sh build-firmware <env> [<env> ...]` | Build one or more explicit environments. |
| `sh build.sh build-matching-firmwares <text>` | Build every environment whose name contains `<text>`. |
| `sh build.sh build-firmwares` | Companion + repeater + room-server sets. |
| `sh build.sh build-companion-firmwares` | The selected BLE companion set (no USB companions). |
| `sh build.sh build-repeater-firmwares` | The selected repeater set. |
| `sh build.sh build-room-server-firmwares` | Every environment ending in `_room_server`. |
| `sh build.sh build-kiss-radio-firmwares` | Every environment ending in `_kiss_modem`. |
| `sh build.sh get-companion-firmwares-to-build` | Print all companion environments (USB and BLE). |
| `sh build.sh get-repeater-firmwares-to-build` | Print every environment ending in `_repeater`. |
| `sh build.sh get-room-server-firmwares-to-build` | Print every environment ending in `_room_server`. |

`build-companion-firmwares` intentionally builds **BLE companions only**. ESP32
BLE companions also enable the Wi-Fi interface, so they can be used over BLE or
Wi-Fi. USB and Wi-Fi companion environments still exist in the project and can
be built with `build-firmware`:

```bash
export FIRMWARE_VERSION=v1.18.0
sh build.sh build-firmware MTL_1_companion_radio_usb
sh build.sh build-firmware MTL_1_companion_radio_wifi
```

`build-iotthinks.sh` is an alternate convenience wrapper: it sets
`FIRMWARE_VERSION=PowerSaving17.1.6` and calls `build.sh` for the wider
IoTThinks PowerSaving target list (repeaters, room servers, companions and
sensors). It uses the same artifact layout as `build.sh`.

### Selected companion set

`build-companion-firmwares` builds these environments:

```text
WioTrackerL1_companion_radio_ble
ThinkNode_M1_companion_radio_ble
ThinkNode_M5_companion_radio_ble
GAT562_Mesh_Tracker_Pro_companion_radio_ble
GAT562_30S_Mesh_Kit_companion_radio_ble
Mesh_pocket_companion_radio_ble
LilyGo_T-Echo_companion_radio_ble
LilyGo_T-Echo-Lite_companion_radio_ble
RAK_WisMesh_Tag_companion_radio_ble
t1000e_companion_radio_ble
Heltec_Wireless_Paper_companion_radio_ble
Heltec_t114_companion_radio_ble
Heltec_v3_companion_radio_ble
heltec_v4_companion_radio_ble
Heltec_t096_companion_radio_ble
ProMicro_companion_radio_ble
RAK_4631_companion_radio_ble
Xiao_nrf52_companion_radio_ble
LilyGo_TBeam_1W_companion_radio_ble
Tbeam_SX1262_companion_radio_ble
Tbeam_SX1276_companion_radio_ble
MTL_1_companion_radio_ble
MTL_1_SX1268_companion_radio_ble
MTL_2_sx1262_sh1106_companion_radio_ble
MTL_2_sx1262_ssd1306_companion_radio_ble
MTL_2_sx1268_sh1106_companion_radio_ble
MTL_2_sx1268_ssd1306_companion_radio_ble
```

### Selected repeater set

`build-repeater-firmwares` builds these environments:

```text
Heltec_v3_repeater
heltec_v4_repeater
Heltec_t096_repeater
Heltec_t114_repeater
GAT562_30S_Mesh_Kit_repeater
ProMicro_repeater
MTL_1_repeater
MTL_1_SX1268_repeater
MTL_2_sx1262_sh1106_repeater
MTL_2_sx1262_ssd1306_repeater
MTL_2_sx1268_sh1106_repeater
MTL_2_sx1268_ssd1306_repeater
Tbeam_SX1276_repeater
Tbeam_SX1262_repeater
LilyGo_TBeam_1W_repeater
heltec_v4_repeater_bridge_espnow
MTL_1_repeater_bridge_espnow
MTL_1_SX1268_repeater_bridge_espnow
```

### Bot firmware

`simple_bot_ping` is not part of the selected companion, repeater or room-server
sets. Build it explicitly:

```bash
export FIRMWARE_VERSION=v1.18.0
sh build.sh build-firmware LilyGo_T-Echo_bot_ping
```

See [Bot Mr.Pong](./simple_bot_ping.md) for the bot commands and behaviour.

## Environment variables

| Variable | Meaning |
|---|---|
| `FIRMWARE_VERSION` | Required. Base version, for example `v1.18.0`. |
| `COMMIT_HASH` | Optional. Short git hash embedded in the firmware and filename. Defaults to `git rev-parse --short HEAD`. |
| `FIRMWARE_VERSION_STRIP` | Suffix removed from the archive name. Defaults to `-meshcore.spb.ru`. |
| `DISABLE_DEBUG=1` | Removes debug logging flags (`MESH_DEBUG`, `MESH_PACKET_LOGGING`, BLE/Wi-Fi/bridge/GPS debug, RadioLib debug, etc.). |

The version part of the `ver` output is:

```text
<FIRMWARE_VERSION>-<COMMIT_HASH>
```

The `ver` command appends the build date, which `build.sh` generates from the
current date: `<FIRMWARE_VERSION>-<COMMIT_HASH> (Build: <date>)`.

## Artifacts

For every target, `build.sh` copies the platform-specific files into `out/`,
then packs them into one archive and removes the individual files:

| Platform | Files copied to `out/` before packing |
|---|---|
| ESP32 | `<name>-upgrade.bin`, `<name>-freshInstall-merged.bin` |
| nRF52 | `<name>.uf2`, `<name>.zip` |
| STM32 | `<name>.bin`, `<name>.hex` |
| RP2040 | `<name>.bin`, `<name>.uf2` |

The final file is `<env>-<archive-version>.zip`, where the archive version is
`FIRMWARE_VERSION` with `FIRMWARE_VERSION_STRIP` removed. For example, a build of
`RAK_4631_repeater` with `FIRMWARE_VERSION=v1.18.0-meshcore.spb.ru` produces:

```text
out/RAK_4631_repeater-v1.18.0.zip
```

## Global radio defaults

The shared `[arduino_base]` section defines the fallback radio settings for
every variant that does not override them:

```ini
-D LORA_FREQ=868.856
-D LORA_BW=62.5
-D LORA_SF=7
-D LORA_CR=7
```

This is the single place to change the frequency, bandwidth, spreading factor or
coding rate for a whole build set. Variants that need different values
(433 MHz boards, for example) override them in their own `platformio.ini`.
`LORA_TX_POWER` is deliberately left to the variant, because the safe value
depends on the board's PA/FEM.

Variants that used to repeat the same `868.856 / 62.5 / SF7 / CR7` values no
longer do so; they inherit them from `[arduino_base]` and only keep their
board-specific overrides.

## Variant radio defaults

The table below shows the effective values for variants that still need a
board-specific radio setting. 868 MHz entries inherit frequency, bandwidth, SF
and CR from `[arduino_base]`; only TX power remains in the variant file. The
433 MHz variants keep their frequency/CR overrides.

| Variant | `LORA_FREQ` | `LORA_BW` | `LORA_SF` | `LORA_CR` | `LORA_TX_POWER` |
|---|---:|---:|---:|---:|---:|
| GAT562 30S Mesh Kit | 868.856 MHz | 62.5 kHz | 7 | 7 | 18 dBm |
| Heltec T096 | 868.856 MHz | 62.5 kHz | 7 | 7 | 18 dBm |
| Heltec V3 | 868.856 MHz | 62.5 kHz | 7 | 7 | 22 dBm |
| Heltec V4 | 868.856 MHz | 62.5 kHz | 7 | 7 | 18 dBm |
| MTL-1 (SX1262) | 868.856 MHz | 62.5 kHz | 7 | 7 | 18 dBm |
| MTL-1 (SX1268) | 433.000 MHz | 62.5 kHz | 7 | 5 | 18 dBm |

All of these can be overridden with the `LORA_FREQ`, `LORA_BW`, `LORA_SF`,
`LORA_CR` and `LORA_TX_POWER` build flags.

## Variant documentation

- [MTL-1](https://github.com/alexxxst/MeshCore/blob/main/variants/mtl_1/README.md)
- [MTL-2](https://github.com/alexxxst/MeshCore/blob/main/variants/mtl_2/README.md)
