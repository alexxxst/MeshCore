# Power Saving

This firmware has several independent power-saving mechanisms. They can be
combined, but they do different things:

| Mechanism | What it does | Control |
|---|---|---|
| `radio.rxps` | Duty-cycles the radio receiver between RX and sleep windows. | `get/set radio.rxps`, see [CLI commands](./cli_commands.md#view-or-change-rx-power-saving). |
| `powersaving` | Puts the MCU to sleep between work items. | `powersaving`, `powersaving on`, `powersaving off`. |
| GPS power saving | Switches the GPS module on and off on a schedule. | Tied to `powersaving`; see below. |
| `poweroff` / `shutdown` | Enters the deepest sleep the board supports. | CLI command or board button. |
| `reboot.interval` | Periodically reboots the node. | `get/set reboot.interval`. |

`radio.rxps` and `powersaving` are independent. Enabling `powersaving` does not
enable RX duty-cycle, and enabling `radio.rxps` does not put the MCU to sleep.

## Fresh-install defaults

The repeater, room server, sensor and companion firmwares all default to power
saving **on** for a fresh install (an existing `/prefs.json` keeps its saved
value):

| Setting | Fresh-install value |
|---|---|
| `powersaving` | `on` (repeater, room server, sensor and companion) |
| `radio.rxps` | `balanced` on repeater and sensor; companion always applies `balanced` |
| `reboot.interval` | `0` (disabled) |

The companion firmware does not expose the `powersaving` or `radio.rxps` text
commands; it enables power saving internally and applies the fixed `balanced`
RXPS profile at startup and after radio-parameter changes.

## `powersaving` command

**Usage:**

- `powersaving` — report `on` or `off`.
- `powersaving on` — enable power saving.
- `powersaving off` — disable power saving.

The setting is saved. `powersaving off` takes effect immediately; `on` takes
effect immediately on nRF52, while ESP32 starts sleeping once the first-sleep
timer (120 seconds from boot) has elapsed. The reply depends on the board:

| Build | `powersaving on` reply | Effect |
|---|---|---|
| nRF52 | `on - Immediate effect` | Event-driven sleep (`sd_app_evt_wait()` / `WFE`); the `secs` argument is ignored and the node wakes on any interrupt. |
| ESP32, no bridge | `on - After 2 minutes` | Light sleep after the first 120 seconds, then between loop iterations. Wakes on LoRa IRQ or the user button; a button press inhibits sleep for 30 seconds. |
| ESP32 bridge build | `Bridge not supported` | The bridge needs the CPU awake, so `powersaving on` is rejected on ESP32. `powersaving off` remains available. |
| Other boards | `Board not supported` | No board-level sleep implementation. |

`powersaving` is available on repeater, room server and sensor firmware.
Companion firmware uses the same saved preference internally but does not
expose the text command.

## GPS power saving

When `powersaving` is on and a GPS provider is present, the GPS is not kept
running continuously. It wakes for a window and then powers down:

| Firmware | GPS on window | GPS off window |
|---|---:|---:|
| Companion | 10 minutes | 30 minutes |
| Repeater, room server, sensor | 10 minutes | 1 day |

While the GPS is awake it can go back to sleep early as soon as it has obtained
a valid time sync. It wakes again when:

* the off window expires;
* the user runs `gps sync`;
* a telemetry request asks for GPS data. Companion firmware wakes the GPS on
  every telemetry request; sensor firmware wakes it only when the last valid
  time sync is older than the compile-time `GPS_READ_INTERVAL_SECS` value
  (60 seconds by default).

`gps on` and `gps off` still control whether the GPS is enabled at all. When
the GPS is disabled, the power-saving schedule is not used.

`gps` shows the current power-saving state, for example:

```text
on, powered (powersaving, sleep in 0h 7m), fix, 8 sats, last sync: 12:34 - 16/11/2026 UTC
on, unpowered (powersaving, wake in 0h 24m), last sync: 12:34 - 16/11/2026 UTC
```

Use `sensor set gps_interval <seconds>` to change how often NMEA data is read
while the GPS is awake (default 1 second). This is not the same as the GPS
on/off schedule above.

## Deep sleep and power-off

`poweroff` and `shutdown` enter the deepest sleep the board supports and do not
send a reply:

* ESP32 boards call `enterDeepSleep(0)`: peripherals are shut down, stale wake
  sources are cleared, and the CPU enters deep sleep with no timer wake-up.
* nRF52 boards enter `SYSTEMOFF` after shutting down the display, radio, GPS and
  crypto peripheral. Wake sources depend on the board (button, VBUS/LPCOMP, or
  none).

Some boards support a user-button long press for power-off. `enterDeepSleep(secs)`
is also used internally for timed sleep: ESP32 wakes from the timer, and boards
that override deep sleep (for example MTL-1) can also wake on LoRa/button
sources. nRF52 shuts peripherals down, waits for the requested time and then
reboots for a fresh recovery.

To restart periodically instead of staying off, use `reboot.interval`:

```text
get reboot.interval
set reboot.interval 24
```

`reboot.interval` is enforced by repeater and sensor firmware. `0` disables it.

## Clock across resets

The clock is kept across resets so a reboot does not reset the node to the
compile-time seed:

* nRF52 keeps the time in the `.persistent_magic` / `.persistent_data` RAM
  sections, with magic `0xAA55CC33`.
* ESP32 keeps it in `RTC_NOINIT_ATTR` variables; ESP-IDF also preserves the
  system time across deep sleep.

If the backup is missing or invalid, the clock starts at the firmware seed
(1 March 2026). A repeater with a trust list can then correct it from a trusted
advert (see [Repeater Time Sync](./time_sync.md)), and any GPS-equipped board can
correct it with `gps sync`.

## Diagnostics

* `powerlog` — last reset reason; on nRF52 it also shows the last shutdown
  reason and the boot voltage.
* `get pwrmgt.support`, `get pwrmgt.source`, `get pwrmgt.bootreason`,
  `get pwrmgt.bootmv` — power-management status, source and boot reason.
* `gps diag` — GPS provider diagnostics.
* `POWERSAVING_DEBUG=1` — build-time compile flag that prints GPS sleep/wake
  transitions on the serial console.

See [nRF52 Power Management](./nrf52_power_management.md) for the nRF52
voltage-protection and shutdown-reason details.
