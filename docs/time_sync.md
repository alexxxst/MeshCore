# Repeater Time Sync

Time sync is available in the `simple_repeater` firmware only. A repeater can
set its clock from adverts signed by repeaters that are explicitly listed in its
trust list. Adverts from every other node are ignored, and a forged advert
cannot move the clock because the mesh layer verifies the signature before the
time-sync code sees it.

## Trust list

The trusted public keys are stored in `/timesync` on the repeater filesystem:

| Offset | Size | Meaning |
|---|---|---|
| 0 | 1 | magic `0x54` (`'T'`) |
| 1 | 1 | format version, currently `1` |
| 2 | 1 | number of keys, `0`-`8` |
| 3 | 32 × count | Ed25519 public keys |

* At most **8** trusted keys are kept.
* A key is a full 32-byte Ed25519 public key.
* The file is written when the list changes. A zero-byte file is a valid,
  explicitly empty list: it wins over the build-time defaults and leaves time
  sync off until a key is added.
* A malformed non-empty file is ignored and the build-time defaults are used.
* If `/timesync` does not exist, the build-time defaults from
  `examples/simple_repeater/TimeSyncDefaults.h` are loaded into RAM.

### Build-time defaults

`examples/simple_repeater/TimeSyncDefaults.h` contains an array of 32-byte
keys. The file ships with the keys used by the `meshcore.spb.ru` deployment.
Review and replace them before building firmware for your own mesh; otherwise
your repeaters will trust those sources by default. The list is compiled into
the firmware, so changing it requires a rebuild.

`timesync defaults` restores the compiled-in list and removes `/timesync`, so
the defaults are used from RAM and are not written to flash. The next change to
the list creates the file again.

## Commands

| Command | Description |
|---|---|
| `timesync` / `timesync list` | Show trusted keys and the last sync. |
| `timesync add <prefix\|pubkey>` | Add a trusted key. |
| `timesync remove <prefix>` | Remove every key matching the prefix. |
| `timesync clear` | Empty the list and save an empty file (time sync off). |
| `timesync defaults` | Load the build-time defaults and remove `/timesync`. |

`add` and `remove` accept **8 to 64 hex characters** (4 to 32 bytes). A prefix
shorter than the full key is resolved against the neighbours shown by the
`neighbors` command and is accepted only when it matches exactly one neighbour.
A full 64-character public key is accepted without a neighbour lookup. A
repeater cannot trust itself, and the list is capped at 8 keys.

`timesync list` prints one short (4-byte) prefix per trusted key, for example:

```text
trusted 3/8: abcd0e17 a000000b af589427 | build defaults | last sync 42s ago from abcd0e17
```

When the command is run on the serial console, the full 32-byte keys are also
printed, one per line, so they can be copied to another repeater.

`timesync clear` and `timesync defaults` also reset the last-sync status. The
defaults command reports an error if it loaded the keys but could not remove
`/timesync`; the loaded keys still apply until reboot.

## How an advert moves the clock

The clock is updated only when all of these checks pass:

1. The trust list is not empty and contains the advert sender's public key.
2. The advert timestamp is inside the supported epoch window:
   **2026-11-01 00:00 UTC** to **2029-11-01 00:00 UTC**.
3. The timestamp is newer than the last timestamp accepted from that sender.
   Replayed and older adverts are ignored. Up to 8 sources are tracked in RAM;
   when the cache is full, the least recently heard source is evicted.
4. The corrected target time differs from the current clock by more than the
   10-second deadband.

The target is the advert timestamp plus the estimated LoRa airtime of the
source transmission and every hop:

```text
target = advert_timestamp + airtime_ms * (1 + hops)
```

`hops` is the number of path hashes in the advert, capped at 16 so a bogus path
cannot overflow the correction. The result is rounded to the nearest second.

The clock normally moves **forward only**. A backward correction is allowed
only for a direct, zero-hop advert that is not a flood packet and not a Share
advert; flood, relayed and Share adverts can never move the clock back.

Time sync is passive: it does not poll, and it does not send requests. The
clock changes only when a trusted repeater's advert arrives.
