# Repeater Remote Command Packets

Remote command packets let a signed `GRP_DATA` message execute CLI commands on
a repeater. The repeater still forwards the packet as usual; it never marks it
as delivered locally.

This feature is compiled into `simple_repeater` only.

## Channel and keys

The repeater listens on one group channel, **K0** (16-byte MeshCore channel
key). The packet must use `data_type` `0xFFFE` (`REMOTE_CMD_DATA_TYPE`, also
aliased as `DATA_TYPE_REMOTE_COMMAND`).
The signed payload is placed in the group datagram's `blob` field:

```
GRP_DATA decrypted: [data_type LE16][data_len][blob]
```

The blob is the output of
[ed25519-signer](https://github.com/alexxxst/ed25519-signer):

```
blob = C || S
P    = timestamp_le32 || dict_id_le16 || cmdpack(text)
S    = Ed25519_sign(P)
C    = P XOR SHA512("MC1-mask" || S[:32] || counter_le32)
```

- `timestamp` is Unix UTC seconds.
- `dict_id` identifies the token dictionary; the current value is `0xB8A1`
  (wire bytes `A1 B8`).
- The last 64 bytes are the signature. K1 is tried first, then K2.
- The blob limit is 160 bytes, matching the signer.

### Keys

`examples/simple_repeater/RemoteCommandKeys.h` contains the 16-byte channel
key K0 and the two 32-byte Ed25519 public keys K1/K2.

## Dictionary

`examples/simple_repeater/RemoteCommandDictionary.h` is generated from
`tools/remote_command_dictionary.txt` and commits the exact token order:

```bash
python3 tools/gen_remote_command_dict.py
```

The generated header contains the packed token blob, `uint16_t` offsets and
`dict_id`. The dictionary has 156 tokens. If the signer dictionary changes,
regenerate the header and update the pinned test.

## Command execution

- Commands are split on `0x0A`; `\r` is stripped.
- Leading spaces and an optional companion-style `xx|` prefix are removed.
- At most **10 commands** per packet; more rejects the whole packet.
- A command longer than **128 characters** is logged `fail` and skipped.
- Commands are executed as if typed into the serial console:
  `handleCommand(0, NULL, ...)`.
- The first command waits until the packet was retransmitted and the outbound
  queue is empty. If the packet is not forwarded, a 35-second fallback starts
  execution.
- Commands are spaced 1.5 seconds apart.
- Up to four verified packets are queued in arrival order. A fifth is dropped
  and does not advance the replay timestamp.
- Every command is written to F2 before it runs.

`set guest.password ` keeps exactly one trailing space so an empty guest
password can be set. Other trailing spaces are removed.

## Blocked commands

Matching is a raw prefix, matching `CommonCLI`'s own `memcmp()` behaviour, so
`poweroffblabla` is blocked even though a naive word-boundary check would let it
through.

```
poweroff                 shutdown
advert                   start ota
erase                    clear stats
set name                 set lat
set lon                  set prv.key
get prv.key              password
set owner.info           set repeat
setperm                  gps setloc
gps advert               sensor set
set bridge               set freq
set reboot.interval      region load
log
```

`advert.zerohop` is the only exception: it is allowed when followed by the end
of the line or a space. `advert.zerohopX` is blocked because `CommonCLI` would
otherwise fall through to the flood `advert` handler.

Other `get ...` commands, `get/set guest.password`, `tempradio`, `reboot` and
`clkreboot` remain allowed. `get prv.key` is explicitly blocked.

## Radio bands

Timed radio changes may only stay in the same band:

| Band | Range |
|---|---|
| `433` | 430–450 MHz |
| `868` | 863–870 MHz |
| `915` | 902–928 MHz |
| `2.4G` | 2400–2500 MHz |

A frequency outside the table is rejected. The occupied bandwidth
(`freq ± bw/2`) must also stay inside the band. This check applies to
`set radio` and `tempradio`.

## `set radio` guard (F1)

`set radio` is applied only after a reboot, so the 10-minute connectivity
check starts after the reboot.

1. Validate the parameters and the band.
2. Save the old `freq/bw/sf/cr` and the target `freq/bw/sf/cr` to
   `/remote_cmd_radio` (F1).
3. Write `set radio ... : pending` to F2.
4. Execute the command (saves the new preferences for the next boot).
5. On the next boot, if F1 exists:
   - check that the boot parameters match F1's target. If they do not (power
     was lost before preferences were saved), mark the command `fail`, restore
     the old parameters and remove F1;
   - otherwise increment `attempts` and run the window; if a window is
     interrupted by another reboot after the fifth attempt, restore the old
     settings unconditionally; the fifth window itself still runs normally;
   - start a 10-minute window;
   - at 8 minutes, if fewer than 3 packets were received, send
     `sendNodeDiscoverReq()`;
   - at 10 minutes:
     - 3 or more received packets: mark `ok`, delete F1;
     - fewer: mark `fail`, restore the old settings, delete F1, reboot.

The received-packet counter is maintained by the remote-command module and is
not reset by `clear stats`. While the 10-minute window is active the node is
kept awake so the deadline cannot be lost to MCU sleep.

The reboot is mandatory: until it happens the new parameters are not active,
F1 stays armed, and later radio-changing commands fail. A packet that contains
`set radio` should include `reboot` as the next command.

Any successfully parsed received packet counts, including adverts, ACKs,
control packets and group data. The timer uses `millis()` and does not depend on
the RTC. While the window is active the node is kept awake.

If `set radio` specifies the current parameters, no guard is armed and the log
entry is `ok` immediately.

## `set tx` guard

`set tx` is special because it can be tested without a reboot:

- If the new value is **equal to or greater than** the current value, the
  setting is applied and saved immediately, and the log entry is `ok`.
- If the new value is **lower**, the radio is changed in the chip only. The
  preferences are not written until the guard succeeds:
  - at 10 seconds send `sendNodeDiscoverReq()`;
  - wait 60 seconds for a response to that discover;
  - if none, send a second discover and wait another 60 seconds;
  - any response to either of our tags means success: save the new tx and mark
    `ok`;
  - after the second minute with no response, restore the old tx and mark
    `fail`.

Only discover responses count. The discover request asks for repeaters, so a
lower-tx guard needs at least one repeater neighbour; companion-only meshes do
not answer and the guard will revert. A reboot during the guard is safe:
preferences still contain the old tx, so the next boot applies it. A pending `set tx` entry
is reconciled on boot by comparing its requested value with the saved
preference: equal means the save happened (`ok`), different means `fail`.

The range is **-9 to 22 dBm**. `get tx` reports the saved value; while a lower
tx guard is running, the chip already has the new value.

## `tempradio`

`tempradio` remains a normal command, but it must pass the same band check. It
keeps its own revert timer and does not use F1. The timeout is capped at 1440
minutes. `tempradio` and `set radio` are mutually exclusive: whichever
radio-changing command runs first makes the other commands fail until the
temporary settings revert or the radio guard finishes.

## F2 command log

`/remote_cmd_log` is a compact binary log of the last 30 commands. The CLI view
is newest-first:

```bash
cmdlog        # newest page
cmdlog 10     # skip the 10 newest entries
```

Format:

```
2026-11-17 12:34:56 : advert : fail
2026-11-17 12:35:10 : set radio 869.618,62.5,8,5 : pending
2026-11-17 12:35:12 : reboot : ok
2026-11-17 12:45:10 : set radio 869.618,62.5,8,5 : ok
```

Status is separated by a second colon and is `ok`, `fail` or `pending`.
Blocked, too-long and radio-blocked commands are logged `fail` immediately.
Pending entries are protected from rotation.

## Replay protection

A packet is accepted only when its timestamp is strictly greater than the
maximum already accepted; the rule assumes at most one command packet per
second. The maximum is persisted at acceptance in one of two small journal
files (`/remote_cmd_ts0`, `/remote_cmd_ts1`, alternating writes), so a reboot
between acceptance and execution does not reopen the replay window. F2 also
carries the maximum as a convenience. A packet rejected because the queue is
full does not advance the maximum, so it can be resent.

## Known CommonCLI limitations

This module deliberately does not modify `CommonCLI`. Two existing behaviours
are compensated for:

1. **Raw prefixes without a boundary.** `CommonCLI` matches several commands
   with `memcmp()` only:
   - `poweroffblabla` calls `powerOff()`;
   - `advertblabla` sends a flood advert;
   - `logblabla` dumps the log;
   - `get prv.keyblabla` returns the private key in the reply under the serial
     context;
   - `advert.zerohopX` falls through to the `advert` flood handler.
   The remote ban list therefore uses raw prefixes and whitelists only
   `advert.zerohop`.

2. **`CommonCLI::tmp[132]`.** `tempradio` copies its arguments into a
   132-byte `tmp` buffer with `strcpy`. A serial line of 159 characters can
   overflow it by 18 bytes (`tempradio ` + 149 argument characters). Remote
   command lines are capped at 128 characters until `CommonCLI` is changed to
   bounded copies.

## Files

| File | Purpose |
|---|---|
| `/remote_cmd_radio` | F1: old and target radio parameters, attempts, pending F2 entry |
| `/remote_cmd_log` | F2: append-only journal of the last 30 commands and status updates |
| `/remote_cmd_ts0`, `/remote_cmd_ts1` | two-slot journal with the maximum accepted timestamp |

All files are optional. A missing or malformed F1 is ignored; a malformed F2
is reset. The timestamp journal keeps replay protection even if F2 is lost.

Full-file writes (F1, F2 compaction and the timestamp slots) go through a
`.tmp` file followed by rename. If power is lost between the write and the
rename, the previous file remains valid and the `.tmp` copy is used as a
recovery source on the next boot. F2 command records are still appended
directly.

F2 is an append-only journal: a small command record is appended before the
command runs, and a `pending → ok/fail` transition appends a small status
record. When the journal grows past about 60 records it is compacted to the
current 30 commands. A failed journal append stops the command, so execution
cannot outrun the audit log.
