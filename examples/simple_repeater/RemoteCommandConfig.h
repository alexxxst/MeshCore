#pragma once

#include <stdint.h>

// Wire limits from ed25519-signer. The signer refuses to produce a blob larger
// than 160 bytes; the firmware refuses to accept one.
#define REMOTE_CMD_MAX_BLOB          160
#define REMOTE_CMD_SIGNATURE_SIZE     64
#define REMOTE_CMD_KEY_SIZE           32
#define REMOTE_CMD_CHANNEL_KEY_LEN    16

// The compressed text is at most 90 bytes, but token expansion can be much
// larger. This is the hard ceiling for one packet: ten commands of at most
// REMOTE_CMD_MAX_LINE bytes plus separators.
#define REMOTE_CMD_MAX_LINE          128
#define REMOTE_CMD_MAX_COMMANDS       10
#define REMOTE_CMD_MAX_TEXT         1536

#define REMOTE_CMD_MAX_PAYLOAD       (REMOTE_CMD_MAX_BLOB - REMOTE_CMD_SIGNATURE_SIZE)

// Last free value in the testing/development range (0xFF00-0xFFFE). 0xFFFF is
// DATA_TYPE_DEV.
#define REMOTE_CMD_DATA_TYPE      0xFFFEu
#define DATA_TYPE_REMOTE_COMMAND  REMOTE_CMD_DATA_TYPE

#define REMOTE_CMD_LOG_ENTRIES        30
