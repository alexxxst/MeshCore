#pragma once

// Comma-separated full 32-byte public keys (64 hex chars), maximum 8.
// Override at build time with:
//   -D TIMESYNC_DEFAULT_KEYS='"key1,key2,..."'
// Keys below are the trusted time sources listed at
// https://meshcore.spb.ru/wiki/timesync
#ifndef TIMESYNC_DEFAULT_KEYS
#define TIMESYNC_DEFAULT_KEYS \
  "abcd0e17584f7b9e89517e7532ef58870282213f8e4d593e662b75498689b43e," \
  "a000000bf62d456f973a61290f094d9fc4a36b9599291218618b1042b45a2a16," \
  "af589427f32897cef48d72003177d5a238a7289bf03897f6776df01ca37ae3f6," \
  "a6a6a689b135b84d205b716fc2f4149a8f78efb04210845cc5d3872a93be7b84," \
  "3f01c7a47ed236af8fe4ac03c8f7387483065006204ffbf94a80ddccc94e0215," \
  "8c0b5c461bc4158a3ec7d974798fe11c234712b86bf2fc15a405028d476b8573"
#endif
