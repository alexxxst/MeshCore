#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Display-side Cyrillic support. Header-only on purpose: files in
// subdirectories are not picked up by the variant build_src_filter, so this
// keeps platformio.ini / variants untouched (and mergeable with upstream).
//
// Glyph coverage is Russian: U+0410..U+044F plus U+0401/U+0451. Everything
// else non-ASCII is rendered as a single space.

#ifndef CYRILLIC_SUPPORT
#define CYRILLIC_SUPPORT 1
#endif

namespace cyr {

// Returns the next code point. A malformed or truncated sequence yields its
// lead byte as the code point and advances by one byte, so callers can never
// read past the terminating NUL.
inline const char* utf8Next(const char* s, uint32_t& cp) {
  const uint8_t c = (uint8_t)s[0];
  if (c == 0) {
    cp = 0;
    return s;
  }
  if (c < 0x80) {
    cp = c;
    return s + 1;
  }
  if ((c & 0xE0) == 0xC0 && ((uint8_t)s[1] & 0xC0) == 0x80) {
    cp = ((uint32_t)(c & 0x1F) << 6) | ((uint8_t)s[1] & 0x3F);
    return s + 2;
  }
  if ((c & 0xF0) == 0xE0 && ((uint8_t)s[1] & 0xC0) == 0x80 &&
      ((uint8_t)s[2] & 0xC0) == 0x80) {
    cp = ((uint32_t)(c & 0x0F) << 12) |
         ((uint32_t)((uint8_t)s[1] & 0x3F) << 6) |
         ((uint8_t)s[2] & 0x3F);
    return s + 3;
  }
  if ((c & 0xF8) == 0xF0 && ((uint8_t)s[1] & 0xC0) == 0x80 &&
      ((uint8_t)s[2] & 0xC0) == 0x80 && ((uint8_t)s[3] & 0xC0) == 0x80) {
    cp = ((uint32_t)(c & 0x07) << 18) |
         ((uint32_t)((uint8_t)s[1] & 0x3F) << 12) |
         ((uint32_t)((uint8_t)s[2] & 0x3F) << 6) |
         ((uint8_t)s[3] & 0x3F);
    return s + 4;
  }
  cp = c;
  return s + 1;
}

// Index into the Cyrillic glyph table: А..Я (0..31), Ё (32), а..я (33..64),
// ё (65). Returns -1 for anything else.
inline int glyphIndex(uint32_t cp) {
  if (cp >= 0x0410 && cp <= 0x042F) return (int)(cp - 0x0410);
  if (cp == 0x0401) return 32;
  if (cp >= 0x0430 && cp <= 0x044F) return 33 + (int)(cp - 0x0430);
  if (cp == 0x0451) return 65;
  return -1;
}

inline bool hasNonAscii(const char* s) {
  for (; *s; s++) {
    if ((uint8_t)*s >= 0x80) return true;
  }
  return false;
}

// Collapses runs of spaces into one and trims both ends, in place.
inline void squeezeSpaces(char* s) {
  char* out = s;
  for (const char* p = s; *p; ++p) {
    if (*p == ' ' && (out == s || out[-1] == ' ')) continue;
    *out++ = *p;
  }
  while (out > s && out[-1] == ' ') --out;
  *out = 0;
}

// Keeps printable ASCII and Russian Cyrillic bytes; any other code point
// becomes one space. The result is always NUL-terminated and never contains a
// half-written UTF-8 sequence.
inline void translateUtf8KeepCyrillic(char* dest, const char* src,
                                      size_t dest_size) {
  if (dest == nullptr || dest_size == 0) return;

  size_t j = 0;
  const char* p = src;
  while (*p && j + 1 < dest_size) {
    const char* start = p;
    uint32_t cp = 0;
    p = utf8Next(p, cp);

    if (cp >= 32 && cp <= 126) {
      dest[j++] = (char)cp;
    } else if (glyphIndex(cp) >= 0) {
      const size_t n = (size_t)(p - start);
      if (j + n >= dest_size) break;  // leave room for the NUL
      memcpy(dest + j, start, n);
      j += n;
    } else {
      dest[j++] = ' ';
    }
  }

  dest[j] = 0;
  squeezeSpaces(dest);
}

}  // namespace cyr
