#pragma once

#include <string.h>

#include "CyrillicFont.h"
#include "CyrillicGlyphs.h"

// Text layout for GFX-like devices whose Latin font is the classic 6x8 cell
// font (Adafruit GFX default, TFT_eSPI GLCD, LovyanGFX Font0). Cyrillic glyphs
// are drawn with drawPixel() at the same pen position; ASCII still goes through
// the device's own print(). All coordinates are physical pixels and the pen y
// is the top of the Latin cell.

namespace cyr {

template <typename D>
inline int advance(const D&, int size, uint32_t cp) {
  const int s = size < 1 ? 1 : size;
  if (cp < 32) return 0;  // controls and newlines take no horizontal room
  const int idx = glyphIndex(cp);
  if (idx >= 0) return (int)advanceFor(CYR_OLED, idx) * s;
  return 6 * s;  // ASCII and unsupported code points (rendered as a space)
}

template <typename D>
inline void drawGlyph(D& d, int size, int x, int y, int idx, uint16_t color) {
  if (idx < 0) return;
  const int s = size < 1 ? 1 : size;
  const int w = advanceFor(CYR_OLED, idx);
  for (int row = 0; row < CYR_OLED.height; row++) {
    int run = -1;
    for (int col = 0; col <= w; col++) {
      const bool on = col < w && pixelFor(CYR_OLED, idx, col, row);
      if (on) {
        if (run < 0) run = col;
      } else if (run >= 0) {
        for (int px = run; px < col; px++) {
          for (int sy = 0; sy < s; sy++) {
            for (int sx = 0; sx < s; sx++) {
              d.drawPixel(x + px * s + sx, y + row * s + sy, color);
            }
          }
        }
        run = -1;
      }
    }
  }
}

// Prints [begin, end) at the device cursor. ASCII runs go through d.print(),
// Cyrillic is drawn manually, unsupported code points consume a space.
template <typename D>
inline void printRange(D& d, int size, const char* begin, const char* end,
                       uint16_t color) {
  const int s = size < 1 ? 1 : size;
  int x = d.getCursorX();
  int y = d.getCursorY();
  const char* p = begin;

  while (p < end && *p) {
    if ((uint8_t)*p < 0x80) {
      if (*p == '\n') {
        x = 0;
        y += 8 * s;
        d.setCursor(x, y);
        p++;
        continue;
      }
      if ((uint8_t)*p < 32) {
        p++;
        continue;
      }
      char buf[48];
      size_t n = 0;
      while (p < end && *p && (uint8_t)*p < 0x80 && *p != '\n' &&
             n + 1 < sizeof(buf)) {
        buf[n++] = *p++;
      }
      if (n > 0) {
        buf[n] = 0;
        d.setCursor(x, y);
        d.print(buf);
        x = d.getCursorX();
        y = d.getCursorY();
      }
      continue;
    }

    uint32_t cp = 0;
    const char* next = utf8Next(p, cp);
    if (next > end) next = end;
    if (cp == '\n') {
      x = 0;
      y += 8 * s;
      d.setCursor(x, y);
    } else {
      const int idx = glyphIndex(cp);
      drawGlyph(d, s, x, y, idx, color);
      x += advance(d, s, cp);
      d.setCursor(x, y);
    }
    p = next;
  }

  d.setCursor(x, y);
}

// Returns true when the string contained non-ASCII characters and was fully
// handled here. ASCII-only strings are left to the driver's own print().
template <typename D>
inline bool print(D& d, int size, const char* str, uint16_t color) {
  if (!hasNonAscii(str)) return false;
  printRange(d, size, str, str + strlen(str), color);
  return true;
}

template <typename D>
inline int width(const D& d, int size, const char* str) {
  int w = 0;
  const char* p = str;
  while (*p) {
    uint32_t cp = 0;
    p = utf8Next(p, cp);
    w += advance(d, size, cp);
  }
  return w;
}

// Word-wraps mixed text inside max_width and above max_y. Breaks at spaces,
// '-' and '/', or between code points when a single word is too long.
template <typename D>
inline bool wordWrap(D& d, int size, const char* str, int max_width, int max_y,
                     uint16_t color) {
  if (!hasNonAscii(str)) return false;

  const int s = size < 1 ? 1 : size;
  const int origin = d.getCursorX();
  int y = d.getCursorY();
  const int glyph_h = (int)CYR_OLED.height * s;
  int step = 8 * s;
  if ((int)(CYR_OLED.height + 1) * s > step) step = (int)(CYR_OLED.height + 1) * s;

  const char* p = str;
  while (*p) {
    if (y + glyph_h > max_y) break;

    const char* line = p;
    int w = 0;
    const char* brk = nullptr;
    const char* end = p;
    while (*end && *end != '\n') {
      uint32_t cp = 0;
      const char* next = utf8Next(end, cp);
      const int cw = advance(d, s, cp);
      if (w + cw > max_width && end != line) break;
      w += cw;
      if (cp == ' ' || cp == '-' || cp == '/') brk = next;
      end = next;
    }

    const char* cut = end;
    if (*end && *end != '\n' && brk && brk > line) cut = brk;
    d.setCursor(origin, y);
    printRange(d, s, line, cut, color);

    if (*cut == '\n' || *cut == ' ') cut++;
    if (cut == p) break;
    p = cut;
    if (*p) y += step;
  }

  return true;
}

}  // namespace cyr
