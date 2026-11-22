#ifdef ST7789

#include "ST7789Display.h"

#ifndef X_OFFSET
#define X_OFFSET 0  // No offset needed for landscape
#endif

#ifndef Y_OFFSET
#define Y_OFFSET 1  // Vertical offset to prevent top row cutoff
#endif

#ifdef HELTEC_VISION_MASTER_T190
  #define SCALE_X  2.5f        // 320 / 128
  #define SCALE_Y  2.65625f    // 170 / 64
#else
  #define SCALE_X  1.875f      // 240 / 128
  #define SCALE_Y  2.109375f   // 135 / 64
#endif

#ifdef DISPLAY_SCALE_X
  #define SCALE_X DISPLAY_SCALE_X
#endif

#ifdef DISPLAY_SCALE_Y
  #define SCALE_Y DISPLAY_SCALE_Y
#endif

// Color scheme
ColorVal UIColor::window_bkg = OLEDDISPLAY_COLOR::BLACK;
ColorVal UIColor::title_bkg = OLEDDISPLAY_COLOR::BLACK;
ColorVal UIColor::title_txt = OLEDDISPLAY_COLOR::WHITE;
ColorVal UIColor::primary_txt = OLEDDISPLAY_COLOR::WHITE;
ColorVal UIColor::secondary_txt = OLEDDISPLAY_COLOR::WHITE;
ColorVal UIColor::warning_txt = OLEDDISPLAY_COLOR::WHITE;
ColorVal UIColor::popup_bkg = OLEDDISPLAY_COLOR::BLACK;
ColorVal UIColor::popup_txt = OLEDDISPLAY_COLOR::WHITE;
ColorVal UIColor::corp_blue = OLEDDISPLAY_COLOR::WHITE;

bool ST7789Display::begin() {
  if(!_isOn) {
    pinMode(PIN_TFT_VDD_CTL, OUTPUT);
    pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
    digitalWrite(PIN_TFT_VDD_CTL, LOW);
  #ifdef PIN_TFT_LEDA_CTL_ACTIVE
    digitalWrite(PIN_TFT_LEDA_CTL, PIN_TFT_LEDA_CTL_ACTIVE);
  #else
    digitalWrite(PIN_TFT_LEDA_CTL, LOW);
  #endif
    digitalWrite(PIN_TFT_RST, HIGH);

    display.init();
    display.landscapeScreen();
    #ifdef DISPLAY_FLIP_VERTICALLY
    display.flipScreenVertically();
    #endif
    display.displayOn();
    setCursor(0,0);

    _isOn = true;
  }
  return true;
}

void ST7789Display::turnOn() {
  if (!_isOn) {
    // Restore power to the display but keep backlight off
    digitalWrite(PIN_TFT_VDD_CTL, LOW);
    digitalWrite(PIN_TFT_RST, HIGH);
    
    // Re-initialize the display
    display.init();
    display.displayOn();
    #ifdef DISPLAY_FLIP_VERTICALLY
    display.flipScreenVertically();
    #endif
    delay(20);

    // Now turn on the backlight
  #ifdef PIN_TFT_LEDA_CTL_ACTIVE
    digitalWrite(PIN_TFT_LEDA_CTL, PIN_TFT_LEDA_CTL_ACTIVE);
  #else
    digitalWrite(PIN_TFT_LEDA_CTL, LOW);
  #endif    
    _isOn = true;
  }
}

void ST7789Display::turnOff() {
  digitalWrite(PIN_TFT_VDD_CTL, HIGH);
#ifdef PIN_TFT_LEDA_CTL_ACTIVE
  digitalWrite(PIN_TFT_LEDA_CTL, !PIN_TFT_LEDA_CTL_ACTIVE);
#else
  digitalWrite(PIN_TFT_LEDA_CTL, HIGH);
#endif
  digitalWrite(PIN_TFT_RST, LOW);
  _isOn = false;
}

void ST7789Display::clear() {
  display.clear();
}

void ST7789Display::startFrame(ColorVal bkg) {
  display.clear();  // TODO: use bkg
  setColor(UIColor::primary_txt);
  _textSize = 1;
  display.setFont(ArialMT_Plain_16);
}

void ST7789Display::setTextSize(int sz) {
  _textSize = sz;
  switch(sz) {
    case 1 :
      display.setFont(ArialMT_Plain_16);
      break;
    case 2 :
      display.setFont(ArialMT_Plain_24);
      break;
    default:
      display.setFont(ArialMT_Plain_16);
  }
}

void ST7789Display::setColor(ColorVal c) {
  _color = c;
  display.setColor((OLEDDISPLAY_COLOR)_color);
  display.setRGB(_color == OLEDDISPLAY_COLOR::WHITE ? ST77XX_WHITE : ST77XX_BLACK);
}

void ST7789Display::setCursor(int x, int y) {
  _lx = x;
  _ly = y;
  _x = x*SCALE_X + X_OFFSET;
  _y = y*SCALE_Y + Y_OFFSET;
}

const CyrBitmapFont& ST7789Display::activeCyrFace() const {
  return _textSize >= 2 ? CYR_TFT2 : CYR_TFT;
}

void ST7789Display::drawCyrGlyph(int idx, int x, int y) {
  if (idx < 0) return;
  const CyrBitmapFont& font = activeCyrFace();
  const int w = advanceFor(font, idx);
  for (int row = 0; row < font.height; row++) {
    int run = -1;
    for (int col = 0; col <= w; col++) {
      const bool on = col < w && pixelFor(font, idx, col, row);
      if (on) {
        if (run < 0) run = col;
      } else if (run >= 0) {
        display.fillRect(x + run, y + row, col - run, 1);
        run = -1;
      }
    }
  }
}

int ST7789Display::codepointWidth(uint32_t cp) {
  if (cp < 32 || cp == '\n') return 0;
  const int idx = cyr::glyphIndex(cp);
  if (idx >= 0) return advanceFor(activeCyrFace(), idx);
  if (cp < 0x80) {
    char buf[2] = {(char)cp, 0};
    return display.getStringWidth(buf);
  }
  return (activeCyrFace().height * 2) / 3;
}

void ST7789Display::printSpan(const char* begin, const char* end) {
  const char* p = begin;
  while (p < end && *p) {
    if ((uint8_t)*p < 0x80) {
      char buf[64];
      int n = 0;
      while (p < end && *p && (uint8_t)*p < 0x80 && n < (int)sizeof(buf) - 1) {
        buf[n++] = *p++;
      }
      buf[n] = 0;
      display.drawString(_x, _y, buf);
      _x += display.getStringWidth(buf);
    } else {
      uint32_t cp = 0;
      const char* next = cyr::utf8Next(p, cp);
      if (next > end) next = end;
      const int idx = cyr::glyphIndex(cp);
      drawCyrGlyph(idx, _x, _y);
      _x += codepointWidth(cp);
      p = next;
    }
  }
}

void ST7789Display::print(const char* str) {
#if CYRILLIC_SUPPORT
  if (cyr::hasNonAscii(str)) {
    printSpan(str, str + strlen(str));
    return;
  }
#endif
  display.drawString(_x, _y, str);
}

void ST7789Display::printWordWrap(const char* str, int max_width) {
#if CYRILLIC_SUPPORT
  if (cyr::hasNonAscii(str)) {
    int origin = _lx;
    int y = _ly;
    const int origin_px = (int)(origin * SCALE_X + X_OFFSET);
    int limit = display.getWidth() - origin_px;
    if (limit < 1) limit = 1;
    int max_px = (int)(max_width * SCALE_X + 0.5f);
    if (max_px > limit) max_px = limit;
    if (max_px < 1) max_px = 1;
    int step = (int)((activeCyrFace().height + 2) / SCALE_Y + 0.999f);
    if (step < 9) step = 9;

    const char* p = str;
    while (*p && y < height()) {
      const char* line = p;
      int width = 0;
      const char* brk = nullptr;
      const char* end = p;
      while (*end && *end != '\n') {
        uint32_t cp = 0;
        const char* next = cyr::utf8Next(end, cp);
        const int cw = codepointWidth(cp);
        if (width + cw > max_px && end != line) break;
        width += cw;
        if (cp == ' ' || cp == '-' || cp == '/') brk = next;
        end = next;
      }
      const char* cut = end;
      if (*end && *end != '\n' && brk && brk > line) cut = brk;
      setCursor(origin, y);
      printSpan(line, cut);
      if (*cut == '\n' || *cut == ' ') cut++;
      if (cut == p) break;
      p = cut;
      if (*p) {
        const int next_y = y + step;
        if (next_y + step - 1 > height()) break;
        y = next_y;
      }
    }
    _lx = origin;
    _ly = y;
    return;
  }
#endif
  display.drawStringMaxWidth(_x, _y, max_width*SCALE_X, str);
}

void ST7789Display::translateUTF8ToBlocks(char* dest, const char* src, size_t dest_size) {
#if CYRILLIC_SUPPORT
  cyr::translateUtf8KeepCyrillic(dest, src, dest_size);
#else
  DisplayDriver::translateUTF8ToBlocks(dest, src, dest_size);
#endif
}

void ST7789Display::fillRect(int x, int y, int w, int h) {
  display.fillRect(x*SCALE_X + X_OFFSET, y*SCALE_Y + Y_OFFSET, w*SCALE_X, h*SCALE_Y);
}

void ST7789Display::drawRect(int x, int y, int w, int h) {
  display.drawRect(x*SCALE_X + X_OFFSET, y*SCALE_Y + Y_OFFSET, w*SCALE_X, h*SCALE_Y);
}

void ST7789Display::drawXbm(int x, int y, const uint8_t* bits, int w, int h) {
  // Calculate the base position in display coordinates
  uint16_t startX = x * SCALE_X + X_OFFSET;
  uint16_t startY = y * SCALE_Y + Y_OFFSET;
  
  // Width in bytes for bitmap processing
  uint16_t widthInBytes = (w + 7) / 8;
  
  // Process the bitmap row by row
  for (uint16_t by = 0; by < h; by++) {
    // Calculate the target y-coordinates for this logical row
    int y1 = startY + (int)(by * SCALE_Y);
    int y2 = startY + (int)((by + 1) * SCALE_Y);
    int block_h = y2 - y1;
    
    // Scan across the row bit by bit
    for (uint16_t bx = 0; bx < w; bx++) {
      // Calculate the target x-coordinates for this logical column
      int x1 = startX + (int)(bx * SCALE_X);
      int x2 = startX + (int)((bx + 1) * SCALE_X);
      int block_w = x2 - x1;
      
      // Get the current bit
      uint16_t byteOffset = (by * widthInBytes) + (bx / 8);
      uint8_t bitMask = 0x80 >> (bx & 7);
      bool bitSet = pgm_read_byte(bits + byteOffset) & bitMask;
      
      // If the bit is set, draw a block of pixels
      if (bitSet) {
        // Draw the block as a filled rectangle
        display.fillRect(x1, y1, block_w, block_h);
      }
    }
  }
}

uint16_t ST7789Display::getTextWidth(const char* str) {
#if CYRILLIC_SUPPORT
  if (cyr::hasNonAscii(str)) {
    int px = 0;
    const char* p = str;
    while (*p) {
      uint32_t cp = 0;
      p = cyr::utf8Next(p, cp);
      px += codepointWidth(cp);
    }
    return (uint16_t)(px / SCALE_X);
  }
#endif
  return display.getStringWidth(str) / SCALE_X;
}

void ST7789Display::endFrame() {
  display.display();
}

#endif