
#include "GxEPDDisplay.h"

#ifdef EXP_PIN_BACKLIGHT
  #include <PCA9557.h>
  extern PCA9557 expander;
#endif

#ifndef DISPLAY_ROTATION
  #define DISPLAY_ROTATION 3
#endif

#ifdef ESP32
  SPIClass SPI1 = SPIClass(FSPI);
#endif

#ifndef EPD_WASHING_MACHINE_CYCLES
  #define EPD_WASHING_MACHINE_CYCLES 0
#endif

// Color scheme
ColorVal UIColor::window_bkg = GxEPD_WHITE;
ColorVal UIColor::title_bkg = GxEPD_WHITE;
ColorVal UIColor::title_txt = GxEPD_BLACK;
ColorVal UIColor::primary_txt = GxEPD_BLACK;
ColorVal UIColor::secondary_txt = GxEPD_BLACK;
ColorVal UIColor::warning_txt = GxEPD_BLACK;
ColorVal UIColor::popup_bkg = GxEPD_WHITE;
ColorVal UIColor::popup_txt = GxEPD_BLACK;
ColorVal UIColor::corp_blue = GxEPD_BLACK;

bool GxEPDDisplay::begin() {
  display.epd2.selectSPI(SPI1, SPISettings(4000000, MSBFIRST, SPI_MODE0));
#ifdef ESP32
  SPI1.begin(PIN_DISPLAY_SCLK, PIN_DISPLAY_MISO, PIN_DISPLAY_MOSI, PIN_DISPLAY_CS);
#else
  SPI1.begin();
#endif
  display.init(115200, true, 2, false);
  display.setRotation(DISPLAY_ROTATION);
  setTextSize(1);  // Default to size 1

  display.setFullWindow();

  for (int i = 0; i < EPD_WASHING_MACHINE_CYCLES; i++) {
    display.fillScreen(GxEPD_BLACK);
    display.display(false);
    delay(2000);
    display.fillScreen(GxEPD_WHITE);
    display.display(false);
    delay(2000);
  }

  display.setPartialWindow(0, 0, display.width(), display.height());
  resetPartialRefreshCounter();

  #if DISP_BACKLIGHT
  digitalWrite(DISP_BACKLIGHT, LOW);
  pinMode(DISP_BACKLIGHT, OUTPUT);
  #endif
  _init = true;
  _isOn = true;
  return true;
}

void GxEPDDisplay::turnOn() {
  if (!_init) begin();
#if defined(DISP_BACKLIGHT) && !defined(BACKLIGHT_BTN)
  digitalWrite(DISP_BACKLIGHT, HIGH);
#elif defined(EXP_PIN_BACKLIGHT) && !defined(BACKLIGHT_BTN)
  expander.digitalWrite(EXP_PIN_BACKLIGHT, HIGH);
#endif
  if (!_isOn) {
    _isOn = true;
  }
}

void GxEPDDisplay::turnOff() {
#if defined(DISP_BACKLIGHT) && !defined(BACKLIGHT_BTN)
  digitalWrite(DISP_BACKLIGHT, LOW);
#elif defined(EXP_PIN_BACKLIGHT) && !defined(BACKLIGHT_BTN)
  expander.digitalWrite(EXP_PIN_BACKLIGHT, LOW);
#endif
  _isOn = false;
  // do full refresh before powering off to clear screen
  // no full refresh needed at wakeup
  display.clearScreen(0xFF); // Clears microcontroller side RAM
  display.writeScreenBuffer(0xFF); // Forces 0xFF (White) into the display controller's history registers
  resetPartialRefreshCounter();
  last_display_crc_value=0;
  display.hibernate();
}

void GxEPDDisplay::clear() {
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display_crc.reset();
}

void GxEPDDisplay::startFrame(ColorVal bkg) {
  display.fillScreen(bkg);
  display.setTextColor(_curr_color = UIColor::primary_txt);
  display_crc.reset();
  if (_cycles_before_full_refresh != 0) {
    display.setPartialWindow(0, 0, display.width(), display.height());
  } else {
    // forces a full wipe of the screen ...
    display.clearScreen(0xFF); // Clears microcontroller side RAM
    display.writeScreenBuffer(0xFF); // Forces 0xFF (White) into the display controller's history registers
    // we'll need a partial refresh after that (whatever crc value is)
    last_display_crc_value = 0;
    resetPartialRefreshCounter();
  }
}

void GxEPDDisplay::setTextSize(int sz) {
  if (sz < 1) sz = 1;
  _textSize = sz;
  display_crc.update<int>(sz);
  switch(sz) {
    case 1:  // Small
      display.setFont(&FreeSans9pt7b);
      break;
    case 2:  // Medium Bold
      display.setFont(&FreeSansBold12pt7b);
      break;
    case 3:  // Large
      display.setFont(&FreeSans18pt7b);
      break;
    default:
      display.setFont(&FreeSans9pt7b);
      break;
  }
}

void GxEPDDisplay::setColor(ColorVal c) {
  display_crc.update<ColorVal> (c);
  display.setTextColor(_curr_color = c);
}

void GxEPDDisplay::setCursor(int x, int y) {
  _lx = x;
  _ly = y;
  display_crc.update<int>(x);
  display_crc.update<int>(y);
  display.setCursor((x+offset_x)*scale_x, (y+offset_y)*scale_y);
}

static const GFXfont* latinFace(int textSize) {
  switch (textSize) {
    case 2: return &FreeSansBold12pt7b;
    case 3: return &FreeSans18pt7b;
    default: return &FreeSans9pt7b;
  }
}

static int latinAdvance(const GFXfont* font, uint32_t cp) {
  if (cp < 32 || cp > 255) return 0;
  GFXfont face;
  memcpy_P(&face, font, sizeof(face));
  if ((uint16_t)cp < face.first || (uint16_t)cp > face.last) return 0;
  GFXglyph glyph;
  memcpy_P(&glyph, face.glyph + ((uint16_t)cp - face.first), sizeof(glyph));
  return glyph.xAdvance;
}

const CyrBitmapFont& GxEPDDisplay::cyrFace() const {
  return _textSize >= 2 ? CYR_EINK2 : CYR_EINK;
}

int GxEPDDisplay::codepointWidthPx(uint32_t cp) const {
  if (cp < 32 || cp == '\n') return 0;
  const int idx = cyr::glyphIndex(cp);
  if (idx >= 0) return advanceFor(cyrFace(), idx);
  if (cp < 0x80) return latinAdvance(latinFace(_textSize), cp);
  return (cyrFace().height * 2) / 3;
}

int GxEPDDisplay::textLineStep() const {
  GFXfont face;
  memcpy_P(&face, latinFace(_textSize), sizeof(face));
  int phys = face.yAdvance;
  const int cyr = (int)cyrFace().height + 2;
  if (cyr > phys) phys = cyr;
  if (phys < 1) phys = 1;
  int step = (int)(phys / scale_y);
  if ((float)step * scale_y < (float)phys - 0.01f) step++;
  return step < 1 ? 1 : step;
}

void GxEPDDisplay::drawCyrGlyph(int idx, int x, int baseline_y) {
  if (idx < 0) return;
  const CyrBitmapFont& font = cyrFace();
  const int w = advanceFor(font, idx);
  const int top = cyr::glyphTop(font, baseline_y);
  for (int row = 0; row < font.height; row++) {
    int run = -1;
    for (int col = 0; col <= w; col++) {
      const bool on = col < w && pixelFor(font, idx, col, row);
      if (on) {
        if (run < 0) run = col;
      } else if (run >= 0) {
        display.fillRect(x + run, top + row, col - run, 1, _curr_color);
        run = -1;
      }
    }
  }
}

void GxEPDDisplay::printSpan(const char* begin, const char* end) {
  if (!begin || end < begin) return;
  display_crc.update<char>(begin, (size_t)(end - begin));
  const char* p = begin;
  while (p < end && *p) {
    if ((uint8_t)*p < 0x80) {
      char buf[64];
      int n = 0;
      while (p < end && *p && (uint8_t)*p < 0x80 && n < (int)sizeof(buf) - 1) {
        buf[n++] = *p++;
      }
      buf[n] = 0;
      display.print(buf);
    } else {
      uint32_t cp = 0;
      const char* next = cyr::utf8Next(p, cp);
      if (next > end) next = end;
      const int idx = cyr::glyphIndex(cp);
      const int x = display.getCursorX();
      const int baseline = display.getCursorY();
      drawCyrGlyph(idx, x, baseline);
      const int adv = idx >= 0 ? (int)advanceFor(cyrFace(), idx)
                               : (cyrFace().height * 2) / 3;
      display.setCursor(x + adv, baseline);
      p = next;
    }
  }
}

void GxEPDDisplay::print(const char* str) {
#if CYRILLIC_SUPPORT
  if (cyr::hasNonAscii(str)) {
    printSpan(str, str + strlen(str));
    return;
  }
#endif
  display_crc.update<char>(str, strlen(str));
  display.print(str);
}

void GxEPDDisplay::printWordWrap(const char* str, int max_width) {
#if CYRILLIC_SUPPORT
  if (cyr::hasNonAscii(str)) {
    display_crc.update<char>(str, strlen(str));
    display_crc.update<int>(max_width);
    const int origin = _lx;
    int y = _ly;
    const int origin_px = (int)((origin + offset_x) * scale_x);
    int limit = display.width() - origin_px;
    if (limit < 1) limit = 1;
    int max_px = (int)(max_width * scale_x + 0.5f);
    if (max_px > limit) max_px = limit;
    if (max_px < 1) max_px = 1;
    const int step = textLineStep();
    int descent = (int)cyrFace().height - (int)cyrFace().baseline;
    if (descent < 0) descent = 0;
    display.setTextWrap(false);

    bool first = true;
    const char* p = str;
    while (*p) {
      const int baseline = (int)((y + offset_y) * scale_y);
      if (!first && (baseline < 0 || baseline + descent >= display.height())) break;
      const char* line = p;
      int width = 0;
      const char* brk = nullptr;
      const char* end = p;
      while (*end && *end != '\n') {
        uint32_t cp = 0;
        const char* next = cyr::utf8Next(end, cp);
        const int cw = codepointWidthPx(cp);
        if (width + cw > max_px && end != line) break;
        width += cw;
        if (cp == ' ' || cp == '-' || cp == '/') brk = next;
        end = next;
      }
      const char* cut = end;
      if (*end && *end != '\n' && brk && brk > line) cut = brk;
      setCursor(origin, y);
      printSpan(line, cut);
      first = false;
      if (*cut == '\n' || *cut == ' ') cut++;
      if (cut == p) break;
      p = cut;
      if (*p) y += step;
    }
    display.setTextWrap(true);
    _lx = origin;
    _ly = y;
    return;
  }
#endif
  print(str);
}

void GxEPDDisplay::translateUTF8ToBlocks(char* dest, const char* src, size_t dest_size) {
#if CYRILLIC_SUPPORT
  cyr::translateUtf8KeepCyrillic(dest, src, dest_size);
#else
  DisplayDriver::translateUTF8ToBlocks(dest, src, dest_size);
#endif
}

void GxEPDDisplay::fillRect(int x, int y, int w, int h) {
  display_crc.update<int>(x);
  display_crc.update<int>(y);
  display_crc.update<int>(w);
  display_crc.update<int>(h);
  display.fillRect(x*scale_x, y*scale_y, w*scale_x, h*scale_y, _curr_color);
}

void GxEPDDisplay::drawRect(int x, int y, int w, int h) {
  display_crc.update<int>(x);
  display_crc.update<int>(y);
  display_crc.update<int>(w);
  display_crc.update<int>(h);
  display.drawRect(x*scale_x, y*scale_y, w*scale_x, h*scale_y, _curr_color);
}

void GxEPDDisplay::drawXbm(int x, int y, const uint8_t* bits, int w, int h) {
  display_crc.update<int>(x);
  display_crc.update<int>(y);
  display_crc.update<int>(w);
  display_crc.update<int>(h);
  display_crc.update<uint8_t>(bits, w * h / 8);
  // Calculate the base position in display coordinates
  uint16_t startX = x * scale_x;
  uint16_t startY = y * scale_y;
  
  // Width in bytes for bitmap processing
  uint16_t widthInBytes = (w + 7) / 8;
  
  // Process the bitmap row by row
  for (uint16_t by = 0; by < h; by++) {
    // Calculate the target y-coordinates for this logical row
    int y1 = startY + (int)(by * scale_y);
    int y2 = startY + (int)((by + 1) * scale_y);
    int block_h = y2 - y1;
    
    // Scan across the row bit by bit
    for (uint16_t bx = 0; bx < w; bx++) {
      // Calculate the target x-coordinates for this logical column
      int x1 = startX + (int)(bx * scale_x);
      int x2 = startX + (int)((bx + 1) * scale_x);
      int block_w = x2 - x1;
      
      // Get the current bit
      uint16_t byteOffset = (by * widthInBytes) + (bx / 8);
      uint8_t bitMask = 0x80 >> (bx & 7);
      bool bitSet = pgm_read_byte(bits + byteOffset) & bitMask;
      
      // If the bit is set, draw a block of pixels
      if (bitSet) {
        // Draw the block as a filled rectangle
        display.fillRect(x1, y1, block_w, block_h, _curr_color);
      }
    }
  }
}

uint16_t GxEPDDisplay::getTextWidth(const char* str) {
#if CYRILLIC_SUPPORT
  if (cyr::hasNonAscii(str)) {
    int px = 0;
    const char* p = str;
    while (*p) {
      uint32_t cp = 0;
      p = cyr::utf8Next(p, cp);
      px += codepointWidthPx(cp);
    }
    return (uint16_t)ceil((px + 1) / scale_x);
  }
#endif
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
  return ceil((w + 1) / scale_x);
}

void GxEPDDisplay::endFrame() {
  if (_isOn == false) return;
  uint32_t crc = display_crc.finalize();
  if (crc != last_display_crc_value) {
    display.display(true);
    if (_cycles_before_full_refresh > 0) {
      _cycles_before_full_refresh--;
    }
  }
  last_display_crc_value = crc;
}
