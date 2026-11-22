#include <gtest/gtest.h>

#include <cstring>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <helpers/UTF8Helpers.h>
#include <helpers/ui/DisplayDriver.h>
#include <helpers/ui/cyrillic/CyrillicFont.h>
#include <helpers/ui/cyrillic/CyrillicGlyphs.h>
#include <helpers/ui/cyrillic/CyrText.h>

TEST(CyrillicFont, DecodesAsciiCyrillicAndEmoji) {
  uint32_t cp = 0;
  const char* p = "A\xD0\x90\xD1\x91\xF0\x9F\x99\x82";
  p = cyr::utf8Next(p, cp);
  EXPECT_EQ('A', cp);
  p = cyr::utf8Next(p, cp);
  EXPECT_EQ(0x0410u, cp);
  p = cyr::utf8Next(p, cp);
  EXPECT_EQ(0x0451u, cp);
  p = cyr::utf8Next(p, cp);
  EXPECT_EQ(0x1F642u, cp);
  EXPECT_EQ('\0', *p);
}

TEST(CyrillicFont, TruncatedSequenceDoesNotRunPastNul) {
  uint32_t cp = 0;
  const char two_byte[] = {(char)0xD0, 0};
  const char* p = cyr::utf8Next(two_byte, cp);
  EXPECT_EQ((uint32_t)(unsigned char)0xD0, cp);
  EXPECT_EQ(1, p - two_byte);

  const char three_byte[] = {(char)0xE0, (char)0x80, 0};
  p = cyr::utf8Next(three_byte, cp);
  EXPECT_EQ((uint32_t)(unsigned char)0xE0, cp);
  EXPECT_EQ(1, p - three_byte);

  const char four_byte[] = {(char)0xF0, (char)0x9F, (char)0x99, 0};
  p = cyr::utf8Next(four_byte, cp);
  EXPECT_EQ((uint32_t)(unsigned char)0xF0, cp);
  EXPECT_EQ(1, p - four_byte);

  const char lone_continuation[] = {(char)0x80, 0};
  p = cyr::utf8Next(lone_continuation, cp);
  EXPECT_EQ((uint32_t)0x80, cp);
  EXPECT_EQ(1, p - lone_continuation);
}

TEST(CyrillicFont, MapsRussianLetters) {
  EXPECT_EQ(0, cyr::glyphIndex(0x0410));
  EXPECT_EQ(31, cyr::glyphIndex(0x042F));
  EXPECT_EQ(32, cyr::glyphIndex(0x0401));
  EXPECT_EQ(33, cyr::glyphIndex(0x0430));
  EXPECT_EQ(64, cyr::glyphIndex(0x044F));
  EXPECT_EQ(65, cyr::glyphIndex(0x0451));
  EXPECT_EQ(-1, cyr::glyphIndex('A'));
}

TEST(CyrillicFont, KeepsAsciiAndCyrillicDropsEmoji) {
  char dest[64];
  cyr::translateUtf8KeepCyrillic(dest,
      "Hi \xD0\x9F\xD1\x80\xD0\xB8 \xF0\x9F\x99\x82!", sizeof(dest));
  EXPECT_STREQ("Hi \xD0\x9F\xD1\x80\xD0\xB8 !", dest);
}

TEST(CyrillicFont, TruncatesOnCodePointBoundary) {
  char dest[5];
  cyr::translateUtf8KeepCyrillic(dest, "\xD0\x9F\xD1\x80\xD0\xB8\xD0\xB2", sizeof(dest));
  EXPECT_STREQ("\xD0\x9F\xD1\x80", dest);
}

TEST(CyrillicFont, ZeroSizeLeavesBufferUntouched) {
  char dest[2] = {'X', 0};
  cyr::translateUtf8KeepCyrillic(dest, "\xD0\x9F", 0);
  EXPECT_EQ('X', dest[0]);
}

TEST(CyrillicFont, SizeOneWritesEmptyString) {
  char dest[1] = {'X'};
  cyr::translateUtf8KeepCyrillic(dest, "\xD0\x9F", 1);
  EXPECT_EQ(0, dest[0]);
}

TEST(CyrillicFont, ExactFitCopiesWholeString) {
  char dest[7];
  cyr::translateUtf8KeepCyrillic(dest, "\xD0\x9F\xD1\x80\xD0\xB8", sizeof(dest));
  EXPECT_STREQ("\xD0\x9F\xD1\x80\xD0\xB8", dest);
}

TEST(CyrillicGlyphs, OledFaceHas66Glyphs) {
  EXPECT_EQ(10, CYR_OLED.height);
  EXPECT_EQ(6, advanceFor(CYR_OLED, 0));
  for (int i = 0; i < 66; i++) {
    EXPECT_GE(advanceFor(CYR_OLED, i), 1);
    EXPECT_LE(advanceFor(CYR_OLED, i), 16);
  }
}

TEST(CyrillicGlyphs, PixelAccessIsBounded) {
  for (int i = 0; i < 66; i++) {
    EXPECT_FALSE(pixelFor(CYR_OLED, i, advanceFor(CYR_OLED, i), 0));
    EXPECT_FALSE(pixelFor(CYR_OLED, i, 0, CYR_OLED.height));
    EXPECT_FALSE(pixelFor(CYR_OLED, i, -1, 0));
  }
  bool any = false;
  for (int y = 0; y < CYR_OLED.height && !any; y++) {
    for (int x = 0; x < advanceFor(CYR_OLED, 0); x++) {
      any = any || pixelFor(CYR_OLED, 0, x, y);
    }
  }
  EXPECT_TRUE(any) << "glyph A must not be empty";
}


class FakeDisplay : public DisplayDriver {
 public:
  std::string out;
  int charW = 1;

  FakeDisplay() : DisplayDriver(128, 64) {}

  bool isOn() override { return true; }
  void turnOn() override {}
  void turnOff() override {}
  void clear() override {}
  void startFrame(ColorVal) override {}
  void setTextSize(int) override {}
  void setColor(ColorVal) override {}
  void setCursor(int, int) override {}
  void print(const char* s) override { out += s; }
  void fillRect(int, int, int, int) override {}
  void drawRect(int, int, int, int) override {}
  void drawXbm(int, int, const uint8_t*, int, int) override {}
  uint16_t getTextWidth(const char* s) override { return strlen(s) * charW; }
  void endFrame() override {}
};

TEST(DisplayText, EllipsisDoesNotSplitCyrillic) {
  FakeDisplay d;
  d.drawTextEllipsized(0, 0, 9, "\xD0\x9F\xD1\x80\xD0\xB8\xD0\xB2\xD0\xB5\xD1\x82");
  EXPECT_EQ("\xD0\x9F\xD1\x80\xD0\xB8...", d.out);
  EXPECT_EQ(d.out.size(), mesh::validUtf8PrefixLength(d.out.c_str(), d.out.size()));
}

TEST(DisplayText, EllipsisLeavesFittingTextAlone) {
  FakeDisplay d;
  d.drawTextEllipsized(0, 0, 12, "\xD0\x9F\xD1\x80\xD0\xB8\xD0\xB2\xD0\xB5\xD1\x82");
  EXPECT_EQ("\xD0\x9F\xD1\x80\xD0\xB8\xD0\xB2\xD0\xB5\xD1\x82", d.out);
}

TEST(DisplayText, EllipsisStillWorksForAscii) {
  FakeDisplay d;
  d.drawTextEllipsized(0, 0, 4, "Hello");
  EXPECT_EQ("H...", d.out);
}

TEST(DisplayText, EllipsisAtEightKeepsWholeCodePoints) {
  FakeDisplay d;
  d.drawTextEllipsized(0, 0, 8, "\xD0\x9F\xD1\x80\xD0\xB8\xD0\xB2\xD0\xB5\xD1\x82");
  EXPECT_EQ("\xD0\x9F\xD1\x80...", d.out);
  EXPECT_EQ(d.out.size(), mesh::validUtf8PrefixLength(d.out.c_str(), d.out.size()));
}

TEST(DisplayText, EllipsisAtTenKeepsWholeCodePoints) {
  FakeDisplay d;
  d.drawTextEllipsized(0, 0, 10, "\xD0\x9F\xD1\x80\xD0\xB8\xD0\xB2\xD0\xB5\xD1\x82");
  EXPECT_EQ("\xD0\x9F\xD1\x80\xD0\xB8...", d.out);
  EXPECT_EQ(d.out.size(), mesh::validUtf8PrefixLength(d.out.c_str(), d.out.size()));
}

TEST(DisplayText, EllipsisWithEmojiKeepsValidUtf8) {
  FakeDisplay d;
  d.drawTextEllipsized(0, 0, 8, "A\xF0\x9F\x99\x82" "BCDEF");
  EXPECT_EQ("A\xF0\x9F\x99\x82...", d.out);
  EXPECT_EQ(d.out.size(), mesh::validUtf8PrefixLength(d.out.c_str(), d.out.size()));
}


struct FakeGfx {
  int x = 0, y = 0;
  std::string out;
  std::vector<std::pair<int, int>> pixels;
  std::vector<std::pair<int, int>> cursors;
  int textSize = 1;

  int16_t getCursorX() const { return x; }
  int16_t getCursorY() const { return y; }
  void setCursor(int nx, int ny) {
    x = nx;
    y = ny;
    cursors.emplace_back(nx, ny);
  }
  void print(const char* s) {
    out += s;
    x += 6 * textSize * (int)strlen(s);
  }
  void drawPixel(int px, int py, uint16_t) { pixels.emplace_back(px, py); }
};

TEST(CyrText, AsciiOnlyIsNotHandled) {
  FakeGfx g;
  if (!cyr::print(g, 1, "Hello", 1)) g.print("Hello");
  EXPECT_EQ("Hello", g.out);
  EXPECT_TRUE(g.pixels.empty());
}

TEST(CyrText, MixedStringDrawsBothParts) {
  FakeGfx g;
  EXPECT_TRUE(cyr::print(g, 1, "Hi \xD0\x9F\xD1\x80\xD0\xB8", 1));
  EXPECT_EQ("Hi ", g.out);
  EXPECT_FALSE(g.pixels.empty());
  EXPECT_EQ(36, g.x);  // "Hi " = 18, "При" = 18
}

TEST(CyrText, WidthCountsUtf8CodePoints) {
  FakeGfx g;
  EXPECT_EQ(36, cyr::width(g, 1, "Hi \xD0\x9F\xD1\x80\xD0\xB8"));
  EXPECT_EQ(72, cyr::width(g, 2, "Hi \xD0\x9F\xD1\x80\xD0\xB8"));
}

TEST(CyrText, EmojiBecomesSingleSpaceAdvance) {
  FakeGfx g;
  EXPECT_TRUE(cyr::print(g, 1, "A\xF0\x9F\x99\x82" "B", 1));
  EXPECT_EQ("AB", g.out);
  EXPECT_TRUE(g.pixels.empty());
  EXPECT_EQ(18, g.x);
}

TEST(CyrText, WordWrapBreaksAtSpacesAndScales) {
  FakeGfx g;
  EXPECT_TRUE(cyr::wordWrap(g, 1, "Hi \xD0\x9F\xD1\x80\xD0\xB8\xD0\xB2\xD0\xB5\xD1\x82", 24, 64, 1));
  std::set<int> line_ys;
  for (auto& c : g.cursors) {
    if (c.first == 0) line_ys.insert(c.second);
  }
  EXPECT_EQ(3u, line_ys.size());
  EXPECT_EQ(22, *line_ys.rbegin());
}

TEST(CyrText, WordWrapStopsAboveBottom) {
  FakeGfx g;
  EXPECT_TRUE(cyr::wordWrap(g, 1, "Hi \xD0\x9F\xD1\x80\xD0\xB8\xD0\xB2\xD0\xB5\xD1\x82", 24, 30, 1));
  std::set<int> line_ys;
  for (auto& c : g.cursors) {
    if (c.first == 0) line_ys.insert(c.second);
  }
  EXPECT_EQ(2u, line_ys.size());
}


TEST(CyrText, AsciiRunRespectsDeviceTextSize) {
  FakeGfx g;
  g.textSize = 2;
  EXPECT_TRUE(cyr::print(g, 2, "A\xD0\x91", 1));
  EXPECT_EQ("A", g.out);
  EXPECT_EQ(24, g.x);  // A = 12 physical px, Б = 12 physical px
  EXPECT_FALSE(g.pixels.empty());
}

TEST(CyrText, WordWrapLongUnbrokenWordStaysInBounds) {
  FakeGfx g;
  EXPECT_TRUE(cyr::wordWrap(g, 1, "\xD0\x9F\xD1\x80\xD0\xB8\xD0\xB2\xD0\xB5\xD1\x82\xD1\x81\xD1\x82\xD0\xB2\xD0\xB8\xD0\xB5", 24, 64, 1));
  ASSERT_FALSE(g.pixels.empty());
  for (auto& px : g.pixels) {
    EXPECT_LT(px.first, 24);
    EXPECT_LT(px.second, 64);
  }
  std::set<int> line_ys;
  for (auto& c : g.cursors) {
    if (c.first == 0) line_ys.insert(c.second);
  }
  EXPECT_EQ(3u, line_ys.size());
}

TEST(CyrText, WordWrapAtSizeTwoScalesAndFits) {
  FakeGfx g;
  EXPECT_TRUE(cyr::wordWrap(g, 2, "\xD0\x9F\xD1\x80\xD0\xB8\xD0\xB2\xD0\xB5\xD1\x82", 48, 64, 1));
  ASSERT_FALSE(g.pixels.empty());
  for (auto& px : g.pixels) {
    EXPECT_LT(px.first, 48);
    EXPECT_LT(px.second, 64);
  }
  std::set<int> line_ys;
  for (auto& c : g.cursors) {
    if (c.first == 0) line_ys.insert(c.second);
  }
  EXPECT_EQ(2u, line_ys.size());
}

TEST(CyrText, NewlineMovesToNextLine) {
  FakeGfx g;
  EXPECT_TRUE(cyr::print(g, 1, "A\n\xD0\x91", 1));
  EXPECT_EQ("A", g.out);
  ASSERT_FALSE(g.pixels.empty());
  for (auto& px : g.pixels) {
    EXPECT_GE(px.second, 8);
    EXPECT_LT(px.second, 18);
  }
}

TEST(CyrText, EmojiInsideWrapTakesOneSpace) {
  FakeGfx g;
  EXPECT_TRUE(cyr::wordWrap(g, 1, "A\xF0\x9F\x99\x82 B", 128, 64, 1));
  EXPECT_EQ("A B", g.out);
  EXPECT_EQ(24, g.x);
}

TEST(CyrillicGlyphs, EinkFacesAreWellFormed) {
  EXPECT_EQ(21, CYR_EINK.height);
  EXPECT_EQ(29, CYR_EINK2.height);
  for (int i = 0; i < 66; i++) {
    EXPECT_GE(advanceFor(CYR_EINK, i), 1);
    EXPECT_GE(advanceFor(CYR_EINK2, i), 1);
  }
  bool any = false;
  for (int y = 0; y < CYR_EINK.height && !any; y++) {
    for (int x = 0; x < advanceFor(CYR_EINK, 0); x++) {
      any = any || pixelFor(CYR_EINK, 0, x, y);
    }
  }
  EXPECT_TRUE(any) << "e-ink glyph A must not be empty";
}


TEST(CyrEinkAdapter, GlyphTopHasNoExtraCapShift) {
  EXPECT_EQ(10 - (int)CYR_EINK.baseline, cyr::glyphTop(CYR_EINK, 10));
  EXPECT_EQ(10 - (int)CYR_EINK2.baseline, cyr::glyphTop(CYR_EINK2, 10));
}

TEST(CyrillicGlyphs, EinkOffsetsAreContiguous) {
  for (int i = 0; i < 65; i++) {
    const int row_bytes = (advanceFor(CYR_EINK, i) + 7) / 8;
    EXPECT_EQ(CYR_EINK_OFF[i] + CYR_EINK.height * row_bytes, CYR_EINK_OFF[i + 1]);
    const int row_bytes2 = (advanceFor(CYR_EINK2, i) + 7) / 8;
    EXPECT_EQ(CYR_EINK2_OFF[i] + CYR_EINK2.height * row_bytes2, CYR_EINK2_OFF[i + 1]);
  }
  const int last = 65;
  EXPECT_EQ(sizeof(CYR_EINK_BITS),
            (size_t)(CYR_EINK_OFF[last] + CYR_EINK.height * ((advanceFor(CYR_EINK, last) + 7) / 8)));
  EXPECT_EQ(sizeof(CYR_EINK2_BITS),
            (size_t)(CYR_EINK2_OFF[last] + CYR_EINK2.height * ((advanceFor(CYR_EINK2, last) + 7) / 8)));
}

TEST(CyrillicGlyphs, TftFacesAreWellFormedAndContiguous) {
  EXPECT_EQ(19, CYR_TFT.height);
  EXPECT_EQ(28, CYR_TFT2.height);
  for (int i = 0; i < 66; i++) {
    EXPECT_GE(advanceFor(CYR_TFT, i), 1);
    EXPECT_GE(advanceFor(CYR_TFT2, i), 1);
  }
  for (int i = 0; i < 65; i++) {
    const int rowb = (advanceFor(CYR_TFT, i) + 7) / 8;
    EXPECT_EQ(CYR_TFT_OFF[i] + CYR_TFT.height * rowb, CYR_TFT_OFF[i + 1]);
    const int rowb2 = (advanceFor(CYR_TFT2, i) + 7) / 8;
    EXPECT_EQ(CYR_TFT2_OFF[i] + CYR_TFT2.height * rowb2, CYR_TFT2_OFF[i + 1]);
  }
  EXPECT_EQ(sizeof(CYR_TFT_BITS),
            (size_t)(CYR_TFT_OFF[65] + CYR_TFT.height * ((advanceFor(CYR_TFT, 65) + 7) / 8)));
  EXPECT_EQ(sizeof(CYR_TFT2_BITS),
            (size_t)(CYR_TFT2_OFF[65] + CYR_TFT2.height * ((advanceFor(CYR_TFT2, 65) + 7) / 8)));
}
