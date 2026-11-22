#include <gtest/gtest.h>

#include <cstring>
#include <string>

#include <helpers/UTF8Helpers.h>
#include <helpers/ui/DisplayDriver.h>
#include <helpers/ui/cyrillic/CyrillicFont.h>
#include <helpers/ui/cyrillic/CyrillicGlyphs.h>

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
