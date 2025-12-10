#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

void setup() {
  u8g2.begin();
}

void loop() {
  u8g2.clearBuffer();

  // Circle (face)
  u8g2.drawCircle(64, 32, 20);

  // Eyes
  u8g2.drawDisc(56, 26, 3);
  u8g2.drawDisc(72, 26, 3);

  // Smile using correct 5-argument drawArc()
  // radius = 12, start=200°, end=340° for smile shape
  u8g2.drawArc(64, 36, 12, 200, 340);

  u8g2.sendBuffer();
  delay(500);
}

