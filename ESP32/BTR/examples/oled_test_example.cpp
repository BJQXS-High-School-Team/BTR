#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// SSD1306 OLED test example (ESP32-S3, 128x64, I2C)
// Pin connections:
//   OLED VCC -> 3.3V
//   OLED GND -> GND
//   OLED SDA -> GPIO8
//   OLED SCL -> GPIO9
//   OLED RES -> (optional, not used in this example)
//
// If your OLED address is not 0x3C, change kOledAddress to 0x3D.

constexpr uint8_t kI2cSdaPin = 8;
constexpr uint8_t kI2cSclPin = 9;
constexpr uint8_t kOledAddress = 0x3C;
constexpr int kScreenWidth = 128;
constexpr int kScreenHeight = 64;
constexpr int kOledReset = -1; // no reset pin
constexpr uint32_t kBaudRate = 115200;

Adafruit_SSD1306 display(kScreenWidth, kScreenHeight, &Wire, kOledReset);

void DrawSplash() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("OLED Test");
  display.println("SSD1306 128x64");
  display.println("SDA=GPIO8");
  display.println("SCL=GPIO9");
  display.drawRect(0, 50, 128, 14, SSD1306_WHITE);
  display.fillRect(4, 54, 40, 6, SSD1306_WHITE);
  display.display();
}

void DrawCounter(uint32_t seconds) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("OLED Running...");
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print("t=");
  display.print(seconds);
  display.println("s");

  int x = seconds % kScreenWidth;
  display.drawLine(0, 63, x, 63, SSD1306_WHITE);
  display.display();
}

void setup() {
  Serial.begin(kBaudRate);
  Wire.begin(kI2cSdaPin, kI2cSclPin);

  Serial.println("OLED test example");
  Serial.println("Pins: VCC->3.3V, GND->GND, SDA->GPIO8, SCL->GPIO9");

  if (!display.begin(SSD1306_SWITCHCAPVCC, kOledAddress)) {
    Serial.println("SSD1306 init failed. Check wiring/address (0x3C/0x3D).");
    while (true) {
      delay(1000);
    }
  }

  DrawSplash();
  delay(1500);
}

void loop() {
  static uint32_t startMs = millis();
  uint32_t elapsedSec = (millis() - startMs) / 1000;
  DrawCounter(elapsedSec);
  delay(200);
}
