#include <Arduino.h>

// MAX471 current sensor example (ESP32-S3)
// Pin connections:
//   MAX471 VCC  -> 5V
//   MAX471 GND  -> GND
//   MAX471 OUT  -> GPIO4 (ADC1_CH3), use a voltage divider if OUT can exceed 3.3V
// Notes:
// - Many MAX471 modules are powered by 5V.
// - ESP32 ADC pin must stay within 0-3.3V.
// - kDividerRatio and kVoltPerAmp must be calibrated for your module.

constexpr int kMax471OutPin = 4;   // ADC input pin
constexpr uint32_t kBaudRate = 115200;
constexpr float kAdcMax = 4095.0f; // 12-bit ADC
constexpr float kAdcVref = 3.3f;   // ESP32 ADC reference

// If using a divider (for example 100k:100k), ratio is 2.0.
constexpr float kDividerRatio = 1.0f;

// Typical MAX471 current output scale is module-dependent.
// Start with 1.0 V/A and calibrate with a known load.
constexpr float kVoltPerAmp = 1.0f;

void setup() {
  Serial.begin(kBaudRate);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // full-scale close to 3.3V

  Serial.println("MAX471 current sensor example");
  Serial.println("Pins: VCC->5V, GND->GND, OUT->GPIO4");
}

void loop() {
  int raw = analogRead(kMax471OutPin);
  float adcVoltage = (raw / kAdcMax) * kAdcVref;
  float sensorOutVoltage = adcVoltage * kDividerRatio;
  float currentA = sensorOutVoltage / kVoltPerAmp;

  Serial.print("ADC: ");
  Serial.print(raw);
  Serial.print("\tADC Voltage: ");
  Serial.print(adcVoltage, 3);
  Serial.print(" V\tSensor OUT: ");
  Serial.print(sensorOutVoltage, 3);
  Serial.print(" V\tCurrent: ");
  Serial.print(currentA, 3);
  Serial.println(" A");

  delay(1000);
}
