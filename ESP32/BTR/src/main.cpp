#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>
#include <SensirionI2CSht4x.h>
#include <SensirionI2CSgp41.h>
#include <MAX30105.h>
#include <DNSServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <Wire.h>

namespace {
// ====== 引脚定义（请根据实际硬件连线调整） ======
// 所有引脚在此集中管理，避免分散定义导致冲突或遗漏。
// 实际接线可能因开发板不同而变化，请以硬件原理图为准。
constexpr uint8_t kButtonPin = 17;          // 按键，高电平为按下
constexpr uint8_t kBuzzerPin = 18;          // 有源蜂鸣器
constexpr uint8_t kNeoPixelPin = 48;        // WS2812 RGB 灯
constexpr uint8_t kNeoPixelCount = 1;

// SPI for MAX31865 (RTD)
// MAX31865 通过 SPI 与 ESP32S3 通信，读取 PT100/ PT1000 电阻并换算温度。
constexpr uint8_t kMax31865CsPin = 10;
constexpr uint8_t kMax31865MosiPin = 11;
constexpr uint8_t kMax31865MisoPin = 13;
constexpr uint8_t kMax31865SckPin = 12;

// MQ 气体传感器模拟输入引脚
// MQ 传感器输出模拟电压，这里使用 ADC 读取后做简化线性换算。
constexpr uint8_t kMq2Pin = 1;
constexpr uint8_t kMq4Pin = 2;
constexpr uint8_t kMq8Pin = 3;
constexpr uint8_t kMq7Pin = 4;

// OLED
// SSD1306 128x64 OLED，I2C 地址默认 0x3C。
constexpr uint8_t kScreenWidth = 128;
constexpr uint8_t kScreenHeight = 64;
constexpr uint8_t kOledReset = 255; // no reset pin

// ====== 采样与记录 ======
// 2Hz 采样与 10 秒历史缓冲，满足“每秒 2 次采样、记录 10 秒”的要求。
constexpr uint32_t kSampleIntervalMs = 500; // 2Hz
constexpr uint8_t kHistoryLength = 20;      // 10s * 2Hz
constexpr uint32_t kWifiRetryIntervalMs = 10000;
constexpr uint32_t kApOnlineAutoCloseMs = 5UL * 60UL * 1000UL;

constexpr char kWifiConfigPath[] = "/wifi.txt";
constexpr char kWifiPrevConfigPath[] = "/wifi_prev.txt";
constexpr char kThresholdConfigPath[] = "/thresholds.txt";

// ====== MQTT 配置 ======
// MQTT 服务器：bemfa.com:9501，发布 topic 为 sensor，订阅 topic 为 statue。
constexpr char kMqttHost[] = "bemfa.com";
constexpr uint16_t kMqttPort = 9501;
constexpr char kMqttPrivateKey[] = "84810b9b5f5245fdbc1e1738837f27a9";
constexpr char kMqttPubTopic[] = "sensor";
constexpr char kConfigApSsid[] = "BTR-Config";
constexpr char kConfigApPassword[] = "12345678";
constexpr uint16_t kMqttBufferSize = 1024;
const IPAddress kConfigApIp(192, 168, 4, 1);
const IPAddress kConfigApGateway(192, 168, 4, 1);
const IPAddress kConfigApSubnet(255, 255, 255, 0);
constexpr char kMqttSubTopic[] = "statue"; // 按需求拼写

// ====== 阈值（常量） ======
// 阈值使用常量定义，便于统一修改。
struct ThresholdConfig {
  float tempAmbientWarnC = 50.0f;
  float tempInternalWarnC = 50.0f;
  float humidityWarnPercent = 80.0f;
  float mq2WarnPpm = 200.0f;
  float mq4WarnPpm = 250.0f;
  float mq8WarnPpm = 450.0f;
  float mq7WarnPpm = 750.0f;
  float vocIndexWarn = 200.0f;
  uint32_t max30105SmokeWarn = 750;
  float max30105TempWarnC = 50.0f;
};

// MAX30105 阈值：烟雾、温度

// MQ 传感器转换比例（简化线性换算）
// 未做标定曲线时，先用 0~4095 ADC 映射到 0~maxPpm。
constexpr float kMq2MaxPpm = 1000.0f;
constexpr float kMq4MaxPpm = 1000.0f;
constexpr float kMq8MaxPpm = 1000.0f;
constexpr float kMq7MaxPpm = 1000.0f;

Adafruit_SSD1306 display(kScreenWidth, kScreenHeight, &Wire, kOledReset);
Adafruit_NeoPixel pixels(kNeoPixelCount, kNeoPixelPin, NEO_GRB + NEO_KHZ800);
Adafruit_MAX31865 rtd(kMax31865CsPin, kMax31865MosiPin, kMax31865MisoPin, kMax31865SckPin);
SensirionI2cSht4x sht4x;
SensirionI2CSgp41 sgp41;
MAX30105 max30105;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer webServer(80);
DNSServer dnsServer;
ThresholdConfig thresholds;

// 传感器历史记录结构，用于保存 10 秒内的采样快照与时间戳。
struct SensorSample {
  uint32_t timestampMs;
  float tempAmbientC;
  float tempInternalC;
  float humidityPercent;
  float mq2Ppm;
  float mq4Ppm;
  float mq8Ppm;
  float mq7Ppm;
  float vocIndex;
  uint32_t max30105Smoke;    // 烟雾浓度（IR）
  float max30105TempC;       // MAX30105温度
};

SensorSample history[kHistoryLength] = {};
uint8_t historyIndex = 0;
uint8_t historyCount = 0;

uint32_t lastSampleMs = 0;
uint32_t lastMqttMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastBuzzerToggleMs = 0;
uint32_t lastWifiRetryMs = 0;
uint32_t apOnlineStartMs = 0;
uint8_t wifiConnectFailCount = 0;

uint8_t screenIndex = 0;
bool buttonLatched = false;
bool webServerStarted = false;
bool configPortalActive = false;

enum class StatusLevel { kNormal, kWarning, kDanger };
StatusLevel localStatus = StatusLevel::kNormal;
StatusLevel effectiveStatus = StatusLevel::kNormal;
bool overrideActive = false;
StatusLevel overrideStatus = StatusLevel::kNormal;
bool cloudDangerLatched = false;
bool cloudNormalPending = false;
StatusLevel lastCloudStatus = StatusLevel::kNormal;

String BuildThresholdJson();

float lastTempAmbientC = 0.0f;
float lastTempInternalC = 0.0f;
float lastHumidityPercent = 0.0f;
float lastMq2Ppm = 0.0f;
float lastMq4Ppm = 0.0f;
float lastMq8Ppm = 0.0f;
float lastMq7Ppm = 0.0f;
float lastVocIndex = 0.0f;

// MAX30105 传感器数据：烟雾（IR）、温度
uint32_t lastMax30105Smoke = 0;      // 烟雾浓度（红外值）
float lastMax30105TempC = 0.0f;      // MAX30105温度

// MQ 传感器 ADC 读数转换为 ppm（简化线性模型）。
float ConvertMqToPpm(int adc, float maxPpm) {
  float ratio = static_cast<float>(adc) / 4095.0f;
  return ratio * maxPpm;
}

// 统计超过阈值的传感器数量，以划分正常/可疑/紧急三种情况。
StatusLevel EvaluateStatus() {
  int exceedCount = 0;
  if (lastTempAmbientC > thresholds.tempAmbientWarnC) {
    exceedCount++;
  }
  if (lastTempInternalC > thresholds.tempInternalWarnC) {
    exceedCount++;
  }
  if (lastHumidityPercent > thresholds.humidityWarnPercent) {
    exceedCount++;
  }
  if (lastMq2Ppm > thresholds.mq2WarnPpm) {
    exceedCount++;
  }
  if (lastMq4Ppm > thresholds.mq4WarnPpm) {
    exceedCount++;
  }
  if (lastMq8Ppm > thresholds.mq8WarnPpm) {
    exceedCount++;
  }
  if (lastMq7Ppm > thresholds.mq7WarnPpm) {
    exceedCount++;
  }
  if (lastVocIndex < thresholds.vocIndexWarn) {
    exceedCount++;
  }
  // MAX30105 检测：烟雾、温度
  if (lastMax30105Smoke > thresholds.max30105SmokeWarn) {
    exceedCount++;
  }
  if (lastMax30105TempC > thresholds.max30105TempWarnC) {
    exceedCount++;
  }

  if (exceedCount == 0) {
    return StatusLevel::kNormal;
  }
  if (exceedCount <= 2) {
    return StatusLevel::kWarning;
  }
  return StatusLevel::kDanger;
}

// OLED 不支持中文字体，因此状态文本保持 ASCII。
const char *StatusToString(StatusLevel status) {
  switch (status) {
    case StatusLevel::kNormal:
      return "NORMAL";
    case StatusLevel::kWarning:
      return "WARNING";
    case StatusLevel::kDanger:
      return "DANGER";
    default:
      return "UNKNOWN";
  }
}

String CurrentIpText() {
  if (WiFi.status() == WL_CONNECTED) {
    return WiFi.localIP().toString();
  }
  if (configPortalActive) {
    return WiFi.softAPIP().toString();
  }
  return "No IP";
}

void DrawIpFooter() {
  display.fillRect(0, 54, kScreenWidth, 10, SSD1306_BLACK);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 56);
  display.print("IP:");
  display.print(CurrentIpText());
}

// 处理本地计算状态与远程覆盖状态的优先级。
// 云端 danger 为锁存报警；云端恢复 normal 后，仍需设备按键确认解除。
// 解除后回到本地阈值判断，直到云端再次返回 danger。
void UpdateStatusLogic(uint32_t nowMs) {
  localStatus = EvaluateStatus();
  if (cloudDangerLatched) {
    effectiveStatus = StatusLevel::kDanger;
  } else if (overrideActive) {
    effectiveStatus = overrideStatus;
  } else {
    effectiveStatus = localStatus;
  }
}

// WS2812 RGB 灯显示状态：正常绿、可疑黄、紧急红。
void UpdateNeoPixel(StatusLevel status) {
  uint32_t color = 0;
  switch (status) {
    case StatusLevel::kNormal:
      color = pixels.Color(0, 200, 0);
      break;
    case StatusLevel::kWarning:
      color = pixels.Color(200, 200, 0);
      break;
    case StatusLevel::kDanger:
      color = pixels.Color(200, 0, 0);
      break;
  }
  pixels.setPixelColor(0, color);
  pixels.show();
}

// 紧急状态蜂鸣器快速鸣叫，其余状态关闭。
// 通过 100ms 翻转输出形成快速提示音。
void UpdateBuzzer(uint32_t nowMs) {
  if (effectiveStatus != StatusLevel::kDanger) {
    digitalWrite(kBuzzerPin, HIGH);
    lastBuzzerToggleMs = nowMs;
    return;
  }

  if (nowMs - lastBuzzerToggleMs >= 100) {
    lastBuzzerToggleMs = nowMs;
    digitalWrite(kBuzzerPin, !digitalRead(kBuzzerPin));
  }
}

// 将当前采样写入环形缓冲区（10 秒历史）。
void PushHistory(uint32_t nowMs) {
  SensorSample &sample = history[historyIndex];
  sample.timestampMs = nowMs;
  sample.tempAmbientC = lastTempAmbientC;
  sample.tempInternalC = lastTempInternalC;
  sample.humidityPercent = lastHumidityPercent;
  sample.mq2Ppm = lastMq2Ppm;
  sample.mq4Ppm = lastMq4Ppm;
  sample.mq8Ppm = lastMq8Ppm;
  sample.mq7Ppm = lastMq7Ppm;
  sample.vocIndex = lastVocIndex;
  sample.max30105Smoke = lastMax30105Smoke;
  sample.max30105TempC = lastMax30105TempC;

  historyIndex = (historyIndex + 1) % kHistoryLength;
  if (historyCount < kHistoryLength) {
    historyCount++;
  }
}

// OLED 界面 1：显示整体状态与是否被远程覆盖。
void DrawStatusScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Status");
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println(StatusToString(effectiveStatus));
  display.setTextSize(1);
  display.setCursor(0, 44);
  display.print("Local: ");
  display.println(StatusToString(localStatus));
  if (overrideActive) {
    display.setCursor(72, 44);
    display.print("Override");
  }
  DrawIpFooter();
  display.display();
}

// OLED 界面 2：显示环境/内部温度与湿度。
void DrawTemperatureScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Temp/Humidity");
  display.setCursor(0, 16);
  display.print("Ambient: ");
  display.print(lastTempAmbientC, 1);
  display.println("C");
  display.setCursor(0, 30);
  display.print("Internal: ");
  display.print(lastTempInternalC, 1);
  display.println("C");
  display.setCursor(0, 44);
  display.print("Humidity: ");
  display.print(lastHumidityPercent, 1);
  display.println("%");
  DrawIpFooter();
  display.display();
}

// OLED 界面 3：显示 MQ 气体浓度（ppm）。
void DrawGasScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Gas (ppm)");
  display.setCursor(0, 14);
  display.print("MQ-2  (Fuel): ");
  display.println(lastMq2Ppm, 0);
  display.setCursor(0, 28);
  display.print("MQ-4  (CH4): ");
  display.println(lastMq4Ppm, 0);
  display.setCursor(0, 42);
  display.print("MQ-8  (H2): ");
  display.println(lastMq8Ppm, 0);
  display.setCursor(68, 42);
  display.print("CO:");
  display.println(lastMq7Ppm, 0);
  DrawIpFooter();
  display.display();
}

// OLED 界面 4：显示 VOC 指数。
void DrawVocScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("VOC Index");
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print(lastVocIndex, 0);
  display.setTextSize(1);
  display.setCursor(0, 44);
  display.print("Air Quality");
  DrawIpFooter();
  display.display();
}

// OLED 界面 5：显示 MAX30105 烟雾、温度检测。
void DrawMax30105Screen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("MAX30105 Sensor");
  display.setCursor(0, 20);
  display.print("Smoke(IR): ");
  display.println(lastMax30105Smoke);
  display.setCursor(0, 40);
  display.print("Temp: ");
  display.print(lastMax30105TempC, 1);
  display.println("C");
  DrawIpFooter();
  display.display();
}


// 菜单切换动画：利用 SSD1306 硬件滚动实现页面切换反馈。
void PlayMenuSwitchAnimation() {
  display.startscrollleft(0x00, 0x0F);
  delay(180);
  display.stopscroll();
}

// 根据当前页索引刷新 OLED。
void UpdateDisplay() {
  switch (screenIndex) {
    case 0:
      DrawStatusScreen();
      break;
    case 1:
      DrawTemperatureScreen();
      break;
    case 2:
      DrawGasScreen();
      break;
    case 3:
      DrawVocScreen();
      break;
    case 4:
    default:
      DrawMax30105Screen();
      break;
  }
}

// MQTT 订阅回调：接收 normal/warning/danger 覆盖指令。
void OnMqttMessage(char *topic, byte *payload, unsigned int length) {
  String message;
  message.reserve(length);
  for (unsigned int i = 0; i < length; ++i) {
    message += static_cast<char>(payload[i]);
  }
  message.trim();

  if (String(topic) == kMqttSubTopic) {
    if (message == "normal") {
      overrideStatus = StatusLevel::kNormal;
      if (cloudDangerLatched || lastCloudStatus == StatusLevel::kDanger) {
        cloudNormalPending = true;
      }
      if (!cloudDangerLatched) {
        overrideActive = false;
      }
      lastCloudStatus = StatusLevel::kNormal;
    } else if (message == "warning") {
      overrideStatus = StatusLevel::kWarning;
      if (!cloudDangerLatched) {
        overrideActive = false;
        cloudNormalPending = false;
      }
      lastCloudStatus = StatusLevel::kWarning;
    } else if (message == "danger") {
      overrideStatus = StatusLevel::kDanger;
      overrideActive = true;
      cloudDangerLatched = true;
      cloudNormalPending = false;
      lastCloudStatus = StatusLevel::kDanger;
    }
  }
}

// 断线重连并保持订阅主题。
void EnsureMqttConnected() {
  if (mqttClient.connected()) {
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  String clientId = "esp32s3-" + String(random(0xffff), HEX);
  Serial.print("Connecting to MQTT...");
  if (mqttClient.connect(kMqttPrivateKey, "", "")) {
    mqttClient.subscribe(kMqttSubTopic);
    Serial.println(" connected");
  } else {
    Serial.println(" failed");
  }
}

// 发布传感器数据与时间戳到 MQTT。
void PublishMqtt(uint32_t nowMs) {
  if (!mqttClient.connected()) {
    return;
  }
  if (nowMs - lastMqttMs < kSampleIntervalMs) {
    return;
  }
  lastMqttMs = nowMs;

  String payload = String("{");
  payload += "\"timestamp_ms\":" + String(nowMs) + ",";
  payload += "\"temp_ambient_c\":" + String(lastTempAmbientC, 2) + ",";
  payload += "\"temp_internal_c\":" + String(lastTempInternalC, 2) + ",";
  payload += "\"humidity_percent\":" + String(lastHumidityPercent, 2) + ",";
  payload += "\"mq2_ppm\":" + String(lastMq2Ppm, 1) + ",";
  payload += "\"mq4_ppm\":" + String(lastMq4Ppm, 1) + ",";
  payload += "\"mq8_ppm\":" + String(lastMq8Ppm, 1) + ",";
  payload += "\"mq7_ppm\":" + String(lastMq7Ppm, 1) + ",";
  payload += "\"voc_index\":" + String(lastVocIndex, 1) + ",";
  payload += "\"max30105_smoke\":" + String(lastMax30105Smoke) + ",";
  payload += "\"max30105_temp_c\":" + String(lastMax30105TempC, 2) + ",";
  payload += "\"thresholds\":" + BuildThresholdJson() + ",";
  payload += "\"status\":\"" + String(StatusToString(effectiveStatus)) + "\"";
  payload += "}";

  if (!mqttClient.publish(kMqttPubTopic, payload.c_str())) {
    Serial.print("MQTT publish failed, payload bytes=");
    Serial.println(payload.length());
  }
}

// 读取所有传感器并转换为工程单位（°C、%、ppm）。
// MQ 传感器未标定时使用简化线性换算。
void ReadSensors(uint32_t nowMs) {
  float ambientTemp = 0.0f;
  float humidity = 0.0f;
  uint16_t shtError = sht4x.measureHighPrecision(ambientTemp, humidity);
  if (shtError) {
    Serial.print("SHT4x error: ");
    Serial.println(shtError);
  } else {
    lastTempAmbientC = ambientTemp;
    lastHumidityPercent = humidity;
  }

  // MAX31865 读取 PT100 温度，参考电阻 430Ω（需与硬件一致）。
  lastTempInternalC = rtd.temperature(100.0f, 430.0f);

  uint16_t vocRaw = 0;
  uint16_t noxIndex = 0;
  uint16_t sgpError = sgp41.measureRawSignals(0, 0, vocRaw, noxIndex);
  if (sgpError) {
    Serial.print("SGP41 error: ");
    Serial.println(sgpError);
  }
  // 简化转换：将原始值映射到 0-500 VOC 指数。
  lastVocIndex = map(vocRaw, 0, 65535, 0, 500);

  lastMq2Ppm = ConvertMqToPpm(analogRead(kMq2Pin), kMq2MaxPpm);
  lastMq4Ppm = ConvertMqToPpm(analogRead(kMq4Pin), kMq4MaxPpm);
  lastMq8Ppm = ConvertMqToPpm(analogRead(kMq8Pin), kMq8MaxPpm);
  lastMq7Ppm = ConvertMqToPpm(analogRead(kMq7Pin), kMq7MaxPpm);

  // MAX30105 读取：烟雾浓度（IR）、温度
  lastMax30105Smoke = max30105.getIR();      // 红外光检测烟雾粒子
  lastMax30105TempC = max30105.readTemperature();  // 读取内置温度传感器

  PushHistory(nowMs);
}

void PrintSensorData(uint32_t nowMs) {
  Serial.print("ts_ms=");
  Serial.print(nowMs);
  Serial.print(", temp_ambient_c=");
  Serial.print(lastTempAmbientC, 2);
  Serial.print(", temp_internal_c=");
  Serial.print(lastTempInternalC, 2);
  Serial.print(", humidity_percent=");
  Serial.print(lastHumidityPercent, 2);
  Serial.print(", mq2_ppm=");
  Serial.print(lastMq2Ppm, 1);
  Serial.print(", mq4_ppm=");
  Serial.print(lastMq4Ppm, 1);
  Serial.print(", mq8_ppm=");
  Serial.print(lastMq8Ppm, 1);
  Serial.print(", mq7_ppm=");
  Serial.print(lastMq7Ppm, 1);
  Serial.print(", voc_index=");
  Serial.print(lastVocIndex, 1);
  Serial.print(", max30105_smoke=");
  Serial.print(lastMax30105Smoke);
  Serial.print(", max30105_temp_c=");
  Serial.print(lastMax30105TempC, 2);
  Serial.print(", status=");
  Serial.println(StatusToString(effectiveStatus));
}

float ReadFloatLine(File &file, float fallback) {
  if (!file.available()) {
    return fallback;
  }
  String line = file.readStringUntil('\n');
  line.trim();
  return line.length() > 0 ? line.toFloat() : fallback;
}

uint32_t ReadUIntLine(File &file, uint32_t fallback) {
  if (!file.available()) {
    return fallback;
  }
  String line = file.readStringUntil('\n');
  line.trim();
  return line.length() > 0 ? static_cast<uint32_t>(line.toInt()) : fallback;
}

void LoadThresholds() {
  if (!SPIFFS.exists(kThresholdConfigPath)) {
    Serial.println("Threshold config missing, using default constants");
    return;
  }

  File file = SPIFFS.open(kThresholdConfigPath, FILE_READ);
  if (!file) {
    Serial.println("Threshold config open failed, using default constants");
    return;
  }

  thresholds.tempAmbientWarnC = ReadFloatLine(file, thresholds.tempAmbientWarnC);
  thresholds.tempInternalWarnC = ReadFloatLine(file, thresholds.tempInternalWarnC);
  thresholds.humidityWarnPercent = ReadFloatLine(file, thresholds.humidityWarnPercent);
  thresholds.mq2WarnPpm = ReadFloatLine(file, thresholds.mq2WarnPpm);
  thresholds.mq4WarnPpm = ReadFloatLine(file, thresholds.mq4WarnPpm);
  thresholds.mq8WarnPpm = ReadFloatLine(file, thresholds.mq8WarnPpm);
  thresholds.mq7WarnPpm = ReadFloatLine(file, thresholds.mq7WarnPpm);
  thresholds.vocIndexWarn = ReadFloatLine(file, thresholds.vocIndexWarn);
  thresholds.max30105SmokeWarn = ReadUIntLine(file, thresholds.max30105SmokeWarn);
  thresholds.max30105TempWarnC = ReadFloatLine(file, thresholds.max30105TempWarnC);
  file.close();
}

bool SaveThresholds() {
  File file = SPIFFS.open(kThresholdConfigPath, FILE_WRITE);
  if (!file) {
    return false;
  }
  file.println(thresholds.tempAmbientWarnC, 2);
  file.println(thresholds.tempInternalWarnC, 2);
  file.println(thresholds.humidityWarnPercent, 2);
  file.println(thresholds.mq2WarnPpm, 1);
  file.println(thresholds.mq4WarnPpm, 1);
  file.println(thresholds.mq8WarnPpm, 1);
  file.println(thresholds.mq7WarnPpm, 1);
  file.println(thresholds.vocIndexWarn, 1);
  file.println(thresholds.max30105SmokeWarn);
  file.println(thresholds.max30105TempWarnC, 2);
  file.close();
  return true;
}

bool LoadWifiConfigFrom(const char *path, String &ssid, String &password) {
  ssid = "";
  password = "";
  if (!SPIFFS.exists(path)) {
    return false;
  }

  File file = SPIFFS.open(path, FILE_READ);
  if (!file) {
    return false;
  }

  ssid = file.readStringUntil('\n');
  password = file.readStringUntil('\n');
  file.close();
  ssid.trim();
  password.trim();
  return ssid.length() > 0;
}

bool SaveWifiConfigTo(const char *path, const String &ssid, const String &password) {
  File file = SPIFFS.open(path, FILE_WRITE);
  if (!file) {
    return false;
  }
  file.println(ssid);
  file.println(password);
  file.close();
  return true;
}

bool LoadWifiConfig(String &ssid, String &password) {
  return LoadWifiConfigFrom(kWifiConfigPath, ssid, password);
}

bool SaveWifiConfig(const String &ssid, const String &password) {
  return SaveWifiConfigTo(kWifiConfigPath, ssid, password);
}

bool SavePreviousWifiConfig(const String &ssid, const String &password) {
  return SaveWifiConfigTo(kWifiPrevConfigPath, ssid, password);
}

bool RevertToPreviousWifi(String &ssid, String &password) {
  if (!LoadWifiConfigFrom(kWifiPrevConfigPath, ssid, password)) {
    return false;
  }
  return SaveWifiConfig(ssid, password);
}

String JsonString(const String &value) {
  String out = "\"";
  for (size_t i = 0; i < value.length(); ++i) {
    char c = value[i];
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  out += "\"";
  return out;
}

String BuildThresholdJson() {
  String json = "{";
  json += "\"temp_ambient_c\":" + String(thresholds.tempAmbientWarnC, 2) + ",";
  json += "\"temp_internal_c\":" + String(thresholds.tempInternalWarnC, 2) + ",";
  json += "\"humidity_percent\":" + String(thresholds.humidityWarnPercent, 2) + ",";
  json += "\"mq2_ppm\":" + String(thresholds.mq2WarnPpm, 1) + ",";
  json += "\"mq4_ppm\":" + String(thresholds.mq4WarnPpm, 1) + ",";
  json += "\"mq8_ppm\":" + String(thresholds.mq8WarnPpm, 1) + ",";
  json += "\"mq7_ppm\":" + String(thresholds.mq7WarnPpm, 1) + ",";
  json += "\"voc_index\":" + String(thresholds.vocIndexWarn, 1) + ",";
  json += "\"max30105_smoke\":" + String(thresholds.max30105SmokeWarn) + ",";
  json += "\"max30105_temp_c\":" + String(thresholds.max30105TempWarnC, 2);
  json += "}";
  return json;
}

String BuildStatusJson() {
  String json = "{";
  json += "\"timestamp_ms\":" + String(millis()) + ",";
  json += "\"temp_ambient_c\":" + String(lastTempAmbientC, 2) + ",";
  json += "\"temp_internal_c\":" + String(lastTempInternalC, 2) + ",";
  json += "\"humidity_percent\":" + String(lastHumidityPercent, 2) + ",";
  json += "\"mq2_ppm\":" + String(lastMq2Ppm, 1) + ",";
  json += "\"mq4_ppm\":" + String(lastMq4Ppm, 1) + ",";
  json += "\"mq8_ppm\":" + String(lastMq8Ppm, 1) + ",";
  json += "\"mq7_ppm\":" + String(lastMq7Ppm, 1) + ",";
  json += "\"voc_index\":" + String(lastVocIndex, 1) + ",";
  json += "\"max30105_smoke\":" + String(lastMax30105Smoke) + ",";
  json += "\"max30105_temp_c\":" + String(lastMax30105TempC, 2) + ",";
  json += "\"local_status\":\"" + String(StatusToString(localStatus)) + "\",";
  json += "\"status\":\"" + String(StatusToString(effectiveStatus)) + "\",";
  json += "\"override_active\":" + String(overrideActive ? "true" : "false") + ",";
  json += "\"cloud_danger_latched\":" + String(cloudDangerLatched ? "true" : "false") + ",";
  json += "\"cloud_normal_pending\":" + String(cloudNormalPending ? "true" : "false") + ",";
  json += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  json += "\"ap_active\":" + String(configPortalActive ? "true" : "false") + ",";
  json += "\"ap_close_allowed\":" + String((configPortalActive && WiFi.status() == WL_CONNECTED) ? "true" : "false") + ",";
  json += "\"ip\":" + JsonString(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString());
  json += "}";
  return json;
}

String BuildHistoryJson() {
  String json = "[";
  for (uint8_t i = 0; i < historyCount; ++i) {
    uint8_t idx = (historyIndex + kHistoryLength - historyCount + i) % kHistoryLength;
    const SensorSample &sample = history[idx];
    if (i > 0) {
      json += ",";
    }
    json += "{";
    json += "\"timestamp_ms\":" + String(sample.timestampMs) + ",";
    json += "\"temp_ambient_c\":" + String(sample.tempAmbientC, 2) + ",";
    json += "\"temp_internal_c\":" + String(sample.tempInternalC, 2) + ",";
    json += "\"humidity_percent\":" + String(sample.humidityPercent, 2) + ",";
    json += "\"mq2_ppm\":" + String(sample.mq2Ppm, 1) + ",";
    json += "\"mq4_ppm\":" + String(sample.mq4Ppm, 1) + ",";
    json += "\"mq8_ppm\":" + String(sample.mq8Ppm, 1) + ",";
    json += "\"mq7_ppm\":" + String(sample.mq7Ppm, 1) + ",";
    json += "\"voc_index\":" + String(sample.vocIndex, 1) + ",";
    json += "\"max30105_smoke\":" + String(sample.max30105Smoke) + ",";
    json += "\"max30105_temp_c\":" + String(sample.max30105TempC, 2);
    json += "}";
  }
  json += "]";
  return json;
}

const char kDashboardHtml[] PROGMEM = R"rawliteral(
<!doctype html><html lang="zh-CN"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>BTR Monitor</title>
<style>
:root{--bg:#f6f7f2;--ink:#18201c;--muted:#69746d;--line:#d8ded5;--panel:#fffdf7;--accent:#0f766e;--warn:#b7791f;--danger:#b91c1c}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(180deg,#eef3ec,#f7f7f1 38%,#ece8df);color:var(--ink);font-family:"Microsoft YaHei",Segoe UI,sans-serif}
main{max-width:1180px;margin:0 auto;padding:18px}.top{display:flex;justify-content:space-between;gap:16px;align-items:flex-end;margin-bottom:14px}
h1{margin:0;font-size:28px;letter-spacing:0}.sub{color:var(--muted);font-size:13px;margin-top:4px}.badge{padding:10px 14px;border:1px solid var(--line);background:var(--panel);font-weight:700;min-width:108px;text-align:center}
.NORMAL{color:#16803b}.WARNING{color:var(--warn)}.DANGER{color:var(--danger)}
.grid{display:grid;grid-template-columns:repeat(5,1fr);gap:10px}.card,.chart,.settings{background:rgba(255,253,247,.92);border:1px solid var(--line);border-radius:8px;padding:12px;box-shadow:0 10px 30px rgba(34,39,32,.06)}
.card label{display:block;color:var(--muted);font-size:12px}.card.alert{border-color:#b91c1c;background:#fff1f0;box-shadow:0 0 0 2px rgba(185,28,28,.14),0 10px 30px rgba(185,28,28,.12)}.card.alert label,.card.alert .value{color:#b91c1c}.value{font-size:24px;font-weight:800;margin-top:6px}.unit{font-size:12px;color:var(--muted);margin-left:3px}
.chart{margin-top:12px;height:330px}canvas{width:100%;height:100%}.settings{margin-top:12px}.settings h2{font-size:17px;margin:0 0 10px}
.form{display:grid;grid-template-columns:repeat(5,1fr);gap:10px}.field label{display:block;font-size:12px;color:var(--muted);margin-bottom:4px}input,select{width:100%;height:36px;border:1px solid var(--line);border-radius:6px;padding:0 8px;background:white}
button{height:38px;border:0;border-radius:6px;background:var(--accent);color:white;font-weight:700;padding:0 14px;cursor:pointer}.row{display:flex;gap:10px;align-items:center;margin-top:10px}.msg{color:var(--muted);font-size:13px}
.auto{display:grid;grid-template-columns:150px 130px auto 1fr;gap:10px;align-items:end;margin-top:10px}.auto .field{max-width:150px}
@media(max-width:900px){.grid,.form{grid-template-columns:repeat(2,1fr)}.top{align-items:flex-start;flex-direction:column}.chart{height:280px}}
</style></head><body><main>
<div class="top"><div><h1>BTR 实时监测</h1><div class="sub" id="net">等待设备数据...</div></div><div id="status" class="badge">UNKNOWN</div></div>
<section class="grid" id="cards"></section>
<section class="chart"><canvas id="chart" width="1000" height="320"></canvas></section>
<section class="settings"><h2>传感器阈值</h2><form id="thresholdForm" class="form"></form><div class="row"><button type="submit" form="thresholdForm">保存阈值</button><span class="msg" id="thresholdMsg">阈值只在打开页面和保存后读取</span></div><div class="auto"><div class="field"><label>取值方式</label><select id="autoMode"><option value="max">最大值</option><option value="avg">平均值</option></select></div><div class="field"><label>冗余量 %</label><input id="autoMargin" type="number" min="0" max="100" step="1" value="10"></div><button id="autoThresholdBtn" type="button">按最近10秒生成阈值</button><span class="msg">VOC 为下限判断，会自动向下留冗余</span></div></section>
<section class="settings"><h2>AP 配网</h2><form id="wifiForm" class="form"><div class="field"><label>WiFi SSID</label><input name="ssid" required></div><div class="field"><label>WiFi 密码</label><input name="password" type="password"></div></form><div class="row"><button type="submit" form="wifiForm">保存 WiFi</button><button id="closeApBtn" type="button">关闭 AP</button><button id="clearSpiffsBtn" type="button">清除 SPIFFS</button><span class="msg" id="wifiMsg">保存后设备会尝试连接</span></div></section>
</main><script>
const sensors=[["temp_internal_c","内部温度","°C"],["temp_ambient_c","外部温度","°C"],["humidity_percent","湿度","%"],["mq2_ppm","MQ-2","ppm"],["mq4_ppm","MQ-4","ppm"],["mq8_ppm","MQ-8","ppm"],["mq7_ppm","MQ-7","ppm"],["voc_index","VOC",""],["max30105_smoke","烟雾IR",""],["max30105_temp_c","MAX温度","°C"]];
const fields=[["temp_internal_c","内部温度 °C"],["temp_ambient_c","外部温度 °C"],["humidity_percent","湿度 %"],["mq2_ppm","MQ-2 ppm"],["mq4_ppm","MQ-4 ppm"],["mq8_ppm","MQ-8 ppm"],["mq7_ppm","MQ-7 ppm"],["voc_index","VOC 下限"],["max30105_smoke","烟雾IR"],["max30105_temp_c","MAX温度 °C"]];
const cards=document.getElementById("cards"),form=document.getElementById("thresholdForm"),ctx=document.getElementById("chart").getContext("2d"),lowerLimitFields=new Set(["voc_index"]);let currentThresholds={};
sensors.forEach(([k,n,u])=>cards.insertAdjacentHTML("beforeend",`<div class="card"><label>${n}</label><div class="value" id="${k}">--<span class="unit">${u}</span></div></div>`));
fields.forEach(([k,n])=>form.insertAdjacentHTML("beforeend",`<div class="field"><label>${n}</label><input name="${k}" type="number" step="0.01"></div>`));
function setStatus(s){const e=document.getElementById("status");e.textContent=s||"UNKNOWN";e.className="badge "+(s||"")}
async function loadThresholds(){const r=await fetch("/api/thresholds");const t=await r.json();currentThresholds=t;fields.forEach(([k])=>{if(form[k])form[k].value=t[k]??""});document.getElementById("thresholdMsg").textContent="阈值已刷新 "+new Date().toLocaleTimeString()}
function isExceeded(key,value){const threshold=Number(currentThresholds[key]);if(!Number.isFinite(value)||!Number.isFinite(threshold))return false;return lowerLimitFields.has(key)?value<threshold:value>threshold}
async function tick(){try{const s=await (await fetch("/api/status")).json();setStatus(s.status);document.getElementById("net").textContent=`IP ${s.ip} · 本地 ${s.local_status} · ${s.wifi_connected?"已联网":"未联网"} · ${s.ap_active?"AP开":"AP关"}`;document.getElementById("closeApBtn").disabled=!s.ap_close_allowed;sensors.forEach(([k,,u])=>{const v=Number(s[k]);const e=document.getElementById(k);e.innerHTML=(Number.isFinite(v)?v:0).toFixed(k.includes("ppm")||k.includes("smoke")||k=="voc_index"?0:1)+`<span class="unit">${u}</span>`;e.closest(".card").classList.toggle("alert",isExceeded(k,v))});draw(await (await fetch("/api/history")).json())}catch(e){document.getElementById("net").textContent="连接设备中..."}} 
function niceMax(v){if(v<=60)return 60;const p=Math.pow(10,Math.floor(Math.log10(v)));return Math.ceil(v/p)*p}
function draw(data){ctx.clearRect(0,0,1000,320);const plot={l:62,t:38,r:980,b:278};const series=[["temp_internal_c","#b91c1c","内温 °C"],["temp_ambient_c","#0f766e","外温 °C"],["humidity_percent","#2563eb","湿度 %"],["mq2_ppm","#a16207","MQ2 ppm"],["voc_index","#7c3aed","VOC"]];const max=niceMax(Math.max(60,...data.flatMap(d=>series.map(([k])=>Number(d[k])||0))));ctx.font="12px sans-serif";ctx.lineWidth=1;ctx.strokeStyle="#d8ded5";ctx.fillStyle="#69746d";for(let i=0;i<=5;i++){const value=max*(5-i)/5;const y=plot.t+i*(plot.b-plot.t)/5;ctx.beginPath();ctx.moveTo(plot.l,y);ctx.lineTo(plot.r,y);ctx.stroke();ctx.textAlign="right";ctx.fillText(value.toFixed(value>=100?0:1),plot.l-8,y+4)}ctx.strokeStyle="#8a938b";ctx.beginPath();ctx.moveTo(plot.l,plot.t);ctx.lineTo(plot.l,plot.b);ctx.lineTo(plot.r,plot.b);ctx.stroke();for(let i=0;i<=4;i++){const x=plot.l+i*(plot.r-plot.l)/4;ctx.strokeStyle="#eef1eb";ctx.beginPath();ctx.moveTo(x,plot.t);ctx.lineTo(x,plot.b);ctx.stroke();ctx.fillStyle="#69746d";ctx.textAlign="center";const sec=data.length?(-10+i*2.5):0;ctx.fillText(sec===0?"now":sec+"s",x,plot.b+18)}ctx.textAlign="left";ctx.fillStyle="#18201c";ctx.fillText("标度: 0 - "+max.toFixed(max>=100?0:1)+"   时间: 最近10秒",plot.l,20);series.forEach(([k,c,label],si)=>{ctx.strokeStyle=c;ctx.lineWidth=2;ctx.beginPath();data.forEach((d,i)=>{const x=plot.l+i*((plot.r-plot.l)/Math.max(1,data.length-1));const y=plot.b-(Number(d[k]||0)/max)*(plot.b-plot.t);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();ctx.fillStyle=c;ctx.fillRect(plot.l+si*120,296,10,10);ctx.fillStyle="#18201c";ctx.fillText(label,plot.l+14+si*120,305)})}
function autoValue(data,key,mode,margin){const values=data.map(d=>Number(d[key])).filter(Number.isFinite);if(!values.length)return null;const avg=values.reduce((a,b)=>a+b,0)/values.length;const base=mode==="avg"?avg:(lowerLimitFields.has(key)?Math.min(...values):Math.max(...values));return lowerLimitFields.has(key)?Math.max(0,base*(1-margin)):base*(1+margin)}
function fillAutoThresholds(data){const mode=document.getElementById("autoMode").value,margin=Math.max(0,Number(document.getElementById("autoMargin").value)||0)/100;fields.forEach(([k])=>{const v=autoValue(data,k,mode,margin);if(v===null||!form[k])return;form[k].value=k==="max30105_smoke"?String(Math.ceil(v)):v.toFixed(k.includes("ppm")||k==="voc_index"?1:2)});document.getElementById("thresholdMsg").textContent="已按最近10秒"+(mode==="avg"?"平均值":"最大值")+"生成并保存"}
form.addEventListener("submit",async e=>{e.preventDefault();await fetch("/api/thresholds",{method:"POST",body:new FormData(form)});await loadThresholds()});
document.getElementById("autoThresholdBtn").addEventListener("click",async()=>{const data=await (await fetch("/api/history")).json();if(!data.length){document.getElementById("thresholdMsg").textContent="历史数据不足，稍等几秒再生成";return}fillAutoThresholds(data);await fetch("/api/thresholds",{method:"POST",body:new FormData(form)});await loadThresholds()});
document.getElementById("wifiForm").addEventListener("submit",async e=>{e.preventDefault();await fetch("/api/wifi",{method:"POST",body:new FormData(e.target)});document.getElementById("wifiMsg").textContent="已保存，设备正在尝试连接"});
document.getElementById("closeApBtn").addEventListener("click",async()=>{const r=await fetch("/api/ap/close",{method:"POST"});document.getElementById("wifiMsg").textContent=r.ok?"AP 已关闭":"未联网前禁止关闭 AP";tick()});
document.getElementById("clearSpiffsBtn").addEventListener("click",async()=>{if(!confirm("清除 SPIFFS 会删除已保存的 WiFi 配置，继续？"))return;const r=await fetch("/api/spiffs/clear",{method:"POST"});document.getElementById("wifiMsg").textContent=r.ok?"SPIFFS 已清除":"SPIFFS 清除失败";tick()});
loadThresholds();setInterval(tick,1000);tick();
</script></body></html>
)rawliteral";

void SendJson(const String &json) {
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", json);
}

void RedirectToRoot() {
  IPAddress ip = configPortalActive ? WiFi.softAPIP() : WiFi.localIP();
  webServer.sendHeader("Location", String("http://") + ip.toString() + "/", true);
  webServer.send(302, "text/plain", "");
}

void HandleThresholdPost() {
  if (webServer.hasArg("temp_ambient_c")) thresholds.tempAmbientWarnC = webServer.arg("temp_ambient_c").toFloat();
  if (webServer.hasArg("temp_internal_c")) thresholds.tempInternalWarnC = webServer.arg("temp_internal_c").toFloat();
  if (webServer.hasArg("humidity_percent")) thresholds.humidityWarnPercent = webServer.arg("humidity_percent").toFloat();
  if (webServer.hasArg("mq2_ppm")) thresholds.mq2WarnPpm = webServer.arg("mq2_ppm").toFloat();
  if (webServer.hasArg("mq4_ppm")) thresholds.mq4WarnPpm = webServer.arg("mq4_ppm").toFloat();
  if (webServer.hasArg("mq8_ppm")) thresholds.mq8WarnPpm = webServer.arg("mq8_ppm").toFloat();
  if (webServer.hasArg("mq7_ppm")) thresholds.mq7WarnPpm = webServer.arg("mq7_ppm").toFloat();
  if (webServer.hasArg("voc_index")) thresholds.vocIndexWarn = webServer.arg("voc_index").toFloat();
  if (webServer.hasArg("max30105_smoke")) thresholds.max30105SmokeWarn = static_cast<uint32_t>(webServer.arg("max30105_smoke").toInt());
  if (webServer.hasArg("max30105_temp_c")) thresholds.max30105TempWarnC = webServer.arg("max30105_temp_c").toFloat();
  if (!SaveThresholds()) {
    webServer.send(500, "application/json", "{\"saved\":false,\"error\":\"threshold_spiffs_write_failed\"}");
    return;
  }
  UpdateStatusLogic(millis());
  SendJson(BuildThresholdJson());
}

void HandleWifiPost() {
  String ssid = webServer.arg("ssid");
  String password = webServer.arg("password");
  ssid.trim();
  if (ssid.length() > 0) {
    String oldSsid;
    String oldPass;
    if (LoadWifiConfig(oldSsid, oldPass) && (oldSsid != ssid || oldPass != password)) {
      SavePreviousWifiConfig(oldSsid, oldPass);
    }
    bool saved = SaveWifiConfig(ssid, password);
    if (saved) {
      wifiConnectFailCount = 0;
      WiFi.begin(ssid.c_str(), password.c_str());
      SendJson("{\"saved\":true}");
    } else {
      webServer.send(500, "application/json", "{\"saved\":false,\"error\":\"spiffs_write_failed\"}");
    }
    return;
  }
  webServer.send(400, "application/json", "{\"saved\":false,\"error\":\"empty_ssid\"}");
}

void StartConfigPortal();

void StopConfigPortal() {
  if (!configPortalActive) {
    return;
  }
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  configPortalActive = false;
  apOnlineStartMs = 0;
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
  }
  Serial.println("Config AP stopped");
}

void HandleCloseApPost() {
  if (WiFi.status() != WL_CONNECTED) {
    webServer.send(409, "application/json", "{\"closed\":false,\"error\":\"wifi_not_connected\"}");
    return;
  }
  StopConfigPortal();
  SendJson("{\"closed\":true}");
}

void HandleClearSpiffsPost() {
  bool ok = SPIFFS.format();
  if (!ok) {
    webServer.send(500, "application/json", "{\"cleared\":false,\"error\":\"format_failed\"}");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    StartConfigPortal();
  }
  SendJson("{\"cleared\":true}");
}

void StartWebServer() {
  if (webServerStarted) {
    return;
  }
  webServer.on("/", HTTP_GET, []() { webServer.send_P(200, "text/html; charset=utf-8", kDashboardHtml); });
  webServer.on("/api/status", HTTP_GET, []() { SendJson(BuildStatusJson()); });
  webServer.on("/api/history", HTTP_GET, []() { SendJson(BuildHistoryJson()); });
  webServer.on("/api/thresholds", HTTP_GET, []() { SendJson(BuildThresholdJson()); });
  webServer.on("/api/thresholds", HTTP_POST, HandleThresholdPost);
  webServer.on("/api/wifi", HTTP_POST, HandleWifiPost);
  webServer.on("/api/ap/close", HTTP_POST, HandleCloseApPost);
  webServer.on("/api/spiffs/clear", HTTP_POST, HandleClearSpiffsPost);
  webServer.on("/generate_204", HTTP_GET, RedirectToRoot);
  webServer.on("/gen_204", HTTP_GET, RedirectToRoot);
  webServer.on("/hotspot-detect.html", HTTP_GET, RedirectToRoot);
  webServer.on("/connecttest.txt", HTTP_GET, RedirectToRoot);
  webServer.onNotFound(RedirectToRoot);
  webServer.begin();
  webServerStarted = true;
}

void StartConfigPortal() {
  if (configPortalActive) {
    return;
  }
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(kConfigApIp, kConfigApGateway, kConfigApSubnet);
  WiFi.softAP(kConfigApSsid, kConfigApPassword);
  dnsServer.start(53, "*", kConfigApIp);
  configPortalActive = true;
  StartWebServer();
  Serial.print("Config AP started: ");
  Serial.println(kConfigApSsid);
  Serial.print("Portal IP: ");
  Serial.println(WiFi.softAPIP());
}

bool ConnectToWiFi() {
  WiFi.mode(configPortalActive ? WIFI_AP_STA : WIFI_STA);
  String savedSsid;
  String savedPass;

  if (LoadWifiConfig(savedSsid, savedPass)) {
    Serial.print("Trying saved WiFi: ");
    Serial.println(savedSsid);
    WiFi.begin(savedSsid.c_str(), savedPass.c_str());
    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      Serial.print(".");
      delay(500);
      webServer.handleClient();
      if (configPortalActive) {
        dnsServer.processNextRequest();
      }
      attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.print("Connected to saved WiFi, IP: ");
      Serial.println(WiFi.localIP());
      wifiConnectFailCount = 0;
      StartWebServer();
      return true;
    }
    Serial.println(" Saved WiFi failed");
    WiFi.disconnect(false);
    wifiConnectFailCount++;

    if (wifiConnectFailCount >= 3) {
      String prevSsid;
      String prevPass;
      if (RevertToPreviousWifi(prevSsid, prevPass)) {
        Serial.print("Reverting to previous WiFi: ");
        Serial.println(prevSsid);
        wifiConnectFailCount = 0;
        WiFi.begin(prevSsid.c_str(), prevPass.c_str());
        uint8_t prevAttempts = 0;
        while (WiFi.status() != WL_CONNECTED && prevAttempts < 20) {
          Serial.print(".");
          delay(500);
          webServer.handleClient();
          if (configPortalActive) {
            dnsServer.processNextRequest();
          }
          prevAttempts++;
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println();
          Serial.print("Connected to previous WiFi, IP: ");
          Serial.println(WiFi.localIP());
          StartWebServer();
          return true;
        }
        Serial.println(" Previous WiFi failed");
        WiFi.disconnect(false);
        wifiConnectFailCount = 1;
      } else {
        Serial.println("No previous WiFi config available");
        wifiConnectFailCount = 0;
      }
    }
  }

  Serial.println("No usable WiFi config in SPIFFS");
  return false;
}

// 初始化硬件、网络和显示。
} // namespace

void setup() {
  Serial.begin(115200);
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
  }
  LoadThresholds();
  pinMode(kButtonPin, INPUT_PULLDOWN);
  pinMode(kBuzzerPin, OUTPUT);
  digitalWrite(kBuzzerPin, HIGH);

  analogReadResolution(12);

  Wire.begin();

  // 初始化 OLED 屏幕（SSD1306）。
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed");
    while (true) {
      delay(1000);
    }
  }
  display.clearDisplay();
  display.display();

  pixels.begin();
  pixels.clear();
  pixels.show();

  // MAX31865 配置为 3 线制 PT100。
  rtd.begin(MAX31865_3WIRE);

  sht4x.begin(Wire, 0x44);
  sgp41.begin(Wire);
  // sht4x.startPeriodicMeasurement();
  uint16_t conditioningVoc = 0;
  sgp41.executeConditioning(0, 0, conditioningVoc);

  // MAX30105 初始化失败时仍可继续运行，但 IR 数据无效。
  if (!max30105.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 init failed");
  } else {
    max30105.setup();
  }

  // 连接 Wi-Fi（尝试多个网络）。
  if (!ConnectToWiFi()) {
    Serial.println("WiFi connection failed, continuing without network...");
    StartConfigPortal();
    // 可以选择进入离线模式或重启设备
  }

  // MQTT 配置并连接。
  StartWebServer();

  mqttClient.setServer(kMqttHost, kMqttPort);
  mqttClient.setBufferSize(kMqttBufferSize);
  mqttClient.setCallback(OnMqttMessage);
  if (WiFi.status() == WL_CONNECTED) {
    EnsureMqttConnected();
  }

  UpdateDisplay();
}


void loop() {
  const uint32_t nowMs = millis();
  webServer.handleClient();
  if (configPortalActive) {
    dnsServer.processNextRequest();
  }

  if (configPortalActive && WiFi.status() == WL_CONNECTED) {
    if (apOnlineStartMs == 0) {
      apOnlineStartMs = nowMs;
    } else if (nowMs - apOnlineStartMs >= kApOnlineAutoCloseMs) {
      StopConfigPortal();
    }
  } else {
    apOnlineStartMs = 0;
  }
  
  if (WiFi.status() != WL_CONNECTED && nowMs - lastWifiRetryMs >= kWifiRetryIntervalMs) {
    lastWifiRetryMs = nowMs;
    Serial.println("WiFi disconnected, reconnecting...");
    if (!ConnectToWiFi() && !configPortalActive) {
      StartConfigPortal();
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    mqttClient.loop();
  }

  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    EnsureMqttConnected();
  }

  if (nowMs - lastSampleMs >= kSampleIntervalMs) {
    lastSampleMs = nowMs;
    ReadSensors(nowMs);
    UpdateStatusLogic(nowMs);
    UpdateNeoPixel(effectiveStatus);
    PrintSensorData(nowMs);
    PublishMqtt(nowMs);
  }

  UpdateBuzzer(nowMs);

  bool buttonPressed = digitalRead(kButtonPin) == HIGH;
  if (buttonPressed && !buttonLatched) {
    buttonLatched = true;
    if (cloudDangerLatched && cloudNormalPending) {
      cloudDangerLatched = false;
      cloudNormalPending = false;
      lastCloudStatus = StatusLevel::kNormal;
      overrideActive = false;
      overrideStatus = StatusLevel::kNormal;
      UpdateStatusLogic(nowMs);
      UpdateNeoPixel(effectiveStatus);
      digitalWrite(kBuzzerPin, HIGH);
    } else if (cloudDangerLatched) {
      UpdateStatusLogic(nowMs);
      UpdateNeoPixel(effectiveStatus);
    } else {
      // PlayMenuSwitchAnimation();
      screenIndex = (screenIndex + 1) % 5;
    }
    UpdateDisplay();
  } else if (!buttonPressed) {
    buttonLatched = false;
  }
  if (nowMs - lastDisplayMs >= 500) {
    lastDisplayMs = nowMs;
    UpdateDisplay();
  }
}
