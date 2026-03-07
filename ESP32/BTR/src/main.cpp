#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>
#include <SensirionI2CSht4x.h>
#include <SensirionI2CSgp41.h>
#include <MAX30105.h>
#include <WiFi.h>
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

// ====== WiFi 配置 ======
// 支持多个 WiFi 网络，按顺序尝试连接
struct WiFiCredential {
  const char* ssid;
  const char* password;
};

constexpr WiFiCredential kWifiNetworks[] = {
  {"zzz", "12345678"},
  {"blackmi", "wxy358800"},
  {"点击确认父子关系", "12345678"}
};
constexpr uint8_t kWifiNetworkCount = sizeof(kWifiNetworks) / sizeof(kWifiNetworks[0]);

// ====== MQTT 配置 ======
// MQTT 服务器：bemfa.com:9501，发布 topic 为 sensor，订阅 topic 为 statue。
constexpr char kMqttHost[] = "bemfa.com";
constexpr uint16_t kMqttPort = 9501;
constexpr char kMqttPrivateKey[] = "84810b9b5f5245fdbc1e1738837f27a9";
constexpr char kMqttPubTopic[] = "sensor";
constexpr char kMqttSubTopic[] = "statue"; // 按需求拼写

// ====== 阈值（常量） ======
// 阈值使用常量定义，便于统一修改。
constexpr float kTempAmbientWarnC = 50.0f;
constexpr float kTempInternalWarnC = 50.0f;
constexpr float kHumidityWarnPercent = 80.0f;
constexpr float kMq2WarnPpm = 200.0f;
constexpr float kMq4WarnPpm = 150.0f;
constexpr float kMq8WarnPpm = 300.0f;
constexpr float kMq7WarnPpm = 600.0f;
constexpr float kVocIndexWarn = 200.0f;

// MAX30105 阈值：烟雾、温度
constexpr uint32_t kMax30105SmokeWarn = 750;   // 烟雾浓度阈值（IR值）
constexpr float kMax30105TempWarnC = 50.0f;       // MAX30105温度阈值

// MQ 传感器转换比例（简化线性换算）
// 未做标定曲线时，先用 0~4095 ADC 映射到 0~maxPpm。
constexpr float kMq2MaxPpm = 1000.0f;
constexpr float kMq4MaxPpm = 1000.0f;
constexpr float kMq8MaxPpm = 1000.0f;
constexpr float kMq7MaxPpm = 1000.0f;

constexpr uint32_t kOverrideClearMs = 60000; // 本地正常持续 60s 清除远程覆盖

Adafruit_SSD1306 display(kScreenWidth, kScreenHeight, &Wire, kOledReset);
Adafruit_NeoPixel pixels(kNeoPixelCount, kNeoPixelPin, NEO_GRB + NEO_KHZ800);
Adafruit_MAX31865 rtd(kMax31865CsPin, kMax31865MosiPin, kMax31865MisoPin, kMax31865SckPin);
SensirionI2cSht4x sht4x;
SensirionI2CSgp41 sgp41;
MAX30105 max30105;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

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

uint8_t screenIndex = 0;
bool buttonLatched = false;

enum class StatusLevel { kNormal, kWarning, kDanger };
StatusLevel localStatus = StatusLevel::kNormal;
StatusLevel effectiveStatus = StatusLevel::kNormal;
bool overrideActive = false;
StatusLevel overrideStatus = StatusLevel::kNormal;
uint32_t localNormalStartMs = 0;

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
  if (lastTempAmbientC > kTempAmbientWarnC) {
    exceedCount++;
  }
  if (lastTempInternalC > kTempInternalWarnC) {
    exceedCount++;
  }
  if (lastHumidityPercent > kHumidityWarnPercent) {
    exceedCount++;
  }
  if (lastMq2Ppm > kMq2WarnPpm) {
    exceedCount++;
  }
  if (lastMq4Ppm > kMq4WarnPpm) {
    exceedCount++;
  }
  if (lastMq8Ppm > kMq8WarnPpm) {
    exceedCount++;
  }
  if (lastMq7Ppm > kMq7WarnPpm) {
    exceedCount++;
  }
  if (lastVocIndex < kVocIndexWarn) {
    exceedCount++;
  }
  // MAX30105 检测：烟雾、温度
  if (lastMax30105Smoke > kMax30105SmokeWarn) {
    exceedCount++;
  }
  if (lastMax30105TempC > kMax30105TempWarnC) {
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

// 处理本地计算状态与远程覆盖状态的优先级。
// 当远程覆盖生效后，只有连续 60 秒本地正常才会解除覆盖。
void UpdateStatusLogic(uint32_t nowMs) {
  localStatus = EvaluateStatus();
  if (overrideActive) {
    if (localStatus == StatusLevel::kNormal) {
      if (localNormalStartMs == 0) {
        localNormalStartMs = nowMs;
      } else if (nowMs - localNormalStartMs >= kOverrideClearMs) {
        overrideActive = false;
        localNormalStartMs = 0;
      }
    } else {
      localNormalStartMs = 0;
    }
  }

  if (overrideActive) {
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
  display.setCursor(0, 50);
  display.print("Local: ");
  display.println(StatusToString(localStatus));
  if (overrideActive) {
    display.setCursor(64, 50);
    display.print("Override");
  }
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
  display.setCursor(0, 56);
  display.print("MQ-7  (CO): ");
  display.println(lastMq7Ppm, 0);
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
  display.setCursor(0, 50);
  display.print("Air Quality");
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
      overrideActive = true;
      localNormalStartMs = 0;
    } else if (message == "warning") {
      overrideStatus = StatusLevel::kWarning;
      overrideActive = true;
      localNormalStartMs = 0;
    } else if (message == "danger") {
      overrideStatus = StatusLevel::kDanger;
      overrideActive = true;
      localNormalStartMs = 0;
    }
  }
}

// 断线重连并保持订阅主题。
void EnsureMqttConnected() {
  if (mqttClient.connected()) {
    return;
  }
  while (!mqttClient.connected()) {
    String clientId = "esp32s3-" + String(random(0xffff), HEX);
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect(kMqttPrivateKey, "", "")) {
      mqttClient.subscribe(kMqttSubTopic);
    } else {
      delay(1000);
    }
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
  payload += "\"status\":\"" + String(StatusToString(effectiveStatus)) + "\"";
  payload += "}";

  mqttClient.publish(kMqttPubTopic, payload.c_str());
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

// 尝试连接多个 WiFi 网络，返回是否成功连接
bool ConnectToWiFi() {
  WiFi.mode(WIFI_STA);
  
  for (uint8_t i = 0; i < kWifiNetworkCount; ++i) {
    Serial.print("Trying WiFi: ");
    Serial.println(kWifiNetworks[i].ssid);
    
    WiFi.begin(kWifiNetworks[i].ssid, kWifiNetworks[i].password);
    
    // 尝试连接 10 秒
    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      Serial.print(".");
      delay(500);
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.print("Connected to: ");
      Serial.println(kWifiNetworks[i].ssid);
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      return true;
    }
    
    Serial.println(" Failed");
    WiFi.disconnect();
  }
  
  Serial.println("All WiFi networks failed");
  return false;
}

// 初始化硬件、网络和显示。
} // namespace

void setup() {
  Serial.begin(115200);
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
    // 可以选择进入离线模式或重启设备
  }

  // MQTT 配置并连接。
  mqttClient.setServer(kMqttHost, kMqttPort);
  mqttClient.setCallback(OnMqttMessage);
  EnsureMqttConnected();

  UpdateDisplay();
}

// 主循环：采样、状态计算、显示、MQTT 与蜂鸣器控制。
void loop() {
  const uint32_t nowMs = millis();
  
  // 检查 WiFi 连接状态，断线重连
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    ConnectToWiFi();
  }
  
  mqttClient.loop();

  if (!mqttClient.connected()) {
    EnsureMqttConnected();
  }

  // 定时采样与上传。
  if (nowMs - lastSampleMs >= kSampleIntervalMs) {
    lastSampleMs = nowMs;
    ReadSensors(nowMs);
    UpdateStatusLogic(nowMs);
    UpdateNeoPixel(effectiveStatus);
    PrintSensorData(nowMs);
    PublishMqtt(nowMs);
  }

  UpdateBuzzer(nowMs);

  // 按键高电平触发页面切换。
  bool buttonPressed = digitalRead(kButtonPin) == HIGH;
  if (buttonPressed && !buttonLatched) {
    buttonLatched = true;
    // PlayMenuSwitchAnimation();
    screenIndex = (screenIndex + 1) % 5;  // 5 个页面循环
    UpdateDisplay();
  } else if (!buttonPressed) {
    buttonLatched = false;
  }

  if (nowMs - lastDisplayMs >= 500) {
    lastDisplayMs = nowMs;
    UpdateDisplay();
  }
}
