#基于多源感知与时序预测的电动车锂电池热失控预警系统

本仓库包含一个端云协同的电池热失控监测方案：

- ESP32 端负责多传感器采集、本地阈值判断、OLED 显示、声光报警、网页监控、AP 配网和 MQTT 上报。
- 电脑端负责 MQTT 数据接收、RF/LSTM/CNN-LSTM/Transformer 在线推理、状态纠偏和桌面状态显示。
- 云端/电脑端可向 ESP32 下发 `normal`、`warning`、`danger` 状态，其中 `danger` 采用本地锁存报警机制。

## 项目结构

```text
BTR/
├── ESP32/BTR/                         # PlatformIO ESP32-S3 固件工程
│   ├── src/main.cpp                   # 采样、网页、SPIFFS、MQTT、状态机、OLED/蜂鸣器/灯控
│   ├── platformio.ini                 # 板卡与依赖配置
│   └── examples/                      # 传感器/模块示例代码
│
├── computer/                          # Python 端
│   ├── rf_status_app.py               # RF 推理 + Tkinter 桌面状态显示二合一
│   ├── main_RF.py                     # 随机森林在线推理
│   ├── main_LSTM.py                   # LSTM 在线推理
│   ├── main_cnn_lstm.py               # CNN-LSTM 在线推理
│   ├── main_transformer.py            # Transformer 在线推理
│   ├── train_model_random_forest.py   # 随机森林训练脚本
│   ├── train_modelLSTM.py             # LSTM 训练脚本
│   ├── train_model_lstm_60s.py        # 60 秒 LSTM 训练脚本
│   ├── train_model_cnnlstm.py         # CNN-LSTM 训练脚本
│   ├── train_model_transformer.py     # Transformer 训练脚本
│   ├── requirements.txt               # Python 依赖
│   ├── data/                          # 训练数据
│   └── models_*/                      # 各模型产物目录
│
└── README.md
```

## ESP32 端功能

`ESP32/BTR/src/main.cpp` 当前实现：

- 多传感器采集：
  - 内部温度：MAX31865 + PT100
  - 外部温湿度：SHT4x
  - VOC：SGP41
  - 气体：MQ-2 / MQ-4 / MQ-7 / MQ-8
  - 烟雾和 MAX 温度：MAX30105
- 2Hz 采样，维护最近 10 秒历史窗口。
- 本地状态分级：`NORMAL` / `WARNING` / `DANGER`。
- OLED 常驻显示当前 IP 地址。
- WS2812、蜂鸣器与状态联动。
- 网页端实时显示传感器数值、状态和变化曲线。
- 网页端支持修改传感器阈值。
- 网页端支持按最近 10 秒平均值或最大值加冗余量自动生成阈值。
- 网页端传感器数值超过阈值时立即高亮。
- 初次使用或 WiFi 不可用时启动 AP 配网页面。
- 支持 Captive Portal，连接 AP 后设备会尝试自动弹出网页。
- 支持清除 SPIFFS、关闭 AP。
- 已联网时 AP 5 分钟后自动关闭。
- 未连接 WiFi 前禁止关闭 AP。
- WiFi 连接失败超过 3 次会回退到上一个 WiFi。

### SPIFFS 存储

ESP32 使用 SPIFFS 保存运行配置：

| 文件 | 用途 |
|------|------|
| `/wifi.txt` | 当前 WiFi SSID 和密码 |
| `/wifi_prev.txt` | 上一个 WiFi SSID 和密码 |
| `/thresholds.txt` | 当前传感器阈值 |

阈值默认值仍在程序 `ThresholdConfig` 中定义。启动时如果 `/thresholds.txt` 不存在、打不开或某一项缺失，则对应项使用程序默认常量作为判断逻辑。

### 阈值判断逻辑

ESP32 本地状态判断位于 `EvaluateStatus()`：

- 大多数传感器为上限判断：`当前值 > 阈值`
- VOC 为下限判断：`voc_index < vocIndexWarn`
- 超阈值数量为 0：`NORMAL`
- 超阈值数量为 1 到 2：`WARNING`
- 超阈值数量大于 2：`DANGER`

网页端和桌面端高亮逻辑与 ESP32 本地判断保持一致。

## 云端状态逻辑

MQTT 下行主题为 `statue`，支持消息：

- `danger`
- `warning`
- `normal`

当前 ESP32 逻辑：

1. 云端返回 `danger`
   - ESP32 进入云端危险锁存。
   - 本地持续保持 `DANGER`，蜂鸣器持续报警。

2. 云端随后返回 `normal`
   - ESP32 不会立刻切换到 `NORMAL`。
   - 设备进入 `cloud_normal_pending`，仍保持 `DANGER`。

3. 用户按下 ESP32 设备按键确认
   - 解除云端危险锁存。
   - 关闭云端覆盖。
   - 状态重新交给本地 `EvaluateStatus()` 判断。

4. 确认后
   - 如果本地传感器仍超阈值，仍可能显示 `WARNING` 或 `DANGER`。
   - 如果本地正常，显示 `NORMAL`。
   - 后续一直使用本地判断，直到云端再次返回 `danger`。

## MQTT 配置

默认配置位于 ESP32 `main.cpp` 和电脑端 `main_RF.py`：

| 项 | 值 |
|----|----|
| Broker | `bemfa.com` |
| Port | `9501` |
| ESP32 发布主题 | `sensor` |
| ESP32 订阅主题 | `statue` |

> 注意：订阅主题当前拼写为 `statue`，代码两端保持一致，暂不改为 `status`，避免破坏兼容。

### ESP32 上报 JSON 示例

```json
{
  "timestamp_ms": 123456,
  "temp_ambient_c": 25.5,
  "temp_internal_c": 32.1,
  "humidity_percent": 45.2,
  "mq2_ppm": 15.3,
  "mq4_ppm": 12.1,
  "mq8_ppm": 8.5,
  "mq7_ppm": 22.0,
  "voc_index": 85.0,
  "max30105_smoke": 120,
  "max30105_temp_c": 33.5,
  "thresholds": {
    "temp_ambient_c": 50.0,
    "temp_internal_c": 50.0,
    "humidity_percent": 80.0,
    "mq2_ppm": 200.0,
    "mq4_ppm": 250.0,
    "mq8_ppm": 450.0,
    "mq7_ppm": 750.0,
    "voc_index": 200.0,
    "max30105_smoke": 750,
    "max30105_temp_c": 50.0
  },
  "status": "NORMAL"
}
```

## 电脑端功能

### 桌面端 RF 状态显示二合一

推荐运行：

```powershell
cd D:\competition\BTR\computer
.\env\Scripts\python.exe .\rf_status_app.py
```

功能：

- 连接 MQTT 并订阅 `sensor`。
- 实时显示 ESP32 上传的各传感器数值。
- 传感器超阈值时立即高亮。
- 绘制实时变化曲线。
- 使用随机森林模型对内部温度和外部温度进行风险推理。
- 当 AI 判断和 ESP32 状态不一致时，向 `statue` 发送纠偏状态。
- 支持手动发送 `normal` / `danger`。

### 其他在线推理脚本

```powershell
cd D:\competition\BTR\computer
.\env\Scripts\python.exe .\main_RF.py
.\env\Scripts\python.exe .\main_LSTM.py
.\env\Scripts\python.exe .\main_cnn_lstm.py
.\env\Scripts\python.exe .\main_transformer.py
```

## 快速开始

### ESP32 固件编译

```powershell
cd D:\competition\BTR\ESP32\BTR
pio run
```

上传固件：

```powershell
pio run -t upload
```

### Python 环境

```powershell
cd D:\competition\BTR\computer
python -m venv env
.\env\Scripts\pip.exe install -r requirements.txt
```

### 训练模型

```powershell
cd D:\competition\BTR\computer
.\env\Scripts\python.exe .\train_model_random_forest.py
.\env\Scripts\python.exe .\train_modelLSTM.py
.\env\Scripts\python.exe .\train_model_lstm_60s.py
.\env\Scripts\python.exe .\train_model_cnnlstm.py
.\env\Scripts\python.exe .\train_model_transformer.py
```

## 硬件清单

| 组件 | 型号 | 数量 |
|------|------|------|
| 主控板 | ESP32-S3 DevKitM-1 | 1 |
| 温湿度传感器 | Sensirion SHT4x | 1 |
| RTD 温度传感器 | Adafruit MAX31865 + PT100 | 1 |
| VOC 传感器 | Sensirion SGP41 | 1 |
| MQ 气体传感器 | MQ-2 / MQ-4 / MQ-7 / MQ-8 | 各 1 |
| 烟雾/光学传感器 | SparkFun MAX30105 | 1 |
| OLED | SSD1306 128x64 I2C | 1 |
| RGB 灯 | WS2812 NeoPixel | 1 |
| 蜂鸣器 | 有源蜂鸣器 | 1 |
| 按键 | 普通按键 | 1 |

## 引脚配置

| 模块 | ESP32-S3 引脚 |
|------|---------------|
| WS2812 RGB 灯 | GPIO 48 |
| 蜂鸣器 | GPIO 18 |
| 按键 | GPIO 17 |
| MAX31865 CS | GPIO 10 |
| MAX31865 MOSI | GPIO 11 |
| MAX31865 MISO | GPIO 13 |
| MAX31865 SCK | GPIO 12 |
| MQ-2 | GPIO 1 |
| MQ-4 | GPIO 2 |
| MQ-8 | GPIO 3 |
| MQ-7 | GPIO 4 |
| SHT4x / SGP41 / OLED / MAX30105 | I2C GPIO 21 / 22 |

## 常见排查

- 桌面端提示 MQTT 未连接：检查网络、Broker、端口和 client id。
- 桌面端已连接但没有数据：检查 ESP32 串口是否有 `Connecting to MQTT... connected`。
- ESP32 串口出现 `MQTT publish failed`：payload 过大或 MQTT 未连接，当前固件已将 PubSubClient 缓冲区增大到 1024。
- 修改阈值后状态未变化：网页保存阈值后会立即写入 SPIFFS 并重新计算状态。
- 清除 SPIFFS 后：WiFi 和阈值配置都会清空，阈值回到程序默认常量，设备会重新进入 AP 配网。
