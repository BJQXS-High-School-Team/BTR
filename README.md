# 基于多源感知与时序预测的电动车锂电池热失控预警系统

BTR 是一个围绕电动车锂电池热失控早期预警设计的软硬件项目。系统使用 ESP32-S3 连接温度、湿度、气体、VOC、烟雾/光学等传感器，完成本地采集、网页显示、MQTT 上报和声光报警；电脑端接收实时数据后，可以调用随机森林、LSTM、CNN-LSTM、Transformer 等模型进行在线推理，并在需要时向 ESP32 回传修正后的状态。

项目希望解决的问题比较明确：单独依靠温度阈值报警容易偏晚，也容易受环境影响。BTR 尝试把多类传感器数据放在同一个时间窗口中观察，结合本地规则和模型判断，尽量更早发现异常趋势。

---

## 项目特点

- **多源采集**：温度、湿度、MQ 系列气体、VOC、烟雾/光学信号同步采集。
- **边缘判断**：ESP32 端可独立完成 `NORMAL` / `WARNING` / `DANGER` 三级判断。
- **网页端监控**：支持实时数值、曲线、阈值配置、AP 配网和 SPIFFS 管理。
- **桌面端推理**：Python 桌面端接收 MQTT 数据，显示曲线，并调用随机森林模型输出风险概率。
- **状态回传**：电脑端可向 ESP32 下发 `normal`、`warning`、`danger`，用于纠偏和联动演示。
- **多模型训练**：仓库中包含 RF、LSTM、CNN-LSTM、Transformer 等模型训练和在线推理脚本。

---

## 系统思路

```text
ESP32 多传感器采集
    ↓
本地阈值判断与状态分级
    ↓
网页端实时显示 / OLED 显示 / 声光报警
    ↓
MQTT 上传 JSON 数据
    ↓
Python 端接收数据并构建时间窗口
    ↓
RF / LSTM / CNN-LSTM / Transformer 模型推理
    ↓
输出风险概率或风险状态
    ↓
必要时通过 MQTT 回传 normal / warning / danger
```

---

## 网页端与桌面端的分工

### ESP32 网页端

网页端运行在 ESP32 内置 WebServer 中，主要面向现场调试和快速部署：

- 显示内部温度、外部温湿度、MQ 气体、VOC、烟雾等实时数据。
- 显示当前状态：`NORMAL` / `WARNING` / `DANGER`。
- 绘制最近一段时间的传感器变化曲线。
- 支持手动修改阈值。
- 支持按近期数据自动生成参考阈值。
- 支持 AP 配网、关闭 AP、清除 SPIFFS。

### Python 桌面端

桌面端位于 `computer/rf_status_app.py`，主要面向电脑端监控和模型推理：

- 通过 MQTT 订阅 ESP32 上传的 `sensor` 数据。
- 实时显示传感器数值和变化曲线。
- 使用随机森林模型输出热失控风险概率。
- 显示 ESP32 本地判断状态和模型纠偏状态。
- 当模型判断与 ESP32 本地状态不一致时，可向 `statue` 主题回传纠偏状态。
- 支持手动发送 `normal` / `danger`，方便联调和演示。

---

## 数据与模型

项目中的数据主要用于训练和验证热失控风险识别模型。数据包括项目组自主测试采集数据，也包括合作实验室授权的真实测试数据。相关数据保存在 `computer/data/` 和 `data_backup_*` 目录中。

目前仓库中包含以下模型路线：

| 模型 | 相关文件 | 说明 |
|------|----------|------|
| Random Forest | `train_model_random_forest.py` / `main_RF.py` | 用于桌面端在线推理和风险概率显示，解释性较好 |
| LSTM | `train_modelLSTM.py` / `main_LSTM.py` | 用于连续温度时序变化判断 |
| 60s LSTM | `train_model_lstm_60s.py` | 用于更长时间窗口的趋势识别 |
| CNN-LSTM | `train_model_cnnlstm.py` / `main_cnn_lstm.py` | 结合局部特征提取和时序建模 |
| Transformer | `train_model_transformer.py` / `main_transformer.py` | 用于较长序列中的关联特征建模 |

---

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


---

## 后续计划

- 继续补充不同工况下的测试数据。
- 对比 RF、LSTM、CNN-LSTM、Transformer 在准确率、召回率和实时性上的差异。
- 优化传感器固定结构和走线方式。
- 将网页端、桌面端和报警逻辑整理成更稳定的演示流程。
- 在保证安全的前提下，增加更多可复现实验记录。
