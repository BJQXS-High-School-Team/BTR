"""
电池热失控实时预测 - 随机森林版本
从 MQTT 接收传感器数据（内部温度、外部温度、ESP32状态），
对两个温度序列分别提取特征，用随机森林模型预测热失控概率，
并与 ESP32 自检状态进行双向纠正。
"""

import json
import os
from collections import deque
import numpy as np
import paho.mqtt.client as mqtt
import pickle
import joblib
from scipy.signal import savgol_filter

# ====== MQTT 配置（与 ESP32 一致）=======
MQTT_BROKER = "bemfa.com"
MQTT_PORT = 9501
MQTT_CLIENT_ID = "84810b9b5f5245fdbc1e1738837f27a9"   # 巴法云私钥
MQTT_TOPIC = "sensor"                                 # 订阅的数据主题
MQTT_STATUS_TOPIC = "statue"                          # 发布纠正状态的主题

# ====== 随机森林模型目录 ======
MODEL_DIR = "models_random_forest"


# ====== 特征提取函数（必须与训练时完全一致）======
def extract_features_from_sequence(seq, use_smoothing=True, smooth_window=11, smooth_polyorder=3):
    """
    从单个温度序列提取特征向量
    参数：
        seq: 长度为 SEQ_LEN 的一维 numpy 数组
        use_smoothing: 是否使用平滑导数（需与训练时一致）
        smooth_window, smooth_polyorder: Savitzky-Golay 平滑参数
    返回：
        特征列表（长度取决于 use_smoothing，基础特征 31 维，平滑额外 8 维）
    """
    seq = np.asarray(seq, dtype=np.float64)

    # 输入有效性检查
    if len(seq) == 0 or np.any(np.isnan(seq)) or np.any(np.isinf(seq)):
        print("警告: 输入序列无效，返回零特征")
        base_len = 31
        extra_len = 8 if use_smoothing else 0
        return [0.0] * (base_len + extra_len)

    features = []

    # 1. 基本统计量 (9个)
    features.append(np.mean(seq))
    features.append(np.std(seq))
    features.append(np.min(seq))
    features.append(np.max(seq))
    features.append(np.ptp(seq))          # 峰峰值
    features.append(np.median(seq))
    features.append(np.percentile(seq, 25))
    features.append(np.percentile(seq, 75))
    features.append(np.percentile(seq, 90))

    # 2. 线性趋势 (2个)
    x = np.arange(len(seq))
    try:
        slope, intercept = np.polyfit(x, seq, 1)
    except:
        slope, intercept = 0.0, 0.0
    features.append(slope)
    features.append(intercept)

    # 3. 一阶差分特征 (6个)
    diff1 = np.diff(seq)
    features.append(np.mean(diff1) if len(diff1) > 0 else 0.0)
    features.append(np.std(diff1) if len(diff1) > 1 else 0.0)
    features.append(np.max(diff1) if len(diff1) > 0 else 0.0)
    features.append(np.min(diff1) if len(diff1) > 0 else 0.0)
    pos_ratio = np.sum(diff1 > 0) / len(diff1) if len(diff1) > 0 else 0.0
    neg_ratio = np.sum(diff1 < 0) / len(diff1) if len(diff1) > 0 else 0.0
    features.append(pos_ratio)
    features.append(neg_ratio)

    # 4. 二阶差分特征 (4个)
    if len(diff1) > 1:
        diff2 = np.diff(diff1)
        features.append(np.mean(diff2))
        features.append(np.std(diff2) if len(diff2) > 1 else 0.0)
        features.append(np.max(diff2))
        features.append(np.min(diff2))
    else:
        features.extend([0.0, 0.0, 0.0, 0.0])

    # 5. 相对变化 (2个)
    features.append(seq[-1] - seq[0])
    features.append((seq[-1] - seq[0]) / len(seq))

    # 6. 最后10点斜率 (1个)
    last10 = seq[-10:]
    if len(last10) >= 2:
        try:
            slope_last10 = np.polyfit(np.arange(len(last10)), last10, 1)[0]
        except:
            slope_last10 = 0.0
    else:
        slope_last10 = 0.0
    features.append(slope_last10)

    # 7. 平滑导数特征（如果训练时使用）
    if use_smoothing and len(seq) >= smooth_window:
        try:
            deriv1 = savgol_filter(seq, window_length=smooth_window,
                                   polyorder=smooth_polyorder, deriv=1)
            deriv2 = savgol_filter(seq, window_length=smooth_window,
                                   polyorder=smooth_polyorder, deriv=2)
            features.append(np.mean(deriv1))
            features.append(np.std(deriv1))
            features.append(np.max(deriv1))
            features.append(np.min(deriv1))
            features.append(np.mean(deriv2))
            features.append(np.std(deriv2))
            features.append(np.max(deriv2))
            features.append(np.min(deriv2))
        except Exception as e:
            print(f"平滑导数计算错误: {e}，填充零")
            features.extend([0.0] * 8)
    else:
        features.extend([0.0] * 8)

    return features


# ====== 加载随机森林模型、标准化器和配置 ======
def load_rf_model(model_dir):
    """返回 (model, scaler, config)"""
    config_path = os.path.join(model_dir, 'config.pkl')
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"配置文件不存在: {config_path}")

    with open(config_path, 'rb') as f:
        config = pickle.load(f)

    model = joblib.load(os.path.join(model_dir, 'random_forest_model.pkl'))
    with open(os.path.join(model_dir, 'scaler.pkl'), 'rb') as f:
        scaler = pickle.load(f)

    return model, scaler, config


# 尝试加载模型
try:
    MODEL_RF, SCALER_RF, CONFIG = load_rf_model(MODEL_DIR)
    SEQ_LEN = CONFIG.get('sequence_length', 60)
    BEST_THRESHOLD = CONFIG.get('best_threshold', 0.5)
    USE_SMOOTHING = CONFIG.get('use_smoothing', False)
    SMOOTH_WINDOW = CONFIG.get('smooth_window', 11)
    SMOOTH_POLYORDER = CONFIG.get('smooth_polyorder', 3)
    print(f"随机森林模型加载成功，序列长度: {SEQ_LEN}，最佳阈值: {BEST_THRESHOLD}")
    print(f"平滑使用: {USE_SMOOTHING}")
except Exception as e:
    print(f"加载模型失败: {e}")
    MODEL_RF = None
    SCALER_RF = None
    SEQ_LEN = 60
    BEST_THRESHOLD = 0.5
    USE_SMOOTHING = False
    SMOOTH_WINDOW = 11
    SMOOTH_POLYORDER = 3

# 温度缓冲区（分别存储内部和外部温度）
temp_internal_buffer = deque(maxlen=SEQ_LEN)
temp_ambient_buffer = deque(maxlen=SEQ_LEN)
time_buffer = deque(maxlen=SEQ_LEN)   # 用于监控数据间隔


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"已连接到巴法云，订阅主题: {MQTT_TOPIC}")
        client.subscribe(MQTT_TOPIC)
    else:
        print(f"连接失败，错误码: {rc}")


def process_and_predict(esp32_status):
    """缓冲区满时调用，进行预测并返回需要发送的纠正状态（若需要）"""
    if MODEL_RF is None or SCALER_RF is None:
        return None

    if len(temp_internal_buffer) < SEQ_LEN or len(temp_ambient_buffer) < SEQ_LEN:
        return None

    # 内部温度特征
    seq_int = np.array(list(temp_internal_buffer))
    feat_int = extract_features_from_sequence(
        seq_int,
        use_smoothing=USE_SMOOTHING,
        smooth_window=SMOOTH_WINDOW,
        smooth_polyorder=SMOOTH_POLYORDER
    )
    if any(np.isnan(feat_int)):
        print("内部特征包含 nan，跳过本次预测")
        return None

    # 外部温度特征
    seq_amb = np.array(list(temp_ambient_buffer))
    feat_amb = extract_features_from_sequence(
        seq_amb,
        use_smoothing=USE_SMOOTHING,
        smooth_window=SMOOTH_WINDOW,
        smooth_polyorder=SMOOTH_POLYORDER
    )
    if any(np.isnan(feat_amb)):
        print("外部特征包含 nan，跳过本次预测")
        return None

    # 标准化并预测
    try:
        feat_int_scaled = SCALER_RF.transform([feat_int])   # 二维数组 (1, n_features)
        prob_int = MODEL_RF.predict_proba(feat_int_scaled)[0, 1]

        feat_amb_scaled = SCALER_RF.transform([feat_amb])
        prob_amb = MODEL_RF.predict_proba(feat_amb_scaled)[0, 1]
    except Exception as e:
        print(f"预测过程出错: {e}")
        return None

    # 打印结果
    print("\n=== 随机森林预测结果 ===")
    print(f"内部热失控概率: {prob_int:.2%}")
    print(f"外部热失控概率: {prob_amb:.2%}")
    print(f"ESP32状态: {esp32_status}")

    ai_is_danger = prob_int >= BEST_THRESHOLD or prob_amb >= BEST_THRESHOLD

    # 双向纠正逻辑
    if esp32_status == "DANGER" and not ai_is_danger:
        return "normal"
    elif esp32_status in ["NORMAL", "WARNING"] and ai_is_danger:
        return "danger"
    return None


def on_message(client, userdata, msg):
    """MQTT 消息回调"""
    try:
        payload = msg.payload.decode('utf-8')
        data = json.loads(payload)

        ts = data.get('timestamp_ms')
        temp_ambient = data.get('temp_ambient_c')
        temp_internal = data.get('temp_internal_c')
        esp32_status = data.get('status', 'UNKNOWN')

        # 数据有效性检查
        if temp_internal is None or temp_ambient is None:
            print("消息缺少温度字段")
            return

        try:
            temp_int_val = float(temp_internal)
            temp_amb_val = float(temp_ambient)
        except (ValueError, TypeError):
            print(f"温度值无法转换为浮点数: 内部={temp_internal}, 外部={temp_ambient}")
            return

        if not (np.isfinite(temp_int_val) and np.isfinite(temp_amb_val)):
            print(f"温度值无效: 内部={temp_int_val}, 外部={temp_amb_val}")
            return

        # 加入缓冲区
        temp_internal_buffer.append(temp_int_val)
        temp_ambient_buffer.append(temp_amb_val)
        if ts is not None:
            time_buffer.append(ts / 1000.0)

        print(f"\n收到数据 - 内部: {temp_int_val}°C, 外部: {temp_amb_val}°C, 缓冲区: {len(temp_internal_buffer)}/{SEQ_LEN}")

        # 当缓冲区满时进行预测
        if len(temp_internal_buffer) == SEQ_LEN:
            status = process_and_predict(esp32_status)
            if status:
                client.publish(MQTT_STATUS_TOPIC, status)
                print(f"✉️ 已发送纠正状态到 {MQTT_STATUS_TOPIC}: {status}")
            else:
                print("ℹ️ 无需纠正")

    except json.JSONDecodeError:
        print(f"非 JSON 消息: {msg.payload}")
    except Exception as e:
        print(f"处理消息时发生异常: {e}")


def start_mqtt_listener():
    """启动 MQTT 监听主循环"""
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=MQTT_CLIENT_ID)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        print("MQTT 客户端已启动，等待数据...")
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n程序手动停止")
        client.disconnect()
    except Exception as e:
        print(f"MQTT 连接异常: {e}")


if __name__ == "__main__":
    start_mqtt_listener()