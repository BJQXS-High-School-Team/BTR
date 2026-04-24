"""
电池热失控实时预测 - Transformer 版本
从 MQTT 接收传感器数据（内部温度、外部温度、ESP32状态），
对两个温度序列分别进行预处理（添加差分特征、标准化），
用 Transformer 模型预测热失控概率，并与 ESP32 自检状态进行双向纠正。
"""

import json
import os
from collections import deque
import numpy as np
import paho.mqtt.client as mqtt
import pickle
import tensorflow as tf
from tensorflow import keras
from scipy.signal import savgol_filter

# ====== MQTT 配置（与 ESP32 一致）=======
MQTT_BROKER = "bemfa.com"
MQTT_PORT = 9501
MQTT_CLIENT_ID = "84810b9b5f5245fdbc1e1738837f27a9"   # 巴法云私钥
MQTT_TOPIC = "sensor"                                 # 订阅的数据主题
MQTT_STATUS_TOPIC = "statue"                          # 发布纠正状态的主题

# ====== Transformer 模型目录（根据训练时设置的目录修改）======
MODEL_DIR = "models_transformer"   # 训练 Transformer 时使用的目录


# ====== 预处理函数：添加差分特征（与训练时保持一致）======
def add_derivative_features_single(seq, use_smoothing, smooth_window, smooth_polyorder):
    """
    对单个温度序列（长度为 seq_len）添加一阶和二阶导数特征
    返回形状 (seq_len, n_features) 的数组，n_features = 3（温度、一阶导、二阶导）
    """
    seq = np.asarray(seq, dtype=np.float64).reshape(1, -1)  # 变为 (1, seq_len)

    if use_smoothing and seq.shape[1] >= smooth_window:
        # 平滑后的一阶导
        deriv1 = savgol_filter(seq, window_length=smooth_window, polyorder=smooth_polyorder,
                               deriv=1, axis=1)
        # 平滑后的二阶导
        deriv2 = savgol_filter(seq, window_length=smooth_window, polyorder=smooth_polyorder,
                               deriv=2, axis=1)
    else:
        # 简单差分
        diff1 = np.diff(seq, n=1, axis=1)
        deriv1 = np.concatenate([np.zeros((1, 1)), diff1], axis=1)
        diff2 = np.diff(seq, n=2, axis=1)
        deriv2 = np.concatenate([np.zeros((1, 2)), diff2], axis=1)

    # 堆叠：温度, 一阶导, 二阶导 -> (1, seq_len, 3)
    X_multi = np.stack([seq, deriv1, deriv2], axis=2)  # shape (1, seq_len, 3)
    return X_multi[0]  # 返回 (seq_len, 3)


# ====== 加载 Transformer 模型、标准化器和配置 ======
def load_transformer_model(model_dir):
    """返回 (model, scaler, config)"""
    config_path = os.path.join(model_dir, 'config.pkl')
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"配置文件不存在: {config_path}")

    with open(config_path, 'rb') as f:
        config = pickle.load(f)

    # 关键修改：添加 compile=False，避免版本兼容性问题
    model = keras.models.load_model(
        os.path.join(model_dir, 'transformer_model.h5'),
        compile=False
    )
    # 手动编译模型（使用与训练时相同的优化器和损失）
    model.compile(
        optimizer=keras.optimizers.Adam(learning_rate=0.001),  # 与训练时的初始学习率一致
        loss='binary_crossentropy',
        metrics=['accuracy']
    )

    with open(os.path.join(model_dir, 'scaler.pkl'), 'rb') as f:
        scaler = pickle.load(f)

    return model, scaler, config

# 尝试加载模型
# try:
MODEL_TRANS, SCALER_TRANS, CONFIG = load_transformer_model(MODEL_DIR)
SEQ_LEN = CONFIG.get('sequence_length', 60)
BEST_THRESHOLD = CONFIG.get('best_threshold', 0.5)
USE_DIFF_FEATURES = CONFIG.get('use_diff_features', False)
USE_SMOOTHING = CONFIG.get('use_smoothing', False)
SMOOTH_WINDOW = CONFIG.get('smooth_window', 11)
SMOOTH_POLYORDER = CONFIG.get('smooth_polyorder', 3)
print(f"Transformer 模型加载成功，序列长度: {SEQ_LEN}，最佳阈值: {BEST_THRESHOLD}")
print(f"使用差分特征: {USE_DIFF_FEATURES}, 平滑: {USE_SMOOTHING}")
"""except Exception as e:
    print(f"加载模型失败: {e}")
    MODEL_TRANS = None
    SCALER_TRANS = None
    SEQ_LEN = 60
    BEST_THRESHOLD = 0.5
    USE_DIFF_FEATURES = False
    USE_SMOOTHING = False
    SMOOTH_WINDOW = 11
    SMOOTH_POLYORDER = 3"""

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


def preprocess_and_predict(seq):
    """
    对单个温度序列进行预处理（添加差分特征、标准化）并预测概率
    参数：
        seq: 长度为 SEQ_LEN 的一维数组
    返回：
        预测概率 float，若出错返回 None
    """
    try:
        if USE_DIFF_FEATURES:
            # 添加差分特征，得到 (SEQ_LEN, n_features)
            X_multi = add_derivative_features_single(seq, USE_SMOOTHING,
                                                      SMOOTH_WINDOW, SMOOTH_POLYORDER)
        else:
            X_multi = seq.reshape(-1, 1)  # (SEQ_LEN, 1)

        # 标准化：SCALER_TRANS 是按特征通道拟合的，需要对每个时间步的特征进行转换
        # 将 X_multi reshape 为 (SEQ_LEN, n_features)，然后 transform
        X_reshaped = X_multi.reshape(-1, X_multi.shape[1])   # (seq_len, n_features)
        X_scaled = SCALER_TRANS.transform(X_reshaped)        # (seq_len, n_features)
        X_scaled = X_scaled.reshape(1, SEQ_LEN, -1)          # (1, seq_len, n_features)

        # 预测
        prob = MODEL_TRANS.predict(X_scaled, verbose=0)[0, 0]
        return float(prob)
    except Exception as e:
        print(f"预处理或预测出错: {e}")
        return None


def process_and_predict(esp32_status):
    """缓冲区满时调用，进行预测并返回需要发送的纠正状态（若需要）"""
    if MODEL_TRANS is None or SCALER_TRANS is None:
        return None

    if len(temp_internal_buffer) < SEQ_LEN or len(temp_ambient_buffer) < SEQ_LEN:
        return None

    # 内部温度预测
    seq_int = np.array(list(temp_internal_buffer))
    prob_int = preprocess_and_predict(seq_int)
    if prob_int is None:
        return None

    # 外部温度预测
    seq_amb = np.array(list(temp_ambient_buffer))
    prob_amb = preprocess_and_predict(seq_amb)
    if prob_amb is None:
        return None

    # 打印结果
    print("\n=== Transformer 预测结果 ===")
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