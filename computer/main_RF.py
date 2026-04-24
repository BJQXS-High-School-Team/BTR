import json
import os
from collections import deque

import joblib
import numpy as np
import paho.mqtt.client as mqtt
import pickle
from scipy.signal import savgol_filter

# ====== MQTT config ======
MQTT_BROKER = "bemfa.com"
MQTT_PORT = 9501
MQTT_CLIENT_ID = "84810b9b5f5245fdbc1e1738837f27a9"
MQTT_TOPIC = "sensor"
MQTT_STATUS_TOPIC = "statue"

# ====== Random forest model directory ======
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(BASE_DIR, "models_random_forest")


def extract_features_from_sequence(seq, use_smoothing=True, smooth_window=11, smooth_polyorder=3):
    """
    Extract the same 32-dimensional feature vector used during training.
    """
    seq = np.asarray(seq, dtype=np.float64)

    if len(seq) == 0 or np.any(np.isnan(seq)) or np.any(np.isinf(seq)):
        print("Warning: invalid input sequence, returning zero features")
        base_len = 31
        extra_len = 8 if use_smoothing else 0
        return [0.0] * (base_len + extra_len)

    features = []

    # 1. Basic statistics (9)
    features.append(np.mean(seq))
    features.append(np.std(seq))
    features.append(np.min(seq))
    features.append(np.max(seq))
    features.append(np.ptp(seq))
    features.append(np.median(seq))
    features.append(np.percentile(seq, 25))
    features.append(np.percentile(seq, 75))
    features.append(np.percentile(seq, 90))

    # 2. Linear trend (2)
    x = np.arange(len(seq))
    try:
        slope, intercept = np.polyfit(x, seq, 1)
    except Exception:
        slope, intercept = 0.0, 0.0
    features.append(slope)
    features.append(intercept)

    # 3. First-order differences (6)
    diff1 = np.diff(seq)
    features.append(np.mean(diff1) if len(diff1) > 0 else 0.0)
    features.append(np.std(diff1) if len(diff1) > 1 else 0.0)
    features.append(np.max(diff1) if len(diff1) > 0 else 0.0)
    features.append(np.min(diff1) if len(diff1) > 0 else 0.0)
    pos_ratio = np.sum(diff1 > 0) / len(diff1) if len(diff1) > 0 else 0.0
    neg_ratio = np.sum(diff1 < 0) / len(diff1) if len(diff1) > 0 else 0.0
    features.append(pos_ratio)
    features.append(neg_ratio)

    # 4. Second-order differences (4)
    if len(diff1) > 1:
        diff2 = np.diff(diff1)
        features.append(np.mean(diff2))
        features.append(np.std(diff2) if len(diff2) > 1 else 0.0)
        features.append(np.max(diff2))
        features.append(np.min(diff2))
    else:
        features.extend([0.0, 0.0, 0.0, 0.0])

    # 5. Relative change (2)
    features.append(seq[-1] - seq[0])
    features.append((seq[-1] - seq[0]) / len(seq))

    # 6. Slope of last 10 points (1)
    last10 = seq[-10:]
    if len(last10) >= 2:
        try:
            slope_last10 = np.polyfit(np.arange(len(last10)), last10, 1)[0]
        except Exception:
            slope_last10 = 0.0
    else:
        slope_last10 = 0.0
    features.append(slope_last10)

    # 7. Smoothing-derived features (8)
    if use_smoothing and len(seq) >= smooth_window:
        try:
            deriv1 = savgol_filter(seq, window_length=smooth_window, polyorder=smooth_polyorder, deriv=1)
            deriv2 = savgol_filter(seq, window_length=smooth_window, polyorder=smooth_polyorder, deriv=2)
            features.append(np.mean(deriv1))
            features.append(np.std(deriv1))
            features.append(np.max(deriv1))
            features.append(np.min(deriv1))
            features.append(np.mean(deriv2))
            features.append(np.std(deriv2))
            features.append(np.max(deriv2))
            features.append(np.min(deriv2))
        except Exception as e:
            print(f"Smoothing derivative calculation failed: {e}; filling zeros")
            features.extend([0.0] * 8)
    else:
        features.extend([0.0] * 8)

    return features


def load_rf_model(model_dir):
    config_path = os.path.join(model_dir, "config.pkl")
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with open(config_path, "rb") as f:
        config = pickle.load(f)

    model = joblib.load(os.path.join(model_dir, "random_forest_model.pkl"))
    with open(os.path.join(model_dir, "scaler.pkl"), "rb") as f:
        scaler = pickle.load(f)

    return model, scaler, config


try:
    MODEL_RF, SCALER_RF, CONFIG = load_rf_model(MODEL_DIR)
    SEQ_LEN = CONFIG.get("sequence_length", 60)
    BEST_THRESHOLD = float(CONFIG.get("best_threshold", 0.5))
    USE_SMOOTHING = CONFIG.get("use_smoothing", False)
    SMOOTH_WINDOW = CONFIG.get("smooth_window", 11)
    SMOOTH_POLYORDER = CONFIG.get("smooth_polyorder", 3)
    EXPECTED_FEATURES = getattr(SCALER_RF, "n_features_in_", None)
    print(
        f"Random forest model loaded successfully. "
        f"seq_len={SEQ_LEN}, threshold={BEST_THRESHOLD}, smoothing={USE_SMOOTHING}"
    )
    if EXPECTED_FEATURES is not None:
        print(f"Scaler expects {EXPECTED_FEATURES} features")
except Exception as e:
    print(f"Failed to load model: {e}")
    MODEL_RF = None
    SCALER_RF = None
    CONFIG = {}
    SEQ_LEN = 60
    BEST_THRESHOLD = 0.5
    USE_SMOOTHING = False
    SMOOTH_WINDOW = 11
    SMOOTH_POLYORDER = 3
    EXPECTED_FEATURES = None


temp_internal_buffer = deque(maxlen=SEQ_LEN)
temp_ambient_buffer = deque(maxlen=SEQ_LEN)
time_buffer = deque(maxlen=SEQ_LEN)


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected successfully, subscribing to topic...")
        client.subscribe(MQTT_TOPIC)
    else:
        print(f"Connect failed, rc={rc}")


def _prepare_feature_row(features, label):
    if features is None:
        print(f"{label} features are missing")
        return None

    arr = np.asarray(features, dtype=np.float64)
    if arr.ndim != 1:
        print(f"{label} features have unexpected shape: {arr.shape}")
        return None
    if np.any(np.isnan(arr)) or np.any(np.isinf(arr)):
        print(f"{label} features contain NaN or Inf")
        return None
    if EXPECTED_FEATURES is not None and arr.size != EXPECTED_FEATURES:
        print(f"{label} feature count mismatch: got {arr.size}, expected {EXPECTED_FEATURES}")
        return None

    return arr.reshape(1, -1)


def process_and_predict(esp32_status):
    if MODEL_RF is None or SCALER_RF is None:
        return None

    if len(temp_internal_buffer) < SEQ_LEN or len(temp_ambient_buffer) < SEQ_LEN:
        return None

    seq_int = np.asarray(list(temp_internal_buffer), dtype=np.float64)
    feat_int = extract_features_from_sequence(
        seq_int,
        use_smoothing=USE_SMOOTHING,
        smooth_window=SMOOTH_WINDOW,
        smooth_polyorder=SMOOTH_POLYORDER,
    )
    feat_int_row = _prepare_feature_row(feat_int, "internal")
    if feat_int_row is None:
        return None

    seq_amb = np.asarray(list(temp_ambient_buffer), dtype=np.float64)
    feat_amb = extract_features_from_sequence(
        seq_amb,
        use_smoothing=USE_SMOOTHING,
        smooth_window=SMOOTH_WINDOW,
        smooth_polyorder=SMOOTH_POLYORDER,
    )
    feat_amb_row = _prepare_feature_row(feat_amb, "ambient")
    if feat_amb_row is None:
        return None

    try:
        feat_int_scaled = SCALER_RF.transform(feat_int_row)
        prob_int = MODEL_RF.predict_proba(feat_int_scaled)[0, 1]

        feat_amb_scaled = SCALER_RF.transform(feat_amb_row)
        prob_amb = MODEL_RF.predict_proba(feat_amb_scaled)[0, 1]
    except Exception as e:
        print(f"Prediction failed: {e}")
        return None

    print("\n=== Prediction result ===")
    print(f"Internal thermal runaway probability: {prob_int:.2%}")
    print(f"Ambient thermal runaway probability: {prob_amb:.2%}")
    print(f"ESP32 status: {esp32_status}")

    ai_is_danger = prob_int >= BEST_THRESHOLD or prob_amb >= BEST_THRESHOLD

    if esp32_status == "DANGER" and not ai_is_danger:
        return "normal"
    if esp32_status in ["NORMAL", "WARNING"] and ai_is_danger:
        return "danger"
    return None


def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)

        ts = data.get("timestamp_ms")
        temp_ambient = data.get("temp_ambient_c")
        temp_internal = data.get("temp_internal_c")
        esp32_status = data.get("status", "UNKNOWN")

        if temp_internal is None or temp_ambient is None:
            return

        temp_int_val = float(temp_internal)
        temp_amb_val = float(temp_ambient)
        if np.isnan(temp_int_val) or np.isnan(temp_amb_val):
            print("Received NaN temperature, ignoring message")
            return

        temp_internal_buffer.append(temp_int_val)
        temp_ambient_buffer.append(temp_amb_val)
        if ts is not None:
            time_buffer.append(ts / 1000.0)

        print(
            f"\nReceived data - internal: {temp_int_val:.2f}°C, "
            f"external: {temp_amb_val:.2f}°C, "
            f"buffer: {len(temp_internal_buffer)}/{SEQ_LEN}"
        )

        if len(temp_internal_buffer) == SEQ_LEN:
            status = process_and_predict(esp32_status)
            if status:
                client.publish(MQTT_STATUS_TOPIC, status)
                print(f"Published corrected status: {status}")
    except Exception as e:
        print(f"Message processing error: {e}")


def start_mqtt_listener():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=MQTT_CLIENT_ID)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_forever()


if __name__ == "__main__":
    start_mqtt_listener()
