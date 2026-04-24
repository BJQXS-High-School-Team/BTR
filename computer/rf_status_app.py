import json
import math
import queue
import threading
from collections import deque
from datetime import datetime

import numpy as np
import paho.mqtt.client as mqtt
import tkinter as tk
import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from tkinter import ttk

from main_RF import (
    BEST_THRESHOLD,
    MODEL_RF,
    MQTT_BROKER,
    MQTT_CLIENT_ID,
    MQTT_PORT,
    MQTT_STATUS_TOPIC,
    MQTT_TOPIC,
    SCALER_RF,
    SEQ_LEN,
    SMOOTH_POLYORDER,
    SMOOTH_WINDOW,
    USE_SMOOTHING,
    extract_features_from_sequence,
)


matplotlib.rcParams["font.sans-serif"] = [
    "Microsoft YaHei",
    "SimHei",
    "Microsoft YaHei UI",
    "Arial Unicode MS",
    "DejaVu Sans",
]
matplotlib.rcParams["axes.unicode_minus"] = False
MAX_POINTS = 180
DEFAULT_SENSOR_THRESHOLDS = {
    "temp_ambient_c": 50.0,
    "temp_internal_c": 50.0,
    "humidity_percent": 80.0,
    "mq2_ppm": 200.0,
    "mq4_ppm": 250.0,
    "mq8_ppm": 450.0,
    "mq7_ppm": 750.0,
    "voc_index": 200.0,
    "max30105_smoke": 750.0,
    "max30105_temp_c": 50.0,
}
LOWER_LIMIT_SENSORS = {"voc_index"}


class RfStatusApp:
    def __init__(self, root):
        self.root = root
        self.root.title("BTR RF 状态显示二合一")
        self.root.geometry("1180x760")
        self.root.minsize(980, 640)

        self.events = queue.Queue()
        self.internal_buffer = deque(maxlen=SEQ_LEN)
        self.ambient_buffer = deque(maxlen=SEQ_LEN)
        self.history = deque(maxlen=MAX_POINTS)
        self.last_prob_internal = 0.0
        self.last_prob_ambient = 0.0
        self.last_corrected_status = "-"
        self.connected = False
        self.sensor_thresholds = DEFAULT_SENSOR_THRESHOLDS.copy()
        if MODEL_RF is not None and hasattr(MODEL_RF, "verbose"):
            MODEL_RF.verbose = 0

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=MQTT_CLIENT_ID)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

        self.build_ui()
        self.start_mqtt()
        self.root.after(200, self.drain_events)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#f4f6f1")
        style.configure("TLabel", background="#f4f6f1", foreground="#17211b")
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 22, "bold"))
        style.configure("Metric.TLabel", font=("Microsoft YaHei UI", 18, "bold"))
        style.configure("AlertCard.TFrame", background="#fff1f0")
        style.configure("AlertName.TLabel", background="#fff1f0", foreground="#b91c1c")
        style.configure("AlertMetric.TLabel", background="#fff1f0", foreground="#b91c1c", font=("Microsoft YaHei UI", 18, "bold"))
        style.configure("Status.TLabel", font=("Microsoft YaHei UI", 30, "bold"))
        style.configure("TButton", font=("Microsoft YaHei UI", 10, "bold"))

        main = ttk.Frame(self.root, padding=18)
        main.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(main)
        header.pack(fill=tk.X)
        ttk.Label(header, text="BTR RF 状态显示二合一", style="Title.TLabel").pack(side=tk.LEFT)
        self.connection_label = ttk.Label(header, text="MQTT 未连接")
        self.connection_label.pack(side=tk.RIGHT)

        status_row = ttk.Frame(main)
        status_row.pack(fill=tk.X, pady=(16, 10))
        self.status_label = ttk.Label(status_row, text="UNKNOWN", style="Status.TLabel")
        self.status_label.pack(side=tk.LEFT)
        self.rf_label = ttk.Label(status_row, text=f"RF 阈值: {BEST_THRESHOLD:.3f}")
        self.rf_label.pack(side=tk.RIGHT)

        metric_frame = ttk.Frame(main)
        metric_frame.pack(fill=tk.X)
        self.metric_vars = {}
        self.metric_widgets = {}
        metrics = [
            ("temp_internal_c", "内部温度", "°C"),
            ("temp_ambient_c", "外部温度", "°C"),
            ("humidity_percent", "湿度", "%"),
            ("mq2_ppm", "MQ-2", "ppm"),
            ("mq4_ppm", "MQ-4", "ppm"),
            ("mq8_ppm", "MQ-8", "ppm"),
            ("mq7_ppm", "MQ-7", "ppm"),
            ("voc_index", "VOC", ""),
            ("max30105_smoke", "烟雾IR", ""),
            ("max30105_temp_c", "MAX温度", "°C"),
        ]
        for idx, (key, name, unit) in enumerate(metrics):
            card = ttk.Frame(metric_frame, padding=10)
            card.grid(row=idx // 5, column=idx % 5, sticky="ew", padx=5, pady=5)
            metric_frame.columnconfigure(idx % 5, weight=1)
            name_label = ttk.Label(card, text=name)
            name_label.pack(anchor=tk.W)
            var = tk.StringVar(value=f"-- {unit}".strip())
            self.metric_vars[key] = (var, unit)
            value_label = ttk.Label(card, textvariable=var, style="Metric.TLabel")
            value_label.pack(anchor=tk.W)
            self.metric_widgets[key] = (card, name_label, value_label)

        self.figure = Figure(figsize=(9, 3.5), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_facecolor("#fffdf7")
        self.ax.grid(True, color="#d8ded5", linewidth=0.8)
        self.canvas = FigureCanvasTkAgg(self.figure, master=main)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(12, 8))

        bottom = ttk.Frame(main)
        bottom.pack(fill=tk.X)
        self.prob_label = ttk.Label(bottom, text="RF 概率: 内部 -- / 外部 --")
        self.prob_label.pack(side=tk.LEFT)
        self.publish_label = ttk.Label(bottom, text="纠正发布: -")
        self.publish_label.pack(side=tk.LEFT, padx=(24, 0))
        ttk.Button(bottom, text="手动 normal", command=lambda: self.publish_status("normal")).pack(side=tk.RIGHT)
        ttk.Button(bottom, text="手动 danger", command=lambda: self.publish_status("danger")).pack(side=tk.RIGHT, padx=8)

    def start_mqtt(self):
        thread = threading.Thread(target=self.mqtt_loop, daemon=True)
        thread.start()

    def mqtt_loop(self):
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_forever()
        except Exception as exc:
            self.events.put(("error", f"MQTT 连接异常: {exc}"))

    def on_connect(self, client, userdata, flags, reason_code, properties):
        self.connected = not reason_code.is_failure
        if self.connected:
            client.subscribe(MQTT_TOPIC)
        self.events.put(("connection", (self.connected, str(reason_code))))

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        self.connected = False
        self.events.put(("connection", (False, str(reason_code))))

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode("utf-8")
            data = json.loads(payload)
            self.events.put(("sample", data))
        except Exception as exc:
            self.events.put(("error", f"消息解析失败: {exc}"))

    def drain_events(self):
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "connection":
                connected, reason = payload
                text = "MQTT 已连接" if connected else f"MQTT 未连接: {reason}"
                self.connection_label.config(text=text)
            elif kind == "sample":
                self.handle_sample(payload)
            elif kind == "error":
                self.connection_label.config(text=payload)
        self.root.after(200, self.drain_events)

    def handle_sample(self, data):
        temp_int = self.to_float(data.get("temp_internal_c"))
        temp_amb = self.to_float(data.get("temp_ambient_c"))
        if temp_int is None or temp_amb is None:
            return

        self.internal_buffer.append(temp_int)
        self.ambient_buffer.append(temp_amb)
        data["_received_at"] = datetime.now()
        self.history.append(data)
        self.update_sensor_thresholds(data.get("thresholds"))

        for key, (var, unit) in self.metric_vars.items():
            value = self.to_float(data.get(key))
            if value is not None:
                decimals = 0 if key.endswith("_ppm") or key in {"voc_index", "max30105_smoke"} else 1
                var.set(f"{value:.{decimals}f} {unit}".strip())
                self.set_metric_alert(key, self.is_sensor_exceeded(key, value))

        esp32_status = data.get("status", "UNKNOWN")
        corrected = self.predict_and_publish(esp32_status)
        self.update_status(esp32_status, corrected)
        self.update_chart()

    def predict_and_publish(self, esp32_status):
        if MODEL_RF is None or SCALER_RF is None:
            return None
        if len(self.internal_buffer) < SEQ_LEN or len(self.ambient_buffer) < SEQ_LEN:
            self.prob_label.config(text=f"RF 概率: 缓冲 {len(self.internal_buffer)}/{SEQ_LEN}")
            return None

        try:
            row_int = self.feature_row(self.internal_buffer)
            row_amb = self.feature_row(self.ambient_buffer)
            self.last_prob_internal = MODEL_RF.predict_proba(SCALER_RF.transform(row_int))[0, 1]
            self.last_prob_ambient = MODEL_RF.predict_proba(SCALER_RF.transform(row_amb))[0, 1]
        except Exception as exc:
            self.connection_label.config(text=f"RF 推理异常: {exc}")
            return None

        self.prob_label.config(
            text=f"RF 概率: 内部 {self.last_prob_internal:.2%} / 外部 {self.last_prob_ambient:.2%}"
        )
        ai_is_danger = self.last_prob_internal >= BEST_THRESHOLD or self.last_prob_ambient >= BEST_THRESHOLD

        corrected = None
        if esp32_status == "DANGER" and not ai_is_danger:
            corrected = "normal"
        elif esp32_status in {"NORMAL", "WARNING"} and ai_is_danger:
            corrected = "danger"

        if corrected:
            self.publish_status(corrected)
        return corrected

    def feature_row(self, values):
        features = extract_features_from_sequence(
            np.asarray(list(values), dtype=np.float64),
            use_smoothing=USE_SMOOTHING,
            smooth_window=SMOOTH_WINDOW,
            smooth_polyorder=SMOOTH_POLYORDER,
        )
        row = np.asarray(features, dtype=np.float64).reshape(1, -1)
        if np.any(np.isnan(row)) or np.any(np.isinf(row)):
            raise ValueError("特征包含 NaN/Inf")
        return row

    def publish_status(self, status):
        try:
            self.client.publish(MQTT_STATUS_TOPIC, status)
            self.last_corrected_status = f"{status} @ {datetime.now().strftime('%H:%M:%S')}"
            self.publish_label.config(text=f"纠正发布: {self.last_corrected_status}")
        except Exception as exc:
            self.connection_label.config(text=f"发布失败: {exc}")

    def update_status(self, esp32_status, corrected):
        text = corrected.upper() if corrected else esp32_status
        self.status_label.config(text=text)
        color = {"NORMAL": "#16803b", "WARNING": "#b7791f", "DANGER": "#b91c1c"}.get(text, "#17211b")
        self.status_label.config(foreground=color)

    def update_sensor_thresholds(self, thresholds):
        if not isinstance(thresholds, dict):
            return
        for key in DEFAULT_SENSOR_THRESHOLDS:
            value = self.to_float(thresholds.get(key))
            if value is not None:
                self.sensor_thresholds[key] = value

    def is_sensor_exceeded(self, key, value):
        threshold = self.sensor_thresholds.get(key)
        if threshold is None:
            return False
        if key in LOWER_LIMIT_SENSORS:
            return value < threshold
        return value > threshold

    def set_metric_alert(self, key, active):
        widgets = self.metric_widgets.get(key)
        if not widgets:
            return
        card, name_label, value_label = widgets
        if active:
            card.configure(style="AlertCard.TFrame")
            name_label.configure(style="AlertName.TLabel")
            value_label.configure(style="AlertMetric.TLabel")
        else:
            card.configure(style="TFrame")
            name_label.configure(style="TLabel")
            value_label.configure(style="Metric.TLabel")

    def update_chart(self):
        self.ax.clear()
        self.ax.set_facecolor("#fffdf7")
        self.ax.grid(True, color="#d8ded5", linewidth=0.8)
        x = list(range(len(self.history)))
        series = [
            ("temp_internal_c", "内部温度", "#b91c1c"),
            ("temp_ambient_c", "外部温度", "#0f766e"),
            ("humidity_percent", "湿度", "#2563eb"),
            ("mq2_ppm", "MQ-2", "#a16207"),
            ("voc_index", "VOC", "#7c3aed"),
        ]
        for key, label, color in series:
            y = [self.to_float(item.get(key)) or math.nan for item in self.history]
            self.ax.plot(x, y, label=label, color=color, linewidth=1.8)
        self.ax.legend(loc="upper left", ncol=5, fontsize=9)
        self.ax.set_title("实时变化曲线")
        self.figure.tight_layout()
        self.canvas.draw_idle()

    @staticmethod
    def to_float(value):
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return None if math.isnan(result) or math.isinf(result) else result

    def on_close(self):
        try:
            self.client.disconnect()
        finally:
            self.root.destroy()


def main():
    root = tk.Tk()
    app = RfStatusApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
