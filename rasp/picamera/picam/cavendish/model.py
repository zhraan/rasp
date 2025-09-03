import cv2
import numpy as np
import threading
import queue
import time
from ultralytics import YOLO

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# === Inisialisasi Servo ===
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50
servo_0 = servo.Servo(pca.channels[0], min_pulse=2500, max_pulse=500)
servo_1 = servo.Servo(pca.channels[1], min_pulse=2500, max_pulse=500)

# === Model dan Kelas ===
model = YOLO("/home/pi/Model/rasp/Model Pisang Cavendish/best_saved_model/best_full_integer_quant_edgetpu.tflite")
classes = ["terlalu-matang", "matang", "mentah"]

# === Webcam USB (ubah index 0 jika perlu) ===
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# === Thread & Queue ===
frame_lock = threading.Lock()
latest_frame = None
latest_result = None
running = True
servo_queue = queue.Queue()

def servo_worker():
    last_movement_time = 0
    cooldown_duration = 3  # detik

    while running:
        now = time.time()
        if now - last_movement_time < cooldown_duration:
            time.sleep(10)
            continue

        try:
            kelas_id = servo_queue.get(timeout=0.1)
            print(f"[SERVO] Deteksi kelas: {classes[kelas_id]}")
            
            if kelas_id == 2:  # matang
                servo_0.angle = 45
                time.sleep(10)
                servo_0.angle = 0

            # elif kelas_id == 2:  # mentah
            #     servo_1.angle = 45
            #     time.sleep(10)
            #     servo_1.angle = 0

            last_movement_time = time.time()

        except queue.Empty:
            continue

def camera_thread():
    global latest_frame
    while running:
        ret, frame = cap.read()
        if not ret:
            continue
        with frame_lock:
            latest_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

def inference_thread():
    global latest_frame, latest_result
    while running:
        frame_copy = None
        with frame_lock:
            if latest_frame is not None:
                frame_copy = latest_frame.copy()
        if frame_copy is not None:
            result = model.predict(source=frame_copy, imgsz=224, conf=0.5, iou=0.1, save=False, verbose=False)
            with frame_lock:
                latest_result = (frame_copy, result)

def draw_boxes(frame, results, conf_threshold=0.5):
    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        cls_ids = result.boxes.cls.cpu().numpy()

        for box, conf, cls_id in zip(boxes, confs, cls_ids):
            if conf < conf_threshold:
                continue

            x1, y1, x2, y2 = map(int, box)
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

            label = f"{classes[int(cls_id)]} {conf:.2f}"
            color = [(255,255,0), (0,165,255), (0,0,255), (0,255,0)][int(cls_id) % 4]
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.circle(frame, (cx, cy), 5, color, -1)

            servo_queue.put(int(cls_id))  # langsung kirim ke servo
            break  # hanya satu deteksi per frame

# === Mulai Thread ===
camera = threading.Thread(target=camera_thread)
inference = threading.Thread(target=inference_thread)
servo_thread = threading.Thread(target=servo_worker)

camera.start()
inference.start()
servo_thread.start()

print("Tekan 'q' untuk keluar.")
prev_time = time.time()

while True:
    frame_result = None
    with frame_lock:
        if latest_result:
            frame_result = latest_result
            latest_result = None

    if frame_result:
        frame_to_show, results = frame_result
        draw_boxes(frame_to_show, results)

        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time

        cv2.putText(frame_to_show, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Deteksi Buah", cv2.cvtColor(frame_to_show, cv2.COLOR_RGB2BGR))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False
        break

# === Bersihkan ===
camera.join()
inference.join()
servo_thread.join()
cap.release()
cv2.destroyAllWindows()
pca.deinit()