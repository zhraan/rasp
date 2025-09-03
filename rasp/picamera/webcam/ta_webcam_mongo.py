import cv2
import numpy as np
import threading
import queue
import time
from ultralytics import YOLO
from pymongo import MongoClient
from datetime import datetime

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# === MongoDB Setup ===
client = MongoClient("mongodb://localhost:27017/")
db = client["sortamanggis_db"]
monitoring_col = db["monitoring_sortasi"]

# === Inisialisasi Servo ===
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50
servo_0 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
servo_1 = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)
servo_2 = servo.Servo(pca.channels[2], min_pulse=500, max_pulse=2500)
servo_2.angle = 90  # netral

# === YOLO Model ===
model = YOLO("/home/pi/Model/rasp/Model manggis EdgeTPU/best_saved_model/best_full_integer_quant_edgetpu.tflite")
classes = ["kelas A", "kelas B", "luar mutu", "super"]

# === Hitungan Buah Tiap Kelas ===
jumlah_buah = {
    "kelas_super": 0,
    "kelas_a": 0,
    "kelas_b": 0,
    "rusak": 0
}
start_dt = datetime.now()
start_time_str = start_dt.strftime("%H:%M:%S")
tanggal_str = start_dt.strftime("%Y-%m-%d")

frame_width, frame_height = 640, 480
line_x = frame_width - 250

# === Kamera USB Webcam ===
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

# === Thread & Queue ===
frame_lock = threading.Lock()
latest_frame = None
latest_result = None
running = True
servo_queue = queue.Queue()

# === Cooldown Setup ===
last_trigger_time = 0
cooldown = 2.0  # detik

def servo_worker():
    while running:
        try:
            kelas_id = servo_queue.get(timeout=1)
            print(f"[SERVO] Kelas {classes[kelas_id]} bergerak.")

            # Simpan hitungan kelas
            if kelas_id == 0:
                jumlah_buah["kelas_a"] += 1
                servo_2.angle = 90
            elif kelas_id == 1:
                jumlah_buah["kelas_b"] += 1
                servo_2.angle = 45
                time.sleep(20)
                servo_2.angle = 90
            elif kelas_id == 2:
                jumlah_buah["rusak"] += 1
                servo_1.angle = 45
                time.sleep(8)
                servo_1.angle = 0
            elif kelas_id == 3:
                jumlah_buah["kelas_super"] += 1
                servo_2.angle = 135
                time.sleep(15)
                servo_2.angle = 90
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
    global last_trigger_time

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
            color = [(255,255,0), (0,165,255), (0,0,255), (0,255,0)][int(cls_id)]
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.circle(frame, (cx, cy), 5, color, -1)

            if cx > line_x:
                now = time.time()
                if now - last_trigger_time > cooldown:
                    print(f"[DETECTED] {classes[int(cls_id)]} melewati garis.")
                    servo_queue.put(int(cls_id))
                    last_trigger_time = now
                break

# === Mulai Thread ===
camera = threading.Thread(target=camera_thread)
inference = threading.Thread(target=inference_thread)
servo_kelas_thread = threading.Thread(target=servo_worker)

camera.start()
inference.start()
servo_kelas_thread.start()

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
        frame_to_show = cv2.cvtColor(frame_to_show, cv2.COLOR_RGB2BGR)
        draw_boxes(frame_to_show, results)
        cv2.line(frame_to_show, (line_x, 0), (line_x, frame_to_show.shape[0]), (255, 0, 0), 2)

        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time

        cv2.putText(frame_to_show, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Manggis Detection", frame_to_show)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False
        break

# === Simpan Data ke MongoDB ===
camera.join()
inference.join()
servo_kelas_thread.join()
cap.release()
cv2.destroyAllWindows()
pca.deinit()

end_dt = datetime.now()
selesai_str = end_dt.strftime("%H:%M:%S")
durasi = end_dt - start_dt
durasi_str = str(durasi).split('.')[0]

total = sum(jumlah_buah.values())

monitoring_col.insert_one({
    "tanggal": tanggal_str,
    "mulai": start_time_str,
    "selesai": selesai_str,
    "durasi": durasi_str,
    "kelas_super": jumlah_buah["kelas_super"],
    "kelas_a": jumlah_buah["kelas_a"],
    "kelas_b": jumlah_buah["kelas_b"],
    "rusak": jumlah_buah["rusak"],
    "total": total
})

print("✅ Data sortasi berhasil disimpan ke MongoDB.")
