import cv2
import numpy as np
import threading
import queue
import time
from gpiozero import DigitalOutputDevice, DistanceSensor, OutputDevice
from stepper import Stepper
from ultralytics import YOLO
from datetime import datetime
from pymongo import MongoClient
from picamera2 import Picamera2, Preview

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# === MongoDB Setup ===
client = MongoClient("mongodb://admin:z4hr4n523@localhost:27017/")
db = client["sortamanggis_db"]
monitoring_col = db["monitoring_sortasi"]

# === Ultrasonik Setup (gpiozero) ===
ultrasonik = DistanceSensor(echo=5, trigger=6, max_distance=2.0)

# === Relay Setup ===
relay = OutputDevice(16, active_high=False, initial_value=False)

# === Stepper Motor Setup ===
pins = [17, 27, 22, 23]  # BCM pin order
outputs = [DigitalOutputDevice(pin) for pin in pins]

def write(value):
    for i in range(4):
        outputs[i].value = (value >> i) & 1

motor = Stepper(write=write, full_step=True, logic=True, delay=0.0012)

# === Servo0 & Servo1 Setup ===
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50
servo_0 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
servo_1 = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)

servo_0.angle = 0
servo_1.angle = 90

# === YOLO Model ===
model = YOLO("/home/pi/Model/rasp/Model manggis EdgeTPU/best_saved_model/best_full_integer_quant_edgetpu.tflite")
classes = ["kelas A", "kelas B", "luar mutu", "super"]
frame_width, frame_height = 640, 480
line_x = frame_width - 250

# === Kamera ===
picam2 = Picamera2()
camera_config = picam2.create_still_configuration(main={"size": (frame_width, frame_height)})
picam2.configure(camera_config)
picam2.start_preview(Preview.NULL)
picam2.start()

# === Variabel global ===
jumlah_buah = {"kelas_super": 0, "kelas_a": 0, "kelas_b": 0, "rusak": 0}
start_dt = datetime.now()
tanggal_str = start_dt.strftime("%d-%m-%Y")
start_time_str = start_dt.strftime("%H:%M:%S")

frame_lock = threading.Lock()
latest_frame = None
latest_result = None
running = True
servo_queue = queue.Queue()

# === Insert dokumen awal di MongoDB ===
inserted_id = monitoring_col.insert_one({
    "tanggal": tanggal_str,
    "mulai": start_time_str,
    "kelas_super": 0,
    "kelas_a": 0,
    "kelas_b": 0,
    "rusak": 0
}).inserted_id

# === Cooldown Setup ===
last_trigger_time = 0
cooldown = 1.0

def servo_worker():
    while running:
        try:
            kelas_id = servo_queue.get(timeout=0.1)
            print(f"[SORTIR] Kelas {classes[kelas_id]} bergerak.")

            if kelas_id == 2:  # rusak 
                update_kelas_field("rusak")
                servo_1.angle = 45
                time.sleep(7)
                servo_1.angle = 90

            elif kelas_id == 3:  # super 
                update_kelas_field("kelas_super")
                time.sleep(9) 
                # for _ in range(175):
                #     motor.single_step(-1)
                # time.sleep(3)
                # for _ in range(150):
                #     motor.single_step(1)
                for _ in range(375):
                    motor.single_step(-1)
                time.sleep(3)
                for _ in range(210):
                    motor.single_step(1)

            elif kelas_id == 0:  # kelas A 
                update_kelas_field("kelas_a")
                time.sleep(9)
                # for _ in range(230):
                #     motor.single_step(1)
                # time.sleep(3)
                # for _ in range(208):
                #     motor.single_step(-1)
                for _ in range(490):
                    motor.single_step(1)
                time.sleep(3)
                for _ in range(270):
                    motor.single_step(-1)

            elif kelas_id == 1:  # kelas B 
                update_kelas_field("kelas_b")
                time.sleep(9)
                # for _ in range(175):
                #     motor.single_step(1)
                # time.sleep(3)
                # for _ in range(150):
                #     motor.single_step(-1)
                for _ in range(375):
                    motor.single_step(1)
                time.sleep(3)
                for _ in range(210):
                    motor.single_step(-1)

        except queue.Empty:
            continue

def camera_thread():
    global latest_frame
    while running:
        frame = picam2.capture_array()
        with frame_lock:
            latest_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        time.sleep(0.1)

def update_kelas_field(field):
    jumlah_buah[field] += 1
    monitoring_col.update_one(
        {"_id": inserted_id},
        {"$set": {field: jumlah_buah[field]}}
    )

def ultrasonik_thread():
    last_detect_time = 0
    while running:
        jarak_cm = ultrasonik.distance * 100  # konversi ke cm
        # print(f"Jarak: {jarak_cm:.2f} cm")
        if jarak_cm < 10:  # jarak < 10 cm
            last_detect_time = time.time()
            relay.on()
            servo_0.angle = 90   
            time.sleep(3)
            servo_0.angle = 0
        else:
            servo_0.angle = 0  
            if time.time() - last_detect_time > 30:  
                relay.off()
        time.sleep(0.1)

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
            cx = int((x1 + x2) / 2)

            label = f"{classes[int(cls_id)]} {conf:.2f}"
            color = [(255,255,0), (0,165,255), (0,0,255), (0,255,0)][int(cls_id)]
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            if cx > line_x:
                now = time.time()
                if now - last_trigger_time > cooldown:
                    servo_queue.put(int(cls_id))
                    last_trigger_time = now
                break

# === Thread Start ===
camera = threading.Thread(target=camera_thread)
inference = threading.Thread(target=inference_thread)
servo_t = threading.Thread(target=servo_worker)
ultra_t = threading.Thread(target=ultrasonik_thread)

camera.start(); inference.start(); servo_t.start(); ultra_t.start()

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
        cv2.line(frame_to_show, (line_x, 0), (line_x, frame_to_show.shape[0]), (255, 0, 0), 2)
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time
        cv2.putText(frame_to_show, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Sortasi Manggis", frame_to_show)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False
        break

# === Cleanup ===
camera.join(); inference.join(); servo_t.join(); ultra_t.join()
picam2.stop()
cv2.destroyAllWindows()
pca.deinit()
for out in outputs:
    out.off()
relay.off()

# === Simpan MongoDB ===
end_dt = datetime.now()
durasi_str = str(end_dt - start_dt).split('.')[0]
selesai_str = end_dt.strftime("%H:%M:%S")
total = sum(jumlah_buah.values())

monitoring_col.update_one(
    {"_id": inserted_id},
    {"$set": {
        "selesai": selesai_str,
        "durasi": durasi_str,
        "total": total
    }}
)

print("Data sortasi berhasil disimpan ke MongoDB.")