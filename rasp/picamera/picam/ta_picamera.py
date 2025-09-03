import cv2
import numpy as np
import threading
import queue
import time
from ultralytics import YOLO
from picamera2 import Picamera2, Preview

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# === Inisialisasi Servo ===
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50
servo_0 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
servo_1 = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)
servo_2 = servo.Servo(pca.channels[2], min_pulse=500, max_pulse=2500)
servo_2.angle = 90  # netral

# === YOLO Mod/home/pi/Model/rasp/Model manggis EdgeTPU/best_saved_model/best_full_integer_quant_edgetpu.logel ===
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

# === Thread & Queue ===
frame_lock = threading.Lock()
latest_frame = None
latest_result = None
running = True
servo_queue = queue.Queue()

# === Servo0 Berjalan Simultan ===
# def servo0_worker():
    # while running:
        # print("[SERVO_0] Bergerak otomatis.")
        # servo_2.angle = 90
        # time.sleep(3)
        # servo_2.angle = 135
        # time.sleep(2)  # jeda
        # servo_2.angle = 90
        # time.sleep(3)
        # servo_2.angle = 45
        # time.sleep(2)  # jeda

# === Cooldown Setup ===
last_trigger_time = 0
cooldown = 1.0  # detik

def servo_worker():
    while running:
        try:
            kelas_id = servo_queue.get(timeout=0.1)
            print(f"[SERVO] Kelas {classes[kelas_id]} bergerak.")

            if kelas_id == 2:  # luar mutu
                servo_1.angle = 45
                time.sleep(5)
                servo_1.angle = 0

            elif kelas_id == 3:  # super
                servo_2.angle = 135  # 45 derajat CCW dari netral
                time.sleep(10)
                servo_2.angle = 90

            elif kelas_id == 0:  # kelas A
                servo_2.angle = 90

            elif kelas_id == 1:  # kelas B
                servo_2.angle = 45  # 45 derajat CW dari netral
                time.sleep(10)
                servo_2.angle = 90
        except queue.Empty:
            continue

def camera_thread():
    global latest_frame
    while running:
        frame = picam2.capture_array()
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
                break  # hanya satu pemicu per frame


# === Mulai Thread ===
camera = threading.Thread(target=camera_thread)
inference = threading.Thread(target=inference_thread)
servo_kelas_thread = threading.Thread(target=servo_worker)
# servo0_thread = threading.Thread(target=servo0_worker)

camera.start()
inference.start()
servo_kelas_thread.start()
# servo0_thread.start()

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
        cv2.imshow("Manggis Detection", frame_to_show)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False
        break

# === Bersihkan ===
camera.join()
inference.join()
servo_kelas_thread.join()
# servo0_thread.join()
picam2.stop()
cv2.destroyAllWindows()
pca.deinit()