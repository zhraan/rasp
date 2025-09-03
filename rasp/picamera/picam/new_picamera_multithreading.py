import cv2
import numpy as np
import threading
from ultralytics import YOLO
import os
from picamera2 import Picamera2, Preview
import time

# Load model YOLO
model_path = "/home/pi/Model/rasp/Model manggis EdgeTPU/best_saved_model/best_full_integer_quant_edgetpu.tflite"
model = YOLO(model_path)

# Label kelas manggis
classes = ["kelas A", "kelas B", "luar mutu", "super"]

# Resolusi kecil agar ringan
frame_width, frame_height = 640, 480
line_x = frame_width - 50  # Sesuaikan ulang karena frame lebih kecil

# Inisialisasi Picamera2
picam2 = Picamera2()
camera_config = picam2.create_still_configuration(
    main={"size": (frame_width, frame_height)}
)
picam2.configure(camera_config)
picam2.start_preview(Preview.NULL)
picam2.start()

# Threading dan sinkronisasi
frame_lock = threading.Lock()
latest_frame = None
latest_result = None
running = True

def draw_boxes(frame, results, conf_threshold=0.5):
    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        cls_ids = result.boxes.cls.cpu().numpy()

        for box, conf, cls_id in zip(boxes, confs, cls_ids):
            if conf >= conf_threshold:
                x1, y1, x2, y2 = map(int, box)
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                label = f"{classes[int(cls_id)]} {conf:.2f}"

                # Warna per kelas
                if cls_id == 0:
                    color = (255, 255, 0)
                elif cls_id == 1:
                    color = (0, 165, 255)
                elif cls_id == 2:
                    color = (0, 0, 255)
                elif cls_id == 3:
                    color = (0, 255, 0)
                else:
                    color = (255, 255, 255)

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.circle(frame, (cx, cy), 5, color, -1)

                if cx > line_x:
                    print(f"Centroid {cx},{cy} dengan label '{classes[int(cls_id)]}' melewati virtual line.")
                    if cls_id == 0:
                        os.system("python servo_super.py")
                    elif cls_id == 1:
                        os.system("python servo_kelasA.py")
                    elif cls_id == 2:
                        os.system("python servo_kelasB.py")
                    elif cls_id == 3:
                        os.system("python servo_luarmutu.py")
                else:
                    print(f"Centroid {cx},{cy} dengan label '{classes[int(cls_id)]}'")

def camera_thread():
    global latest_frame, running
    while running:
        frame = picam2.capture_array()
        with frame_lock:
            latest_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

def inference_thread():
    global latest_frame, latest_result, running
    while running:
        frame_copy = None
        with frame_lock:
            if latest_frame is not None:
                frame_copy = latest_frame.copy()
        if frame_copy is not None:
            result = model.predict(source=frame_copy, imgsz=224, conf=0.5, iou=0.1, save=False, verbose=False)
            with frame_lock:
                latest_result = (frame_copy, result)

# Mulai thread kamera dan inferensi
camera = threading.Thread(target=camera_thread)
inference = threading.Thread(target=inference_thread)
camera.start()
inference.start()

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

        # Gambar garis batas
        cv2.line(frame_to_show, (line_x, 0), (line_x, frame_to_show.shape[0]), (255, 0, 0), 2)

        # Hitung FPS
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time

        cv2.putText(frame_to_show, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Manggis Detection", frame_to_show)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False
        break

# Bersihkan
camera.join()
inference.join()
picam2.stop()
cv2.destroyAllWindows()