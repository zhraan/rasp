import cv2 
import numpy as np
from ultralytics import YOLO
import os
from picamera2 import Picamera2
from picamera2 import Preview
from screeninfo import get_monitors
import time

# Path ke model ONNX
model_path = "/home/pi/Model/rasp/Model manggis EdgeTPU/best_saved_model/best_full_integer_quant_edgetpu.tflite"  # Ganti sesuai nama model manggis kamu

# Load model YOLO dengan ONNX
model = YOLO(model_path)

# Kelas untuk manggis
classes = ["kelas A", "kelas B", "luar mutu", "super"]

# Mengambil resolusi layar monitor
# monitor = get_monitors()[0]
# screen_width = monitor.width
# screen_height = monitor.height

# Konfigurasi Virtual Line
frame_width = 640
frame_height = 480
line_x = frame_width - 150

def draw_boxes(frame, results, conf_threshold=0.5, iou_threshold=0.4):
    for result in results:
        boxes_xyxy = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        cls_ids = result.boxes.cls.cpu().numpy()

        for box, conf, cls_id in zip(boxes_xyxy, confs, cls_ids):
            if conf >= conf_threshold:
                x1, y1, x2, y2 = map(int, box)
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                label = f"{classes[int(cls_id)]} {conf:.2f}"

                # Warna berdasarkan kelas
                if cls_id == 0:  # kelas A
                    color = (255, 255, 0)  # Kuning
                elif cls_id == 1:  # kelas B
                    color = (0, 165, 255)  # Orange
                elif cls_id == 2:  # luar mutu
                    color = (0, 0, 255)  # Merah
                elif cls_id == 3:  # super
                    color = (0, 255, 0)  # Hijau
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

# Inisialisasi kamera
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration())
picam2.start_preview(Preview.NULL)
picam2.start()

prev_time = time.time()

print("Tekan 'q' untuk keluar.")

while True:
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (640, 480))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    results = model.predict(source=frame, imgsz=224, conf=0.5, iou=0.1, save=False, verbose=False)

    draw_boxes(frame, results)

    cv2.line(frame, (line_x, 0), (line_x, frame.shape[0]), (255, 0, 0), 2)

    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time

    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Manggis Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
