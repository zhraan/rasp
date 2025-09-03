import cv2
import numpy as np
from picamera2 import Picamera2, Preview
from screeninfo import get_monitors
import time

from pycoral.adapters import common
from pycoral.adapters import detect
from pycoral.utils.edgetpu import make_interpreter

# Path ke model tflite EdgeTPU
model_path = "/home/pi/Model/rasp/Model manggis EdgeTPU/best_full_integer_quant_edgetpu.tflite"

# Label kelas sesuai model
classes = ["super", "kelas A", "kelas B", "luar mutu"]

# Inisialisasi interpreter EdgeTPU
interpreter = make_interpreter(model_path)
interpreter.allocate_tensors()

# Ambil ukuran input model
input_details = interpreter.get_input_details()
input_shape = input_details[0]['shape']
input_height = input_shape[1]
input_width = input_shape[2]

# Inisialisasi kamera
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration(main={"format": "RGB888", "size": (input_width, input_height)}))
picam2.start_preview(Preview.NULL)
picam2.start()

# Virtual line (contoh posisi di frame)
line_x = input_width - 50

def draw_objects(frame, objs, classes):
    for obj in objs:
        bbox = obj.bbox
        class_id = obj.id
        score = obj.score

        x1, y1, x2, y2 = bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

        # Warna berdasarkan kelas
        if class_id == 0:
            color = (0, 255, 0)  # Hijau
        elif class_id == 1:
            color = (255, 255, 0)  # Kuning
        elif class_id == 2:
            color = (0, 165, 255)  # Orange
        elif class_id == 3:
            color = (0, 0, 255)  # Merah
        else:
            color = (255, 255, 255)

        label = f"{classes[class_id]} {score:.2f}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.circle(frame, (cx, cy), 5, color, -1)

        if cx > line_x:
            print(f"Centroid {cx},{cy} dengan label '{classes[class_id]}' melewati virtual line.")
            # Panggil script sesuai kelas
            if class_id == 0:
                os.system("python servo_super.py")
            elif class_id == 1:
                os.system("python servo_kelasA.py")
            elif class_id == 2:
                os.system("python servo_kelasB.py")
            elif class_id == 3:
                os.system("python servo_luarmutu.py")
        else:
            print(f"Centroid {cx},{cy} dengan label '{classes[class_id]}'")

print("Tekan 'q' untuk keluar.")

prev_time = time.time()

while True:
    frame = picam2.capture_array()
    # Resize frame ke input model (224x224) dan format RGB sudah diatur konfigurasi kamera
    frame_resized = cv2.resize(frame, (input_width, input_height))
    input_data = np.asarray(frame_resized)

    # Set input tensor ke interpreter
    common.set_input(interpreter, input_data)

    # Jalankan inferensi
    interpreter.invoke()

    # Ambil hasil deteksi (threshold confidence 0.5, IOU 0.4)
    objs = detect.get_objects(interpreter, score_threshold=0.5, iou_threshold=0.4)

    # Gambar bounding box dan label
    draw_objects(frame_resized, objs, classes)

    # Gambar garis virtual
    cv2.line(frame_resized, (line_x, 0), (line_x, input_height), (255, 0, 0), 2)

    # Hitung FPS
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time
    cv2.putText(frame_resized, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Manggis Detection with EdgeTPU", frame_resized)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
