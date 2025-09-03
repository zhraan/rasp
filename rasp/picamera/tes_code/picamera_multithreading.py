import cv2
import numpy as np
import onnxruntime
import time
import threading
import os

# === Konstanta dan variabel global ===
model_path = "/home/pi/Model/rasp/Model manggis EdgeTPU/best.onnx"
class_names = ['super', 'kelas A', 'kelas B', 'luar mutu']
conf_threshold = 0.25
nms_threshold = 0.45
input_size = 224

frame = None
results = []
lock = threading.Lock()

# === Load ONNX model ===
session = onnxruntime.InferenceSession(model_path)
input_name = session.get_inputs()[0].name

# === NMS ===
def non_max_suppression(predictions, conf_thres=0.25, iou_thres=0.45):
    boxes, confidences, class_ids = [], [], []
    for pred in predictions:
        x, y, w, h, obj_conf, *class_confs = pred[:5+len(class_names)]
        cls_id = int(np.argmax(class_confs))
        score = obj_conf * class_confs[cls_id]

        if score > conf_thres:
            x1 = x - w / 2
            y1 = y - h / 2
            x2 = x + w / 2
            y2 = y + h / 2
            boxes.append([x1, y1, x2, y2])
            confidences.append(float(score))
            class_ids.append(cls_id)

    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_thres, iou_thres)
    results = []
    if len(indices) > 0:
        for i in indices.flatten():
            results.append((boxes[i], confidences[i], class_ids[i]))
    return results

# === Thread 1: Baca kamera ===
def capture_thread():
    global frame
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        ret, img = cap.read()
        if not ret:
            continue
        with lock:
            frame = img.copy()
        time.sleep(0.01)

# === Thread 2: Inference ===
def inference_thread():
    global frame, results

    while True:
        img = None
        with lock:
            if frame is not None:
                img = frame.copy()

        if img is not None:
            h, w = img.shape[:2]
            img_resized = cv2.resize(img, (input_size, input_size))
            img_input = img_resized[:, :, ::-1].transpose(2, 0, 1)
            img_input = np.expand_dims(img_input, axis=0).astype(np.float32) / 255.0

            preds = session.run(None, {input_name: img_input})[0][0]
            dets = non_max_suppression(preds, conf_threshold, nms_threshold)

            scaled_results = []
            for (box, conf, cls_id) in dets:
                x1, y1, x2, y2 = box
                x1 = int(x1 * w / input_size)
                y1 = int(y1 * h / input_size)
                x2 = int(x2 * w / input_size)
                y2 = int(y2 * h / input_size)
                scaled_results.append(((x1, y1, x2, y2), conf, cls_id))

            with lock:
                results = scaled_results
        time.sleep(0.05)

# === Tampilan utama ===
def main():
    global frame, results
    prev_time = time.time()
    line_x = 500  # Virtual line

    while True:
        img_disp = None
        with lock:
            if frame is not None:
                img_disp = frame.copy()
                boxes = results.copy()

        if img_disp is not None:
            for (x1, y1, x2, y2), conf, cls_id in boxes:
                label = class_names[cls_id]
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                # Warna berdasarkan kelas
                color = [(0,255,0), (255,255,0), (0,0,255), (128,128,128)][cls_id]
                cv2.rectangle(img_disp, (x1, y1), (x2, y2), color, 2)
                cv2.putText(img_disp, f'{label} {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.circle(img_disp, (cx, cy), 5, color, -1)

                # Eksekusi file servo
                if cx > line_x:
                    if cls_id == 0:
                        os.system("python servo_super.py")
                    elif cls_id == 1:
                        os.system("python servo_a.py")
                    elif cls_id == 2:
                        os.system("python servo_b.py")
                    elif cls_id == 3:
                        os.system("python servo_luar_mutu.py")

            cv2.line(img_disp, (line_x, 0), (line_x, img_disp.shape[0]), (255, 0, 0), 2)
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time)
            prev_time = curr_time
            cv2.putText(img_disp, f'FPS: {fps:.2f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cv2.imshow("Deteksi Manggis", img_disp)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

# === Jalankan Threads ===
if __name__ == '__main__':
    t1 = threading.Thread(target=capture_thread)
    t2 = threading.Thread(target=inference_thread)
    t1.start()
    t2.start()
    main()
    t1.join()
    t2.join()
