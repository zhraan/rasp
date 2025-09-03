#!/bin/bash

# Aktifkan virtual environment
source /home/pi/Model/rasp/venv/bin/activate

# Jalankan Flask server di background, simpan PID-nya
/home/pi/Model/rasp/venv/bin/python /home/pi/Model/rasp/picamera/server.py &
SERVER_PID=$!
echo "Server Flask berjalan di PID $SERVER_PID"

# Tunggu server siap
sleep 3

# Jalankan script penyortiran
/home/pi/Model/rasp/venv/bin/python /home/pi/Model/rasp/picamera/picam/ta_picamera_stepper.py &

# Jalankan Chromium dan tunggu sampai ditutup
chromium-browser --new-window http://127.0.0.1:5000
CHROMIUM_STATUS=$?

# Setelah browser ditutup, matikan server.py
echo "Tab browser ditutup. Menghentikan server Flask..."
kill $SERVER_PID
wait $SERVER_PID

# Hentikan service systemd
exit 0