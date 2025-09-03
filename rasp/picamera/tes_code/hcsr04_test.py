from gpiozero import DistanceSensor, Servo, OutputDevice
import time

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Inisialisasi sensor HC-SR04 (ganti pin BCM sesuai wiring)
sensor = DistanceSensor(echo=5, trigger=6)
relay = OutputDevice(16, active_high=False, initial_value=False)

# === Servo0 & Servo1 Setup ===
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50
servo_0 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
last_detect_time = 0
try:
    while True:
        jarak_cm = sensor.distance * 100  # konversi ke cm
        print(f"Jarak: {jarak_cm:.2f} cm")
        
        if jarak_cm < 10:
            # last_detect_time = time.time()
            # relay.on()
            # time.sleep(2)
            # servo_0.angle = 90   
            # time.sleep(3)
            # servo_0.angle = 0
            print(last_detect_time) 
        else:
            servo_0.angle = 0
            if time.time() - last_detect_time > 10:  
                relay.off()
        time.sleep(0.1)
            

except KeyboardInterrupt:
    print("Program dihentikan.")
