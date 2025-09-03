import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Inisialisasi komunikasi I2C
i2c = busio.I2C(SCL, SDA)

# Inisialisasi modul PCA9685
pca = PCA9685(i2c)
pca.frequency = 50  # Frekuensi servo

# Buat objek servo
servo_0 = servo.Servo(pca.channels[0], min_pulse=2500, max_pulse=500)  # MG995 360�
servo_1 = servo.Servo(pca.channels[1], min_pulse=2500, max_pulse=500)  # MG995 180�

try:
    while True:
        print("Servo 0 180")
        servo_0.angle = 180
        time.sleep(1.85)
        print("Servo 0 90")
        servo_0.angle = 90
        time.sleep(5)
        print("Servo 0 0")
        servo_0.angle = 0
        time.sleep(1.95)
        print("Servo 0 90")
        servo_0.angle = 90
        time.sleep(5)

        print("Servo 1 180")
        servo_1.angle = 180
        time.sleep(1.85)
        print("Servo 1 90")
        servo_1.angle = 90
        time.sleep(5)
        print("Servo 1 0")
        servo_1.angle = 0
        time.sleep(1.95)
        print("Servo 1 90")
        servo_1.angle = 90
        time.sleep(5)
except KeyboardInterrupt:
    print("Dihentikan oleh pengguna")

finally:
    pca.deinit()
