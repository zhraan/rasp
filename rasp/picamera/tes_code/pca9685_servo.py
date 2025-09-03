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

# Buat objek servo untuk channel 0, 1, dan 2
servo_0 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
servo_1 = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)
# servo_2 = servo.Servo(pca.channels[2], min_pulse=500, max_pulse=2500)
servo_1.angle = 90  # netral

# Fungsi utama
try:
    while True:
        # print("Servo 0 ke 0")
        # servo_0.angle = 0
        # time.sleep(0.5)

        print("Servo 0 ke 90")
        servo_0.angle = 90
        time.sleep(2)

        print("Servo 0 ke 0")
        servo_0.angle = 0
        time.sleep(2)
        
        print("Servo 1 ke 45")
        servo_1.angle = 45
        time.sleep(2)

        print("Servo 1 ke 0")
        servo_1.angle = 90
        time.sleep(2)       

        # print("Servo 0 ke 45")
        # servo_2.angle = 45
        # time.sleep(1)

        # print("Servo 0 ke 0")
        # servo_2.angle = 0
        # time.sleep(1)

        # print("Servo 0 ke 0")
        # servo_2.angle = -135
        # time.sleep(1)

        # print("Servo 0 ke 0")
        # servo_2.angle = 0
        # time.sleep(1)
        
        # print("Servo 2 ke 45")
        # servo_2.angle = 45
        # time.sleep(3)
        # print("Servo 2 ke 0")
        # servo_2.angle = 0
        # time.sleep(2)  # jeda
        # print("Servo 2 ke 0")
        # servo_2.angle = 90
        # time.sleep(3)
        # print("Servo 2 ke -45")
        # servo_2.angle = 45
        # time.sleep(2)  # jeda

except KeyboardInterrupt:
    print("Dihentikan oleh pengguna")

finally:
    pca.deinit()