from gpiozero import DigitalOutputDevice
import time

# Definisi pin (BCM)
rst = DigitalOutputDevice(17)   # reset
slp = DigitalOutputDevice(27)   # sleep
step = DigitalOutputDevice(22)  # step
direction = DigitalOutputDevice(23)  # dir

# Fungsi untuk enable driver
def enable_driver():
    rst.on()
    slp.on()

# Fungsi untuk disable driver (motor bebas berputar)
def disable_driver():
    rst.off()
    slp.off()

# Fungsi untuk gerakkan motor
def step_motor(steps, dir_cw=True, delay=0.001):
    if dir_cw:
        direction.on()   # CW
    else:
        direction.off()  # CCW

    for _ in range(steps):
        step.on()
        time.sleep(delay)
        step.off()
        time.sleep(delay)

# Aktifkan driver
enable_driver()

try:
    while True:
        cmd = input("Input (1=135 CW, 2=180 CW, 3=135 CCW, q=quit): ").strip()

        if cmd == '1':
            print("135 derajat CW")
            step_motor(200, dir_cw=True)  # hitung sesuai microstep
            time.sleep(2)
            step_motor(200, dir_cw=False)

        elif cmd == '2':
            print("180 derajat CW")
            step_motor(100, dir_cw=True)
            time.sleep(2)
            step_motor(100, dir_cw=False)

        elif cmd == '3':
            print("135 derajat CCW")
            step_motor(75, dir_cw=False)
            time.sleep(2)
            step_motor(75, dir_cw=True)

        elif cmd == 'q':
            print("Keluar")
            break

        else:
            print("Input tidak dikenali.")

except KeyboardInterrupt:
    print("Dihentikan oleh pengguna")

finally:
    disable_driver()
    print("Driver dimatikan, motor bebas berputar.")