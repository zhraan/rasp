from gpiozero import OutputDevice

relay = OutputDevice(16, active_high=False, initial_value=False)

while True:
    try:
        cmd = input("Masukkan 1 untuk ON, 0 untuk OFF: ")

        if cmd == "1":
            relay.on()
            print("Relay AKTIF")
            print(relay.value)
        elif cmd == "0":
            relay.off()
            print("Relay NONAKTIF")
            print(relay.value)
        else:
            print("Input tidak valid, masukkan 1 atau 0.")
    except KeyboardInterrupt:
        print("\nProgram dihentikan.")
        relay.off()
        break