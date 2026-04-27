import serial

PORT_TGAM = "COM8"  # 6 | 7 8
BAUD_TGAM = 57600

ser = serial.Serial(PORT_TGAM, BAUD_TGAM, timeout=1)
print("Reading TGAM on", PORT_TGAM)

try:
    while True:
        data = ser.read(64)
        if data:
            print(" ".join(f"{b:02X}" for b in data))
except KeyboardInterrupt:
    ser.close()
    print("Closed.")
    