import serial
import time

PORT_TGAM = "COM8"     # TGAM -> PC (6) 7 8
BAUD_TGAM = 57600

PORT_ESP32 = "COM5"    # PC <-> ESP32 USB (5) 7
BAUD_ESP32 = 115200    # must match Serial.begin(...) on ESP32

tgam = serial.Serial(PORT_TGAM, BAUD_TGAM, timeout=0.02)
esp  = serial.Serial(PORT_ESP32, BAUD_ESP32, timeout=0.02)

print(f"Bridge+Monitor: {PORT_TGAM} -> {PORT_ESP32}, and read back CSV")

n = 0
t0 = time.time()

try:
    while True:
        # 1) forward raw TGAM bytes to ESP32
        data = tgam.read(256)
        if data:
            esp.write(data)
            n += len(data)

        # 2) read parsed lines returned by ESP32
        line = esp.readline().decode(errors="ignore").strip()
        if line.startswith("TGAM,"):
            print("[ESP32]", line)
        
        # stats
        if time.time() - t0 >= 1.0:
            print(f"[PC] forwarded {n} bytes/s")
            n = 0
            t0 = time.time()

except KeyboardInterrupt:
    pass
finally:
    tgam.close()
    esp.close()
    print("Closed.")
    