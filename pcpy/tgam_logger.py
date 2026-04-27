import serial
import time
import json
from datetime import datetime

PORT = "COM8"   # 7 8
BAUD = 57600
TIMEOUT = 1.0
WRITE_INTERVAL_SEC = 1.0  # 1 second per row in jsonl

def read_packet(ser):
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] != 0xAA:
            continue
        b2 = ser.read(1)
        if not b2 or b2[0] != 0xAA:
            continue

        plen_b = ser.read(1)
        if not plen_b:
            return None
        plen = plen_b[0]
        if plen > 169:
            continue

        payload = ser.read(plen)
        if len(payload) != plen:
            return None

        chk_b = ser.read(1)
        if not chk_b:
            return None

        calc = (~(sum(payload) & 0xFF)) & 0xFF
        if calc == chk_b[0]:
            return payload

def parse_raw_wave(payload: bytes):
    i, out, n = 0, [], len(payload)
    while i < n:
        code = payload[i]
        i += 1

        if code == 0x80 and i < n:
            vlen = payload[i]
            i += 1
            if i + vlen <= n and vlen == 2:
                msb, lsb = payload[i], payload[i + 1]
                i += 2
                val = (msb << 8) | lsb
                if val >= 32768:
                    val -= 65536
                out.append(val)
            else:
                i += vlen
        elif code >= 0x80 and i < n:
            vlen = payload[i]
            i += 1 + vlen
        else:
            if i < n:
                i += 1
    return out

def main():
    start_dt = datetime.now()
    out_name = f"tgam_raw_{start_dt.strftime('%Y%m%d_%H%M%S')}.jsonl"

    sample_count = 0
    second_index = 0
    bucket = []
    window_start = time.time()

    with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser, open(out_name, "w", encoding="utf-8") as f:
        header = {
            "session": {
                "date": start_dt.strftime("%Y-%m-%d"),
                "start_time": start_dt.strftime("%H:%M:%S"),
                "mode": "raw_only_0x80",
                "row_interval_sec": WRITE_INTERVAL_SEC
            }
        }
        f.write(json.dumps(header, separators=(",", ":")) + "\n")
        f.flush()

        print(f"Reading RAW only from {PORT}@{BAUD}. Writing 1 row / {WRITE_INTERVAL_SEC}s. Ctrl+C to stop.")

        try:
            while True:
                payload = read_packet(ser)
                if payload is None:
                    continue

                raw_vals = parse_raw_wave(payload)
                if raw_vals:
                    bucket.extend(raw_vals)
                    sample_count += len(raw_vals)

                now = time.time()
                if now - window_start >= WRITE_INTERVAL_SEC:
                    row = {
                        "sec": second_index,
                        "ts": now,
                        "raw": bucket  # ~512 samples if TGAM raw is 512Hz
                    }
                    f.write(json.dumps(row, separators=(",", ":")) + "\n")
                    f.flush()

                    print(f"sec={second_index}, samples={len(bucket)}")
                    second_index += 1
                    bucket = []
                    window_start = now

        except KeyboardInterrupt:
            pass

        # flush remainder
        if bucket:
            row = {"sec": second_index, "ts": time.time(), "raw": bucket}
            f.write(json.dumps(row, separators=(",", ":")) + "\n")

        end_dt = datetime.now()
        f.write(json.dumps({"session_end": {"end_time": end_dt.strftime("%H:%M:%S"), "raw_samples": sample_count}}, separators=(",", ":")) + "\n")
        f.flush()

    print(f"Saved {out_name} (raw_samples={sample_count})")

if __name__ == "__main__":
    main()
    