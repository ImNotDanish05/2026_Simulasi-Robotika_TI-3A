#!/usr/bin/env python3
import serial, time, os, datetime

PORT    = "/dev/ttyUSB0"   # cek dengan: ls /dev/ttyUSB* /dev/ttyACM*
BAUD    = 115200
OUT_DIR = "/media/data1/project/kuliah/2026_Simulasi-Robotika_TI-3A/Bab-08-Line_Follower_PID"

os.makedirs(OUT_DIR, exist_ok=True)
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2.5)            # membuka port me-reset ESP32, tunggu boot selesai
ser.reset_input_buffer()
ser.write(b"d")            # minta dump log

lines, recording = [], False
deadline = time.time() + 60
while time.time() < deadline:
    line = ser.readline().decode(errors="ignore").strip()
    if line == "---BEGIN---":
        recording = True
    elif line == "---END---":
        break
    elif recording and line:
        lines.append(line)
else:
    raise SystemExit("Timeout: log tidak diterima (sudah pernah dijalankan?)")

name = datetime.datetime.now().strftime("log_%Y%m%d_%H%M%S.csv")
path = os.path.join(OUT_DIR, name)
with open(path, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"Tersimpan: {path} ({len(lines)-1} baris data)")