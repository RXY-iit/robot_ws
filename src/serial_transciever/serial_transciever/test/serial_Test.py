import serial
import struct

# 接続デバイス名は自分の環境に合わせて書き換える
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

while True:
    if ser.in_waiting >= 12:
        raw = ser.read(12)
        print(f"RAW: {raw}")
        try:
            a1, a2, p = struct.unpack('fff', raw)
            print(f"角度1: {a1:.2f}, 角度2: {a2:.2f}, 気圧: {p:.2f}")
        except Exception as e:
            print("unpack error:", e)
    else:
        print("受信待機中...")
