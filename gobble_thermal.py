#!/usr/bin/env python3

import serial
import time

port = "/dev/tty.usbserial-11240"   # Replace with your actual port
baud_rate = 115200

ser = serial.Serial(port, baud_rate, timeout=1)

# Reset Arduino (toggle DTR)
ser.setDTR(False)
time.sleep(1)
ser.setDTR(True)
linecount = 0

print(f"Connected to {port}. Reading data...\n")

with open("solarthermal_log.txt", "a", buffering=1) as f:
  try:
    while True:
      line = ser.readline().decode('utf-8').strip()
      if line:
        f.write(line + "\n")
        f.flush()
        linecount += 1

  except KeyboardInterrupt:
    print(f"Stopping after reading and logging {linecount} lines.")
    ser.close()
