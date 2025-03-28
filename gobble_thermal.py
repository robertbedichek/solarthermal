#!/usr/bin/env python3

import serial
import time
import subprocess

GNUPLOT_SCRIPT = "solarthermal.gnuplot"
GNUPLOT_INTERVAL = 1800  # seconds

SCP_COMMAND = [
    "/usr/bin/env", "scp",
    "-i", "/Users/robertbedichek/.ssh/id_rsa",
    "/Users/robertbedichek/Documents/Arduino/solarthermal/solarthermal.png",
    "root@bedichek.org:/var/www/home"
]

port = "/dev/tty.usbserial-11240"   # Replace with your actual port
baud_rate = 115200

ser = serial.Serial(port, baud_rate, timeout=1)

# Reset Arduino (toggle DTR)
ser.setDTR(False)
time.sleep(1)
ser.setDTR(True)
linecount = 0

# Track when we last ran gnuplot
last_gnuplot_time = time.time()

first_time = True

print(f"Connected to {port}. Reading data...\n")

with open("solarthermal_log.txt", "a", buffering=1) as f:
  try:
    while True:
      line = ser.readline().decode('utf-8').strip()
      if line:
        f.write(line + "\n")
        f.flush()
        linecount += 1
# Check if an hour has passed
        current_time = time.time()
        if first_time or (current_time - last_gnuplot_time >= GNUPLOT_INTERVAL):
          first_time = False
          print("Running gnuplot...")
          try:
            subprocess.run(["/opt/homebrew/bin/gnuplot", GNUPLOT_SCRIPT], check=True)
            print("gnuplot finished.")
          except subprocess.CalledProcessError as e:
            print(f"gnuplot error: {e}")

          print("Running scp ...")
          try:
            subprocess.run(SCP_COMMAND, check=True)
            print("ssh finished.")
          except subprocess.CalledProcessError as e:
            print(f"ssh error: {e}")

          last_gnuplot_time = current_time

  except KeyboardInterrupt:
    print(f"Stopping after reading and logging {linecount} lines.")
    ser.close()
