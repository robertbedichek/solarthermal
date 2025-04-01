#!/usr/bin/env python3

# This attempt to supress the (harmless) warning about OpenSSL does not work, but leaving it here nonetheles
import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="urllib3")

import serial
import time
import subprocess
import os
import requests

home_dir = os.path.expanduser("~")
app_token_path = os.path.join(home_dir, ".pushover", "ap_token.txt")
user_token_path = os.path.join(home_dir, ".pushover", "user_token.txt")

with open(app_token_path, "r") as f:
    app_token = f.read().strip()

with open(user_token_path, "r") as f:
    user_token = f.read().strip()

SCP_INTERVAL = 1800 # Seconds
SCP_COMMAND = [
    "/usr/bin/env", "scp",
    "-i", "/Users/robertbedichek/.ssh/id_rsa",
    "/Users/robertbedichek/log/solarthermal.txt",
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

# We remember the last time we sent the data file to the web server
last_scp_time = time.time()

def is_valid_data_line(line):
    if line.startswith("#"):
        return True  # Comment line is always OK

    parts = line.strip().split()
    if len(parts) != 16:
        return False  # Must have exactly 16 fields

    try:
        # Try to parse the timestamp fields
        date_part, time_part = parts[0], parts[1]
        from datetime import datetime
        datetime.strptime(f"{date_part} {time_part}", "%Y-%m-%d %H:%M:%S")

        # Try to parse all 13 remaining fields as integers
        for val in parts[2:]:
            int(val)

        return True
    except Exception:
        return False

# Read and validate lines
# with open("solar_data.txt") as f:
#     for i, line in enumerate(f, start=1):
#         if not is_valid_data_line(line):
#             print(f"Malformed line {i}: {line.strip()}")


first_time = True

print(f"Connected to {port}. Reading data...\n")

with open("/Users/robertbedichek/log/solarthermal.txt", "a", buffering=1) as f:
  try:
    while True:
      try:
        line = ser.readline().decode('utf-8').strip() 
      except UnicodeDecodeError:
        print("⚠️  Corrupted data skipped:", line)
        line = ""
      if line and is_valid_data_line(line):
        f.write(line + "\n")
        f.flush()
        linecount += 1
        if "alert" in line.lower():
          data = {
              "token": app_token,        # This App's Pushover app token
              "user": user_token,        # My personal user key
              "message": line 
          }

          response = requests.post("https://api.pushover.net/1/messages.json", data=data)

          # Optional: log result
          print(f"Status: {response.status_code}")
          print(f"Response: {response.text}")

# Check if an hour has passed
        current_time = time.time()
        if first_time or (current_time - last_scp_time >= SCP_INTERVAL):
          first_time = False

          print("Running scp ...")
          try:
            subprocess.run(SCP_COMMAND, check=True)
            print("ssh finished.")
          except subprocess.CalledProcessError as e:
            print(f"ssh error: {e}")

          last_scp_time = current_time

  except KeyboardInterrupt:
    print(f"Stopping after reading and logging {linecount} lines.")
    ser.close()
