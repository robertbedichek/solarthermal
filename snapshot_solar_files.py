#!/usr/bin/env python3

import os
import shutil
from datetime import datetime, timedelta

# File paths
base_dir = '/var/www/home'
history_dir = os.path.join(base_dir, 'history')
files = ['solarthermal.txt', 'solartracker.txt']

# Get today's date for directory naming
today = datetime.now()
snapshot_dir_name = today.strftime('%d-%m-%Y')  # e.g., 07-04-2025
snapshot_dir_path = os.path.join(history_dir, snapshot_dir_name)

# Create the snapshot directory if it doesn't exist
os.makedirs(snapshot_dir_path, exist_ok=True)

# Calculate the earliest date to keep (2 days ago)
cutoff_date = (today - timedelta(days=2)).date()

# Process each file
for filename in files:
    full_path = os.path.join(base_dir, filename)
    snapshot_path = os.path.join(snapshot_dir_path, filename)

    # Make a copy for the snapshot
    shutil.copy2(full_path, snapshot_path)

    # Trim the original file
    with open(full_path, 'r') as f:
        lines = f.readlines()

    with open(full_path, 'w') as f:
        for line in lines:
            if line.startswith('#'):
                f.write(line)  # Keep comment lines
            else:
                try:
                    line_date = datetime.strptime(line[:10], '%Y-%m-%d').date()
                    if line_date >= cutoff_date:
                        f.write(line)
                except ValueError:
                    # If the line doesn't start with a valid date, skip it
                    continue

