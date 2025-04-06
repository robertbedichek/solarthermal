#!/bin/zsh

# Unload the watcher safely using bootout
launchctl bootout gui/$(id -u) ~/Library/LaunchAgents/com.bedichek.watch_serial.plist

open -a "Arduino IDE"

# Wait until it exits
echo "⏳ Waiting for Arduino IDE to quit..."

while pgrep -x "Arduino IDE" >/dev/null; do
    sleep 2
done

echo "✅ Arduino IDE has quit. Restarting watcher..."

# Restart watcher
launchctl bootstrap gui/$(id -u) ~/Library/LaunchAgents/com.bedichek.watch_serial.plist
