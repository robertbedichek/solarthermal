#!/bin/zsh

sudo launchctl bootstrap gui/$(id -u) $HOME/Library/LaunchAgents/com.bedichek.watch_gobble_solarthermal.plist		
ps aux|egrep solarthermal
