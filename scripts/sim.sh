#!/bin/bash
# Launch the practice simulator pointed at the bridge and press its Connect button.
# In the default 960x568 window the menu toggle is at window-relative (40, 45) and the Connect
# button it reveals at (65, 220). Clicks use `cliclick` (brew install cliclick) and need
# Accessibility permission for the terminal. Without cliclick, click Connect by hand.
#   scripts/sim.sh            # GUI (what the submission video uses)
#   scripts/sim.sh headless   # -batchmode -nographics, auto-connects, no click needed
SIM="$HOME/autodrive-sim/autodrive_simulator/AutoDRIVE Simulator.app/Contents/MacOS/AutoDRIVE Simulator"
pkill -f "MacOS/AutoDRIVE Simulator" 2>/dev/null; sleep 1
if [ "$1" = headless ]; then
  nohup "$SIM" -batchmode -nographics -ip 127.0.0.1 -port 4567 >/dev/null 2>&1 &
  exit 0
fi
nohup "$SIM" -ip 127.0.0.1 -port 4567 >/dev/null 2>&1 &
sleep 12
read wx wy <<<"$(osascript -e 'tell application "System Events" to tell process "AutoDRIVE Simulator" to get position of window 1' | tr -d ',')"
osascript -e 'tell application "System Events" to tell process "AutoDRIVE Simulator" to set frontmost to true'
sleep 1
if command -v cliclick >/dev/null; then
  cliclick "c:$((wx + 480)),$((wy + 10))"; sleep 0.5   # focus (first click only activates the window)
  cliclick "c:$((wx + 40)),$((wy + 45))"; sleep 0.5    # open the menu
  cliclick "c:$((wx + 65)),$((wy + 220))"              # Connect
else
  echo "cliclick not found: click Connect in the simulator window"
fi
