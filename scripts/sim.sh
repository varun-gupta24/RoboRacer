#!/bin/bash
# Launch the practice simulator pointed at the bridge and press its Connect button.
# The Connect button sits at window-relative (65, 220) in the default 960x568 window; the
# click uses `cliclick` (brew install cliclick) and needs Accessibility permission for the
# terminal. Without cliclick, click Connect by hand.
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
command -v cliclick >/dev/null && cliclick "c:$((wx + 65)),$((wy + 220))" || echo "cliclick not found: click Connect in the simulator window"
