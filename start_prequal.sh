#!/bin/bash

# Resolve repo root relative to this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

echo "first pico flash"
./program_pico.sh

echo "waiting 5 seconds"
sleep 5

echo "launching first UART bridge terminal"
gnome-terminal -- bash -c "
cd '$SCRIPT_DIR'
source install/setup.bash
ros2 run pico_UART uart_ros_bridge
exec bash
"

echo "waiting 5 seconds"
sleep 5

echo "second pico flash"
./program_pico.sh

echo "waiting 5 seconds"
sleep 5

echo "launching second UART bridge terminal"
gnome-terminal -- bash -c "
cd '$SCRIPT_DIR'
source install/setup.bash
ros2 run pico_UART uart_ros_bridge
exec bash
"
echo "waiting 5 seconds"
sleep 5

echo "launching prequal"
gnome-terminal -- bash -c "
cd '$SCRIPT_DIR'
source install/setup.bash
ros2 launch ros_controls prequal_full.launch.py
exec bash
"
