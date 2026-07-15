#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1
SESSION="auv"

tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION -n 'pico'
tmux new-window -t $SESSION -n 'uart'

# === FIRST RUN ===
echo "First pico flash..."
tmux send-keys -t $SESSION:pico "$SCRIPT_DIR/program_pico.sh" C-m
echo "Waiting 10 seconds..."
sleep 10

echo "Launching first UART bridge..."
tmux send-keys -t $SESSION:uart "cd '$SCRIPT_DIR' && source install/setup.bash && ros2 run pico_UART uart_ros_bridge" C-m
echo "Waiting 10 seconds..."
sleep 5

# === SECOND RUN ===
echo "Second pico flash..."
tmux send-keys -t $SESSION:pico "$SCRIPT_DIR/program_pico.sh" C-m
echo "Waiting 10 seconds..."
sleep 5

echo "Launching second UART bridge..."
tmux send-keys -t $SESSION:uart "ros2 run pico_UART uart_ros_bridge" C-m
echo "Waiting 10 seconds..."
sleep 5

# === MAIN STACK ===
echo "Launching prequal..."
tmux new-window -t $SESSION -n 'prequal'
tmux send-keys -t $SESSION:prequal "cd '$SCRIPT_DIR' && source install/setup.bash && ros2 launch ros_controls prequal_full.launch.py" C-m