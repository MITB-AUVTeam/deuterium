#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1
SESSION="test"
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION -n 'pico'
tmux new-window -t $SESSION -n 'uart'

echo "First pico flash..."
tmux send-keys -t $SESSION:pico "$SCRIPT_DIR/program_pico.sh" C-m
sleep 10

echo "First UART bridge..."
tmux send-keys -t $SESSION:uart "cd '$SCRIPT_DIR' && source install/setup.bash && ros2 run pico_UART uart_ros_bridge" C-m
sleep 10

echo "Second pico flash..."
tmux send-keys -t $SESSION:pico "$SCRIPT_DIR/program_pico.sh" C-m
sleep 10

echo "Second UART bridge..."
tmux send-keys -t $SESSION:uart "ros2 run pico_UART uart_ros_bridge" C-m
sleep 10

# echo "Third pico flash..."
# tmux send-keys -t $SESSION:pico "$SCRIPT_DIR/program_pico.sh" C-m