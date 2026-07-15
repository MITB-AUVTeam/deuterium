#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1
SESSION="auv"

# Force localhost-only DDS discovery so the Jetson never tries to reach out
# over an interface that may not exist (e.g. laptop not connected).
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION -n 'pico'
tmux new-window -t $SESSION -n 'uart'

# === FIRST RUN ===
echo "First pico flash..."
tmux send-keys -t $SESSION:pico "$SCRIPT_DIR/program_pico.sh" C-m
echo "Waiting 10 seconds..."
sleep 10

echo "Launching first UART bridge..."
tmux send-keys -t $SESSION:uart "cd '$SCRIPT_DIR' && export ROS_LOCALHOST_ONLY=1 && source install/setup.bash && ros2 run pico_UART uart_ros_bridge" C-m
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
echo "Launching surge service..."
tmux new-window -t $SESSION -n 'surge'
tmux send-keys -t $SESSION:surge "cd '$SCRIPT_DIR' && export ROS_LOCALHOST_ONLY=1 && source install/setup.bash && ros2 run ros_controls surge_service" C-m
sleep 2

echo "Launching front camera + detections..."
tmux new-window -t $SESSION -n 'front_cam'
tmux send-keys -t $SESSION:front_cam "cd '$SCRIPT_DIR' && export ROS_LOCALHOST_ONLY=1 && source install/setup.bash && ros2 launch ros_controls front_only.launch.py" C-m
sleep 2

echo "Launching main task BT..."
tmux new-window -t $SESSION -n 'bt'
tmux send-keys -t $SESSION:bt "cd '$SCRIPT_DIR' && export ROS_LOCALHOST_ONLY=1 && source install/setup.bash && ros2 run main_task main_task_node --ros-args --params-file '$SCRIPT_DIR/install/main_task/share/main_task/config/mission_params.yaml'" C-m

echo "All done. Attach with: tmux attach -t $SESSION"
