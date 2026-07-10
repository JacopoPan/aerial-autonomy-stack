#!/bin/bash

# Analyze an aircraft's rosbag using PlotJuggler
# To use this scrip, enable RECORD_ROSBAG in 'aircraft.yml.erb'

TMUX_PANE="logging.0" # The tmux <window_name>.<pane_index> where the recording is happening
BAG_PARENT_DIR="/aas/rosbags" # The directory where bags are saved

echo "Stopping ros2 bag record in tmux pane: $TMUX_PANE..."
tmux send-keys -t "$TMUX_PANE" C-c

echo "Waiting for bag file to write metadata..."
sleep 2

# Find the most recently created bag directory
LATEST_BAG_DIR=$(ls -t "$BAG_PARENT_DIR" | head -n 1)
FULL_BAG_PATH="${BAG_PARENT_DIR}/${LATEST_BAG_DIR}"

if [ -d "$FULL_BAG_PATH" ]; then
    echo "Launching PlotJuggler with bag: $FULL_BAG_PATH"
    ros2 run plotjuggler plotjuggler "$FULL_BAG_PATH"
else
    echo "Error: No bag file found in $BAG_PARENT_DIR"
    exit 1
fi
