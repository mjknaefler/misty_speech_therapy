#!/bin/bash

# Start Ollama in the background
echo "Starting Ollama service..."
ollama serve &
OLLAMA_PID=$!

# Wait for Ollama to start
sleep 5

# Source ROS 2
source /opt/ros/foxy/setup.bash

# Source virtual environment
source ~/Documents/misty_speech_therapy/misty_venv/bin/activate

# Source workspace
source ~/Documents/misty_speech_therapy/install/setup.bash

# Launch package
echo "Starting ROS 2 nodes..."
ros2 launch misty_speech_therapy speech_therapy.launch.py

# Cleanup when script is terminated
function cleanup {
  echo "Shutting down..."
  kill $OLLAMA_PID
  exit 0
}

trap cleanup SIGINT SIGTERM

# Wait for ROS 2 to finish
wait