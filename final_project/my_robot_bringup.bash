#!/bin/bash
# Bringup script for setting up the development environment
echo "Setting up development environment..."
cd ./final_project_ws 
source ./install/setup.bash
echo "Environment setup complete."
ros2 launch my_robot my_robot_map.launch.py