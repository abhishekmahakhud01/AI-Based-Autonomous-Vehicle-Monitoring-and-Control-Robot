#!/usr/bin/env bash
# Alternative: official TurtleBot3 empty world via its own launch file (server + GUI + robot in one command).
set -e
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
