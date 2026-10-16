#!/usr/bin/env bash
# Run AFTER the simulation is up: checks that the robot's topics exist.
source /opt/ros/humble/setup.bash
for p in /scan /odom /cmd_vel; do
  if ros2 topic list | grep -qx "$p"; then echo "Found topic: $p"; else echo "MISSING topic: $p"; fi
done
