#!/usr/bin/env bash
# Starts everything: 3D Gazebo world, robot, obstacle-avoidance controller, web dashboard.
# Usage:  ./start_project.sh            -> world with boxes/walls (default, best for demo)
#         ./start_project.sh empty      -> empty TurtleBot3 world
#         ./start_project.sh nogui      -> no 3D window (much faster on a slow VM; use the dashboard)
set -euo pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_MODEL="/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"

WORLD="$PROJECT_DIR/worlds/obstacle_world.world"
SHOW_GUI=1
for arg in "$@"; do
  case "$arg" in
    empty) WORLD="/opt/ros/humble/share/turtlebot3_gazebo/worlds/empty_world.world" ;;
    nogui) SHOW_GUI=0 ;;
  esac
done

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS 2 Humble not found at $ROS_SETUP (Ubuntu 22.04 + ROS 2 Humble required)."; exit 1
fi
set +u; source "$ROS_SETUP"; set -u
export TURTLEBOT3_MODEL=burger

for cmd in gnome-terminal ros2; do
  command -v "$cmd" >/dev/null 2>&1 || { echo "ERROR: '$cmd' is missing."; exit 1; }
done
if [[ ! -f "$WORLD" || ! -f "$ROBOT_MODEL" ]]; then
  echo "ERROR: world or robot model not found. Install: sudo apt install ros-humble-turtlebot3-gazebo"; exit 1
fi
python3 -c 'import flask' >/dev/null 2>&1 || {
  echo "Flask missing. Install: sudo apt install python3-flask"; exit 1; }

echo "World: $WORLD"
echo "Dashboard will be at http://127.0.0.1:5000"

gnome-terminal --title="Robot - Gazebo Server" -- bash -lc \
  "source '$ROS_SETUP'; export TURTLEBOT3_MODEL=burger; ros2 launch gazebo_ros gzserver.launch.py world:='$WORLD'; exec bash"
if [[ "$SHOW_GUI" == "1" ]]; then
  gnome-terminal --title="Robot - Gazebo 3D Window" -- bash -lc \
    "source '$ROS_SETUP'; sleep 5; gzclient; exec bash"
fi
gnome-terminal --title="Robot - Spawn Burger" -- bash -lc \
  "source '$ROS_SETUP'; export TURTLEBOT3_MODEL=burger; sleep 12; ros2 run gazebo_ros spawn_entity.py -entity burger -file '$ROBOT_MODEL' -x 0 -y 0 -z 0.01; exec bash"
gnome-terminal --title="Robot - Dashboard" -- bash -lc \
  "source '$ROS_SETUP'; sleep 10; python3 '$PROJECT_DIR/dashboard.py'; exec bash"
gnome-terminal --title="Robot - Obstacle Avoidance" -- bash -lc \
  "source '$ROS_SETUP'; until ros2 topic list 2>/dev/null | grep -qx /scan; do echo Waiting for robot LiDAR...; sleep 2; done; sleep 3; python3 '$PROJECT_DIR/obstacle_avoidance.py'; exec bash"

sleep 22
command -v xdg-open >/dev/null 2>&1 && { xdg-open http://127.0.0.1:5000 >/dev/null 2>&1 || true; }
echo "Started. If a window shows an error, read that window's message."
