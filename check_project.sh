#!/usr/bin/env bash
set -u
echo "Checking ROS 2 Humble..."
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u; source /opt/ros/humble/setup.bash; set -u
  echo "  ROS 2 setup: found"
else
  echo "  ERROR: /opt/ros/humble/setup.bash not found"
fi
echo "Checking Python files..."
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -m py_compile "$PROJECT_DIR/obstacle_avoidance.py" "$PROJECT_DIR/dashboard.py" \
  && echo "  Python syntax: OK" || echo "  ERROR: Python syntax check failed"
python3 -c 'import flask' >/dev/null 2>&1 \
  && echo "  Flask: installed" || echo "  Flask: missing (python3 -m pip install --user -r requirements.txt)"
for f in \
  /opt/ros/humble/share/turtlebot3_gazebo/worlds/empty_world.world \
  /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf; do
  [[ -f "$f" ]] && echo "  Found: $f" || echo "  MISSING: $f"
done
[[ -f "$PROJECT_DIR/worlds/obstacle_world.world" ]] && echo "  Found: worlds/obstacle_world.world" || echo "  MISSING: worlds/obstacle_world.world"
