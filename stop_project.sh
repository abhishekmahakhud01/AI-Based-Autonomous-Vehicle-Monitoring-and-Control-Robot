#!/usr/bin/env bash
# Best-effort shutdown. Avoid pkill matching this script itself.
set +e
stop_pattern() {
  local pattern="$1"
  pkill -TERM -f "$pattern" 2>/dev/null
  sleep 1
  pkill -KILL -f "$pattern" 2>/dev/null
}
stop_pattern '[o]bstacle_avoidance.py'
stop_pattern '[d]ashboard.py'
stop_pattern '[s]pawn_entity.py'
pkill -TERM gzclient 2>/dev/null
pkill -TERM gzserver 2>/dev/null
sleep 1
pkill -KILL gzclient 2>/dev/null
pkill -KILL gzserver 2>/dev/null
if pgrep -af 'obstacle_avoidance.py|dashboard.py|gzclient|gzserver' >/dev/null; then
  echo 'Shutdown attempted; some matching processes may still be running. Check the process list.'
else
  echo 'Robot controller, dashboard, and Gazebo processes stopped (where running).'
fi
