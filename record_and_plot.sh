#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
python3 -c 'import matplotlib' 2>/dev/null || {
  echo 'Matplotlib is missing. Install it with: sudo apt install python3-matplotlib'
  exit 1
}
exec python3 "$(dirname "$0")/record_and_plot.py" "$@"
