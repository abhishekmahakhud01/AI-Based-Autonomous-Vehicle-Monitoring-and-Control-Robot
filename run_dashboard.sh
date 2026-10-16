#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
python3 "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/dashboard.py"
