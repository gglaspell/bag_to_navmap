#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/navmap_ws/install/setup.bash

exec python3 /app/bag_to_navmap.py "$@"
