#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}" ROS_LOCALHOST_ONLY=1
mkdir -p work
export ROS_LOG_DIR="$PWD/work/ros-dashboard-log"
exec 8>work/ros-dashboard.lock
if ! flock -n 8; then
  echo 'ROS kontrol paneli zaten açık.'
  exit 1
fi
exec /usr/bin/python3 -m host.ros_dashboard "$@"
