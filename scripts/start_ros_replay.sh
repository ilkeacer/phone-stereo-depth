#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
exec /usr/bin/python3 -m host.ros_session replay "$@"
