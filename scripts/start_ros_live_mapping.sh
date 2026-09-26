#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
args=()
args+=(--depth-engine "${PHONE_DEPTH_ENGINE:-auto}")
if [[ -n "${PHONE_TRACKING_LOSS_POLICY:-}" ]]; then args+=(--tracking-loss-policy "$PHONE_TRACKING_LOSS_POLICY"); fi
if [[ "${PHONE_DIAGNOSTIC:-0}" == 1 ]]; then args+=(--diagnostic); fi
if [[ -n "${PHONE_LOCALIZE_MAP_DB:-}" ]]; then args+=(--localize-map-db "$PHONE_LOCALIZE_MAP_DB"); fi
exec /usr/bin/python3 -m host.ros_session live --seconds "${PHONE_CAPTURE_SECONDS:-120}" "${args[@]}"
