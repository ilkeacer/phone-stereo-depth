#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
if ! ros2 pkg prefix rtabmap_launch >/dev/null 2>&1; then
  echo 'RTAB-Map eksik. Kurulum: sudo apt install ros-humble-rtabmap-ros'
  exit 1
fi
mkdir -p work
if [[ -n "${PHONE_MAP_DIR:-}" ]]; then
  run_dir="$PHONE_MAP_DIR"
  if [[ "${PHONE_LOCALIZATION_ONLY:-0}" == "1" ]]; then
    if [[ ! -f "$run_dir/map.db" ]]; then
      echo 'Konum bulma için hazırlanmış harita kopyası yok.' >&2
      exit 1
    fi
  else
    mkdir "$run_dir"
  fi
else
  if [[ "${PHONE_LOCALIZATION_ONLY:-0}" == "1" ]]; then
    echo 'Konum bulma için PHONE_MAP_DIR gerekli.' >&2
    exit 1
  fi
  run_dir="$(mktemp -d "$PWD/work/ros-map-XXXXXXXX")"
fi
echo "Yeni harita oturumu: $run_dir"
rtabmap_args="--Rtabmap/WorkingDirectory $run_dir"
if [[ "${PHONE_LOCALIZATION_ONLY:-0}" == "1" ]]; then
  rtabmap_args+=' --Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true'
fi
if [[ "${PHONE_INPUT_KIND:-stereo}" == "hybrid" ]]; then
  exec ros2 launch "$PWD/configs/hybrid_mapping.launch.py"
fi
if [[ "${PHONE_INPUT_KIND:-stereo}" == "rgbd" ]]; then
  input_args=(stereo:=false depth:=true approx_sync:=false
    topic_queue_size:=100 sync_queue_size:=100 qos:=1
    rgb_topic:=/phone/left/image_rect depth_topic:=/phone/depth/image_rect
    camera_info_topic:=/phone/left/camera_info)
elif [[ "${PHONE_INPUT_KIND:-stereo}" == "stereo" ]]; then
  input_args=(stereo:=true stereo_namespace:=/phone approx_sync:=true approx_sync_max_interval:=0.02
    left_image_topic:=/phone/left/image_rect right_image_topic:=/phone/right/image_rect
    left_camera_info_topic:=/phone/left/camera_info right_camera_info_topic:=/phone/right/camera_info)
else
  echo 'Bilinmeyen görüntü kaynağı.' >&2
  exit 1
fi
exec ros2 launch rtabmap_launch rtabmap.launch.py \
  "${input_args[@]}" use_sim_time:="${PHONE_USE_SIM_TIME:-true}" frame_id:=phone_link \
  database_path:="$run_dir/map.db" rtabmap_args:="$rtabmap_args" \
  odom_args:="--Odom/ResetCountdown 30" \
  rtabmap_viz:="${PHONE_RTABMAP_VIZ:-true}" rviz:=false
