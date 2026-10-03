# Phone Stereo SLAM

**Two phone cameras → USB or Wi-Fi stereo stream → ROS 2 / RTAB-Map → inspectable 3D maps.**

A research prototype using the telephoto and ultrawide rear cameras of a Xiaomi Mi 9T Pro. It includes camera acquisition, cached-corner calibration, stereo diagnostics, recorded/live ROS input, map export, and a desktop control panel.

**Status:** the end-to-end mapping pipeline runs on recorded data. Room-scale accuracy and robot-mounted operation are not yet validated. Some maps are fragmented and dense clouds contain visible artifacts. Point count and valid-disparity coverage are not accuracy measurements.

The alternate ARCore pose/depth route now streams a moving phone's 3D points and path into ROS 2/RViz without saving camera images. A user-started 60 s run on Android v0.9 delivered 346 packets, 56,880 accumulated 5 cm cells and 344 tracked poses. A separate local analysis marks 7,995 cells as a near-horizontal surface candidate; its plane tilted 1.119° from gravity horizontal. The earlier 60 s run had long downward artifacts, while the v0.9 cloud shows a bounded floor band. The physical floor, metric accuracy and robot navigation safety remain unverified. [Measured ARCore results and next test](docs/ROS_WIFI_TR.md#arcore-icin-kisa-yol-deneyi).

An offline YOLO26n check measured 3.943 ms median inference on the local RTX 4080 using a public example resized to the project frame size. This does not establish live phone throughput or object accuracy. A third simultaneous phone camera is unproven and is not needed for detection from an existing camera image. [Camera/YOLO decision and limits](docs/YOLO_KAMERA_KARARI_TR.md).

A separate saved-map diagnostic now marks cells with recorded floor-candidate or other depth returns and leaves cells without returns explicitly unknown. It does not infer obstacle clearance or fill unseen geometry. The user intends a fully open-source release; the project license has not yet been selected.

**Object detail remains experimental.** Detailed capture now includes matched private RGB, raw depth/confidence, smoothed depth and poses. A user-started wireless capture delivered 273 depth frames and 564 tracked poses. On nine recorded tabletop views, the optional 1 cm/mean/minimum-three-frame raw map measured 37.74 → 36.96 mm median plane residual against the former 2 cm map; three views worsened. These are internal consistency measurements, not physical accuracy. [Detailed scan and comparison procedure](docs/ROS_WIFI_TR.md#arcore-ayrintili-harita-v010).

An optional recorded-only **Depth Pro + ARCore pose** candidate measured 61.77 → 11.81 mm tabletop plane residual, and 12.86 → 14.39 mm carpet residual, on nine views excluded from map construction. The same 240 source frames and common visible pixels were compared; this evaluation differs from the preceding nine-view measurement. The saved candidate contains 285,972 points and 564 poses and reached ROS 2/RViz. GPU inference measured 432 ms median across 240 frames; live phone AI throughput and physical geometry are unverified. [Recorded AI procedure and evidence](docs/ARCORE_AI_KAYITLI_DENEME_TR.md).

Saved supported surfaces can now be exported as a colored **GLB 3D scene with the recorded camera path**, plus an offline HTML orbit viewer. The existing 285,972-point candidate yielded 269,812 surface vertices and 1,363,572 triangles with no retained XYZ displacement. The GLB passed the official Khronos validator with zero errors or warnings. This is an observed, partial scene with gaps and overlapping surfaces; it is not a watertight or completed room model, and export does not improve geometric accuracy. Private scene files and images remain excluded from the public source package. [Export and remaining room-mapping steps](docs/ARCORE_ODA_CIKTISI_TR.md).

Android v0.12 adds **3 / 5 / 10 minute detailed room scans** (default five minutes) and **Stop and save**. Elapsed-time prompts hold only the first five seconds; deliberate user saves remain distinct from interrupted/error captures. The APK and 15 Android tests passed. Version 0.12 was subsequently installed and a user-started five-minute scan retained 1,287 matched RGB/depth frames and 2,776 tracked poses, with no sequence gaps or reported phone error. An optional `--min-face-frames 2` surface export retains only triangles seen in two independent depth frames: 1,363,572 → 420,390 triangles on the saved candidate, preserving all 285,972 source cloud points. This leaves more gaps and does not establish physical accuracy.

An optional **local CUDA batch reducer** now preserves the existing independent-frame voxel means. On the same 240 cached frames, CPU fusion took 5,704.40 ms; GPU fusion including CPU address preparation and transfers took 347.04 ms. All 285,972 output XYZ coordinates were exactly equal, with the same 564 poses. These are single-run offline fusion measurements, excluding depth inference and live transport. A separate standalone nvblox TSDF experiment lost tabletop coverage and remains a diagnostic option. Neither route requires a paid API or a ROS upgrade. [Commands, comparison scope and license notes](docs/ARCORE_UCRETSIZ_GPU_TR.md).

The optional **AI ROS stream** now separates camera ingestion, inference and ROS publication. A latest-frame slot reduced recorded GPU replay arrival-to-map p95 from 80.13 s in a FIFO control to 0.772 s, preserving all 564 poses. An independent ROS subscriber received 80,818 accumulated points and 378 path updates while inference was active. Processing 105 of 240 eligible frames yields fewer confirmed points than the 285,972-point full replay; the complete private camera stream remains available for offline refinement. Physical phone AI streaming and room geometry are unverified. [Replay commands, tradeoff and live listener](docs/ARCORE_AI_CANLI_TR.md).

## Latest source snapshot

The five-minute recording was processed locally without another capture: Depth Pro predicted 1,255 frames and skipped 32 at the existing scale-validation guard. The accumulated cloud contains 1,560,048 points and 2,776 poses; a separate surface export requiring two independent frame observations contains 2,099,823 triangles. An independent ROS 2 subscriber received the cloud and path with zero coordinate difference from the saved files. These counts demonstrate the processing and export pipeline, not physical room accuracy.

The saved HTML viewer is a **recorded scene**, not a live camera display. The live AI listener now waits for the user to start capture until explicitly cancelled; the active-stream 20-second timeout remains. This waiting behavior has synthetic test coverage but has not yet been rechecked on the phone. Instant current-frame preview work is unfinished and excluded from this source snapshot.

Only audited text sources and aggregate measurement documents are distributed. Captured images, generated scene viewers, point clouds, raw streams, device logs, local continuation notes, credentials and model checkpoints are excluded.

## What works

| Capability | Evidence / boundary |
|---|---|
| Independent cameras 20 + 21 | Physically identified on one Mi 9T Pro, Android 11; portability to other phones is unknown |
| Timestamped stereo over USB | Exact image/capture metadata join; pairs within 20 ms; no hardware exposure synchronization |
| Wi-Fi image transport | The latest 90 s moving Wi-Fi session captured 870 stereo pairs and published 854 online depth frames; the scene remains too narrow for a complete room map |
| Phone inertial stream | Raw gyro/accelerometer samples are timestamped, logged, and gyro is published as `/phone/imu_raw`; a 7 s device check recorded 353 samples of each with zero phone-buffer drops. Sensor fusion is pending camera–IMU alignment and bias validation |
| Calibration | 27 training and 10 held-out poses; vertical-error p95 1.451 px on those poses |
| ROS 2 mapping | Latest 90 s Wi-Fi run yielded 30 connected poses / 63,365 exported points. The view remains a nearby-furniture section, not a validated room map |
| ARCore live and saved 3D | `/phone/arcore_map` and `/phone/arcore_path` reached RViz during user-started scans; a saved-map ROS reload independently delivered 56,880 points, 7,995 colored surface candidates, and 344 poses. Room geometry and cross-session localization are unverified |
| Saved-map localization | The stored map, live path, and current odometry pose can be shown in RViz. Same-recording replay produced 10 accepted map matches; a separate earlier room recording produced 0, so cross-session relocalization is unverified |
| Session lifecycle | Source and mapper process groups shut down before export; tracking loss preserves raw capture and rejects the interrupted map |
| Tracking-loss salvage | Separate pre/post-loss raw sections can be exported as independent ROS maps; no unverified pose bridge or automatic relocalization |
| Online SGBM depth | Explicit live/replay option; latest 90 s moving phone/Wi-Fi/ROS2 run published 895 depth frames from 906 stereo pairs |
| Map inspection | PLY, trajectory, RViz topics, separately labeled disconnected components; recorded ARCore/AI surfaces as GLB and an offline orbit viewer |
| Reproducible diagnostics | Actual matcher settings, LR-stage counts, input hashes, per-frame outputs |

The [recorded depth consistency audit](docs/DEPTH_CONSISTENCY.md) compares identical optical tracks across SGBM and learned depth. It also exposed an odometry/image timestamp mismatch: the updated hybrid path retained the previously omitted motion interval. [Measured hybrid results and remaining limitations](docs/RGBD_MAPPING.md).

The [live 3D status and localization audit](docs/LIVE_3D_STATUS_TR.md) records the latest map, path, and replay measurements. Independent geometry, room coverage and cross-session localization remain unverified.

The desktop panel also provides a separate, guided **3-minute whole-room capture**. A later 90 s recording retained all 981 stereo pairs but lost SGBM tracking about 29 s into capture in a dark scene; its first safe section produced a separate 7-pose offline map. The remaining section is retained without asserting one continuous room map. See [the Wi-Fi ROS guide](docs/ROS_WIFI_TR.md).

[Online SGBM operation and load test](docs/ONLINE_SGBM.md): depth is now computed from the incoming ROS stereo stream. The panel includes an explicit experimental SGBM scan button and a camera-free SGBM replay button.

Held-out camera-21 corners cover only the central part of its image. The local author's calibration has an **approximate ruler-derived** 21.44 mm checker-square sidecar; this does not validate map geometry. New calibrations need their own physical scale. See [evaluation protocol](docs/EVALUATION.md) before interpreting a map as usable robot geometry.

## Architecture

```mermaid
flowchart LR
    A[Android Camera2: 20 + 21] --> B[USB or Wi-Fi ADB / timestamp and geometry checks]
    I[Android gyro + accelerometer] --> J[Timestamped IMU log + ROS /phone/imu_raw]
    B --> C[Common-intrinsics rectification]
    C --> D[ROS stereo images + CameraInfo + TF]
    D --> E[RTAB-Map odometry and mapping]
    E --> F[Database / PLY / trajectory / RViz]
    C --> G[Offline BM / SGBM diagnostics]
    H[Android ARCore pose + Depth API] --> K[ROS live 3D cloud + camera path]
    K --> L[Local PLY + trajectory + RViz reload]
```

Early RTAB-Map stereo recordings used **StereoBM**; the latest hybrid ROS2 session uses online controlled **SGBM** depth. The local depth viewer is another path. Results from one mode must not be attributed to another.

## Quick start: tests without a phone

Python 3.10 on Linux:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r host/requirements.txt
.venv/bin/python -m unittest discover -s tests -v
```

ROS-message tests skip when ROS is unavailable. CI checks the source tests; it does not certify camera hardware, mapping accuracy, or GPU inference.

## Camera capture and calibration

Prerequisites: Ubuntu, JDK 11, adb, Python/Tk, curl/unzip and an Android phone with authorized USB debugging.

```bash
scripts/setup.sh
scripts/build.sh testDebugUnitTest
adb install -r android/app/build/outputs/apk/debug/app-debug.apk
scripts/start_assistant.sh
```

The assistant guides target observations and preserves selected poses. The calibration gate is not relaxed for mapping. This specific camera combination was tested; starting arbitrary hidden camera IDs is not a portability strategy. See [reproduction notes](docs/REPRODUCE.md).

## ROS 2 mapping

Install ROS 2 Humble and `ros-humble-rtabmap-ros` on Ubuntu 22.04. ROS tools use the system Python and its NumPy/OpenCV, independently of the pinned offline virtual environment. A compatible local calibration is required; private calibration/capture files are not included in the source repository.

```bash
# Desktop panel; opening it does not start the cameras.
scripts/start_ros_dashboard.sh

# Completed, validated recording -> bag. Supply your actual paths and scale.
source /opt/ros/humble/setup.bash
python3 -m host.ros_stereo --trial data/my-recording \
  --calibration data/my-calibration/calibration.npz \
  --square-mm 25 --output data/ros/my-recording

# Recorded input, no phone required.
scripts/start_ros_replay.sh --bag data/ros/my-recording
```

`--square-mm 25` is an example **measured** size, not the size of the author's target. For an explicitly estimated scale use `--estimated-square-mm`; exports retain that provenance. The bundled live launcher uses the author's local calibration-bound scale sidecar and camera settings, so it must be adapted to a different calibration/device. [Turkish operating guide](docs/ROS_3D_MAPPING_TR.md).

Each session receives its own directory. `map/map.db` is the database; `export/map_cloud.ply` and `map_poses.txt` are the cloud and poses. `summary.json`, `lifecycle.json`, and `export-result.json` distinguish connected, partial, failed, and interrupted results. Failed export attempts are retained and can be retried.

The [Wi-Fi ROS guide](docs/ROS_WIFI_TR.md) opens the desktop panel over a selected wireless ADB link. It includes a 30-second camera-only preview to check scene brightness and detail before mapping. A measured 15-second Wi-Fi run delivered stereo, ROS images, online SGBM depth and tracking; the stationary dark view produced only one map pose, so wireless room mapping remains unvalidated.

To reopen a saved ARCore diagnostic without starting the phone camera, run `python3 -m host.ros_arcore_saved --session work/<session> --rviz` after sourcing ROS 2. Add `--classified work/<session>/floor-analysis/classified_map.ply` to show the separately derived surface candidate in green. This restores the saved cloud and path in the capture's local frame; it does not relocalize a later session.

During an SGBM session, **Anlık 3B derinlik (RViz)** opens `/phone/local_cloud`: metric points from the current stereo pair in `phone_left_optical`. This moving, camera-relative view does not accumulate a room map. **Biriken ROS haritası (RViz)** opens RTAB-Map's `/rtabmap/cloud_map` in `map`. The desktop panel keeps a provisional RTAB-Map stream visible after tracking loss, records the loss, and saves only a clearly labeled diagnostic fragment; it does not accept the session as a continuous robot map. A recorded-room replay produced a 75-pose, 232,317-point connected diagnostic fragment and published accumulated cloud messages while running. See [live 3D status](docs/LIVE_3D_STATUS_TR.md).

For new recordings, a tracking-loss session retains source images and records a timestamp-based recovery plan. [`ros_recovery_segments`](docs/ROS_RECOVERY_SEGMENTS_TR.md) copies safe pre/post-loss sections into new directories; `host.ros_recovery_map` processes those sections sequentially as **independent** ROS maps. Older recordings without the exact ROS/source timestamp offset are not split automatically. Neither step joins the maps or certifies a robot-ready trajectory.

## Stereo / heterogeneous-lens benchmark

```bash
python3 -m host.stereo_benchmark \
  --trial work/my-completed-recording \
  --calibration data/my-calibration/calibration.npz \
  --output work/benchmark-new
```

The benchmark selects equally spaced frames within each recorded phase before examining results. It evaluates fixed regions and illumination/sampling ablations, saves actual matcher getter values and both disparity maps, and checks input hashes. Output directories must be new. Full raw evidence remains local. [Measured results](docs/BENCHMARK_RESULTS.md) · [Method and interpretation](docs/EVALUATION.md).

## Experimental monocular depth

An optional local **Depth Anything V2 Metric Hypersim Small** adapter and recorded-data comparator are available. It predicts dense depth from one rectified camera image. Output is model-predicted metres, not validated metric geometry; it is not enabled in the live mapper. A [recorded hybrid path](docs/RGBD_MAPPING.md) retains stereo odometry while mapping precomputed model or SGBM surfaces. [Setup, measured results and limitations](docs/DEPTH_ANYTHING.md).

`host.arcore_ai_map` optionally processes recorded matched RGB with Metric Small or Depth Pro, recorded gravity orientation and ARCore poses. It anchors predictions to explicitly unvalidated raw depth, preserves discontinuities by rejecting pixels, writes a new directory and checks the source hash. It never opens the camera. Metric Small uses Apache-2.0; Depth Pro code and weights use [Apple's upstream license](https://github.com/apple-aiml-research/ml-depth-pro/blob/main/LICENSE). Optional Ultralytics YOLO code/weights use AGPL-3.0 by default; see [Ultralytics licensing](https://www.ultralytics.com/license) before integrating or redistributing them. YOLO masks have not been used to alter map geometry.

## Development and release

- `android/`: Camera2 acquisition and bounded stereo transport.
- `host/`: calibration, depth, ROS lifecycle, map tools and offline evaluation.
- `tests/`: geometry, timing, process cleanup, exports, and synthetic regressions.
- `configs/`: calibration guidance and RViz display configuration.
- `docs/`: measurement boundaries and reproduction instructions.

Use the [public source export procedure](docs/PUBLICATION.md) to prepare a source-only ZIP. Captures, maps, local logs, machine paths, credentials, and model weights are excluded. There is currently no project-wide license grant; third-party code and model terms remain separate.
