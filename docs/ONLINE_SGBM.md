# Online SGBM mapping

The project can now compute checked stereo depth while ROS receives the images, without a model, GPU, or precomputed depth cache. `host.ros_sgbm` is a separate process owned by the mapping supervisor. Stereo odometry continues to receive both camera images; RTAB-Map maps the aligned SGBM depth using the image-time TF route described in [RGB-D mapping](RGBD_MAPPING.md).

The desktop panel offers **90 sn rehberli oda testi** as an explicit SGBM choice, alongside the previous stereo path. **Kayıttan SGBM harita oluştur** runs the available stereo recording through the online engine. Opening the panel never opens the cameras. The initial implementation was checked on recordings; later user trials encountered tracking loss. Accurate room mapping and robot operation remain unvalidated. `auto` in the CLI preserves the previous source pipeline.

```bash
# No phone or precomputed depth. Supply a validated stereo-only bag and its calibration.
scripts/start_ros_replay.sh --bag data/ros/my-stereo-recording \
  --depth-engine sgbm --calibration data/my-calibration/calibration.npz --rate 1

# Only when physically ready, for the existing project/device configuration:
PHONE_DEPTH_ENGINE=sgbm PHONE_CAPTURE_SECONDS=90 scripts/start_ros_live_mapping.sh
```

The live launcher retains this project's explicitly estimated display scale. It must be adapted before use with a different device/calibration. Calibration rejection thresholds are unchanged. Adding online SGBM to a bag already containing model/precomputed depth is rejected, preventing two depth publishers.

## Processing contract

- Same `controlled_sgbm` getter settings as the recorded benchmark, external LR difference ≤1 px, finite positive 3D points and the same common-support region.
- Four stereo image/CameraInfo subscriptions with a 20 ms exposure difference limit. Each CameraInfo must exactly match its own image timestamp, frame, dimensions and expected projection. Duplicate/non-increasing images are rejected.
- One compute worker and at most one waiting pair. If work falls behind, a newer pair replaces the waiting older pair, with its timestamp and an explicit `superseded` count. This bounds the work queue; it is not a guarantee of a fixed frame rate.
- The ROS executor remains responsive to `/clock` while SGBM computes. Source age over one second is rejected before and after computation; replay clock age is not measured physical USB latency.
- Output is 16UC1 millimetres, NaN becomes zero, and values outside the transport range fail the node instead of wrapping. An entirely invalid frame contains only zero depth. The original left timestamp and optical frame are retained.
- Worker exit/failure stops the source and mapper and prevents successful map export. The hybrid tracking-loss guard still invalidates the map on the first loss; no acceptance threshold was relaxed. Since 2026-09-23, live sessions default to `--tracking-loss-policy keep-capture`: stop the mapper/depth worker but let the existing raw capture source finish. Map recovery/resumption is not implemented by this policy. Source/camera errors still stop capture. Replay defaults to `stop`; its continuation policy can be selected explicitly for fault testing. Source raw recording operates independently of SGBM frame replacement.

Every session has `depth/settings.json`, complete per-frame `depth/frames.jsonl`, `depth/status.json`, `depth.log` and `depth-engine.json`. Logs include real matcher getters, input stamps, coverage at each filter, computation time, source-age estimate and serialized depth/image SHA-256 values. Summary separates published depth, synchronized odometry, observer counts and stored/active map nodes.

`mapping-fallback.json` records when mapping stopped and capture continued. The original tracking failure is retained. Export rejects either durable failure marker even if a stale summary says the graph is intact. Completed/partial raw recording is reported separately from failed map tracking. Early process shutdown services ROS callbacks while waiting. Rosbag playback uses a bounded 3000 ms acknowledgement wait (the CLI value is milliseconds).

## Recorded evidence, 2026-09-20

The available recording is 226 pairs over 29.97 sensor seconds (about 7.5 pairs/s), not a room tour. At original playback speed, the worker implementation processed and published 226/226 depth frames; all 225 matched odometry results reported non-lost. SGBM computation median/p95 was 54.669/58.177 ms. That run exported 21,294 points and 10 connected poses. Replay timing/keyframe selection varies between runs, so point count is not a deterministic quality score.

At 2× playback (about 15 input pairs per wall second), 226 pairs synchronized, 178 depth frames were published and 48 pending pairs were explicitly replaced. All 225 matched odometry results remained non-lost. SGBM median/p95 was 55.068/58.299 ms; estimated publish age against the scaled ROS clock was median 158.052 ms, p95 223.822 ms, maximum 254.348 ms. Export: 18,157 points / nine connected poses. This stress trial does **not** demonstrate 15 Hz depth output or hardware-camera performance.

An independent ROS subscriber received 178 unique depth messages in that 2× trial. All 178 serialized uint16 depth payloads were bit-identical to the corresponding earlier OpenCV 4.13 cache after the same millimetre quantization. The online engine uses ROS system OpenCV 4.5.4 / NumPy 1.21.5. This equality applies to those received frames, not to all possible images or both library versions generally.

The final-code rate-1 run repeated this independent receiver check for **226/226** depth messages, all bit-identical to the quantized cache. It retained 225/225 non-lost odometry results, exported 21,228 points / 10 connected poses, and measured SGBM median/p95 54.804/61.844 ms. Its source-clock publish age p95 was 65.691 ms; that is replay timing, not a USB latency measurement.

A separate recorded-only fault test stopped the depth worker during active mapping. The supervisor returned exit code 1, shut down all three processes without forced kill, and returned export `skipped`. No phone or robot was involved.

The next physical validation is one deliberate 60–90-second room scan, with full raw recording and marked return position/orientation. Until then these results establish an operational online pipeline and bounded queue behavior, not an accurate room map or safe robot navigation.
