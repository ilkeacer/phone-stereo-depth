# Recorded RGB-D and hybrid mapping

The project can replay precomputed depth aligned with its recorded rectified left camera. `host/ros_rgbd.py` has separate venv preparation and system-ROS bag export stages. No camera is opened by these commands. Each destination must be new.

```bash
# Learned depth cache, using the pinned assets described in DEPTH_ANYTHING.md:
.venv/bin/python -m host.ros_rgbd prepare --trial work/my-recording \
  --calibration data/my-calibration/calibration.npz --repository work/Depth-Anything-V2 \
  --checkpoint data/models/depth-anything-v2/hypersim-small.pth \
  --rotation 90 --output work/my-model-depth

# Or geometric depth; specify your explicit estimated square size:
.venv/bin/python -m host.stereo_recording_depth --trial work/my-recording \
  --calibration data/my-calibration/calibration.npz \
  --estimated-square-mm 25 --output work/my-sgbm-depth

source /opt/ros/humble/setup.bash
python3 -m host.ros_rgbd hybrid-bag --cache work/my-model-depth \
  --stereo-bag data/ros/my-stereo-recording --output data/ros/my-hybrid-recording
scripts/start_ros_replay.sh --bag data/ros/my-hybrid-recording --rate 0.5
```

The `25` and rotation above are examples, not validated device defaults. Follow the source-specific scale and orientation declarations. `hybrid-bag` requires exact source-time and left-pixel correspondence with the stereo bag. CameraInfo remains the left rectified projection; right camera information and estimated baseline are retained for stereo odometry. Model outputs have no per-frame depth fit or hidden scale alignment. The model-depth and odometry scale sources remain separate.

`ros_rgbd bag` instead exports RGB-D only, using learned depth for odometry too. On the available 226-pair, 30-second diagnostic this produced only 75 non-lost results out of 191 received odometry samples and three graph components. The hybrid route retained stereo motion estimation: 225/225 received samples reported non-lost from 226 input pairs, with one six-pose component. Model surfaces exported 35,051 points; checked-SGBM surfaces exported 11,175. These are pipeline/coverage observations, not ground-truth accuracy. The sequence does not cover a whole room. The two surface types visibly retain overlap artifacts or holes.

The six-pose hybrid results above used an equal-stamp odometry subscription that omitted part of this recording. The current hybrid launch keeps exact RGB/depth/CameraInfo synchronization but obtains stereo motion through `odom -> phone_link` TF at the left-image timestamp. In this recording all 225 odometry stamps match the later of the two camera exposures, while depth uses left time. Source image and odometry timestamps are preserved. The first replays with this change produced nine connected poses: 20,673 SGBM points and 53,117 model points. Stored mapping timestamps had a maximum gap of 10.789267 seconds before and 1.066079 seconds afterwards (19 versus 29 stored samples, including nodes later merged). A subsequent final-code SGBM replay produced 18,574 points and nine poses, with 225 independently observed non-lost statuses and 225 matched odometry/status pairs. Intermediate replay: 21,425 points / 10 poses / 217 matched pairs. Keyframe selection and replay delivery are not deterministic; all trials are retained. These are delivery observations, not physical map accuracy.

TF has no per-message odometry covariance: this route explicitly uses the RTAB-Map defaults of 0.001 linear and 0.01 angular variance. It also cannot convey the odometry lost/reset flag. The supervisor therefore independently observes OdomInfo and fails the whole hybrid session on reported loss or a non-increasing status timestamp. It shuts down the source/mapper and prevents successful map export; it does not claim atomic per-image gating or continuation through a reset. An isolated ROS replay with an injected synthetic loss stopped with exit code 1, clean process shutdown and export `skipped`. Undetected failures and resets without a loss/timestamp signal remain unsupported. One final image in normal replays lacked a future pose and was rejected with a TF extrapolation warning. Received odometry counts must not be described as successful TF lookups for every image.

Transport uses 16UC1 millimetres (≤0.5 mm quantization) with original float caches retained. RGB-D subscribers request reliable transport and queues of 100; recorded trials used half-speed playback. Initial 32FC1 and unbuffered trials processed too few frames and were not considered successful mapping. Even later runs can drop data: input count, observer count, synchronized odometry count and active map nodes are separate metrics. `summary.json` now also reports the stored-node timeline independently of the active graph.

Raw OdomInfo `lost` flags define reported tracking; console feature quality is retained only as a separate diagnostic. Expected child SIGINT after supervised shutdown is distinguished from a crash during processing. Export scope distinguishes connected and partial graphs. None of these changes relax calibration rejection thresholds.

Normal live sessions now preserve all admitted post-warm-up image pairs in `raw/`, capped at 12,000 pairs / 1 GiB. The separate 30-second diagnostic keeps its original bounds. Completion, cancellation and failure remain explicit; unfinished recordings do not silently pass the offline importer. This new recording behavior has source regression coverage but awaits a real-device run.

The [depth consistency audit](DEPTH_CONSISTENCY.md) compares the surface methods using identical held-out optical tracks. Learned surfaces and global scale fitting remain experiments. [Online SGBM](ONLINE_SGBM.md) is now an explicit panel/CLI option for deriving depth from the incoming stereo stream; the existing automatic stereo/BM path remains available.
