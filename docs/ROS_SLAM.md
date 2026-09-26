# ROS / SLAM integration status

Recorded/live stereo input, CameraInfo, optical transforms, RTAB-Map supervision, raw odometry logging, PLY/trajectory export and RViz inspection are implemented. Recorded input has produced a connected 10-pose map. This is a working research pipeline; it is not yet a validated robotic mapping system.

Start with the [operating guide](ROS_3D_MAPPING_TR.md), [recorded benchmark](BENCHMARK_RESULTS.md), and [acceptance protocol](EVALUATION.md). The nominal display-based scale remains estimated. Exposure synchronization, wide-angle edge calibration and scene geometry accuracy remain unresolved. Graph fragmentation and cloud artifacts have been observed.

Before mounting on a robot, measure scale and the camera-to-body transform, verify timestamps and controlled-motion tracking, and evaluate a marked physical return with raw odometry separately from optimized poses. Encoders/IMU require their own timing and extrinsic calibration. Nav2 and a motor controller are not implemented.
