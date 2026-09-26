# Evaluation protocol

## Separate causes, not a binary diagnosis

Investigate four interacting categories:

1. Geometry and calibration coverage, including out-of-distribution image edges.
2. Exposure timing, rolling shutter and physical motion.
3. Heterogeneous optics: sampling density, blur/PSF, noise, exposure/tone differences and residual perspective/occlusion differences.
4. Matcher parameters and downstream odometry/graph failure.

Poor BM and SGBM results do not choose one of these causes. Poor results after a narrow crop do not prove lens asymmetry or eliminate calibration/timing.

### Why raw crop-and-resize is not sufficient

OpenCV stereoRectify defines P1/P2 in rectified camera coordinates with a common focal length. Remapping therefore already resamples the two cameras onto a common angular sampling grid. Native telephoto/ultrawide focal lengths do not imply that a fourfold patch-scale difference remains in the rectified images. However, interpolation cannot restore missing detail from the lower-resolution angular sample, and matching appearance can still differ.

Reference: [OpenCV stereoRectify and projection matrices](https://docs.opencv.org/4.8.0/d9/d0c/group__calib3d.html). The inference about this project's remaining optical mismatch must be checked against the saved matrices and data, not assumed from the lens names.

The current experiment evaluates a fixed central 200×200 rectangle and a conservative common remap/disparity support region. It measures source-coordinate bounds and remap scale within the center. This is an auditable counterpart to a manual crop, without inventing a new principal point or discarding the stereo search halo. These regions are evaluated on the same full-image match result; they are not independently rerun cropped matchers.

## Paired benchmark

Use one completed capture and one frozen calibration. Select four evenly spaced indices per phase, before looking at matching results. Keep image dimensions, disparity range, LR threshold, region masks, and sample selection fixed across comparisons.

Profiles:

- `rtabmap_bm`: recorded RTAB-Map BM defaults, evaluated through the installed Python OpenCV API. This is a parameter reproduction, not instrumentation of the C++ ROS node.
- `controlled_bm` / `controlled_sgbm`: same block size, disparity range, uniqueness, speckle and LR settings where APIs share controls. BM-specific prefilter/texture controls and SGBM smoothness terms still differ; this does not isolate one mathematical variable.
- `host_sgbm`: local depth-profile parameters, evaluated with the benchmark's common region rules.

Ablations: original rectified images; Gaussian sigma=1 px on telephoto only; CLAHE on both views. Sigma is a declared probe, not a measured PSF. Improvements or deteriorations in an ablation do not uniquely identify optical or photometric causation. No variant is automatically promoted to live mapping.

Report full-image, common-support and central-window denominators, backend-valid counts, external LR counts, and positive-finite-depth counts. A backend count already contains its internal checks. Actual getter values, all selected frame results, left/right raw disparities and final masks are retained. Counts measure coverage, not correct matches.

The historical 4.3% live value is not paired with this dataset and cannot be used as a causal before/after baseline.

## Proposed acceptance gate for the next physical test

These are **prospective engineering targets**, not already achieved results or universal SLAM standards. Freeze them before capture; do not loosen them after seeing the run.

1. Use a 60–90 second slow route in a well-lit textured scene. Return the phone to a physically marked starting position **and orientation**. A rough return to the same area is not ground truth.
2. No camera/mapper failure or forced shutdown. At least 95% of odometry updates tracked, with no continuous tracking-loss interval longer than 1 second.
3. One connected active pose graph; no unexplained resets into disconnected maps.
4. Record start/end translation and rotation for the **pre-loop-optimization odometry** and optimized trajectory separately. Initial engineering target: translation ≤10 cm and rotation ≤5 degrees at the repeatable physical endpoint.
5. The centimeter test requires a measured scale. With nominal display scale, print the estimated value and mark the metric gate **not assessable**, not passed. No nominal ruler-free estimate becomes a measurement by multiplying units.
6. A small optimized start/end error is not sufficient: loop closure can enforce it. Report whether a closure was accepted, its supporting inliers if available, and inspect consistency against the physically repeated pose and one independently measured distance.
7. Robot integration requires measured phone-to-body mounting transform and scale. A closed graph alone is not permission to use it for autonomous navigation.

Timing, coverage, graph continuity, closure and real-distance checks must all be visible in the result. Passing a unit test, exporting a dense cloud, or increasing valid-pixel ratio cannot substitute for these gates. The supervisor records pre-optimization `/rtabmap/odom` as `odometry-raw.txt` only when exact-stamp `OdomInfo.lost=false`; status and inliers are retained separately. A zero-inlier initialization may be a valid origin and is not silently discarded. `python -m host.trajectory_check` scores its endpoint separately from an optional optimized trajectory. Physical endpoint/scale confirmation remains external; no automatic physical ground truth is available. Passing the endpoint sub-gate does not pass the full SLAM protocol.
