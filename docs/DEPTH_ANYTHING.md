# Depth Anything V2: recorded phone evaluation

Date: 2026-09-19. No new phone capture; calibration and rejection thresholds are unchanged. This is an optional **offline experimental backend**, not the live ROS depth source. All inference runs locally; scene data is not uploaded.

## Choice and integration boundary

The official [Depth Anything V2 repository](https://github.com/DepthAnything/Depth-Anything-V2) provides relative-depth models and separate [metric models](https://github.com/DepthAnything/Depth-Anything-V2/tree/main/metric_depth). This experiment uses **Metric Hypersim Small**, an indoor model with a 20 m output range. Its [model card](https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small) declares Apache-2.0. The adapter outputs model-predicted metres; these are not camera-calibrated, independently measured distances.

A monocular model avoids matching telephoto and ultrawide patches. This removes that requirement from its inference path, but does not establish correct scale, camera trajectory or consistent mapping. No prediction was merged into stereo holes or passed to RTAB-Map as validated depth.

`host/monocular_depth.py` loads an explicit local upstream checkout and checkpoint, checks their pinned revision/hash, and returns aligned float32 predictions. `host/depth_anything_benchmark.py` compares the same recorded view and keeps every prediction, stereo mask/disparity, frame statistic, input hash and console output. Torch is optional and imported only when inference starts; base source tests run without weights or the upstream checkout.

## Reproduce

Run from the project root. The commands download public model/code assets, not private data. Upstream code and weights stay in excluded `work/` and `data/` directories. Use a new output directory for each run.

```bash
.venv/bin/python -m pip install -r host/requirements-depth-anything.txt
git clone https://github.com/DepthAnything/Depth-Anything-V2 work/Depth-Anything-V2
git -C work/Depth-Anything-V2 checkout a561b849ebae10a6f5ef49e26c83cbbcd36c71bf
mkdir -p data/models/depth-anything-v2
curl -fL -o data/models/depth-anything-v2/hypersim-small.pth \
  https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small/resolve/main/depth_anything_v2_metric_hypersim_vits.pth

.venv/bin/python -m host.depth_anything_benchmark \
  --trial work/ros-live-VV2Vb5Xl/raw \
  --calibration data/calibration/screen_20260910/calibration.npz \
  --repository work/Depth-Anything-V2 \
  --checkpoint data/models/depth-anything-v2/hypersim-small.pth \
  --estimated-square-mm 21.979195166653625 \
  --output work/depth-anything-20260919-final
```

The second actual run adds `--rotation 90` and uses `--output work/depth-anything-20260919-rot90`. This clockwise input rotation is inverted on the prediction before any stereo comparison; camera calibration remains unchanged. It is a declared orientation probe motivated by the recorded sideways scene, not a fitted depth correction or independently measured gravity direction. Defaults remain rotation 0. The data paths and estimated square size above refer to this private recording; other users must supply their own completed recording/calibration/scale declaration.

Checkpoint SHA-256: `b782898d8a3e8be1f639de33837ed85e9b4b73e40f8f5e5cd99067588d722545`. Only source and numeric aggregates are in the public archive, not the upstream code, weights or scene images.

## Measurement scope

Twelve anchor frames, fixed before inference: 0, 24, 49, 74, 75, 100, 125, 150, 151, 175, 200, 225. Nine next-frame samples stay within their recorded phase: 21 predictions per run. Source images are rectified telephoto grayscale replicated into three channels at 640×480; they are not a native-color wide-FOV input. Model input size 518; default preprocessing preserves aspect ratio. Both runs use the same images and calibration.

Runtime: RTX 4080 Laptop GPU, torch 2.10.0+cu128, OpenCV 4.13.0. The original BM/SGBM benchmark used system OpenCV 4.5.4; this experiment recomputes its controlled-SGBM comparator in 4.13.0. Neural timings include preprocessing, inference, GPU synchronization and transfer to CPU, after one warm-up; disk input, rectification, model loading and ROS are excluded. CPU SGBM and GPU neural timings are not equal-hardware algorithm comparisons or complete system FPS.

## Results

Values are medians across fixed anchor frames unless indicated. The disagreement denominator is the SGBM depth converted using **estimated** display scale, on the SGBM final valid mask. Formula: `100 * abs(predicted / stereo_estimate - 1)`; take the median over pixels per frame, then across frames. This is agreement between two unvalidated methods, not accuracy error. No per-frame scale alignment or offset fitting was applied.

| Probe | Inference median (ms, 21 frames) | Inference p95 (ms) | Median predicted / stereo ratio | Median absolute relative disagreement |
|---|---:|---:|---:|---:|
| Input rotation 0° | 36.500 | 41.799 | 0.9246 | 48.568% |
| Input rotation 90° | 38.421 | 43.235 | 1.3211 | 35.456% |

All pixels received positive finite predictions; that is the model's output coverage, **not** a left-right-validated match ratio. Controlled-SGBM final coverage was 35.3652% of the full image. Those percentages have different definitions and must not be presented as a quality increase. Peak torch CUDA allocation was 283.662 MiB (not total GPU/driver memory).

### Adjacent-frame change

We track up to 500 image corners with forward/backward optical flow (≤1 px round-trip discrepancy) and measure absolute relative predicted-depth change at the tracked pixels. **No camera-pose compensation or ground-truth motion is available**. This includes real optical-Z changes, tracking errors and prediction variation. Phase names are capture instructions, not proof the phone was stationary. Each row below summarizes the three tested adjacent pairs in that phase.

| Rotation | Recorded phase | Median of pair-wise median changes | Maximum pair-wise median change |
|---|---|---:|---:|
| 0° | sabit | 2.496% | 13.644% |
| 0° | otele | 7.308% | 11.945% |
| 0° | sabit_son | 15.331% | 20.451% |
| 90° | sabit | 14.211% | 20.503% |
| 90° | otele | 5.918% | 9.030% |
| 90° | sabit_son | 3.740% | 5.336% |

Full per-pair track counts and delta times are in [the numeric record](evidence/depth-anything-benchmark.json); full per-frame statistics are in local `frames.jsonl`. These nine pairs do not establish long-sequence consistency or loop closure.

## Decision for the robot project

Keep the adapter as an inspectable offline candidate. Its dense output and measured local latency justify further evaluation, but the disagreement and orientation sensitivity do not justify switching the live mapper to predicted depth. Stereo and the model can both be wrong; the current recording has no independent depth ground truth.

The subsequent [RGB-D/hybrid replay implementation and results](RGBD_MAPPING.md) are available. The original next-step plan was a separate RGB-D replay branch with camera-aligned predicted depth and explicit predicted-scale provenance, compared against the existing stereo trajectory on the same whole recording. That would evaluate mapping behavior, not establish real accuracy. Before choosing a robot depth source, use the marked-return and independently measured scene-distance protocol in [EVALUATION.md](EVALUATION.md). A future temporal model or stereo-anchored fusion needs its own evaluation; neither is silently provided by this integration.
