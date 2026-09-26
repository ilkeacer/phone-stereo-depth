# Recorded stereo benchmark — 2026-09-15

This measures disparity coverage, **not depth accuracy or SLAM readiness**. No new phone capture was made. Original calibration, recordings and validation thresholds were retained. Numerical aggregates are distributed in [stereo-benchmark.json](evidence/stereo-benchmark.json); private scene arrays remain local.

## Method and evidence

OpenCV 4.5.4 / NumPy 1.21.5, system Python with ROS 2 Humble. The existing 226-pair recording spans 29.9702 seconds, with maximum paired timestamp difference 18.9807 ms. Twelve frames were selected before looking at results: 0, 24, 49, 74, 75, 100, 125, 150, 151, 175, 200, 225. Four profiles × three image variants × twelve frames = 144 evaluations.

Actual invocation from the repository root:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m host.stereo_benchmark \
  --trial work/ros-live-VV2Vb5Xl/raw \
  --calibration data/calibration/screen_20260910/calibration.npz \
  --output work/stereo-benchmark-20260915-v3
```

Local artifacts: `work/stereo-benchmark-20260915-v3/summary.json`, `geometry.json`, full `frames.jsonl` with actual matcher getter values and all stage counts, 144 NPZ files with both raw disparity maps and final masks, and `input-before.json` / `input-after.json`. The console transcript is `work/stereo-benchmark-20260915-v3-console.txt`. These are generated measurements, not independent verification; a reviewer can recompute the masks from the saved arrays. Hash equality covers the exact 30 listed inputs: calibration and report, manifest and pair metadata, two implementation files, and 24 selected source images. It does not certify every repository file. Earlier v1/v2 outputs are retained. v3 preserves the host depth matcher’s default preFilterCap (getter 0), whereas v2 forced 31. v3 is authoritative; no earlier host-profile row should be treated as the actual host matcher configuration.

All percentages below are medians across the same twelve frames. Full image: 307,200 pixels. Common remap/disparity support: 232,534 pixels. Fixed central 200×200 window: 40,000 pixels. The region/support masks and denominators stay fixed across all profiles and variants.

Backend includes matcher-internal filtering. LR adds the same external left-right test (≤1 px). Final additionally requires positive finite reprojected depth and common support. A larger count can include more incorrect matches.

## Original images: full-image stages

| Profile | Backend % | External LR % | Final % |
|---|---:|---:|---:|
| rtabmap_bm | 7.8854 | 4.4801 | 4.4801 |
| controlled_bm | 7.6424 | 4.4523 | 4.4523 |
| controlled_sgbm | 54.1476 | 36.5877 | 35.3652 |
| host_sgbm | 38.1481 | 21.1776 | 20.5353 |

`rtabmap_bm` reproduces recorded BM settings through Python OpenCV; it does not instrument the C++ mapper. The controlled BM/SGBM pair shares block size 15 and other available matching controls, but algorithm-specific controls still differ. `host_sgbm` uses block size 5. See [the protocol](EVALUATION.md) and the saved getters rather than interpreting these profiles as algorithm-only changes. The older 4.3% live figure came from a different sample and is not a paired baseline.

## Fixed-region and appearance probes

| Image variant | Profile | Final / full % | Final / common support % | Final / central 200×200 % |
|---|---|---:|---:|---:|
| original | rtabmap_bm | 4.4801 | 5.9187 | 7.5587 |
| original | controlled_bm | 4.4523 | 5.8819 | 7.5188 |
| original | controlled_sgbm | 35.3652 | 46.7209 | 57.6925 |
| original | host_sgbm | 20.5353 | 27.1292 | 34.9975 |
| teleBlurSigma1 | rtabmap_bm | 8.9289 | 11.7959 | 13.0400 |
| teleBlurSigma1 | controlled_bm | 8.9007 | 11.7587 | 12.9275 |
| teleBlurSigma1 | controlled_sgbm | 35.5791 | 47.0034 | 56.0250 |
| teleBlurSigma1 | host_sgbm | 22.7464 | 30.0502 | 37.3012 |
| claheBoth | rtabmap_bm | 3.8571 | 5.0956 | 8.2363 |
| claheBoth | controlled_bm | 3.8503 | 5.0866 | 8.2363 |
| claheBoth | controlled_sgbm | 21.4604 | 28.3513 | 33.8475 |
| claheBoth | host_sgbm | 14.0830 | 18.6050 | 21.8212 |

`teleBlurSigma1` applies Gaussian sigma 1 px to the rectified telephoto only. `claheBoth` uses clip limit 2 and 8×8 tiles on both views. These are declared probes, not an estimated optical PSF or fitted photometric correction. Matching runs on full images; the central crop evaluates the resulting mask with its search context preserved.

## Measured sampling asymmetry

| Quantity | Telephoto 20 | Ultrawide 21 |
|---|---:|---:|
| Native fx (px) | 1969.7695 | 564.6108 |
| Rectified fx at 640×480 (px) | 1102.6182 | 1102.6182 |
| Source x span of central output window (px) | 365.0501 | 102.8244 |
| Median output pixels / source pixel in center | 0.5603 | 1.9554 |

Both rectified focal lengths are equal, while source sampling differs. The data supports testing heterogeneous optics as a separate category. It does not establish that optics alone caused missing matches, nor exclude calibration, timing, occlusion, or texture effects. Blur increased BM coverage but changed controlled-SGBM central coverage from 57.6925% to 56.0250%; there is no universally favorable appearance probe here.

## Raw odometry functional check — reviewed 2026-09-19

The latest existing-bag replay (`ros-replay-cy8v1adw`) contains 222 exactly timestamp-matched Odometry/OdomInfo samples, all `lost=false`, over 29.7035 s. Of these, 221 have positive inliers and the initial identity origin has zero inliers. There are 10 connected map poses. Both process groups exited with code 0 without forced termination.

An earlier review inferred a lost/null pose from the console's `quality=0` line. The synchronized OdomInfo evidence did **not** support that inference: the initial pose reports `lost=false`. Do not discard the odometry origin simply for being zero. The recorder now retains same-stamp status evidence and excludes explicitly lost messages; a regression covers the exclusion policy. Neither finite quaternion checks nor console quality alone are a tracking-status contract.

The latest endpoint result is 3.6160 **estimated** cm and 19.5342°. Physical endpoint repeatability and measured scale are unavailable, so the result remains **not_assessable**. These are first-to-last accepted odometry-pose differences, not measured loop drift. Earlier run values remain in their original files and are not mixed into this run.

## Decision and remaining work

Keep the production configuration unchanged until a paired mapping/accuracy check supports changing it. Controlled SGBM is the next candidate because it retained more matches on this recorded sample, including the fixed central window. This is a candidate selection, not a declaration of correct geometry. The next physical step is the frozen 60–90 second marked-return protocol in EVALUATION.md, with measured scale and a known scene distance. No new capture is required to inspect or rerun these offline results.
