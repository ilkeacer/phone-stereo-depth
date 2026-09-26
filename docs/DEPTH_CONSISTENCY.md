# Recorded depth consistency

`host.depth_consistency` compares SGBM and Depth Anything caches aligned to identical rectified images. It opens no camera and requires a new output directory. Existing calibration and acceptance gates are unchanged.

```bash
.venv/bin/python -m host.depth_consistency \
  --stereo work/my-sgbm-cache --model work/my-model-cache \
  --session work/my-completed-hybrid-session --output work/my-new-audit
```

The session must supply the replay provenance, capture/lifecycle records, raw odometry and same-stamp OdomInfo status. The evaluator checks the `odom -> phone_link` frames and rejects trajectories containing a lost status. It uses exact left-time poses where available, otherwise bracketed linear translation and shortest SO(3) rotation interpolation. Brackets over 250 ms and extrapolation are rejected. This time support limit is specific to the audit; it is not a new calibration threshold or an assertion that the mapper applies the same limit.

Training uses every tenth frame in even 30-frame blocks. A single scalar is the median of per-frame medians of SGBM/model depth ratios. Evaluation uses anchors at block offsets 5, 15 and 25 in odd blocks, with targets four frames before and after within the same block. This leaves the evaluated frames out of the scalar fit, but does not constitute independent-scene validation.

Shi–Tomasi/LK image tracks use 800 maximum corners, quality 0.01, minimum distance 8 px, 21×21 LK windows, three pyramid levels, and ≤1 px forward/backward error. Track selection does not use any depth result. All three methods share these tracks; no reprojection-error rejection is applied afterwards. Source depth is projected with raw stereo motion into the target camera. Pixel error is measured against the optical track. Motion-compensated depth disagreement is `abs(projected_Z - target_Z) / target_Z`. Nearest-pixel depth sampling is explicit. Flow errors, occlusions, moving objects and pose errors contribute; this is consistency, not ground-truth accuracy.

On the existing 226-pair/29.97-second recording, 12 training frames and 20 evaluated pairs produced 1,513 optical tracks. One pair lacked bracketing pose support at the recording end. SGBM supplied finite source depth on 800 tracks; both model orientations supplied all 1,513. The comparison below uses the same 800 reprojections and the same 713 tracks with source and target depth in all methods. The extra model-only tracks are not silently counted as validated geometry.

| Method | Reprojection median / p95, px | Depth disagreement median / p95, % |
|---|---:|---:|
| SGBM | 0.2533 / 2.5551 | 0.5773 / 3.9059 |
| Model, 90° input | 0.4133 / 3.9106 | 4.6994 / 22.2171 |
| Model, 90°, one training-fit scalar | 0.3748 / 4.2490 | 4.7016 / 22.0819 |
| Model, 0° input | 0.3497 / 7.2273 | 9.5129 / 36.7079 |
| Model, 0°, one training-fit scalar | 0.2996 / 5.9867 | 9.5341 / 36.6057 |

The 90° scalar was 0.69924894. Its median and p95 reprojection effects differ, so it was not applied to mapping or production settings. The reference poses themselves come from stereo, favoring neither a claim of independent SGBM validation nor a claim of model accuracy. Measured scene scale is still unavailable.

Each run preserves `protocol.json`, `scale-fit.json`, full `summary.json`, every pair's statistics and NPZ arrays (tracks, relative transform, errors and masks), plus explicit before/after hashes for 462 inputs. Command arguments and library versions are recorded. Failed/intermediate runs remain separate. Public source archives exclude these private scene arrays.

The audit exposed a separate delivery issue: equal-stamp RGB-D/odometry synchronization omitted the interval where right exposure time was later than left. [Hybrid mapping](RGBD_MAPPING.md) now queries the stereo TF at left-image time, while retaining exact image synchronization. This restores the omitted interval in the recorded replay; it does not validate whole-room geometry or tracking resets.
