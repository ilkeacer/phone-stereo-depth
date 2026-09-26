# Reproducing the accepted screen result

Use the unmodified guided capture `data/guided/20260910_122808`. Its30 training/10 validation pairs are private local data, not in the source ZIP. Validation used a separately restarted Android camera session. Final refinement retained27/10 poses.

```bash
OPENBLAS_NUM_THREADS=1 OPENCV_FOR_THREADS_NUM=2 .venv/bin/python -m host.calibrate \
  data/guided/20260910_122808/training \
  --validation-trial data/guided/20260910_122808/validation \
  --scale-free --output data/calibration/screen_20260910

.venv/bin/python -m host.depth data/calibration/screen_20260910/calibration.npz \
  data/guided/20260910_122808/validation/camera_20_631_84160510502414.jpg \
  data/guided/20260910_122808/validation/camera_21_629_84160509351088.jpg \
  --output data/depth/screen-sgbm

.venv/bin/python -m host.raft \
  --left data/depth/screen-sgbm/rectified-left.png \
  --right data/depth/screen-sgbm/rectified-right.png \
  --consistency --output data/depth/screen-raft
```

The sample was selected by smallest timestamp difference among held-out poses (1.151 ms), not by depth appearance or epipolar residual. Calibration output is horizontal in the original camera coordinates. UI-only90° rotation also rotates the depth/mask display; the calibrated processing coordinates stay unchanged.

The guided target's stored location masks out the second checkerboard visible inside the computer's live-preview image. SB refinement must stay within3 pixels mean displacement of the recorded target corners. The same condition is applied to all training and validation images. Guided startup settling uses camera frame sequence≥60, rather than erroneously treating the first saved pose as camera startup. These corrections preceded final acceptance. The uncorrected global detector produced invalid geometry; the intermediate26/9 subset gave p95=1.53px. The final27/10 dataset with both corrections gives p95=1.451px under the original1.5px gate. No gate was loosened and no validation residual was used to reject an individual pose.

A four-model diagnostic used only training pose-block cross-validation to compare fixed/free k3 and fixed/joint intrinsics. It preferred the same default model. Final reported geometry uses the default `host.calibrate` command above, not a model chosen to minimize the external validation error.

## Optional GPU dependencies

```bash
.venv/bin/pip install torch==2.10.0 torchvision==0.25.0 --index-url https://download.pytorch.org/whl/cu128
.venv/bin/pip install scipy==1.15.3 tqdm==4.67.1 opt_einsum==3.4.0
# Official upstream; inspect its LICENSE and download_models.sh.
git clone https://github.com/princeton-vl/RAFT-Stereo work/RAFT-Stereo
git -C work/RAFT-Stereo checkout 6e93ed2169bd858dbb43033988563f3b0bb49506
```

Official pretrained checkpoint source and SHA256 are in `docs/evidence/model-provenance.json` and `raft-weights.sha256`. The tested checkpoint is `data/models/raftstereo-middlebury.pth`; it is not redistributed. The repository is MIT-licensed; a separate explicit license for the linked weight archive was not found. CUDA13 shown by nvidia-smi is not a toolkit-installation claim; the tested PyTorch runtime is CUDA12.8.

## Remaining physical measurement

For metres, measure **four adjacent squares on the same monitor-rendered target**, at the recorded151 pixels per square and2560×1600 display mode, and record the actual total mm. The earlier printed A4 target has a different scale and must not be substituted. Divide the measured total by4, then rerun calibration with `--square-mm` instead of `--scale-free`.

After scale is known, measure static targets near0.5,1 and2m from the left-camera optical reference; these are proposed test distances, not a verified operating range. Record absolute/relative error, invalid-pixel ratio and temporal variation. A target distance is not full-scene ground-truth depth. Moving-target and end-to-end latency tests remain separate work. Point-cloud export is deferred until basic metric measurements are validated.

Create the comparison image and NumPy/JSON result bundle with:

```bash
.venv/bin/python -m scripts.export_screen_results --output /path/to/stereo-result
scripts/start_viewer.sh --calibration data/calibration/screen_20260910/calibration.npz --results /path/to/stereo-result
```

The current viewer intentionally supports `checker_square` only. Its metre UI and distance tests must be updated once actual scale is measured; it refuses to silently mislabel a metre calibration.

## Independent live receiver (stage 1)

```bash
scripts/start_viewer.sh --live
.venv/bin/python -m unittest discover -s tests -v
```

Unlock the phone and leave the probe activity in the foreground. The host starts the same fixed 1280×960, focus1.4, ISO400 capture as before. The first four seconds after receiving a new camera session are preview-only settling time; calibration geometry is checked before any depth is admitted. Subsequent focus/crop changes stop the pipeline.

`host/transport.py` contains the bounded USB decoder without Tk or calibration imports. `host/pipeline.py` continuously receives snapshots while a separate worker computes SGBM. Only one pair waits for computation; newer eligible pairs replace pending work. Both camera timestamps must advance. Preview, depth and status have independent last-value channels. Reconnect/session changes invalidate pending and in-flight results. Device startup can be cancelled and the Tk window remains responsive while cleanup completes.

Each live attempt writes numeric `events.jsonl` and a final `session.json` in a timestamped subdirectory of `data/live/`. Received, eligible, replaced, duplicate, unpaired, stale and completed counts are separate. The summary includes whole-session depth Hz, the largest internal gap, leading/trailing gaps, and medians over the latest512 completed results. Logs contain local session identifiers but no JPEG payloads.

`callbackAgeUpperMs` conservatively combines phone callback-to-snapshot age with the entire host request/processing interval. It does **not** include the unknown sensor-to-callback delay and must not be called exposure-to-display latency. Results older than one second by this callback-age estimate are rejected; displayed numeric values expire using the same age estimate.

Android's v1 snapshot selection is unchanged in this stage. It may omit older eligible pairs; the receiver cannot recover frames the phone never transmits. This change is not hardware synchronization, live RAFT, point-cloud generation or ROS/SLAM. Those are subsequent packages in `docs/architecture/LIVE_MAPPING_V1_TR.md`.

## September11 timing profile and live capture

The opt-in 15/30 profile leaves camera20 at15FPS and requests the advertised fixed30FPS range and33,333,334ns period on camera21:

```bash
scripts/start_viewer.sh --live --wide-fps 30
```

The default remains `--wide-fps 15`. An updated APK is required for the30FPS option; older APKs do not implement these intent extras. Rebuild/install with `scripts/build.sh testDebugUnitTest` and `adb install -r android/app/build/outputs/apk/debug/app-debug.apk`.

Live, non-archival capture now uses `ImageReader.acquireLatestImage` on a separate image handler. Capture metadata stays on the camera handler so JPEG work cannot starve exact timestamp joins. `saveAll` and non-live capture retain sequential image acquisition. Consequently, live image sequence counts describe acquired images, not every exposure; sensor timestamp intervals reveal dropped images. Pixel-stride1 planes use row-wise bulk copying with original buffer positions preserved.

Updated packets carry `timestampSource`. When both cameras declare Android REALTIME, the host can compare source timestamps with the phone snapshot's elapsed time. `sensorAgeUpperMs` includes source-to-snapshot time plus the entire host read/processing interval; this is a conservative completion-age bound, not measured display latency. `freshnessAgeUpperMs` uses this bound when available and falls back to callback age for old/unknown-source packets. Neither policy modifies source timestamps. The1s freshness and20ms stereo-delta limits remain unchanged.

Audit existing numeric phone logs independently of the viewer:

```bash
.venv/bin/python -m host.timing /path/to/trial --output /path/to/timing.json
```

The directory must contain both `camera_20_images.jsonl`, `camera_21_images.jsonl` and their corresponding metadata JSONL files. The audit reports all recorded image timestamps and exact-metadata subsets separately. It checks optimistic pair existence, allowing reuse for this diagnostic upper bound; its eligible-left count is not a count of unique processed stereo pairs. Incomplete final records after a force-stop are counted explicitly. Interior corruption is rejected.

Primary references: [ImageReader acquisition semantics](https://developer.android.com/reference/android/media/ImageReader), [Android timestamp clock domains](https://developer.android.com/reference/android/hardware/camera2/CameraMetadata), [Camera2 frame-duration controls](https://developer.android.com/reference/android/hardware/camera2/CaptureRequest).


### Recorded quality diagnostics (stage3)

Run from project root: `.venv/bin/python -m host.quality --calibration data/calibration/screen_20260910/calibration.npz --output work/quality-stage3/offline`. Requires private accepted held-out captures and anchored corner cache. Saves numeric quality.json and private first-pair images under ignored work; never publish that directory. Viewer “Çift kontrolü” displays the exact depth pair. Live defaults stay unchanged. See docs/STAGE3_RESULTS_TR.md for experimental limits.


### Half-resolution depth (stage4)

Offline corner comparison: `.venv/bin/python -m host.corner_quality --calibration data/calibration/screen_20260910/calibration.npz --output work/quality-stage4/corner-comparison.json`. Live opt-in: `scripts/start_viewer.sh --live --wide-fps 30 --depth-scale 0.5`. Source stays1280x960; output640x480; default depth-scale1. Correlated checker reference is not metric ground truth. See docs/STAGE4_RESULTS_TR.md.


### Single-frame point cloud (stage5)

Run the640x480viewer and use “Nokta bulutunu kaydet”. Private bundles are written to data/pointcloud/snapshot_*/cloud.ply with metadata.json. CLI: `python -m host.pointcloud --help`. Unit checker_square, rectified left optical XYZ; no accumulated map, no measured metric accuracy. See docs/STAGE5_RESULTS_TR.md.


### Interactive saved cloud (stage6)

`python -m host.cloud_view <snapshot-directory>` opens a separate Matplotlib/Tk process. Or use “Kaydedilmiş bulutu3B aç” in Viewer. Requires our binaryPLYplus metadata bundle. Display samples at most12,000points and optionally limits Z; file remains unchanged. No live map or measured metric accuracy. See docs/STAGE6_RESULTS_TR.md.


### Optional small-disparity cloud rejection (stage7)

Use “Uzak noktaları eleyip kaydet” or add `--min-disparity 8` to host.pointcloud. Threshold uses original calibrated pixel units (4pixels at halfresolution). CLIdefaults and normal export remain unchanged. Evaluate checker correspondence with `host.corner_quality --min-disparity 8`. Counts/threshold saved under provenance.cloudFilter. Not calibrated confidence. See docs/STAGE7_RESULTS_TR.md.
# Hareket tutarlılığı (kayıtlı telefon verisi)

Her komut için yeni bir çıktı adı kullanın; mevcut kanıtların üzerine yazılmaz:

```sh
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m host.motion_audit \
  --trial data/raw/run_1788989043241/manual_20-21_vga_1788989043390 \
  --calibration data/calibration/screen_20260910/calibration.npz \
  --start-pair 0 --limit 300 --output work/motion-stage10/recheck-904.json

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m host.motion_summary \
  work/motion-stage10/recheck-904.json \
  --output work/motion-stage10/recheck-summary.json
```

Bu çıktı iç tutarlılık tanısıdır; gerçek yörünge veya metre doğruluğu değildir.
