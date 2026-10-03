# Public source release scope

Use `scripts/export_public.py --output /path/to/source-only.zip` for the reviewed public source set. The export uses an explicit directory/document allowlist, permits only text source formats, rejects symlinks and excludes local experiment data. The complete local `outputs/` folder is NOT a public upload package.

Before updating GitHub, stage only paths returned by `public_files()` in `scripts/export_public.py`, then run `python3 scripts/check_public_tree.py` and `git diff --cached --check`. The tree check rejects any tracked path outside that allowlist and any index content that differs from the audited working tree. Compare the Git tree and the remote tree after upload. Keep the GitHub repository private unless the owner explicitly requests public visibility.

Excluded: all captured/rectified photos, desktop screenshots, video, disparity/depth arrays, point clouds, ROS bags/databases, per-frame logs, model weights, SDK/build products, local Android paths, Git history and private continuation notes. Numerical aggregate measurements are included with a statement that private raw evidence is not distributed.

No code in the public archive transmits the private dataset. Reproducing the real-device measurements requires collecting one's own dataset; the synthetic unit tests run without the author's camera photos. The local dataset is retained separately and has not been deleted or uploaded.

The owner intends the final project to be fully open source, but has not selected a project-wide license yet. Public hosting and permission to reuse code are separate decisions. Choose a project license before advertising the repository as licensed open source. Direct Ultralytics code/model integration would require accounting for its AGPL-3.0 terms; a detector with permissive code and separately verified weight terms is another option. Third-party software/checkpoint terms remain separate; pretrained weights are not redistributed.

The repository includes a working recorded/live ROS input and RTAB-Map export pipeline. It remains a research prototype: room accuracy, scale and robot-mounted navigation are unvalidated. See `EVALUATION.md` and `ROS_3D_MAPPING_TR.md` for evidence and acceptance boundaries. The GitHub workflow has been prepared; a remote Actions run has not been performed.
