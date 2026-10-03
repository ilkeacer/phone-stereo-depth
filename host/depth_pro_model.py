"""Optional pinned Depth Pro adapter; downloads nothing and sends no images out."""
import importlib
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from host.arcore_confirmed_map import file_sha256

UPSTREAM_COMMIT = '9e65e4dbe9568d23c546fcec53302b10445e109e'
CHECKPOINT_SHA256 = '3eb35ca68168ad3d14cb150f8947a4edf85589941661fdb2686259c80685c0ce'


def validate_source(repository):
    repository = Path(repository).resolve()
    revision = subprocess.check_output(['git', '-C', str(repository), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', '-C', str(repository), 'status', '--porcelain', '--untracked-files=no'], text=True)
    # Include ignored files too: imports must use only the pinned source files.
    extra = subprocess.check_output(['git', '-C', str(repository), 'ls-files', '--others', '--', 'src'], text=True).splitlines()
    if revision != UPSTREAM_COMMIT or dirty.strip() or extra:
        raise ValueError('Expected clean pinned official Depth Pro source; src must have no extra files')
    return revision


class DepthProModel:
    def __init__(self, repository, checkpoint, device='cuda'):
        from dataclasses import replace
        import torch
        repository, checkpoint = Path(repository).resolve(), Path(checkpoint).resolve()
        revision = validate_source(repository)
        digest = file_sha256(checkpoint)
        if digest != CHECKPOINT_SHA256:
            raise ValueError('Expected official Depth Pro checkpoint hash')
        source = repository/'src'
        if 'depth_pro' in sys.modules and Path(sys.modules['depth_pro'].__file__).resolve() != source/'depth_pro/__init__.py':
            raise ValueError('Different Depth Pro package already loaded')
        sys.path.insert(0, str(source))
        old_bytecode_flag = sys.dont_write_bytecode
        try:
            sys.dont_write_bytecode = True
            module = importlib.import_module('depth_pro')
        finally:
            sys.dont_write_bytecode = old_bytecode_flag
        if Path(module.__file__).resolve() != source/'depth_pro/__init__.py':
            raise ValueError('Wrong Depth Pro package')
        if device not in ('cuda', 'cpu'):
            raise ValueError('Only CPU or CUDA supported')
        self.torch, self.device, self.rotation = torch, device, 0
        precision = torch.float16 if device == 'cuda' else torch.float32
        config = replace(module.depth_pro.DEFAULT_MONODEPTH_CONFIG_DICT, checkpoint_uri=str(checkpoint))
        self.model, self.transform = module.create_model_and_transforms(
            config=config, device=torch.device(device), precision=precision)
        self.model.eval()
        self.metadata = dict(model='Apple Depth Pro', upstreamCommit=revision,
            checkpointSha256=digest, license='Apple sample code license (upstream LICENSE)',
            outputUnits='model_predicted_metres', metricScaleValidated=False,
            device=device, precision=str(precision), torch=torch.__version__,
            inputSize=1536, inputRotationClockwise='per-frame recorded gravity',
            focalSource='recorded texture intrinsics transformed to CPU RGB')

    def predict(self, bgr, *, focal_px):
        from PIL import Image
        if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError('uint8 BGR image required')
        if not np.isfinite(focal_px) or focal_px <= 0:
            raise ValueError('Positive recorded RGB focal length required')
        if self.rotation not in (0, 90, 180, 270):
            raise ValueError('Clockwise quarter turn required')
        t = self.torch
        if self.device == 'cuda':
            t.cuda.synchronize()
        started = time.monotonic()
        rgb = np.ascontiguousarray(np.rot90(bgr[..., ::-1], -self.rotation//90))
        image = self.transform(Image.fromarray(rgb))
        with t.inference_mode():
            prediction = self.model.infer(image, f_px=t.tensor(float(focal_px), device=self.device))
        depth = prediction['depth'].float().cpu().numpy()
        depth = np.ascontiguousarray(np.rot90(depth, self.rotation//90))
        if self.device == 'cuda':
            t.cuda.synchronize()
        if depth.shape != bgr.shape[:2] or not np.isfinite(depth).all() or np.any(depth <= 0):
            raise ValueError('Invalid Depth Pro prediction')
        return depth, (time.monotonic()-started)*1000
