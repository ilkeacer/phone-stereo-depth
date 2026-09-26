"""Official RAFT-Stereo wrapper. Requires rectified input; emits pixel disparity only."""
import argparse,json,sys,time,hashlib,subprocess
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import cv2
import torch
from host.depth import consistency_mask


def load_model(repo,checkpoint):
    sys.path.insert(0,str(repo.resolve()))
    sys.path.insert(0,str(repo.resolve()/'core'))
    from raft_stereo import RAFTStereo
    from utils.utils import InputPadder
    args=SimpleNamespace(hidden_dims=[128,128,128],corr_implementation='reg',shared_backbone=False,
        corr_levels=4,corr_radius=4,n_downsample=2,context_norm='batch',slow_fast_gru=False,n_gru_layers=3,mixed_precision=False)
    model=RAFTStereo(args)
    state=torch.load(checkpoint,map_location='cpu',weights_only=True)
    model.load_state_dict({k.removeprefix('module.'):v for k,v in state.items()},strict=True)
    return model.cuda().eval(),InputPadder


def predict(model,padder_class,left,right,iters):
    tensors=[torch.from_numpy(np.ascontiguousarray(im[:,:,::-1])).permute(2,0,1)[None].float().cuda() for im in [left,right]]
    padder=padder_class(tensors[0].shape,divis_by=32);a,b=padder.pad(*tensors)
    torch.cuda.synchronize();start=time.perf_counter()
    with torch.inference_mode():_,flow=model(a,b,iters=iters,test_mode=True)
    torch.cuda.synchronize();ms=(time.perf_counter()-start)*1000
    disparity=-padder.unpad(flow)[0,0].cpu().numpy()
    return disparity,ms


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--repo',type=Path,default=Path('work/RAFT-Stereo'))
    ap.add_argument('--checkpoint',type=Path,default=Path('data/models/raftstereo-middlebury.pth'))
    ap.add_argument('--left',type=Path);ap.add_argument('--right',type=Path);ap.add_argument('--smoke',action='store_true')
    ap.add_argument('--iters',type=int,default=16);ap.add_argument('--max-width',type=int,default=640)
    ap.add_argument('--consistency',action='store_true',help='Also infer mirrored reverse direction and mask inconsistent matches')
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    if not torch.cuda.is_available():raise SystemExit('CUDA unavailable')
    if args.smoke:
        rng=np.random.default_rng(42);left=rng.integers(0,255,(128,192,3),np.uint8);right=np.roll(left,-4,axis=1)
    else:
        if not args.left or not args.right:raise SystemExit('Rectified left/right inputs required')
        left=cv2.imread(str(args.left));right=cv2.imread(str(args.right))
        if left is None or right is None or left.shape!=right.shape:raise SystemExit('Missing or differently sized images')
    original=left.shape[:2];scale=min(1,args.max_width/left.shape[1])
    if scale<1:left,right=[cv2.resize(im,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA) for im in [left,right]]
    model,padder=load_model(args.repo,args.checkpoint)
    torch.cuda.reset_peak_memory_stats()
    _,cold=predict(model,padder,left,right,args.iters)
    output,ms=predict(model,padder,left,right,args.iters)
    reverse=None;reverse_ms=None
    if args.consistency:
        flipped,reverse_ms=predict(model,padder,np.fliplr(right),np.fliplr(left),args.iters)
        reverse=-np.fliplr(flipped).copy()
    # Restore disparity to original calibrated pixel coordinates, scaling magnitude too.
    if scale<1:output=cv2.resize(output,(original[1],original[0]),interpolation=cv2.INTER_LINEAR)/scale
    if reverse is not None and scale<1:reverse=cv2.resize(reverse,(original[1],original[0]),interpolation=cv2.INTER_LINEAR)/scale
    finite=bool(np.isfinite(output).all())
    if not finite:raise SystemExit('Non-finite disparity output')
    args.output.mkdir(parents=True,exist_ok=True)
    np.save(args.output/'disparity_px.npy',output)
    valid=None
    if reverse is not None:
        valid=consistency_mask(output,reverse)
        np.save(args.output/'right_disparity_px.npy',reverse)
        cv2.imwrite(str(args.output/'valid-mask.png'),valid.astype(np.uint8)*255)
    report=dict(model='RAFT-Stereo Middlebury',repoCommit=subprocess.check_output(['git','-C',str(args.repo),'rev-parse','HEAD'],text=True).strip(),
        checkpointSha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),torch=torch.__version__,cudaRuntime=torch.version.cuda,
        device=torch.cuda.get_device_name(0),smokeTest=args.smoke,accuracyEvaluated=False,finite=finite,
        iterations=args.iters,inputScale=scale,outputShape=list(output.shape),coldForwardMs=cold,warmForwardMs=ms,
        reverseForwardMs=reverse_ms,leftRightThresholdPx=(1 if args.consistency else None),consistencyValidRatio=(float(valid.mean()) if valid is not None else None),
        peakAllocatedMiB=torch.cuda.max_memory_allocated()/1024**2,units='original input pixels',metricDepth=False,endToEndLatencyMeasured=False)
    (args.output/'metrics.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
