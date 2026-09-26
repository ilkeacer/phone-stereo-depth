"""Export audited text sources only. Never include private camera/derived scene data."""
import argparse,hashlib,json,re,zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SOURCE_DIRS=('android','host','scripts','tests','configs','.github')
DOCS=('README.md','.gitignore','docs/EVALUATION.md','docs/DEPTH_ANYTHING.md','docs/DEPTH_CONSISTENCY.md','docs/ONLINE_SGBM.md','docs/RGBD_MAPPING.md','docs/BENCHMARK_RESULTS.md','docs/ROS_3D_MAPPING_TR.md','docs/DESKTOP_ASSISTANT.md','docs/REPRODUCE.md',
      'docs/RESULTS_TR.md','docs/ROS_SLAM.md','docs/PUBLICATION.md','docs/ROS_WIFI_TR.md','docs/LIVE_3D_STATUS_TR.md','docs/ROS_RECOVERY_SEGMENTS_TR.md',
      'docs/evidence/public-measurements.json','docs/evidence/stereo-benchmark.json','docs/evidence/depth-anything-benchmark.json','docs/evidence/model-provenance.json',
      'docs/evidence/raft-weights.sha256')
TEXT_SUFFIXES={'.py','.kt','.gradle','.properties','.xml','.md','.json','.sh','.txt','.sha256','.yml','.yaml','.rviz'}
EXCLUDE_PARTS={'build','.gradle','__pycache__','data','work','.git','.venv','outputs'}
EXCLUDE_FILES={'local.properties','export_evidence.py'}

def public_files(root=ROOT):
    files=[root/name for name in DOCS]
    for directory in SOURCE_DIRS:
        files.extend((root/directory).rglob('*'))
    result=[]
    for path in sorted(set(files)):
        relative=path.relative_to(root)
        if any(part in EXCLUDE_PARTS for part in relative.parts) or path.name in EXCLUDE_FILES:continue
        if path.is_symlink():raise ValueError(f'Symlink excluded: {relative}')
        if not path.is_file():continue
        if path.suffix not in TEXT_SUFFIXES and path.name!='.gitignore':continue
        text=path.read_text(encoding='utf-8')
        if '\x00' in text:raise ValueError(f'Non-text payload: {relative}')
        if re.search(r'/home/[A-Za-z0-9_.-]+|-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----|data:image/[^;]+;base64,',text):
            raise ValueError(f'Private path or embedded payload requires review: {relative}')
        result.append(path)
    return result

def write_archive(output,files,root=ROOT):
    manifest=[]
    with zipfile.ZipFile(output,'x',zipfile.ZIP_DEFLATED) as archive:
        for path in files:
            data=path.read_bytes();relative=path.relative_to(root).as_posix()
            info=zipfile.ZipInfo.from_file(path,relative)
            info.compress_type=zipfile.ZIP_DEFLATED
            archive.writestr(info,data)
            manifest.append({'path':relative,'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()})
    return manifest

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    files=public_files();args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.output.with_suffix('.manifest.json').exists():raise FileExistsError('Manifest already exists')
    manifest=write_archive(args.output,files)
    with zipfile.ZipFile(args.output) as archive:
        assert archive.testzip() is None
    report={'kind':'public text-source-only archive','fileCount':len(manifest),'includesCapturedImages':False,
        'includesDerivedSceneArrays':False,'includesRawLogsOrGitHistory':False,
        'scope':'explicit allowlist; all entries decoded as text; symlinks excluded; paths/embedded image/private-key markers checked',
        'archiveSha256':hashlib.sha256(args.output.read_bytes()).hexdigest(),'files':manifest}
    with args.output.with_suffix('.manifest.json').open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k!='files'},indent=2))
if __name__=='__main__':main()
