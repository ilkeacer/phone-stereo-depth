#!/usr/bin/env python3
"""Run one foreground experiment and pull its own directory; preserve unrelated apps."""
import argparse,datetime,json,subprocess,time
from pathlib import Path

PACKAGE='org.research.phonestereo'
BASE=f'/sdcard/Android/data/{PACKAGE}/files'

def adb(*args,check=True):
    return subprocess.run(['adb',*args],check=check,text=True,capture_output=True)

def runs():
    r=adb('shell','ls',BASE,check=False)
    return set(r.stdout.split()) if r.returncode==0 else set()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--ids',help='Comma-separated IDs; omit for bounded automatic matrix')
    ap.add_argument('--seconds',type=int,default=65)
    ap.add_argument('--light',action='store_true')
    ap.add_argument('--fixed',action='store_true')
    ap.add_argument('--width',type=int,default=0)
    ap.add_argument('--height',type=int,default=0)
    ap.add_argument('--iso',type=int,default=400)
    ap.add_argument('--focus',type=float,default=2.0)
    ap.add_argument('--calibration-guide',action='store_true')
    ap.add_argument('--save-all',action='store_true')
    ap.add_argument('--lens-test',action='store_true')
    ap.add_argument('--timeout',type=int,default=900)
    args=ap.parse_args()
    before=runs()
    adb('shell','am','force-stop',PACKAGE)
    command=['shell','am','start','-n',f'{PACKAGE}/.MainActivity']
    if args.ids:command+=['--es','ids',args.ids,'--ei','seconds',str(args.seconds)]
    if args.light:command+=['--ez','light','true']
    if args.fixed:command+=['--ez','fixed','true','--ei','iso',str(args.iso),'--ef','focus',str(args.focus)]
    if args.width:command+=['--ei','width',str(args.width),'--ei','height',str(args.height)]
    if args.calibration_guide:command+=['--ez','calibrationGuide','true']
    if args.save_all:command+=['--ez','saveAll','true']
    if args.lens_test:command+=['--ez','lensTest','true']
    print(adb(*command).stdout,flush=True)
    deadline=time.monotonic()+args.timeout; name=None
    while time.monotonic()<deadline:
        added=runs()-before
        if added:name=sorted(added)[-1]
        if name and adb('shell','test','-f',f'{BASE}/{name}/ABORTED',check=False).returncode==0:
            raise SystemExit(f'Probe aborted: {BASE}/{name}/ABORTED; inspect events.')
        if name and adb('shell','test','-f',f'{BASE}/{name}/DONE',check=False).returncode==0:break
        time.sleep(2)
    else:raise SystemExit('Timed out. Check phone camera permission and foreground app; partial data remains on phone.')
    target=Path('data/raw');target.mkdir(parents=True,exist_ok=True)
    print(adb('pull',f'{BASE}/{name}',str(target)).stdout)
    print(target/name)
if __name__=='__main__':main()
