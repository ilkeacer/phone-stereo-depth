"""Guided, bounded real-phone motion capture. Images stay in local ignored data/."""
import json
import os
import fcntl
import hashlib
import io
import shutil
import uuid
from pathlib import Path
import queue
import subprocess
import threading
import time
import tkinter as tk
from PIL import Image
from host.contracts import ReceivedPair,check_geometry
from host.device import timing_args,run_command,select_adb_serial
from host.transport import read_pair
from host.motion_guide import CaptureGuide,STEPS
from host.motion_capture_ui import GuideView,latest
from host.motion_recording import atomic_json


PROJECT=Path(__file__).resolve().parents[1]
PACKAGE='org.research.phonestereo'
def authorized_devices(output):
    devices=[]
    for line in output.splitlines()[1:]:
        fields=line.split()
        if len(fields)>=2 and fields[1]=='device':devices.append(fields[0])
    return devices


def packet_reason(pair,report,size,source,last_timestamps,now):
    if source is not None and pair.source!=source:return 'source_changed'
    try:check_geometry(pair.header,report,size)
    except (KeyError,TypeError,ValueError):return 'geometry_mismatch'
    if not pair.header.get('paired'):return 'unpaired'
    if last_timestamps is not None and any(new<=old for old,new in zip(last_timestamps,pair.timestamps)):
        return 'non_increasing_timestamp'
    if pair.age_upper(now)>1:return 'stale'
    return None


class CaptureWriter:
    def __init__(self,directory,calibration_report,provenance=None,*,max_frames=600,max_bytes=128*1024*1024):
        if type(max_frames) is not int or type(max_bytes) is not int or min(max_frames,max_bytes)<1:raise ValueError('Positive recording resource limits required')
        self.directory=directory;directory.mkdir(parents=True,exist_ok=False)
        self.report=calibration_report;self.frames=0;self.phase_counts={};self.rejects={}
        self.source=None;self.last_timestamps=None
        self.bytes=0;self.provenance=provenance or {};self.events=[]
        self.committed=[]
        self.max_frames=max_frames;self.max_bytes=max_bytes
        atomic_json(directory/'committed-pairs.json',self.committed)

    def reject(self,reason):self.rejects[reason]=self.rejects.get(reason,0)+1

    def save(self,pair,phase,host_monotonic):
        incoming=sum(len(blob) for blob in pair.blobs)
        if self.bytes+incoming>self.max_bytes or self.frames>=self.max_frames:
            raise RuntimeError('Kayıt sınırına ulaşıldı. Mevcut görüntüler korundu.')
        if shutil.disk_usage(self.directory).free<incoming+64*1024*1024:
            raise RuntimeError('Kayıt için boş disk alanı azaldı. Mevcut görüntüler korundu.')
        names=[];cameras={}
        for camera,blob in zip(('20','21'),pair.blobs):
            image=dict(pair.header[camera]['image'])
            name=f"camera_{camera}_{image['sequence']}_{image['imageTimestampNs']}.jpg"
            with (self.directory/name).open('xb') as stream:stream.write(blob)
            image['sampleFile']=name;names.append(name)
            cameras[camera]=dict(image=image,capture=pair.header[camera]['capture'])
            for kind,value in (('images',image),('metadata',pair.header[camera]['capture'])):
                with (self.directory/f'camera_{camera}_{kind}.jsonl').open('a') as stream:
                    stream.write(json.dumps(value,allow_nan=False)+'\n')
        row=dict(frameIndex=self.frames,phase=phase,sourceRun=pair.source,
                 sourceTimestampsNs=list(pair.timestamps),sampleFiles=names,
                 stereoDeltaMs=abs(pair.timestamps[0]-pair.timestamps[1])/1e6,
                 hostReadStartedMonotonic=pair.read_started,hostReceivedMonotonic=pair.received,
                 hostSavedMonotonic=host_monotonic,
                 callbackAgeUpperMs=pair.callback_age_upper(host_monotonic)*1000,
                 freshnessAgeUpperMs=pair.age_upper(host_monotonic)*1000,
                 deviceElapsedNs=pair.header.get('deviceElapsedNs'))
        with (self.directory/'frames.jsonl').open('a') as stream:
            stream.write(json.dumps(row,allow_nan=False)+'\n')
        committed_row=dict(row,cameras=cameras)
        atomic_json(self.directory/'committed-pairs.json',self.committed+[committed_row])
        self.committed.append(committed_row)
        self.frames+=1;self.phase_counts[phase]=self.phase_counts.get(phase,0)+1
        self.bytes+=incoming
        self.source=pair.source;self.last_timestamps=pair.timestamps

    def finish(self,status,error=None):
        manifest=dict(status=status,error=error,frames=self.frames,phaseCounts=self.phase_counts,
            schemaVersion=2,guideMode='user-paced stages; admitted contiguous packet time',
            jpegBytes=self.bytes,provenance=self.provenance,guideEvents=self.events,
            rejectedSnapshots=self.rejects,sourceRun=self.source,ids=['20','21'],sourceSize=[1280,960],
            requestedWideFps=30,requestedTeleFps=15,focusDioptersRequested=1.4,isoRequested=400,
            metricScaleVerified=False,unit='checker_square',
            phaseAssignment='host receipt time; freshness upper bound stored per frame; not sensor-exact phase boundary',
            scope='controlled motion input for offline diagnostics; not motion ground truth')
        atomic_json(self.directory/'manifest.json',manifest)


class MotionCaptureApp(GuideView):
    def __init__(self,root):
        self.root=root;self.stop=threading.Event();self.worker=None;self.closing=False
        self.adb_serial=None
        self.messages=queue.Queue(maxsize=1);self.preview_queue=queue.Queue(maxsize=1)
        self.commands=queue.Queue(maxsize=4)
        self.build();root.after(60,self.poll)

    def post(self,kind,message,guide=None,frames=0):
        latest(self.messages,dict(kind=kind,message=message,guide=guide.snapshot() if guide else None,frames=frames))

    def action(self):
        if self.closing:return
        if self.worker is None or not self.worker.is_alive():
            self.stop=threading.Event()
            self.commands=queue.Queue(maxsize=4)
            self.post('connecting','Telefon açık ve kilitsiz olsun. Kamera odağı yerleşince önizleme hazır olacak.')
            self.worker=threading.Thread(target=self.capture,daemon=True);self.worker.start()
            return
        state=self.view_state or {};guide=state.get('guide')
        if guide and (state['kind']=='ready' or guide['active']):
            try:self.commands.put_nowait(('pause' if guide['active'] else 'start',guide['revision']))
            except queue.Full:pass

    def adb(self,*args):
        prefix=['adb','-s',self.adb_serial] if self.adb_serial and args[0]!='devices' else ['adb']
        return run_command([*prefix,*args],self.stop)

    def capture(self):
        writer=None;configured=False;guide=CaptureGuide()
        final_kind='cancelled';final_message='Kayıt durduruldu.'
        try:
            calibration=PROJECT/'data/calibration/screen_20260910/calibration.npz'
            sidecar=calibration.with_suffix('.json')
            report=json.loads(sidecar.read_text())
            provenance={'calibrationSha256':hashlib.sha256(calibration.read_bytes()).hexdigest(),
                        'calibrationReportSha256':hashlib.sha256(sidecar.read_bytes()).hexdigest()}
            devices=authorized_devices(self.adb('devices','-l').stdout)
            self.adb_serial=select_adb_serial(devices,os.environ.get('PHONE_ADB_SERIAL'))
            self.adb('shell','am','force-stop',PACKAGE)
            configured=True
            self.adb('forward','tcp:8765','tcp:8765')
            self.adb('shell','am','start','-n',f'{PACKAGE}/.MainActivity','--es','ids','20,21','--ei','seconds','660',
                     '--ei','width','1280','--ei','height','960','--ez','fixed','true','--ez','live','true',
                     '--ef','focus','1.4','--ei','iso','400',*timing_args(30))
            beginning=time.monotonic();settling_started=beginning;source=None;last_seen=None
            ready=False;good_since=None;last_good=beginning;last_save=0.;last_preview=0.;last_update=0.
            while not self.stop.wait(.04):
                now=time.monotonic()
                if now-beginning>600:raise RuntimeError('Oturum süresi doldu. Kaydınız korundu; yeniden bağlanabilirsiniz.')
                if (not ready and now-beginning>20) or (ready and now-last_good>10):
                    raise RuntimeError('Güncel ve kalibrasyonla uyumlu görüntü alınamadı. Telefonu açık tutup yeniden bağlanın.')
                # Commands are processed before a blocking read; stop/pause remains bounded.
                try:
                    while True:
                        action,revision=self.commands.get_nowait()
                        if action=='start' and (not ready or now-last_good>1):continue
                        if guide.command(action,revision) and writer:
                            writer.events.append(dict(action=action,step=guide.index,hostMonotonic=now))
                except queue.Empty:pass
                try:
                    started=time.monotonic();packet=read_pair();received=time.monotonic()
                    if packet is None:raise ConnectionError('Waiting')
                    pair=ReceivedPair.create(packet,started,received)
                except (ConnectionError,OSError,TimeoutError,ValueError):
                    guide.interrupt();good_since=None
                    self.post('waiting' if ready else 'connecting','Telefon görüntüsü bekleniyor. Ekran kilidini ve USB bağlantısını kontrol edin.',guide if ready else None)
                    continue
                if received-last_preview>=.16:
                    images=[]
                    for blob in pair.blobs:
                        with Image.open(io.BytesIO(blob)) as im:
                            im.thumbnail((700,500));images.append(im.convert('RGB'))
                    expires=received+max(0.,1.-pair.age_upper(received))
                    latest(self.preview_queue,(images,received,abs(pair.timestamps[0]-pair.timestamps[1])/1e6,expires))
                    last_preview=received
                if not ready and source is not None and pair.source!=source:
                    source=None;last_seen=None;good_since=None;settling_started=received
                reason=packet_reason(pair,report,(1280,960),source,last_seen,received)
                if reason:
                    if writer:writer.reject(reason)
                    if ready and reason in ('source_changed','geometry_mismatch'):
                        raise RuntimeError('Kamera oturumu değişti. Yeniden bağlanın.' if reason=='source_changed' else
                                           'Kamera ayarları kayıt sırasında değişti. Görüntüler korundu; yeniden bağlanın.')
                    if reason not in ('non_increasing_timestamp','unpaired'):
                        guide.interrupt();good_since=None
                    if received-last_good>1:
                        self.post('waiting' if ready else 'connecting','Kamera odağı ve uygun görüntü çifti bekleniyor.',guide if ready else None)
                    continue
                source=pair.source;last_seen=pair.timestamps;last_good=received
                if not ready:
                    if good_since is None:good_since=received
                    if received-settling_started<4 or received-good_since<1:continue
                    ready=True
                if guide.active and received-last_save>=.2:
                    if writer is None:
                        directory=PROJECT/'data/motion'/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
                        writer=CaptureWriter(directory,report,provenance)
                        writer.events.append(dict(action='start',step=guide.index,hostMonotonic=received))
                    phase=STEPS[guide.index][0]
                    writer.save(pair,phase,time.monotonic());last_save=received
                    if guide.saved(received):
                        writer.events.append(dict(action='step_complete',step=guide.index-1,hostMonotonic=received))
                        writer.finish('in_progress')
                    if guide.finished:
                        final_kind='done';final_message=f'{writer.frames} stereo çift kaydedildi. Kameralar kapatıldı.'
                        break
                if received-last_update>=.12:
                    message='Kayıt sürüyor. İsterseniz duraklatabilirsiniz.' if guide.active else 'Görüntüler hazır. Hazır olduğunuzda bu adımı başlatın.'
                    self.post('ready',message,guide,writer.frames if writer else 0);last_update=received
        except InterruptedError:
            pass
        except Exception as error:
            final_kind='error'
            final_message=str(error) if isinstance(error,RuntimeError) else 'Bağlantı veya kayıt tamamlanamadı. Telefonu kontrol edip yeniden bağlanın.'
            if writer:writer.events.append(dict(action='error',detail=str(error),hostMonotonic=time.monotonic()))
        finally:
            if configured:
                for args in (('shell','am','force-stop',PACKAGE),('forward','--remove','tcp:8765')):
                    prefix=['adb','-s',self.adb_serial] if self.adb_serial else ['adb']
                    try:subprocess.run([*prefix,*args],capture_output=True,timeout=3)
                    except (subprocess.SubprocessError,OSError):pass
            if writer:
                try:writer.finish({'done':'complete','cancelled':'cancelled','error':'failed'}[final_kind],
                                  final_message if final_kind=='error' else None)
                except OSError:
                    final_kind='error';final_message='Görüntüler yazıldı, sonuç dosyası tamamlanamadı. Boş disk alanını kontrol edin.'
            self.post(final_kind,final_message,guide,writer.frames if writer else 0)

    def close(self):
        if self.closing:return
        self.closing=True;self.stop.set()
        self.present(dict(kind='closing',message='Kayıt korunuyor, kamera bağlantısı kapatılıyor.'))
        self.poll_close()

    def poll_close(self):
        if self.worker is not None and self.worker.is_alive():self.root.after(50,self.poll_close)
        else:self.root.destroy()


def main():
    lock=(PROJECT/'work/assistant.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    root=tk.Tk();MotionCaptureApp(root);root.mainloop()


if __name__=='__main__':main()
