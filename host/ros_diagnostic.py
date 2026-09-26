"""Bounded raw stereo evidence for a 30-second live motion diagnostic."""
import json
import math
from host.motion_capture import CaptureWriter
from host.motion_recording import atomic_json


def phase_at(elapsed):
    if elapsed<0:raise ValueError('Negative elapsed time')
    if elapsed<10:return 'sabit'
    if elapsed<20:return 'otele'
    if elapsed<30:return 'sabit_son'
    return 'complete'


class DiagnosticWriter:
    def __init__(self,directory,report,provenance):
        self.writer=CaptureWriter(directory,report,provenance)
        self.start=None;self.last=None;self.phase=None;self.last_tick=None
        self.finish('in_progress')

    def observe(self,pair,now):
        if self.start is None:self.start=now
        phase=phase_at(now-self.start)
        if phase!=self.phase:
            messages={'sabit':'10 saniye telefonu SABİT tut.',
                      'otele':'10 saniye yavaşça SAĞA kaydır; aynı yöne bakmaya devam et.',
                      'sabit_son':'Şimdi 10 saniye SABİT tut.',
                      'complete':'Tanılama tamamlandı.'}
            print('\a'+messages[phase],flush=True);self.phase=phase
        if phase!='complete' and (self.last is None or now-self.last>=.1):
            self.writer.save(pair,phase,now);self.last=now
        tick=int(now-self.start)
        if tick!=self.last_tick:
            self.last_tick=tick
            elapsed=now-self.start
            if phase!='complete':
                step=min(3,int(elapsed//10)+1)
                names={'sabit':'SABİT TUT','otele':'YAVAŞÇA SAĞA KAYDIR','sabit_son':'SABİT TUT'}
                print(f"[{step}/3] {names[phase]} | Aşama: {math.ceil(10-elapsed%10)} sn kaldı"
                      f" | Toplam: {math.ceil(30-elapsed)} sn kaldı | Kaydedilen çift: {self.writer.frames}",flush=True)
        return phase=='complete'

    def finish(self,status,error=None):
        self.writer.finish(status,error)
        path=self.writer.directory/'manifest.json'
        manifest=json.loads(path.read_text())
        manifest.update(guideMode='30s host elapsed: 10s stationary, 10s translation, 10s stationary',
            scope='bounded live ROS diagnostic samples; not continuous full-rate recording or motion ground truth',
            maxSampleHz=10)
        atomic_json(path,manifest)
        if status!='in_progress':
            labels={'complete':'TAMAMLANDI','cancelled':'YARIDA DURDU','failed':'HATA'}
            print(f"\nTEST {labels.get(status,status)} · {self.writer.frames} stereo çift kaydedildi.",flush=True)
