"""Pure state for a user-paced capture guide. No camera, clock or GUI side effects."""
STEPS = (
    ('sabit', 'Sabit tutun', 'Telefonu bir yere dayayın veya rahatça sabit tutun. Küçük el titremeleri normal.', 'steady'),
    ('otele', 'Yavaşça sağa kaydırın', 'Aynı nesneyi görmeye devam ederek telefonu yavaşça sağa taşıyın. Telefonun baktığı yönü koruyun.', 'slide'),
    ('dondur', 'Hafifçe sağa çevirin', 'Yerinizde kalın. Telefonun baktığı yönü yavaşça biraz sağa çevirin; aynı nesne görüntüde kalsın.', 'turn'),
    ('geri_don', 'Başlangıca dönün', 'Telefonu yavaşça ilk konumuna ve ilk baktığı yöne getirin. Tam isabet etmesi gerekmiyor.', 'return'),
)


class CaptureGuide:
    """Only consecutive admitted packets advance time; pauses break continuity."""
    def __init__(self, duration=8.):
        if duration <= 0:raise ValueError('Invalid stage duration')
        self.duration=duration;self.index=0;self.elapsed=0.;self.counts=[0]*len(STEPS)
        self.active=False;self.finished=False;self.last=None;self.revision=0

    def command(self, action, revision):
        # A delayed double click must not begin the next stage.
        if revision!=self.revision or self.finished:return False
        if action=='start' and not self.active:
            self.active=True;self.last=None;self.revision+=1;return True
        if action=='pause' and self.active:
            self.active=False;self.last=None;self.revision+=1;return True
        return False

    def interrupt(self):
        self.last=None

    def saved(self, now):
        if not self.active:return False
        self.counts[self.index]+=1
        if self.last is not None and 0 < now-self.last <= .5:
            self.elapsed=min(self.duration,self.elapsed+now-self.last)
        self.last=now
        if self.elapsed>=self.duration:
            self.active=False;self.last=None;self.index+=1;self.elapsed=0.;self.revision+=1
            self.finished=self.index==len(STEPS)
            return True
        return False

    def snapshot(self):
        return dict(index=self.index,elapsed=self.elapsed,duration=self.duration,
                    counts=list(self.counts),active=self.active,finished=self.finished,revision=self.revision)
