import unittest
from host.ros_guidance import mapping_guidance, STEPS, ROOM_SECONDS, ROOM_STEPS


class GuidanceTests(unittest.TestCase):
    def active(self,elapsed,**extra):
        return dict(stage='mapping',durationSeconds=90,elapsedSeconds=elapsed,
                    remainingSeconds=90-elapsed,updatedMonotonic=100,publishedPairs=12,**extra)

    def test_warmup_then_only_short_start_and_end_holds(self):
        warmup=mapping_guidance(dict(stage='warming_up'),100)
        self.assertIn('sabit tut',warmup[2])
        self.assertIn('90 sn başlamadı',warmup[1])
        for elapsed in range(90):
            title,_,detail=mapping_guidance(self.active(elapsed),101)
            self.assertEqual('SABİT TUT' in title,elapsed<5 or elapsed>=85)
            if 5<=elapsed<85:self.assertNotIn('sabit tut',detail)

    def test_each_boundary_has_correct_phase_and_countdown(self):
        for index,(start,end,title,_) in enumerate(STEPS):
            with self.subTest(start=start):
                result=mapping_guidance(self.active(start),100)
                self.assertEqual(result[0],f'{index+1}/5 · {title}')
                self.assertEqual(result[1],f'{end-start} sn bu adım')
                self.assertIn(f'Toplam {90-start} sn kaldı',result[2])
                last=mapping_guidance(self.active(end-.1),100)
                self.assertEqual(last[0],result[0]);self.assertEqual(last[1],'1 sn bu adım')
        self.assertEqual(mapping_guidance(self.active(90),100)[0],'HAREKETİ BİTİR')

    def test_stale_stream_does_not_keep_telling_user_to_move(self):
        stale=mapping_guidance(dict(stage='mapping',remainingSeconds=89,updatedMonotonic=100),104)
        self.assertNotIn('HAREKET ET',stale[0])
        self.assertIn('Hareketi durdur',stale[2])

    def test_finished_state_ends_movement(self):
        result=mapping_guidance(dict(stage='finished'),200)
        self.assertEqual(result[0],'Kayıt bitti')

    def test_error_or_lost_guard_overrides_active_and_finished(self):
        for status in [self.active(30),dict(stage='finished')]:
            self.assertIn('TAKİP KAYBOLDU',mapping_guidance(status,100,True)[0])
            status['error']='geometry_mismatch'
            result=mapping_guidance(status,100)
            self.assertIn('OTURUM HATASI',result[0]);self.assertIn('geometry_mismatch',result[2])

    def test_loss_can_end_mapping_without_ending_capture(self):
        title,counter,detail=mapping_guidance(self.active(30),100,True,True)
        self.assertIn('KAYIT SÜRÜYOR',title);self.assertEqual(counter,'60 sn kaldı')
        self.assertIn('canlı haritası artık büyümüyor',detail)
        self.assertNotIn('otomatik sonlandırılır',detail)
        title,_,_=mapping_guidance(dict(stage='finished'),100,True,True)
        self.assertIn('KAYIT BİTTİ',title)

    def test_continued_capture_still_stops_motion_on_stale_source_or_error(self):
        for status,now in [(self.active(30),104),(self.active(30,error='disconnected'),100)]:
            title,_,detail=mapping_guidance(status,now,True,True)
            self.assertTrue(title.startswith('DUR'));self.assertIn('Hareketi durdur',detail)

    def test_replay_never_directs_physical_movement(self):
        for now in (101,110):
            result=mapping_guidance(dict(stage='replay',remainingSeconds=20,updatedMonotonic=100),now)
            self.assertIn('Telefon kullanılmıyor',result[2])
            self.assertNotIn('Hareketi durdur',result[2])
        result=mapping_guidance(dict(stage='replay',remainingSeconds=20,updatedMonotonic=100),101,True,True)
        self.assertIn('Telefon kullanılmıyor',result[2])

    def test_nonstandard_duration_has_no_false_ninety_second_phases(self):
        status=self.active(10);status['durationSeconds']=120
        result=mapping_guidance(status,101)
        self.assertEqual(result[1],'110 sn kaldı');self.assertNotIn('/5',result[0])

    def test_full_room_guide_uses_all_180_seconds_and_ends_safely(self):
        self.assertEqual(mapping_guidance(None,100,duration_seconds=ROOM_SECONDS)[1],'180 sn başlamadı')
        for index,(start,end,title,_) in enumerate(ROOM_STEPS):
            status=dict(stage='mapping',durationSeconds=ROOM_SECONDS,elapsedSeconds=start,
                        remainingSeconds=ROOM_SECONDS-start,updatedMonotonic=100,publishedPairs=12)
            result=mapping_guidance(status,100)
            self.assertEqual(result[0],f'{index+1}/4 · {title}')
            self.assertEqual(result[1],f'{end-start} sn bu adım')
            self.assertIn('12 stereo çift',result[2])
        status['elapsedSeconds']=ROOM_SECONDS;status['remainingSeconds']=0
        self.assertEqual(mapping_guidance(status,100)[0],'TURU BİTİR')
