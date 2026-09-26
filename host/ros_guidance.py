"""Turkish movement instructions driven by the publisher, never GUI uptime."""
import math

TEST_SECONDS = 90
ROOM_SECONDS = 180
# Times exclude ROS/camera preparation. Distances are suggestions, not measurements.
STEPS = (
    (0, 5, 'BAŞLANGIÇTA SABİT TUT',
     'İşaretlediğin yerde, iki elinle telefonu aynı yükseklikte tut. '
     'Arka kameralar seçtiğin kitaplık / eşyalı bölgeye baksın. Yalnız bu ilk 5 saniye bekle.'),
    (5, 35, 'YAVAŞÇA YANA İLERLE',
     '30 saniyeye yayarak sağa veya sola toplam 1–2 küçük adım at (yaklaşık 0,5–1 m). '
     'Telefon da yer değiştirsin; yalnız bileğini çevirme. Aynı eşyaları görüntüde tut. '
     'Kablo takılıysa kablonun izin verdiği kadar ilerle.'),
    (35, 55, 'KÜÇÜK BİR İLERİ ADIM',
     'Aynı eşyalara bakarak 20 saniyede yavaşça bir küçük adım ileri git '
     '(yaklaşık 20–40 cm; mesafe ölçümü gerekmiyor). '
     'Telefonu aniden çevirme veya eğme. Önün kapalıysa mesafeyi kısalt.'),
    (55, 85, 'AYNI YOLDAN GERİ DÖN',
     'Önce küçük ileri adımı geri al, sonra yana geldiğin yolu ters yönde izle. '
     '30 saniyede başlangıç işaretine dön. Telefon hâlâ aynı eşyalara baksın; '
     '180° dönme. Başlangıç yüksekliğini ve bakış yönünü yeniden yakala.'),
    (85, 90, 'SON 5 SANİYE SABİT TUT',
     'Başlangıç işaretinde, ilk yükseklik ve bakış yönünde telefonu sabit tut. '
     'Aynı eşya görüntünün aynı bölgesinde olsun. Kayıt otomatik bitecek; '
     'Kaydediliyor yazısından sonra telefonu indirebilirsin.'),
)

# A complete-room observation is deliberately a separate capture. It does not
# imply that every camera pose can be joined into one verified map.
ROOM_STEPS = (
    (0, 5, 'BAŞLANGIÇTA SABİT TUT',
     'Kapıya yakın güvenli bir yer seç. Telefon dik, iki elle göğüs hizasında olsun. '
     'Aydınlık ve ayrıntılı eşyalara bak. Yalnız ilk 5 saniye bekle.'),
    (5, 145, 'ODAYI YAVAŞÇA DOLAŞ',
     'Odanın erişebildiğin bölümlerini tek bir yavaş turla dolaş. Telefon da seninle ilerlesin; '
     'yalnız bileğini döndürme. Her birkaç adımda aynı eşyalar kadrajda kalsın. '
     'Dönerken telefonla birlikte yumuşakça dön; zemine, tavana, parlak ekrana, '
     'perdeye veya boş duvara uzun süre bakma. Mesafe veya açı ölçmene gerek yok.'),
    (145, 175, 'BAŞLANGICA YAKLAŞ',
     'Güvenli yolu izleyerek başladığın bölgeye yavaşça yaklaş. Aynı eşyayı yine kadraja almaya çalış. '
     'Aynı noktaya santimetre hassasiyetinde dönmen gerekmiyor; yolu tamamlayamazsan koşma.'),
    (175, 180, 'SON 5 SANİYE SABİT TUT',
     'Bulunduğun yerde telefonu 5 saniye sabit tut. Sayaç bittiğinde kaydın sonlanmasını bekle.'),
)

ROOM_PREPARATION = (
    'Telefonun kilidini aç; Wi-Fi veya USB ADB bağlantısı hazır olsun. Kamerayı bilgisayardaki düğme açacak.',
    'Odanın ışıklarını aç. Karanlık perde/boş duvar yerine hareketsiz, desenli eşyalara bak. '
    'Önce 30 sn kamera önizlemesinde sahnenin aydınlık ve görüntünün dik olduğunu kontrol et.',
    'Telefonu tüm tur boyunca DİK ve iki elle, yaklaşık göğüs hizasında tut. '
    'Yürürken kamerayı yere/tavana eğme; arka lensleri parmağınla kapatma.',
    'Takılabileceğin kablo ve engelleri kaldır. Yaklaşık başlangıç yerini işaretlemek yeterli; '
    'rotayı veya mesafeyi cetvelle ölçme. Bu testte 5 sn sabit + 140 sn tur + 30 sn yaklaşma + 5 sn sabit var.',
    'Takip kaybolsa bile görüntü kaydı devam eder; yazılım bu kayıt üzerinden ayrı haritaları inceleyebilir. '
    'İlk kayıpta yeniden başlatma veya hızlı hareket etme. Süre bitince kamera kendiliğinden durur.',
)

PREPARATION = (
    'Telefonu yetkili ADB bağlantısıyla (USB veya Wi-Fi) bağla, ekran kilidini aç; hata ayıklama izni sorulursa onayla. '
    'Kamerayı bilgisayardaki başlat düğmesi açacak.',
    'Aydınlık, hareketsiz ve eşyalı bir bölge seç: kitaplık, sandalye, masa gibi. '
    'Yalnız boş duvarı, parlak ekranı, aynayı veya pencereyi hedefleme. Dama gerekmiyor.',
    'Başlangıç yerini bir kâğıt / bant / yer çizgisiyle belirle. '
    'Karşıdaki bir eşyayı başlangıç bakış yönü olarak seç. Dönüşte aynı yer, yükseklik ve yönü kullan.',
    'Kısa ve güvenli bir hareket alanı ayır; kablo takılıysa ayağına dolanmasın. '
    'Telefonu iki elle, göğüs hizasında DİK tut; arka lensleri kapatma. '
    'Test boyunca telefonu yataydan dikeye veya dikeyden yataya çevirme. '
    'Bilgisayardaki görüntü ters/yatay görünürse “Dik göster” / “Yatay göster” düğmesini kullan; telefonu sırf ekrana uydurmak için döndürme.',
    'Bilgisayarı yönergeleri görebileceğin yere koy. İlk 5 ve son 5 saniye sabit, '
    'aradaki 80 saniye yavaş hareket var. Hazırlık süresi 90 saniyeye dahil değil. '
    'Santimetre hassasiyetinde adım, açı veya başlangıca tam dönüş ölçümü gerekmiyor; mesafeler yaklaşık hareket tarifidir. '
    'Telefonun jiroskop/ivmeölçer verisi kayıtla birlikte otomatik gelir.',
)

IDLE_TEXT = ('ŞİMDİ GEREKEN: kısa oda rotası veya 3 dakikalık tüm oda turu.\n\n'
             '1. USB veya Wi-Fi ADB bağlantısını hazırla, telefonun kilidini aç.\n'
             '2. Aydınlık, eşyalı bir bölge ve kısa, güvenli bir yol seç.\n'
             '3. Telefonu dik tut; başlangıç yerini ve bakış yönünü kabaca işaretle.\n'
             '4. Seçtiğin testin hazırlık penceresini oku.\n\n'
             'Dama, cetvel veya robot hassasiyetinde ölçüm gerekmiyor. Jiroskop otomatik kaydedilir. '
             'Bu kısa rota odanın görünen bölümünü kaydeder; tam oda kapsamı garanti değildir.')


def mapping_guidance(status, now, tracking_failed=False, capture_continues=False, duration_seconds=TEST_SECONDS):
    if status and status.get('error'):
        return ('DUR · OTURUM HATASI', 'Test duruyor',
                'Hareketi durdur. Kayıt başarıyla tamamlanmadı. Hata: '+str(status['error']))
    if tracking_failed and capture_continues:
        if status and status.get('stage')=='replay':
            return ('TAKİP KESİLDİ · KAYIT OYNATILIYOR', f"{max(0,int(status.get('remainingSeconds',0)))} sn kaldı",
                    'Telefon kullanılmıyor. Harita birikimi durdu; kayıtlı görüntüler sonuna kadar oynatılıyor. '
                    'Bu oturumun haritası kabul edilmeyecek.')
        if status and status.get('stage')=='finished':
            return ('KAYIT BİTTİ · TAKİP KESİLDİ', 'Kaydediliyor…',
                    'Telefonu indirebilirsin. Görüntü kaydı saklanıyor. '
                    'Bu oturumda takip kaybolduğu için canlı harita kabul edilmedi; '
                    'kaydı incelemek için aynı hareketi yeniden yapman gerekmiyor.')
        if status and now-status.get('updatedMonotonic',0)<=3:
            return ('TAKİP KESİLDİ · KAYIT SÜRÜYOR', f"{max(0,int(status.get('remainingSeconds',0)))} sn kaldı",
                    'Test bitmedi; görüntüler kaydedilmeye devam ediyor. '
                    'Yavaş hareket et ve ayrıntılı eşyaları kadrajda tut. '
                    'Bu oturumun canlı haritası artık büyümüyor; harita bu testte otomatik yeniden başlamaz. '
                    'Süre bitince kayıt otomatik durur. İstersen Durdur ve kaydet ile erken bitir. '
                    'Testi yeniden başlatman gerekmiyor.')
        return ('DUR · GÖRÜNTÜ AKIŞI BEKLENİYOR', 'Kayıt kontrol ediliyor',
                'Takip kesildi, ayrıca güncel görüntü durumu gelmiyor. Hareketi durdur; telefonun ADB bağlantısını kontrol et.')
    if tracking_failed:
        return ('DUR · TAKİP KAYBOLDU', 'Test duruyor',
                'Hareketi durdur. Bu SGBM oturumu takip kaybında otomatik sonlandırılır. '
                'Kurtarmak için yürümeye devam etme. Kamera kapanıp sonuç yazılana kadar bekle; '
                'ekrandaki hata ve mevcut kayıt inceleme için saklanır.')
    if status and status.get('stage') == 'finished':
        return ('Kayıt bitti', 'Kaydediliyor…',
                'Telefonu indirebilirsin. Harita ve kayıt sonucu hazırlanıyor. '
                'Sonuç görünene kadar uygulamayı kapatma; sonra “Bu haritayı 3B aç” düğmesini kullan.')
    if not status or status.get('stage') == 'warming_up':
        duration = status.get('durationSeconds', duration_seconds) if status else duration_seconds
        return ('HAZIRLIK · SABİT TUT', f'{duration} sn başlamadı',
                'Başlangıç işaretinde telefonu sabit tut. ROS ve kameralar hazırlanıyor. '
                'Hazırlık bitince ekranda 1. aşama ve sayaç görünecek. '
                'Yalnız ilk 5 saniye sabit dur, sonraki aşamada yavaşça hareket et.')
    if status.get('stage') == 'replay':
        return ('Kayıttan 3B harita oluşturuluyor',
                'Durum bekleniyor…' if now-status.get('updatedMonotonic',0)>3 else f"{max(0,int(status['remainingSeconds']))} sn kaldı",
                'Telefon kullanılmıyor. Kayıtlı hareket ROS içinde yeniden işleniyor.\nBitince harita otomatik kaydedilecek.')
    if now-status.get('updatedMonotonic', 0)>3:
        return ('DUR · GÖRÜNTÜ AKIŞI KESİLDİ', 'Bekleniyor…',
                'Hareketi durdur. Telefon ekranını ve ADB bağlantısını kontrol et. '
                'Akış dönmeden hareket etme. Sayaç bu sırada sıfırlanmaz; kesinti kayda işlenir.')
    if status.get('stage') != 'mapping':
        return ('DUR · DURUM BEKLENİYOR', 'Bekleniyor…', 'Hareket etmeden yayıncı durumunu bekle.')
    duration = status.get('durationSeconds', TEST_SECONDS)
    elapsed = max(0., status.get('elapsedSeconds', duration-status.get('remainingSeconds', duration)))
    remaining = max(0, math.ceil(duration-elapsed))
    if duration != TEST_SECONDS:
        if duration == ROOM_SECONDS:
            if elapsed >= ROOM_SECONDS:
                return ('TURU BİTİR', '0 sn kaldı', 'Telefonu sabit tut; kayıt sonlandırılıyor.')
            for index, (start, end, title, detail) in enumerate(ROOM_STEPS):
                if elapsed < end:
                    return (f'{index+1}/4 · {title}', f'{math.ceil(end-elapsed)} sn bu adım',
                            f'Toplam {remaining} sn kaldı · Kayıt: {status.get("publishedPairs", 0)} stereo çift\n\n'+detail)
        return ('YAVAŞÇA HAREKET ET', f'{remaining} sn kaldı',
                'Kısa yolda yavaş ilerle ve başlangıç yönüne dön. Bu süre 90 saniyelik aşamalı rehbere ait değil.')
    if elapsed >= TEST_SECONDS:
        return ('HAREKETİ BİTİR', '0 sn kaldı', 'Kayıt sonlandırılıyor; başlangıç yerinde bekle.')
    for index, (start, end, title, detail) in enumerate(STEPS):
        if elapsed < end:
            return (f'{index+1}/5 · {title}', f'{math.ceil(end-elapsed)} sn bu adım',
                    f'Toplam {remaining} sn kaldı · Kayıt: {status.get("publishedPairs", 0)} stereo çift\n\n'+detail)
