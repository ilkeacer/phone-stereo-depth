# Takip kaybından sonra güvenli ham kayıt bölümleri

Canlı `host.ros_session` hâlâ ilk `OdomInfo.lost=true` olayında haritalamayı durdurur, başarısız haritayı dışa aktarmaz ve görüntü kaydını sürdürür. Bu koruma değiştirilmedi. Yeni kayıtlar artık `capture.json` içine yayınlanan ROS görüntü zamanını ham kamera zamanından üreten **tek seferlik** `rosStampOffsetNs` değerini yazar. Oturum kapanırken gözlenen kayıp varsa `recovery-plan.json` kendiliğinden hesaplanır. Plan, kayıp zamanından en az 200 ms önceki ve en az 200 ms sonraki tamamlanmış stereo çiftleri ayrı aralıklar olarak tanımlar; ortadaki kareler karantinada kalır. 200 ms bir doğruluk ayarı değil, kayıp sınırını dışarıda tutmak için koruyucu zaman boşluğudur.

Plan yalnız aralıkları hesaplar; yeni haritalar canlıda başlamaz. Kaynak kaydın başarısız/tamamlanmış/iptal durumunu değiştirmez. Kayıt sonlandıktan sonra kaynak oturum diziniyle:

```bash
PYTHONPATH=. .venv/bin/python -m host.ros_recovery_segments work/ros-live-OTURUM
PYTHONPATH=. .venv/bin/python -m host.ros_recovery_segments work/ros-live-OTURUM --output work/ayri-parcalar-OTURUM
```

İlk komut yalnız planı gösterir. İkincisi kaynak JPEG'leri **yeni** iki dizine kopyalar, kopya SHA-256'larını doğrular ve her parçanın kendi `committed-pairs.json` ile `manifest.json` dosyasını üretir. Çıkış zaten varsa reddeder. Parçaların manifestleri kaynak oturum durumunu, kaynak indeksleri, dışlanan sınırı ve `sourceTrackingLossNotRecovered=true` / `separateMapOriginRequired=true` açıklamalarını taşır. `host.ros_stereo` bu açıklamalar olmadan türetilmiş bag'i kabul etmez. Her parça ayrı ROS bag/replay ile yeni koordinat başlangıcında işlenebilir; bu bir yeniden yer bulma veya tek haritaya bağlama işlemi değildir.

İki bölümün ayrı ROS haritalarını tek komutta oluşturma:

```bash
PYTHONPATH=. .venv/bin/python -m host.ros_recovery_map \
  --segment work/ayri-parcalar-OTURUM/before-loss \
  --segment work/ayri-parcalar-OTURUM/after-loss \
  --output work/ayri-haritalar-OTURUM --rate 0.5
```

Komut her bölümü kendi ROS bag'ine çevirir, ayrı RTAB-Map oynatması ve harita dışa aktarımı yapar. `fragment-index.json` her parçanın durumunu ve kaynak aralığını listeler; `mapsMerged=false` ve `mapContinuityVerified=false` korunur. Bir bölümün takibi/bağlantısı/export'u başarısızsa onu başarılı harita ilan etmez, logları bırakır. Bu komut **telefonu veya kamerayı açmaz**. Yeni canlı kayıtta `recovery-plan.json` otomatik oluşsa da bu iki komutluk çevrimdışı harita oluşturma henüz kullanıcı isteğiyle çalışır.

Önceden elle ayrılmış gerçek kayıp öncesi/sonrası bölümlerle iki bölümlü komut uçtan uca çalıştı: 518 çift → 8 poz/21.967 nokta ve 78 çift → 6 poz/19.434 nokta. İki DB `quick_check=ok`, takip kaybı 0 ve tüm alt süreçler temiz çıktı. Bunlar aynı haritanın iki kısmı değil, ayrı başlangıçlı haritalardır. Kanıt ve dosyalar `outputs/Otomatik-Ayri-Haritalar-20260924/SONUC.md`.

Eski `ros-live-bgiwilct` kaydında `rosStampOffsetNs` saklanmadığı için otomatik plan **reddedildi**. Bu kayıt önceki özel denetimde, kaynak indeksleri ve sınırlaması açıkça belgelenerek iki ayrı harita parçasına elle ayrılmıştı. Yeni aracın eski kaydı gözlemci kare sayısından tahmin ederek sessizce bölmesine izin verilmiyor. Yeni zaman bilgisiyle gerçek telefon kaybı yaşanmadığı için canlı uçtan uca sınır doğrulaması henüz yapılmadı; sentetik zaman/kayıp testleri ve mevcut gerçek kaydın reddi doğrulandı. Daha önce elle ayrılmış gerçek 78 çiftlik kayıp sonrası bölüm yeni tek-komut işleyicide ayrıca denendi: 78 bag çifti, 0 gözlenen takip kaybı, 6 poz ve 19.474 noktalı ayrı harita; `mapsMerged=false`. Önceki bağımsız oynatmada aynı bölüm 19.363 noktaydı; RTAB-Map örnekleme/çalışma zamanı sonucu birebir sabit değil. İki değer ayrı ölçümlerdir, biri diğerini düzeltmez.

Kayıp çevresindeki 16 kayıp öncesi × 16 kayıp sonrası karede 256 SIFT karşılaştırması daha yapıldı. Dört çiftte üçer iki yönlü derinlikli eşleşme vardı; hiçbir çiftte üçten fazla yoktu. Üç eşleşme, bir dönüşümü bağımsız eşleşmelerle sınamaya yetmez. `work/recovery-segment-20260924/bridge-window.json` tüm 256 satırı, `bridge-window.log` özetini ve `bridge_window_probe.py` tekrar üretme kodunu içerir. Parçalar hâlâ ayrı tutuluyor.

Bir sonraki iş: yeni kayıt üzerinde zaman sınırını gerçek kayıp olayıyla doğrulamak ve ardından otomatik **ayrı harita** işleme akışını eklemek. Eski/yeniyi tek koordinatta birleştirmek için çok kareli tekrar gözlem ve yeterli bağımsız 3B eşleşme gerekir. Daha geniş bir oda haritası için odanın daha büyük alanını gören yeni hareket kaydı ayrıca gerekli olacak.
