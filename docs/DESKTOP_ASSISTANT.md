# Stereo Rehber

Çalıştırma: `scripts/start_assistant.sh`. Telefon USB ile bağlı, ADB yetkili ve kamera izni verilmiş olmalı. Ubuntu üzerinde Tkinter, Pillow ve projedeki OpenCV ortamını kullanır. `--demo` yalnız arayüzü gösterir.

Sol taraftaki 10×7 kare dama sabittir (9×6 iç köşe). Sağdaki büyük görüntü üst telefoto kameranın, küçük görüntü alt geniş açılı kameranın görüşüdür. Önizlemeler görüntüleme için 90° döndürülür; ham JPEG ve köşe koordinatları değiştirilmez. Başlat, Duraklat/Devam et, Boşluk ve Kapat kontrolleri vardır. Esc pencere moduna, F11 tam ekrana geçer. Bir poz kaydından sonra dama piksel boyutu değişirse oturum geçersizleşir; yeni oturum açılmalıdır.

Önce telefonun hareketinin sağdaki canlı görüntüde izlendiğini kontrol edin. Ardından Başlat'a basın. Damanın tamamı, beyaz dış payla birlikte iki kamerada görünmelidir. Yeşil kutu hedef merkezini gösterir. Telefonun sağ/sol veya üst/alt kenarını küçük miktarda ekrana yaklaştırarak gerçek açı değişimi yapılır. Ekrandaki damayı sanal olarak eğmek, fiziksel ekran düzlemini eğmez ve kalibrasyonun gerektirdiği gerçek pozları oluşturmaz.

Uygulama 30 eğitim ve 10 doğrulama pozu için sıra gösterir. Kamera çiftini doğrulama başlangıcında yeniden açar. Zaman sınırı yoktur; telefon kaydı güvenlik üst sınırı bir saattir. Yalnız seçilen JPEG çiftleri bilgisayara kalıcı kaydedilir. Kamera açıkken telefon tüm karelerin hafif metadata loglarını tutar, sürekli JPEG dosyası yazmaz.

## Otomatik kayıt koşulları

- İki sensörde gerçek SENSOR_TIMESTAMP ile tam eşleşen metadata.
- Poz kaydı için sensör zaman farkı en fazla 20 ms. Önizleme bu eşleşmeyi beklemez: zamanları yeterince yakın olmayan kareler gösterilir ama kaydedilmez.
- Hedef iki görüntüde bulunmalı; dış sınır 18 piksel payla içeride kalmalı.
- Hedef merkezi istenen bölgeye ulaşmalı; yinelenen pozlar reddedilir.
- Yan/üst eğim ve dönme yönergeleri farklı görüntü özellikleriyle kontrol edilir; bunlar ölçülmüş derece değildir. Ölçülen özellikler selected-poses.jsonl'e yazılır.
- Son kaynak kare aralığı iki kamerada da 40–100 ms olmalı. Her karede maksimum kaynak boşluğu da aktarılır.
- Elde tutma için kısa seri seçimi kullanılır: yaklaşık1,5 saniye aday toplanır, en fazla2,5 saniyelik/20 örneklik tampon tutulur. Küçük titremede sayaç sıfırlanmaz. Adayın önceki ve sonraki örneğine göre hareket hızı, sensörler arası zaman farkı, pozlama ve rolling shutter süresiyle birlikte değerlendirilir; tahmini hareket riski≤3 piksel ve hedef bölgesinde Laplacian varyansı≥30 olan adaylar hareket/netlik puanına göre sıralanır. Bu sezgisel bir seçimdir, gerçek epipolar doğruluk kanıtı değildir. Büyük hareket, bulanıklık veya eskimiş örnekler kayda alınmaz.
- Köşe bulma arka plandadır; önizleme kuyruğu yalnız en yeni görüntüyü tutar. İşlenmiş eski kareler kayda kabul edilmez.

`data/guided/TARIH_SAAT/` altındaki session.json, training/ ve validation/ dizinleri yeniden analiz içindir. Android kaynak run kimliği de poz kayıtlarında tutulur. Tüm JPEG'ler yeniden kodlanmadan saklanır. Doğrulama görüntüleri eğitim optimizasyonuna verilmez.

Çekimin 40/40 olması kabul edilmiş kalibrasyon anlamına gelmez. `host.calibrate` ayrıca 25 eğitim, 5 farklı doğrulama pozu, her iki grupta dört kadran ve ayrı görüntülerde p95 epipolar hata ≤1,5 piksel koşullarını uygular. Ekranın nominal 17,3 inç ölçüsü ve EDID bilgisi fiziksel cetvel ölçümü yerine kullanılmaz. Çıktı ancak checker_square veya piksel birimindedir; gerçek metre ölçeği doğrulanmadı.

USB servisi Android loopback 127.0.0.1:8765 üzerinde, yalnız ADB forward ile erişilir; ağda dinlemez. Kapat yalnız bu kamera uygulamasını durdurur ve kendi yönlendirmesini kaldırır.

Kapanıp açıldığında tamamlanmamış en son oturumdaki kayıtlı pozlar otomatik geri yüklenir. Başlat düğmesi “Kaldığım yerden devam et” olur. İstenirse komut satırından `--new` yeni oturum açar; eski kayıtlar silinmez. `--resume DIZIN` belirli bir oturumu açar.
