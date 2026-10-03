# AI haritası, telefon yolu ve anlık poz için isteğe bağlı ROS dinleyicisi

Yerel AI akışında üç iş ayrıldı: paket kaydı/poz girişi, tek model işçisi,
ROS yayını. Model çalışırken pozlar alınır ve yol yayınlanır. Bekleyen tek
derinlik işi daha yeni kare gelince değiştirilir; çalışan işin pozu değiştirilmez.
ROS zamanlayıcısı telefon paketi beklerken de durum ve hazır haritayı yayınlar.

Bu yol önce **kayıtlı akışın gerçek zaman hızında yeniden oynatılmasıyla**
sınandı. Kullanıcıdan yeni test istenmedi, ADB/telefon bağlantısı veya kamera
başlatılmadı. Gerçek telefondan AI canlı aktarımı henüz ölçülmedi.

## Önce/sonra ölçümü

Aynı 273 derinlik kareli kaynak, aynı 33 açık karşılaştırma dışlaması ve 240
uygun kare. Aynı yerel RTX4080 Laptop, Depth Pro float16 ve kayıtlı odak/pozlar.
FIFO kontrolü saf kayıt CLI'sinde, son-kare denemesi ROS mesajları yayınlayan
CLI'de çalıştı. İki koşul da kaydın orijinal zamanlamasıyla beslendi; bunlar
birer çalıştırmadır. FIFO fiziksel canlı kullanım için seçilemez.

| Ölçüt | Önce: bütün uygun kareleri sıraya alan kontrol | Sonra: en yeni bekleyen kare |
|---|---:|---:|
| En fazla bekleyen kare | 135 | 1 |
| Kuyruk bekleme p95 | 79.454,24 ms | 262,33 ms |
| Hosta girişten haritaya eklemeye p95 | 80.129,99 ms | 771,77 ms |
| Aynı gecikmenin medyanı | 37.102,63 ms | 634,75 ms |
| Model çıkarımı medyanı | 444,88 ms | 443,87 ms |
| İşlenen AI kareleri | 240 | 105 |
| Daha yeni kare için atlanan işler | 0 | 135 |
| Korunan kamera pozları | 564 | 564 |
| Üç ayrı kareyle desteklenen noktalar | 285.972 | 80.818 |
| Replay başladıktan son harita kaydına süre | 144,34 s | 60,39 s |

Gecikme model, ölçekleme, füzyon ve gerektiğinde önbellek/snapshot hazırlığını
içerir; telefonun görüntüyü alma/aktarma gecikmesini ve RViz çizim süresini
ölçmez. Son-kare sonucunun en yüksek host giriş→harita süresi 940,48 ms idi.
Modelin kendisi hızlandırılmadı. Daha az bağımsız bakış işlendiği için canlı
önizleme daha seyrek; şekil doğruluğu veya sehpa keskinliği artışı iddia edilmiyor.

Bağımsız ROS abonesi 86 bulut mesajı, 418 yol ve 418 anlık poz mesajı aldı.
378 yol mesajı model çalışırken geldi. Son bulut **80.818 nokta**, son yol
**564 poz**; anlık poz ile yolun son noktasının koordinatları aynı.
Yolun tamamı önceki kayıtlı adayla sayısal olarak aynı, en büyük fark **0**.
Bu, başka bir oturumda eski haritada yer bulma değildir.

Tam RGB/ham derinlik akışı yeni ROS çıktı dizinine **bayt bayt aynı** kopyalandı.
Kuyruktan çıkarılan görüntüler kayıttan silinmedi. 105 tahmin önbelleği mevcut
scene/cache yükleyicisinde kabul edildi; kamerayı/modeli tekrar çalıştırmadan
harita yeniden kuruldu: 80.818 → 80.818 nokta, en büyük XYZ farkı **0 m**.
Akış, manifestler, yol, 105 cache, karşılaştırılan bulut ve orijinal akış dahil
**111 dosyanın** SHA değeri karşılaştırma öncesi/sonrası aynı.

## Komutlar

Mevcut yerel model kurulumuyla, kamera kapalı ROS replay:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=work/depth-pro-runtime-20261001/pkgs:.:$PYTHONPATH \
HF_HUB_OFFLINE=1 OMP_NUM_THREADS=8 \
  .venv/bin/python -m host.ros_arcore_ai \
  --replay work/arcore-detail-confirmed-wifi-20261001-01/stream.jsonl \
  --repository work/Depth-Pro-20261001 \
  --checkpoint data/models/depth-pro-20261001/depth_pro.pt \
  --output work/ai-replay-new --save-depths --rviz --cell-limit 4000000
```

Bu genel örnek bütün kaynak karelerini değerlendirir. Yukarıdaki sayıları
üreten gerçek komut ayrıca `--exit-after-replay`, `--queue-policy latest` ve
`--exclude-frames 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 145 150 155 160 165 170 175 180 185`
kullandı. Kontrol `host.arcore_ai_stream --replay ... --queue-policy fifo`
ile aynı dışlamalar/ayarlarla ve aynı 1x hızda çalıştı. ROS gerektirmeyen kayıt
CLI'sinde `--replay` zorunlu; bu CLI telefona bağlanamaz.

Kullanıcı hazır olduğunda canlı dinleyici için yukarıdaki komutta `--replay`
yerine `--serial <yetkili-adb-seri>` kullanılır. Bu **yalnız dinleyicidir**;
kamerayı telefondaki düğmeyle kullanıcı başlatır. Süre telefonda seçilir.
FIFO, kare dışlama ve `--exit-after-replay` canlı girişte reddedilir.
Mevcut ham ARCore dinleyicisi değiştirilmedi; yeni AI yolu isteğe bağlıdır.
Dinleyici kullanıcı başlamadan önce süre dolduğu için kapanmaz; Ctrl+C ile
iptal edilebilir. İlk veri geldikten sonra 20 saniyelik veri kesintisi hata
sayılmaya devam eder. Bu değişiklik derinlik/geometri kabul eşiklerini değiştirmez.
Eksik kayıt için konsol tamamlanmış harita mesajı göstermez.

ROS topic'leri: `/phone/arcore_ai_map`, `/phone/arcore_ai_path`,
`/phone/arcore_ai_pose`, `/phone/arcore_ai_status`; koordinat çerçevesi
`arcore_map`. AI topic'leri eski ham harita yayıncısıyla aynı topic'i kullanmaz.
RViz config'i harita/yol ve telefonun kayıtlı pozunu eksenlerle gösterir;
Transient Local QoS ile son mesajlar sonradan bağlanan aboneye de açıktır.
Bu config sentetik kontrolden geçti; yeni RViz ekranının görsel kontrolü yapılmadı.

## Kaydetme ve yoğun son harita

`finished` / `saved_by_user` bitişinde kalan son iş tamamlanır. İptal, ağ hatası,
model hatası, eksik son paket veya kaynak değişmesi tamamlanmış AI haritası
olarak işaretlenmez. Model hatası poz girişini durdurmaz; durum topic'inde hata
görünür. JSON/ham paketler özel dosyada korunur. Soket ve replay girişleri
durdurma olayıyla kapatılır; işçiler bitmeden nihai harita kaydedilmez.

Çıktılar: `map_cloud.ply`, `map_poses.txt`, tam `result.json`, `frame_metrics.json`,
`input_metrics.json`; ROS modunda ayrıca tam `stream.jsonl`, `rejections.jsonl`
ve yayın sayıları. `--save-depths` seçilirse işlenen karelerin NPZ'leri saklanır.
Tüm çıktı dizinleri yeni olmalı ve depo içinde ignore altındaki work/data/outputs
alanında olmalı. Canlı kaydın SHA'sı kapanmış kaynak dosyası üzerinde nihai harita
yazımı öncesi/sonrası alınır; bu işlem canlı akış öncesi var olmayan dosyaya hash
ölçtüğü anlamına gelmez.

Yoğun son harita için kaydedilen **tam** akış, mevcut `host.arcore_ai_map`
ile yeni dizine çevrimdışı işlenebilir. Bu ek iş otomatik başlatılmıyor;
son-kare önizlemesi bütün uygun kareleri işlemeye eşdeğer değildir.
Üç-kare desteği, 1 cm hücre, confidence192/500 ölçek pikseli, 0,5–5 m aralık ve
kenar reddi korunur. Aynı kamera karesinin farklı derinlik zaman damgaları
ek bağımsız kamera görüşü sayılmaz.

## Kanıt ve sınırlar

Tam gerçek konsol/manifestler yerel `work/arcore-ai-stream-fifo-20261001-01/`,
`work/arcore-ai-stream-latest-ros-20261001-02/` ve
`work/arcore-ai-stream-evidence-20261001-02/` altında. Sonuncuda gerçek bağımsız
ROS abone kodu/JSON'u, karşılaştırma kodu/tam JSON'u ve test günlüğü var.
30 ilgili test ROS ortamında geçti. Native salt okunur incelemede kaynak SHA ve
paket beklerken ROS yayını sorunları bulundu, root giderdi; son incelemede maddi
bulgu yok. Kamera-kare desteği için ek test ve kayıtlı cache eşitliği ayrıca geçti.

Yeni yoğun/renkli GLB veya fiziksel geometri kazanımı bu adımda ölçülmedi.
1 Ekim'de v0.12 kuruldu ve kullanıcı beş dakikalık taramayı tamamladı:
1.287 RGB/derinlik karesi, 2.776 kayıtlı poz, telefon raporunda hata yok.
Dinleyici kullanıcı başlamadan eski 300 saniyelik bekleme süresinde sona
erdiği için canlı AI aktarımı 0 paket aldı. Telefonun tam yerel kaydı alındı;
cihaz/PC SHA256 eşit. Bu, canlı AI başarısı değildir. Başlangıç beklemesi
artık kullanıcı başlayana veya Ctrl+C gelene kadar sürer. Sentetik saatle
602. saniyede bağlantı, başlangıç iptali ve aktif akışın 20 saniyelik
kesinti sınırı test edildi; 15 ilgili test geçti. Telefon ısısı ölçülmedi;
yeni dinleyici davranışı ve yeniden yer tanıma fiziksel olarak bekliyor.

Ücretli servis veya eğitim kullanılmaz. Metric Small Apache-2.0; isteğe bağlı
Depth Pro kod/ağırlıkları Apple'ın ayrı lisansına tabidir. Bütün proje için
açık kaynak lisansı henüz seçilmedi. Ev görüntüleri, sayısal haritalar ve model
ağırlıkları public kaynak paketine/GitHub'a eklenmedi.
