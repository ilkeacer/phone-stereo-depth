# Canlı 3B harita için sıradaki kararlar

Hedef: Kullanıcının telefonda başlattığı ARCore oturumundan, bilgisayarda canlı
3B nokta bulutu ve kamera yolu oluşturmak; oturum sonunda bunları kaydedip ROS 2'de
yeniden açmak. Bu zaten yapılabiliyor. Şimdi asıl ölçüt sehpa ve zemin gibi düz
yüzeylerin şekli; nesne etiketleri ve robot kontrolü bundan sonra gelecek.

## 1. Kayıtlı sehpa verisindeki karar (kamera gerekmiyor)

`work/table-arcore-detail-20260929-01/stream.jsonl` içindeki 232 ham derinlik
karesi ve 526 takip pozuyla iki harita ayrı çıktılarda karşılaştırıldı. Eski
yöntem, 2 cm hücrede en son ölçülen noktayı tutuyor ve en az iki ayrı kare istiyor.
Yeni aday, 1 cm hücrede kare başına ölçülen noktaların ortalamasını alıyor,
ardından en az üç ayrı karenin ortalamasını tutuyor. Doldurma veya yapay yüzey
yumuşatma yapılmıyor; RGB ev görüntüleri harita dosyasına yazılmıyor.

Sehpanın göründüğü altı kayıtlı bakışta aynı RGB maskesi ve her bakışın tek
karelik ham derinlik düzlemi kullanıldı. Birleştirilmiş harita noktaları ilgili
kareye yeniden izdüşürüldü; yalnız maske içindeki, o karenin derinliğine 5 cm
yakın noktalar ölçüldü. Ölçüt **fiziksel yer doğruluğu değil**, seçili sahnede
düz yüzeyden sapma ve görünür yüzey piksel kapsamıdır.

| Ölçüt, altı bakışın medyanı | Önce: 2 cm / son nokta / 2 kare | Aday: 1 cm / ortalama / 3 kare |
|---|---:|---:|
| Sehpa düzleminden p95 mutlak sapma | 36,30 mm | 26,97 mm |
| Sehpa maskesinde nokta görülen piksel payı | %77,97 | %99,90 |
| Kaydedilen bütün harita noktaları | 157.368 | 120.755 |
| Aynı 232 karelik çevrimdışı işleme süresi | 9,81 s | 10,35 s |
| İşlem tepe RAM'i | 436,9 MiB | 654,5 MiB |

Altı bakıştan birinde p95 sapma **35,99 → 39,15 mm** oldu; yeni yöntemin her
açıda daha iyi olduğu söylenemez. Tek karelik sehpa düzlemlerinin p95 sapması
12,81–18,68 mm aralığında; birleştirmedeki kalıntıdan yalnız poz kayması
sorumludur sonucu çıkmaz. Sayılar ARCore'un tahmini metrik koordinatlarında;
cetvelle doğrulanmış mutlak hata değildir. Analiz ve sahneye özel dosyalar
Git'in yok saydığı `work/table-map-quality-20260930-01/` altında kalır.

Yeni aday `host.arcore_confirmed_map` ile kayıtlı bir oturumdan yeni klasöre
çıkarılabilir. `host.ros_arcore_detail --confirmed-map` aynı yöntemi canlı ROS
bulutunda **isteğe bağlı** uygular; bayrak verilmezse eski yöntem çalışır.
Kaydedilen aday `host.ros_arcore_saved` ile telefona bağlanmadan ROS 2'de
açılabilir. 1 Ekim'de canlı bayrak kablosuz ARCore kaydında sınandı;
aşağıda sonuçları var.
Kayıtlı akışın çevrimdışı ve canlı yayın için kullanılan artımlı işlem yolları
120.755 aynı hücreyi verdi; karşılık gelen XYZ'lerin en büyük farkı
0,000169 mm (yuvarlama düzeyi). Kaydedilen bulut ve yol ROS mesajlarına
120.755 nokta / 526 poz olarak çevrildi.

Kayıtlı veriden tekrar üretme (her seferinde yeni `--output` klasörü seçilir):

```bash
PYTHONPATH=. python3 -m host.arcore_confirmed_map \
  --input work/table-arcore-detail-20260929-01/stream.jsonl \
  --output work/table-confirmed-map-yeni
```

Kayıtlı çıktıyı telefona bağlanmadan ROS/RViz'de açma:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=. python3 -m host.ros_arcore_saved \
  --session work/table-confirmed-map-yeni --rviz
```

## 2. Kablosuz canlı deneme (2026-10-01)

Bilgisayar ADB'de yalnız Wi-Fi telefonu gördü; ROS dinleyicisi
`--confirmed-map --rviz` ile açıldı. Kamerayı kullanıcı telefondaki
**“ARCore 60 sn ayrıntılı harita”** düğmesine basarak başlattı.
`work/arcore-detail-confirmed-wifi-20261001-01/stream.jsonl` 273 ham derinlik
karesi, 564 takip pozu, 2 duraklatılmış paket, 0 reddedilen paket,
0 sıra boşluğu ve `finished` bitişi içerir. ARCore'un tahmini poz yolu
58,74 saniyede toplam 7,752 m; başlangıç-son poz farkı 0,086 m.
Bu son değer gerçek konuma dönüş hatası olarak doğrulanmadı.

Canlı 1 cm haritada **263.033 nokta** yayınlandı, fakat eski 1 milyon hücre
sınırında **30.056 hücre kabul edilmedi**. Aynı ham kaydın hücre sınırı
olmayan çevrimdışı işlemi **264.343 nokta** verdi; canlı çıktıdan **1.310 nokta** fazla.
Sınır 2 milyona yükseltildi ve aynı 273 kare artımlı olarak tekrar oynatıldı:
**264.343 nokta, 0 kapasite reddi**. Bu yeni sınırın canlı telefondaki
performansı henüz ölçülmedi. Tam çevrimdışı sonuç, kamera kapalıyken
ROS/RViz'de **264.343 nokta / 564 poz** olarak açıldı.

Bu kayıttaki gerçek beyaz sehpanın 9 açısında, eski 2 cm/son nokta/2 kare
ile yeni 1 cm/ortalama/3 kare karşılaştırması:

| Ölçüt, aynı 9 bakışın medyanı | Önce | Sonra |
|---|---:|---:|
| Sehpa düzleminden p95 mutlak sapma | 37,74 mm | 36,96 mm |
| Maskede nokta görülen piksel payı | %88,31 | %97,10 |
| Toplam harita noktası | 192.285 | 264.343 |

Dokuz bakıştan üçünde p95 sapma arttı. Görünür alan artışı, bütün nesne
şeklinin doğru olduğu anlamına gelmez. Geometrik zemin adayı da çıktı:
50 mm yakınlık sınırındaki aday noktalar **73.362 → 172.694**, bu
noktaların düzleme p95 uzaklığı **46,5 → 44,5 mm**. İki ayrı nokta kümesi
ve düzlem uydurması kullanıldığından bu fiziksel zemin doğruluğu değildir;
serbest alan ve navigasyon haritası olarak kullanılmaz. Sahne çıktıları ve
ölçüm betiği yalnız `work/arcore-detail-confirmed-wifi-20261001-*` altında.

## 3. Şekil ve hareket kararı

1 Ekim kayıtlı veri denemesi tamamlandı. Dokuz test karesi haritalamadan
çıkarılarak aynı 240 kaynak kare karşılaştırıldı: ham ARCore → Depth Pro,
sehpa p95 düzlem sapması **61,77 → 11,81 mm**, açık halı **12,86 → 14,39 mm**.
Bu ölçüm yukarıdaki eski ±5cm yakınlık seçimi ölçümüyle aynı deney değildir;
aynı ortak z-buffer pikselleri ve her adayın kendi düzlemi kullanıldı.
Sehpa 9/9 bakışta daha düşük sapma verdi; zeminde kazanım yok. Aday 285.972
nokta ve 564 pozla kayıtlı ROS2/RViz gösterimine ulaştı. Fiziksel mm doğruluğu,
canlı AI ve bütün oda kalitesi doğrulanmadı. [Tam yöntem ve kanıt](ARCORE_AI_KAYITLI_DENEME_TR.md).

Kayıtlı renkli aday şimdi kamera yoluyla birlikte GLB yüzey dosyası ve
çevrimdışı döndürülebilir HTML olarak da çıkarıldı: 285.972 kaynak nokta
korundu, 269.812 köşe ve 1.363.572 üçgen; korunan XYZ farkı 0 m. Bu gösterim
ve kayıt adımıdır, geometri doğruluğuna yeni kanıt değildir. Üst duvar/tavan
kayıtta olmadığından tam oda çıktısı sayılmaz. [Dosyayı açma ve kalan sıra](ARCORE_ODA_CIKTISI_TR.md).

Metric Small ve TSDF ayrıca ölçüldü; halı sapması arttığı için mevcut
canlı varsayılanı değiştirmedi. Yeni yön: isteğe bağlı Depth Pro + ARCore poz,
en-yeni-kare kuyruğu, bağımsız poz/yol güncellemesi. Bu GPU'da 240 kare AI
çıkarım medyanı 431,94ms; eski paketleri sırayla işleyip gecikme biriktirmek
yerine önce kayıtlı replay'de bu kuyruğu ölçmek gerekiyor. Sonra kullanıcı
hazır olduğunda fiziksel sehpa yüksekliğiyle ölçek kontrolü yapılacak.
Doğrulanmamış tahminle serbest alan veya engel güvenliği kararı verilmeyecek.

## 4. Daha sonra: yer tanıma ve nesne etiketleri

Kalıcı harita üzerinde yeniden konum bulma için fiziksel olarak örtüşen iki
oturum ve ortak koordinat doğrulaması gerekir; şu anki yol oturuma özgüdür.
Bu doğrulanmadan robot navigasyonu veya tutma konumu ilan edilmeyecek.
YOLO/segmentasyon daha sonra aynı RGB akışında çalışıp yalnız derinliği
**ölçülmüş** maskelere etiket ve 3B merkez ekleyebilir. Aynı kamera görüntüsü
kör noktaya yeni geometri sağlamaz. Üçüncü kamera ve model eğitimi bu ilk
harita kalitesi denemesinin ön koşulu değildir.

YOLO26n-seg ve s-seg 273 kayıtlı karede ayrıca denendi (3,87/4,22ms medyan),
ancak dokuz sehpa bakışında tutarlı sehpa maskesi alınmadı. Geometriyi
otomatik nesne etiketine göre değiştirmek şimdilik uygulanmadı. AGPL
lisans notu README'de. Yeni kalibrasyon istemeden kayıtlı aday üzerinde
devam edilecek; kamera kullanıcının izni/düğmesi olmadan açılmayacak.

Kayıtlı ev görüntüleri, harita bulutları ve model ağırlıkları GitHub'a
eklenmeyecek; bütün yeni deney çıktıları ayrı `work/` klasörlerinde tutulacak.
