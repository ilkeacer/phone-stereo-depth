# Kayıttan odanın görülen bölümünü 3B dosyaya çıkarma

Renkli yüzeyler, telefonun kayıtlı yolu ve son haritalanan derinlik karesinin
kamera işareti artık tek bir GLB dosyasında kaydediliyor. Yanındaki `viewer.html`
tek dosyalık, çevrimdışı döndürülebilir görüntüleyici. Kamera, telefon bağlantısı,
yeni model çıkarımı veya harici sunucu gerektirmez. WebGL 2 destekli tarayıcı gerekir.

Bu **tam oda rekonstrüksiyonu değildir**. Mevcut kayıt çoğunlukla zemini,
sehpayı, masayı ve dolapların alt bölümlerini gösteriyor. Üst duvarlar ve tavan
kapsanmıyor. Görülmeyen yüzeyler doldurulmuyor. Nesnelerde boşluklar, çoklu
bakışlardan üst üste binen yüzeyler ve tahmin hataları kalabilir. Çıktı kapalı
bir katı model veya robot için doğrulanmış engel haritası değildir.

## Bu adımda ölçülenler

Kaynak: önceden kaydedilmiş kablosuz ARCore oturumu, aynı 240 kareli Depth Pro
adayı ve ayrı renkli bulut. Kamera bu işlemde açılmadı.

| Ölçüt | Önce: renkli bulut | Sonra: 3B sahne |
|---|---:|---:|
| Korunan kaynak bulut noktaları | 285.972 | 285.972 |
| Üçgen yüzey sayısı | 0 | 1.363.572 |
| Kaydedilmiş kamera pozları | 564 | 564 |
| Yüzeyde kullanılan kaynak noktaları | — | 269.812 |
| Korunan köşelerin kaynak XYZ konumundan en büyük farkı | — | 0 m |

Son dışa aktarım süresi 3,86 saniye; model çıkarımı önceden yapılmıştı.
GLB 22.854.900 bayt, çevrimdışı HTML 30.484.488 bayt. Kaynak akış, bulutlar,
iki yol dosyası, manifestler ve 240 tahmin önbelleği dahil **248 girdi dosyasının**
SHA-256 değerleri işlem öncesi ve sonrası aynı. Renkli yol, model yoluyla ve
akıştan yeniden okunan bütün takip pozlarıyla karşılaştırılıyor. Son kamera
işareti kendi derinlik karesinin pozu ve intrinsics değerleriyle çiziliyor.

Yüzeyler aynı kayıtlı karede komşu olan piksellerden kuruluyor. Bütün köşeler
en az üç ayrı kareyle desteklenen mevcut 1 cm hücrelerden geliyor; her üçgenin
üç ayrı karede görülmüş olduğu iddia edilmiyor. Desteksiz noktaları, çöken
üçgenleri ve 5 cm / göreli %3 sınırını aşan derinlik sıçramalarını bağlamıyor.
Köşeler taşınmıyor; boşluk doldurma ve geometri yumuşatma uygulanmıyor.
**Üçgen sayısının artması fiziksel şekil doğruluğu artışı değildir.**

glTF eksen dönüşümü ROS +Z yukarıdan glTF +Y yukarıya yapılır; renkler glTF'nin
lineer renk tanımına çevrilir. Khronos glTF Validator 2.0.0-dev.3.10 dosyada
0 hata, 0 uyarı, 0 bilgi ve 0 ipucu bildirdi. Bu dosya biçimi kontrolüdür;
gerçek oda geometrisini doğrulamaz. 12 ilgili Python/ROS ortam testi geçti.

## Uzun oda taraması: uygulama v0.12

Telefon uygulamasında oda taraması artık **3 / 5 / 10 dakika** olarak seçilir;
varsayılan **5 dakika**. Önce oda düğmesi 60 saniyeydi, Activity'nin üst sınırı
120 saniyeydi; yeni üst sınır 600 saniye. 60 saniyelik ayrıntılı kısa test ve
12 saniyelik uyumluluk denemesi ayrı durur.

Bilgisayar dinleyicisi hazır olduğunda süreyi seçip **Ayrıntılı oda taramasını
başlat** düğmesine yalnız kullanıcı basar. İlk 5 saniye aydınlık bir eşya
kenarını sabit gösterir; sonraki sürede yavaş yan adımlarla zemini, eşya yanlarını,
duvarları ve üst yüzeyleri kapsar. Her yeni görüş öncekinin bir bölümünü içersin.
Tam aynı başlangıç noktasına dönmek şart değil. Süre dolmadan **Durdur ve kaydet**
ile bitirmek mümkündür. Geçen/kalan süre, anlık takip durumu ve RGB/derinlik
sayıları görüntülenir. Takip duraklarsa kayıt sürer, yavaşça aydınlık/dokulu
alana dönme yönlendirmesi gösterilir.

İlk-5-saniye mesajı kalan zamana göre belirlenmiyor: eski `remaining > 55`
kuralını uzun taramaya taşımak, uzun süre sabit tutma yönlendirmesine yol açardı.
Yeni plan geçen zamana bakar; 180/300/600 saniyelik sentetik zaman çizelgelerinde
yalnız 0–4. saniyelerde sabit tutma mesajı doğrulandı.

Kullanıcının bilinçli kaydı bitirmesi `saved_by_user` son paketi üretir. Bu ve
otomatik `finished` bitişi, ham replay/AI/3B dışa aktarım yollarında tamamlanmış
kayıt kabul edilir. İptal, arka plana geçiş, hata veya eksik son paket tamamlanmış
harita kabul edilmez. Kare desteği, güven, zaman damgası ve kaynak hash kontrolleri
korunur. Çok uzun taramaların telefon ısısı, depolama ve harita hücre kapasitesi
üzerindeki gerçek etkisi henüz ölçülmedi; hücre kaybı hâlâ `mapTruncated` ile
açıkça raporlanır.

v0.12 APK ve **15 Android + 35 Python/ROS testi** önceki hazırlıkta geçti.
1 Ekim'de APK telefona kuruldu; kullanıcı beş dakikalık taramayı tamamladı:
1.287 RGB/derinlik karesi ve 2.776 kayıtlı takip pozu. Telefonun kendi
raporunda 8.728 takip edilen / 17 bekleyen kamera karesi, hata yok.
Bilgisayar dinleyicisi çekim başlamadan önce eski 300 saniyelik başlangıç
bekleme süresinde kapanmıştı: canlı aktarımda 0 paket. Telefonun yerel
185.855.247 bayt kaydı alındı; cihaz/PC SHA256 aynı. Bu deneme canlı AI
başarısı olarak raporlanmaz. Yeni dinleyici kullanıcı düğmeye basana kadar
iptal edilebilir biçimde bekler; aktif akışta 20 saniyelik veri kesintisi
kontrolü korunur. 15 ilgili kuyruk/dinleyici testi geçti; yeni davranışın
telefonla canlı doğrulaması ayrıca bekliyor.

## Tek kareye özgü yüzey bağlantılarını ayırma

İsteğe bağlı `--min-face-frames 2`, aynı üçgenin iki ayrı derinlik karesinde
görülmesini ister. Bir karenin yinelenen pikselleri ek destek oluşturmaz.
Varsayılan 1 korunur; yeni seçenek bir geometri doğruluğu sonucu değil,
yüzey gösterimi için ayrı destek filtresidir.

| Aynı kayıtlı aday | Önce: en az 1 yüzey karesi | Ayrı seçenek: en az 2 |
|---|---:|---:|
| Üçgenler | 1.363.572 | 420.390 |
| Yüzeyde kullanılan köşeler | 269.812 | 223.413 |
| Korunan tam kaynak bulutu | 285.972 | 285.972 |
| Kamera pozları | 564 | 564 |

943.182 üçgen (%69,17) yalnız tek bağımsız karede görülmüş olduğu için ayrıldı.
Kaynak bulut dosyası bayt bayt aynı ve korunan köşelerde XYZ farkı 0 m;
248 girdi dosyası değişmedi. Daha fazla boşluk kalır; hatalı derinlik veya poz
kayması düzeltilmiş sayılmaz. Filtre bütün yüzeyleri elerse bulut/yol yine
kaydedilir ve GLB nokta görünümü üretir; yüzey yokluğu açıkça gösterilir.
Gerçek iki-kare GLB'si ve sentetik yüzeysiz GLB Khronos doğrulayıcısında
0 hata/uyarı verdi. Tüm oda kapsamı ve fiziksel şekil doğruluğu doğrulanmadı.

## Tekrar üretme ve açma

Her çalıştırmada var olmayan yeni bir çıktı dizini seçin:

```bash
python3 -m host.arcore_scene_export \
  --input work/arcore-detail-confirmed-wifi-20261001-01/stream.jsonl \
  --model-map work/arcore-depthpro-map-heldout-20261001-01 \
  --colored-map work/arcore-depthpro-map-colored-20261001-01 \
  --output work/room-scene-new
```

İki bağımsız kare desteği isteyen ayrı yüzey denemesi için komuta
`--min-face-frames 2` ekleyin ve yine yeni çıktı dizini seçin.

Çıktılar: `observed_room.glb`, üçgenli `observed_surfaces.ply`, kaynakla aynı
renkli `map_cloud.ply`, aynı `map_poses.txt`, `viewer.html` ve bütün girdi/çıktı
hash'lerini içeren `result.json`. Depo içindeki çıktılar yalnız Git'in yok
saydığı `work/`, `data/` veya `outputs/` altına yazılabilir. Mevcut dizinin
üstüne yazılmaz. Gerçek ölçüm ve doğrulama günlükleri özel yerel `work/`
altındadır; kaynak paketinde yer almaz.

`viewer.html` dosyasını doğrudan tarayıcıda açabilirsiniz. Sol tuşla döndürün,
sağ tuşla kaydırın, tekerlekle yaklaşın. **Yüzey / Noktalar**, **Üstten**,
**Tümünü göster** ve kamera yolunu gösterme düğmeleri var. HTML içinde bütün
sahne gömülüdür; ağ isteği veya CDN yok. Bu HTML de evin görüntülerini ve
haritasını içerdiğinden özel veridir. Public kaynak dışa aktarımı yalnız boş
görüntüleyici şablonunu kabul eder, üretilmiş HTML/GLB dosyalarını dışlar.

ROS 2'de bulut ve yolu kamera kapalıyken tekrar gösterme:

```bash
source /opt/ros/humble/setup.bash
python3 -m host.ros_arcore_saved --session work/room-scene-new --rviz
```

## Tam oda ve canlı sonuç için kalan sıra

1. İsteğe bağlı AI dinleyicisi ve en yeni kare kuyruğu hazır; kayıtlı gerçek GPU
   ROS replay'inde p95 gecikme 80,13 → 0,772 s, 564 poz korundu. Telefonla canlı
   AI denemesi bekliyor. Seyrek hızlı önizleme ile tüm kayıttan yoğun son harita
   farklı işlemler: [komutlar ve sınırlar](ARCORE_AI_CANLI_TR.md).
2. Zemin ve eşyaların farklı bakışlarda aynı konumda kalmasını değerlendirmek.
   Önceki sehpa ölçümü daha düşük düzlem sapması verdi; zemin ölçümünde kazanım
   yok. Dosyayı yüzeyle göstermek bu problemi çözmüş sayılmaz.
3. Kullanıcı tekrar hazır olduğunda duvarları, eşya yanlarını ve eksik üst
   yüzeyleri de gösteren yavaş oda turunu yalnız kendisi başlatır. Bu tur
   mevcut kaydın eksik kapsamını tamamlamak içindir; şimdi yeni test istenmiyor.

Kayıtta görülen bölümün kaydedilebilir 3B dosyası bugün var. Odanın tamamı için
eksik yüzeylerin gözlenmesi gerekir. Aynı dosyayı tekrar açmak, sonraki bir
oturumda telefonun eski haritada yerini bulduğu anlamına gelmez; çapraz oturum
konumlama ayrıca bekliyor. Gerçek oda ölçüsü ve güvenilir robot mesafesi hâlâ
fiziksel referansla doğrulanmadı.
