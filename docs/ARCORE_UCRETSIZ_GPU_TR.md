# Ücretsiz yerel GPU ile kayıtlı harita birleştirme

Kayıtlı derinliği aynı harita hücrelerinde birleştiren isteğe bağlı CUDA yolu
eklendi. Telefon kamerasını açmaz, model çıkarımı yapmaz, görüntü göndermez.
Mevcut Ubuntu 22.04 / ROS 2 Humble / sürücü kurulumu değiştirilmedi.
Ücretli API, abonelik veya bulut hizmeti kullanılmadı.

## Ölçülen sonuç

Aynı kaydın aynı 240 tahmin karesi, aynı CPU piksel/poz dönüşümü, 1 cm hücre
ve en az üç ayrı kare desteği kullanıldı. Bir karedeki çok sayıda piksel
birden fazla görüş sayılmıyor. Önce kare içindeki noktalar, sonra her hücreyi
gören farklı kareler eşit ağırlıkla ortalanıyor.

| Ölçüt | Önce: mevcut CPU birleştirme | Sonra: CUDA toplu birleştirme |
|---|---:|---:|
| Birleştirme süresi | 5.704,40 ms | 347,04 ms |
| Giriş kareleri | 240 | 240 |
| Giriş noktaları | 3.273.632 | 3.273.632 |
| Çıkış noktaları | 285.972 | 285.972 |
| Kaydedilmiş kamera pozları | 564 | 564 |
| En düşük ayrı kare desteği | 3 | 3 |
| Kapasite nedeniyle reddedilen hücre | 0 | 0 |
| Önceki haritayla en büyük XYZ farkı | 0 m | 0 m |

CPU süresi bütün `ConfirmedMeanVoxels.add` çağrılarının toplamıdır. GPU süresi
112,63 ms CPU hücre adresleme/paketleme ve 234,41 ms veri aktarımı/GPU azaltma
toplamıdır. CPU ve GPU haritaları nokta sırası dahil aynı; PLY ve yol dosyaları
bayt bayt eşit. Bu süreler **birer kayıtlı çalıştırmanın** ölçümüdür; canlı FPS
veya tekrarlı performans medyanı değildir. Dosya okuma, piksel/pozdan 3B'ye
dönüşüm, Torch/CUDA ilk yükleme ve model çıkarımı bu karşılaştırmaya dahil değil.
CUDA komutunun önbellekten tam haritayı yazması ayrıca 1,84 s sürdü.

Modelin daha önce ölçülen 431,94 ms/kare medyan çıkarım süresi değişmedi.
Bu adım haritanın fiziksel şeklini değiştirmedi; sehpanın veya zeminin gerçek
ölçülerine daha yakın olduğunu göstermiyor. Kayıtlı bulut ve 564 poz mevcut ROS
kayıt yükleyicisinde okunabiliyor; canlı AI yolu henüz bağlanmadı.

## Tekrar çalıştırma

CUDA destekli PyTorch ve mevcut proje bağımlılıklarını içeren ortam gerekir.
Bu reducer nvblox yüklemesini gerektirmez. Yerel geçerli tahmin önbelleğini
kullanın ve her defasında var olmayan yeni çıktı dizini seçin:

```bash
.venv/bin/python -m host.arcore_gpu_map \
  --input work/arcore-detail-confirmed-wifi-20261001-01/stream.jsonl \
  --model-map work/arcore-depthpro-map-heldout-20261001-01 \
  --output work/arcore-gpu-new
```

`map_cloud.ply`, aynı `map_poses.txt` ve tam `result.json` yazılır.
Konsol çıktısı `gpuBatchStats`, ayarlar ve dosya hash'lerini içerir; tam girdi
SHA listeleri `result.json` içindedir. Kaynak akış, iki manifest, yol ve 240
tahmin dahil **244 girdi** işlem öncesi/sonrası değişmedi. Önce/sonra XYZ
karşılaştırması bunlara ek olarak orijinal `map_cloud.ply` dosyasını okur:
bu karşılaştırmanın girdi kapsamı **245 dosyadır**.

Toplu reducer en fazla 5 milyon giriş noktası ve 2 milyon farklı hücreyi kabul
eder. Taşmayı sessizce kırpmak yerine işlemi reddeder. Daha uzun oda taramasında
aynı toplu sınırların yeterli olduğu henüz ölçülmedi. Tamamlanmamış veya kırpılmış
cache, farklı zaman damgası, intrinsics, yol ya da kaynak SHA reddedilir.
Doğrulama eşikleri değiştirilmedi.

Gerçek yerel kanıt: `work/arcore-gpu-confirmed-20261001-01/run.py`, onun tam
`console.log` / `result.json` dosyaları; son CLI çıktısı
`work/nvblox-runtime-20261001-01/gpu-confirmed-02.log` ve
`work/arcore-gpu-confirmed-20261001-02/result.json`. Bu özel kanıt dosyaları
public kaynak paketinde bulunmaz.

## Ayrı NVIDIA nvblox denemesi

[Standalone nvblox kurulum](https://nvidia-isaac.github.io/nvblox/v0.0.10/pages/installation.html)
ve [Torch yeniden oluşturma örnekleri](https://nvidia-isaac.github.io/nvblox/v0.0.10/pages/torch_examples_reconstruction.html)
temel alındı. Ayrı sanal ortamda resmî 0.0.10 CUDA 12 / Ubuntu 22.04 wheel'i
kullanıldı; ana Python ortamı ve ROS kurulumu değiştirilmedi. Wheel SHA-256:
`9ffb308c2d94a01321d8c2558183118903cfcd4b13fe881fae197d9e5b5c96a8`.
GPU TSDF yüzeyi gözlemler arasında hesaplama yapabilir; ağırlık değeri bağımsız
kare sayısı değildir. Kaynak yöntemdeki üç-kare desteğine eşdeğer sayılmaz.

Aynı 240 kare, 1 cm TSDF / 2 hücre kesim uzaklığı ve 0,5–5 m aralık kullanıldı.
Ham derinlik ve önbellek Depth Pro, hem mesafeye göre hem sabit ağırlıkla denendi.
Harita yapımına katılmamış dokuz sehpa/halı görüşünde aynı RGB maskeleri kullanıldı.
Yüzey köşelerinin z-buffer örneklemesi ile kaynak nokta örneklemesi farklıdır;
aşağıdaki kapsama ve kendi düzlemine sapma sayıları fiziksel doğruluk değildir.
Her önce/sonra çifti kendi ortak görünür piksellerini kullanır; farklı satırların
sapmaları birbirine karşı doğrudan model sıralaması değildir.

| Önce → TSDF seçeneği | Sehpa kapsama medyanı | Sehpa için yeterli görüş | Halı p95 düzlem sapma medyanı |
|---|---:|---:|---:|
| Ham ortalama → mesafe ağırlığı | %100 → %53,47 | 9 → 8 | 12,97 → 17,85 mm |
| Depth Pro ortalama → mesafe ağırlığı | %100 → %3,99 | 9 → 3 | 14,86 → 20,83 mm |
| Ham ortalama → sabit ağırlık | %100 → %30,89 | 9 → 9 | 12,68 → 63,91 mm |
| Depth Pro ortalama → sabit ağırlık | %100 → %13,33 | 9 → 5 | 14,87 → 12,23 mm |

Yeterli piksel olmayan sehpa görüşleri için sayı uydurulmadı ve seçilmiş alt
kümeyle dokuz-görüş sapma medyanı raporlanmadı. Ham/sabit seçenekte dokuz görüş
ölçülebildi: sehpa p95 sapma medyanı 52,87 → 91,79 mm. Bu sonuçlarla TSDF ana
yöntem yapılmadı. Son sabit/Depth Pro çalıştırması 282.708 köşe, 385.733 üçgen ve
564 poz verdi; aktarım dahil kare birleştirme medyanı 6,32 ms. Bu süre model
çıkarımı ve canlı iletişimi içermez, şekil doğruluğunu göstermez.

İsteğe bağlı deney komutu; uygun ayrı nvblox ortamında:

```bash
python -m host.arcore_nvblox \
  --input work/arcore-detail-confirmed-wifi-20261001-01/stream.jsonl \
  --model-map work/arcore-depthpro-map-heldout-20261001-01 \
  --depth-source cache --weighting constant \
  --output work/nvblox-new
```

Diğer ayrı denemeler için `--depth-source raw` veya `--weighting distance`
kullanılır. Gerçek yerel ölçüm kodu/tam çıktı:
`work/arcore-nvblox-comparison-20261001-02/evaluate.py`, `metrics.json`,
`console.log`. Kalite tablosu Depth Pro için `arcore-nvblox-depthpro-20261001-02`
(mesafe) ve `-03` (sabit); ham için `arcore-nvblox-raw-20261001-01` (mesafe)
ve `-02` (sabit) çıktılarından ölçüldü. `work/` altındaki ayrı son `-04` çıktısı
TSDF etiketlerini yeniden üretir; kalite tablosu o çıktıda tekrar ölçülmedi.
Son çalıştırmanın ayar getter'ları, giriş/çıktı hash'leri ve süreleri
`work/arcore-nvblox-depthpro-20261001-04/result.json` içinde. Görüntüleyici
yerel tarayıcıda sahneyi çizdi, hata konsolu boştu. Son GLB Khronos denetiminde
0 hata / 0 uyarı ve 1 bilgi mesajı verdi: native TSDF mesh'te 9.324 çökmüş üçgen.
Bu da ayrı deney çıktısının sınırlamasıdır; ana gözlem yüzeyi değiştirilmedi.

27 ilgili proje testinin 26'sı geçti; bir ROS ortam testi atlandı. GPU reducer'ın
üç testi gerçek CUDA'da geçti. Kurulu nvblox'un 10 resmî GPU testi ayrıca geçti.
Bu kontroller gerçek oda geometrisini doğrulamaz.

## Lisans ve sonraki adım

nvblox [Apache-2.0](https://github.com/nvidia-isaac/nvblox/blob/public/LICENSE.md)
lisanslı. Metric Small Apache-2.0; bu kayıtlı adayın Depth Pro kod/ağırlıkları
[Apple'ın ayrı lisansına](https://github.com/apple-aiml-research/ml-depth-pro/blob/main/LICENSE)
tabi. Ücretsiz çalışması bütün bileşenlerin aynı açık kaynak lisansına sahip
olduğu anlamına gelmez. Ultralytics seçenekleri AGPL-3.0; proje ana lisansı
henüz seçilmedi. Bu adımda YOLO, cuVSLAM veya ücretli hizmet bağlanmadı.

En yeni RGB/derinlik karesini işleyen kuyruk ve AI'dan bağımsız poz/yol
güncellemesi artık ayrı isteğe bağlı dinleyicide hazır;
[gerçek kayıtlı GPU/ROS ölçümü](ARCORE_AI_CANLI_TR.md) mevcut. Telefonla AI canlı
denemesi bekliyor. Kullanıcı hazır olduğunda v0.12'nin 5 dakikalık taramasıyla
duvarlar/eşya yanları/zemin kapsamı denenecek. Şimdi yeni çekim gerekmez.
Ev görüntüleri, haritalar ve model ağırlıkları Git/public paket dışında kalır.
