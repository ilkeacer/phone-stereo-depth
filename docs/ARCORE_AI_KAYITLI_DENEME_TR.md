# Kayıtlı RGB + ARCore pozu ile AI harita adayı

1 Ekim 2026. Bu çalışma telefonu/kamerayı açmadı. Önceden alınmış kablosuz
kaydın 273 derinlik paketi, eşleşen RGB'si ve 564 takip pozu kullanıldı.
Kaynak SHA-256 işlem öncesi/sonrası aynı:
`c057b380669c74d98bcd99ae8ba6e76edcd18d429844d6bbd78cc692fce21ad7`.
Ev görüntüleri, sensör dosyaları, haritalar ve ağırlıklar yalnız yok sayılan
`work/` / `data/` dizinlerinde. Bu tur GitHub'a hiçbir şey gönderilmedi.

## Karşılaştırılan yöntemler

Telefon bu kayıtta ARCore **ham Depth API** verisi verdi. Önceki haritanın
derinliği Depth Anything V2 değildi. Ham derinliğin tek karede verdiği sehpa
ile birleşik harita birbirinden ayrı ölçüldü; poz ve derinlik hatasının
katkıları fiziksel referans olmadan tamamen ayrıştırılmadı.

Metric Small ve Apple Depth Pro, aynı kayıtlı CPU RGB üzerinde çalıştırıldı.
Görüntü ARCore yerçekimine göre dik çevrildi, tahmin tekrar native piksellere
döndürüldü ve kayıtlı dört köşe dönüşümüyle derinlik dokusuna örneklendi.
Depth Pro'ya odak uzunluğu kayıtlı ARCore iç parametrelerinden RGB kırpması
ve yönü hesaba katılarak verildi. Her tahmin, en az 500 kararlı/güven ≥192
ham pikselde medyan oranıyla ölçeklendi. **Bu referans fiziksel doğruluk
olarak doğrulanmadı.** 5 cm veya yakın derinliğin %3'ünden büyük sıçramanın
iki yanındaki pikseller atıldı; kenarlar bulanıklaştırılmadı.

Haritalar aynı 240 kareden, 1 cm hücre ve en az üç ayrı kare desteğiyle
oluşturuldu. İlk 24 kare referans desteği karşılaştırmasına alınmadı;
145,150,155,160,165,170,175,180,185 numaralı dokuz test karesi bütün
haritaların oluşturulmasından çıkarıldı. Test maskeleri RGB'deki beyaz sehpa
üstü ve açık halı bölgesidir; halı ROI'si native RGB'de [340,312,420,364].
Üç haritanın aynı görünür pikselleri karşılaştırıldı. Her yüzeye kendi düzlemi
uyduruldu; tablodaki p95, bu düzleme uzaklıkların yüzde 95 değeridir.
Dokuz bakışın medyanları verilir. Gerçek nesneye mesafe hatası değildir.

| Birleşik harita, aynı test pikselleri | Ham ARCore ortalama | Metric Small | Depth Pro |
|---|---:|---:|---:|
| Sehpa p95 düzlem sapması | 61,77 mm | 51,96 mm | 11,81 mm |
| Halı p95 düzlem sapması | 12,86 mm | 34,00 mm | 14,39 mm |
| Sehpa maskesi görünür piksel payı, medyan | %100 | %100 | %100 |
| Halı maskesi görünür piksel payı, medyan | %100 | %100 | %100 |

Depth Pro sehpa p95'i 9/9 bakışta ham haritadan düşük çıktı. Zeminde kazanım
gözlenmedi. Depth Pro adayı 285.972 nokta, 564 poz, 0 hücre kapasite reddi
verdi; 240 karenin çıkarım medyanı 431,94 ms. İlk dokuz kare denemesinde
tepe GPU tahsisi 3.791,76 MiB (float16) ölçüldü. Bütün adayın üretimi
111,95 s sürdü. Bu süre gerçek zamanlı yayın veya fiziksel telefon testi değildir.

Seçili dokuz **tek karede**, aynı maskede ham → ölçeklenen Depth Pro:
sehpa p95 medyanı 11,68 → 7,96 mm; halı 5,48 → 4,42 mm;
sehpa–halı yükseklik tahmininin açı aralığı 138,33 → 66,14 mm.
Fiziksel sehpa yüksekliği ölçülmedi. Bütün 240 karede kullanılan ölçek
katsayısı 0,741–1,638 arasında değişti; farklı oda parçalarının metre ölçeği
ve zamanlar arası tutarlılığı henüz doğrulanmış değildir.

TSDF (1 cm hücre, 4 cm truncation) aynı kayıtla ayrı denendi:
ham sehpa 61,77 → 41,40 mm, halı 12,86 → 63,78 mm.
Üç bakış desteği/free-space çelişki filtresi de bu adayı yeterli hale
getirmedi. TSDF canlı varsayılan yapılmadı. YOLO26n-seg ve YOLO26s-seg,
273 RGB karede sırasıyla 3,87 ve 4,22 ms medyan çıkarım verdi; dokuz sehpa
bakışında tutarlı sehpa maskesi vermedi. Nesne etiketi geometriye zorlanmadı.
Model eğitilmedi, üçüncü kamera açılmadı.

## Kod ve tekrar çalıştırma

`host/arcore_ai_map.py`: yalnız kayıtlı veri; tamamlanmış protokol, artan
kare/poz zamanları, eşleşen JPEG ve geometri, kaynak hash'i, yeni çıktı
klasörü zorunlu. JPEG boyutları decode öncesi denetlenir. GPU hatasında
kısmi kanıt tutulur ve `status=failed` manifesti yazılır; başka yeni klasöre
tekrar çalıştırılır. `host/depth_pro_model.py`: sabit resmî commit ve ağırlık
hash'i; fazladan kaynak/bytecode veya değiştirilmiş upstream reddedilir.
Model adaptörü indirme yapmaz, görüntüleri dışarı göndermez.

Depth Pro upstream commit:
`9e65e4dbe9568d23c546fcec53302b10445e109e`.
Ağırlık SHA-256:
`3eb35ca68168ad3d14cb150f8947a4edf85589941661fdb2686259c80685c0ce`.
Upstream kaynak `work/Depth-Pro-20261001`, ağırlık
`data/models/depth-pro-20261001/depth_pro.pt` altında. Kaynak `src` içine
model/ağırlık/başka dosya koymayın; adaptör bytecode yazmadan import eder.
Bağımlılıklar ana ortamı değiştirmeden
`work/depth-pro-runtime-20261001/pkgs` içinde tutuldu (timm 0.9.16,
Pillow-HEIF, safetensors, huggingface_hub). PyTorch 2.10.0+cu128 ve
RTX 4080 Laptop kullanıldı. Upstream kaynak lisans/bağımlılıkları ayrıca geçerlidir.

Yerelde gerçekten çalıştırılan bütün-kayıt komutu:

```bash
PYTHONPATH=work/depth-pro-runtime-20261001/pkgs:. HF_HUB_OFFLINE=1 OMP_NUM_THREADS=8 \
  .venv/bin/python work/arcore-depthpro-20261001-01/build_map.py \
  > work/arcore-depthpro-20261001-01/build_map.log 2>&1
```

Bu özel betik, `host.arcore_ai_map.build` çağrısına yukarıdaki 33 açık
karşılaştırma dışlamasını, `use_recorded_focal=True`, `save_depths=True`
ve 4.000.000 hücre kapasitesini verir. Kabul/güven/kalibrasyon eşiği
gevşetilmedi; yalnız hesaplama kapasitesi farklıdır.

Genel CLI (yeni çıktı adı ve kendi kayıt/yolunuzla):

```bash
PYTHONPATH=work/depth-pro-runtime-20261001/pkgs:. HF_HUB_OFFLINE=1 \
  .venv/bin/python -m host.arcore_ai_map --model depth-pro \
  --input work/<kayit>/stream.jsonl --output work/<yeni-aday> \
  --repository work/Depth-Pro-20261001 \
  --checkpoint data/models/depth-pro-20261001/depth_pro.pt \
  --cell-limit 4000000 --save-depths
```

## Tam yerel kanıt ve ROS

Özet dışında bütün sayılar ve kod şu özel dosyalarda tutuldu:

- `work/arcore-depthpro-20261001-01/metrics.json`: dokuz tek kare, bütün maskeler/noktalar/sayılar.
- `work/arcore-depthpro-20261001-01/heldout_metrics.json`: dokuz harita bakışı, bütün varyantların sayıları.
- `work/arcore-depthpro-20261001-01/build_map.py` ve `build_map.log`: gerçek çağrı ve tam konsol sonucu.
- `work/arcore-depthpro-map-heldout-20261001-01/result.json`, `frame_metrics.json`, `predictions/`: bütün model/füzyon ayarları ve kare kayıtları.
- `work/arcore-tsdf-comparison-20261001-01/`: ayrı ham/AI/TSDF ve filtre denemeleri, kod ve tam ölçümler.
- `work/arcore-segmentation-20261001-01/`, `work/arcore-segmentation-s-20261001-01/`: bütün 273 kare nesne etiketleri/maskeleri.
- `work/arcore-depthpro-20261001-01/rgb-depth-comparison.png`: özel aynı-kare RGB/derinlik görseli.

Ayrı renkli aday `work/arcore-depthpro-map-colored-20261001-01/`;
285.972 noktanın tamamı kayıtlı aynı hücre RGB ortalamasıyla renklendirildi.
XYZ farkı 0 m, orijinal geometri dosyasının hash'i değişmedi. Renklendirme
geometri doğrulaması değildir. Kayıtlı aday ve 564 poz ROS yayınından ayrı
aboneyle alındı; RViz açıldı. Kamerayı açmadan yeniden görüntüleme:

```bash
source /opt/ros/humble/setup.bash
python3 -m host.ros_arcore_saved \
  --session work/arcore-depthpro-map-colored-20261001-01 --rviz
```

22 ilgili Python testi geçti. Gerçek telefonda canlı Depth Pro çıkarımı,
yeni oturumda yer tanıma ve fiziksel geometri doğrulaması yapılmadı.

## Sonraki somut adım

En yeni kareyi işleyen isteğe bağlı AI ROS dinleyicisi artık hazır. Gerçek GPU
kayıt replay'inde FIFO kontrol → son-kare p95 gecikmesi 80,13 → 0,772 s;
564 pozun tamamı korundu. Daha az kare işlendiği için onaylı bulut daha seyrek.
[Ölçüm ve komutlar](ARCORE_AI_CANLI_TR.md). Mevcut canlı ham ARCore seçeneği
değişmedi; telefonla AI denemesi henüz yapılmadı. Kullanıcı hazır olmadan
yeni kamera testi yok.

Kullanıcı hazır olduğunda fiziksel kontrol için gereken tek basit ölçü:
sehpanın zeminden üst düz yüzeyine dik yüksekliği, mm olarak. Yeni çekim
ancak kullanıcı isterse; kameranın düğmesine kullanıcı basar. Bu aşamada
yeni dama veya kamera-IMU kalibrasyonu istenmiyor.

## Lisans

[Depth Pro upstream lisansı](https://github.com/apple-aiml-research/ml-depth-pro/blob/main/LICENSE)
kod ve ağırlıklar için geçerlidir; üçüncü taraf bileşenleri ayrı bildirilir.
Metric Small Apache-2.0'dır. Ultralytics YOLO kod/ağırlıklarının varsayılan
lisansı AGPL-3.0; [resmî lisans açıklaması](https://www.ultralytics.com/license)
README'ye eklendi. Bu not projenin bütününe ayrıca bir lisans atamaz.
