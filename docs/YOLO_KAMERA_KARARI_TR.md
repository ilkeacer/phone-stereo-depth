# Üçüncü kamera ve YOLO: mevcut kanıt ve karar

## Cihaz kanıtı

Yerel 12 `data/raw/**/discovery.json` donanım keşfinde Android'in uygulamaya bildirdiği `cameraIdList` her seferinde `["0", "1"]`, `concurrentCameraIds` ise boş. Buna karşılık özel ID 20+21'in iki ayrı fiziksel kamera olarak 65 saniyeden uzun eşzamanlı çalıştığı ayrıca ölçüldü. Bu nedenle boş API listesi 20+21 sonucunu geçersiz kılmaz; **0+20+21 veya ARCore+20+21 için hiçbir çalışma kanıtı yoktur**. [Android'in çoklu kamera kılavuzu](https://developer.android.com/media/camera/camera2/multi-camera), garantili birleşimlerin ötesindeki akışların cihaz üzerinde denenmesi gerektiğini söyler. Kullanıcı şu anda fiziksel test yapamadığı için üçüncü kamera açılmadı.

ARCore ile eşzamanlı ikinci Camera2 oturumu açmak mevcut 3B yolu riske atar. ARCore'un [paylaşımlı kamera API'si](https://developers.google.com/ar/reference/java/com/google/ar/core/SharedCamera) ayrı bir entegrasyondur; etkin ARCore sırasında `setRepeatingRequest` kullanımına izin vermez ve dokümana göre derinlik sensörünü kullanamaz. Mevcut ayrıntılı derinlik modunda böyle bir geçişin yararı gösterilmedi. Bu aşamada üç kamera talebi veya kamera paylaşımı uygulanmayacak.

## Aynı kameranın görüntüsü yeterli

YOLO bir RGB görüntüde nesne kutusu üretir; fiziksel olarak üçüncü lens istemez. Mevcut Camera2 stereo yolunda sol görüntü zaten ROS'ta `/phone/left/image_rect` olarak vardır. ARCore yolunda [aynı `Frame` üzerinden kamera görüntüsü alınabilir](https://developers.google.com/ar/reference/java/com/google/ar/core/Frame#acquireCameraImage()); şu anki ARCore uygulaması mahremiyet ve yük nedeniyle RGB görüntü göndermiyor. İleride aynı ARCore kare zaman damgasıyla düşük hızlı, isteğe bağlı bir RGB yan akış kurulabilir. Görüntü→derinlik→3B nesne konumu için aynı kameranın iç parametreleri, derinlik ve poz kullanılmalı; farklı üçüncü kamera eklenirse ayrıca kamera-kamera dış kalibrasyonu ve zaman eşleştirmesi gerekir.

YOLO harita noktasını veya kamera pozunu tek başına doğrulamaz. Nesne algılamasını odometri ve derinlik akışından ayrı, sınırlı kuyruklu bilgisayar işlemi olarak tutmak gerekir. Haritalama kararlılığı ölçülmeden YOLO'yu telefonun çizim/derinlik döngüsüne eklemeyeceğiz.

Kullanıcının hedefi tamamen açık kaynak bir robot yazılımı. YOLO'nun olası yararı: ölçülmüş derinlik ve kameranın aynı zamanlı pozu bulunan bir nesneye semantik ad eklemek; ileride hareket eden insan/hayvan gibi nesneleri harita güncellemesinden ayırmak; tekrar görülen **ayırt edici** nesneleri konumlandırma için aday işaret olarak değerlendirmek. Tek bir sınıf etiketi (ör. “sandalye”) güvenilir loop closure değildir. Kutular hareketli nesneyi maskelerken fazla arka planı da silebilir; bu iş için gerektiğinde piksel maskesi ve zamansal doğrulama gerekir. YOLO kameranın görmediği alanın uzaklığını vermez, o alanı güvenli/boş olarak işaretleyemez. Aynı kamerayı paylaşmak haritayı yavaşlatmadan algılama yapma olanağı sağlar ama fiziksel görüş açısını büyütmez.

## Ölçülmüş dönüş ve ölçülmemiş bölge

`host.arcore_observation_grid` kayıtlı 3B noktalardan ve pozlardan 10 cm XY hücreli **tanı katmanı** üretir; boşluklara derinlik atamaz. 26 Eylül v0.9 uzun oturumundaki 56.880 nokta/344 pozla çalıştırıldı: kamera yolu etrafında 3 m paylı **keyfi** 69×74 hücreli pencerede 2.698 hücrede kaydedilmiş derinlik dönüşü yok; 35 hücre yalnız zemin düzlemi adayı, 1.162 hücre yalnız diğer 3B dönüş, 1.211 hücrede ikisi de var. Pencere oda sınırı değildir; bu sayılar oda kapsama yüzdesi, engel haritası veya serbest alan ölçümü değildir. Yerel görüntü ve tam metrikler `work/arcore-observation-20260929-v2/` altında, GitHub dışında. YOLO etiketi ancak kendi görüntü pikseliyle **aynı karede ölçülmüş** derinlik/pozla eşleştirilirse 3B haritaya aday olarak eklenebilir.

```bash
.venv/bin/python -m host.arcore_observation_grid \
  --session work/arcore-room-v09-20260926132325 \
  --output work/arcore-observation-yeni --resolution 0.1 --padding 3
```

## Telefon açılmadan GPU ölçümü

`scripts/benchmark_yolo_offline.py` yalnız mevcut bir resim ve yerel model ağırlığını okur; kamera açmaz, görüntü veya işaretli çıktı kaydetmez, yeni sayısal JSON rapor oluşturur. İsteğe bağlı Ultralytics paketi ile model ağırlığı `work/` altında tutulur; ana çalışma bağımlılığına veya GitHub'a eklenmez. [Ultralytics'in lisans açıklamasına göre](https://www.ultralytics.com/license) paket ve model için AGPL-3.0 veya ayrı ticari lisans koşulları vardır; robotta dağıtım öncesi model seçimi ayrıca yapılmalıdır.

29 Eylül çevrimdışı denemesinde RTX 4080 Laptop GPU üzerinde `yolo26n.pt` kullanıldı. Halka açık örnek görsel 1280×960'a yeniden boyutlandırıldı ve modele 640 girdi verildi. Beş ısınma sonrası 20 çalışmanın medyan uçtan uca çıkarım süresi **3,943 ms**, p95 **4,171 ms**; her çalışmada 1 kutu vardı. Ağırlık SHA-256: `9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef`. Tam yerel rapor `work/yolo-eval-20260929/project-size-benchmark.json`. Bu yalnız boşta GPU'da tek görüntü çıkarımıdır; telefon aktarımı, ARCore derinlik, ROS, Depth Anything ile eşzamanlı yük, odadaki nesnelerin doğruluğu ve canlı gecikme **ölçülmedi**.

Tekrarlanabilir komut (Ultralytics ve CUDA PyTorch isteğe bağlı ortamda kurulu, model ağırlığı yerel olmalı):

```bash
PYTHONPATH=work/yolo-eval-20260929/pkgs .venv/bin/python -m scripts.benchmark_yolo_offline \
  --image work/Depth-Anything-V2/assets/examples/demo09.jpg \
  --weights work/yolo-eval-20260929/yolo26n.pt \
  --report work/yolo-eval-20260929/yeni-olcum.json \
  --source-width 1280 --source-height 960 --image-size 640 --device 0
```

Tam açık kaynak hedefi lisans seçimini ayrıca gerektirir. Önceki çevrimdışı YOLO26n denemesi Ultralytics/AGPL ile **yerel ve isteğe bağlı** yapıldı; ana uygulamaya katılmadı. Ultralytics'i doğrudan dağıtılacak projeye entegre etmek AGPL-3.0 uyumluluğu ister. Daha serbest proje lisansı istenirse [YOLOX'un resmî deposu](https://github.com/Megvii-BaseDetection/YOLOX) Apache-2.0 lisanslı bir algılayıcı adayıdır; seçilecek ağırlığın kökeni/lisansı ayrıca denetlenmelidir. Projeye henüz bir lisans atanmadı ve kullanıcının “tam açık kaynak” sözü lisans metni seçimi olarak yorumlanmadı.

Öncelik: önce kullanıcının başlatacağı ham/yumuşatılmış derinlik karşılaştırmasıyla harita yüzeyini denetle. Sonra aynı ARCore kamera görüntüsünden isteğe bağlı 1–2 FPS algılama yan akışı kurup harita FPS, takip kesintisi ve gecikmeyi ölç. YOLO sonucu önce yalnız etiket katmanına girecek; zemin/engel/serbest alan ve kamera pozu ile karıştırılmayacak. Yalnız gerçekten gerekli olursa üçüncü kamera için ayrı, kısa donanım uygunluk testi planla.
