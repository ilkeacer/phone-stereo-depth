# Telefon stereo → ROS 2 → 3B harita

Amaç, telefon hareket ederken RTAB-Map ile görsel odometri ve 3B haritalama. Dama deneyleri haritalama ekranının parçası değildir. Mevcut kalibrasyon kullanılır.

## Kullanım

Güncel panelde 90 sn rehberli oda testi ve 30 sn haritasız kamera önizlemesi var. Kablosuz bağlantı için `scripts/start_ros_wifi_dashboard.sh` kullanılır; önce görüntüyü önizlemede kontrol edin. Ölçülen kablosuz durum ve adımlar `docs/ROS_WIFI_TR.md` içinde. Aşağıdaki 2 dk düğme açıklaması önceki sürümün tarihsel kaydıdır.

Kontrol panelinde:

- **Kayıttan harita oluştur:** Mevcut 30 saniyelik hareket kaydını ROS'ta yeniden işler. Telefon gerekmez. Sonuç yeni bir oturuma kaydedilir.
- **2 dk oda haritası:** Telefonla yeni haritalamayı başlatır. Hazırlık sırasında sabit tut; ekranda HAREKET ET görünce kısa bir yol boyunca yavaşça ilerle, çevreyi yavaşça göster ve başlangıca dön. Dama hedefi gerekmez.
- **Durdur ve kaydet:** Önce görüntü kaynağını, sonra haritalamayı kapatır; veritabanı ve uygun harita çıktısını kaydeder.
- **Bu haritayı 3B aç:** Seçili/son oturumun nokta bulutunu ve kamera yolunu açar. Başarısız oturum yerine sessizce eski bir harita göstermez.
- **Kayıtlı haritalar:** Önceki oturumları nokta/poz sayısıyla listeler. Parçalı sonuçlar “parça” olarak işaretlenir.
- **ROS / RViz görünümü:** Çalışan oturumda canlı ROS haritasını, oturum yoksa kaydedilmiş haritayı gösterir.
- **30 sn tanılama:** Kısa sabit–öteleme–sabit kayıt için ayrı seçenek; normal oda haritalaması değildir.

Her oturum `work/ros-live-*` veya `work/ros-replay-*` altında ayrı tutulur. `map/map.db` ROS harita veritabanı; `export/map_cloud.ply` nokta bulutu; `export/map_poses.txt` kamera yolu. `summary.json`, `lifecycle.json`, `export-result.json` ve günlükler aynı klasördedir. Başarısız dışa aktarma denemeleri ayrı klasörlerde korunur ve tekrar denenebilir.

## Bu sürümde yapılan doğrulama

Mevcut kısa kayıtla ROS haritalama, otomatik kapanış, dosya dışa aktarma ve panel üzerinden tekrar çalıştırma gerçek kurulu ROS 2 Humble / RTAB-Map üzerinde çalıştırıldı. Yeni telefon çekimi yapılmadı.

Tam replay örnekleri:

| Oturum | Gözlemciye ulaşan sol görüntü | Pozitif / toplam odometri | Bağlantılı poz | PLY nokta |
|---|---:|---:|---:|---:|
| ros-replay-5m7gc6lj | 226 | 220 / 221 | 10 | 7777 |
| ros-replay-yz4hv8m8 | 225 | 220 / 221 | 10 | 7688 |
| ros-replay-z8095h05 | 225 | 222 / 223 | 10 | 7720 |

Kaynak kayıtta 226 çift vardır. Gözlemci sayısı yayıncının teslim garantisi değildir. ROS replay zamanlaması, başlama/eşzamanlama ve işleme örneklemesi nedeniyle odometri/harita sonuçları birebir deterministik değildir. Bu tabloda farklı çalıştırmalar tek bir sonuç gibi birleştirilmedi.

Eski `ros-live-8G3rpXNQ` oturumunun terk edilmiş işlemleri kapatıldı ve SQLite yedeği `ros-recovered-20260914` içine alındı. 74 aktif poz 7 ayrı bileşende: en büyük parça 28 poz. Dışa aktarılan parça 12155 nokta içerir; tüm oda veya tüm 74 pozun tek haritası değildir.

Otomatik testlerde: işlem lideri ölse bile alt işlemlerin kapanması, zorla kapatma kaydı, kilit bırakma, parçalı grafiğin etiketlenmesi, hatalı oturumun başarılı dışa aktarım sayılmaması, dışa aktarım tekrar denemesi, ROS mesajları ve ayrı kayıtlı-harita çerçevesi kontrol edildi. GUI kapanışında ROS alıcı iş parçacığının bitmesi beklenir.

## ROS bağlantıları

- Canlı stereo: `/phone/left/image_rect`, `/phone/right/image_rect` ve karşılık gelen `camera_info`.
- RTAB-Map canlı nokta bulutu: `/rtabmap/cloud_map`, `map` çerçevesi.
- Kayıtlı bulut/yol: `/phone/saved_map`, `/phone/saved_path`, `saved_map` çerçevesi. Bu ayrı görselleştirme çerçevesi canlı robot haritasıyla karışmaz.
- Gövde çerçevesi `phone_link`; sol/sağ optik kamera dönüşümleri yayımlanır. Robotun gerçek gövde–telefon montaj dönüşümü henüz ölçülmüş değildir.
- Varsayılan yerel ROS domain 72. Panel ve yayıncı aynı ROS_DOMAIN_ID ve ROS_LOCALHOST_ONLY ayarlarıyla çalışmalıdır.

## Kalan fiziksel doğrulama

Harita oluşturma/görüntüleme yazılım akışı kayıtlı veride çalışıyor. Mevcut veri küçük bir alanı kapsıyor; tam oda doğruluğunu veya robot üzerinde güvenilir hareket takibini kanıtlamıyor. Ölçek, kullanıcının ekran ölçülerinden tahmini. Gerçek ölçek ve telefonun robot gövdesine montajı robot üzerinde doğrulanmalı.

Kullanıcı hazır olduğunu söyleyene kadar yeni kamera testi başlatılmayacak. Sonraki fiziksel adım, normal haritalama düğmesiyle ayrıntılı/aydınlık bir oda sahnesinde kısa bir hareket rotasıdır. Yeni kalibrasyon çekimi bu adımın ön koşulu olarak istenmiyor.
