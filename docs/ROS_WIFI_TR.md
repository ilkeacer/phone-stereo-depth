# Kablosuz telefon görüntüsü ve ROS2 haritası

Mi 9T Pro'nun iki arka kamerası artık ADB'nin **Wi-Fi bağlantısı** seçilerek aynı stereo protokolüyle bilgisayara ulaşabiliyor. Kameranın Android kodu, kalibrasyon, 20 ms stereo sınırı ve 1 saniyelik tazelik reddi değiştirilmedi. Her ADB komutu ve `tcp:8765` yönlendirmesi `PHONE_ADB_SERIAL` ile **aynı kablosuz cihaza** sabitleniyor; USB ve Wi-Fi aynı anda bağlıysa yanlış cihaza geçilmiyor. ROS2 paneli canlı sol kamerayı gösterir, RTAB-Map haritası için `ROS / RViz görünümü` düğmesi vardır.

Paneli açmak için:

```bash
scripts/start_ros_wifi_dashboard.sh
```

İlk kurulumda telefon USB ile yetkili ve aynı yerel Wi-Fi ağında olmalıdır. Başlatıcı mevcut yetkili Wi-Fi ADB bağlantısını seçer; yoksa USB'den telefonun Wi-Fi IP'sini alıp ADB TCP bağlantısını kurar, iki yolun aynı telefon olduğunu denetler. **Paneli açmak kamerayı başlatmaz.** Bağlantı kurulduktan sonra USB kablosu çıkarılabilir; yalnızca kablosuz yol gerçekten bağlı kaldığında test başlatılmalıdır. Telefon IP'si veya ağı değişirse USB ile yeniden bağlayıp aynı başlatıcıyı çalıştırın. Bu bağlantı, güvenilen yerel ağdaki ADB hata ayıklaması içindir.

Panelde önce **“Kamera önizlemesi · 30 sn (harita yok)”** düğmesine basın. 4 saniyelik hazırlıktan sonra gerçek sol kamera görüntüsü görünür; bu adım yalnız sahne ve bağlantı kontrolüdür, harita kaydetmez. Telefonu **dik tutun**. Kamera sensörü yatay piksel dizisi üretir; masaüstü paneli görüntüyü varsayılan olarak dik gösterir. Gerekirse **“Yatay göster” / “Dik göster”** düğmesine basın. Bu düğme yalnız ekrandaki resmi çevirir; stereo hesaplama veya ROS görüntüsü dönmez. Panel gösterilen karenin medyan parlaklığını ve 50/255 altındaki piksel oranını yazar; **“Karanlık sahne”** uyarısı yalnız bilgi verir, testi engellemez. Telefonu aydınlık, hareketsiz, ayrıntılı eşyalara (kitaplık, sandalye, masa kenarları) çevirin. Karanlık perde/boş duvar/parlak ekran kadrajın çoğunu kaplamasın. Önizleme sonunda kamera kapanır. Düğme çalışırken **“Önizlemeyi durdur”** ile erken bitirilebilir.

Görüntü uygunken **“90 sn rehberli oda testi”** düğmesine basın, açılan yönergeleri okuyup “Hazırım”ı seçin. İlk 5 saniye başlangıçta sabit durun; sonraki 30 saniyede aynı eşyalara bakarak 0,5–1 m kadar yavaş yana gidin; 20 saniyede 20–40 cm ileri; 30 saniyede aynı yolu geri; son 5 saniye başlangıçta sabit. Telefonu yalnız bilekten döndürmeyin. Canlı 3B noktaları görmek için oturum sürerken **“ROS / RViz görünümü”** düğmesine basın. Kapanıştan sonra **“Bu haritayı 3B aç”** veya **“Kayıtlı haritalar”** ile sonucu inceleyin. Bir takip kaybında başarısız harita kabul edilmez; ham görüntüler korunur.

**“3 dk tüm odayı dolaş”** ayrı bir testtir. Önce 30 saniyelik önizlemede ışığı ve kadrajı kontrol edin. Oda turunu başlatınca **hazırlık süresi sayılmaz**; 1/4 aşaması ve sayaç görünene kadar sabit bekleyin. 0–5 saniye sabit durun; 5–145 saniye güvenli ve yavaş bir turla erişebildiğiniz oda bölümlerini dolaşın; 145–175 saniye başlangıç bölgesine yavaşça yaklaşın; 175–180 saniye sabit durun. Telefonu dik ve iki elle tutun, yumuşakça dönün, yalnız bilekten sallamayın; her birkaç adımda ortak eşyalar kadrajda kalsın. Rotayı veya dönüş açısını ölçmeniz gerekmez. Test 180 saniyede kendiliğinden biter; isterseniz **“Durdur ve kaydet”** ile erken sonlandırabilirsiniz. Takip kaybolursa harita kabul edilmez ama ham kayıt sona kadar sürer; kayıtlı bölümler daha sonra ayrı haritalar olarak sınanabilir. Ekrandaki “akış kesildi” uyarısında yürümeyi durdurun.

Bu hareket mesafeleri **yaklaşık adım tarifidir**; cetvel, robot hassasiyetinde yol, açı veya tam başlangıca dönüş ölçümü istenmez. Başlangıç yerine kâğıt koymak ve kameranın baktığı bir eşyayı seçmek yeterlidir. Son 90 saniyelik testin ağırlıkla karanlık tezgâh/ekran bölgesini göstermesi nedeniyle bu kez aydınlık bir odada eşyaları kadrajın çoğunda tutun. Telefonu test boyunca dik ve yaklaşık göğüs hizasında, iki elle, ani dönme yapmadan taşıyın. Ekranda takip kaybı görünürse görüntü kaydı sürer; yeni bir test başlatmadan mevcut kaydın bitmesini bekleyin. Harita uygun bulunmazsa aynı hareketi hemen tekrarlamak yerine kaydı inceleyin.

26 Eylül zemin denemeleri `work/ros-live-0qepzv_b` ve `work/ros-live-s_72e45b` sırasıyla 949/935 stereo çifti ve 150/923 ile 469/919 başarılı odometri sonucu kaydetti. Her ikisinde takip kaybı bulunduğu için veritabanları yalnız tanı amaçlıdır; kesintisiz zemin haritası kabul edilmez. İkinci kayıtta telefoto görüntünün örnek karelerindeki medyan gri değer 32/255, başarılı eski oda kaydında 72,5/255 idi (her ikisi 20 ms, ISO 400). Sonraki canlı çekimler için varsayılan ISO 800'e çıkarıldı; pozlama süresi, odak ve kırpma değişmedi. Bu ayarın 90 saniyelik zemin haritasına etkisi henüz ölçülmedi. Panel önizlemesinde telefoto kadrajında zemine ek olarak halı/eşya kenarı ve yeterli ışık görünmeden yeni uzun kayıt başlatılmamalı.

ISO 800 ile 30 saniyelik canlı önizleme `work/ros-live-sljfbp86` içinde 429 stereo çift ve 423/427 başarılı odometri sonucu verdi; gerçek kamera meta verisi iki kamerada da ISO 800 ve 20 ms gösteriyor. Örnek telefoto karelerinin medyan gri değeri 42/255'tir. Bu kısa önizleme uzun kaydın başarısını kanıtlamaz. Telefona yüklenen uygulama telefoto önizlemesini büyük gösterir; iki kameranın kare sayısı, ortalama parlaklığı ve gerçek ISO değerini ayrı yazar. **“AZ IŞIK”** uyarısı yalnız bilgi verir, kaydı engellemez ve odometri başarı ölçüsü değildir.

## ARCore için kısa yol deneyi

Sonraki 90 saniyelik AI + stereo denemesinde 827 çift ve 826 AI derinlik karesi oluştu, ancak 821 odometri sonucunun yalnız 593'ü takipte kaldı. Sahne parlaklığı medyan 114,5/255 olduğundan takip kaybı yalnız karanlıkla açıklanamıyor. Alternatif olarak telefonda ARCore oturumu açıldı: sabit 12 saniyelik derinlik denemesinde 318/347 kare `TRACKING`, 32 derinlik görüntüsü 160×90, fakat örneklenen tüm derinlikler sıfır. Cihaz içindeki destek sorgusu `isDepthModeSupported(AUTOMATIC)=true` döndürdü; bu tek başına harita üretimi değildir. Google'ın [Depth API kılavuzu](https://developers.google.com/ar/develop/java/depth/developer-guide) hareket veya izlenen görsel özellik yokken derinliğin bulunamayabileceğini açıklar.

Telefonun uygulama simgesine dokunulduğunda seçim ekranı açılır. **“ARCore 12 sn derinlik denemesi”** seçeneğinde aydınlık zemin ve mobilya kenarını kamera önizlemesine alıp telefonu 20–30 cm yavaşça hareket ettirin. Sonuç ekranı takip edilen kare, derinlik karesi ve 3B nokta sayısını gösterir. Bu tanı fotoğraf veya video kaydetmez. Yerel PLY ve poz günlüğü `host.arcore_probe_import` ile mevcut RViz biçimine çevrilebilir; sonuç `diagnostic` olarak etiketlenir. `host.ros_arcore_live` ADB üzerinden yalnız poz ve 3B noktaları canlı ROS2 `/phone/arcore_path` ve `/phone/arcore_map` konularına yayınlar. Hareketli gerçek cihaz akışı aşağıdaki ölçümlerde çalıştı.

Canlı denemeyi bilgisayardan başlatmak için (telefon düğmesine **komut hazır** yazısından sonra basılır):

```bash
source /opt/ros/humble/setup.bash
PHONE_ADB_SERIAL=<secilen-adb-serisi> python3 -m host.ros_arcore_live \
  --output work/arcore-live-yeni-deneme --rviz
```

Hem USB hem Wi-Fi ADB bağlıysa `PHONE_ADB_SERIAL` zorunludur. Program yeni klasöre yalnız yerel tanı bulutu/yolu yazar. Nokta yoksa oda haritası dosyası oluşturmaz. ARCore dünya koordinatı ROS Z-yukarıya dönüştürülür; doğruluk ve sonraki oturumda aynı haritada konum bulma henüz kanıtlanmadı.

**26 Eylül hareketli cihaz ölçümü:** Kullanıcının düğmeye kendisinin bastığı 12 saniyelik ARCore denemesinde 334 telefon karesinin 317'si takipte, 17'si duraklamada; 32 derinlik görüntüsünün 26'sında sıfırdan büyük örnek vardı. Medyan geçerli derinlik 1.324,5 mm. Telefon 24.960 örnek 3B nokta çıkardı. ROS dinleyicisi 66 paketin 63'ünde takip aldı, 0 paketi reddetti ve 5 cm hücrelemeden sonra **9.852 noktalı bulut / 63 pozlu yol** kaydetti. ROS2'de `/phone/arcore_map` (`sensor_msgs/PointCloud2`) ve `/phone/arcore_path` (`nav_msgs/Path`) konularının her birinde bir yayıncı ve bir RViz abonesi gözlendi. Kamera yolu 10,758 s içinde 42,7 cm uzunluk; ilk-son poz arası 41,1 cm. Bu değerler ARCore tahminidir, cetvelle ölçülmüş gerçek yol değildir.

Bulutun yerel sınırları X −2,412…0,813 m, Y 0,316…4,829 m, Z −1,246…0,279 m. Görsel incelemede kısmi/eğri bir yüzey görülüyor; zeminin temiz ve düz tamamı veya oda duvarları henüz çıkarılmadı. Yakın-yatay bir düzleme ±5 cm'de uyan noktalar 1.517/9.852 (%15,4); bu oran sahnede eşya da bulunabileceğinden tek başına derinlik hatası yüzdesi değildir. Fotoğraf/video bu ARCore testinde kaydedilmedi. Telefonun poz+derinlik günlüğü ve PLY'si yalnız `work/` altında; 24.960 ham noktanın ayrı dönüştürülmesi 63 pozu yeniden üretti. Kaydedilen bulut GitHub'a eklenmedi.

Uygulamada **“ARCore 60 sn canlı oda haritası”** seçeneği de var: ilk 5 saniye sabit, ardından yaklaşık 45 saniye zemini ve sabit eşya kenarlarını örtüşmeli biçimde yavaşça tara, son 10 saniye başlangıç bölgesine yaklaş. Sayaç, takip ve 3B nokta sayısı telefonda görünür. Önce ROS dinleyicisi hazır olmalı, sonra kullanıcı telefondaki düğmeye kendisi basmalı. Kamera, bilgisayardan otomatik başlatılmaz.

**v0.8 uzun cihaz denemesi:** Telefon 1.520 kare/1.503 takip, 100 derinlik karesi, 333.572 ham nokta kaydetti. ROS 304 paket/301 takip/3 duraklama/0 ret ile 101.748 adet 5 cm harita hücresi ve 301 poz çıkardı. ARCore tahmini kamera yolu 5,402 m; ilk-son poz farkı 0,061 m. Bulutun Z sınırları −9,223…0,356 m ve yataya yakın yüzey adayı 5.388/101.748 (%5,3) hücredir. Görsel olarak aşağı uzayan uzak/niteliksiz noktalar var; bu çıktı temiz oda zemini veya robot için güvenli alan değildir.

**v0.9 kısa karşılaştırma:** Google'ın derinlik pikseli için kullandığı doku kamerası iç parametreleri derinlik çözünürlüğüne ölçeklendi; kabul aralığı 0,5–5 m yapıldı. Yeni 12 saniyelik kullanıcı kaydında telefon 338 kare/324 takip, 32 derinlik karesi/27 pozitif, 25.920 ham nokta; ROS 67 paket/65 takip/2 duraklama/0 ret, 8.422 hücre/65 poz kaydetti. Bulut Z sınırları −1,189…−0,051 m. Yerel yatay yüzey ayrıştırıcısı 2.047/8.422 (%24,3) aday hücre, 3,552° yatay eğim, kamera konumunun yaklaşık 0,995 m altında düzlem buldu. Önceki 12 s kayıtta aynı ayrıştırıcı 1.469/9.852 (%14,9), 8,475° buldu. Çekimler aynı piksel/sahne olmadığından bu fark projeksiyon değişikliğinin nedensel etkisini kanıtlamaz. Fiziksel zemin, metrik ölçek ve diğer oturumda aynı haritaya konumlanma doğrulanmadı.

**v0.9 uzun cihaz denemesi:** Telefon 1.733 kare/1.719 takip, 115 derinlik karesi/103 pozitif, 367.200 ham nokta kaydetti. ROS 346 paket/344 takip/2 duraklama/0 ret ile 56.880 adet 5 cm hücre ve 344 poz üretti. ARCore tahmini yol 5,869 m; ilk-son poz farkı 0,049 m. Bulut Z sınırları −1,793…0,636 m. Aynı ayrıştırıcı 7.995/56.880 (%14,1) aday hücre ve 1,119° düzlem eğimi buldu; düzlem kamera medyanından 1,304 m aşağıda. v0.8 uzun kaydında karşılık gelen sayılar 5.388/101.748 (%5,3), 8,974° ve Z en az −9,223 m idi. Bu iki tur aynı kareleri çekmedi; sayılar nitelik karşılaştırmasıdır, tek değişkenli algoritma deneyi değildir. Yeni bulutta yerel bir yatay bant ve çevresinde yükselen yüzeyler seçiliyor; oda ölçüsü, zemin güvenliği ve fiziksel ölçek bağımsız olarak denetlenmedi. Telefon görüntüsü kaydedilmedi, bütün nokta/poz dosyaları `work/` altında kaldı.

Kaydedilmiş buluttan ayrı, kaynak dosyalarını değiştirmeyen yeşil yüzey adayı oluşturma:

```bash
python3 -m host.arcore_floor \
  --cloud work/<oturum>/map_cloud.ply \
  --poses work/<oturum>/map_poses.txt \
  --output work/<oturum>/floor-analysis
```

`classified_map.ply` tüm bulutu ve yeşil adayı, `surface_candidate.ply` yalnız adayı, `floor_metrics.json` ham ölçüleri içerir. `occupied10cmCellAreaM2` yalnız örnek bulunan 10 cm XY hücrelerinin toplamıdır; ölçülmüş oda alanı değildir. Bunlar engel/boş alan veya robot navigasyonu haritası değildir. Sonraki adım, kayıtlı haritayı yeniden açma ve ikinci oturumun aynı haritada konumlanıp konumlanmadığını sınama; yeni telefon kaydı bu inceleme için hemen gerekmez.

**Kaydedilen haritayı yeniden açma** (telefon kamerası çalışmaz):

```bash
source /opt/ros/humble/setup.bash
python3 -m host.ros_arcore_saved --session work/<oturum> \
  --classified work/<oturum>/floor-analysis/classified_map.ply --rviz
```

v0.9 uzun kaydın yerel çıktısı bu yolla ROS'a tekrar verildi. Bağımsız abone 56.880 nokta (7.995 yeşil aday) ve 344 poz aldı; `/phone/arcore_map` ile `/phone/arcore_path` konularının her birinde 1 yayıncı ve 1 RViz abonesi vardı. Bu, eski oturumun kendi yolunu gösterir; telefon ikinci kez açıldığında eski haritada nerede olduğunun bulunduğunu **göstermez**.

Telefonun jiroskopu ve ivmeölçeri yeni Android uygulamasında stereo oturumu boyunca otomatik örneklenir. Ham Android eksenleri ve sensör zamanları oturumun `imu.jsonl` dosyasında, jiroskop ve en yakın ivme örneği ROS2 `/phone/imu_raw` (`sensor_msgs/Imu`) konusundadır. Kullanıcının ayrıca sensör açması veya ölçüm yapması gerekmez. Kamera–IMU katı dönüşümü, jiroskop sapması ve sensör füzyonu henüz doğrulanmadığı için bu akış RTAB-Map pozunu **şimdilik değiştirmez**. Jiroskop dönmeyi ölçer; konumu/metre ölçeğini tek başına ölçmez. Masaüstü önizlemesini çevirmek de kayıtlı haritanın dünya eksenini değiştirmez.

Kısa cihaz doğrulamasında `work/imu-smoke-20260925/capture.json` **104 stereo çift**, `imu.jsonl` **353 jiroskop + 353 ivme** satırı, `imuDroppedOnPhone=0`, `error=null` kaydetti. Her sensörde ölçülen medyan örnek aralığı **20,011 ms**, kaydedilen dizide sıra boşluğu **0**. Bağımsız ROS2 abonesi `/phone/imu_raw` üzerinden 12 mesaj aldı. Bu testte telefon sabitti; hareketli robotta sensör füzyonu veya yön hatası doğrulanmış sayılmaz.

Son kullanıcı 90 saniyelik turu `work/ros-live-3d5dtans` dizininde **981/981** stereo çift ve **4.498 jiroskop + 4.498 ivme** örneğiyle tamamladı; telefondaki tampon kaybı **0**. SGBM haritalama kayıt başlangıcından yaklaşık 28,65 saniye sonra `lost=true` aldı; 363 olumlu / 1 kayıp takip sonucundan sonra harita dışa aktarımı güvenlik gereği atlandı. Ham çiftlerin tamamı korundu. 31 telefoto örneğinin medyan parlaklığı **38/255**; piksellerin medyan **%80,15**'i 50/255'in altındaydı. Son 50 kayıp öncesi olumlu takipte özellik eşleşmesi medyanı **60**, kayıp karesinde **13** idi. Örnek kareler koyu perde ve eşyalara yoğunlaşıyor; bu birliktelik tek başına kesin neden kanıtı değildir. Kayıp sınırı etrafındaki beş kare dışlanarak **361** ve **615** çiftlik bağımsız aralıklar oluşturuldu. İlk aralık çevrimdışı oynatmada **7 poz / 27.838 nokta** üretti; ikinci aralık SGBM ile ilk üç görüntü içinde tekrar takip kaybetti. İkinci aralığın eski stereo yöntemiyle ayrı oynatımı **312/591 olumlu takip**, **6 grafik parçası** ve yalnız en büyük parçada **4 poz / 4.926 nokta** üretti; bu parçalı tanı sonucudur, tam oda haritası değildir. Aralıklar tek harita sayılmıyor. Okunabilir tam sayılar `work/ros-live-3d5dtans-audit-20260925.json`, `work/ros-live-3d5dtans-independent-maps-20260925/fragment-index.json` ve `work/ros-replay-ljjhrsmx/summary.json` içindedir.

## 25 Eylül 2026 ölçümü ve sınır

Telefon USB ile fiziksel olarak takılıydı, fakat seçilen kablosuz ADB aygıtı ve `adb forward --list` çıktısı stereo yönlendirmesinin **Wi-Fi taşıması** üzerinden olduğunu doğruladı. Yerel ağ adresi yayımlanmıyor. USB çıkarılmış durumda uzun hareket testi henüz yapılmadı.

| Deneme | Ölçülen çıktı |
|---|---|
| 10 sn kablosuz stereo | 121/121 çift kaydedildi; stereo zaman farkı medyan 4,225 ms, en büyük 7,600 ms; tazelik üst sınırı medyan 175 ms, en büyük 302 ms; hata yok |
| ROS2 görüntü dinleyicisi | `/phone/left/image_rect` üzerinde 20 adet 640×480 `mono8` görüntü alındı ve ilk kare kaydedildi |
| 15 sn kablosuz ROS2/SGBM | 202/202 çift kaydedildi; 175 derinlik yayımlandı, 27 kuyrukta daha yeni çiftle değiştirildi; 201/201 takip sonucu olumlu, 0 kayıp, temiz kapanış |
| Harita | Telefon sabit ve karanlık perde/ekran sahnesine bakıyordu: 1 aktif poz, 0 bağlantı; 3B harita dışa aktarımı doğru biçimde atlandı |

Kaydedilen ROS görüntüsünün gri ortalaması 38,29/255, medyanı 39/255; piksellerin %79,61'i 50'nin altında. Bu ölçüm sahnenin karanlığını gösterir; Wi-Fi bağlantısının harita kalitesini bozduğunu **kanıtlamaz**. Uzun ve hareketli, aydınlık oda testinin sonucu hâlâ bilinmiyor. Gerçek telefon görüntüsü, komut günlüğü, tam özet ve metrikler `outputs/WIFI-ROS-20260925/SONUC.md` içindedir.

## ARCore ayrintili harita v0.10

v0.9'daki eşya geçişleri yuvarlak ve yumuşak görünüyor. Kullanılan API `acquireDepthImage16Bits()` boşlukları doldurulmuş/yumuşatılmış derinlik veriyor. [Google, Raw Depth ile farkı burada açıklıyor](https://developers.google.com/ar/develop/java/depth/raw-depth). Eski uzun kayıtta 115 adet 160×90 derinlik karesi vardı; telefon 2 piksel aralıkla örnekliyordu. Bilgisayar her 5 cm hücrede son noktayı tutuyordu; ek bir yüzey yumuşatma işlemi yapmıyordu.

Aynı kaydedilmiş 367.200 XYZ örneği tekrar işlendi: 5 cm → 56.880 hücre; 2 cm → 236.208 hücre. Hücre temsilcisine örnek uzaklığının p95'i 51,26 → 18,65 mm. Bu sayı fiziksel eşya hatası değildir; yalnız örnek azaltma etkisidir. Aynı 20 cm genişlikteki kesitte yuvarlanmış yüzey profili iki çıktıda da kaldı. Eski kayıtta ham derinlik ve güven dizileri bulunmadığı için Raw Depth sonradan üretilemez.

**Yeni mod:** telefonda **“ARCore 60 sn ayrıntılı harita”**. Bu mod bütün yerel derinlik piksellerini, 0–255 güven değerlerini, ölçeklenmiş kamera iç parametrelerini ve kamera pozunu sayısal olarak saklar. Aynı ARCore kamera güncellemesinden eski yumuşatılmış çıktı da alınır. Sonraki sürümde aynı kareye ait RGB JPEG de özel akışa eklendi; kayıt ev görüntüsü içerebilir ve yalnız telefonda/`work/` altında tutulmalıdır.

Ham harita 0,5–5 m aralığı ve güven ≥128 ile çalışır. Daha önce görülen ham derinlik zaman damgası yeniden bağımsız gözlem sayılmaz. Varsayılan 2 cm haritada en az iki farklı ham derinlik güncellemesi gerekir; `--confirmed-map` ile 1 cm ve üç ayrı karede ortalama kullanılır. Bunlar doğruluk iddiası değildir. Eksikler yumuşatılmış referansla doldurulmaz. Düz/desensiz yerlerde boşluk kalabilir. 1 Ekim kablosuz deneme ve sayısal karşılaştırma için `docs/ARCORE_HARITA_KALITESI_YOL_HARITASI_TR.md`.

Bilgisayarda önce dinleyici:

```bash
source /opt/ros/humble/setup.bash
python3 -m host.ros_arcore_detail --serial <secilen-adb-serisi> \
  --output work/arcore-detail-yeni --rviz
```

Hazır mesajından sonra telefondaki düğmeye **kullanıcı** basar. İlk 5 saniye sabit tutulur; ardından zemini ve belirgin bir eşya kenarını birlikte görerek yavaş yan adımlar atılır. Aynı kenar birkaç farklı açıdan görülür. Ani yerinde dönüş yerine küçük öteleme yapılır. Son 10 saniyede başlangıç bölgesine yaklaşılır. Boşluk görünmesi testi bitirmez; görüntüleri üst üste düşürmek için yavaş hareket sürdürülür.

ROS'ta ana bulut `/phone/arcore_map`, yol `/phone/arcore_path`. Eşleştirilmiş karelerin karşılaştırması için `/phone/arcore_paired_raw` ve `/phone/arcore_smooth_reference` var; RViz'de bu iki karşılaştırma görünümü başlangıçta kapalıdır. Ana bulutu kapatıp bunlardan birini tek tek açın. Yumuşatılmış referansı alınamayan ham kare ana haritaya girebilir, fakat karşılaştırma haritalarına girmez.

Oturum çıktıları: `map_cloud.ply` tekrar görülen ham noktalar, `raw_observed.ply` tek gözlemler dahil ham noktalar, `paired_raw_reference.ply` ve `smooth_reference.ply` aynı kare kümesinin karşılaştırması, `stream.jsonl` alınan tam sayısal veri, `depth_comparison.json` kare metrikleri, `result.json` ayarlar/sayaçlar. Bitiş paketi gelmezse `eof_without_end_marker` kaydedilir; kayıp paketin üstü örtülmez. Telefon kendi tam arşivini `arcore-detail-<zaman>.jsonl` adıyla saklar; bağlantı kaybolursa bu dosya sonradan çekilebilir.

Yeni çekim olmadan aynı arşivi yeniden işleme:

```bash
python3 -m host.arcore_detail --replay work/<oturum>/stream.jsonl \
  --output work/arcore-detail-tekrar
```

İlk hedef, aynı eşya kenarında ham/referans derinlik ve aynı karelerden biriken bulutları karşılaştırmak. Yüksek nokta sayısı veya daha keskin görünen kenar, tek başına gerçek geometri doğruluğu değildir. Bu aşama mevcut eşya şekli sorununa yöneliktir; oturumlar arası konumlama hâlâ bekliyor.

**Pozlama (v0.11):** ARCore modunda telefonun Camera2 isteğine elle ISO yazılmıyor. Uygulama, ARCore'un [kare meta verisinden](https://developers.google.com/ar/develop/java/camera-metadata) yaklaşık saniyede bir gerçekleşen ISO, poz süresi ve AE modunu okur; telefonda gösterir ve görüntü içermeyen `arcore-probe-<zaman>.jsonl` günlüğüne `exposure` satırı olarak yazar. Meta veri yoksa değer uydurulmaz; sayaç `exposureUnavailable` olur. Bu sadece gözlemdir, derinlik/poz hesabına müdahale etmez. [ARCore paylaşımlı kamera](https://developers.google.com/ar/reference/java/com/google/ar/core/SharedCamera) başka bir entegrasyon gerektirir ve aktif ARCore oturumunda Camera2 `setRepeatingRequest` kullanılamaz. Stereo yolunun sabit 20 ms/ISO 800 deneyi ayrıca durur. Hareketli yeni testte gerçek ISO ve poz süreleri ile takip boşlukları birlikte incelenecek; otomatik ISO'nun haritaya olumlu etkisi ölçülmedi.

## Haritaya kalan doğrulama

Canlı ROS2/RViz 3B görüntüleme yolu hazır: önceki USB oturumu 15 bağlantılı poz ve 19.539 nokta üretti. 25 Eylül'deki yeni **hareketli Wi-Fi** oturumu 906 stereo çift, 895 derinlik karesi, 26 bağlantılı poz ve 125.448 dışa aktarılan nokta üretti. RViz kayıtlı bulutu ve yolu aldı. Bununla birlikte ham görüntüler ağırlıkla karanlık tezgâh/monitor ve yakın eşya bölgesini gösteriyor; tüm oda kapsamı ve geometrik doğruluk doğrulanmadı. `capture.json` Wi-Fi taşımasını kaydetti; inceleme anında `adb devices` yalnız Wi-Fi bağlantısını gösterdi. Kayıt süresince kablonun fiziksel durumu ayrıca ölçülmedi. Ayrıntılı kaynak-hash'li rapor `outputs/Son-Kablosuz-Harita-20260925/SONUC.md` içindedir.

Bir sonraki fiziksel adım gerekiyorsa, yukarıdaki 30 sn önizleme ile gerçekten aydınlık ve oda yüzeylerini kapsayan kadrajı seçmek ve sonra 90 sn yavaş tur atmaktır. Kayıt bittiğinde yazılım uygun haritayı otomatik dışa aktarır; uygun olmayan kaydı başarılı oda haritası diye sunmaz.

İlk kabul denetimi: en az %95 gözlenen takip, 1 saniyeyi aşan kesintisiz kayıp olmaması, tek bağlantılı poz grafiği ve temiz kapanış. Aynı başlangıç konumu/yönü fiziksel olarak gerçekten tekrarlandıysa ham odometri dönüş hedefi en fazla 10 cm ve 5°; bağımsız nesne uzunluğu hedefi en fazla %5 bağıl hata. Bu değerler mevcut oturumun sonucu değildir, önceden belirlenmiş hedeflerdir. Harita alanı ayrıca görünen oda yüzeyleri ve eksik/çift yüzeyler üzerinden incelenecek. Robot montajı için kamera-robot dönüşümü, hareket sırasında bağlantı ve mümkünse enkoder/IMU doğrulaması ayrı aşamadır; mevcut veriden bitiş tarihi çıkarılamaz.
