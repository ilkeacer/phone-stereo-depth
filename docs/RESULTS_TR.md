# Gerçek telefon deneyi — sonuç

İki farklı arka sensörden eşzamanlı akış kanıtlandı. Ekran damasıyla çekilen40 pozun uygun27 eğitim ve10 ayrı doğrulama pozu geometrik kalibrasyon için kullanıldı. Doğrulama medyan hatası0,49px, p95hatası1,45px; belirlenen1,5px sınırı geçildi.

Kayıtlı aynı görüntü çiftinde:

| Yöntem | Tutarlılık maskesinden geçen alan | Ölçülen hesap süresi |
|---|---:|---:|
| StereoSGBM | %19,6 | İki yön toplam123,8ms |
| RAFT-Stereo / RTX4080 | %62,8 | Sol198,7ms + ters yön199,6ms |

Bu alan oranları doğruluk veya güven yüzdesi değildir. RAFT daha dolu bir harita üretti; gerçek sahne derinliğiyle karşılaştırmalı doğruluk henüz ölçülmedi. İşlem süreleri uçtan uca gecikme değildir.

Canlı SGBM90s gerçek cihaz testinde210 harita üretti: beklemeler dahil ortalama2,33Hz. Bağımsız sensörlerin faz kayması nedeniyle uygun çiftler arasında18,16s ara da oluştu. Bu nedenle kesintisiz gerçek zamanlı derinlik henüz sağlanmış değildir. Eski haritanın sayısal değeri güncel ölçüm gibi gösterilmez.

`Stereo Derinlik` uygulamasında kayıtlıRAFT/SGBM sonuçlarını seçebilir, haritada bir bölgeye tıklayarak geçerli piksellerin medyanını görebilir veya canlıSGBM'i açabilirsiniz. Görüntüler ve derinlik aynı yönde gösterilir. Koyu alanlar maskelenmiştir.

**Birim şu anda dama karesi.** Ekrandaki gerçek kare kenarı ölçülmedi;17,3inç nominal panel ölçüsünden metre uydurulmadı. Metre ölçeği, bilinen mesafelerde doğruluk ve hareketli sahne testi kalan aşamalardır. Mevcut40poz ve hamfotoğraflar korundu; bu aşamada yeniden40poz çekmek gerekmiyor.

Yayın veya GitHub push yapılmadı. APK, kaynak ZIP, tekrar üretme komutları, kalibrasyon dosyaları, JSON sonuçlar ve örnek haritalar yerel olarak teslim edildi.
