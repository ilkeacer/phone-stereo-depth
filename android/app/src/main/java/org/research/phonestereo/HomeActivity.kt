package org.research.phonestereo

import android.app.Activity
import android.content.Intent
import android.os.Bundle
import android.widget.Button
import android.widget.LinearLayout
import android.widget.TextView
import android.widget.ScrollView
import android.widget.Spinner
import android.widget.ArrayAdapter

/** Manual launcher; desktop ADB tools keep addressing MainActivity directly. */
class HomeActivity : Activity() {
    override fun onCreate(state: Bundle?) {
        super.onCreate(state)
        val layout = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(28, 32, 28, 24)
        }
        layout.addView(TextView(this).apply {
            text = "Telefon 3B harita denemesi"
            textSize = 24f
        })
        layout.addView(TextView(this).apply {
            text = "Oda taraması: Önce bilgisayardaki dinleyici hazır olsun. İlk 5 sn aydınlık bir eşya kenarını sabit göster; sonra yavaşça dolaş. Yalnız zemine bakma: eşya yanlarını, duvarları ve üst yüzeyleri de göster. Küçük yan adımlar at; aynı alanın bir kısmını farklı açıdan tekrar göster. Tam aynı yere dönmen şart değil. Haritayı bilgisayar oluşturur."
            textSize = 17f
            setPadding(0,20,0,12)
        })
        layout.addView(TextView(this).apply {
            text = "Oda taraması süresi (önerilen: 5 dakika). İstediğin anda Durdur ve kaydet ile erken bitirebilirsin."
            textSize = 17f
        })
        val roomDuration=Spinner(this).apply {
            adapter=ArrayAdapter(this@HomeActivity,android.R.layout.simple_spinner_dropdown_item,
                listOf("3 dakika", "5 dakika (önerilen)", "10 dakika"))
            setSelection(ScanPlan.defaultRoomIndex)
        }
        layout.addView(roomDuration)
        layout.addView(Button(this).apply {
            text = "Ayrıntılı oda taramasını başlat"
            setOnClickListener {
                startActivity(Intent(this@HomeActivity,ArCoreProbeActivity::class.java)
                    .putExtra("durationSeconds",ScanPlan.roomDurations[roomDuration.selectedItemPosition])
                    .putExtra("rawDepth",true))
            }
        })
        layout.addView(TextView(this).apply {
            text = "Ayrıntılı mod RGB görüntülerini, ham derinliği ve pozu yalnız yerel kayda alır. Düz, desensiz yüzeylerde boşluk kalabilir. Sayaç, takip durumu, RGB/derinlik sayıları ve otomatik ISO/poz değerleri taramada görünür."
            textSize = 15f
            setPadding(0,12,0,12)
        })
        layout.addView(Button(this).apply {
            text = "60 sn ayrıntılı kısa test"
            setOnClickListener {
                startActivity(Intent(this@HomeActivity,ArCoreProbeActivity::class.java)
                    .putExtra("durationSeconds",60).putExtra("rawDepth",true))
            }
        })
        layout.addView(TextView(this).apply {
            text = "ARCore kısa deneme: Aydınlık zemini ve sabit eşya kenarlarını kadraja al. Başlatınca telefonu 12 saniye boyunca yavaşça 20–30 cm hareket ettir. Görüntü dosyası kaydedilmez; poz ve derinlik sayıları cihazda tutulur."
            textSize = 17f
            setPadding(0, 20, 0, 20)
        })
        layout.addView(Button(this).apply {
            text = "ARCore 12 sn derinlik denemesi"
            setOnClickListener { startActivity(Intent(this@HomeActivity, ArCoreProbeActivity::class.java)) }
        })
        layout.addView(TextView(this).apply {
            text = "Görüntü kaydetmeden kısa, yumuşatılmış derinlik önizlemesi. Tam oda taraması için yukarıdaki ayrıntılı modu seç."
            textSize = 16f
            setPadding(0, 20, 0, 8)
        })
        layout.addView(Button(this).apply {
            text = "60 sn derinlik önizlemesi"
            setOnClickListener {
                startActivity(Intent(this@HomeActivity, ArCoreProbeActivity::class.java)
                    .putExtra("durationSeconds", 60))
            }
        })
        layout.addView(TextView(this).apply {
            text = "Stereo kamera donanım tanısı uzun sürebilir. Masaüstündeki ROS paneli kameralara doğrudan bağlanır."
            textSize = 15f
            setPadding(0, 32, 0, 8)
        })
        layout.addView(Button(this).apply {
            text = "Gelişmiş stereo donanım tanısı"
            setOnClickListener { startActivity(Intent(this@HomeActivity, MainActivity::class.java)) }
        })
        setContentView(ScrollView(this).apply { addView(layout) })
    }
}
