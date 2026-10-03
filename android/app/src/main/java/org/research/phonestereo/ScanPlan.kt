package org.research.phonestereo

/** Timing and prompts shared by short checks and manually finishable room scans. */
class ScanPlan(requestedSeconds: Int) {
    val durationSeconds = requestedSeconds.coerceIn(12, 600)

    fun remaining(elapsedSeconds: Int) = (durationSeconds - elapsedSeconds.coerceAtLeast(0)).coerceAtLeast(0)

    fun guidance(elapsedSeconds: Int): String {
        val elapsed = elapsedSeconds.coerceAtLeast(0)
        if (durationSeconds <= 15) return "20–30 cm yavaşça hareket et"
        if (elapsed < 5) return "İlk 5 sn: aydınlık, eşya kenarlı bir alanı sabit göster"
        if (remaining(elapsed) <= 10) return "İstersen başlangıçta gördüğün bölgeye yavaşça dön"
        return when (((elapsed - 5) / 30) % 4) {
            0 -> "Yavaş dolaş; zemin ve eşya kenarlarını birlikte göster"
            1 -> "Eşyaların yanlarını farklı açıdan göster; ani dönme"
            2 -> "Biraz yukarı bak: duvarları, dolap ve eşya üstlerini de göster"
            else -> "Aynı alanın bir kısmını tekrar göster; önceki görüntüyle örtüşsün"
        }
    }

    companion object {
        val roomDurations = listOf(180, 300, 600)
        const val defaultRoomIndex = 1
        fun manualEndReason(active: Boolean) = if (active) "saved_by_user" else "stopped_by_user"
    }
}
