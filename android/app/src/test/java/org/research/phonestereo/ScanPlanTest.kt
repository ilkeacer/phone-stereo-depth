package org.research.phonestereo

import org.junit.Assert.*
import org.junit.Test

class ScanPlanTest {
    @Test fun longScansHoldOnlyFirstFiveSeconds() {
        for (duration in ScanPlan.roomDurations) {
            val plan=ScanPlan(duration)
            for (elapsed in 0..4) assertTrue(plan.guidance(elapsed).contains("İlk 5 sn"))
            for (elapsed in 5 until duration) assertFalse(plan.guidance(elapsed).contains("İlk 5 sn"))
            assertTrue(plan.guidance(duration-10).contains("İstersen"))
        }
        assertEquals(300,ScanPlan.roomDurations[ScanPlan.defaultRoomIndex])
    }
    @Test fun shortCheckAndRemainingTimeDoNotBecomeLongScanPrompts() {
        val plan=ScanPlan(12)
        assertTrue(plan.guidance(0).contains("20–30 cm"))
        assertEquals(12,plan.remaining(-5))
        assertEquals(0,plan.remaining(99))
        assertEquals(600,ScanPlan(900).durationSeconds)
    }
    @Test fun onlyAnActiveManualStopIsSaved() {
        assertEquals("saved_by_user",ScanPlan.manualEndReason(true))
        assertEquals("stopped_by_user",ScanPlan.manualEndReason(false))
    }
}
