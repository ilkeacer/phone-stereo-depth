package org.research.phonestereo

import org.junit.Assert.assertArrayEquals
import org.junit.Test

class DepthProjectionTest {
    @Test fun textureIntrinsicsProjectDepthPixelInCameraAxes() {
        val projection=DepthProjection.fromTexture(floatArrayOf(600f,800f),
            floatArrayOf(320f,240f),intArrayOf(640,480),160,90)
        assertArrayEquals(floatArrayOf(0f,0f,-1f),projection.cameraPoint(80,45,1000),1e-6f)
        assertArrayEquals(floatArrayOf(.1f,.1f,-1f),projection.cameraPoint(95,30,1000),1e-6f)
    }
}
