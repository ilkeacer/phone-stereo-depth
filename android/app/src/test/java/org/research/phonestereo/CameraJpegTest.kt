package org.research.phonestereo

import org.junit.Assert.assertArrayEquals
import org.junit.Test
import java.nio.ByteBuffer

class CameraJpegTest {
    @Test fun planarYuvWithPaddingAndUvPixelStrideBecomesNv21() {
        val y=ByteBuffer.wrap(byteArrayOf(99,1,2,3,4,0,0,5,6,7,8)).apply { position(1) }
        val u=ByteBuffer.wrap(byteArrayOf(11,0,12))
        val v=ByteBuffer.wrap(byteArrayOf(21,0,22))
        val got=CameraJpeg.nv21(4,2,y,6,1,u,4,2,v,4,2)
        assertArrayEquals(byteArrayOf(1,2,3,4,5,6,7,8,21,11,22,12),got)
    }
}
