package org.research.phonestereo

import java.nio.ByteBuffer
import org.junit.Assert.*
import org.junit.Test

class DepthBuffersTest {
    @Test fun respectsBufferPositionPixelStrideAndPadding() {
        val data=ByteBuffer.wrap(byteArrayOf(99,10,11,88,20,21,88,77,30,31,88,40,41))
        data.position(1)
        assertArrayEquals(byteArrayOf(10,11,20,21,30,31,40,41),DepthBuffers.packed(data,2,2,7,3,2))
        assertEquals(1,data.position())
    }
    @Test fun confidenceIsUnsignedAndDepthIsLittleEndian() {
        val raw=byteArrayOf(0xe8.toByte(),3,0xd0.toByte(),7,0xe8.toByte(),3,0,0)
        assertEquals(2,DepthBuffers.accepted(raw,byteArrayOf(128.toByte(),255.toByte(),127,255.toByte())))
    }
    @Test fun repeatedReprojectedDepthIsNotNewEvidence() {
        val gate=FreshDepth()
        assertTrue(gate.accept(100)); assertFalse(gate.accept(100))
        assertFalse(gate.accept(90)); assertTrue(gate.accept(110))
    }
    @Test(expected=IllegalArgumentException::class) fun rejectsTruncatedPlane() {
        DepthBuffers.packed(ByteBuffer.allocate(7),2,2,4,2,2)
    }
}
