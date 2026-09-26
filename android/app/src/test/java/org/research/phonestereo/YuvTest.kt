package org.research.phonestereo
import org.junit.Test
import org.junit.Assert.*
import java.nio.ByteBuffer
class YuvTest {
 @Test fun paddedPlaneWithOffsetAndPixelStride() {
  val b=ByteBuffer.wrap(byteArrayOf(99,1,88,2,88,77,77,3,88,4))
  b.position(1)
  assertArrayEquals(byteArrayOf(1,2,3,4),Yuv.plane(b,2,2,6,2))
 }
 @Test fun nv21UsesVUOrder() {
  assertArrayEquals(byteArrayOf(1,2,3,4,9,8),Yuv.nv21(byteArrayOf(1,2,3,4),byteArrayOf(8),byteArrayOf(9)))
 }
 @Test fun tightLuma() { assertArrayEquals(byteArrayOf(1,2,3,4),Yuv.plane(ByteBuffer.wrap(byteArrayOf(1,2,3,4)),2,2,2,1)) }
 @Test fun bulkLumaRespectsOffsetPaddingAndOriginalPosition() {
  val b=ByteBuffer.wrap(byteArrayOf(99,1,2,88,88,3,4));b.position(1)
  assertArrayEquals(byteArrayOf(1,2,3,4),Yuv.plane(b,2,2,4,1))
  assertEquals(1,b.position())
 }
 @Test(expected=IllegalArgumentException::class) fun rejectsMismatchedPlanes() { Yuv.nv21(byteArrayOf(1,2),byteArrayOf(1),byteArrayOf(2)) }
}
