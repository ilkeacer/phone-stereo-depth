package org.research.phonestereo

import java.nio.ByteBuffer

/** Copy image planes without row padding. Depth bytes stay little-endian. */
object DepthBuffers {
    fun packed(buffer:ByteBuffer,width:Int,height:Int,rowStride:Int,pixelStride:Int,bytes:Int):ByteArray {
        require(width>0 && height>0 && width.toLong()*height<=640*480)
        require(bytes in 1..2 && pixelStride>=bytes && rowStride>=(width-1)*pixelStride+bytes)
        val start=buffer.position()
        require(start.toLong()+(height-1L)*rowStride+(width-1L)*pixelStride+bytes<=buffer.limit())
        val result=ByteArray(width*height*bytes)
        var dst=0
        for(y in 0 until height) for(x in 0 until width) {
            val src=start+y*rowStride+x*pixelStride
            for(i in 0 until bytes) result[dst++]=buffer.get(src+i)
        }
        return result
    }

    fun accepted(raw:ByteArray,confidence:ByteArray):Int {
        require(raw.size==confidence.size*2)
        var count=0
        for(i in confidence.indices) {
            val mm=(raw[i*2].toInt() and 255) or ((raw[i*2+1].toInt() and 255) shl 8)
            if(mm in 500..5000 && (confidence[i].toInt() and 255)>=128) count++
        }
        return count
    }
}

class FreshDepth {
    private var lastTimestamp=0L
    fun accept(timestamp:Long):Boolean {
        if(timestamp<=lastTimestamp) return false
        lastTimestamp=timestamp
        return true
    }
}
