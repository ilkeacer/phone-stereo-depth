package org.research.phonestereo

import java.nio.ByteBuffer

object Yuv {
    fun plane(buffer: ByteBuffer, width: Int, height: Int, row: Int, pixel: Int): ByteArray {
        val out = ByteArray(width * height)
        val base = buffer.position()
        if(pixel==1) {
            val view=buffer.duplicate()
            for(y in 0 until height) {
                view.position(base+y*row)
                view.get(out,y*width,width)
            }
            return out
        }
        for (y in 0 until height) for (x in 0 until width)
            out[y * width + x] = buffer.get(base + y * row + x * pixel)
        return out
    }
    fun nv21(y: ByteArray, u: ByteArray, v: ByteArray): ByteArray {
        require(u.size == v.size && y.size == 4 * u.size)
        val out = ByteArray(y.size + 2 * u.size)
        y.copyInto(out)
        for (i in u.indices) { out[y.size + 2*i] = v[i]; out[y.size + 2*i+1] = u[i] }
        return out
    }
}
