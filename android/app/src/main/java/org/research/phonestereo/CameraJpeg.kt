package org.research.phonestereo

import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.media.Image
import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer

/** Copy an ARCore CPU camera image into a private, compact JPEG for depth-edge diagnosis. */
object CameraJpeg {
    private fun sample(buffer:ByteBuffer,rowStride:Int,pixelStride:Int,x:Int,y:Int):Byte {
        val offset=buffer.position()+y*rowStride+x*pixelStride
        require(offset<buffer.limit()) { "Camera plane shorter than its dimensions" }
        return buffer.get(offset)
    }

    fun nv21(width:Int,height:Int,y:ByteBuffer,yRow:Int,yPixel:Int,
             u:ByteBuffer,uRow:Int,uPixel:Int,v:ByteBuffer,vRow:Int,vPixel:Int):ByteArray {
        require(width>0 && height>0 && width%2==0 && height%2==0 && width.toLong()*height<=1920L*1080)
        require(yRow>0 && yPixel>0 && uRow>0 && uPixel>0 && vRow>0 && vPixel>0)
        val bytes=ByteArray(width*height*3/2)
        for(row in 0 until height) for(col in 0 until width)
            bytes[row*width+col]=sample(y,yRow,yPixel,col,row)
        var dest=width*height
        for(row in 0 until height/2) for(col in 0 until width/2) {
            bytes[dest++]=sample(v,vRow,vPixel,col,row)
            bytes[dest++]=sample(u,uRow,uPixel,col,row)
        }
        return bytes
    }

    fun encode(image:Image):ByteArray {
        require(image.format==ImageFormat.YUV_420_888 && image.planes.size==3)
        val p=image.planes
        val nv21=nv21(image.width,image.height,
            p[0].buffer,p[0].rowStride,p[0].pixelStride,
            p[1].buffer,p[1].rowStride,p[1].pixelStride,
            p[2].buffer,p[2].rowStride,p[2].pixelStride)
        val output=ByteArrayOutputStream()
        check(YuvImage(nv21,ImageFormat.NV21,image.width,image.height,null)
            .compressToJpeg(Rect(0,0,image.width,image.height),75,output))
        return output.toByteArray()
    }
}
