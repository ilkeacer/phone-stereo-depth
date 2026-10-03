package org.research.phonestereo

/** ARCore depth pixels use scaled GPU texture intrinsics, as in Google's raw-depth sample. */
data class DepthProjection(val fx:Float,val fy:Float,val cx:Float,val cy:Float) {
    fun cameraPoint(x:Int,y:Int,depthMm:Int):FloatArray {
        val z=depthMm/1000f
        return floatArrayOf(z*(x-cx)/fx,z*(cy-y)/fy,-z)
    }

    companion object {
        fun fromTexture(focal:FloatArray,principal:FloatArray,dimensions:IntArray,
                        depthWidth:Int,depthHeight:Int):DepthProjection {
            require(dimensions.size==2 && focal.size==2 && principal.size==2)
            require(dimensions[0]>0 && dimensions[1]>0 && focal.all { it>0f })
            return DepthProjection(focal[0]*depthWidth/dimensions[0],
                focal[1]*depthHeight/dimensions[1],
                principal[0]*depthWidth/dimensions[0],
                principal[1]*depthHeight/dimensions[1])
        }
    }
}
