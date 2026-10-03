package org.research.phonestereo

import android.Manifest
import android.app.Activity
import android.content.pm.PackageManager
import android.opengl.GLES11Ext
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.util.Base64
import android.view.WindowManager
import android.widget.LinearLayout
import android.widget.Button
import android.widget.TextView
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Coordinates2d
import com.google.ar.core.ImageMetadata
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.NotYetAvailableException
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.io.BufferedWriter
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

/** Bounded ARCore probe. Detailed mode privately records paired RGB/depth for diagnosis. */
class ArCoreProbeActivity : Activity(), GLSurfaceView.Renderer {
    private lateinit var status:TextView
    private lateinit var closeButton:Button
    private lateinit var surface:GLSurfaceView
    private lateinit var report:File
    @Volatile private var session:Session?=null
    private var pointServer:ArCorePointServer?=null
    @Volatile private var active=false
    @Volatile private var finished=false
    private var frames=0
    private var tracking=0
    private var paused=0
    private var depthFrames=0
    private var depthUnavailable=0
    private var depthEnabled=false
    private val mapPoints=ArrayList<FloatArray>()
    private var previousTimestamp=0L
    private var cameraTexture=0
    private var previewProgram=0
    private val quad=floatArrayOf(-1f,-1f,1f,-1f,-1f,1f,1f,1f)
    private val textureUv=FloatArray(8)
    private val quadBuffer:FloatBuffer=ByteBuffer.allocateDirect(8*4).order(ByteOrder.nativeOrder()).asFloatBuffer().apply { put(quad);position(0) }
    private val textureBuffer:FloatBuffer=ByteBuffer.allocateDirect(8*4).order(ByteOrder.nativeOrder()).asFloatBuffer()
    private val handler=Handler(Looper.getMainLooper())
    private var durationSeconds=12
    private lateinit var scanPlan:ScanPlan
    private var runningSinceNs=0L
    private var detailMode=false
    private var detailArchive:File?=null
    private var detailWriter:BufferedWriter?=null
    private var detailSequence=0L
    private var lastDetailFrameNs=0L
    private val freshDepth=FreshDepth()
    private var repeatedDepth=0
    private var acceptedDetailPoints=0L
    private var rgbFrames=0
    private var rgbUnavailable=0
    private var rgbTimestampMismatches=0
    private var lastExposureReportNs=0L
    private var observedIso:Int?=null
    private var observedExposureNs:Long?=null
    private var observedAeMode:Int?=null
    private var exposureSamples=0
    private var exposureUnavailable=0
    private var userRequestedInstall=true
    private var awaitingInstall=false

    @Synchronized private fun record(kind:String,vararg fields:Pair<String,Any?>) {
        val row=JSONObject().put("event",kind).put("elapsedNs",SystemClock.elapsedRealtimeNanos())
        fields.forEach { row.put(it.first,it.second ?: JSONObject.NULL) }
        report.appendText(row.toString()+"\n")
    }

    override fun onCreate(state:Bundle?) {
        super.onCreate(state)
        scanPlan=ScanPlan(intent.getIntExtra("durationSeconds",12))
        durationSeconds=scanPlan.durationSeconds
        detailMode=intent.getBooleanExtra("rawDepth",false)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        report=File(getExternalFilesDir(null),"arcore-probe-${System.currentTimeMillis()}.jsonl")
        status=TextView(this).apply {
            textSize=20f
            setPadding(24,32,24,20)
            text=if(detailMode)
                "ARCore uyumluluğu kontrol ediliyor…\nAyrıntılı mod RGB kareleri ve derinliği yalnız yerel tanı kaydına ekler."
            else "ARCore uyumluluğu kontrol ediliyor…\nFotoğraf/video kaydedilmez. Derinlik ve konum verisi yerel saklanır."
        }
        val layout=LinearLayout(this).apply { orientation=LinearLayout.VERTICAL }
        layout.addView(status)
        closeButton=Button(this).apply {
            text="İptal"
            setOnClickListener { if(finished) finish() else end(ScanPlan.manualEndReason(active)) }
        }
        layout.addView(closeButton)
        setContentView(layout)
        handler.postDelayed({ if(!isFinishing && !active && !awaitingInstall) end("availability_timeout") },15_000)
        if(checkSelfPermission(Manifest.permission.CAMERA)!=PackageManager.PERMISSION_GRANTED)
            requestPermissions(arrayOf(Manifest.permission.CAMERA),11)
        else checkAvailability()
    }

    override fun onRequestPermissionsResult(code:Int,p:Array<out String>,r:IntArray) {
        super.onRequestPermissionsResult(code,p,r)
        if(finished || isFinishing || isDestroyed) return
        if(code==11 && r.firstOrNull()==PackageManager.PERMISSION_GRANTED) checkAvailability()
        else end("camera_permission_missing")
    }

    private fun checkAvailability() {
        if(finished || isFinishing || isDestroyed) return
        try {
            ArCoreApk.getInstance().checkAvailabilityAsync(this) { availability ->
                runOnUiThread {
                    if(finished || isFinishing || isDestroyed) return@runOnUiThread
                    record("availability","value" to availability.name,"supported" to availability.isSupported)
                    if(!availability.isSupported) { end("unsupported_or_unavailable");return@runOnUiThread }
                    startSession()
                }
            }
        } catch(e:Exception) { end("availability_error",e) }
    }

    private fun startSession() {
        if(finished || active || isFinishing || isDestroyed) return
        try {
            val install=ArCoreApk.getInstance().requestInstall(this,userRequestedInstall)
            record("installation","value" to install.name)
            if(install==ArCoreApk.InstallStatus.INSTALL_REQUESTED) {
                userRequestedInstall=false
                awaitingInstall=true
                status.text="Google Play Services for AR kurulumu/güncellemesi bekleniyor. Tamamlanınca bu ekrana dön."
                return
            }
            val current=Session(this)
            session=current
            depthEnabled=current.isDepthModeSupported(Config.DepthMode.AUTOMATIC)
            val config=current.config
            if(depthEnabled) config.depthMode=Config.DepthMode.AUTOMATIC
            current.configure(config)
            record("session_created","depthSupported" to depthEnabled,"depthMode" to config.depthMode.name,
                "projection" to "scaled_texture_intrinsics","acceptedDepthMm" to JSONArray(listOf(500,5000)),
                "depthSource" to if(detailMode) "raw_confidence_with_smoothed_reference" else "smoothed",
                "confidenceMinimum" to if(detailMode) 128 else null)
            if(detailMode) {
                if(!depthEnabled) { end("depth_not_supported");return }
                detailArchive=File(getExternalFilesDir(null),"arcore-detail-${System.currentTimeMillis()}.jsonl")
                detailWriter=detailArchive!!.bufferedWriter()
            }
            try { pointServer=ArCorePointServer();record("point_stream_ready","port" to 8766) }
            catch(e:Exception) { record("point_stream_unavailable","error" to e.toString()) }
            surface=GLSurfaceView(this).apply { setEGLContextClientVersion(2);setRenderer(this@ArCoreProbeActivity) }
            (status.parent as LinearLayout).addView(surface,LinearLayout.LayoutParams(-1,0,1f))
            current.resume()
            active=true
            surface.onResume()
            runningSinceNs=SystemClock.elapsedRealtimeNanos()
            closeButton.text="Durdur ve kaydet"
            status.text=if(detailMode)
                "Ayrıntılı oda taraması ($durationSeconds sn).\nİlk 5 sn aydınlık bir eşya kenarını sabit göster. Sonra zemini, eşya yanlarını ve duvarları yavaşça tara. Durdur ve kaydet ile erken bitirebilirsin."
            else if(durationSeconds<=15)
                "ARCore poz ve derinlik ölçülüyor (12 sn).\nAydınlık zemini kadrajda tutup 20–30 cm yavaşça hareket et."
            else "ARCore canlı oda taraması ($durationSeconds sn).\nÖnce 5 sn sabit; sonra yavaşça oda yüzeylerini tara."
            handler.postDelayed({ if(!isFinishing) end("finished") },durationSeconds*1000L)
        } catch(e:Exception) { end("session_error",e) }
    }

    override fun onSurfaceCreated(gl:GL10?,config:EGLConfig?) {
        try {
            val textures=IntArray(1)
            GLES20.glGenTextures(1,textures,0)
            GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,textures[0])
            GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,GLES20.GL_TEXTURE_MIN_FILTER,GLES20.GL_LINEAR)
            GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,GLES20.GL_TEXTURE_MAG_FILTER,GLES20.GL_LINEAR)
            cameraTexture=textures[0]
            val vertex=compileShader(GLES20.GL_VERTEX_SHADER,
                "attribute vec2 aPosition; attribute vec2 aTexCoord; varying vec2 vTexCoord; void main(){ gl_Position=vec4(aPosition,0.0,1.0); vTexCoord=aTexCoord; }")
            val fragment=compileShader(GLES20.GL_FRAGMENT_SHADER,
                "#extension GL_OES_EGL_image_external : require\nprecision mediump float; varying vec2 vTexCoord; uniform samplerExternalOES uTexture; void main(){ gl_FragColor=texture2D(uTexture,vTexCoord); }")
            previewProgram=GLES20.glCreateProgram()
            GLES20.glAttachShader(previewProgram,vertex)
            GLES20.glAttachShader(previewProgram,fragment)
            GLES20.glLinkProgram(previewProgram)
            GLES20.glDeleteShader(vertex);GLES20.glDeleteShader(fragment)
            val linked=IntArray(1);GLES20.glGetProgramiv(previewProgram,GLES20.GL_LINK_STATUS,linked,0)
            if(linked[0]!=GLES20.GL_TRUE) throw IllegalStateException("Preview shader link failed: ${GLES20.glGetProgramInfoLog(previewProgram)}")
            session?.setCameraTextureName(textures[0])
        } catch(e:Exception) { runOnUiThread { end("texture_error",e) } }
    }

    override fun onSurfaceChanged(gl:GL10?,width:Int,height:Int) {
        session?.setDisplayGeometry(windowManager.defaultDisplay.rotation,width,height)
    }

    override fun onDrawFrame(gl:GL10?) {
        GLES20.glClearColor(0f,0f,0f,1f)
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT)
        if(!active) return
        try {
            val frame=session?.update() ?: return
            drawPreview(frame)
            if(frame.timestamp==previousTimestamp) return
            previousTimestamp=frame.timestamp
            val camera=frame.camera
            frames++
            if(lastExposureReportNs==0L || frame.timestamp-lastExposureReportNs>=1_000_000_000L)
                observeExposure(frame)
            if(camera.trackingState==TrackingState.TRACKING) tracking++ else paused++
            val newWorld=ArrayList<FloatArray>()
            val longScan=durationSeconds>15
            var detailSent=false
            if(detailMode && depthEnabled && camera.trackingState==TrackingState.TRACKING &&
                frame.timestamp-lastDetailFrameNs>=200_000_000L) {
                try { detailSent=sampleDetail(frame) }
                catch(_:NotYetAvailableException) { depthUnavailable++ }
            }
            if(!detailMode && depthEnabled && frames%(if(longScan)15 else 10)==0 && camera.trackingState==TrackingState.TRACKING) {
                try {
                    frame.acquireDepthImage16Bits().use { image ->
                        val plane=image.planes[0]
                        val buffer=plane.buffer.order(ByteOrder.nativeOrder())
                        val samples=ArrayList<Int>()
                        var nonZero=0
                        var maximum=0
                        val intrinsics=camera.textureIntrinsics
                        val projection=DepthProjection.fromTexture(intrinsics.focalLength,
                            intrinsics.principalPoint,intrinsics.imageDimensions,image.width,image.height)
                        val pose=camera.pose
                        val stepX=maxOf(1,image.width/(if(longScan)64 else 32))
                        val stepY=maxOf(1,image.height/(if(longScan)45 else 24))
                        for(y in 0 until image.height step stepY)
                            for(x in 0 until image.width step stepX) {
                                val offset=y*plane.rowStride+x*plane.pixelStride
                                val value=buffer.getShort(offset).toInt() and 0xffff
                                if(value>0) nonZero++
                                maximum=maxOf(maximum,value)
                                if(value !in 500..5000) continue
                                samples.add(value)
                                val cameraPoint=projection.cameraPoint(x,y,value)
                                val world=pose.transformPoint(cameraPoint)
                                mapPoints.add(world)
                                newWorld.add(world)
                            }
                        samples.sort()
                        depthFrames++
                        record("depth","timestampNs" to image.timestamp,"width" to image.width,"height" to image.height,
                            "sampleCount" to samples.size,"nonZeroCount" to nonZero,"maxSampleMm" to maximum,
                            "medianSampleMm" to samples.getOrNull(samples.size/2),"mapPoints" to mapPoints.size)
                    }
                } catch(_:NotYetAvailableException) { depthUnavailable++ }
            }
            if(frames%5==0) {
                val pose=camera.pose
                record("frame","timestampNs" to frame.timestamp,"trackingState" to camera.trackingState.name,
                    "failureReason" to camera.trackingFailureReason.name,
                    "translationM" to JSONArray(pose.translation.map { it.toDouble() }),
                    "quaternion" to JSONArray(pose.rotationQuaternion.map { it.toDouble() }))
                if(detailMode) {
                    if(!detailSent) emitDetail(detailPacket(frame,"pose"))
                } else pointServer?.send(JSONObject().put("schemaVersion",1).put("timestampNs",frame.timestamp)
                    .put("trackingState",camera.trackingState.name)
                    .put("translationM",JSONArray(pose.translation.map { it.toDouble() }))
                    .put("quaternion",JSONArray(pose.rotationQuaternion.map { it.toDouble() }))
                    .put("points",JSONArray(newWorld.map { JSONArray(it.map { value -> value.toDouble() }) }))
                    .toString())
                val elapsed=((SystemClock.elapsedRealtimeNanos()-runningSinceNs)/1_000_000_000L).toInt().coerceAtLeast(0)
                val remaining=scanPlan.remaining(elapsed)
                val guidance=if(camera.trackingState==TrackingState.TRACKING) scanPlan.guidance(elapsed)
                    else "Takip bekliyor: ani hareketi bırak; aydınlık, eşya kenarlı alana yavaşça dön. Kayıt sürüyor."
                val details=if(detailMode) "Ayrıntı: $acceptedDetailPoints örnek · Derinlik: $depthFrames · RGB: $rgbFrames"
                    else "Derinlik: $depthFrames kare · 3B nokta: ${mapPoints.size}"
                val exposure=if(observedIso!=null && observedExposureNs!=null)
                    "ISO $observedIso · poz ${((observedExposureNs!!+50_000L)/100_000L)/10.0} ms · AE ${if(observedAeMode==0) "kapalı" else if(observedAeMode==1) "açık" else "?"}"
                    else "ISO/poz: kamera verisi bekleniyor"
                val clock="${elapsed/60}:${(elapsed%60).toString().padStart(2,'0')}"
                val left="${remaining/60}:${(remaining%60).toString().padStart(2,'0')}"
                val trackingState=camera.trackingState.name
                runOnUiThread { if(active && !finished) status.text="Geçen $clock · kalan $left\n$guidance\nTakip: $trackingState · $tracking/$frames · $details\n$exposure" }
            }
        } catch(e:Exception) { runOnUiThread { end("update_error",e) } }
    }

    private fun observeExposure(frame:Frame) {
        lastExposureReportNs=frame.timestamp
        // Read ARCore's capture result only. Changing Camera2 requests during an
        // ARCore session could alter the exposure time and motion tracking.
        val metadata=runCatching { frame.imageMetadata }.getOrNull()
        if(metadata==null) { exposureUnavailable++;return }
        val iso=runCatching { metadata.getInt(ImageMetadata.SENSOR_SENSITIVITY) }.getOrNull()
        val exposureNs=runCatching { metadata.getLong(ImageMetadata.SENSOR_EXPOSURE_TIME) }.getOrNull()
        val aeMode=runCatching { metadata.getByte(ImageMetadata.CONTROL_AE_MODE).toInt() and 0xff }.getOrNull()
        if(iso==null && exposureNs==null && aeMode==null) { exposureUnavailable++;return }
        observedIso=iso
        observedExposureNs=exposureNs
        observedAeMode=aeMode
        exposureSamples++
        record("exposure","cameraTimestampNs" to frame.timestamp,"iso" to iso,
            "exposureTimeNs" to exposureNs,"aeMode" to aeMode)
    }

    private fun detailPacket(frame:Frame,kind:String):JSONObject {
        val camera=frame.camera
        return JSONObject().put("schemaVersion",2).put("type",kind).put("timestampNs",frame.timestamp)
            .put("trackingState",camera.trackingState.name)
            .put("translationM",JSONArray(camera.pose.translation.map { it.toDouble() }))
            .put("quaternion",JSONArray(camera.pose.rotationQuaternion.map { it.toDouble() }))
    }

    private fun emitDetail(packet:JSONObject) {
        packet.put("sequence",++detailSequence)
        val row=packet.toString()
        detailWriter?.apply { write(row);newLine();flush() }
        pointServer?.send(row)
    }

    private fun packedImage(image:android.media.Image,bytes:Int):ByteArray {
        val plane=image.planes[0]
        return DepthBuffers.packed(plane.buffer,image.width,image.height,plane.rowStride,plane.pixelStride,bytes)
    }

    private fun attachRgb(frame:Frame,packet:JSONObject) {
        try {
            frame.acquireCameraImage().use { image ->
                // Depth pixels use texture coordinates. Save the transform to CPU-image
                // pixels so the JPEG and this exact depth frame can be aligned offline.
                val corners=floatArrayOf(0f,0f,1f,0f,0f,1f,1f,1f)
                val mapped=FloatArray(corners.size)
                frame.transformCoordinates2d(Coordinates2d.TEXTURE_NORMALIZED,corners,
                    Coordinates2d.IMAGE_PIXELS,mapped)
                val jpeg=CameraJpeg.encode(image)
                packet.put("rgbTimestampNs",image.timestamp)
                    .put("rgbWidth",image.width).put("rgbHeight",image.height)
                    .put("rgbEncoding","jpeg")
                    .put("textureToRgbCornersPx",JSONArray(mapped.map { it.toDouble() }))
                    .put("rgbJpegBase64",Base64.encodeToString(jpeg,Base64.NO_WRAP))
                if(image.timestamp!=frame.timestamp) rgbTimestampMismatches++
                rgbFrames++
            }
        } catch(_:NotYetAvailableException) {
            rgbUnavailable++
        } catch(e:Exception) {
            rgbUnavailable++
            record("rgb_capture_error","error" to "${e.javaClass.simpleName}: ${e.message}")
        }
    }

    private data class RawPlanes(val width:Int,val height:Int,val timestamp:Long,
                                 val confidenceTimestamp:Long,val depth:ByteArray,val confidence:ByteArray)

    private fun sampleDetail(frame:Frame):Boolean {
        val capture=frame.acquireRawDepthImage16Bits().use { raw ->
            frame.acquireRawDepthConfidenceImage().use { confidence ->
                require(raw.width==confidence.width && raw.height==confidence.height) { "Depth/confidence size mismatch" }
                // ARCore reprojects old raw data into the current camera view. A changed
                // depth timestamp is independent evidence; equality to camera time is not required.
                if(!freshDepth.accept(raw.timestamp)) { repeatedDepth++;return false }
                RawPlanes(raw.width,raw.height,raw.timestamp,confidence.timestamp,
                    packedImage(raw,2),packedImage(confidence,1))
            }
        }
        // Release raw/confidence image handles before acquiring the reference.
        // Session.update has not run again, so all buffers describe this camera view.
        val intrinsics=frame.camera.textureIntrinsics
        val k=DepthProjection.fromTexture(intrinsics.focalLength,intrinsics.principalPoint,
            intrinsics.imageDimensions,capture.width,capture.height)
        val packet=detailPacket(frame,"depth")
            .put("width",capture.width).put("height",capture.height)
            .put("depthTimestampNs",capture.timestamp).put("confidenceTimestampNs",capture.confidenceTimestamp)
            .put("intrinsics",JSONArray(listOf(k.fx,k.fy,k.cx,k.cy)))
            .put("rawDepthU16LE",Base64.encodeToString(capture.depth,Base64.NO_WRAP))
            .put("confidenceU8",Base64.encodeToString(capture.confidence,Base64.NO_WRAP))
        var smoothTimestamp:Long?=null
        try {
            frame.acquireDepthImage16Bits().use { smooth ->
                require(smooth.width==capture.width && smooth.height==capture.height) { "Raw/full depth size mismatch" }
                smoothTimestamp=smooth.timestamp
                packet.put("smoothTimestampNs",smooth.timestamp)
                    .put("smoothDepthU16LE",Base64.encodeToString(packedImage(smooth,2),Base64.NO_WRAP))
            }
        } catch(_:NotYetAvailableException) {
            packet.put("smoothDepthU16LE",JSONObject.NULL)
        }
        val accepted=DepthBuffers.accepted(capture.depth,capture.confidence)
        attachRgb(frame,packet)
        acceptedDetailPoints+=accepted
        depthFrames++
        lastDetailFrameNs=frame.timestamp
        record("raw_depth","timestampNs" to capture.timestamp,"cameraTimestampNs" to frame.timestamp,
            "confidenceTimestampNs" to capture.confidenceTimestamp,"smoothTimestampNs" to smoothTimestamp,
            "width" to capture.width,"height" to capture.height,"acceptedSamples" to accepted)
        emitDetail(packet)
        return true
    }

    private fun compileShader(kind:Int,source:String):Int {
        val shader=GLES20.glCreateShader(kind)
        GLES20.glShaderSource(shader,source)
        GLES20.glCompileShader(shader)
        val compiled=IntArray(1);GLES20.glGetShaderiv(shader,GLES20.GL_COMPILE_STATUS,compiled,0)
        if(compiled[0]!=GLES20.GL_TRUE) throw IllegalStateException("Preview shader compile failed: ${GLES20.glGetShaderInfoLog(shader)}")
        return shader
    }

    private fun drawPreview(frame:com.google.ar.core.Frame) {
        if(previewProgram==0 || cameraTexture==0) return
        frame.transformCoordinates2d(Coordinates2d.OPENGL_NORMALIZED_DEVICE_COORDINATES,quad,
            Coordinates2d.TEXTURE_NORMALIZED,textureUv)
        textureBuffer.position(0);textureBuffer.put(textureUv);textureBuffer.position(0)
        GLES20.glUseProgram(previewProgram)
        val position=GLES20.glGetAttribLocation(previewProgram,"aPosition")
        val texCoord=GLES20.glGetAttribLocation(previewProgram,"aTexCoord")
        GLES20.glEnableVertexAttribArray(position);GLES20.glEnableVertexAttribArray(texCoord)
        quadBuffer.position(0);GLES20.glVertexAttribPointer(position,2,GLES20.GL_FLOAT,false,0,quadBuffer)
        GLES20.glVertexAttribPointer(texCoord,2,GLES20.GL_FLOAT,false,0,textureBuffer)
        GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES,cameraTexture)
        GLES20.glUniform1i(GLES20.glGetUniformLocation(previewProgram,"uTexture"),0)
        GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP,0,4)
        GLES20.glDisableVertexAttribArray(position);GLES20.glDisableVertexAttribArray(texCoord)
    }

    private fun end(reason:String,error:Exception?=null) {
        if(isFinishing || finished) return
        finished=true
        active=false
        handler.removeCallbacksAndMessages(null)
        if(::surface.isInitialized) surface.onPause()
        if(detailMode && detailWriter!=null) {
            try {
                emitDetail(JSONObject().put("schemaVersion",2).put("type","end").put("reason",reason))
                detailWriter?.close()
                detailWriter=null
                record("depth_archive","path" to detailArchive?.absolutePath,"packets" to detailSequence,
                    "acceptedSamples" to acceptedDetailPoints,"repeatedDepthSkipped" to repeatedDepth,
                    "rgbFrames" to rgbFrames,"rgbUnavailable" to rgbUnavailable,
                    "rgbTimestampMismatches" to rgbTimestampMismatches)
            } catch(e:Exception) { record("depth_archive_error","error" to e.toString()) }
        }
        pointServer?.close();pointServer=null
        try { session?.pause() } catch(_:Exception) {}
        session?.close();session=null
        record("result","reason" to reason,"frames" to frames,"tracking" to tracking,
            "paused" to paused,"depthFrames" to depthFrames,"depthUnavailable" to depthUnavailable,
            "rgbFrames" to rgbFrames,"rgbUnavailable" to rgbUnavailable,
            "rgbTimestampMismatches" to rgbTimestampMismatches,
            "exposureSamples" to exposureSamples,"exposureUnavailable" to exposureUnavailable,
            "error" to error?.let { "${it.javaClass.simpleName}: ${it.message}" })
        val ply=if(mapPoints.isNotEmpty()) File(getExternalFilesDir(null),"arcore-map-${System.currentTimeMillis()}.ply") else null
        try {
            ply?.bufferedWriter()?.use { out ->
                out.write("ply\nformat ascii 1.0\ncomment ARCore diagnostic; local frame; metric accuracy unverified\n")
                out.write("element vertex ${mapPoints.size}\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
                for(point in mapPoints) out.write("${point[0]} ${point[1]} ${point[2]}\n")
            }
            record("map_export","points" to mapPoints.size,"path" to ply?.absolutePath)
        } catch(e:Exception) { record("map_export_error","error" to e.toString()) }
        val endText=when(reason) {
            "saved_by_user" -> "Durdur ve kaydet ile bitirildi"
            "finished" -> "Seçilen süre tamamlandı"
            "interrupted" -> "Uygulama arka plana geçti; kayıt kesildi"
            else -> "Kayıt sona erdi: $reason"
        }
        status.text=if(detailMode)
            "$endText\nPoz takibi: $tracking / $frames kare\nHam derinlik: $depthFrames kare\nRGB: $rgbFrames kare\nKabul edilen örnek: $acceptedDetailPoints\nRGB, derinlik ve poz yerel tanı kaydına alındı; harita bilgisayarda."
        else "$endText\nPoz takibi: $tracking / $frames kare\nDerinlik: $depthFrames kare\n3B nokta: ${mapPoints.size}\nGörüntü kaydedilmedi."
        closeButton.text="Kapat"
    }

    override fun onPause() {
        if(active && !finished) end("interrupted")
        super.onPause()
    }

    override fun onResume() {
        super.onResume()
        if(awaitingInstall && !finished) {
            awaitingInstall=false
            startSession()
        }
    }

    override fun onDestroy() {
        handler.removeCallbacksAndMessages(null)
        try { session?.close() } catch(_:Exception) {}
        super.onDestroy()
    }
}
