package org.research.phonestereo

import android.Manifest
import android.app.Activity
import android.content.Intent
import android.os.*
import android.content.pm.PackageManager
import android.graphics.*
import android.hardware.camera2.*
import android.hardware.camera2.params.*
import android.media.ImageReader
import android.util.Log
import android.util.Range
import android.view.WindowManager
import android.widget.TextView
import android.widget.LinearLayout
import android.widget.ImageView
import org.json.*
import java.io.File
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean

class MainActivity : Activity() {
    private lateinit var manager: CameraManager
    private lateinit var display: TextView
    private lateinit var root: File
    private val previewViews=mutableMapOf<String,ImageView>()
    private val previewStatusViews=mutableMapOf<String,TextView>()
    private val stopped = AtomicBoolean(false)
    private var started = false
    private var livePairs:LivePairs?=null
    private var inertialCapture:InertialCapture?=null
    private val dumpsysIds = listOf("0","1","20","21","60","61","62","63","100","101","120")
    private fun obj(vararg values: Pair<String, Any?>): JSONObject = JSONObject().also { j -> values.forEach { j.put(it.first, it.second ?: JSONObject.NULL) } }
    private fun arr(values: Iterable<*>): JSONArray = JSONArray().also { a -> values.forEach { a.put(it) } }
    @Synchronized private fun event(dir: File, kind: String, id: String? = null, vararg fields: Pair<String, Any?>) {
        val j = obj("event" to kind, "cameraId" to id, "elapsedNs" to SystemClock.elapsedRealtimeNanos(), *fields)
        File(dir,"events.jsonl").appendText(j.toString()+"\n")
        Log.i("StereoProbe", j.toString())
        runOnUiThread {
            display.text=if(intent.getBooleanExtra("live",false))
                "STEREO HARİTA · ${dir.name}\n$kind ${id ?: ""}"
            else "Phone Stereo Probe\n${root.name}\n${dir.name}\n$kind ${id ?: ""}\n\nKamera deneyi çalışıyor.\nSonuçlar uygulama dosyalarına kaydediliyor."
        }
    }
    override fun onCreate(state: Bundle?) {
        super.onCreate(state)
        if(intent.getBooleanExtra("arcoreProbe",false)) {
            // Legacy desktop request opens the chooser; only a phone button starts ARCore.
            startActivity(Intent(this,HomeActivity::class.java))
            finish()
            return
        }
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        val live=intent.getBooleanExtra("live",false)
        display = TextView(this).apply { textSize=if(live) 14f else 20f; setPadding(24,24,24,8) }
        val layout=LinearLayout(this).apply { orientation=LinearLayout.VERTICAL }
        layout.addView(display,LinearLayout.LayoutParams(-1,0,if(live) 0.12f else 0.40f))
        if(live) layout.addView(TextView(this).apply {
            text="HARİTA KADRAJI: 20 TELEFOTO\nZeminle birlikte halı sınırı veya sabit eşya kenarı bu büyük görüntüde görünsün. Az ışık uyarısı bilgi amaçlıdır."
            textSize=16f; setPadding(24,4,24,8); setTextColor(Color.rgb(255,220,140))
        },LinearLayout.LayoutParams(-1,0,0.12f))
        val previews=LinearLayout(this).apply { orientation=if(live) LinearLayout.VERTICAL else LinearLayout.HORIZONTAL }
        for(id in listOf("20","21")) {
            val box=LinearLayout(this).apply { orientation=LinearLayout.VERTICAL }
            box.addView(TextView(this).apply {
                text=if(id=="20") "20 · TELEFOTO · AI derinlik + stereo takip" else "21 · GENİŞ AÇI · stereo takip"
                textSize=if(live) 17f else 15f
            })
            if(live) {
                val status=TextView(this).apply { text="Kamera ve ışık ölçümü bekleniyor"; textSize=15f }
                previewStatusViews[id]=status;box.addView(status)
            }
            val image=ImageView(this).apply { scaleType=ImageView.ScaleType.FIT_CENTER }
            previewViews[id]=image;box.addView(image,LinearLayout.LayoutParams(-1,0,1f))
            previews.addView(box,if(live) LinearLayout.LayoutParams(-1,0,if(id=="20") 0.68f else 0.32f)
                else LinearLayout.LayoutParams(0,-1,1f))
        }
        layout.addView(previews,LinearLayout.LayoutParams(-1,0,if(live) 0.76f else 0.60f));setContentView(layout)
        manager = getSystemService(CameraManager::class.java)
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED)
            requestPermissions(arrayOf(Manifest.permission.CAMERA), 10)
    }
    override fun onWindowFocusChanged(hasFocus:Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if(hasFocus) scheduleStart()
    }
    private fun scheduleStart() {
        // MIUI can still classify the UID as background during onCreate.
        // Wait for the actual foreground window before opening Camera2.
        window.decorView.postDelayed({
            if(!isFinishing && !isDestroyed && hasWindowFocus() &&
                checkSelfPermission(Manifest.permission.CAMERA)==PackageManager.PERMISSION_GRANTED) start()
        },300)
    }
    override fun onRequestPermissionsResult(code:Int, p:Array<out String>, r:IntArray) {
        super.onRequestPermissionsResult(code,p,r)
        if (r.firstOrNull() == PackageManager.PERMISSION_GRANTED) scheduleStart() else display.text="Kamera izni gerekli."
    }
    override fun onDestroy() { stopped.set(true); inertialCapture?.close(); livePairs?.close(); super.onDestroy() }
    private fun start() {
        if (started) return
        started=true
        root=File(getExternalFilesDir(null), "run_${System.currentTimeMillis()}").apply { mkdirs() }
        if(intent.getBooleanExtra("live",false)) {
            livePairs=LivePairs(root.name)
            inertialCapture=InertialCapture(this,livePairs!!).also { it.start() }
        }
        Thread({
            try {
                discover()
                val custom = intent.getStringExtra("ids")
                if (custom != null) {
                    trial(custom.split(","), intent.getBooleanExtra("light",false), intent.getIntExtra("seconds",65), "manual")
                } else {
                    val back=(manager.cameraIdList.toList()+dumpsysIds).distinct().filter { id -> try { manager.getCameraCharacteristics(id).get(CameraCharacteristics.LENS_FACING)==CameraCharacteristics.LENS_FACING_BACK } catch(e:Exception) { event(root,"direct_characteristics_error",id,"exception" to e.toString()); false } }
                    val good=mutableSetOf<String>()
                    for (id in back) {
                        if (stopped.get()) break
                        if (trial(listOf(id),false,6,"single")) good.add(id)
                    }
                    val pairs=mutableListOf(listOf("0","20"),listOf("0","21"),listOf("20","21"))
                    if ("100" in good) { pairs.add(listOf("100","20")); pairs.add(listOf("100","21")) }
                    for (pair in pairs.filter { p -> p.all { it in good } }) {
                        var success=false
                        for (light in listOf(false,true)) {
                            for (order in listOf(pair,pair.reversed())) {
                                if (stopped.get()) break
                                if (trial(order,light,8,"pair")) {
                                    trial(order,light,65,"sustain")
                                    success=true
                                    break
                                }
                            }
                            if (success) break
                        }
                    }
                }
                if(stopped.get()) { event(root,"suite_stopped"); File(root,"ABORTED").writeText("Stopped; inspect events\n") }
                else { event(root,"suite_complete"); File(root,"DONE").writeText("complete\n") }
            } catch (e:Exception) { event(root,"suite_exception",null,"exception" to e.toString(),"stack" to Log.getStackTraceString(e)) }
        },"experiment").start()
    }
    private fun discover() {
        val ids=manager.cameraIdList.toList()
        val cameras=JSONArray()
        for (id in (ids+dumpsysIds).distinct()) {
            try {
                val c=manager.getCameraCharacteristics(id)
                val streams=JSONObject()
                val map=c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)!!
                for (f in map.outputFormats) streams.put(f.toString(),arr(map.getOutputSizes(f).map { obj("width" to it.width,"height" to it.height,"minFrameDurationNs" to map.getOutputMinFrameDuration(f,it)) }))
                cameras.put(obj("id" to id,"enumerated" to (id in ids),"facing" to c.get(CameraCharacteristics.LENS_FACING),
                    "focalLengthsMm" to arr(c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)!!.toList()),
                    "activeArray" to c.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE).toString(),
                    "pixelArray" to c.get(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE).toString(),
                    "sensorOrientation" to c.get(CameraCharacteristics.SENSOR_ORIENTATION),
                    "timestampSource" to c.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE),
                    "capabilities" to arr(c.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES)!!.toList()),
                    "physicalCameraIds" to arr(c.physicalCameraIds),
                    "sensorSyncType" to c.get(CameraCharacteristics.LOGICAL_MULTI_CAMERA_SENSOR_SYNC_TYPE),
                    "fpsRanges" to arr(c.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)!!.map { listOf(it.lower,it.upper) }.map { arr(it) }),
                    "streams" to streams))
            } catch (e:Exception) { cameras.put(obj("id" to id,"exception" to e.toString())) }
        }
        val concurrent=try { arr(manager.concurrentCameraIds.map { arr(it) }) } catch(e:Exception) { arr(listOf(e.toString())) }
        File(root,"discovery.json").writeText(obj("cameraIdList" to arr(ids),"dumpsysIds" to arr(dumpsysIds),
            "missingFromApp" to arr(dumpsysIds-ids.toSet()),"concurrentCameraIds" to concurrent,
            "device" to Build.MODEL,"sdk" to Build.VERSION.SDK_INT,"cameras" to cameras).toString(2))
        event(root,"discovery_complete",null,"ids" to arr(ids),"concurrentCameraIds" to concurrent)
    }
    inner class Feed(val id:String,val dir:File,val light:Boolean) {
        val c=manager.getCameraCharacteristics(id)
        val sizes=c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)!!.getOutputSizes(ImageFormat.YUV_420_888).toList()
        val requestedWidth=intent.getIntExtra("width",0)
        val requestedHeight=intent.getIntExtra("height",0)
        val size= if(requestedWidth>0) sizes.firstOrNull { it.width==requestedWidth && it.height==requestedHeight } ?: throw IllegalArgumentException("Unsupported requested YUV size for $id") else if (light) sizes.minBy { it.width*it.height } else sizes.firstOrNull { it.width==640 && it.height==480 } ?: sizes.minBy { it.width*it.height }
        val ranges=c.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)!!.toList()
        val fps=if(intent.hasExtra("fpsMin$id") || intent.hasExtra("fpsMax$id")) {
            ranges.firstOrNull { it.lower==intent.getIntExtra("fpsMin$id",-1) && it.upper==intent.getIntExtra("fpsMax$id",-1) }
                ?: throw IllegalArgumentException("Unsupported requested FPS range for $id")
        } else ranges.firstOrNull { it.lower==15 && it.upper==15 } ?: ranges.minWith(compareBy<Range<Int>> { it.upper }.thenBy { it.lower })
        val thread=HandlerThread("camera-$id").apply { start() }
        val handler=Handler(thread.looper)
        val imageThread=if(livePairs!=null) HandlerThread("image-$id").apply { start() } else null
        val imageHandler=imageThread?.let { Handler(it.looper) } ?: handler
        val opened=CountDownLatch(1); val configured=CountDownLatch(1); val first=CountDownLatch(1); val closed=CountDownLatch(1)
        val closing=AtomicBoolean(false)
        @Volatile var failed=false
        @Volatile var actualIso:Int?=null
        @Volatile var device:CameraDevice?=null
        @Volatile var session:CameraCaptureSession?=null
        @Volatile var count=0
        @Volatile var firstNs=0L
        @Volatile var lastNs=0L
        @Volatile var retired=false
        @Volatile var maxGapNs=0L
        var reader:ImageReader?=null
        val images=File(dir,"camera_${id}_images.jsonl").bufferedWriter()
        val metadata=File(dir,"camera_${id}_metadata.jsonl").bufferedWriter()
        fun error(stage:String,e:Exception) {
            failed=true
            event(dir,stage,id,"exception" to e.toString(),"reason" to (e as? CameraAccessException)?.reason)
        }
        fun open() {
            event(dir,"open_requested",id,"width" to size.width,"height" to size.height,"fpsRange" to fps.toString())
            try {
                manager.openCamera(id,object:CameraDevice.StateCallback() {
                    override fun onOpened(d:CameraDevice) {
                        device=d
                        if(closing.get()) { d.close(); opened.countDown(); return }
                        event(dir,"opened",id); opened.countDown()
                    }
                    override fun onDisconnected(d:CameraDevice) { failed=true; device=d; event(dir,"disconnected",id); opened.countDown(); d.close() }
                    override fun onError(d:CameraDevice,code:Int) { failed=true; device=d; event(dir,"device_error",id,"code" to code); opened.countDown(); d.close() }
                    override fun onClosed(d:CameraDevice) { event(dir,"closed",id); closed.countDown(); if(retired) thread.quitSafely() }
                },handler)
            } catch(e:Exception) { error("open_exception",e); opened.countDown() }
        }
        fun prepare():SessionConfiguration {
            reader=ImageReader.newInstance(size.width,size.height,ImageFormat.YUV_420_888,4)
            reader!!.setOnImageAvailableListener({ r ->
                if(closing.get()) return@setOnImageAvailableListener
                val im=try {
                    // Live depth values need current exposures, not a growing
                    // backlog when JPEG conversion cannot sustain sensor FPS.
                    if(livePairs!=null && !intent.getBooleanExtra("saveAll",false)) r.acquireLatestImage()
                    else r.acquireNextImage()
                } catch(e:Exception) { error("acquire_exception",e); null }
                if(im!=null) {
                    try {
                        val ts=im.timestamp
                        count++
                        if(firstNs==0L) { firstNs=ts; event(dir,"first_frame",id,"sensorTimestampNs" to ts); first.countDown() }
                        if(lastNs!=0L) maxGapNs=maxOf(maxGapNs,ts-lastNs)
                        val precedingInterval=if(lastNs==0L) 0L else ts-lastNs
                        lastNs=ts
                        val record=obj("cameraId" to id,"sequence" to count,"imageTimestampNs" to ts,
                            "arrivalElapsedNs" to SystemClock.elapsedRealtimeNanos(),"timestampSource" to c.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE),"precedingIntervalNs" to precedingInterval,"maxGapNs" to maxGapNs,"width" to im.width,"height" to im.height,
                            "imageCrop" to im.cropRect.toString(),"rowStrides" to arr(im.planes.map { it.rowStride }),"pixelStrides" to arr(im.planes.map { it.pixelStride }))
                        // Samples at start (after AE settles) and every ~second, full timestamps for ALL frames.
                        if(livePairs!=null || intent.getBooleanExtra("saveAll",false) || count%15==0) {
                            val planes=im.planes.mapIndexed { index,p -> Yuv.plane(p.buffer,im.width/(if(index==0)1 else 2),im.height/(if(index==0)1 else 2),p.rowStride,p.pixelStride) }
                            val file="camera_${id}_${count}_${ts}.jpg"
                            val out=java.io.ByteArrayOutputStream()
                            YuvImage(Yuv.nv21(planes[0],planes[1],planes[2]),ImageFormat.NV21,im.width,im.height,null).compressToJpeg(Rect(0,0,im.width,im.height),90,out)
                            val bytes=out.toByteArray()
                            if(livePairs==null || intent.getBooleanExtra("saveAll",false)) {
                                File(dir,file).writeBytes(bytes);record.put("sampleFile",file)
                            }
                            livePairs?.frame(id,bytes,record)
                            if(count%15==0) {
                                val temp=File(root,"latest_${id}.tmp")
                                temp.writeBytes(bytes);temp.renameTo(File(root,"latest_${id}.jpg"))
                                val bitmap=BitmapFactory.decodeByteArray(bytes,0,bytes.size)
                                runOnUiThread {
                                    val rotation=(c.get(CameraCharacteristics.SENSOR_ORIENTATION) ?: 90)-windowManager.defaultDisplay.rotation*90
                                    val matrix=Matrix().apply { postRotate(rotation.toFloat()) }
                                    previewViews[id]?.setImageBitmap(Bitmap.createBitmap(bitmap,0,0,bitmap.width,bitmap.height,matrix,true))
                                }
                            }
                            val meanLuma=planes[0].sumOf { it.toInt() and 255 }.toDouble()/planes[0].size
                            record.put("meanLuma",meanLuma)
                            if(livePairs!=null && count%15==0) {
                                val dim=meanLuma<50.0 // Advisory threshold; tracking is decided by the mapper.
                                val isoText=actualIso?.toString() ?: "…"
                                runOnUiThread {
                                    previewStatusViews[id]?.apply {
                                        text="$count kare · ışık ${meanLuma.toInt()}/255 · ISO $isoText"+
                                            if(dim) " · AZ IŞIK" else " · ışık daha iyi"
                                        setTextColor(if(dim) Color.rgb(255,190,90) else Color.rgb(130,230,170))
                                    }
                                }
                            }
                        }
                        images.write(record.toString()); images.newLine()
                    } catch(e:Exception) { error("image_exception",e) } finally { im.close() }
                }
            },imageHandler)
            return SessionConfiguration(SessionConfiguration.SESSION_REGULAR,listOf(OutputConfiguration(reader!!.surface)),java.util.concurrent.Executor { handler.post(it) },object:CameraCaptureSession.StateCallback() {
                override fun onConfigured(s:CameraCaptureSession) {
                    session=s
                    if(closing.get()) { s.close(); configured.countDown(); return }
                    event(dir,"configured",id)
                    try {
                        val req=device!!.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW).apply {
                            addTarget(reader!!.surface)
                            set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE,fps)
                            set(CaptureRequest.CONTROL_MODE,CaptureRequest.CONTROL_MODE_AUTO)
                            set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
                            if(c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION)?.contains(0)==true) set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,0)
                            if(c.get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES)?.contains(CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE)==true) set(CaptureRequest.CONTROL_AF_MODE,CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE)
                            set(CaptureRequest.SCALER_CROP_REGION,c.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE))
                            if(intent.getBooleanExtra("fixed",false)) {
                                require(c.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES)!!.contains(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR)) { "Manual sensor settings unavailable" }
                                set(CaptureRequest.CONTROL_AF_MODE,CaptureRequest.CONTROL_AF_MODE_OFF)
                                val focus=intent.getFloatExtra("focus",2.0f).coerceIn(0f,c.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE) ?: 0f)
                                set(CaptureRequest.LENS_FOCUS_DISTANCE,focus)
                                set(CaptureRequest.CONTROL_AE_MODE,CaptureRequest.CONTROL_AE_MODE_OFF)
                                val exposure=c.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE)!!.clamp(intent.getLongExtra("exposureNs",20_000_000L))
                                val iso=c.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE)!!.clamp(intent.getIntExtra("iso",400))
                                set(CaptureRequest.SENSOR_EXPOSURE_TIME,exposure)
                                set(CaptureRequest.SENSOR_SENSITIVITY,iso)
                                // Explicit per-sensor experiments; defaults remain unchanged.
                                // CaptureResult and SENSOR_TIMESTAMP must verify the actual rate.
                                val duration=intent.getLongExtra("frameDurationNs$id",66_666_667L)
                                val minimum=maxOf(exposure,c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)!!
                                    .getOutputMinFrameDuration(ImageFormat.YUV_420_888,size))
                                val maximum=c.get(CameraCharacteristics.SENSOR_INFO_MAX_FRAME_DURATION)
                                require(duration>=minimum && (maximum==null || duration<=maximum)) { "Frame duration outside supported range" }
                                set(CaptureRequest.SENSOR_FRAME_DURATION,duration)
                                event(dir,"fixed_requested",id,"focusDiopters" to focus,"exposureTimeNs" to exposure,"iso" to iso,"frameDurationNs" to duration)
                            }
                        }.build()
                        s.setRepeatingRequest(req,object:CameraCaptureSession.CaptureCallback() {
                            override fun onCaptureCompleted(s:CameraCaptureSession,r:CaptureRequest,result:TotalCaptureResult) {
                                actualIso=result.get(CaptureResult.SENSOR_SENSITIVITY)
                                val record=obj("cameraId" to id,"sensorTimestampNs" to result.get(CaptureResult.SENSOR_TIMESTAMP),"frameNumber" to result.frameNumber,
                                    "exposureTimeNs" to result.get(CaptureResult.SENSOR_EXPOSURE_TIME),"iso" to result.get(CaptureResult.SENSOR_SENSITIVITY),
                                    "focusDiopters" to result.get(CaptureResult.LENS_FOCUS_DISTANCE),"focalLengthMm" to result.get(CaptureResult.LENS_FOCAL_LENGTH),
                                    "crop" to result.get(CaptureResult.SCALER_CROP_REGION).toString(),"frameDurationNs" to result.get(CaptureResult.SENSOR_FRAME_DURATION),
                                    "rollingShutterSkewNs" to result.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW),"aeState" to result.get(CaptureResult.CONTROL_AE_STATE),
                                    "videoStabilizationMode" to result.get(CaptureResult.CONTROL_VIDEO_STABILIZATION_MODE),"opticalStabilizationMode" to result.get(CaptureResult.LENS_OPTICAL_STABILIZATION_MODE),
                                    "activePhysicalId" to result.get(CaptureResult.LOGICAL_MULTI_CAMERA_ACTIVE_PHYSICAL_ID))
                                metadata.write(record.toString()); metadata.newLine()
                                livePairs?.metadata(id,record)
                            }
                            override fun onCaptureFailed(s:CameraCaptureSession,r:CaptureRequest,f:CaptureFailure) { event(dir,"capture_failed",id,"reason" to f.reason,"frameNumber" to f.frameNumber) }
                            override fun onCaptureBufferLost(s:CameraCaptureSession,r:CaptureRequest,target:android.view.Surface,frame:Long) { event(dir,"buffer_lost",id,"frameNumber" to frame) }
                        },handler)
                        event(dir,"repeating",id)
                    } catch(e:Exception) { error("repeating_exception",e) }
                    configured.countDown()
                }
                override fun onConfigureFailed(s:CameraCaptureSession) { session=s; failed=true; event(dir,"configure_failed",id); configured.countDown() }
            })
        }
        fun close() {
            closing.set(true)
            val barrier=CountDownLatch(1)
            handler.post {
                try { session?.stopRepeating() } catch(_:Exception) {}
                session?.close(); device?.close()
                barrier.countDown()
            }
            barrier.await(3,TimeUnit.SECONDS)
            if(device!=null && !closed.await(4,TimeUnit.SECONDS)) event(dir,"close_timeout",id)
            val drained=CountDownLatch(2)
            imageHandler.post { reader?.close(); images.close(); drained.countDown() }
            handler.post { metadata.close(); drained.countDown() }
            if(!drained.await(3,TimeUnit.SECONDS)) { stopped.set(true);event(dir,"drain_timeout",id) }
            imageThread?.quitSafely();imageThread?.join(3000)
            if(imageThread?.isAlive==true) { stopped.set(true);event(dir,"image_close_timeout",id) }
            // Keep callback looper alive for a late onOpened after timeout.
            handler.post {
                retired=true
                if(opened.count==0L && (device==null || closed.count==0L)) thread.quitSafely()
                else {
                    stopped.set(true); event(dir,"pending_close_suite_stopped",id)
                    handler.postDelayed({
                        event(dir,"unresolved_callback_process_cleanup",id)
                        File(root,"ABORTED").writeText("Camera callback did not resolve; probe process terminated to release Binder resources.\n")
                        android.os.Process.killProcess(android.os.Process.myPid())
                    },10_000L)
                }
            }
            thread.join(3000)
        }
    }
    private fun trial(ids:List<String>,light:Boolean,seconds:Int,label:String):Boolean {
        if(stopped.get()) return false
        val dir=File(root,"${label}_${ids.joinToString("-")}_${if(light)"min" else "vga"}_${System.currentTimeMillis()}").apply { mkdirs() }
        event(dir,"trial_start",null,"ids" to arr(ids),"requestedSeconds" to seconds)
        val feeds=mutableListOf<Feed>()
        var success=false
        try {
            val recoveryDeadline=SystemClock.elapsedRealtime()+20_000L
            var ready=false
            while(!ready && SystemClock.elapsedRealtime()<recoveryDeadline) {
                ready=ids.all { id -> try { manager.getCameraCharacteristics(id); true } catch(_:Exception) { false } }
                if(!ready) Thread.sleep(1000)
            }
            if(!ready) { event(dir,"provider_recovery_timeout"); return false }
            ids.forEach { feeds.add(Feed(it,dir,light)) }
            // Open BOTH devices before configuring EITHER session.
            for(f in feeds) {
                f.open()
                if(!f.opened.await(8,TimeUnit.SECONDS)) { f.failed=true; event(dir,"open_timeout",f.id) }
                if(f.failed || feeds.any { it.failed }) return false
            }
            val configs=feeds.associate { it.id to it.prepare() }
            if(feeds.size>1) {
                try { event(dir,"concurrent_configuration_query",null,"supported" to manager.isConcurrentSessionConfigurationSupported(configs)) }
                catch(e:Exception) { event(dir,"concurrent_query_exception",null,"exception" to e.toString()) }
            }
            for(f in feeds) {
                event(dir,"configure_requested",f.id)
                try { f.device!!.createCaptureSession(configs[f.id]!!) } catch(e:Exception) { f.error("configure_exception",e); f.configured.countDown() }
            }
            for(f in feeds) if(!f.configured.await(8,TimeUnit.SECONDS)) { f.failed=true; event(dir,"configure_timeout",f.id) }
            for(f in feeds) if(!f.failed && !f.first.await(8,TimeUnit.SECONDS)) { f.failed=true; event(dir,"first_frame_timeout",f.id) }
            if(feeds.any { it.failed }) return false
            event(dir,"observation_start",null,"seconds" to seconds)
            val start=SystemClock.elapsedRealtime()
            val end=start+seconds*1000L
            var lastPrompt=-1
            val lensTest=intent.getBooleanExtra("lensTest",false)
            val calibrationGuide=intent.getBooleanExtra("calibrationGuide",false)
            val poseHints=listOf("Merkez · düz", "Sol üst · hafif sağa eğik", "Sağ alt · hafif yukarı eğik", "Sol alt · hafif sola eğik", "Sağ üst · hafif aşağı eğik", "Merkez · saat yönünde eğik", "Sol kenar · farklı uzaklık", "Üst kenar · sola eğik", "Sağ kenar · farklı uzaklık", "Alt kenar · sağa eğik")
            val prompts=listOf("HEPSİ AÇIK — hazırlanın", "EN ÜST LENSi KAPATIN", "HEPSİ AÇIK — elinizi çekin", "ORTA LENSİ KAPATIN", "HEPSİ AÇIK — elinizi çekin", "EN ALT LENSİ KAPATIN", "HEPSİ AÇIK — elinizi çekin", "HEPSİ AÇIK — tamamlanıyor")
            while(SystemClock.elapsedRealtime()<end && !stopped.get() && feeds.none { it.failed }) {
                val elapsed=((SystemClock.elapsedRealtime()-start)/1000).toInt()
                if(lensTest && elapsed!=lastPrompt) {
                    lastPrompt=elapsed
                    val phase=minOf(elapsed/10,prompts.lastIndex)
                    if(elapsed%10==0) event(dir,"occlusion_phase",null,"phase" to phase,"instruction" to prompts[phase])
                    runOnUiThread { display.text="Fiziksel lens testi\n\n${prompts[phase]}\n\nSonraki adım: ${10-elapsed%10} saniye\n\nSadece belirtilen lensi kapatın. Flaşa dokunmayın. Telefonu sabit tutun." }
                }
                if(calibrationGuide && elapsed!=lastPrompt) {
                    lastPrompt=elapsed
                    val phase=elapsed/6
                    if(elapsed%6==0) event(dir,"calibration_pose_prompt",null,"phase" to phase,"hint" to poseHints[phase%poseHints.size])
                    runOnUiThread { display.text="Kalibrasyon · Poz ${phase+1}\n${poseHints[phase%poseHints.size]}\n\n${if(elapsed%6<2) "Hedefi yeni poza getirin" else "SABİT TUTUN"} · ${6-elapsed%6} sn\n\nTüm dama iki görüntüde de görünsün. Her tekrarda açı/uzaklığı değiştirin." }
                } else if(!lensTest && !calibrationGuide && elapsed!=lastPrompt) {
                    lastPrompt=elapsed
                    runOnUiThread { display.text="İki arka kamera · ${seconds-elapsed} sn\n\nDama hedefinin tamamı iki görüntüye de sığmalı. Telefon ve hedefi sabit tutun.\n\nÖnizleme 1 Hz; kayıt yaklaşık 15 FPS." }
                }
                Thread.sleep(100)
            }
            val overlap=feeds.minOf { it.lastNs }-feeds.maxOf { it.firstNs }
            success=!stopped.get() && overlap>=(seconds-1)*1_000_000_000L && feeds.all {
                !it.failed && it.count>=seconds*7 && it.maxGapNs<=200_000_000L && (it.lastNs-it.firstNs)>=(seconds-1)*1_000_000_000L
            }
            return success
        } catch(e:Exception) { event(dir,"trial_exception",null,"exception" to e.toString()); return false }
        finally {
            feeds.forEach { try { it.close() } catch(e:Exception) { event(dir,"cleanup_exception",it.id,"exception" to e.toString()) } }
            val summary=obj("ids" to arr(ids),"streamingCandidate" to success,"distinctSensorsVerified" to false,
                "secondsRequested" to seconds,"light" to light,"feeds" to arr(feeds.map { f -> obj("cameraId" to f.id,"frames" to f.count,"firstNs" to f.firstNs,"lastNs" to f.lastNs,"failed" to f.failed,"width" to f.size.width,"height" to f.size.height,"fpsRange" to f.fps.toString()) }))
            File(dir,"summary.json").writeText(summary.toString(2))
            event(dir,"trial_end",null,"streamingCandidate" to success)
            Thread.sleep(750)
        }
    }
}
