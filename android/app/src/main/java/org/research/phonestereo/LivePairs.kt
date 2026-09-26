package org.research.phonestereo

import android.os.SystemClock
import org.json.JSONArray
import org.json.JSONObject
import java.io.DataOutputStream
import java.net.InetAddress
import java.net.ServerSocket
import java.util.ArrayDeque

/** ADB-forwarded loopback transport. Bounded buffers; sensor timestamps stay intact. */
class LivePairs(private val sourceRun:String) : AutoCloseable {
    data class Frame(val timestamp:Long,val bytes:ByteArray,val row:JSONObject)
    private data class Inertial(val sequence:Long,val kind:String,val timestampNs:Long,
                                val x:Float,val y:Float,val z:Float,val accuracy:Int)
    private data class Packet(val header:JSONObject,val frames:List<Frame>,val lastInertialSequence:Long)
    private val frames=mutableMapOf<String,LinkedHashMap<Long,Frame>>()
    private val meta=mutableMapOf<String,LinkedHashMap<Long,JSONObject>>()
    private val inertial=ArrayDeque<Inertial>()
    private var nextInertialSequence=0L
    private var deliveredInertialSequence=0L
    private var droppedInertial=0L
    private var inertialSensors=JSONObject()
    private val server=ServerSocket(8765,2,InetAddress.getByName("127.0.0.1"))
    @Volatile private var running=true
    init {
        Thread({
            while(running) {
                try {
                    server.accept().use { socket ->
                        socket.soTimeout=2000
                        // One bounded snapshot per connection; no remote control commands.
                        val packet=snapshot()
                        val header=packet.header.toString().toByteArray(Charsets.UTF_8)
                        val out=DataOutputStream(socket.getOutputStream())
                        out.writeInt(header.size);out.write(header)
                        for(frame in packet.frames) out.write(frame.bytes)
                        out.flush()
                        acknowledgeInertial(packet.lastInertialSequence)
                    }
                } catch(_:Exception) { /* Disconnects never interrupt the camera callbacks. */ }
            }
        },"adb-pair-server").start()
    }
    @Synchronized fun sensorNames(gyro:String?,accelerometer:String?) {
        inertialSensors=JSONObject().put("gyro",gyro ?: JSONObject.NULL).put("accelerometer",accelerometer ?: JSONObject.NULL)
    }
    @Synchronized fun inertial(kind:String,timestampNs:Long,x:Float,y:Float,z:Float,accuracy:Int) {
        if(!running || timestampNs<=0 || !x.isFinite() || !y.isFinite() || !z.isFinite()) return
        nextInertialSequence++
        inertial.addLast(Inertial(nextInertialSequence,kind,timestampNs,x,y,z,accuracy))
        while(inertial.size>512) {
            val removed=inertial.removeFirst()
            if(removed.sequence>deliveredInertialSequence) droppedInertial++
        }
    }
    @Synchronized private fun acknowledgeInertial(sequence:Long) {
        if(sequence<=deliveredInertialSequence) return
        deliveredInertialSequence=sequence
        while(inertial.isNotEmpty() && inertial.first.sequence<=sequence) inertial.removeFirst()
    }
    @Synchronized fun frame(id:String,bytes:ByteArray,row:JSONObject) {
        val ts=row.getLong("imageTimestampNs")
        val cache=frames.getOrPut(id) { linkedMapOf() }
        cache[ts]=Frame(ts,bytes,JSONObject(row.toString()))
        while(cache.size>16) cache.remove(cache.keys.first())
    }
    @Synchronized fun metadata(id:String,row:JSONObject) {
        val cache=meta.getOrPut(id) { linkedMapOf() }
        cache[row.getLong("sensorTimestampNs")]=JSONObject(row.toString())
        while(cache.size>32) cache.remove(cache.keys.first())
    }
    @Synchronized private fun snapshot():Packet {
        val now=SystemClock.elapsedRealtimeNanos()
        val a=frames["20"]?.values?.toList().orEmpty()
        val b=frames["21"]?.values?.toList().orEmpty()
        for(left in a.asReversed()) {
            if(now-left.row.getLong("arrivalElapsedNs")>500_000_000L || meta["20"]?.get(left.timestamp)==null) continue
            val right=b.filter { meta["21"]?.get(it.timestamp)!=null && now-it.row.getLong("arrivalElapsedNs")<500_000_000L }
                .minByOrNull { kotlin.math.abs(left.timestamp-it.timestamp) } ?: continue
            val dt=kotlin.math.abs(left.timestamp-right.timestamp)
            // Always display the newest frames. Temporal eligibility gates capture ONLY.
            val header=JSONObject().put("ok",true).put("paired",dt<=20_000_000L).put("deltaNs",dt).put("deviceElapsedNs",now).put("sourceRun",sourceRun)
            for((id,f) in listOf("20" to left,"21" to right)) {
                header.put(id,JSONObject().put("length",f.bytes.size).put("image",f.row).put("capture",meta[id]!![f.timestamp]))
            }
            val samples=inertial.filter { it.sequence>deliveredInertialSequence }.take(128)
            val rows=JSONArray()
            for(sample in samples) rows.put(JSONObject().put("sequence",sample.sequence).put("kind",sample.kind)
                .put("timestampNs",sample.timestampNs).put("x",sample.x.toDouble()).put("y",sample.y.toDouble())
                .put("z",sample.z.toDouble()).put("accuracy",sample.accuracy))
            header.put("imuSamples",rows).put("imuDroppedTotal",droppedInertial).put("imuSensors",inertialSensors)
            return Packet(header,listOf(left,right),samples.lastOrNull()?.sequence ?: deliveredInertialSequence)
        }
        return Packet(JSONObject().put("ok",false).put("reason","Waiting for fresh pair within 20 ms with metadata"),
                      emptyList(),deliveredInertialSequence)
    }
    override fun close() { running=false;server.close() }
}
