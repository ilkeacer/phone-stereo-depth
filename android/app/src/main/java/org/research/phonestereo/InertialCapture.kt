package org.research.phonestereo

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread

/** Raw phone-axis angular velocity and acceleration, sampled independently of cameras. */
class InertialCapture(context:Context,private val transport:LivePairs):SensorEventListener,AutoCloseable {
    private val manager=context.getSystemService(SensorManager::class.java)
    private val thread=HandlerThread("phone-inertial")
    private var started=false
    fun start() {
        if(started) return
        started=true
        thread.start()
        val handler=Handler(thread.looper)
        val gyro=manager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        val accel=manager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        transport.sensorNames(gyro?.name,accel?.name)
        if(gyro!=null) manager.registerListener(this,gyro,20_000,0,handler)
        if(accel!=null) manager.registerListener(this,accel,20_000,0,handler)
    }
    override fun onSensorChanged(event:SensorEvent) {
        if(event.values.size<3) return
        val kind=when(event.sensor.type) {
            Sensor.TYPE_GYROSCOPE -> "gyro"
            Sensor.TYPE_ACCELEROMETER -> "accel"
            else -> return
        }
        transport.inertial(kind,event.timestamp,event.values[0],event.values[1],event.values[2],event.accuracy)
    }
    override fun onAccuracyChanged(sensor:Sensor?,accuracy:Int) {}
    override fun close() {
        if(!started) return
        manager.unregisterListener(this)
        thread.quitSafely()
        thread.join(1000)
        started=false
    }
}
