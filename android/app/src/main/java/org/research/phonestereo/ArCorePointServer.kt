package org.research.phonestereo

import java.net.InetAddress
import java.net.ServerSocket
import java.net.SocketException
import java.net.Socket
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong

/** Local ADB-forwarded ARCore feed; detailed mode may include private RGB JPEGs. */
class ArCorePointServer(port:Int=8766) : AutoCloseable {
    private val server = ServerSocket(port, 1, InetAddress.getByName("127.0.0.1"))
    val localPort:Int get()=server.localPort
    private val pending = ArrayBlockingQueue<String>(24)
    private val offered=AtomicLong(0)
    private val sent=AtomicLong(0)
    private val dropped=AtomicLong(0)
    val droppedPackets:Long get()=dropped.get()
    @Volatile private var running = true
    @Volatile private var connected:Socket?=null
    private val worker = Thread({
        while (running) {
            try {
                server.accept().use { client ->
                    connected=client
                    val output = client.getOutputStream().bufferedWriter()
                    while (running) {
                        val row = pending.poll(500, TimeUnit.MILLISECONDS) ?: continue
                        output.write(row)
                        output.newLine()
                        output.flush()
                        sent.incrementAndGet()
                    }
                }
            } catch (_: SocketException) {
                if (!running) break
            } catch (_: Exception) {
                // A disconnected observer may reconnect while this probe remains active.
            } finally { connected=null }
        }
    }, "arcore-points").apply { isDaemon = true; start() }

    fun send(row: String) {
        if (!running) return
        offered.incrementAndGet()
        if (!pending.offer(row)) {
            if(pending.poll()!=null) dropped.incrementAndGet()
            pending.offer(row)
        }
    }

    override fun close() {
        // Flush the final depth/end packet for an attached observer before EOF.
        val deadline=System.nanoTime()+1_500_000_000L
        while(connected!=null && sent.get()+dropped.get()<offered.get() && System.nanoTime()<deadline)
            Thread.sleep(5)
        running = false
        server.close()
        connected?.close()
        worker.interrupt()
    }
}
