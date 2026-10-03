package org.research.phonestereo

import java.net.Socket
import org.junit.Assert.*
import org.junit.Test

class ArCorePointServerTest {
    @Test fun closeFlushesFinalPacketForAttachedObserver() {
        val server=ArCorePointServer(0)
        Socket("127.0.0.1",server.localPort).use { client ->
            client.soTimeout=3000
            val reader=client.getInputStream().bufferedReader()
            server.send("ready")
            assertEquals("ready",reader.readLine())
            server.send("depth");server.send("end")
            server.close()
            assertEquals("depth",reader.readLine())
            assertEquals("end",reader.readLine())
            assertNull(reader.readLine())
        }
    }
}
