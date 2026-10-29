package org.botforge.rosdroid

import org.botforge.rosdroid.core.SystemStatus
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class SystemStatusTest {

    @Test
    fun testDefaultSystemStatus() {
        val status = SystemStatus()
        assertEquals(156, status.rosDomainId)
        assertEquals("Not connected", status.ipAddress)
        assertFalse(status.isRosRunning)
        assertFalse(status.isUsbDeviceConnected)
    }

    @Test
    fun testStatusTransitions() {
        val initial = SystemStatus()
        val running = initial.copy(
            isRosRunning = true,
            isUsbDeviceConnected = true,
            usbDeviceName = "ESP32-S3 CDC",
            ipAddress = "192.168.1.120"
        )

        assertTrue(running.isRosRunning)
        assertTrue(running.isUsbDeviceConnected)
        assertEquals("ESP32-S3 CDC", running.usbDeviceName)
        assertEquals("192.168.1.120", running.ipAddress)
    }
}
