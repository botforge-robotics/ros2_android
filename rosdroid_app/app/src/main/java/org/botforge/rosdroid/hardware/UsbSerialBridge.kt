package org.botforge.rosdroid.hardware

import android.content.Context
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.util.Log
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress

sealed class UsbConnectionState {
    object Disconnected : UsbConnectionState()
    data class Connecting(val deviceName: String) : UsbConnectionState()
    data class Connected(val deviceName: String, val baudRate: Int) : UsbConnectionState()
    data class Error(val message: String) : UsbConnectionState()
}

class UsbSerialBridge(
    private val context: Context,
    private val targetAgentPort: Int = 8888,
    private val baudRate: Int = 115200
) {
    private val usbManager = context.getSystemService(Context.USB_SERVICE) as UsbManager
    private var serialPort: UsbSerialPort? = null
    private var udpSocket: DatagramSocket? = null
    private val localhost = InetAddress.getByName("127.0.0.1")

    private val _connectionState = MutableStateFlow<UsbConnectionState>(UsbConnectionState.Disconnected)
    val connectionState: StateFlow<UsbConnectionState> = _connectionState.asStateFlow()

    private var bridgeJob: Job? = null
    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())

    fun findAndConnectDevice(): Boolean {
        val availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager)
        if (availableDrivers.isEmpty()) {
            Log.d(TAG, "No USB serial devices detected.")
            _connectionState.value = UsbConnectionState.Disconnected
            return false
        }

        val driver = availableDrivers[0]
        val device = driver.device

        if (!usbManager.hasPermission(device)) {
            Log.w(TAG, "Missing USB permission for device ${device.deviceName}")
            _connectionState.value = UsbConnectionState.Error("USB permission required for ${device.deviceName}")
            return false
        }

        return connect(device)
    }

    fun connect(device: UsbDevice): Boolean {
        disconnect()
        _connectionState.value = UsbConnectionState.Connecting(device.deviceName)

        val driver = UsbSerialProber.getDefaultProber().probeDevice(device)
            ?: UsbSerialProber.getDefaultProber().findAllDrivers(usbManager).firstOrNull { it.device == device }

        if (driver == null) {
            _connectionState.value = UsbConnectionState.Error("No compatible USB serial driver found")
            return false
        }

        val connection = usbManager.openDevice(driver.device)
        if (connection == null) {
            _connectionState.value = UsbConnectionState.Error("Failed to open USB connection")
            return false
        }

        try {
            val port = driver.ports[0]
            port.open(connection)
            port.setParameters(baudRate, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)

            // DTR/RTS for ESP32 auto-reset handling
            port.dtr = true
            port.rts = true

            serialPort = port
            _connectionState.value = UsbConnectionState.Connected(device.deviceName, baudRate)

            startBridgeThreads()
            Log.i(TAG, "Connected to ${device.deviceName} at $baudRate baud -> Bridging to UDP $targetAgentPort")
            return true
        } catch (e: Exception) {
            Log.e(TAG, "Failed to initialize USB serial port", e)
            _connectionState.value = UsbConnectionState.Error("Init failed: ${e.message}")
            disconnect()
            return false
        }
    }

    private fun startBridgeThreads() {
        bridgeJob = scope.launch {
            try {
                // Open local UDP socket for relaying to/from Micro-ROS Agent
                val socket = DatagramSocket()
                udpSocket = socket

                // Task 1: USB -> UDP (Micro-ROS Agent)
                val usbToUdpJob = launch {
                    val rxBuffer = ByteArray(2048)
                    while (isActive) {
                        try {
                            val bytesRead = serialPort?.read(rxBuffer, 50) ?: 0
                            if (bytesRead > 0) {
                                val packet = DatagramPacket(rxBuffer, bytesRead, localhost, targetAgentPort)
                                socket.send(packet)
                            }
                        } catch (e: Exception) {
                            if (isActive) delay(10)
                        }
                    }
                }

                // Task 2: UDP -> USB (Microcontroller TX)
                val udpToUsbJob = launch {
                    val udpBuffer = ByteArray(2048)
                    while (isActive) {
                        try {
                            val packet = DatagramPacket(udpBuffer, udpBuffer.size)
                            socket.receive(packet)
                            if (packet.length > 0) {
                                val payload = packet.data.copyOfRange(packet.offset, packet.offset + packet.length)
                                serialPort?.write(payload, 50)
                            }
                        } catch (e: Exception) {
                            if (isActive) delay(10)
                        }
                    }
                }

                joinAll(usbToUdpJob, udpToUsbJob)
            } catch (e: Exception) {
                Log.e(TAG, "Bridge loop terminated", e)
            }
        }
    }

    fun sendVelocity(linearX: Float, angularZ: Float) {
        try {
            val cmd = "CMD,${String.format(java.util.Locale.US, "%.2f,%.2f", linearX, angularZ)}\n"
            serialPort?.write(cmd.toByteArray(), 50)
        } catch (ignored: Exception) {}
    }

    fun disconnect() {
        bridgeJob?.cancel()
        bridgeJob = null

        try {
            udpSocket?.close()
        } catch (e: Exception) {
            e.printStackTrace()
        }
        udpSocket = null

        try {
            serialPort?.close()
        } catch (e: Exception) {
            e.printStackTrace()
        }
        serialPort = null

        _connectionState.value = UsbConnectionState.Disconnected
        Log.i(TAG, "USB serial bridge disconnected")
    }

    companion object {
        private const val TAG = "UsbSerialBridge"
    }
}
