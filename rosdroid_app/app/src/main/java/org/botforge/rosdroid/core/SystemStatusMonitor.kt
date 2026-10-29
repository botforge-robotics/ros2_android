package org.botforge.rosdroid.core

import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.net.ConnectivityManager
import android.net.NetworkCapabilities
import android.os.BatteryManager
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.net.Inet4Address
import java.net.NetworkInterface

data class SystemStatus(
    val ipAddress: String = "Not connected",
    val batteryPercentage: Int = 0,
    val isCharging: Boolean = false,
    val isUsbDeviceConnected: Boolean = false,
    val usbDeviceName: String = "None",
    val isRosRunning: Boolean = false,
    val isSshRunning: Boolean = false,
    val rosDomainId: Int = 156,
    val activeTopicsCount: Int = 0
)

class SystemStatusMonitor(private val context: Context) {

    private val _status = MutableStateFlow(SystemStatus())
    val status: StateFlow<SystemStatus> = _status.asStateFlow()

    fun refreshStatus() {
        val ip = getDeviceIpAddress()
        val (battery, charging) = getBatteryInfo()

        _status.value = _status.value.copy(
            ipAddress = ip,
            batteryPercentage = battery,
            isCharging = charging
        )
    }

    fun updateUsbStatus(connected: Boolean, deviceName: String = "None") {
        _status.value = _status.value.copy(
            isUsbDeviceConnected = connected,
            usbDeviceName = deviceName
        )
    }

    fun updateServiceStatus(rosRunning: Boolean, sshRunning: Boolean) {
        _status.value = _status.value.copy(
            isRosRunning = rosRunning,
            isSshRunning = sshRunning
        )
    }

    fun updateRosDomain(domainId: Int) {
        _status.value = _status.value.copy(rosDomainId = domainId)
    }

    fun updateTopicsCount(count: Int) {
        _status.value = _status.value.copy(activeTopicsCount = count)
    }

    private fun getDeviceIpAddress(): String {
        try {
            val interfaces = NetworkInterface.getNetworkInterfaces() ?: return "127.0.0.1"
            for (intf in interfaces) {
                if (intf.isLoopback || !intf.isUp) continue
                // Prefer wlan0
                val addrs = intf.inetAddresses
                for (addr in addrs) {
                    if (!addr.isLoopbackAddress && addr is Inet4Address) {
                        return addr.hostAddress ?: "Unknown"
                    }
                }
            }
        } catch (e: Exception) {
            e.printStackTrace()
        }
        return "127.0.0.1"
    }

    private fun getBatteryInfo(): Pair<Int, Boolean> {
        val intentFilter = IntentFilter(Intent.ACTION_BATTERY_CHANGED)
        val batteryStatus = context.registerReceiver(null, intentFilter) ?: return Pair(0, false)

        val level = batteryStatus.getIntExtra(BatteryManager.EXTRA_LEVEL, -1)
        val scale = batteryStatus.getIntExtra(BatteryManager.EXTRA_SCALE, -1)
        val percent = if (level >= 0 && scale > 0) (level * 100) / scale else 0

        val status = batteryStatus.getIntExtra(BatteryManager.EXTRA_STATUS, -1)
        val isCharging = status == BatteryManager.BATTERY_STATUS_CHARGING ||
                status == BatteryManager.BATTERY_STATUS_FULL

        return Pair(percent, isCharging)
    }
}
