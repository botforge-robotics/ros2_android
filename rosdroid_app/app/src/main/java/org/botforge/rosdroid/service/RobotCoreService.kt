package org.botforge.rosdroid.service

import android.app.Notification
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.net.wifi.WifiManager
import android.os.Binder
import android.os.IBinder
import android.os.PowerManager
import android.util.Log
import androidx.core.app.NotificationCompat
import kotlinx.coroutines.*
import org.botforge.rosdroid.MainActivity
import org.botforge.rosdroid.R
import org.botforge.rosdroid.RosdroidApplication
import org.botforge.rosdroid.ai.EdgeAiManager
import org.botforge.rosdroid.core.PRootManager
import org.botforge.rosdroid.core.SystemStatusMonitor
import org.botforge.rosdroid.hardware.UsbSerialBridge
import org.botforge.rosdroid.remote.RemoteAccessManager
import org.botforge.rosdroid.sensors.SensorHub
import org.botforge.rosdroid.sensors.VoiceInteractionManager

class RobotCoreService : Service() {

    private val binder = LocalBinder()
    private val scope = CoroutineScope(Dispatchers.Main + SupervisorJob())

    lateinit var prootManager: PRootManager
        private set
    lateinit var sensorHub: SensorHub
        private set
    lateinit var usbBridge: UsbSerialBridge
        private set
    lateinit var remoteManager: RemoteAccessManager
        private set
    lateinit var statusMonitor: SystemStatusMonitor
        private set
    lateinit var voiceManager: VoiceInteractionManager
        private set
    lateinit var aiManager: EdgeAiManager
        private set
    lateinit var bridgeServer: RosBridgeServer
        private set

    var onAuthRequested: ((goalId: String, message: String) -> Unit)? = null
    var onCameraToggleRequested: ((direction: Int, status: Boolean) -> Unit)? = null
    var onTorchRequested: ((enabled: Boolean) -> Unit)? = null

    private var wakeLock: PowerManager.WakeLock? = null
    private var wifiLock: WifiManager.WifiLock? = null
    private var multicastLock: WifiManager.MulticastLock? = null

    private var isCoreRunning = false

    inner class LocalBinder : Binder() {
        fun getService(): RobotCoreService = this@RobotCoreService
    }

    override fun onCreate() {
        super.onCreate()
        Log.i(TAG, "RobotCoreService created")

        prootManager = PRootManager(this)
        sensorHub = SensorHub(this)
        usbBridge = UsbSerialBridge(this)
        remoteManager = RemoteAccessManager(this, prootManager)
        statusMonitor = SystemStatusMonitor(this)
        voiceManager = VoiceInteractionManager(this)
        aiManager = EdgeAiManager(this)
        bridgeServer = RosBridgeServer(this)

        // Setup Bridge Listeners
        voiceManager.onHotwordTriggered = { hotword ->
            bridgeServer.publishHotwordDetected(hotword)
        }
        voiceManager.onSpeechResult = { text ->
            bridgeServer.publishSpeechResult(text)
        }
        voiceManager.onSpeechStatusChanged = { status ->
            bridgeServer.publishSpeechStatus(status)
        }

        sensorHub.onSensorEventCallback = { type, data ->
            when (type) {
                "imu" -> {
                    val imu = data as org.botforge.rosdroid.sensors.ImuData
                    bridgeServer.publishImu(
                        imu.qW, imu.qX, imu.qY, imu.qZ,
                        imu.gx, imu.gy, imu.gz,
                        imu.ax, imu.ay, imu.az,
                        imu.timestampMs
                    )
                }
                "illuminance" -> bridgeServer.publishIlluminance(data as Float)
                "tof" -> bridgeServer.publishTof((data as Float) / 100.0f)
                "gps" -> {
                    val g = data as org.botforge.rosdroid.sensors.GpsData
                    bridgeServer.publishGps(g.latitude, g.longitude, g.altitude, g.speed)
                }
            }
        }

        bridgeServer.onTtsGoal = { goalId, text, voiceOutput, startExpr, endExpr, sound ->
            bridgeServer.setRobotExpression(startExpr)
            if (voiceOutput) {
                voiceManager.speak(
                    text = text,
                    onWord = { startIdx, endIdx, word ->
                        bridgeServer.sendTtsFeedback(goalId, startIdx, endIdx, word, startExpr)
                    },
                    onDone = {
                        bridgeServer.setRobotExpression(endExpr)
                        bridgeServer.sendTtsResult(goalId, true, endExpr)
                    }
                )
            } else {
                bridgeServer.setRobotExpression(endExpr)
                bridgeServer.sendTtsResult(goalId, true, endExpr)
            }
        }

        bridgeServer.onAuthGoal = { goalId, message ->
            onAuthRequested?.invoke(goalId, message)
        }

        bridgeServer.onCameraService = { srvId, direction, status ->
            onCameraToggleRequested?.invoke(direction, status)
            bridgeServer.sendCameraResult(srvId, true, "Camera lens updated")
        }

        bridgeServer.onTorchRequested = { enabled ->
            if (onTorchRequested != null) {
                onTorchRequested?.invoke(enabled)
            } else {
                bridgeServer.setFlashlight(enabled)
            }
        }

        bridgeServer.onCmdVelReceived = { linX, angZ ->
            usbBridge.sendVelocity(linX, angZ)
        }

        bridgeServer.start()

        // Ensure rootfs networking and symlink integrity
        try {
            val installer = org.botforge.rosdroid.core.RootfsInstaller(this)
            installer.setupRootfsConfigurations()
        } catch (e: Exception) {
            Log.w(TAG, "Error ensuring rootfs configurations", e)
        }

        acquireLocks()
        startForeground(NOTIFICATION_ID, buildNotification("Initializing Rosdroid Core..."))

        // Eagerly start sensors for telemetry
        sensorHub.start()

        // Periodic telemetry monitor
        scope.launch {
            while (isActive) {
                statusMonitor.refreshStatus()
                val st = statusMonitor.status.value
                if (sensorHub.streamConfig.value.streamBattery) {
                    bridgeServer.publishBattery(st.batteryPercentage, st.isCharging)
                }
                updateNotification()
                delay(3000)
            }
        }
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_START -> startRobotCore()
            ACTION_STOP -> stopRobotCore()
        }
        return START_STICKY
    }

    fun startRobotCore() {
        if (isCoreRunning) return
        isCoreRunning = true

        Log.i(TAG, "Starting Rosdroid Robot Core...")

        // 1. Start Sensors & Voice
        sensorHub.start()
        voiceManager.start()

        // 2. Start Micro-ROS Agent & Rosdroid ROS2 Node in PRoot
        prootManager.startMicroRosAgent(port = 8888, domainId = statusMonitor.status.value.rosDomainId)
        prootManager.startRosNode(domainId = statusMonitor.status.value.rosDomainId)

        // 3. Connect USB if available
        usbBridge.findAndConnectDevice()

        // 4. Start Remote Terminal Access (SSH Server)
        remoteManager.startSsh()

        statusMonitor.updateServiceStatus(
            rosRunning = true,
            sshRunning = remoteManager.isSshRunning
        )

        updateNotification()
    }

    fun stopRobotCore() {
        if (!isCoreRunning) return
        isCoreRunning = false

        Log.i(TAG, "Stopping Rosdroid Robot Core...")
        sensorHub.stop()
        voiceManager.stop()
        bridgeServer.stop()
        usbBridge.disconnect()
        remoteManager.stopSsh()
        prootManager.stopAll()

        statusMonitor.updateServiceStatus(rosRunning = false, sshRunning = false)
        stopForeground(STOP_FOREGROUND_REMOVE)
        stopSelf()
    }

    private fun acquireLocks() {
        val powerManager = getSystemService(Context.POWER_SERVICE) as PowerManager
        wakeLock = powerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "Rosdroid::WakeLock").apply {
            setReferenceCounted(false)
            acquire(24 * 60 * 60 * 1000L) // 24 hours
        }

        val wifiManager = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        wifiLock = wifiManager.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "Rosdroid::WifiLock").apply {
            setReferenceCounted(false)
            acquire()
        }

        multicastLock = wifiManager.createMulticastLock("Rosdroid::MulticastLock").apply {
            setReferenceCounted(false)
            acquire()
        }
    }

    private fun releaseLocks() {
        try {
            if (wakeLock?.isHeld == true) wakeLock?.release()
            if (wifiLock?.isHeld == true) wifiLock?.release()
            if (multicastLock?.isHeld == true) multicastLock?.release()
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    private fun buildNotification(statusText: String): Notification {
        val pendingIntent = PendingIntent.getActivity(
            this,
            0,
            Intent(this, MainActivity::class.java),
            PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT
        )

        return NotificationCompat.Builder(this, RosdroidApplication.CHANNEL_ID)
            .setContentTitle("Rosdroid Robot Active")
            .setContentText(statusText)
            .setSmallIcon(R.drawable.ic_launcher_foreground)
            .setContentIntent(pendingIntent)
            .setOngoing(true)
            .setPriority(NotificationCompat.PRIORITY_LOW)
            .build()
    }

    private fun updateNotification() {
        val status = statusMonitor.status.value
        val info = "IP: ${status.ipAddress} | Bat: ${status.batteryPercentage}% | Domain: ${status.rosDomainId}"
        val notification = buildNotification(info)
        val manager = getSystemService(Context.NOTIFICATION_SERVICE) as android.app.NotificationManager
        manager.notify(NOTIFICATION_ID, notification)
    }

    override fun onBind(intent: Intent?): IBinder = binder

    override fun onDestroy() {
        super.onDestroy()
        stopRobotCore()
        releaseLocks()
        scope.cancel()
        Log.i(TAG, "RobotCoreService destroyed")
    }

    companion object {
        private const val TAG = "RobotCoreService"
        private const val NOTIFICATION_ID = 1001

        const val ACTION_START = "org.botforge.rosdroid.action.START"
        const val ACTION_STOP = "org.botforge.rosdroid.action.STOP"

        fun start(context: Context) {
            val intent = Intent(context, RobotCoreService::class.java).apply { action = ACTION_START }
            context.startForegroundService(intent)
        }

        fun stop(context: Context) {
            val intent = Intent(context, RobotCoreService::class.java).apply { action = ACTION_STOP }
            context.startService(intent)
        }
    }
}
