package org.botforge.rosdroid

import android.Manifest
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.content.pm.PackageManager
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Bundle
import android.os.IBinder
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.core.content.ContextCompat
import androidx.fragment.app.FragmentActivity
import org.botforge.rosdroid.core.RootfsInstaller
import org.botforge.rosdroid.core.SystemStatus
import org.botforge.rosdroid.hardware.UsbConnectionState
import org.botforge.rosdroid.security.BiometricSecurityManager
import org.botforge.rosdroid.sensors.CameraConfig
import org.botforge.rosdroid.sensors.CameraStreamer
import org.botforge.rosdroid.sensors.EnvironmentalData
import org.botforge.rosdroid.sensors.GpsData
import org.botforge.rosdroid.sensors.ImuData
import org.botforge.rosdroid.service.RobotCoreService
import org.botforge.rosdroid.ui.*

enum class AppTab(val title: String, val icon: ImageVector) {
    FACE("Face", Icons.Default.Face),
    DASHBOARD("Dashboard", Icons.Default.Dashboard),
    CONSOLE("Console", Icons.Default.Terminal)
}

class MainActivity : FragmentActivity() {

    private var robotService: RobotCoreService? = null
    private var isBound = false
    val biometricManager by lazy { BiometricSecurityManager(this) }
    val cameraStreamer by lazy { CameraStreamer(this) }

    fun triggerBiometricAuth(
        subtitle: String = "Verify Face ID or Fingerprint",
        onResult: (Boolean, String) -> Unit
    ) {
        if (!biometricManager.canAuthenticate()) {
            onResult(false, "Biometrics not configured on this device")
            return
        }
        biometricManager.promptBiometric(
            activity = this,
            title = "Robot Security Authorization",
            subtitle = subtitle,
            onSuccess = { onResult(true, "Authenticated") },
            onError = { err -> onResult(false, err) }
        )
    }

    private val serviceConnection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, service: IBinder?) {
            val binder = service as RobotCoreService.LocalBinder
            robotService = binder.getService()
            isBound = true
            Log.i(TAG, "Connected to RobotCoreService - auto-starting core")

            // Automatically start Robot Core on app open!
            robotService?.startRobotCore()

            // Setup biometric auth action listener
            robotService?.onAuthRequested = { goalId, message ->
                runOnUiThread {
                    triggerBiometricAuth(message) { success, errorMsg ->
                        robotService?.bridgeServer?.sendAuthResult(goalId, success, errorMsg)
                    }
                }
            }

            // Setup camera toggle action listener
            robotService?.onCameraToggleRequested = { direction, status ->
                runOnUiThread {
                    val useFront = (direction == 0)
                    cameraStreamer.switchLens(this@MainActivity, useFront)
                }
            }

            // Setup torch action listener
            robotService?.onTorchRequested = { enabled ->
                runOnUiThread {
                    cameraStreamer.setTorch(enabled)
                }
            }

            // Check if there is an intent device to connect
            checkUsbIntent(intent)
        }

        override fun onServiceDisconnected(name: ComponentName?) {
            robotService?.onTorchRequested = null
            robotService = null
            isBound = false
            Log.i(TAG, "Disconnected from RobotCoreService")
        }
    }

    private var cameraUdpSocket: java.net.DatagramSocket? = null
    private val localhost by lazy { java.net.InetAddress.getByName("127.0.0.1") }

    private fun initCameraStream() {
        if (cameraUdpSocket == null) {
            try {
                cameraUdpSocket = java.net.DatagramSocket()
            } catch (ignored: Exception) {}
        }
        cameraStreamer.startCamera(this, CameraConfig(useFrontCamera = cameraStreamer.isFront.value)) { jpegBytes, _ ->
            if (robotService?.sensorHub?.streamConfig?.value?.streamCamera == true) {
                try {
                    val s = cameraUdpSocket ?: return@startCamera
                    val buf = ByteArray(jpegBytes.size + 1)
                    buf[0] = if (cameraStreamer.isFront.value) 0 else 1
                    System.arraycopy(jpegBytes, 0, buf, 1, jpegBytes.size)
                    val packet = java.net.DatagramPacket(buf, buf.size, localhost, 9872)
                    s.send(packet)
                } catch (ignored: Exception) {}
            }
        }
    }

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val cameraGranted = permissions[Manifest.permission.CAMERA] ?: false
        val locGranted = permissions[Manifest.permission.ACCESS_FINE_LOCATION] ?: false
        Log.i(TAG, "Permissions updated: Camera=$cameraGranted, Location=$locGranted")
        if (cameraGranted) {
            initCameraStream()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        requestRequiredPermissions()

        // Bind to background service
        val serviceIntent = Intent(this, RobotCoreService::class.java)
        startForegroundService(serviceIntent)
        bindService(serviceIntent, serviceConnection, Context.BIND_AUTO_CREATE)

        setContent {
            RosdroidAppUI()
        }
    }

    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        checkUsbIntent(intent)
    }

    private fun checkUsbIntent(intent: Intent?) {
        if (intent?.action == UsbManager.ACTION_USB_DEVICE_ATTACHED) {
            val device: UsbDevice? = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                intent.getParcelableExtra(UsbManager.EXTRA_DEVICE, UsbDevice::class.java)
            } else {
                @Suppress("DEPRECATION")
                intent.getParcelableExtra(UsbManager.EXTRA_DEVICE)
            }
            device?.let {
                Log.i(TAG, "USB Device Attached via Intent: ${it.deviceName}")
                robotService?.usbBridge?.connect(it)
            }
        }
    }

    private fun requestRequiredPermissions() {
        val permissions = mutableListOf(
            Manifest.permission.CAMERA,
            Manifest.permission.RECORD_AUDIO,
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION
        )
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            permissions.add(Manifest.permission.POST_NOTIFICATIONS)
        }

        val needed = permissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (needed.isNotEmpty()) {
            permissionLauncher.launch(needed.toTypedArray())
        } else {
            initCameraStream()
        }
    }

    @Composable
    private fun RosdroidAppUI() {
        val installer = remember { RootfsInstaller(this@MainActivity) }
        var isInstalled by remember { mutableStateOf(installer.isInstalled()) }
        var currentTab by remember { mutableStateOf(AppTab.DASHBOARD) }

        val service = robotService
        val systemStatus = service?.statusMonitor?.status?.collectAsState()?.value ?: SystemStatus()
        val usbState = service?.usbBridge?.connectionState?.collectAsState()?.value ?: UsbConnectionState.Disconnected
        val imuData = service?.sensorHub?.imuState?.collectAsState()?.value ?: ImuData()
        val envData = service?.sensorHub?.envState?.collectAsState()?.value ?: EnvironmentalData()
        val gpsData = service?.sensorHub?.gpsState?.collectAsState()?.value ?: GpsData()
        val streamConfig = service?.sensorHub?.streamConfig?.collectAsState()?.value ?: org.botforge.rosdroid.sensors.SensorStreamConfig()
        val speechText = service?.voiceManager?.lastRecognizedSpeech?.collectAsState()?.value ?: ""
        val isListening = service?.voiceManager?.isListening?.collectAsState()?.value ?: true
        val bioState = biometricManager.state.collectAsState().value
        val isFrontCamera = cameraStreamer.isFront.collectAsState().value
        val cameraFps = cameraStreamer.fps.collectAsState().value
        val isTorchOn = cameraStreamer.isTorchOn.collectAsState().value

        if (!isInstalled) {
            // On fresh open: directly show the setup & installation wizard (no tabs!)
            InstallerScreen(
                installer = installer,
                onInstallComplete = {
                    isInstalled = true
                    robotService?.startRobotCore()
                }
            )
        } else {
            Scaffold(
                bottomBar = {
                    NavigationBar(
                        containerColor = Color(0xFF0A0F1D),
                        contentColor = Color.White
                    ) {
                        AppTab.values().forEach { tab ->
                            NavigationBarItem(
                                icon = { Icon(tab.icon, contentDescription = tab.title) },
                                label = { Text(tab.title) },
                                selected = currentTab == tab,
                                onClick = { currentTab = tab },
                                colors = NavigationBarItemDefaults.colors(
                                    selectedIconColor = Color(0xFF00E676),
                                    selectedTextColor = Color(0xFF00E676),
                                    unselectedIconColor = Color.Gray,
                                    unselectedTextColor = Color.Gray,
                                    indicatorColor = Color(0xFF1E293B)
                                )
                            )
                        }
                    }
                }
            ) { paddingValues ->
                Box(
                    modifier = Modifier
                        .fillMaxSize()
                        .padding(paddingValues)
                        .background(Color(0xFF0F172A))
                ) {
                    when (currentTab) {
                        AppTab.FACE -> {
                            RobotFaceScreen()
                        }
                        AppTab.DASHBOARD -> {
                            DashboardScreen(
                                status = systemStatus,
                                usbState = usbState,
                                imuData = imuData,
                                envData = envData,
                                gpsData = gpsData,
                                streamConfig = streamConfig,
                                speechText = speechText,
                                isSpeechListening = isListening,
                                biometricState = bioState,
                                isFrontCamera = isFrontCamera,
                                cameraFps = cameraFps,
                                isTorchOn = isTorchOn,
                                onToggleTorch = {
                                    cameraStreamer.setTorch(!isTorchOn)
                                },
                                onSwitchCamera = {
                                    cameraStreamer.toggleLens(this@MainActivity)
                                },
                                onToggleSensor = { sensorName, enabled ->
                                    when (sensorName) {
                                        "camera" -> service?.sensorHub?.toggleCamera(enabled)
                                        "imu" -> service?.sensorHub?.toggleImu(enabled)
                                        "gps" -> service?.sensorHub?.toggleGps(enabled)
                                        "illuminance" -> service?.sensorHub?.toggleIlluminance(enabled)
                                        "tof" -> service?.sensorHub?.toggleTof(enabled)
                                        "battery" -> service?.sensorHub?.toggleBattery(enabled)
                                    }
                                },
                                onTestSpeak = { text ->
                                    service?.voiceManager?.speak(text)
                                },
                                onTriggerBiometrics = {
                                    triggerBiometricAuth { success, msg ->
                                        Log.i(TAG, "Biometric auth result: $success, $msg")
                                    }
                                },
                                onDomainChange = { newDomain ->
                                    robotService?.statusMonitor?.updateRosDomain(newDomain)
                                }
                            )
                        }
                        AppTab.CONSOLE -> {
                            robotService?.prootManager?.let { proot ->
                                ConsoleScreen(
                                    prootManager = proot,
                                    domainId = systemStatus.rosDomainId
                                )
                            } ?: Box(modifier = Modifier.fillMaxSize()) {
                                Text("Service connecting...", color = Color.Gray)
                            }
                        }
                    }
                }
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        if (isBound) {
            unbindService(serviceConnection)
            isBound = false
        }
    }

    companion object {
        private const val TAG = "RosdroidMainActivity"
    }
}
