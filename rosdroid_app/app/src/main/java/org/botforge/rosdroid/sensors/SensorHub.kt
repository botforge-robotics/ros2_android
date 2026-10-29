package org.botforge.rosdroid.sensors

import android.annotation.SuppressLint
import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Bundle
import android.util.Log
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.nio.ByteBuffer
import java.nio.ByteOrder

data class ImuData(
    val qW: Float = 1.0f,
    val qX: Float = 0.0f,
    val qY: Float = 0.0f,
    val qZ: Float = 0.0f,
    val gx: Float = 0.0f,
    val gy: Float = 0.0f,
    val gz: Float = 0.0f,
    val ax: Float = 0.0f,
    val ay: Float = 0.0f,
    val az: Float = 9.81f,
    val timestampMs: Long = 0L
)

data class EnvironmentalData(
    val illuminanceLux: Float = 0.0f,
    val pressureHpa: Float = 1013.25f,
    val proximityCm: Float = 0.0f
)

data class GpsData(
    val latitude: Double = 0.0,
    val longitude: Double = 0.0,
    val altitude: Double = 0.0,
    val speed: Float = 0.0f,
    val accuracy: Float = 0.0f,
    val hasFix: Boolean = false
)

data class SensorStreamConfig(
    val streamCamera: Boolean = true,
    val streamImu: Boolean = true,
    val streamGps: Boolean = true,
    val streamIlluminance: Boolean = true,
    val streamTof: Boolean = true,
    val streamBattery: Boolean = true
)

class SensorHub(private val context: Context, private val rosBridgePort: Int = 9870) :
    SensorEventListener, LocationListener {

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager

    private val imuFusion = ImuFusion().apply {
        initFilter(100.0f, 0.1f)
    }

    private val _imuState = MutableStateFlow(ImuData())
    val imuState: StateFlow<ImuData> = _imuState.asStateFlow()

    private val _envState = MutableStateFlow(EnvironmentalData())
    val envState: StateFlow<EnvironmentalData> = _envState.asStateFlow()

    private val _gpsState = MutableStateFlow(GpsData())
    val gpsState: StateFlow<GpsData> = _gpsState.asStateFlow()

    private val _streamConfig = MutableStateFlow(SensorStreamConfig())
    val streamConfig: StateFlow<SensorStreamConfig> = _streamConfig.asStateFlow()

    var onSensorEventCallback: ((type: String, data: Any) -> Unit)? = null

    fun updateStreamConfig(config: SensorStreamConfig) {
        _streamConfig.value = config
    }

    fun toggleCamera(enabled: Boolean) {
        _streamConfig.value = _streamConfig.value.copy(streamCamera = enabled)
    }

    fun toggleImu(enabled: Boolean) {
        _streamConfig.value = _streamConfig.value.copy(streamImu = enabled)
    }

    fun toggleGps(enabled: Boolean) {
        _streamConfig.value = _streamConfig.value.copy(streamGps = enabled)
    }

    fun toggleIlluminance(enabled: Boolean) {
        _streamConfig.value = _streamConfig.value.copy(streamIlluminance = enabled)
    }

    fun toggleTof(enabled: Boolean) {
        _streamConfig.value = _streamConfig.value.copy(streamTof = enabled)
    }

    fun toggleBattery(enabled: Boolean) {
        _streamConfig.value = _streamConfig.value.copy(streamBattery = enabled)
    }

    private var socket: DatagramSocket? = null
    private val localhost = InetAddress.getByName("127.0.0.1")
    private val udpBuffer = ByteBuffer.allocate(128).order(ByteOrder.LITTLE_ENDIAN)

    private var lastAx = 0f; private var lastAy = 0f; private var lastAz = 9.81f
    private var lastGx = 0f; private var lastGy = 0f; private var lastGz = 0f
    private var lastMx = 0f; private var lastMy = 0f; private var lastMz = 0f
    private var lastImuPublishTime = 0L

    private fun dispatchImu(now: Long) {
        val q = imuFusion.updateIMU(lastGx, lastGy, lastGz, lastAx, lastAy, lastAz)
        val imu = ImuData(
            qW = q[0], qX = q[1], qY = q[2], qZ = q[3],
            gx = lastGx, gy = lastGy, gz = lastGz,
            ax = lastAx, ay = lastAy, az = lastAz,
            timestampMs = now
        )
        _imuState.value = imu
        sendImuPacket(imu)

        if (_streamConfig.value.streamImu) {
            if (now - lastImuPublishTime >= 33L) { // ~30Hz streaming
                lastImuPublishTime = now
                onSensorEventCallback?.invoke("imu", imu)
            }
        }
    }

    private var isRunning = false
    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())

    fun start() {
        if (isRunning) return
        isRunning = true

        try {
            socket = DatagramSocket()
        } catch (e: Exception) {
            Log.e(TAG, "Failed to create sensor UDP socket", e)
        }

        // 1. Register IMU sensors
        val accel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        val mag = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)

        sensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_FASTEST)
        sensorManager.registerListener(this, gyro, SensorManager.SENSOR_DELAY_FASTEST)
        sensorManager.registerListener(this, mag, SensorManager.SENSOR_DELAY_GAME)

        // 2. Register Environmental sensors
        val light = sensorManager.getDefaultSensor(Sensor.TYPE_LIGHT)
        val pressure = sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE)
        val proximity = sensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY)

        sensorManager.registerListener(this, light, SensorManager.SENSOR_DELAY_NORMAL)
        sensorManager.registerListener(this, pressure, SensorManager.SENSOR_DELAY_NORMAL)
        sensorManager.registerListener(this, proximity, SensorManager.SENSOR_DELAY_NORMAL)

        // 3. Register GPS
        startGpsUpdates()

        Log.i(TAG, "SensorHub started with IMU, Environmental, and GPS listeners")
    }

    @SuppressLint("MissingPermission")
    private fun startGpsUpdates() {
        try {
            if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
                locationManager.requestLocationUpdates(
                    LocationManager.GPS_PROVIDER,
                    1000L, // 1 second
                    1.0f,  // 1 meter
                    this
                )
            }
            if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
                locationManager.requestLocationUpdates(
                    LocationManager.NETWORK_PROVIDER,
                    2000L,
                    2.0f,
                    this
                )
            }
        } catch (e: SecurityException) {
            Log.w(TAG, "Location permission not granted, GPS disabled", e)
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start GPS updates", e)
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        val now = System.currentTimeMillis()

        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                lastAx = event.values[0]
                lastAy = event.values[1]
                lastAz = event.values[2]
                dispatchImu(now)
            }
            Sensor.TYPE_GYROSCOPE -> {
                lastGx = event.values[0]
                lastGy = event.values[1]
                lastGz = event.values[2]
                dispatchImu(now)
            }
            Sensor.TYPE_MAGNETIC_FIELD -> {
                lastMx = event.values[0]
                lastMy = event.values[1]
                lastMz = event.values[2]
            }
            Sensor.TYPE_LIGHT -> {
                val lux = event.values[0]
                _envState.value = _envState.value.copy(illuminanceLux = lux)
                if (_streamConfig.value.streamIlluminance) {
                    onSensorEventCallback?.invoke("illuminance", lux)
                }
            }
            Sensor.TYPE_PRESSURE -> {
                _envState.value = _envState.value.copy(pressureHpa = event.values[0])
            }
            Sensor.TYPE_PROXIMITY -> {
                val prox = event.values[0]
                _envState.value = _envState.value.copy(proximityCm = prox)
                if (_streamConfig.value.streamTof) {
                    onSensorEventCallback?.invoke("tof", prox)
                }
            }
        }
    }

    private fun sendImuPacket(imu: ImuData) {
        if (!_streamConfig.value.streamImu) return
        val s = socket ?: return
        try {
            synchronized(udpBuffer) {
                udpBuffer.clear()
                udpBuffer.put(0x49.toByte()) // 'I'
                udpBuffer.put(0x4D.toByte()) // 'M'
                udpBuffer.put(0x55.toByte()) // 'U'
                udpBuffer.put(0x01.toByte()) // Version 1

                udpBuffer.putLong(imu.timestampMs)
                udpBuffer.putFloat(imu.qW)
                udpBuffer.putFloat(imu.qX)
                udpBuffer.putFloat(imu.qY)
                udpBuffer.putFloat(imu.qZ)

                udpBuffer.putFloat(imu.gx)
                udpBuffer.putFloat(imu.gy)
                udpBuffer.putFloat(imu.gz)

                udpBuffer.putFloat(imu.ax)
                udpBuffer.putFloat(imu.ay)
                udpBuffer.putFloat(imu.az)

                val packet = DatagramPacket(udpBuffer.array(), udpBuffer.position(), localhost, rosBridgePort)
                s.send(packet)
            }
        } catch (e: Exception) {
            // Ignore temporary socket error to avoid logging flood at 100Hz
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    override fun onLocationChanged(location: Location) {
        val gps = GpsData(
            latitude = location.latitude,
            longitude = location.longitude,
            altitude = location.altitude,
            speed = location.speed,
            accuracy = location.accuracy,
            hasFix = true
        )
        _gpsState.value = gps
        if (_streamConfig.value.streamGps) {
            onSensorEventCallback?.invoke("gps", gps)
        }
    }

    override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) {}
    override fun onProviderEnabled(provider: String) {}
    override fun onProviderDisabled(provider: String) {}

    fun stop() {
        isRunning = false
        sensorManager.unregisterListener(this)
        try {
            locationManager.removeUpdates(this)
        } catch (e: Exception) {
            e.printStackTrace()
        }
        socket?.close()
        socket = null
        scope.cancel()
    }

    companion object {
        private const val TAG = "SensorHub"
    }
}
