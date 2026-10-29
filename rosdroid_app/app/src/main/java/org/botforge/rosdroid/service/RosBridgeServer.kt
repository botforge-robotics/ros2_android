package org.botforge.rosdroid.service

import android.content.Context
import android.hardware.camera2.CameraManager
import android.util.Log
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import org.json.JSONObject
import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.PrintWriter
import java.net.ServerSocket
import java.net.Socket
import java.util.concurrent.ConcurrentLinkedQueue

class RosBridgeServer(
    private val context: Context,
    private val port: Int = 9871
) {
    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())
    private var serverSocket: ServerSocket? = null
    private var clientSocket: Socket? = null
    private var writer: PrintWriter? = null
    private var isRunning = false

    private val outgoingQueue = ConcurrentLinkedQueue<String>()

    private val _currentExpression = MutableStateFlow("idle")
    val currentExpression: StateFlow<String> = _currentExpression.asStateFlow()

    private val _isRosConnected = MutableStateFlow(false)
    val isRosConnected: StateFlow<Boolean> = _isRosConnected.asStateFlow()

    // Callbacks for Actions and Services
    var onTtsGoal: ((goalId: String, text: String, voiceOutput: Boolean, startExpr: String, endExpr: String, sound: Boolean) -> Unit)? = null
    var onAuthGoal: ((goalId: String, message: String) -> Unit)? = null
    var onCameraService: ((srvId: String, direction: Int, status: Boolean) -> Unit)? = null
    var onExpressionService: ((srvId: String, expression: String, sound: Boolean) -> Unit)? = null
    var onCmdVelReceived: ((linearX: Float, angularZ: Float) -> Unit)? = null
    var onTorchRequested: ((enabled: Boolean) -> Unit)? = null

    fun start() {
        if (isRunning) return
        isRunning = true

        scope.launch {
            try {
                serverSocket = ServerSocket(port)
                Log.i(TAG, "RosBridgeServer listening on port $port")

                while (isActive && isRunning) {
                    val socket = serverSocket?.accept() ?: break
                    clientSocket = socket
                    _isRosConnected.value = true
                    Log.i(TAG, "ROS2 Client connected from ${socket.remoteSocketAddress}")

                    val reader = BufferedReader(InputStreamReader(socket.getInputStream()))
                    writer = PrintWriter(socket.getOutputStream(), true)

                    // Start outgoing pump
                    val pumpJob = launch {
                        while (isActive && isRunning && !socket.isClosed) {
                            val msg = outgoingQueue.poll()
                            if (msg != null) {
                                writer?.println(msg)
                            } else {
                                delay(10)
                            }
                        }
                    }

                    try {
                        var line: String?
                        while (reader.readLine().also { line = it } != null) {
                            line?.let { handleIncomingMessage(it) }
                        }
                    } catch (e: Exception) {
                        Log.w(TAG, "Socket read error / client disconnected", e)
                    } finally {
                        pumpJob.cancel()
                        _isRosConnected.value = false
                        writer = null
                        try { socket.close() } catch (ignored: Exception) {}
                        Log.i(TAG, "ROS2 Client connection closed")
                    }
                }
            } catch (e: Exception) {
                if (isRunning) {
                    Log.e(TAG, "RosBridgeServer error", e)
                }
            }
        }
    }

    private fun handleIncomingMessage(jsonStr: String) {
        try {
            val json = JSONObject(jsonStr)
            when (json.optString("type")) {
                "action_tts_goal" -> {
                    val goalId = json.getString("goal_id")
                    val text = json.optString("text", "")
                    val voiceOutput = json.optBoolean("voice_output", true)
                    val startExpr = json.optString("start_expression", "speaking")
                    val endExpr = json.optString("end_expression", "idle")
                    val sound = json.optBoolean("expression_sound", false)
                    Log.i(TAG, "Incoming TTS Action Goal: '$text' (expr: $startExpr -> $endExpr)")
                    onTtsGoal?.invoke(goalId, text, voiceOutput, startExpr, endExpr, sound)
                }
                "action_auth_goal" -> {
                    val goalId = json.getString("goal_id")
                    val message = json.optString("message", "Please authenticate with Face ID or fingerprint")
                    Log.i(TAG, "Incoming Auth Action Goal: '$message'")
                    onAuthGoal?.invoke(goalId, message)
                }
                "srv_camera" -> {
                    val srvId = json.getString("srv_id")
                    val direction = json.optInt("direction", 0)
                    val status = json.optBoolean("status", true)
                    Log.i(TAG, "Incoming Camera Service: direction=$direction status=$status")
                    onCameraService?.invoke(srvId, direction, status)
                }
                "srv_expression" -> {
                    val srvId = json.getString("srv_id")
                    val expression = json.optString("expression", "happy")
                    val sound = json.optBoolean("expression_sound", false)
                    _currentExpression.value = expression
                    Log.i(TAG, "Incoming SetExpression Service: $expression")
                    onExpressionService?.invoke(srvId, expression, sound)
                    sendExpressionResult(srvId, true)
                }
                "srv_get_expression" -> {
                    val srvId = json.getString("srv_id")
                    val res = JSONObject().apply {
                        put("type", "srv_get_expression_result")
                        put("srv_id", srvId)
                        put("expression", _currentExpression.value)
                    }
                    sendJson(res.toString())
                }
                "topic_torch" -> {
                    val enabled = json.optBoolean("data", false)
                    Log.i(TAG, "Incoming topic_torch: $enabled")
                    if (onTorchRequested != null) {
                        onTorchRequested?.invoke(enabled)
                    } else {
                        setFlashlight(enabled)
                    }
                }
                "topic_cmd_vel" -> {
                    val linX = json.optDouble("linear_x", 0.0).toFloat()
                    val angZ = json.optDouble("angular_z", 0.0).toFloat()
                    onCmdVelReceived?.invoke(linX, angZ)
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to parse incoming bridge JSON: $jsonStr", e)
        }
    }

    fun sendJson(jsonStr: String) {
        outgoingQueue.offer(jsonStr)
    }

    // Sensor publishing helpers
    fun publishImu(w: Float, x: Float, y: Float, z: Float, gx: Float, gy: Float, gz: Float, ax: Float, ay: Float, az: Float, timestampMs: Long) {
        val json = JSONObject().apply {
            put("type", "imu")
            put("qw", w)
            put("qx", x)
            put("qy", y)
            put("qz", z)
            put("gx", gx)
            put("gy", gy)
            put("gz", gz)
            put("ax", ax)
            put("ay", ay)
            put("az", az)
            put("ts", timestampMs)
        }
        sendJson(json.toString())
    }

    fun publishIlluminance(lux: Float) {
        val json = JSONObject().apply {
            put("type", "illuminance")
            put("lux", lux)
        }
        sendJson(json.toString())
    }

    fun publishTof(rangeMeters: Float) {
        val json = JSONObject().apply {
            put("type", "tof")
            put("range", rangeMeters)
        }
        sendJson(json.toString())
    }

    fun publishGps(lat: Double, lon: Double, alt: Double, speed: Float) {
        val json = JSONObject().apply {
            put("type", "gps")
            put("lat", lat)
            put("lon", lon)
            put("alt", alt)
            put("speed", speed)
        }
        sendJson(json.toString())
    }

    fun publishBattery(percentage: Int, isCharging: Boolean) {
        val json = JSONObject().apply {
            put("type", "battery")
            put("percentage", percentage)
            put("charging", isCharging)
        }
        sendJson(json.toString())
    }

    fun publishHotwordDetected(hotword: String) {
        val json = JSONObject().apply {
            put("type", "hotword_detected")
            put("hotword", hotword)
        }
        sendJson(json.toString())
    }

    fun publishSpeechResult(text: String) {
        val json = JSONObject().apply {
            put("type", "speech_result")
            put("text", text)
        }
        sendJson(json.toString())
    }

    fun publishSpeechStatus(status: String) {
        val json = JSONObject().apply {
            put("type", "speech_status")
            put("status", status)
        }
        sendJson(json.toString())
    }

    fun setRobotExpression(expression: String) {
        _currentExpression.value = expression
        val json = JSONObject().apply {
            put("type", "expression")
            put("expression", expression)
        }
        sendJson(json.toString())
    }

    // Action response helpers
    fun sendTtsFeedback(goalId: String, startIdx: Int, endIdx: Int, word: String, expression: String) {
        val json = JSONObject().apply {
            put("type", "action_tts_feedback")
            put("goal_id", goalId)
            put("start_idx", startIdx)
            put("end_idx", endIdx)
            put("word", word)
            put("expression", expression)
        }
        sendJson(json.toString())
    }

    fun sendTtsResult(goalId: String, success: Boolean, expression: String) {
        val json = JSONObject().apply {
            put("type", "action_tts_result")
            put("goal_id", goalId)
            put("success", success)
            put("expression", expression)
        }
        sendJson(json.toString())
    }

    fun sendAuthResult(goalId: String, success: Boolean, errorMessage: String = "") {
        val json = JSONObject().apply {
            put("type", "action_auth_result")
            put("goal_id", goalId)
            put("success", success)
            put("error_message", errorMessage)
        }
        sendJson(json.toString())
    }

    fun sendCameraResult(srvId: String, success: Boolean, message: String) {
        val json = JSONObject().apply {
            put("type", "srv_camera_result")
            put("srv_id", srvId)
            put("success", success)
            put("message", message)
        }
        sendJson(json.toString())
    }

    fun sendExpressionResult(srvId: String, success: Boolean) {
        val json = JSONObject().apply {
            put("type", "srv_expression_result")
            put("srv_id", srvId)
            put("success", success)
        }
        sendJson(json.toString())
    }

    fun setFlashlight(enabled: Boolean) {
        try {
            val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
            val cameraId = cameraManager.cameraIdList.firstOrNull { id ->
                val chars = cameraManager.getCameraCharacteristics(id)
                chars.get(android.hardware.camera2.CameraCharacteristics.FLASH_INFO_AVAILABLE) == true
            } ?: cameraManager.cameraIdList.firstOrNull()

            if (cameraId != null) {
                cameraManager.setTorchMode(cameraId, enabled)
                Log.i(TAG, "Flashlight set to $enabled on camera $cameraId")
            }
        } catch (e: Exception) {
            Log.w(TAG, "Failed to control flashlight", e)
        }
    }

    fun stop() {
        isRunning = false
        try { clientSocket?.close() } catch (ignored: Exception) {}
        try { serverSocket?.close() } catch (ignored: Exception) {}
        clientSocket = null
        serverSocket = null
        writer = null
        scope.cancel()
    }

    companion object {
        private const val TAG = "RosBridgeServer"
    }
}
