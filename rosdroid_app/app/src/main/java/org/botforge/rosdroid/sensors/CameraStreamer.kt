package org.botforge.rosdroid.sensors

import android.content.Context
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.util.Log
import androidx.camera.core.*
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import java.util.concurrent.Executors

data class CameraConfig(
    val useFrontCamera: Boolean = false,
    val targetFps: Int = 30,
    val width: Int = 640,
    val height: Int = 480,
    val jpegQuality: Int = 80
)

class CameraStreamer(private val context: Context) {

    private val cameraExecutor = Executors.newSingleThreadExecutor()
    private var cameraProvider: ProcessCameraProvider? = null
    private var camera: Camera? = null
    private var isTorchEnabled = false

    private val _isTorchOn = MutableStateFlow(false)
    val isTorchOn: StateFlow<Boolean> = _isTorchOn.asStateFlow()

    private val _latestFrame = MutableStateFlow<ByteArray?>(null)
    val latestFrame: StateFlow<ByteArray?> = _latestFrame.asStateFlow()

    private val _isFront = MutableStateFlow(false)
    val isFront: StateFlow<Boolean> = _isFront.asStateFlow()

    private val _fps = MutableStateFlow(0)
    val fps: StateFlow<Int> = _fps.asStateFlow()

    private var frameCount = 0
    private var lastFpsTimestamp = System.currentTimeMillis()

    fun toggleLens(lifecycleOwner: LifecycleOwner) {
        val newFacing = !_isFront.value
        _isFront.value = newFacing
        startCamera(lifecycleOwner, CameraConfig(useFrontCamera = newFacing))
    }

    fun switchLens(lifecycleOwner: LifecycleOwner, useFront: Boolean) {
        _isFront.value = useFront
        startCamera(lifecycleOwner, CameraConfig(useFrontCamera = useFront))
    }

    fun startCamera(
        lifecycleOwner: LifecycleOwner,
        config: CameraConfig = CameraConfig(),
        onJpegFrame: ((ByteArray, Long) -> Unit)? = null
    ) {
        val cameraProviderFuture = ProcessCameraProvider.getInstance(context)
        cameraProviderFuture.addListener({
            cameraProvider = cameraProviderFuture.get()

            val cameraSelector = if (config.useFrontCamera) {
                CameraSelector.DEFAULT_FRONT_CAMERA
            } else {
                CameraSelector.DEFAULT_BACK_CAMERA
            }

            val imageAnalysis = ImageAnalysis.Builder()
                .setTargetResolution(android.util.Size(config.width, config.height))
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888)
                .build()

            imageAnalysis.setAnalyzer(cameraExecutor) { imageProxy ->
                val jpegBytes = imageProxyToJpeg(imageProxy, config.jpegQuality)
                imageProxy.close()

                if (jpegBytes != null) {
                    val now = System.currentTimeMillis()
                    _latestFrame.value = jpegBytes
                    onJpegFrame?.invoke(jpegBytes, now)

                    // Track FPS
                    frameCount++
                    if (now - lastFpsTimestamp >= 1000) {
                        _fps.value = frameCount
                        frameCount = 0
                        lastFpsTimestamp = now
                    }
                }
            }

            try {
                camera = null
                cameraProvider?.unbindAll()
                val boundCamera = cameraProvider?.bindToLifecycle(
                    lifecycleOwner,
                    cameraSelector,
                    imageAnalysis
                )
                camera = boundCamera
                Log.i(TAG, "Camera streaming initialized (${config.width}x${config.height})")

                if (isTorchEnabled && boundCamera?.cameraInfo?.hasFlashUnit() == true) {
                    try {
                        boundCamera.cameraControl.enableTorch(true)
                        Log.i(TAG, "Restored torch state on camera re-bind")
                    } catch (e: Exception) {
                        Log.w(TAG, "Failed to restore torch on camera re-bind", e)
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "Camera binding failed", e)
            }
        }, ContextCompat.getMainExecutor(context))
    }

    fun setTorch(enabled: Boolean): Boolean {
        _isTorchOn.value = enabled
        isTorchEnabled = enabled
        val cam = camera
        if (cam != null && cam.cameraInfo.hasFlashUnit()) {
            return try {
                cam.cameraControl.enableTorch(enabled)
                Log.i(TAG, "CameraX torch set to $enabled")
                true
            } catch (e: Exception) {
                Log.w(TAG, "CameraX enableTorch failed, attempting fallback", e)
                fallbackTorch(enabled)
            }
        }
        return fallbackTorch(enabled)
    }

    private fun fallbackTorch(enabled: Boolean): Boolean {
        return try {
            val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as android.hardware.camera2.CameraManager
            val cameraId = cameraManager.cameraIdList.firstOrNull { id ->
                val chars = cameraManager.getCameraCharacteristics(id)
                chars.get(android.hardware.camera2.CameraCharacteristics.FLASH_INFO_AVAILABLE) == true
            } ?: cameraManager.cameraIdList.firstOrNull()

            if (cameraId != null) {
                cameraManager.setTorchMode(cameraId, enabled)
                Log.i(TAG, "CameraManager fallback torch set to $enabled on camera $cameraId")
                true
            } else {
                Log.w(TAG, "No camera with flash found for fallback torch")
                false
            }
        } catch (e: Exception) {
            Log.w(TAG, "Fallback CameraManager failed to set torch", e)
            false
        }
    }

    private fun imageProxyToJpeg(image: ImageProxy, quality: Int): ByteArray? {
        val yBuffer = image.planes[0].buffer
        val uBuffer = image.planes[1].buffer
        val vBuffer = image.planes[2].buffer

        val ySize = yBuffer.remaining()
        val uSize = uBuffer.remaining()
        val vSize = vBuffer.remaining()

        val nv21 = ByteArray(ySize + uSize + vSize)

        yBuffer.get(nv21, 0, ySize)
        vBuffer.get(nv21, ySize, vSize)
        uBuffer.get(nv21, ySize + vSize, uSize)

        val yuvImage = YuvImage(nv21, ImageFormat.NV21, image.width, image.height, null)
        val out = ByteArrayOutputStream()
        val success = yuvImage.compressToJpeg(Rect(0, 0, image.width, image.height), quality, out)
        return if (success) out.toByteArray() else null
    }

    fun stopCamera() {
        if (isTorchEnabled) {
            try { camera?.cameraControl?.enableTorch(false) } catch (ignored: Exception) {}
        }
        _isTorchOn.value = false
        isTorchEnabled = false
        camera = null
        cameraProvider?.unbindAll()
        cameraExecutor.shutdown()
    }

    companion object {
        private const val TAG = "CameraStreamer"
    }
}
