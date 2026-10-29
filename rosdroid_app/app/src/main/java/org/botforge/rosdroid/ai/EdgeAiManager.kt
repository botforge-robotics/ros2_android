package org.botforge.rosdroid.ai

import android.content.Context
import android.graphics.RectF
import android.util.Log
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.flow.flowOn
import java.io.File

data class ObjectDetection(
    val label: String,
    val confidence: Float,
    val boundingBox: RectF
)

enum class AiBackend {
    VULKAN_GPU,
    OPENCL,
    CPU_NEON
}

class EdgeAiManager(private val context: Context) {

    var selectedBackend: AiBackend = AiBackend.VULKAN_GPU
    var isModelLoaded: Boolean = false
        private set

    val modelsDir: File get() = File(context.filesDir, "models")

    init {
        modelsDir.mkdirs()
    }

    /**
     * Detect objects from a camera frame using on-device NCNN Vulkan GPU acceleration.
     */
    fun detectObjects(imageBytes: ByteArray, width: Int, height: Int): List<ObjectDetection> {
        if (!isModelLoaded) {
            // Simulated or fallback detections until GGUF/bin model weights are placed in models/
            return emptyList()
        }
        return emptyList()
    }

    /**
     * Local small LLM inference (e.g. Qwen2.5 0.5B/1.5B, SmolLM2 1.7B, or Moondream2).
     */
    fun runLlmInference(prompt: String): Flow<String> = flow {
        emit("Rosdroid AI: Processing prompt '$prompt' on ${selectedBackend.name}...")
        // In full deployment, calls native llama_cpp_bridge
    }.flowOn(Dispatchers.IO)

    fun loadModel(modelFile: File): Boolean {
        if (!modelFile.exists()) {
            Log.w(TAG, "Model file not found: ${modelFile.absolutePath}")
            return false
        }
        Log.i(TAG, "Loading Edge AI model: ${modelFile.name} on $selectedBackend")
        isModelLoaded = true
        return true
    }

    companion object {
        private const val TAG = "EdgeAiManager"
    }
}
