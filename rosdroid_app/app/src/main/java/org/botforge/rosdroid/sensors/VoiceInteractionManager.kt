package org.botforge.rosdroid.sensors

import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.speech.RecognitionListener
import android.speech.RecognizerIntent
import android.speech.SpeechRecognizer
import android.speech.tts.TextToSpeech
import android.speech.tts.UtteranceProgressListener
import android.util.Log
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.util.Locale

class VoiceInteractionManager(
    private val context: Context,
    var primaryHotword: String = "Hey Robot"
) : RecognitionListener, TextToSpeech.OnInitListener {

    // Supported robust wake words (case-insensitive)
    val supportedHotwords = listOf("hey robot", "robot", "hey jarvis", "computer", "rosdroid", "hey rosdroid")

    private val mainHandler = Handler(Looper.getMainLooper())
    private var speechRecognizer: SpeechRecognizer? = null
    private var textToSpeech: TextToSpeech? = null
    private var isTtsReady = false
    private var isListeningEnabled = false
    private var isSpeaking = false

    private val _lastRecognizedSpeech = MutableStateFlow("")
    val lastRecognizedSpeech: StateFlow<String> = _lastRecognizedSpeech.asStateFlow()

    private val _isListening = MutableStateFlow(false)
    val isListening: StateFlow<Boolean> = _isListening.asStateFlow()

    // Callbacks for ROS2 integration
    var onHotwordTriggered: ((String) -> Unit)? = null
    var onSpeechResult: ((String) -> Unit)? = null
    var onSpeechStatusChanged: ((String) -> Unit)? = null
    var onTtsWordProgress: ((startIdx: Int, endIdx: Int, word: String) -> Unit)? = null
    var onTtsCompleted: (() -> Unit)? = null

    init {
        try {
            textToSpeech = TextToSpeech(context.applicationContext, this)
        } catch (e: Exception) {
            Log.w(TAG, "Failed to initialize TextToSpeech eagerly", e)
        }
    }

    fun start() {
        isListeningEnabled = true
        mainHandler.post {
            initSpeechRecognizer()
            startListening()
        }

        if (textToSpeech == null) {
            textToSpeech = TextToSpeech(context.applicationContext, this)
        }
    }

    private fun initSpeechRecognizer() {
        try {
            speechRecognizer?.destroy()
            speechRecognizer = null
            if (SpeechRecognizer.isRecognitionAvailable(context)) {
                speechRecognizer = SpeechRecognizer.createSpeechRecognizer(context).apply {
                    setRecognitionListener(this@VoiceInteractionManager)
                }
                Log.i(TAG, "SpeechRecognizer created successfully on main looper")
            } else {
                Log.w(TAG, "SpeechRecognizer isRecognitionAvailable reported false")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed creating SpeechRecognizer", e)
        }
    }

    fun startListening() {
        if (!isListeningEnabled || isSpeaking) return

        mainHandler.post {
            try {
                if (speechRecognizer == null) {
                    initSpeechRecognizer()
                }
                val intent = Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH).apply {
                    putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
                    putExtra(RecognizerIntent.EXTRA_LANGUAGE, Locale.getDefault())
                    putExtra(RecognizerIntent.EXTRA_PARTIAL_RESULTS, true)
                    putExtra(RecognizerIntent.EXTRA_MAX_RESULTS, 3)
                }
                speechRecognizer?.startListening(intent)
                _isListening.value = true
                onSpeechStatusChanged?.invoke("listening")
                Log.d(TAG, "Speech listening started")
            } catch (e: Exception) {
                Log.e(TAG, "Error starting speech listening", e)
                restartListeningWithDelay(1000)
            }
        }
    }

    fun stopListening() {
        isListeningEnabled = false
        mainHandler.post {
            try {
                speechRecognizer?.stopListening()
            } catch (ignored: Exception) {}
            _isListening.value = false
            onSpeechStatusChanged?.invoke("done")
        }
    }

    private fun restartListeningWithDelay(delayMs: Long) {
        if (!isListeningEnabled || isSpeaking) return
        mainHandler.postDelayed({
            if (isListeningEnabled && !isSpeaking) {
                startListening()
            }
        }, delayMs)
    }

    fun speak(
        text: String,
        onWord: ((startIdx: Int, endIdx: Int, word: String) -> Unit)? = null,
        onDone: (() -> Unit)? = null
    ) {
        if (!isTtsReady || textToSpeech == null) {
            onDone?.invoke()
            return
        }

        isSpeaking = true
        mainHandler.post {
            try {
                speechRecognizer?.stopListening()
            } catch (ignored: Exception) {}
            _isListening.value = false
        }

        val utteranceId = "RosdroidTTS_${System.currentTimeMillis()}"
        this.onTtsWordProgress = onWord
        this.onTtsCompleted = onDone

        textToSpeech?.setOnUtteranceProgressListener(object : UtteranceProgressListener() {
            override fun onStart(utteranceId: String?) {
                Log.d(TAG, "TTS started: $utteranceId")
            }

            override fun onRangeStart(utteranceId: String?, start: Int, end: Int, frame: Int) {
                val word = if (start >= 0 && end <= text.length && start <= end) {
                    text.substring(start, end)
                } else ""
                onTtsWordProgress?.invoke(start, end, word)
            }

            override fun onDone(utteranceId: String?) {
                Log.d(TAG, "TTS finished: $utteranceId")
                isSpeaking = false
                onTtsCompleted?.invoke()
                restartListeningWithDelay(400)
            }

            override fun onError(utteranceId: String?) {
                Log.w(TAG, "TTS error: $utteranceId")
                isSpeaking = false
                onTtsCompleted?.invoke()
                restartListeningWithDelay(400)
            }
        })

        textToSpeech?.speak(text, TextToSpeech.QUEUE_FLUSH, null, utteranceId)
    }

    override fun onInit(status: Int) {
        if (status == TextToSpeech.SUCCESS) {
            textToSpeech?.language = Locale.US
            isTtsReady = true
            Log.i(TAG, "TextToSpeech initialized successfully")
        }
    }

    override fun onResults(results: Bundle?) {
        val matches = results?.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION)
        if (!matches.isNullOrEmpty()) {
            val text = matches[0]
            _lastRecognizedSpeech.value = text
            Log.i(TAG, "Speech recognized: $text")

            // Always publish recognized speech topic
            onSpeechResult?.invoke(text)

            // Check if any robust hotword triggered
            val lower = text.lowercase(Locale.getDefault())
            val matchedHotword = supportedHotwords.firstOrNull { lower.contains(it) }
                ?: if (lower.contains(primaryHotword.lowercase(Locale.getDefault()))) primaryHotword else null

            if (matchedHotword != null) {
                Log.i(TAG, "Hotword '$matchedHotword' detected! Triggering ROS topic...")
                onHotwordTriggered?.invoke(matchedHotword)
            }
        }
        _isListening.value = false
        onSpeechStatusChanged?.invoke("done")

        // Automatically restart listening loop for continuous robot voice interaction
        restartListeningWithDelay(250)
    }

    override fun onPartialResults(partialResults: Bundle?) {
        val matches = partialResults?.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION)
        if (!matches.isNullOrEmpty()) {
            val partial = matches[0]
            _lastRecognizedSpeech.value = partial

            // Fast hotword detection on partial results
            val lower = partial.lowercase(Locale.getDefault())
            val matchedHotword = supportedHotwords.firstOrNull { lower.contains(it) }
                ?: if (lower.contains(primaryHotword.lowercase(Locale.getDefault()))) primaryHotword else null

            if (matchedHotword != null) {
                Log.i(TAG, "Partial hotword '$matchedHotword' detected!")
                onHotwordTriggered?.invoke(matchedHotword)
            }
        }
    }

    override fun onError(error: Int) {
        val errorMsg = when (error) {
            SpeechRecognizer.ERROR_AUDIO -> "Audio recording error"
            SpeechRecognizer.ERROR_CLIENT -> "Client error"
            SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS -> "Insufficient permissions"
            SpeechRecognizer.ERROR_NETWORK -> "Network error"
            SpeechRecognizer.ERROR_NETWORK_TIMEOUT -> "Network timeout"
            SpeechRecognizer.ERROR_NO_MATCH -> "No match"
            SpeechRecognizer.ERROR_RECOGNIZER_BUSY -> "Recognizer busy"
            SpeechRecognizer.ERROR_SERVER -> "Server error"
            SpeechRecognizer.ERROR_SPEECH_TIMEOUT -> "Speech timeout"
            else -> "Error $error"
        }
        Log.w(TAG, "SpeechRecognizer onError: $errorMsg ($error)")
        _isListening.value = false

        if (!isListeningEnabled || isSpeaking) return

        if (error == SpeechRecognizer.ERROR_RECOGNIZER_BUSY || error == SpeechRecognizer.ERROR_CLIENT) {
            mainHandler.postDelayed({
                if (isListeningEnabled && !isSpeaking) {
                    initSpeechRecognizer()
                    startListening()
                }
            }, 500)
        } else {
            restartListeningWithDelay(350)
        }
    }

    override fun onReadyForSpeech(params: Bundle?) {
        _isListening.value = true
        onSpeechStatusChanged?.invoke("listening")
    }

    override fun onBeginningOfSpeech() {}
    override fun onRmsChanged(rmsdB: Float) {}
    override fun onBufferReceived(buffer: ByteArray?) {}
    override fun onEndOfSpeech() {
        _isListening.value = false
    }
    override fun onEvent(eventType: Int, params: Bundle?) {}

    fun stop() {
        isListeningEnabled = false
        mainHandler.post {
            try {
                speechRecognizer?.destroy()
            } catch (ignored: Exception) {}
            speechRecognizer = null
            textToSpeech?.stop()
            textToSpeech?.shutdown()
            textToSpeech = null
            isTtsReady = false
            _isListening.value = false
        }
    }

    companion object {
        private const val TAG = "VoiceInteraction"
    }
}
