package org.botforge.rosdroid.sensors

import android.content.Context
import android.content.Intent
import android.os.Bundle
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

    // Supported robust wake words
    val supportedHotwords = listOf("hey robot", "robot", "hey jarvis", "computer", "rosdroid", "hey rosdroid")

    private var speechRecognizer: SpeechRecognizer? = null
    private var textToSpeech: TextToSpeech? = null
    private var isTtsReady = false

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
        if (SpeechRecognizer.isRecognitionAvailable(context)) {
            speechRecognizer = SpeechRecognizer.createSpeechRecognizer(context).apply {
                setRecognitionListener(this@VoiceInteractionManager)
            }
            startListening()
        }

        if (textToSpeech == null) {
            textToSpeech = TextToSpeech(context.applicationContext, this)
        }
    }

    fun startListening() {
        try {
            val intent = Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH).apply {
                putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
                putExtra(RecognizerIntent.EXTRA_LANGUAGE, Locale.getDefault())
                putExtra(RecognizerIntent.EXTRA_PARTIAL_RESULTS, true)
            }
            speechRecognizer?.startListening(intent)
            _isListening.value = true
            onSpeechStatusChanged?.invoke("listening")
        } catch (e: Exception) {
            Log.e(TAG, "Error starting speech listening", e)
        }
    }

    fun stopListening() {
        speechRecognizer?.stopListening()
        _isListening.value = false
        onSpeechStatusChanged?.invoke("done")
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
                onTtsCompleted?.invoke()
            }

            override fun onError(utteranceId: String?) {
                Log.w(TAG, "TTS error: $utteranceId")
                onTtsCompleted?.invoke()
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
        startListening()
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
        _isListening.value = false
        // Auto-restart after timeout/silence
        if (error == SpeechRecognizer.ERROR_NO_MATCH || error == SpeechRecognizer.ERROR_SPEECH_TIMEOUT) {
            startListening()
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
        speechRecognizer?.destroy()
        speechRecognizer = null
        textToSpeech?.stop()
        textToSpeech?.shutdown()
        textToSpeech = null
        isTtsReady = false
        _isListening.value = false
    }

    companion object {
        private const val TAG = "VoiceInteraction"
    }
}
