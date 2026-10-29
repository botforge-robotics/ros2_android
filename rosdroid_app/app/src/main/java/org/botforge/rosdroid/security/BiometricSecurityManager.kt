package org.botforge.rosdroid.security

import android.content.Context
import android.os.Build
import android.util.Log
import androidx.biometric.BiometricManager
import androidx.biometric.BiometricPrompt
import androidx.core.content.ContextCompat
import androidx.fragment.app.FragmentActivity
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

sealed class BiometricState {
    object Idle : BiometricState()
    object Authenticating : BiometricState()
    data class Authenticated(val timestamp: Long = System.currentTimeMillis()) : BiometricState()
    data class Failed(val reason: String) : BiometricState()
    object Canceled : BiometricState()
}

class BiometricSecurityManager(private val context: Context) {

    private val _state = MutableStateFlow<BiometricState>(BiometricState.Idle)
    val state: StateFlow<BiometricState> = _state.asStateFlow()

    fun canAuthenticate(): Boolean {
        val biometricManager = BiometricManager.from(context)
        val authenticators = BiometricManager.Authenticators.BIOMETRIC_STRONG or
                BiometricManager.Authenticators.BIOMETRIC_WEAK or
                BiometricManager.Authenticators.DEVICE_CREDENTIAL
        val res = biometricManager.canAuthenticate(authenticators)
        return res == BiometricManager.BIOMETRIC_SUCCESS
    }

    fun promptBiometric(
        activity: FragmentActivity,
        title: String = "Robot Authorization",
        subtitle: String = "Scan Face or Fingerprint to unlock",
        onSuccess: () -> Unit = {},
        onError: (String) -> Unit = {}
    ) {
        val executor = ContextCompat.getMainExecutor(activity)
        _state.value = BiometricState.Authenticating

        val callback = object : BiometricPrompt.AuthenticationCallback() {
            override fun onAuthenticationSucceeded(result: BiometricPrompt.AuthenticationResult) {
                super.onAuthenticationSucceeded(result)
                _state.value = BiometricState.Authenticated()
                Log.i(TAG, "Biometric authentication succeeded")
                onSuccess()
            }

            override fun onAuthenticationError(errorCode: Int, errString: CharSequence) {
                super.onAuthenticationError(errorCode, errString)
                if (errorCode == BiometricPrompt.ERROR_USER_CANCELED || errorCode == BiometricPrompt.ERROR_NEGATIVE_BUTTON) {
                    _state.value = BiometricState.Canceled
                } else {
                    _state.value = BiometricState.Failed(errString.toString())
                }
                Log.w(TAG, "Biometric error: $errString ($errorCode)")
                onError(errString.toString())
            }

            override fun onAuthenticationFailed() {
                super.onAuthenticationFailed()
                _state.value = BiometricState.Failed("Biometric not recognized")
                Log.w(TAG, "Biometric recognition failed")
            }
        }

        val promptInfo = BiometricPrompt.PromptInfo.Builder()
            .setTitle(title)
            .setSubtitle(subtitle)
            .setNegativeButtonText("Cancel")
            .setAllowedAuthenticators(
                BiometricManager.Authenticators.BIOMETRIC_STRONG or
                BiometricManager.Authenticators.BIOMETRIC_WEAK
            )
            .build()

        val biometricPrompt = BiometricPrompt(activity, executor, callback)
        biometricPrompt.authenticate(promptInfo)
    }

    companion object {
        private const val TAG = "BiometricSecurity"
    }
}
