package org.botforge.rosdroid

import android.app.Application
import android.app.NotificationChannel
import android.app.NotificationManager
import android.os.Build
import android.util.Log

class RosdroidApplication : Application() {

    override fun onCreate() {
        super.onCreate()
        instance = this

        // Load native C++ libraries (Madgwick sensor fusion & getifaddrs patch)
        try {
            System.loadLibrary("native_bridge")
            Log.i(TAG, "Successfully loaded libnative_bridge.so")
        } catch (e: UnsatisfiedLinkError) {
            Log.e(TAG, "Failed to load libnative_bridge.so", e)
        }

        try {
            System.loadLibrary("patch_getifaddrs")
            Log.i(TAG, "Successfully loaded libpatch_getifaddrs.so")
        } catch (e: UnsatisfiedLinkError) {
            Log.e(TAG, "Failed to load libpatch_getifaddrs.so", e)
        }

        createNotificationChannels()
    }

    private fun createNotificationChannels() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val channel = NotificationChannel(
                CHANNEL_ID,
                getString(R.string.service_notification_channel),
                NotificationManager.IMPORTANCE_LOW
            ).apply {
                description = getString(R.string.service_running_desc)
                setShowBadge(false)
            }
            val manager = getSystemService(NotificationManager::class.java)
            manager.createNotificationChannel(channel)
        }
    }

    companion object {
        const val TAG = "RosdroidApp"
        const val CHANNEL_ID = "rosdroid_core_service_channel"
        lateinit var instance: RosdroidApplication
            private set
    }
}
