package org.botforge.rosdroid.remote

import android.content.Context
import org.botforge.rosdroid.core.PRootManager

data class RemoteAccessConfig(
    val sshPort: Int = 2222,
    val sshUsername: String = "robot",
    val rosBridgePort: Int = 9090
)

class RemoteAccessManager(
    private val context: Context,
    private val prootManager: PRootManager
) {
    var config = RemoteAccessConfig()
        private set

    var isSshRunning: Boolean = false
        private set

    fun startSsh(): Boolean {
        val proc = prootManager.startSshServer(config.sshPort)
        isSshRunning = proc != null
        return isSshRunning
    }

    fun stopSsh() {
        prootManager.stopSshServer()
        isSshRunning = false
    }

    fun getSshCommand(deviceIp: String): String {
        return "ssh ${config.sshUsername}@$deviceIp -p ${config.sshPort}"
    }
}
