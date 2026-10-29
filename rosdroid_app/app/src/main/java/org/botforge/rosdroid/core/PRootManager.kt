package org.botforge.rosdroid.core

import android.content.Context
import android.system.Os
import android.util.Log
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.io.BufferedReader
import java.io.File
import java.io.InputStreamReader
import java.nio.file.Files

class PRootManager(private val context: Context) {

    val rootfsDir: File
        get() {
            val basePath = context.filesDir.absolutePath.replace("/data/user/0/", "/data/data/")
            return File(basePath, "rootfs/ubuntu")
        }
    private val prootBinary: File
        get() {
            // Check native lib directory first, then private bin
            val libProot = File(context.applicationInfo.nativeLibraryDir, "libproot.so")
            if (libProot.exists()) return libProot

            val binProot = File(context.filesDir, "bin/proot")
            if (binProot.exists()) return binProot

            return libProot
        }

    private var microRosProcess: Process? = null
    private var sshProcess: Process? = null
    private var rosNodeProcess: Process? = null

    private fun getGuestShell(): String {
        val bash2 = File(rootfsDir, "bin/bash")
        if (bash2.exists() && bash2.length() > 0) return "/bin/bash"
        val bash1 = File(rootfsDir, "usr/bin/bash")
        if (bash1.exists() && bash1.length() > 0) return "/usr/bin/bash"
        val sh1 = File(rootfsDir, "usr/bin/sh")
        if (sh1.exists()) return "/usr/bin/sh"
        return "/bin/sh"
    }

    /**
     * Builds a ProcessBuilder configured with PRoot virtualization and ROS 2 Jazzy environment.
     */
    fun buildProotProcess(command: String = "", domainId: Int = 156, sourceRos: Boolean = true, rawArgs: List<String>? = null): ProcessBuilder {
        val nativeDir = context.applicationInfo.nativeLibraryDir
        val filesBase = context.filesDir.absolutePath.replace("/data/user/0/", "/data/data/")
        val cacheBase = context.cacheDir.absolutePath.replace("/data/user/0/", "/data/data/")
        val libDir = File(filesBase, "lib").apply { mkdirs() }
        val talloc2 = File(libDir, "libtalloc.so.2")
        val nativeTalloc = File(nativeDir, "libtalloc.so")
        if (nativeTalloc.exists() && (!talloc2.exists() || !Files.isSymbolicLink(talloc2.toPath()))) {
            try {
                Files.deleteIfExists(talloc2.toPath())
                Os.symlink(nativeTalloc.absolutePath, talloc2.absolutePath)
            } catch (e: Exception) {
                try { nativeTalloc.copyTo(talloc2, overwrite = true) } catch (ignored: Exception) {}
            }
        }

        val cmd = mutableListOf<String>().apply {
            add(prootBinary.absolutePath)
            add("--link2symlink")
            add("-0") // Fake superuser (root)
            add("-r")
            add(rootfsDir.absolutePath)

            // Mount standard virtual filesystems
            add("-b")
            add("/dev")
            add("-b")
            add("/proc")
            add("-b")
            add("/sys")

            // Working directory
            add("-w")
            add("/root")

            if (rawArgs != null && rawArgs.isNotEmpty()) {
                addAll(rawArgs)
            } else {
                // Shell execution
                add(getGuestShell())
                add("-c")
                val preamble = if (sourceRos) {
                    "export PATH=/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:\$PATH " +
                    "HOME=/root USER=root HOSTNAME=rosdroid TERM=xterm-256color LANG=C.UTF-8 LC_ALL=C.UTF-8 " +
                    "PYTHONUTF8=1 PYTHONIOENCODING=utf-8 " +
                    "LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/root/rio_ws/install/rio_interfaces/lib:/home/robot/rosdroid_ws/install/test_pubsub/lib:\$LD_LIBRARY_PATH " +
                    "AMENT_PREFIX_PATH=/opt/ros/jazzy:/root/rio_ws/install/rio_interfaces:/home/robot/rosdroid_ws/install/test_pubsub:\$AMENT_PREFIX_PATH " +
                    "PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.10/dist-packages:/opt/ros/jazzy/lib/python3.10/site-packages:/root/rio_ws/install/rio_interfaces/local/lib/python3.10/dist-packages:/home/robot/rosdroid_ws/install/test_pubsub/local/lib/python3.10/dist-packages:\$PYTHONPATH " +
                    "ROS_DISTRO=jazzy ROS_VERSION=2 ROS_PYTHON_VERSION=3 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp " +
                    "CYCLONEDDS_URI=/etc/cyclonedds/cyclonedds.xml " +
                    "LD_PRELOAD=/usr/local/lib/patch_getifaddrs.so " +
                    "ROS_DOMAIN_ID=$domainId ROS_LOCALHOST_ONLY=0; "
                } else {
                    "export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:\$PATH " +
                    "HOME=/root USER=root HOSTNAME=rosdroid TERM=xterm-256color LANG=C.UTF-8 LC_ALL=C.UTF-8 " +
                    "PYTHONUTF8=1 PYTHONIOENCODING=utf-8; "
                }
                add(preamble + command)
            }
        }

        val dataDir = File("/data/data/${context.packageName}").let { if (it.exists()) it else context.filesDir }
        val prootArgs = cmd.joinToString(" ") { arg ->
            "'" + arg.replace("'", "'\\''") + "'"
        }
        val loaderPath = File(nativeDir, "libloader.so").absolutePath
        val shellScript = "export LD_LIBRARY_PATH=\"${libDir.absolutePath}:$nativeDir\"; " +
            "export PROOT_TMP_DIR=\"$cacheBase\"; " +
            "export PROOT_LOADER=\"$loaderPath\"; " +
            "export PROOT_NO_SECCOMP=1; " +
            "export HOME=/root; " +
            "export USER=root; " +
            "export SHELL=/bin/bash; " +
            "export TERM=xterm-256color; " +
            "export TMPDIR=\"$cacheBase\"; " +
            "exec $prootArgs"
        val pb = ProcessBuilder("/system/bin/sh", "-c", shellScript).directory(dataDir).redirectErrorStream(true)

        val hostPath = System.getenv("PATH") ?: "/product/bin:/apex/com.android.runtime/bin:/system/bin:/system/xbin"
        pb.environment()["PATH"] = hostPath
        pb.environment()["HOSTNAME"] = "rosdroid"
        pb.environment()["HOME"] = "/root"
        pb.environment()["USER"] = "root"
        pb.environment()["SHELL"] = "/bin/bash"
        pb.environment()["TERM"] = "xterm-256color"
        pb.environment()["TMPDIR"] = cacheBase
        pb.environment()["PYTHONUTF8"] = "1"
        pb.environment()["PYTHONIOENCODING"] = "utf-8"
        pb.environment()["LC_ALL"] = "C.UTF-8"
        pb.environment()["LANG"] = "C.UTF-8"
        pb.environment()["LD_LIBRARY_PATH"] = "${libDir.absolutePath}:$nativeDir"
        pb.environment()["PROOT_TMP_DIR"] = cacheBase
        pb.environment()["PROOT_LOADER"] = File(nativeDir, "libloader.so").absolutePath
        pb.environment()["PROOT_NO_SECCOMP"] = "1"

        return pb
    }

    /**
     * Executes an arbitrary ROS 2 command and reads its output line by line.
     */
    suspend fun runCommand(command: String, domainId: Int = 156, onLine: (String) -> Unit): Int = withContext(Dispatchers.IO) {
        try {
            if (!prootBinary.exists()) {
                onLine("PRoot binary initializing at ${prootBinary.absolutePath}...")
                val directProcess = ProcessBuilder("/system/bin/sh", "-c", command)
                    .directory(rootfsDir)
                    .redirectErrorStream(true)
                    .start()
                val reader = BufferedReader(InputStreamReader(directProcess.inputStream))
                var line: String?
                while (reader.readLine().also { line = it } != null) {
                    line?.let { onLine(it) }
                }
                return@withContext directProcess.waitFor()
            }

            val process = buildProotProcess(command, domainId).start()
            val reader = BufferedReader(InputStreamReader(process.inputStream))
            var line: String?
            var count = 0
            while (reader.readLine().also { line = it } != null) {
                val l = line ?: ""
                if (!l.startsWith("proot info:") && !l.startsWith("proot warning:")) {
                    onLine(l)
                    count++
                }
            }
            val exitCode = process.waitFor()
            if (count == 0 && exitCode != 0) {
                onLine("Process exited with code $exitCode")
            }
            exitCode
        } catch (e: Exception) {
            Log.e(TAG, "Command execution failed: $command", e)
            onLine("Error: ${e.message}")
            -1
        }
    }

    private fun drainStream(process: Process?, name: String) {
        if (process == null) return
        Thread({
            try {
                val reader = BufferedReader(InputStreamReader(process.inputStream))
                var line: String?
                while (reader.readLine().also { line = it } != null) {
                    val l = line ?: ""
                    if (!l.startsWith("proot info:") && !l.startsWith("proot warning:")) {
                        Log.i("PRoot-$name", l)
                    }
                }
            } catch (ignored: Exception) {}
        }, "PRoot-Drain-$name").apply {
            isDaemon = true
            start()
        }
    }

    /**
     * Starts the Micro-ROS Agent listening on localhost UDP.
     */
    fun startMicroRosAgent(port: Int = 8888, domainId: Int = 156): Process? {
        stopMicroRosAgent()
        return try {
            Log.i(TAG, "Starting Micro-ROS agent on UDP port $port (ROS 2 Jazzy)...")
            val args = listOf("/bin/bash", "/usr/local/bin/ros2", "run", "micro_ros_agent", "micro_ros_agent", "udp4", "--port", "$port", "-v4")
            microRosProcess = buildProotProcess(rawArgs = args, domainId = domainId).start()
            drainStream(microRosProcess, "MicroRos")
            microRosProcess
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start Micro-ROS agent", e)
            null
        }
    }

    fun stopMicroRosAgent() {
        microRosProcess?.destroy()
        microRosProcess = null
    }

    /**
     * Starts Dropbear or OpenSSH on port 2222.
     */
    fun startSshServer(port: Int = 2222): Process? {
        stopSshServer()
        return try {
            Log.i(TAG, "Starting SSH server on port $port...")
            val dropbearBin = if (File(rootfsDir, "usr/sbin/dropbear").exists()) "/usr/sbin/dropbear" else "dropbear"
            val args = listOf(dropbearBin, "-F", "-E", "-p", "$port", "-R")
            sshProcess = buildProotProcess(rawArgs = args, sourceRos = false).start()
            try { sshProcess?.outputStream?.close() } catch (ignored: Exception) {}
            drainStream(sshProcess, "SSH")
            sshProcess
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start SSH server", e)
            null
        }
    }

    fun stopSshServer() {
        sshProcess?.destroy()
        sshProcess = null
        try {
            buildProotProcess("pkill -f dropbear 2>/dev/null || true", sourceRos = false).start()
        } catch (ignored: Exception) {}
    }

    /**
     * Starts the Rosdroid ROS 2 Python Node bridging Actions, Services, and Publishers.
     */
    fun startRosNode(domainId: Int = 156): Process? {
        stopRosNode()
        return try {
            Log.i(TAG, "Starting Rosdroid ROS 2 Node (domainId=$domainId)...")
            val args = listOf("/bin/bash", "/usr/local/bin/rosdroid-node")
            rosNodeProcess = buildProotProcess(rawArgs = args, domainId = domainId, sourceRos = false).start()
            drainStream(rosNodeProcess, "RosNode")
            rosNodeProcess
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start Rosdroid ROS node", e)
            null
        }
    }

    fun stopRosNode() {
        rosNodeProcess?.destroy()
        rosNodeProcess = null
        try {
            buildProotProcess("pkill -f rosdroid_node.py 2>/dev/null || true", sourceRos = false).start()
        } catch (ignored: Exception) {}
    }

    fun stopAll() {
        stopMicroRosAgent()
        stopRosNode()
        stopSshServer()
    }

    companion object {
        private const val TAG = "PRootManager"
    }
}
