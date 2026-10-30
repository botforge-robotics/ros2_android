package org.botforge.rosdroid.core

import android.content.Context
import android.system.Os
import android.util.Log
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.emitAll
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.flow.flowOn
import org.apache.commons.compress.archivers.tar.TarArchiveInputStream
import java.io.File
import java.io.FileOutputStream
import java.io.InputStream
import java.net.HttpURLConnection
import java.net.URL
import java.nio.file.Files
import java.util.zip.GZIPInputStream

sealed class InstallState {
    object Idle : InstallState()
    data class Downloading(val bytesDownloaded: Long, val totalBytes: Long, val percent: Int) : InstallState()
    data class Extracting(val currentEntry: String, val count: Int) : InstallState()
    data class Configuring(val message: String) : InstallState()
    object Completed : InstallState()
    data class Error(val error: String) : InstallState()
}

class RootfsInstaller(private val context: Context) {

    val rootfsDir: File
        get() {
            val basePath = context.filesDir.absolutePath.replace("/data/user/0/", "/data/data/")
            return File(basePath, "rootfs/ubuntu")
        }
    private val archiveCacheFile: File
        get() {
            val cachePath = context.cacheDir.absolutePath.replace("/data/user/0/", "/data/data/")
            return File(cachePath, "rootfs_download.tar.gz")
        }

    fun isInstalled(): Boolean {
        val usrBash = File(rootfsDir, "usr/bin/bash")
        val binBash = File(rootfsDir, "bin/bash")
        return (usrBash.exists() && usrBash.length() > 0) || (binBash.exists() && binBash.length() > 0)
    }

    fun getLocalArchive(): File? {
        val candidates = mutableListOf<File>()
        try {
            val extDir = context.getExternalFilesDir(null)
            if (extDir != null) {
                candidates.add(File(extDir, "rosdroid_rootfs.tar"))
                candidates.add(File(extDir, "rosdroid_rootfs.tar.gz"))
            }
        } catch (ignored: Exception) {}
        candidates.add(File("/sdcard/Download/rosdroid_rootfs.tar"))
        candidates.add(File("/storage/emulated/0/Download/rosdroid_rootfs.tar"))
        candidates.add(File("/sdcard/rosdroid_rootfs.tar"))

        for (candidate in candidates) {
            try {
                if (candidate.exists() && candidate.canRead() && candidate.length() > 10_000_000) {
                    // Test actual read permission to avoid EACCES crash on scoped storage
                    candidate.inputStream().use { it.read() }
                    return candidate
                }
            } catch (e: Exception) {
                // Ignore candidates without read permission
            }
        }
        return null
    }

    /**
     * Auto installs from local archive on device if available, otherwise downloads from URL.
     */
    fun installAuto(downloadUrl: String): Flow<InstallState> = flow {
        val localArchive = try { getLocalArchive() } catch (e: Exception) { null }
        if (localArchive != null) {
            try {
                emit(InstallState.Extracting("Unpacking rootfs packages...", 0))
                if (rootfsDir.exists()) {
                    rootfsDir.deleteRecursively()
                }
                localArchive.inputStream().use { stream ->
                    extractArchive(stream) { entryName, count ->
                        emit(InstallState.Extracting(entryName, count))
                    }
                }
                emit(InstallState.Configuring("Configuring DNS, CycloneDDS, and Native Bridges..."))
                setupRootfsConfigurations()
                emit(InstallState.Completed)
                return@flow
            } catch (e: Exception) {
                Log.e(TAG, "Local archive extraction failed, falling back to URL download", e)
            }
        }
        emitAll(installFromUrl(downloadUrl))
    }.flowOn(Dispatchers.IO)

    /**
     * Installs rootfs from a remote URL or pre-packaged local archive.
     */
    fun installFromUrl(downloadUrl: String): Flow<InstallState> = flow {
        try {
            emit(InstallState.Downloading(0, 100, 0))

            // Step 1: Download
            val url = URL(downloadUrl)
            val connection = url.openConnection() as HttpURLConnection
            connection.connectTimeout = 30000
            connection.readTimeout = 30000
            connection.connect()

            val contentLength = connection.contentLengthLong

            connection.inputStream.use { input ->
                FileOutputStream(archiveCacheFile).use { output ->
                    val buffer = ByteArray(64 * 1024)
                    var totalRead: Long = 0
                    var read: Int
                    var lastPercent = 0

                    while (input.read(buffer).also { read = it } != -1) {
                        output.write(buffer, 0, read)
                        totalRead += read
                        val percent = if (contentLength > 0) ((totalRead * 100) / contentLength).toInt() else 0
                        if (percent != lastPercent) {
                            lastPercent = percent
                            emit(InstallState.Downloading(totalRead, contentLength, percent))
                        }
                    }
                }
            }

            // Clean previous incomplete rootfs if any
            if (rootfsDir.exists()) {
                rootfsDir.deleteRecursively()
            }

            // Step 2: Extract
            emit(InstallState.Extracting("Preparing filesystem...", 0))
            archiveCacheFile.inputStream().use { stream ->
                extractArchive(stream) { entryName, count ->
                    emit(InstallState.Extracting(entryName, count))
                }
            }

            // Step 3: Configure
            emit(InstallState.Configuring("Configuring DNS, CycloneDDS, and Native Bridges..."))
            setupRootfsConfigurations()

            // Clean up cache
            archiveCacheFile.delete()

            emit(InstallState.Completed)
        } catch (e: Exception) {
            Log.e(TAG, "Rootfs installation failed", e)
            emit(InstallState.Error(e.message ?: "Unknown installation error"))
        }
    }.flowOn(Dispatchers.IO)

    /**
     * Installs from local stream (e.g. assets).
     */
    fun installFromInputStream(inputStream: InputStream): Flow<InstallState> = flow {
        try {
            emit(InstallState.Extracting("Unpacking rootfs...", 0))
            extractArchive(inputStream) { entryName, count ->
                emit(InstallState.Extracting(entryName, count))
            }

            emit(InstallState.Configuring("Applying networking patches..."))
            setupRootfsConfigurations()

            emit(InstallState.Completed)
        } catch (e: Exception) {
            Log.e(TAG, "Rootfs extraction failed", e)
            emit(InstallState.Error(e.message ?: "Extraction failed"))
        }
    }.flowOn(Dispatchers.IO)

    private inline fun extractArchive(rawInput: InputStream, onProgress: (String, Int) -> Unit) {
        rootfsDir.mkdirs()

        val buffered = java.io.BufferedInputStream(rawInput)
        buffered.mark(4)
        val b1 = buffered.read()
        val b2 = buffered.read()
        buffered.reset()

        val tarInput = if (b1 == 0x1f && b2 == 0x8b) {
            TarArchiveInputStream(GZIPInputStream(buffered))
        } else {
            TarArchiveInputStream(buffered)
        }

        var entry = tarInput.nextTarEntry
        var count = 0

        while (entry != null) {
            val entryPath = if (entry.name.startsWith("ubuntu/")) {
                entry.name.removePrefix("ubuntu/")
            } else {
                entry.name
            }
            if (entryPath.isEmpty()) {
                entry = tarInput.nextTarEntry
                continue
            }

            val target = File(rootfsDir, entryPath)

            if (entry.isDirectory) {
                target.mkdirs()
            } else if (entry.isSymbolicLink) {
                target.parentFile?.mkdirs()
                try {
                    Files.deleteIfExists(target.toPath())
                    Os.symlink(entry.linkName, target.absolutePath)
                } catch (e: Exception) {
                    try {
                        target.delete()
                        Os.symlink(entry.linkName, target.absolutePath)
                    } catch (e2: Exception) {
                        Log.w(TAG, "Failed to create symlink: ${target.name} -> ${entry.linkName}", e2)
                    }
                }
            } else if (entry.isLink) {
                target.parentFile?.mkdirs()
                val src = File(rootfsDir, entry.linkName)
                try {
                    Files.deleteIfExists(target.toPath())
                    Os.link(src.absolutePath, target.absolutePath)
                } catch (e: Exception) {
                    try {
                        if (src.exists()) {
                            src.copyTo(target, overwrite = true)
                        }
                    } catch (ignored: Exception) {}
                }
            } else {
                target.parentFile?.mkdirs()
                try {
                    Files.deleteIfExists(target.toPath())
                } catch (ignored: Exception) {}

                FileOutputStream(target).use { output ->
                    tarInput.copyTo(output)
                }

                // Restore execute permissions
                if (entry.mode and 0b001001001 != 0 || entry.name.contains("bin/") || entry.name.endsWith(".sh")) {
                    target.setExecutable(true, false)
                }
            }

            count++
            if (count % 250 == 0) {
                onProgress(entry.name, count)
            }
            entry = tarInput.nextTarEntry
        }
    }

    fun setupRootfsConfigurations() {
        // Ensure parent directories and rootfs base directory are traversable for guest non-root users (like 'robot')
        try {
            context.filesDir.setReadable(true, false)
            context.filesDir.setExecutable(true, false)
            rootfsDir.parentFile?.setReadable(true, false)
            rootfsDir.parentFile?.setExecutable(true, false)
            rootfsDir.setReadable(true, false)
            rootfsDir.setExecutable(true, false)

            File(rootfsDir, "var").apply { setReadable(true, false); setExecutable(true, false) }
            File(rootfsDir, "run").apply { setReadable(true, false); setExecutable(true, false) }
            File(rootfsDir, "tmp").apply { setReadable(true, false); setExecutable(true, false); setWritable(true, false) }
        } catch (e: Exception) {
            Log.w(TAG, "Failed setting rootfs parent permissions", e)
        }

        // Ensure root symlinks: bin -> usr/bin, sbin -> usr/sbin, lib -> usr/lib
        ensureCoreSymlinks()

        val etcDir = File(rootfsDir, "etc").apply { mkdirs() }

        // Hostname & DNS
        File(etcDir, "hostname").writeText("rosdroid\n")
        File(etcDir, "resolv.conf").writeText("nameserver 8.8.8.8\nnameserver 1.1.1.1\n")
        File(etcDir, "hosts").writeText("127.0.0.1 localhost rosdroid\n::1 localhost ip6-localhost\n")

        // SELinux / restorecon stubs (prevents dpkg failure on Android)
        listOf("usr/sbin/restorecon", "sbin/restorecon", "usr/bin/restorecon").forEach { relPath ->
            val f = File(rootfsDir, relPath)
            f.parentFile?.mkdirs()
            f.delete()
            try { Os.symlink("/bin/true", f.absolutePath) } catch (e: Exception) {}
        }

        // Migrate /opt/ros/humble to /opt/ros/jazzy if present
        try {
            val humbleRos = File(rootfsDir, "opt/ros/humble")
            val jazzyRos = File(rootfsDir, "opt/ros/jazzy")
            if (humbleRos.exists() && !jazzyRos.exists()) {
                humbleRos.renameTo(jazzyRos)
            } else if (humbleRos.exists() && jazzyRos.exists()) {
                humbleRos.deleteRecursively()
            }
            if (jazzyRos.exists()) {
                listOf("local_setup.sh", "setup.sh", "local_setup.bash", "setup.bash").forEach { name ->
                    val f = File(jazzyRos, name)
                    if (f.exists()) {
                        val txt = f.readText()
                        if (txt.contains("humble")) {
                            f.writeText(txt.replace("/opt/ros/humble", "/opt/ros/jazzy"))
                        }
                    }
                }
                val envDir = File(jazzyRos, "share/ros_environment/environment")
                if (envDir.exists()) {
                    envDir.listFiles()?.forEach { f ->
                        if (f.isFile) {
                            val txt = f.readText()
                            if (txt.contains("humble")) {
                                f.writeText(txt.replace("humble", "jazzy"))
                            }
                        }
                    }
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "Notice: ROS Distro migration to jazzy: ${e.message}")
        }

        // Global ros2 executable wrapper in /usr/local/bin and /usr/bin
        val ros2Wrapper = File(rootfsDir, "usr/local/bin/ros2").apply { parentFile?.mkdirs() }
        ros2Wrapper.writeText(
            """
            #!/bin/bash
            export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${'$'}PATH
            export HOSTNAME=rosdroid
            export PYTHONUTF8=1
            export PYTHONIOENCODING=utf-8
            export LC_ALL=C.UTF-8
            export LANG=C.UTF-8
            export ROS_DISTRO=jazzy
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            export ROS_DOMAIN_ID=156
            export ROS_LOCALHOST_ONLY=0
            export CYCLONEDDS_URI=/etc/cyclonedds/cyclonedds.xml
            export LD_PRELOAD=/usr/local/lib/patch_getifaddrs.so
            export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/root/rio_ws/install/rio_interfaces/lib:/home/robot/rosdroid_ws/install/test_pubsub/lib:${'$'}{LD_LIBRARY_PATH:-}
            export AMENT_PREFIX_PATH=/opt/ros/jazzy:/root/rio_ws/install/rio_interfaces:/home/robot/rosdroid_ws/install/test_pubsub:${'$'}{AMENT_PREFIX_PATH:-}
            export PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.10/dist-packages:/opt/ros/jazzy/lib/python3.10/site-packages:/root/rio_ws/install/rio_interfaces/local/lib/python3.10/dist-packages:/home/robot/rosdroid_ws/install/test_pubsub/local/lib/python3.10/dist-packages
            if [ -n "${'$'}{PYTHONPATH_EXTRA:-}" ]; then
                export PYTHONPATH="${'$'}PYTHONPATH:${'$'}PYTHONPATH_EXTRA"
            fi
            source /opt/ros/jazzy/setup.bash 2>/dev/null || true
            source /root/rio_ws/install/setup.bash 2>/dev/null || true
            source /home/robot/rosdroid_ws/install/setup.bash 2>/dev/null || true
            exec /opt/ros/jazzy/bin/ros2 "${'$'}@"
            """.trimIndent() + "\n"
        )
        ros2Wrapper.setReadable(true, false)
        ros2Wrapper.setExecutable(true, false)

        val usrBinRos2 = File(rootfsDir, "usr/bin/ros2")
        usrBinRos2.delete()
        try { Os.symlink("/usr/local/bin/ros2", usrBinRos2.absolutePath) } catch (e: Exception) {}

        // Global rosdroid-node daemon executable in /usr/local/bin and /usr/bin
        val rosdroidNodeWrapper = File(rootfsDir, "usr/local/bin/rosdroid-node").apply { parentFile?.mkdirs() }
        rosdroidNodeWrapper.writeText(
            """
            #!/bin/bash
            export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${'$'}PATH
            export HOSTNAME=rosdroid
            export PYTHONUTF8=1
            export PYTHONIOENCODING=utf-8
            export LC_ALL=C.UTF-8
            export LANG=C.UTF-8
            export ROS_DISTRO=jazzy
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            export ROS_DOMAIN_ID=156
            export ROS_LOCALHOST_ONLY=0
            export CYCLONEDDS_URI=/etc/cyclonedds/cyclonedds.xml
            export LD_PRELOAD=/usr/local/lib/patch_getifaddrs.so
            export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/root/rio_ws/install/rio_interfaces/lib:/home/robot/rosdroid_ws/install/test_pubsub/lib:${'$'}{LD_LIBRARY_PATH:-}
            export AMENT_PREFIX_PATH=/opt/ros/jazzy:/root/rio_ws/install/rio_interfaces:/home/robot/rosdroid_ws/install/test_pubsub:${'$'}{AMENT_PREFIX_PATH:-}
            export PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.10/dist-packages:/opt/ros/jazzy/lib/python3.10/site-packages:/root/rio_ws/install/rio_interfaces/local/lib/python3.10/dist-packages:/home/robot/rosdroid_ws/install/test_pubsub/local/lib/python3.10/dist-packages
            source /opt/ros/jazzy/setup.bash 2>/dev/null || true
            source /root/rio_ws/install/setup.bash 2>/dev/null || true
            source /home/robot/rosdroid_ws/install/setup.bash 2>/dev/null || true
            if [ -f /opt/rosdroid/rosdroid_node.py ]; then
                exec /usr/bin/python3 /opt/rosdroid/rosdroid_node.py "${'$'}@"
            elif [ -f /root/rosdroid_node.py ]; then
                exec /usr/bin/python3 /root/rosdroid_node.py "${'$'}@"
            fi
            """.trimIndent() + "\n"
        )
        rosdroidNodeWrapper.setReadable(true, false)
        rosdroidNodeWrapper.setExecutable(true, false)

        val usrBinNode = File(rootfsDir, "usr/bin/rosdroid-node")
        usrBinNode.delete()
        try { Os.symlink("/usr/local/bin/rosdroid-node", usrBinNode.absolutePath) } catch (e: Exception) {}

        // Sudo wrapper in /usr/local/bin and /usr/bin (switches to fake-root UID 0 in PRoot)
        val sudoWrapper = File(rootfsDir, "usr/local/bin/sudo").apply { parentFile?.mkdirs() }
        sudoWrapper.writeText(
            """
            #!/bin/bash
            if [ "${'$'}1" = "-u" ]; then
                shift 2
            fi
            if [ ${'$'}# -eq 0 ]; then
                exit 0
            fi
            export USER=root
            export HOME=/root
            export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${'$'}PATH
            exec "${'$'}@"
            """.trimIndent() + "\n"
        )
        sudoWrapper.setReadable(true, false)
        sudoWrapper.setExecutable(true, false)

        val usrBinSudo = File(rootfsDir, "usr/bin/sudo")
        if (!usrBinSudo.exists() || usrBinSudo.length() == 0L) {
            usrBinSudo.delete()
            try { Os.symlink("/usr/local/bin/sudo", usrBinSudo.absolutePath) } catch (e: Exception) {}
        }

        // Global bashrc prompt configuration
        val etcBashrc = File(etcDir, "bash.bashrc")
        if (etcBashrc.exists()) {
            val content = etcBashrc.readText()
            if (!content.contains("PS1='\\u@rosdroid:")) {
                etcBashrc.appendText("\nexport PS1='\\u@rosdroid:\\w\\$ '\nexport HOSTNAME=rosdroid\n")
            }
            etcBashrc.setReadable(true, false)
        }

        // Environment variables
        val envFile = File(etcDir, "environment")
        envFile.writeText(
            """
            PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
            HOSTNAME=rosdroid
            ROS_DISTRO=jazzy
            ROS_DOMAIN_ID=156
            RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            """.trimIndent() + "\n"
        )
        envFile.setReadable(true, false)

        // Preload patch_getifaddrs globally for system-wide getifaddrs and hostname/uname hooks
        val ldSoPreload = File(etcDir, "ld.so.preload")
        ldSoPreload.writeText("/usr/local/lib/patch_getifaddrs.so\n")
        ldSoPreload.setReadable(true, false)

        val shellInit = """
            export PATH=/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${'$'}PATH
            export HOSTNAME=rosdroid
            export ROS_DISTRO=jazzy
            export ROS_VERSION=2
            export ROS_PYTHON_VERSION=3
            export ROS_DOMAIN_ID=156
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            export CYCLONEDDS_URI=/etc/cyclonedds/cyclonedds.xml
            export LD_PRELOAD=/usr/local/lib/patch_getifaddrs.so
            export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/root/rio_ws/install/rio_interfaces/lib:/home/robot/rosdroid_ws/install/test_pubsub/lib:${'$'}{LD_LIBRARY_PATH:-}
            export AMENT_PREFIX_PATH=/opt/ros/jazzy:/root/rio_ws/install/rio_interfaces:/home/robot/rosdroid_ws/install/test_pubsub:${'$'}{AMENT_PREFIX_PATH:-}
            export PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.10/dist-packages:/opt/ros/jazzy/lib/python3.10/site-packages:/root/rio_ws/install/rio_interfaces/local/lib/python3.10/dist-packages:/home/robot/rosdroid_ws/install/test_pubsub/local/lib/python3.10/dist-packages:${'$'}{PYTHONPATH:-}
            source /opt/ros/jazzy/setup.bash 2>/dev/null || true
            source /root/rio_ws/install/setup.bash 2>/dev/null || true
            source /home/robot/rosdroid_ws/install/setup.bash 2>/dev/null || true
            export PS1='${'$'}{debian_chroot:+(${'$'}debian_chroot)}\u@rosdroid:\w\$ '
        """.trimIndent() + "\n"

        // Symlink Cyclone DDS multiarch libraries directly into /opt/ros/jazzy/lib
        try {
            val jazzyLib = File(rootfsDir, "opt/ros/jazzy/lib")
            val jazzyArchLib = File(jazzyLib, "aarch64-linux-gnu")
            if (jazzyArchLib.exists()) {
                jazzyArchLib.listFiles()?.forEach { libFile ->
                    if (libFile.name.endsWith(".so") || libFile.name.contains(".so.")) {
                        val target = File(jazzyLib, libFile.name)
                        if (!target.exists()) {
                            try { Os.symlink(libFile.absolutePath, target.absolutePath) } catch (ignored: Exception) {}
                        }
                    }
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "Notice: Cyclone DDS multiarch symlink: ${e.message}")
        }

        val profileInit = File(etcDir, "profile.d/rosdroid.sh").apply { parentFile?.mkdirs() }
        profileInit.writeText(shellInit)
        profileInit.setReadable(true, false)
        profileInit.setExecutable(true, false)

        val rootBashrc = File(rootfsDir, "root/.bashrc").apply { parentFile?.mkdirs() }
        if (rootBashrc.exists()) {
            val oldTxt = rootBashrc.readText()
            if (oldTxt.contains("ROS_DISTRO=humble")) {
                rootBashrc.writeText(oldTxt.replace("humble", "jazzy"))
            } else if (!oldTxt.contains("ROS_DISTRO=jazzy")) {
                rootBashrc.appendText("\n" + shellInit)
            }
        } else {
            rootBashrc.writeText(shellInit)
        }
        rootBashrc.setReadable(true, false)

        fun setupUserBashrc(homePath: String) {
            val userBashrc = File(rootfsDir, "$homePath/.bashrc").apply { parentFile?.mkdirs() }
            if (userBashrc.exists()) {
                val oldTxt = userBashrc.readText()
                if (oldTxt.contains("ROS_DISTRO=humble")) {
                    userBashrc.writeText(oldTxt.replace("humble", "jazzy"))
                } else if (!oldTxt.contains("ROS_DISTRO=jazzy")) {
                    userBashrc.appendText("\n" + shellInit)
                }
            } else {
                userBashrc.writeText(shellInit)
            }
            userBashrc.setReadable(true, false)
        }

        setupUserBashrc("home/rosdroid")
        setupUserBashrc("home/robot")

        // Ensure permissions on core /etc files
        File(etcDir, "passwd").setReadable(true, false)
        File(etcDir, "group").setReadable(true, false)
        File(etcDir, "shadow").setReadable(true, false)
        File(etcDir, "hosts").setReadable(true, false)
        File(etcDir, "hostname").setReadable(true, false)
        File(etcDir, "nsswitch.conf").setReadable(true, false)

        // Configure Dropbear directory, users, and home permissions
        File(etcDir, "dropbear").apply { mkdirs() }
        setupDropbearUsers(etcDir)

        try {
            val homeDir = File(rootfsDir, "home").apply { mkdirs() }
            homeDir.setReadable(true, false)
            homeDir.setWritable(true, true)
            homeDir.setExecutable(true, false)

            val rosdroidHome = File(homeDir, "rosdroid").apply { mkdirs() }
            rosdroidHome.setReadable(true, false)
            rosdroidHome.setWritable(true, true)
            rosdroidHome.setExecutable(true, false)
            File(rosdroidHome, ".bashrc").setReadable(true, false)
            File(rosdroidHome, ".profile").setReadable(true, false)
            val rosdroidBashProfile = File(rosdroidHome, ".bash_profile")
            rosdroidBashProfile.writeText("[ -f ~/.bashrc ] && . ~/.bashrc\n")
            rosdroidBashProfile.setReadable(true, false)

            val robotHome = File(homeDir, "robot")
            if (!robotHome.exists()) {
                try {
                    Os.symlink("rosdroid", robotHome.absolutePath)
                } catch (e: Exception) {
                    robotHome.mkdirs()
                    robotHome.setReadable(true, false)
                    robotHome.setWritable(true, true)
                    robotHome.setExecutable(true, false)
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "Failed setting home permissions", e)
        }

        // Global profile prompt configuration
        val etcProfile = File(etcDir, "profile")
        if (etcProfile.exists()) {
            val content = etcProfile.readText()
            if (!content.contains("rosdroid")) {
                etcProfile.appendText("\nexport PS1='\\u@rosdroid:\\w\\$ '\nexport HOSTNAME=rosdroid\n")
            }
            etcProfile.setReadable(true, false)
        }

        // Repair any .egg-info single-file packages (such as notify2, empy) to directories
        // so Python 3.10 importlib.metadata doesn't hit ENOSYS on Android PRoot
        try {
            val distPackages = File(rootfsDir, "usr/lib/python3/dist-packages")
            if (distPackages.exists()) {
                distPackages.listFiles()?.forEach { f ->
                    if (f.isFile && f.name.endsWith(".egg-info")) {
                        val tmp = File(distPackages, f.name + ".tmp_fix")
                        if (f.renameTo(tmp)) {
                            f.mkdirs()
                            tmp.renameTo(File(f, "PKG-INFO"))
                            f.setReadable(true, false)
                            f.setExecutable(true, false)
                            File(f, "PKG-INFO").setReadable(true, false)
                        }
                    }
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "Egg-info fix notice: ${e.message}")
        }

        // Deploy rosdroid_node.py from assets to both /opt/rosdroid and /root
        try {
            val optDir = File(rootfsDir, "opt/rosdroid").apply { mkdirs(); setReadable(true, false); setExecutable(true, false) }
            val optNode = File(optDir, "rosdroid_node.py")
            context.assets.open("rosdroid_node.py").use { input ->
                FileOutputStream(optNode).use { output ->
                    input.copyTo(output)
                }
            }
            optNode.setReadable(true, false)
            optNode.setExecutable(true, false)

            val rootNode = File(rootfsDir, "root/rosdroid_node.py")
            optNode.copyTo(rootNode, overwrite = true)
            rootNode.setReadable(true, false)
            rootNode.setExecutable(true, false)

            val rootDir = File(rootfsDir, "root").apply { setReadable(true, false); setExecutable(true, false) }
            val rioWs = File(rootDir, "rio_ws")
            if (rioWs.exists()) {
                rioWs.walkTopDown().forEach { f ->
                    f.setReadable(true, false)
                    if (f.isDirectory) f.setExecutable(true, false)
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "Notice: rosdroid_node.py asset copy: ${e.message}")
        }

        // Patch ROS 2 CLI strategy to disable daemon in PRoot and avoid empty node name faults
        try {
            val strategyFiles = listOf(
                File(rootfsDir, "opt/ros/jazzy/lib/python3.10/site-packages/ros2cli/node/strategy.py"),
                File(rootfsDir, "opt/ros/jazzy/lib/python3.12/site-packages/ros2cli/node/strategy.py")
            )
            for (strategyFile in strategyFiles) {
                if (strategyFile.exists()) {
                    val sContent = strategyFile.readText()
                    if (sContent.contains("use_daemon = not getattr(args, 'no_daemon', False)")) {
                        strategyFile.writeText(sContent.replace("use_daemon = not getattr(args, 'no_daemon', False)", "use_daemon = False"))
                    }
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "Notice patching ROS 2 CLI strategy: ${e.message}")
        }

        // Fast-path accelerator for ROS 2 CLI graph introspection (topic list, node list, service list)
        // Bypasses heavy ros2cli plugin reflection overhead in PRoot, reducing latency from ~5.2s down to ~0.9s
        try {
            val ros2Bin = File(rootfsDir, "opt/ros/jazzy/bin/ros2")
            if (ros2Bin.exists()) {
                val current = ros2Bin.readText()
                if (!current.contains("Ultra-fast path for graph introspection")) {
                    val fastRos2Script = """
#!/usr/bin/python3
import sys
import os

# Ultra-fast path for graph introspection (topic list, node list, service list)
# Bypasses heavy ros2cli plugin reflection, cuts latency from ~5.2s down to ~0.9s
args = sys.argv
if len(args) >= 3 and '-h' not in args and '--help' not in args:
    verb_group = args[1]
    verb_action = args[2]

    if verb_group in ('topic', 'node', 'service') and verb_action == 'list':
        import time
        from rclpy._rclpy_pybind11 import (
            Context as CContext,
            Node as CNode,
            rclpy_get_topic_names_and_types,
            rclpy_get_service_names_and_types
        )

        domain_id = int(os.environ.get('ROS_DOMAIN_ID', 156))
        ctx = CContext([], domain_id)
        node = CNode('_fast_ros2_cli', '', ctx, [], True, False)

        if verb_group == 'topic':
            include_hidden = '--include-hidden-topics' in args
            show_types = '-t' in args or '--show-types' in args
            count_only = '-c' in args or '--count-topics' in args

            raw_topics = []
            for _ in range(25):
                raw_topics = rclpy_get_topic_names_and_types(node, False)
                if len(raw_topics) >= 5:
                    break
                time.sleep(0.04)

            filtered = []
            for name, types in raw_topics:
                if not include_hidden and (name.startswith('/_') or '/_' in name):
                    continue
                filtered.append((name, types))
            filtered.sort(key=lambda x: x[0])

            if count_only:
                print(len(filtered))
            else:
                for name, types in filtered:
                    if show_types:
                        print(f"{name} [{', '.join(types)}]")
                    else:
                        print(name)

        elif verb_group == 'node':
            include_hidden = '--include-hidden-nodes' in args or '-a' in args or '--all' in args
            count_only = '-c' in args or '--count-nodes' in args

            raw_nodes = []
            for _ in range(25):
                raw_nodes = node.get_node_names_and_namespaces()
                if len(raw_nodes) >= 2:
                    break
                time.sleep(0.04)

            filtered = []
            for name, ns in raw_nodes:
                if not include_hidden and name.startswith('_'):
                    continue
                full = (ns.rstrip('/') + '/' + name) if ns else ('/' + name)
                filtered.append(full)
            filtered.sort()

            if count_only:
                print(len(filtered))
            else:
                for n in filtered:
                    print(n)

        elif verb_group == 'service':
            include_hidden = '--include-hidden-services' in args
            show_types = '-t' in args or '--show-types' in args
            count_only = '-c' in args or '--count-services' in args

            raw_services = []
            for _ in range(25):
                raw_services = rclpy_get_service_names_and_types(node)
                if len(raw_services) >= 3:
                    break
                time.sleep(0.04)

            filtered = []
            for name, types in raw_services:
                if not include_hidden and (name.startswith('/_') or '/_' in name):
                    continue
                filtered.append((name, types))
            filtered.sort(key=lambda x: x[0])

            if count_only:
                print(len(filtered))
            else:
                for name, types in filtered:
                    if show_types:
                        print(f"{name} [{', '.join(types)}]")
                    else:
                        print(name)

        sys.stdout.flush()
        os._exit(0)

# Fallback to standard ros2cli entry point for all other commands
import re
__requires__ = 'ros2cli==0.18.20'
try:
    from importlib.metadata import distribution
except ImportError:
    try:
        from importlib_metadata import distribution
    except ImportError:
        from pkg_resources import load_entry_point

def importlib_load_entry_point(spec, group, name):
    dist_name, _, _ = spec.partition('==')
    matches = (
        entry_point
        for entry_point in distribution(dist_name).entry_points
        if entry_point.group == group and entry_point.name == name
    )
    return next(matches).load()

globals().setdefault('load_entry_point', importlib_load_entry_point)

if __name__ == '__main__':
    sys.argv[0] = re.sub(r'(-script\.pyw?|\.exe)?$', '', sys.argv[0])
    sys.exit(load_entry_point('ros2cli==0.18.20', 'console_scripts', 'ros2')())
""".trimIndent()
                    ros2Bin.writeText(fastRos2Script)
                    ros2Bin.setExecutable(true, false)
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "Notice setting up fast ros2 CLI: ${e.message}")
        }

        // Setup cyclonedds config and ifaddrs list
        val cycloneddsDir = File(rootfsDir, "etc/cyclonedds").apply { mkdirs(); setReadable(true, false) }
        val cycloneddsFile = File(cycloneddsDir, "cyclonedds.xml")
        cycloneddsFile.writeText(
            """
            <?xml version="1.0" encoding="UTF-8" ?>
            <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
                <Domain id="any">
                    <General>
                        <Interfaces>
                            <NetworkInterface name="wlan0"/>
                        </Interfaces>
                        <AllowMulticast>true</AllowMulticast>
                    </General>
                    <Discovery>
                        <ParticipantIndex>auto</ParticipantIndex>
                        <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
                        <Peers>
                            <Peer address="239.255.0.1"/>
                            <Peer address="127.0.0.1"/>
                            <Peer address="255.255.255.255"/>
                            <Peer address="192.168.0.255"/>
                            <Peer address="192.168.0.101"/>
                        </Peers>
                    </Discovery>
                </Domain>
            </CycloneDDS>
            """.trimIndent() + "\n"
        )
        cycloneddsFile.setReadable(true, false)

        try {
            val ifaddrsFile = File(cycloneddsDir, "ifaddrs.txt")
            val ifaceSb = StringBuilder()
            ifaceSb.append("lo 127.0.0.1 255.0.0.0\n")
            try {
                val ifaces = java.net.NetworkInterface.getNetworkInterfaces()
                while (ifaces.hasMoreElements()) {
                    val iface = ifaces.nextElement()
                    if (iface.isUp && !iface.isLoopback) {
                        for (addr in iface.inetAddresses) {
                            if (addr is java.net.Inet4Address && !addr.isLoopbackAddress) {
                                ifaceSb.append("${iface.name} ${addr.hostAddress} 255.255.255.0\n")
                            }
                        }
                    }
                }
            } catch (ignored: Exception) {}
            if (ifaceSb.indexOf("wlan0") == -1) {
                ifaceSb.append("wlan0 192.168.0.152 255.255.255.0\n")
            }
            ifaddrsFile.writeText(ifaceSb.toString())
            ifaddrsFile.setReadable(true, false)
        } catch (e: Exception) {
            Log.w(TAG, "Failed writing ifaddrs.txt", e)
        }

        // Install patch_getifaddrs library into guest rootfs
        val localLibDir = File(rootfsDir, "usr/local/lib").apply { mkdirs(); setReadable(true, false); setExecutable(true, false) }
        val destLib = File(localLibDir, "patch_getifaddrs.so")
        try {
            if (context.assets.list("")?.contains("patch_getifaddrs.so") == true) {
                context.assets.open("patch_getifaddrs.so").use { input ->
                    FileOutputStream(destLib).use { output ->
                        input.copyTo(output)
                    }
                }
            } else {
                val patchLib = File(context.applicationInfo.nativeLibraryDir, "libpatch_getifaddrs.so")
                if (patchLib.exists()) {
                    patchLib.copyTo(destLib, overwrite = true)
                }
            }
            destLib.setReadable(true, false)
            destLib.setExecutable(true, false)
        } catch (e: Exception) {
            Log.w(TAG, "Failed to copy patch_getifaddrs: ${e.message}")
        }

        // Install app private helper libtalloc symlink for proot
        val libDir = File(context.filesDir, "lib").apply { mkdirs() }
        val talloc2 = File(libDir, "libtalloc.so.2")
        val nativeTalloc = File(context.applicationInfo.nativeLibraryDir, "libtalloc.so")
        if (nativeTalloc.exists() && (!talloc2.exists() || !Files.isSymbolicLink(talloc2.toPath()))) {
            try {
                Files.deleteIfExists(talloc2.toPath())
                Os.symlink(nativeTalloc.absolutePath, talloc2.absolutePath)
            } catch (e: Exception) {
                try { nativeTalloc.copyTo(talloc2, overwrite = true) } catch (ignored: Exception) {}
            }
        }

        // GCC assembler/linker stubs and colcon/ament indexing directories
        try {
            val gccDirs = listOf("usr/lib/gcc/aarch64-linux-gnu/11", "usr/lib/gcc/aarch64-linux-gnu/12", "usr/lib/gcc/aarch64-linux-gnu/13", "usr/lib/gcc/aarch64-linux-gnu/14")
            for (relDir in gccDirs) {
                val dir = File(rootfsDir, relDir)
                if (dir.exists()) {
                    val asFile = File(dir, "as")
                    if (!asFile.exists()) {
                        try { Os.symlink("/usr/bin/as", asFile.absolutePath) } catch (ignored: Exception) {}
                    }
                    val ldFile = File(dir, "ld")
                    if (!ldFile.exists()) {
                        try { Os.symlink("/usr/bin/ld", ldFile.absolutePath) } catch (ignored: Exception) {}
                    }
                }
            }
            File(rootfsDir, "root/rio_ws/install/share/ament_index/resource_index/packages").mkdirs()
            val colconDir = File(rootfsDir, "home/robot/.colcon").apply { mkdirs(); setReadable(true, false); setWritable(true, true); setExecutable(true, false) }
            val defaultsYaml = File(colconDir, "defaults.yaml")
            if (!defaultsYaml.exists()) {
                defaultsYaml.writeText("{}\n")
                defaultsYaml.setReadable(true, false)
                defaultsYaml.setWritable(true, true)
            }
        } catch (e: Exception) {
            Log.w(TAG, "Notice: toolchain stubs setup: ${e.message}")
        }
    }

    private fun setupDropbearUsers(etcDir: File) {
        try {
            val passwdFile = File(etcDir, "passwd")
            val shadowFile = File(etcDir, "shadow")

            if (passwdFile.exists()) {
                val passwdContent = passwdFile.readText()
                if (!passwdContent.contains("rosdroid:")) {
                    passwdFile.appendText("rosdroid:x:1000:1000:Rosdroid User:/home/rosdroid:/bin/bash\n")
                }
                if (!passwdContent.contains("robot:")) {
                    passwdFile.appendText("robot:x:1001:1000:Rosdroid Robot:/home/rosdroid:/bin/bash\n")
                }
                passwdFile.setReadable(true, false)
            }

            // Valid crypt(3) SHA-512 password hash for 'rosdroid'
            val hashToUse = "${'$'}6${'$'}rosdroid${'$'}k6LXMXt6hTPGIjY9wDpgdCT02bVYo0HE7c.wXnarxPe4C1yw7oqbZ.70SV3oAb.4g2/8LFrsZilzrS1OycaVP/"

            if (shadowFile.exists()) {
                val shadowLines = shadowFile.readLines().toMutableList()
                val newShadow = shadowLines.map { line ->
                    if (line.startsWith("root:")) "root:$hashToUse:20754:0:99999:7:::"
                    else if (line.startsWith("rosdroid:")) "rosdroid:$hashToUse:20754:0:99999:7:::"
                    else if (line.startsWith("robot:")) "robot:$hashToUse:20754:0:99999:7:::"
                    else line
                }.toMutableList()

                if (newShadow.none { it.startsWith("rosdroid:") }) {
                    newShadow.add("rosdroid:$hashToUse:20754:0:99999:7:::")
                }
                if (newShadow.none { it.startsWith("robot:") }) {
                    newShadow.add("robot:$hashToUse:20754:0:99999:7:::")
                }
                shadowFile.writeText(newShadow.joinToString("\n") + "\n")
            } else {
                shadowFile.writeText(
                    "root:$hashToUse:20754:0:99999:7:::\n" +
                    "daemon:*:20754:0:99999:7:::\n" +
                    "rosdroid:$hashToUse:20754:0:99999:7:::\n" +
                    "robot:$hashToUse:20754:0:99999:7:::\n"
                )
            }
            shadowFile.setReadable(true, false)

            // Sudoers
            val sudoersDir = File(etcDir, "sudoers.d").apply { mkdirs() }
            File(sudoersDir, "rosdroid").writeText("rosdroid ALL=(ALL:ALL) NOPASSWD: ALL\n")
            File(sudoersDir, "rosdroid").setReadable(true, false)
            File(sudoersDir, "robot").writeText("robot ALL=(ALL:ALL) NOPASSWD: ALL\n")
            File(sudoersDir, "robot").setReadable(true, false)
        } catch (e: Exception) {
            Log.w(TAG, "Failed to auto-configure Dropbear users", e)
        }
    }

    private fun ensureCoreSymlinks() {
        val bin = File(rootfsDir, "bin")
        if (!bin.exists() || !Files.isSymbolicLink(bin.toPath())) {
            bin.delete()
            try { Os.symlink("usr/bin", bin.absolutePath) } catch (e: Exception) {}
        }
        val sbin = File(rootfsDir, "sbin")
        if (!sbin.exists() || !Files.isSymbolicLink(sbin.toPath())) {
            sbin.delete()
            try { Os.symlink("usr/sbin", sbin.absolutePath) } catch (e: Exception) {}
        }
        val lib = File(rootfsDir, "lib")
        if (!lib.exists() || !Files.isSymbolicLink(lib.toPath())) {
            lib.delete()
            try { Os.symlink("usr/lib", lib.absolutePath) } catch (e: Exception) {}
        }

        // Ensure ld-linux interpreter symlink
        val ldLinux = File(rootfsDir, "usr/lib/ld-linux-aarch64.so.1")
        if (!ldLinux.exists() || ldLinux.length() == 0L) {
            ldLinux.delete()
            try { Os.symlink("aarch64-linux-gnu/ld-linux-aarch64.so.1", ldLinux.absolutePath) } catch (e: Exception) {}
        }

        // Repair any 0-byte shared libraries in usr/lib/aarch64-linux-gnu
        val gnuLibDir = File(rootfsDir, "usr/lib/aarch64-linux-gnu")
        if (gnuLibDir.exists() && gnuLibDir.isDirectory) {
            gnuLibDir.listFiles()?.forEach { file ->
                if (file.isFile && file.length() == 0L && !Files.isSymbolicLink(file.toPath())) {
                    val name = file.name
                    val target = gnuLibDir.listFiles()?.firstOrNull { it.name.startsWith("$name.") && it.length() > 0 }
                    if (target != null) {
                        file.delete()
                        try {
                            Os.symlink(target.name, file.absolutePath)
                        } catch (e: Exception) {
                            Log.w(TAG, "Repair symlink failed for $name", e)
                        }
                    }
                }
            }
        }
    }

    companion object {
        private const val TAG = "RootfsInstaller"
    }
}
