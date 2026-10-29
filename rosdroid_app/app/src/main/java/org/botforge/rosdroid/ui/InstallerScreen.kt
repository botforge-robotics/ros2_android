package org.botforge.rosdroid.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch
import org.botforge.rosdroid.core.InstallState
import org.botforge.rosdroid.core.RootfsInstaller

@Composable
fun InstallerScreen(
    installer: RootfsInstaller,
    onInstallComplete: () -> Unit
) {
    val scope = rememberCoroutineScope()
    var installState by remember { mutableStateOf<InstallState>(InstallState.Idle) }
    var isInstalled by remember { mutableStateOf(installer.isInstalled()) }
    val defaultUrl = "https://raw.githubusercontent.com/AndronixApp/AndronixOrigin/master/Rootfs/Ubuntu22/jammy-arm64.tar.gz"

    val scrollState = rememberScrollState()

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFF0A0F1D))
            .padding(20.dp)
            .verticalScroll(scrollState),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center
    ) {
        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = Color(0xFF1E293B)),
            shape = RoundedCornerShape(24.dp)
        ) {
            Column(
                modifier = Modifier.padding(24.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.spacedBy(16.dp)
            ) {
                // Header Icon with Neon Green Theme
                Box(
                    modifier = Modifier
                        .size(80.dp)
                        .background(
                            Color(0xFF00E676).copy(alpha = 0.15f),
                            shape = RoundedCornerShape(20.dp)
                        ),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        imageVector = if (isInstalled) Icons.Default.CheckCircle else Icons.Default.PrecisionManufacturing,
                        contentDescription = null,
                        tint = Color(0xFF00E676),
                        modifier = Modifier.size(48.dp)
                    )
                }

                Text(
                    text = if (isInstalled) "Rosdroid Ready!" else "Welcome to Rosdroid",
                    fontSize = 24.sp,
                    fontWeight = FontWeight.Bold,
                    color = Color.White
                )

                Text(
                    text = if (isInstalled) {
                        "Your Ubuntu Linux and ROS 2 Jazzy environment is initialized. You can launch your robot companion now."
                    } else {
                        "Convert this Android phone into a high-performance ROS 2 Robotics Companion Computer with onboard multi-camera vision, 200 Hz IMU fusion, micro-ROS, and remote web desktop."
                    },
                    fontSize = 14.sp,
                    color = Color(0xFF94A3B8),
                    lineHeight = 20.sp,
                    modifier = Modifier.padding(horizontal = 8.dp)
                )

                if (!isInstalled) {
                    // Feature summary checklist (all Green themed)
                    Column(
                        modifier = Modifier
                            .fillMaxWidth()
                            .background(Color(0xFF0F172A), RoundedCornerShape(14.dp))
                            .padding(14.dp),
                        verticalArrangement = Arrangement.spacedBy(10.dp)
                    ) {
                        SetupCheckItem(Icons.Default.Terminal, "Ubuntu Linux (PRoot rootless container)")
                        SetupCheckItem(Icons.Default.SmartToy, "ROS 2 Jazzy Core & CycloneDDS")
                        SetupCheckItem(Icons.Default.Usb, "Micro-ROS Agent (ESP32 USB Serial OTG)")
                        SetupCheckItem(Icons.Default.Sensors, "Sensor Hub (200Hz IMU, Cameras, Mic, GPS, Light)")
                        SetupCheckItem(Icons.Default.Fingerprint, "Biometric Face ID / Fingerprint Auth Services")
                        SetupCheckItem(Icons.Default.Computer, "Remote Terminal Access (SSH :2222)")
                    }

                    // Progress section
                    when (val state = installState) {
                        is InstallState.Downloading -> {
                            Column(modifier = Modifier.fillMaxWidth(), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                                Row(
                                    modifier = Modifier.fillMaxWidth(),
                                    horizontalArrangement = Arrangement.SpaceBetween
                                ) {
                                    Text("Downloading Linux System...", color = Color(0xFF00E676), fontSize = 13.sp)
                                    Text("${state.percent}%", color = Color(0xFF00E676), fontWeight = FontWeight.Bold, fontSize = 13.sp)
                                }
                                LinearProgressIndicator(
                                    progress = { state.percent / 100f },
                                    modifier = Modifier.fillMaxWidth().height(8.dp),
                                    color = Color(0xFF00E676),
                                    trackColor = Color(0xFF334155)
                                )
                            }
                        }
                        is InstallState.Extracting -> {
                            Column(modifier = Modifier.fillMaxWidth(), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                                Text(
                                    "Unpacking rootfs packages (${state.count} files)...",
                                    color = Color(0xFF00E676),
                                    fontSize = 13.sp
                                )
                                LinearProgressIndicator(
                                    modifier = Modifier.fillMaxWidth().height(8.dp),
                                    color = Color(0xFF00E676),
                                    trackColor = Color(0xFF334155)
                                )
                            }
                        }
                        is InstallState.Configuring -> {
                            Row(verticalAlignment = Alignment.CenterVertically) {
                                CircularProgressIndicator(modifier = Modifier.size(16.dp), color = Color(0xFF00E676), strokeWidth = 2.dp)
                                Spacer(modifier = Modifier.width(8.dp))
                                Text(state.message, color = Color(0xFF00E676), fontSize = 13.sp)
                            }
                        }
                        is InstallState.Error -> {
                            Row(
                                modifier = Modifier
                                    .fillMaxWidth()
                                    .background(Color(0xFF7F1D1D), RoundedCornerShape(8.dp))
                                    .padding(10.dp),
                                verticalAlignment = Alignment.CenterVertically
                            ) {
                                Icon(Icons.Default.Warning, contentDescription = null, tint = Color(0xFFEF4444))
                                Spacer(modifier = Modifier.width(8.dp))
                                Text("Error: ${state.error}", color = Color.White, fontSize = 12.sp)
                            }
                        }
                        InstallState.Completed -> {
                            isInstalled = true
                        }
                        InstallState.Idle -> {}
                    }

                    // 1-Click Install Button (Neon Green)
                    Button(
                        onClick = {
                            scope.launch {
                                installer.installAuto(defaultUrl).collectLatest { state ->
                                    installState = state
                                    if (state is InstallState.Completed) {
                                        isInstalled = true
                                        onInstallComplete()
                                    }
                                }
                            }
                        },
                        enabled = installState !is InstallState.Downloading && installState !is InstallState.Extracting,
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF00E676)),
                        shape = RoundedCornerShape(14.dp),
                        modifier = Modifier
                            .fillMaxWidth()
                            .height(52.dp)
                    ) {
                        Icon(Icons.Default.Download, contentDescription = null, tint = Color.Black)
                        Spacer(modifier = Modifier.width(8.dp))
                        Text(
                            text = if (installState is InstallState.Downloading || installState is InstallState.Extracting)
                                "Installing Everything..."
                            else "Install Everything & Setup",
                            color = Color.Black,
                            fontWeight = FontWeight.Bold,
                            fontSize = 16.sp
                        )
                    }
                } else {
                    // Ready to Launch Button
                    Button(
                        onClick = onInstallComplete,
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF00E676)),
                        shape = RoundedCornerShape(14.dp),
                        modifier = Modifier
                            .fillMaxWidth()
                            .height(52.dp)
                    ) {
                        Icon(Icons.Default.PlayArrow, contentDescription = null, tint = Color.Black)
                        Spacer(modifier = Modifier.width(8.dp))
                        Text("Launch Robot Dashboard", color = Color.Black, fontWeight = FontWeight.Bold, fontSize = 16.sp)
                    }
                }
            }
        }
    }
}

@Composable
private fun SetupCheckItem(icon: ImageVector, title: String) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Icon(icon, contentDescription = null, tint = Color(0xFF00E676), modifier = Modifier.size(18.dp))
        Spacer(modifier = Modifier.width(10.dp))
        Text(title, color = Color(0xFFE2E8F0), fontSize = 13.sp)
    }
}
