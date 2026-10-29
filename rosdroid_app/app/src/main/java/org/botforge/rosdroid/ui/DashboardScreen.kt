package org.botforge.rosdroid.ui

import android.content.ClipData
import android.content.ClipboardManager
import android.content.Context
import android.content.Intent
import android.net.Uri
import android.widget.Toast
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
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import org.botforge.rosdroid.core.SystemStatus
import org.botforge.rosdroid.hardware.UsbConnectionState
import org.botforge.rosdroid.security.BiometricState
import org.botforge.rosdroid.sensors.EnvironmentalData
import org.botforge.rosdroid.sensors.GpsData
import org.botforge.rosdroid.sensors.ImuData
import org.botforge.rosdroid.sensors.SensorStreamConfig

@Composable
fun DashboardScreen(
    status: SystemStatus,
    usbState: UsbConnectionState,
    imuData: ImuData,
    envData: EnvironmentalData = EnvironmentalData(),
    gpsData: GpsData = GpsData(),
    streamConfig: SensorStreamConfig = SensorStreamConfig(),
    speechText: String = "",
    isSpeechListening: Boolean = true,
    biometricState: BiometricState = BiometricState.Idle,
    isFrontCamera: Boolean = false,
    cameraFps: Int = 30,
    isTorchOn: Boolean = false,
    onToggleTorch: () -> Unit = {},
    onSwitchCamera: () -> Unit = {},
    onToggleSensor: (sensorName: String, enabled: Boolean) -> Unit = { _, _ -> },
    onTestSpeak: (String) -> Unit = {},
    onTriggerBiometrics: () -> Unit = {},
    onDomainChange: (Int) -> Unit = {}
) {
    val context = LocalContext.current
    val scrollState = rememberScrollState()

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFF0A0F1D))
            .padding(16.dp)
            .verticalScroll(scrollState),
        verticalArrangement = Arrangement.spacedBy(16.dp)
    ) {
        // App Header - Title: Rosdroid Robot (No manual start/stop button needed)
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Column {
                Text(
                    text = "Rosdroid Robot",
                    fontSize = 24.sp,
                    fontWeight = FontWeight.Bold,
                    color = Color.White
                )
                Text(
                    text = "● Online & Active (ROS 2 Jazzy)",
                    fontSize = 13.sp,
                    color = Color(0xFF00E676)
                )
            }
        }

        // Card 1: System, Power & Battery Telemetry
        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = Color(0xFF1E293B)),
            shape = RoundedCornerShape(16.dp)
        ) {
            Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
                Text("System & Battery Telemetry", fontWeight = FontWeight.Bold, color = Color.White, fontSize = 16.sp)

                TelemetryRow(
                    icon = Icons.Default.Wifi,
                    label = "IP Address",
                    value = status.ipAddress
                )

                TelemetryRow(
                    icon = if (status.isCharging) Icons.Default.BatteryChargingFull else Icons.Default.BatteryStd,
                    label = "Battery Level",
                    value = "${status.batteryPercentage}% ${if (status.isCharging) "(Charging)" else ""}"
                )

                SensorStreamToggle(
                    label = "Stream Battery (/battery)",
                    topic = "sensor_msgs/msg/BatteryState",
                    enabled = streamConfig.streamBattery,
                    onToggle = { onToggleSensor("battery", it) }
                )

                HorizontalDivider(color = Color(0xFF334155), thickness = 0.5.dp)

                // Domain ID selector
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Icon(Icons.Default.Tune, contentDescription = null, tint = Color(0xFF38BDF8))
                        Spacer(modifier = Modifier.width(8.dp))
                        Text("ROS Domain ID", color = Color(0xFF94A3B8), fontSize = 14.sp)
                    }

                    Row(verticalAlignment = Alignment.CenterVertically) {
                        IconButton(onClick = { if (status.rosDomainId > 0) onDomainChange(status.rosDomainId - 1) }) {
                            Icon(Icons.Default.Remove, contentDescription = null, tint = Color.White)
                        }
                        Text(
                            "${status.rosDomainId}",
                            color = Color(0xFF38BDF8),
                            fontWeight = FontWeight.Bold,
                            fontSize = 16.sp
                        )
                        IconButton(onClick = { if (status.rosDomainId < 232) onDomainChange(status.rosDomainId + 1) }) {
                            Icon(Icons.Default.Add, contentDescription = null, tint = Color.White)
                        }
                    }
                }
            }
        }

        // Card 2: Multi-Camera Vision
        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = Color(0xFF1E293B)),
            shape = RoundedCornerShape(16.dp)
        ) {
            Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text("Cameras & Vision", fontWeight = FontWeight.Bold, color = Color.White, fontSize = 16.sp)
                    Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                        TextButton(onClick = onToggleTorch) {
                            Icon(
                                if (isTorchOn) Icons.Default.FlashOn else Icons.Default.FlashOff,
                                contentDescription = null,
                                tint = if (isTorchOn) Color(0xFFFFD700) else Color(0xFF94A3B8)
                            )
                            Spacer(modifier = Modifier.width(4.dp))
                            Text(
                                if (isTorchOn) "Torch ON" else "Torch OFF",
                                color = if (isTorchOn) Color(0xFFFFD700) else Color(0xFF94A3B8),
                                fontSize = 12.sp
                            )
                        }
                        TextButton(onClick = onSwitchCamera) {
                            Icon(Icons.Default.FlipCameraAndroid, contentDescription = null, tint = Color(0xFF38BDF8))
                            Spacer(modifier = Modifier.width(4.dp))
                            Text("Switch Lens", color = Color(0xFF38BDF8), fontSize = 12.sp)
                        }
                    }
                }

                TelemetryRow(
                    icon = if (isTorchOn) Icons.Default.FlashOn else Icons.Default.FlashOff,
                    label = "Torch / Flashlight (/torch)",
                    value = if (isTorchOn) "ACTIVE (ON)" else "INACTIVE (OFF)",
                    valueColor = if (isTorchOn) Color(0xFFFFD700) else Color(0xFF94A3B8)
                )

                TelemetryRow(
                    icon = Icons.Default.CameraAlt,
                    label = "Active Camera Lens",
                    value = if (isFrontCamera) "Front Camera (/camera/front)" else "Back Camera (/camera/back)",
                    valueColor = Color(0xFF38BDF8)
                )

                TelemetryRow(
                    icon = Icons.Default.Speed,
                    label = "Vision Stream Rate",
                    value = "$cameraFps FPS (640x480 JPEG)"
                )

                SensorStreamToggle(
                    label = "Stream Camera Feed",
                    topic = "sensor_msgs/msg/CompressedImage",
                    enabled = streamConfig.streamCamera,
                    onToggle = { onToggleSensor("camera", it) }
                )
            }
        }

        // Card 3: Hardware Sensors & Micro-ROS
        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = Color(0xFF1E293B)),
            shape = RoundedCornerShape(16.dp)
        ) {
            Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
                Text("Hardware Sensors & ROS Streams", fontWeight = FontWeight.Bold, color = Color.White, fontSize = 16.sp)

                val (usbText, usbColor) = when (usbState) {
                    is UsbConnectionState.Connected -> Pair("Connected: ${usbState.deviceName}", Color(0xFF00E676))
                    is UsbConnectionState.Connecting -> Pair("Connecting...", Color(0xFFFFD600))
                    is UsbConnectionState.Error -> Pair("Error: ${usbState.message}", Color(0xFFEF4444))
                    UsbConnectionState.Disconnected -> Pair("No ESP32 connected", Color.Gray)
                }

                TelemetryRow(
                    icon = Icons.Default.Usb,
                    label = "ESP32 USB Serial OTG",
                    value = usbText,
                    valueColor = usbColor
                )

                // IMU
                HorizontalDivider(color = Color(0xFF334155), thickness = 0.5.dp)
                TelemetryRow(
                    icon = Icons.Default.CompassCalibration,
                    label = "IMU Orientation (Madgwick)",
                    value = String.format("Q: [%.2f, %.2f, %.2f, %.2f]", imuData.qW, imuData.qX, imuData.qY, imuData.qZ)
                )
                TelemetryRow(
                    icon = Icons.Default.Speed,
                    label = "Linear Acceleration",
                    value = String.format("[%.1f, %.1f, %.1f] m/s²", imuData.ax, imuData.ay, imuData.az)
                )
                SensorStreamToggle(
                    label = "Stream IMU (/imu/data)",
                    topic = "sensor_msgs/msg/Imu",
                    enabled = streamConfig.streamImu,
                    onToggle = { onToggleSensor("imu", it) }
                )

                // Illuminance
                HorizontalDivider(color = Color(0xFF334155), thickness = 0.5.dp)
                TelemetryRow(
                    icon = Icons.Default.WbSunny,
                    label = "Illumination Sensor",
                    value = String.format("%.1f lux", envData.illuminanceLux),
                    valueColor = Color(0xFFFFD600)
                )
                SensorStreamToggle(
                    label = "Stream Illuminance (/illuminance)",
                    topic = "sensor_msgs/msg/Illuminance",
                    enabled = streamConfig.streamIlluminance,
                    onToggle = { onToggleSensor("illuminance", it) }
                )

                // GPS
                HorizontalDivider(color = Color(0xFF334155), thickness = 0.5.dp)
                TelemetryRow(
                    icon = Icons.Default.Navigation,
                    label = "GPS Location",
                    value = if (gpsData.hasFix) String.format("%.5f, %.5f", gpsData.latitude, gpsData.longitude) else "Acquiring Fix"
                )
                SensorStreamToggle(
                    label = "Stream GPS (/gps)",
                    topic = "sensor_msgs/msg/NavSatFix",
                    enabled = streamConfig.streamGps,
                    onToggle = { onToggleSensor("gps", it) }
                )

                // ToF / Range
                HorizontalDivider(color = Color(0xFF334155), thickness = 0.5.dp)
                TelemetryRow(
                    icon = Icons.Default.Thermostat,
                    label = "Barometer & Proximity",
                    value = String.format("%.1f hPa | %.1f cm", envData.pressureHpa, envData.proximityCm)
                )
                SensorStreamToggle(
                    label = "Stream Range ToF (/tof)",
                    topic = "sensor_msgs/msg/Range",
                    enabled = streamConfig.streamTof,
                    onToggle = { onToggleSensor("tof", it) }
                )
            }
        }

        // Card 4: Audio, Voice & Speech Interaction
        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = Color(0xFF1E293B)),
            shape = RoundedCornerShape(16.dp)
        ) {
            Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
                Text("Audio, Voice & Speech (Wake Word)", fontWeight = FontWeight.Bold, color = Color.White, fontSize = 16.sp)

                TelemetryRow(
                    icon = Icons.Default.Mic,
                    label = "Wake Word (Trigger Only)",
                    value = "Wake: 'Hey Robot' / 'Robot'",
                    valueColor = Color(0xFF00E676)
                )

                Text(
                    text = "Topic: /speech_recognition/hotword_detected (std_msgs/Bool, only on trigger)\nTopic: /speech_recognition/result (std_msgs/String)",
                    color = Color(0xFF94A3B8),
                    fontSize = 11.sp
                )

                if (speechText.isNotBlank()) {
                    TelemetryRow(
                        icon = Icons.Default.Hearing,
                        label = "Last Recognized",
                        value = "\"$speechText\"",
                        valueColor = Color(0xFF38BDF8)
                    )
                }

                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Icon(Icons.Default.VolumeUp, contentDescription = null, tint = Color(0xFF38BDF8), modifier = Modifier.size(20.dp))
                        Spacer(modifier = Modifier.width(8.dp))
                        Text("TTS Action (/tts)", color = Color(0xFF94A3B8), fontSize = 14.sp)
                    }

                    OutlinedButton(
                        onClick = { onTestSpeak("Hello, I am Rosdroid Robot. All systems are operational.") },
                        shape = RoundedCornerShape(8.dp),
                        colors = ButtonDefaults.outlinedButtonColors(contentColor = Color(0xFF38BDF8))
                    ) {
                        Text("Test Speak", fontSize = 12.sp)
                    }
                }
            }
        }

        // Card 5: Biometric Face ID & Fingerprint Verification
        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = Color(0xFF1E293B)),
            shape = RoundedCornerShape(16.dp)
        ) {
            Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
                Text("Biometric Security (ROS Action Server: /auth)", fontWeight = FontWeight.Bold, color = Color.White, fontSize = 16.sp)

                val (bioText, bioColor) = when (biometricState) {
                    is BiometricState.Authenticated -> Pair("Verified (Authorized)", Color(0xFF00E676))
                    is BiometricState.Authenticating -> Pair("Scanning Face/Fingerprint...", Color(0xFFFFD600))
                    is BiometricState.Failed -> Pair("Failed: ${biometricState.reason}", Color(0xFFEF4444))
                    is BiometricState.Canceled -> Pair("Canceled", Color.Gray)
                    BiometricState.Idle -> Pair("Standby (/auth ready)", Color(0xFF94A3B8))
                }

                TelemetryRow(
                    icon = Icons.Default.Fingerprint,
                    label = "Biometric Status",
                    value = bioText,
                    valueColor = bioColor
                )

                Button(
                    onClick = onTriggerBiometrics,
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF38BDF8)),
                    shape = RoundedCornerShape(10.dp),
                    modifier = Modifier.fillMaxWidth()
                ) {
                    Icon(Icons.Default.Face, contentDescription = null, tint = Color.Black)
                    Spacer(modifier = Modifier.width(8.dp))
                    Text("Test Biometric Auth (/auth)", color = Color.Black, fontWeight = FontWeight.Bold, fontSize = 13.sp)
                }
            }
        }

        // Card 6: Remote Terminal Access (SSH Server)
        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = Color(0xFF1E293B)),
            shape = RoundedCornerShape(16.dp)
        ) {
            Column(modifier = Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
                Text("Remote Terminal Access (SSH Server)", fontWeight = FontWeight.Bold, color = Color.White, fontSize = 16.sp)

                // Prominently Show SSH Credentials
                Card(
                    modifier = Modifier.fillMaxWidth(),
                    colors = CardDefaults.cardColors(containerColor = Color(0xFF0F172A)),
                    shape = RoundedCornerShape(10.dp)
                ) {
                    Column(modifier = Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Text("SSH Username:", color = Color(0xFF94A3B8), fontSize = 13.sp)
                            Text("robot (or root)", color = Color(0xFF00E676), fontWeight = FontWeight.Bold, fontSize = 14.sp)
                        }

                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Text("SSH Password:", color = Color(0xFF94A3B8), fontSize = 13.sp)
                            Row(verticalAlignment = Alignment.CenterVertically) {
                                Text("rosdroid", color = Color(0xFFFFD600), fontWeight = FontWeight.Bold, fontSize = 14.sp)
                                Spacer(modifier = Modifier.width(8.dp))
                                TextButton(
                                    onClick = { copyToClipboard(context, "SSH Password", "rosdroid") },
                                    contentPadding = PaddingValues(horizontal = 6.dp, vertical = 2.dp)
                                ) {
                                    Text("Copy", color = Color(0xFF38BDF8), fontSize = 11.sp)
                                }
                            }
                        }

                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.SpaceBetween,
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Text("SSH Port:", color = Color(0xFF94A3B8), fontSize = 13.sp)
                            Text("2222", color = Color.White, fontWeight = FontWeight.Bold, fontSize = 14.sp)
                        }
                    }
                }

                val sshCmd = "ssh robot@${status.ipAddress} -p 2222"
                RemoteActionRow(
                    title = "Connect Terminal from PC",
                    subtitle = sshCmd,
                    buttonText = "Copy SSH",
                    onClick = {
                        copyToClipboard(context, "SSH Command", sshCmd)
                    }
                )
            }
        }

        Spacer(modifier = Modifier.height(32.dp))
    }
}

@Composable
fun SensorStreamToggle(
    label: String,
    topic: String,
    enabled: Boolean,
    onToggle: (Boolean) -> Unit
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Column(modifier = Modifier.weight(1f)) {
            Text(label, color = Color.White, fontWeight = FontWeight.Medium, fontSize = 13.sp)
            Text(topic, color = if (enabled) Color(0xFF00E676) else Color.Gray, fontSize = 11.sp)
        }
        Switch(
            checked = enabled,
            onCheckedChange = onToggle,
            colors = SwitchDefaults.colors(
                checkedThumbColor = Color.Black,
                checkedTrackColor = Color(0xFF00E676),
                uncheckedThumbColor = Color.Gray,
                uncheckedTrackColor = Color(0xFF334155)
            )
        )
    }
}

@Composable
fun TelemetryRow(
    icon: ImageVector,
    label: String,
    value: String,
    valueColor: Color = Color.White
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Icon(icon, contentDescription = null, tint = Color(0xFF38BDF8), modifier = Modifier.size(20.dp))
            Spacer(modifier = Modifier.width(8.dp))
            Text(label, color = Color(0xFF94A3B8), fontSize = 14.sp)
        }
        Text(value, color = valueColor, fontWeight = FontWeight.Medium, fontSize = 13.sp)
    }
}

@Composable
fun RemoteActionRow(
    title: String,
    subtitle: String,
    buttonText: String,
    onClick: () -> Unit
) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Column(modifier = Modifier.weight(1f)) {
            Text(title, color = Color.White, fontWeight = FontWeight.SemiBold, fontSize = 14.sp)
            Text(subtitle, color = Color(0xFF38BDF8), fontSize = 12.sp)
        }
        Button(
            onClick = onClick,
            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF334155)),
            shape = RoundedCornerShape(8.dp)
        ) {
            Text(buttonText, color = Color.White, fontSize = 12.sp)
        }
    }
}

private fun copyToClipboard(context: Context, label: String, text: String) {
    val clipboard = context.getSystemService(Context.CLIPBOARD_SERVICE) as ClipboardManager
    val clip = ClipData.newPlainText(label, text)
    clipboard.setPrimaryClip(clip)
    Toast.makeText(context, "$label copied!", Toast.LENGTH_SHORT).show()
}
