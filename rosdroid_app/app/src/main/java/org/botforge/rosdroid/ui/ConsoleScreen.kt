package org.botforge.rosdroid.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.LazyListState
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Add
import androidx.compose.material.icons.filled.Clear
import androidx.compose.material.icons.filled.Close
import androidx.compose.material.icons.filled.KeyboardArrowDown
import androidx.compose.material.icons.filled.KeyboardArrowUp
import androidx.compose.material.icons.filled.Send
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.ImeAction
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch
import org.botforge.rosdroid.core.PRootManager

class TerminalSession(
    val id: Int,
    var name: String = "Term $id"
) {
    var commandInput by mutableStateOf("")
    val logLines = mutableStateListOf<String>(
        "Rosdroid Terminal Session $id Ready.",
        "Use quick actions, TAB for auto-completion, or ↑ / ↓ for bash history."
    )
    var isRunning by mutableStateOf(false)
    var activeProcess: Process? = null
    var activeJob: Job? = null
    val history = mutableListOf<String>()
    var historyIndex by mutableStateOf(-1)

    fun addLine(line: String) {
        if (logLines.size > 800) {
            logLines.removeRange(0, 80)
        }
        logLines.add(line)
    }

    fun interrupt() {
        try {
            activeProcess?.destroyForcibly()
        } catch (ignored: Exception) {}
        activeProcess = null
        activeJob?.cancel()
        activeJob = null
        isRunning = false
        addLine("^C")
    }
}

private val ROS_KEYWORDS = listOf(
    "ros2", "topic", "node", "service", "action", "param", "bag",
    "echo", "list", "info", "pub", "hz", "bw", "type", "find", "show",
    "/battery", "/imu/data", "/camera/front/image_raw/compressed",
    "/camera/back/image_raw/compressed", "/torch", "/cmd_vel",
    "/expression", "/gps", "/illuminance", "/tof",
    "/speech_recognition/result", "/speech_recognition/status",
    "/speech_recognition/hotword_detected",
    "sensor_msgs/msg/Imu", "sensor_msgs/msg/BatteryState",
    "sensor_msgs/msg/CompressedImage", "std_msgs/msg/Bool",
    "std_msgs/msg/String", "geometry_msgs/msg/Twist",
    "source /opt/ros/jazzy/setup.bash", "export ROS_DOMAIN_ID=156",
    "uname -a", "ps aux", "clear"
)

private fun completeCommand(input: String): String {
    if (input.isBlank()) return "ros2 "
    val words = input.split(" ")
    val lastWord = words.last()
    if (lastWord.isBlank()) return input

    val matches = ROS_KEYWORDS.filter { it.startsWith(lastWord, ignoreCase = true) }
    if (matches.isNotEmpty()) {
        val replacement = matches.first()
        val prefix = words.dropLast(1).joinToString(" ")
        return if (prefix.isBlank()) "$replacement " else "$prefix $replacement "
    }
    return input
}

@Composable
fun ConsoleScreen(
    prootManager: PRootManager,
    domainId: Int = 156
) {
    val scope = rememberCoroutineScope()
    var nextSessionId by remember { mutableStateOf(2) }
    val sessions = remember { mutableStateListOf(TerminalSession(1, "Term 1")) }
    var activeSessionIndex by remember { mutableStateOf(0) }
    var isSplitView by remember { mutableStateOf(false) }

    val activeSession = sessions.getOrNull(activeSessionIndex) ?: sessions[0]

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFF05120B))
            .padding(8.dp)
    ) {
        // Top Bar: Terminal Tabs & Split View Toggle
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(bottom = 6.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            Row(
                modifier = Modifier
                    .weight(1f)
                    .horizontalScroll(rememberScrollState()),
                horizontalArrangement = Arrangement.spacedBy(6.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                sessions.forEachIndexed { index, session ->
                    val isSelected = (!isSplitView && index == activeSessionIndex) || (isSplitView && (index == 0 || index == 1))
                    Surface(
                        shape = RoundedCornerShape(8.dp),
                        color = if (isSelected) Color(0xFF133E23) else Color(0xFF091A0E),
                        border = if (isSelected) androidx.compose.foundation.BorderStroke(1.dp, Color(0xFF00E676)) else null,
                        modifier = Modifier
                            .clickable {
                                activeSessionIndex = index
                            }
                    ) {
                        Row(
                            modifier = Modifier.padding(horizontal = 8.dp, vertical = 4.dp),
                            verticalAlignment = Alignment.CenterVertically,
                            horizontalArrangement = Arrangement.spacedBy(4.dp)
                        ) {
                            if (session.isRunning) {
                                Box(
                                    modifier = Modifier
                                        .size(7.dp)
                                        .clip(CircleShape)
                                        .background(Color(0xFF00E676))
                                )
                            }
                            Text(
                                text = session.name,
                                fontSize = 12.sp,
                                fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Normal,
                                color = if (isSelected) Color(0xFF00E676) else Color(0xFF81C784)
                            )
                            if (sessions.size > 1) {
                                Icon(
                                    imageVector = Icons.Default.Close,
                                    contentDescription = "Close Tab",
                                    tint = Color(0xFF81C784),
                                    modifier = Modifier
                                        .size(14.dp)
                                        .clickable {
                                            session.interrupt()
                                            val removeIdx = sessions.indexOf(session)
                                            sessions.remove(session)
                                            if (activeSessionIndex >= sessions.size) {
                                                activeSessionIndex = sessions.size - 1
                                            }
                                        }
                                )
                            }
                        }
                    }
                }

                // Add Terminal Tab Button
                IconButton(
                    onClick = {
                        val newId = nextSessionId++
                        sessions.add(TerminalSession(newId, "Term $newId"))
                        activeSessionIndex = sessions.size - 1
                    },
                    modifier = Modifier.size(28.dp)
                ) {
                    Icon(
                        Icons.Default.Add,
                        contentDescription = "New Terminal",
                        tint = Color(0xFF00E676),
                        modifier = Modifier.size(18.dp)
                    )
                }
            }

            Spacer(modifier = Modifier.width(6.dp))

            // Split View Toggle Button
            OutlinedButton(
                onClick = {
                    if (!isSplitView && sessions.size < 2) {
                        val newId = nextSessionId++
                        sessions.add(TerminalSession(newId, "Term $newId"))
                    }
                    isSplitView = !isSplitView
                },
                contentPadding = PaddingValues(horizontal = 8.dp, vertical = 2.dp),
                shape = RoundedCornerShape(6.dp),
                border = androidx.compose.foundation.BorderStroke(1.dp, Color(0xFF1B432A)),
                colors = ButtonDefaults.outlinedButtonColors(
                    containerColor = if (isSplitView) Color(0xFF133E23) else Color(0xFF091A0E)
                )
            ) {
                Text(
                    text = if (isSplitView) "Single" else "Split",
                    fontSize = 11.sp,
                    color = Color(0xFF00E676),
                    fontWeight = FontWeight.SemiBold
                )
            }
        }

        // Terminal Log Area (Single or Split)
        if (isSplitView && sessions.size >= 2) {
            Column(
                modifier = Modifier
                    .weight(1f)
                    .fillMaxWidth(),
                verticalArrangement = Arrangement.spacedBy(6.dp)
            ) {
                Box(modifier = Modifier.weight(1f).fillMaxWidth()) {
                    TerminalLogView(session = sessions[0])
                }
                Box(modifier = Modifier.weight(1f).fillMaxWidth()) {
                    TerminalLogView(session = sessions[1])
                }
            }
        } else {
            Box(
                modifier = Modifier
                    .weight(1f)
                    .fillMaxWidth()
            ) {
                TerminalLogView(session = activeSession)
            }
        }

        Spacer(modifier = Modifier.height(6.dp))

        // Terminal Controls Bar: Ctrl+C, TAB, Up, Down, Clear, Preset Chips
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(6.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Ctrl+C Interrupt Button
            Button(
                onClick = { activeSession.interrupt() },
                colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFB71C1C)),
                shape = RoundedCornerShape(6.dp),
                contentPadding = PaddingValues(horizontal = 10.dp, vertical = 4.dp),
                modifier = Modifier.height(30.dp)
            ) {
                Text("Ctrl+C", fontSize = 11.sp, fontWeight = FontWeight.Bold, color = Color.White)
            }

            // TAB Auto-Complete Button
            Button(
                onClick = {
                    activeSession.commandInput = completeCommand(activeSession.commandInput)
                },
                colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF1B432A)),
                shape = RoundedCornerShape(6.dp),
                contentPadding = PaddingValues(horizontal = 10.dp, vertical = 4.dp),
                modifier = Modifier.height(30.dp)
            ) {
                Text("TAB ⇥", fontSize = 11.sp, fontWeight = FontWeight.Bold, color = Color(0xFF00E676))
            }

            // History Up Arrow
            IconButton(
                onClick = { navigateHistory(activeSession, up = true) },
                modifier = Modifier
                    .size(30.dp)
                    .background(Color(0xFF0D2818), RoundedCornerShape(6.dp))
            ) {
                Icon(Icons.Default.KeyboardArrowUp, contentDescription = "History Up", tint = Color(0xFF00E676), modifier = Modifier.size(18.dp))
            }

            // History Down Arrow
            IconButton(
                onClick = { navigateHistory(activeSession, up = false) },
                modifier = Modifier
                    .size(30.dp)
                    .background(Color(0xFF0D2818), RoundedCornerShape(6.dp))
            ) {
                Icon(Icons.Default.KeyboardArrowDown, contentDescription = "History Down", tint = Color(0xFF00E676), modifier = Modifier.size(18.dp))
            }

            // Quick Chips
            QuickChip(label = "ros2 topic list") { executeCommand(activeSession, prootManager, domainId, scope, "ros2 topic list") }
            QuickChip(label = "ros2 node list") { executeCommand(activeSession, prootManager, domainId, scope, "ros2 node list") }
            QuickChip(label = "echo /imu/data") { executeCommand(activeSession, prootManager, domainId, scope, "ros2 topic echo /imu/data") }
            QuickChip(label = "echo /battery") { executeCommand(activeSession, prootManager, domainId, scope, "ros2 topic echo /battery") }
            QuickChip(label = "uname -a") { executeCommand(activeSession, prootManager, domainId, scope, "uname -a") }

            IconButton(
                onClick = {
                    activeSession.logLines.clear()
                    activeSession.logLines.add("Console cleared.")
                },
                modifier = Modifier.size(30.dp)
            ) {
                Icon(Icons.Default.Clear, contentDescription = "Clear", tint = Color(0xFF00E676), modifier = Modifier.size(16.dp))
            }
        }

        Spacer(modifier = Modifier.height(6.dp))

        // Input Command Line
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically
        ) {
            OutlinedTextField(
                value = activeSession.commandInput,
                onValueChange = { activeSession.commandInput = it },
                placeholder = { Text("Enter ROS 2 or bash command...", color = Color(0xFF4E775B), fontSize = 12.sp) },
                modifier = Modifier.weight(1f),
                singleLine = true,
                keyboardOptions = KeyboardOptions(imeAction = ImeAction.Send),
                keyboardActions = KeyboardActions(onSend = {
                    executeCommand(activeSession, prootManager, domainId, scope, activeSession.commandInput)
                }),
                colors = OutlinedTextFieldDefaults.colors(
                    focusedTextColor = Color.White,
                    unfocusedTextColor = Color(0xFFE8F5E9),
                    focusedBorderColor = Color(0xFF00E676),
                    unfocusedBorderColor = Color(0xFF1B432A)
                )
            )

            Spacer(modifier = Modifier.width(8.dp))

            IconButton(
                onClick = {
                    executeCommand(activeSession, prootManager, domainId, scope, activeSession.commandInput)
                },
                colors = IconButtonDefaults.iconButtonColors(containerColor = Color(0xFF00E676)),
                modifier = Modifier.size(46.dp)
            ) {
                Icon(Icons.Default.Send, contentDescription = "Send", tint = Color(0xFF05120B))
            }
        }
    }
}

@Composable
fun TerminalLogView(
    session: TerminalSession
) {
    val listState = rememberLazyListState()

    // Auto-scroll when new log lines arrive (e.g. continuous streaming topic echo)
    LaunchedEffect(session.logLines.size) {
        if (session.logLines.isNotEmpty()) {
            listState.scrollToItem(session.logLines.size - 1)
        }
    }

    Card(
        modifier = Modifier.fillMaxSize(),
        colors = CardDefaults.cardColors(containerColor = Color(0xFF020905)),
        shape = RoundedCornerShape(10.dp),
        border = androidx.compose.foundation.BorderStroke(1.dp, Color(0xFF0F301B))
    ) {
        Column(modifier = Modifier.fillMaxSize()) {
            // Header bar for split/tab title
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .background(Color(0xFF091F11))
                    .padding(horizontal = 10.dp, vertical = 4.dp),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    Box(
                        modifier = Modifier
                            .size(6.dp)
                            .clip(CircleShape)
                            .background(if (session.isRunning) Color(0xFF00E676) else Color(0xFF4E775B))
                    )
                    Text(
                        text = session.name,
                        fontSize = 11.sp,
                        fontWeight = FontWeight.Bold,
                        color = Color(0xFF81C784)
                    )
                }
                if (session.isRunning) {
                    Text("RUNNING", fontSize = 10.sp, color = Color(0xFF00E676), fontWeight = FontWeight.Bold)
                }
            }

            LazyColumn(
                modifier = Modifier
                    .fillMaxSize()
                    .padding(horizontal = 8.dp, vertical = 6.dp),
                state = listState,
                verticalArrangement = Arrangement.spacedBy(2.dp)
            ) {
                items(session.logLines) { line ->
                    Text(
                        text = line,
                        color = when {
                            line.startsWith("$") -> Color(0xFF00E676)
                            line.startsWith("^C") -> Color(0xFFFF5252)
                            line.contains("Error", ignoreCase = true) || line.contains("failed", ignoreCase = true) -> Color(0xFFEF5350)
                            line.contains("warning", ignoreCase = true) -> Color(0xFFFFD54F)
                            else -> Color(0xFFC8E6C9)
                        },
                        fontFamily = FontFamily.Monospace,
                        fontSize = 11.sp,
                        lineHeight = 14.sp
                    )
                }
            }
        }
    }
}

@Composable
fun QuickChip(
    label: String,
    onClick: () -> Unit
) {
    AssistChip(
        onClick = onClick,
        label = { Text(label, fontSize = 11.sp, color = Color(0xFFE8F5E9)) },
        colors = AssistChipDefaults.assistChipColors(containerColor = Color(0xFF0D2818)),
        shape = RoundedCornerShape(6.dp),
        border = androidx.compose.foundation.BorderStroke(1.dp, Color(0xFF1B432A)),
        modifier = Modifier.height(30.dp)
    )
}

fun navigateHistory(session: TerminalSession, up: Boolean) {
    if (session.history.isEmpty()) return
    if (up) {
        if (session.historyIndex == -1) {
            session.historyIndex = session.history.size - 1
        } else if (session.historyIndex > 0) {
            session.historyIndex--
        }
        session.commandInput = session.history.getOrElse(session.historyIndex) { "" }
    } else {
        if (session.historyIndex != -1) {
            if (session.historyIndex < session.history.size - 1) {
                session.historyIndex++
                session.commandInput = session.history[session.historyIndex]
            } else {
                session.historyIndex = -1
                session.commandInput = ""
            }
        }
    }
}

fun executeCommand(
    session: TerminalSession,
    prootManager: PRootManager,
    domainId: Int,
    scope: CoroutineScope,
    rawCmd: String
) {
    val clean = rawCmd.trim()
    if (clean.isBlank()) return

    if (session.history.isEmpty() || session.history.last() != clean) {
        session.history.add(clean)
    }
    session.historyIndex = -1
    session.addLine("$ $clean")
    session.commandInput = ""
    session.isRunning = true

    session.activeJob = scope.launch {
        try {
            prootManager.runStreamingCommand(
                clean,
                domainId,
                onProcessCreated = { proc -> session.activeProcess = proc }
            ) { line ->
                session.addLine(line)
            }
        } finally {
            session.isRunning = false
            session.activeProcess = null
            session.activeJob = null
        }
    }
}
