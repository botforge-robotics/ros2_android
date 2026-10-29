package org.botforge.rosdroid.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Clear
import androidx.compose.material.icons.filled.Send
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.input.ImeAction
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.launch
import org.botforge.rosdroid.core.PRootManager

@Composable
fun ConsoleScreen(
    prootManager: PRootManager,
    domainId: Int = 156
) {
    val scope = rememberCoroutineScope()
    var commandInput by remember { mutableStateOf("") }
    val logLines = remember { mutableStateListOf("Rosdroid Terminal Console Ready.", "Type commands below or use quick actions.") }
    val listState = rememberLazyListState()

    fun execute(cmd: String) {
        if (cmd.isBlank()) return
        logLines.add("$ $cmd")
        commandInput = ""
        scope.launch {
            prootManager.runCommand(cmd, domainId) { line ->
                logLines.add(line)
            }
            listState.animateScrollToItem(logLines.size - 1)
        }
    }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFF05120B))
            .padding(12.dp)
    ) {
        // Quick Action Chips
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            AssistChip(
                onClick = { execute("ros2 topic list") },
                label = { Text("ros2 topic list", fontSize = 11.sp, color = Color(0xFFE8F5E9)) },
                colors = AssistChipDefaults.assistChipColors(containerColor = Color(0xFF0D2818))
            )
            AssistChip(
                onClick = { execute("ros2 node list") },
                label = { Text("ros2 node list", fontSize = 11.sp, color = Color(0xFFE8F5E9)) },
                colors = AssistChipDefaults.assistChipColors(containerColor = Color(0xFF0D2818))
            )
            AssistChip(
                onClick = { execute("uname -a") },
                label = { Text("uname -a", fontSize = 11.sp, color = Color(0xFFE8F5E9)) },
                colors = AssistChipDefaults.assistChipColors(containerColor = Color(0xFF0D2818))
            )
            IconButton(onClick = { logLines.clear(); logLines.add("Console cleared.") }) {
                Icon(Icons.Default.Clear, contentDescription = "Clear", tint = Color(0xFF00E676))
            }
        }

        Spacer(modifier = Modifier.height(8.dp))

        // Terminal Log Area
        Card(
            modifier = Modifier
                .weight(1f)
                .fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = Color(0xFF020905)),
            shape = RoundedCornerShape(12.dp)
        ) {
            LazyColumn(
                modifier = Modifier
                    .fillMaxSize()
                    .padding(12.dp),
                state = listState,
                verticalArrangement = Arrangement.spacedBy(4.dp)
            ) {
                items(logLines) { line ->
                    Text(
                        text = line,
                        color = if (line.startsWith("$")) Color(0xFF00E676)
                        else if (line.contains("Error", ignoreCase = true)) Color(0xFFEF4444)
                        else Color(0xFFC8E6C9),
                        fontFamily = FontFamily.Monospace,
                        fontSize = 12.sp
                    )
                }
            }
        }

        Spacer(modifier = Modifier.height(8.dp))

        // Input Command Line
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically
        ) {
            OutlinedTextField(
                value = commandInput,
                onValueChange = { commandInput = it },
                placeholder = { Text("Enter ROS 2 or bash command...", color = Color(0xFF4E775B), fontSize = 12.sp) },
                modifier = Modifier.weight(1f),
                singleLine = true,
                keyboardOptions = KeyboardOptions(imeAction = ImeAction.Send),
                keyboardActions = KeyboardActions(onSend = { execute(commandInput) }),
                colors = OutlinedTextFieldDefaults.colors(
                    focusedTextColor = Color.White,
                    unfocusedTextColor = Color(0xFFE8F5E9),
                    focusedBorderColor = Color(0xFF00E676),
                    unfocusedBorderColor = Color(0xFF1B432A)
                )
            )

            Spacer(modifier = Modifier.width(8.dp))

            IconButton(
                onClick = { execute(commandInput) },
                colors = IconButtonDefaults.iconButtonColors(containerColor = Color(0xFF00E676))
            ) {
                Icon(Icons.Default.Send, contentDescription = "Send", tint = Color(0xFF05120B))
            }
        }
    }
}
