package org.botforge.rosdroid.ui

import androidx.compose.animation.animateColorAsState
import androidx.compose.animation.core.*
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.unit.dp
import kotlinx.coroutines.delay

enum class RobotMood {
    HAPPY,
    THINKING,
    SPEAKING,
    ALERT
}

@Composable
fun RobotFaceScreen(
    modifier: Modifier = Modifier
) {
    var mood by remember { mutableStateOf(RobotMood.HAPPY) }
    var isBlinking by remember { mutableStateOf(false) }
    var lookTargetX by remember { mutableStateOf(0f) }
    var lookTargetY by remember { mutableStateOf(0f) }

    // Natural life loop: Autonomous mood changes, glances, and blinking
    LaunchedEffect(Unit) {
        val moods = RobotMood.values()
        var step = 0
        while (true) {
            // Random glance direction
            lookTargetX = listOf(-25f, 0f, 25f, 0f).random()
            lookTargetY = listOf(-10f, 0f, 10f).random()

            // Random natural blink
            delay((2000..4500).random().toLong())
            isBlinking = true
            delay(120)
            isBlinking = false

            // Occasional natural expression evolution
            step++
            if (step % 3 == 0) {
                mood = moods.random()
            }
        }
    }

    // Smooth animated eye look offset
    val animatedLookX by animateFloatAsState(
        targetValue = lookTargetX,
        animationSpec = spring(dampingRatio = Spring.DampingRatioMediumBouncy, stiffness = Spring.StiffnessLow),
        label = "eye_look_x"
    )
    val animatedLookY by animateFloatAsState(
        targetValue = lookTargetY,
        animationSpec = spring(dampingRatio = Spring.DampingRatioMediumBouncy, stiffness = Spring.StiffnessLow),
        label = "eye_look_y"
    )

    // Smooth blink compression
    val blinkScaleY by animateFloatAsState(
        targetValue = if (isBlinking) 0.05f else 1.0f,
        animationSpec = tween(durationMillis = 100),
        label = "blink_scale"
    )

    // Pulse animation for speaking state
    val infiniteTransition = rememberInfiniteTransition(label = "pulse")
    val pulseScale by infiniteTransition.animateFloat(
        initialValue = 0.95f,
        targetValue = 1.05f,
        animationSpec = infiniteRepeatable(
            animation = tween(durationMillis = 600, easing = FastOutSlowInEasing),
            repeatMode = RepeatMode.Reverse
        ),
        label = "speaking_pulse"
    )

    val targetEyeColor = when (mood) {
        RobotMood.HAPPY -> Color(0xFF00E676)    // Robot Vibrant Emerald Green
        RobotMood.THINKING -> Color(0xFF69F0AE) // Matrix Mint Green
        RobotMood.SPEAKING -> Color(0xFF00FF66) // Ultra Bright Laser Green
        RobotMood.ALERT -> Color(0xFFB2FF59)    // Neon Electric Lime Green
    }

    val eyeColor by animateColorAsState(
        targetValue = targetEyeColor,
        animationSpec = tween(durationMillis = 400),
        label = "eye_color"
    )

    Box(
        modifier = modifier
            .fillMaxSize()
            .background(Color(0xFF040E06)) // Deep OLED matrix cyber-green black
            .clickable {
                // Interactive tap reaction: smiles happily, glances up, and winks!
                mood = RobotMood.HAPPY
                lookTargetX = 0f
                lookTargetY = -15f
                isBlinking = true
            },
        contentAlignment = Alignment.Center
    ) {
        // Dynamic Eye Canvas
        Canvas(
            modifier = Modifier
                .fillMaxWidth()
                .height(320.dp)
        ) {
            val canvasWidth = size.width
            val canvasHeight = size.height

            val baseEyeWidth = 120.dp.toPx()
            val baseEyeHeight = 140.dp.toPx()
            val eyeSpacing = 60.dp.toPx()

            val effectiveHeight = (baseEyeHeight * (if (mood == RobotMood.SPEAKING) pulseScale else 1.0f)) * blinkScaleY
            val effectiveWidth = if (mood == RobotMood.ALERT) baseEyeWidth * 1.15f else baseEyeWidth

            val centerY = (canvasHeight / 2) + animatedLookY
            val leftEyeX = (canvasWidth / 2) - effectiveWidth - (eyeSpacing / 2) + animatedLookX
            val rightEyeX = (canvasWidth / 2) + (eyeSpacing / 2) + animatedLookX

            when (mood) {
                RobotMood.HAPPY -> {
                    // Arched smiling eyes
                    if (blinkScaleY > 0.3f) {
                        val pathLeft = Path().apply {
                            moveTo(leftEyeX, centerY + 20.dp.toPx())
                            quadraticBezierTo(
                                leftEyeX + effectiveWidth / 2,
                                centerY - effectiveHeight / 2,
                                leftEyeX + effectiveWidth,
                                centerY + 20.dp.toPx()
                            )
                        }
                        val pathRight = Path().apply {
                            moveTo(rightEyeX, centerY + 20.dp.toPx())
                            quadraticBezierTo(
                                rightEyeX + effectiveWidth / 2,
                                centerY - effectiveHeight / 2,
                                rightEyeX + effectiveWidth,
                                centerY + 20.dp.toPx()
                            )
                        }
                        drawPath(pathLeft, color = eyeColor, style = Stroke(width = 28.dp.toPx(), cap = androidx.compose.ui.graphics.StrokeCap.Round))
                        drawPath(pathRight, color = eyeColor, style = Stroke(width = 28.dp.toPx(), cap = androidx.compose.ui.graphics.StrokeCap.Round))
                    } else {
                        // Closed blink line
                        drawRoundRect(
                            color = eyeColor,
                            topLeft = Offset(leftEyeX, centerY),
                            size = Size(effectiveWidth, 12.dp.toPx()),
                            cornerRadius = CornerRadius(6.dp.toPx(), 6.dp.toPx())
                        )
                        drawRoundRect(
                            color = eyeColor,
                            topLeft = Offset(rightEyeX, centerY),
                            size = Size(effectiveWidth, 12.dp.toPx()),
                            cornerRadius = CornerRadius(6.dp.toPx(), 6.dp.toPx())
                        )
                    }
                }

                RobotMood.THINKING -> {
                    // Asymmetric quizzical eyes: Left eye raised, Right eye slightly squinted
                    val leftOffset = -15.dp.toPx()
                    val rightSquint = 0.75f

                    drawRoundRect(
                        color = eyeColor,
                        topLeft = Offset(leftEyeX, centerY - (effectiveHeight / 2) + leftOffset),
                        size = Size(effectiveWidth, effectiveHeight),
                        cornerRadius = CornerRadius(36.dp.toPx(), 36.dp.toPx())
                    )
                    drawRoundRect(
                        color = eyeColor,
                        topLeft = Offset(rightEyeX, centerY - (effectiveHeight * rightSquint / 2)),
                        size = Size(effectiveWidth, effectiveHeight * rightSquint),
                        cornerRadius = CornerRadius(30.dp.toPx(), 30.dp.toPx())
                    )
                }

                RobotMood.ALERT -> {
                    // Wide circular alert eyes with glowing outer ring
                    val radius = (effectiveWidth / 2)
                    drawCircle(
                        color = eyeColor.copy(alpha = 0.25f),
                        radius = radius * 1.35f,
                        center = Offset(leftEyeX + radius, centerY)
                    )
                    drawCircle(
                        color = eyeColor.copy(alpha = 0.25f),
                        radius = radius * 1.35f,
                        center = Offset(rightEyeX + radius, centerY)
                    )

                    drawRoundRect(
                        color = eyeColor,
                        topLeft = Offset(leftEyeX, centerY - (effectiveHeight / 2)),
                        size = Size(effectiveWidth, effectiveHeight),
                        cornerRadius = CornerRadius(45.dp.toPx(), 45.dp.toPx())
                    )
                    drawRoundRect(
                        color = eyeColor,
                        topLeft = Offset(rightEyeX, centerY - (effectiveHeight / 2)),
                        size = Size(effectiveWidth, effectiveHeight),
                        cornerRadius = CornerRadius(45.dp.toPx(), 45.dp.toPx())
                    )
                }

                RobotMood.SPEAKING -> {
                    // Expressive eyes with inner pupil glint
                    drawRoundRect(
                        color = eyeColor,
                        topLeft = Offset(leftEyeX, centerY - (effectiveHeight / 2)),
                        size = Size(effectiveWidth, effectiveHeight),
                        cornerRadius = CornerRadius(36.dp.toPx(), 36.dp.toPx())
                    )
                    drawRoundRect(
                        color = eyeColor,
                        topLeft = Offset(rightEyeX, centerY - (effectiveHeight / 2)),
                        size = Size(effectiveWidth, effectiveHeight),
                        cornerRadius = CornerRadius(36.dp.toPx(), 36.dp.toPx())
                    )

                    if (!isBlinking && blinkScaleY > 0.4f) {
                        val glintRadius = 16.dp.toPx()
                        drawCircle(
                            color = Color(0xFFDCEDC8).copy(alpha = 0.9f),
                            radius = glintRadius,
                            center = Offset(leftEyeX + 34.dp.toPx(), centerY - 28.dp.toPx())
                        )
                        drawCircle(
                            color = Color(0xFFDCEDC8).copy(alpha = 0.9f),
                            radius = glintRadius,
                            center = Offset(rightEyeX + 34.dp.toPx(), centerY - 28.dp.toPx())
                        )
                    }
                }
            }
        }
    }
}
