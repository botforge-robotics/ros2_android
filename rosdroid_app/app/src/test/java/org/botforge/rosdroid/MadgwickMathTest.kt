package org.botforge.rosdroid

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sqrt

class MadgwickMathTest {

    @Test
    fun testQuaternionNormalization() {
        var q0 = 1.0f
        var q1 = 0.0f
        var q2 = 0.0f
        var q3 = 0.0f

        // Simulate multiple sensor ticks
        for (i in 0 until 50) {
            val gx = 0.05f
            val gy = -0.02f
            val gz = 0.01f

            val ax = 0.0f
            val ay = 0.0f
            val az = 9.81f

            // Normalized accel
            val aNorm = sqrt(ax * ax + ay * ay + az * az)
            val nAz = az / aNorm

            // Integrate
            val qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz)
            val qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy)
            val qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx)
            val qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx)

            val dt = 1.0f / 100.0f
            q0 += qDot1 * dt
            q1 += qDot2 * dt
            q2 += qDot3 * dt
            q3 += qDot4 * dt

            val norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
            q0 /= norm
            q1 /= norm
            q2 /= norm
            q3 /= norm
        }

        val finalMagnitude = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        assertEquals(1.0f, finalMagnitude, 1e-4f)
        assertTrue("Quaternion w must be positive for initial upright pose", q0 > 0.9f)
    }
}
