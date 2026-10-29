package org.botforge.rosdroid.sensors

class ImuFusion {

    init {
        try {
            System.loadLibrary("native_bridge")
        } catch (e: UnsatisfiedLinkError) {
            e.printStackTrace()
        }
    }

    external fun initFilter(sampleFrequency: Float, beta: Float)

    /**
     * 6-DOF IMU update (Gyroscope + Accelerometer).
     * @return FloatArray of 4 elements: [q0 (w), q1 (x), q2 (y), q3 (z)]
     */
    external fun updateIMU(
        gx: Float, gy: Float, gz: Float,
        ax: Float, ay: Float, az: Float
    ): FloatArray

    /**
     * 9-DOF MARG update (Gyroscope + Accelerometer + Magnetometer).
     * @return FloatArray of 4 elements: [q0 (w), q1 (x), q2 (y), q3 (z)]
     */
    external fun updateMARG(
        gx: Float, gy: Float, gz: Float,
        ax: Float, ay: Float, az: Float,
        mx: Float, my: Float, mz: Float
    ): FloatArray
}
