#include <jni.h>
#include <memory>
#include "madgwick.h"

static std::unique_ptr<MadgwickFilter> g_madgwick = std::make_unique<MadgwickFilter>();

extern "C" {

JNIEXPORT void JNICALL
Java_org_botforge_rosdroid_sensors_ImuFusion_initFilter(
        JNIEnv* env,
        jobject /* this */,
        jfloat sampleFrequency,
        jfloat beta) {
    if (!g_madgwick) {
        g_madgwick = std::make_unique<MadgwickFilter>();
    }
    g_madgwick->setSampleFrequency(sampleFrequency);
    g_madgwick->setBeta(beta);
}

JNIEXPORT jfloatArray JNICALL
Java_org_botforge_rosdroid_sensors_ImuFusion_updateIMU(
        JNIEnv* env,
        jobject /* this */,
        jfloat gx, jfloat gy, jfloat gz,
        jfloat ax, jfloat ay, jfloat az) {
    if (!g_madgwick) {
        g_madgwick = std::make_unique<MadgwickFilter>();
    }

    g_madgwick->update(gx, gy, gz, ax, ay, az);

    jfloatArray result = env->NewFloatArray(4);
    if (result == nullptr) return nullptr;

    jfloat q[4] = {
        g_madgwick->getQ0(), // w
        g_madgwick->getQ1(), // x
        g_madgwick->getQ2(), // y
        g_madgwick->getQ3()  // z
    };
    env->SetFloatArrayRegion(result, 0, 4, q);
    return result;
}

JNIEXPORT jfloatArray JNICALL
Java_org_botforge_rosdroid_sensors_ImuFusion_updateMARG(
        JNIEnv* env,
        jobject /* this */,
        jfloat gx, jfloat gy, jfloat gz,
        jfloat ax, jfloat ay, jfloat az,
        jfloat mx, jfloat my, jfloat mz) {
    if (!g_madgwick) {
        g_madgwick = std::make_unique<MadgwickFilter>();
    }

    g_madgwick->update(gx, gy, gz, ax, ay, az, mx, my, mz);

    jfloatArray result = env->NewFloatArray(4);
    if (result == nullptr) return nullptr;

    jfloat q[4] = {
        g_madgwick->getQ0(), // w
        g_madgwick->getQ1(), // x
        g_madgwick->getQ2(), // y
        g_madgwick->getQ3()  // z
    };
    env->SetFloatArrayRegion(result, 0, 4, q);
    return result;
}

} // extern "C"
