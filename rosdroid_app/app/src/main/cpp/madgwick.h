#ifndef MADGWICK_H
#define MADGWICK_H

#include <cmath>

class MadgwickFilter {
public:
    MadgwickFilter();
    void setSampleFrequency(float sampleFreq);
    void setBeta(float betaVal);
    void update(float gx, float gy, float gz, float ax, float ay, float az);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

    float getQ0() const { return q0; } // w
    float getQ1() const { return q1; } // x
    float getQ2() const { return q2; } // y
    float getQ3() const { return q3; } // z

private:
    float beta;              // algorithm gain
    float q0, q1, q2, q3;    // quaternion elements representing the orientation
    float invSampleFreq;

    static float invSqrt(float x);
};

#endif // MADGWICK_H
