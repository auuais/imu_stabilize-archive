/**
 * @file imu_madgwick.c
 * @brief Madgwick AHRS filter implementation (6-DOF: accel + gyro)
 *
 * Port of Sebastian Madgwick's open-source AHRS algorithm.
 * Reference: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */

#include "imu_madgwick.h"
#include <math.h>

/* -------------------------------------------------------------------------- */
/* Private helpers                                                             */
/* -------------------------------------------------------------------------- */

/** Fast inverse square-root approximation (Quake-style) */
static float inv_sqrt(float x)
{
    /* Use compiler built-in for Cortex-M targets with FPU */
    return 1.0f / sqrtf(x);
}

/* -------------------------------------------------------------------------- */
/* Public implementation                                                       */
/* -------------------------------------------------------------------------- */

void Madgwick_Init(MadgwickFilter_t *filter, float sample_freq, float beta)
{
    filter->beta        = beta;
    filter->sample_freq = sample_freq;
    /* Start with identity quaternion (no rotation) */
    filter->q.q0 = 1.0f;
    filter->q.q1 = 0.0f;
    filter->q.q2 = 0.0f;
    filter->q.q3 = 0.0f;
}

void Madgwick_Update(MadgwickFilter_t *filter,
                     float gx, float gy, float gz,
                     float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2, _4q3;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    float q0 = filter->q.q0;
    float q1 = filter->q.q1;
    float q2 = filter->q.q2;
    float q3 = filter->q.q3;

    /* Rate of change of quaternion from gyroscope */
    qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    /* Apply feedback step only if accelerometer measurement is valid */
    float accel_norm_sq = ax * ax + ay * ay + az * az;
    if (accel_norm_sq > 0.0f) {
        /* Normalise accelerometer measurement */
        recipNorm = inv_sqrt(accel_norm_sq);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Auxiliary variables */
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _4q3 = 4.0f * q3;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        /* Gradient descent algorithm corrective step */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
             - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
             - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        /* Normalise gradient */
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        /* Apply feedback step */
        qDot0 -= filter->beta * s0;
        qDot1 -= filter->beta * s1;
        qDot2 -= filter->beta * s2;
        qDot3 -= filter->beta * s3;
    }

    /* Integrate rate of change of quaternion */
    float dt = 1.0f / filter->sample_freq;
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    /* Normalise quaternion */
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    filter->q.q0 = q0 * recipNorm;
    filter->q.q1 = q1 * recipNorm;
    filter->q.q2 = q2 * recipNorm;
    filter->q.q3 = q3 * recipNorm;
}

void Madgwick_SetSampleFreq(MadgwickFilter_t *filter, float sample_freq)
{
    if (filter == NULL) return;
    if (sample_freq > 1e-3f) {
        filter->sample_freq = sample_freq;
    }
}

void Madgwick_GetEuler(const MadgwickFilter_t *filter, EulerAngles_t *euler)
{
    float q0 = filter->q.q0;
    float q1 = filter->q.q1;
    float q2 = filter->q.q2;
    float q3 = filter->q.q3;

    /* ZYX Euler convention (aerospace / robot arm standard) */

    /* Roll (phi) */
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    euler->roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / 3.14159265358979f);

    /* Pitch (theta) - clamped to avoid gimbal at +-90 deg */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    euler->pitch = asinf(sinp) * (180.0f / 3.14159265358979f);

    /* Yaw (psi) */
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    euler->yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / 3.14159265358979f);
}

void Madgwick_GetQuaternion(const MadgwickFilter_t *filter, Quaternion_t *q)
{
    *q = filter->q;
}