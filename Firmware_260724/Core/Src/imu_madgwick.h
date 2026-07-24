/**
 * @file imu_madgwick.h
 * @brief Madgwick AHRS filter for 6-DOF IMU (accel + gyro, no magnetometer)
 *
 * Implements Sebastian Madgwick's AHRS algorithm using accelerometer and
 * gyroscope data to produce a quaternion orientation estimate.
 *
 * Reference: S. O. H. Madgwick, "An efficient orientation filter for
 * inertial and inertial/magnetic sensor arrays", April 2010.
 */

#ifndef IMU_MADGWICK_H
#define IMU_MADGWICK_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Public types                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Quaternion representation: q0 = w (scalar), q1-q3 = xyz (vector)
 */
typedef struct {
    float q0; /**< Scalar component */
    float q1; /**< i component      */
    float q2; /**< j component      */
    float q3; /**< k component      */
} Quaternion_t;

/**
 * @brief Euler angles in degrees (roll, pitch, yaw)
 */
typedef struct {
    float roll;  /**< Rotation about X-axis, degrees */
    float pitch; /**< Rotation about Y-axis, degrees */
    float yaw;   /**< Rotation about Z-axis, degrees */
} EulerAngles_t;

/**
 * @brief Madgwick filter state
 */
typedef struct {
    float      beta;        /**< Algorithm gain (gradient descent step size) */
    float      sample_freq; /**< Sample frequency in Hz                       */
    Quaternion_t q;         /**< Current quaternion estimate                  */
} MadgwickFilter_t;

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise the Madgwick filter.
 *
 * @param filter      Pointer to filter state.
 * @param sample_freq Sample rate of IMU data in Hz (e.g. 100.0f).
 * @param beta        Filter gain (0.01 = slow, 0.1 = typical, 0.5 = fast).
 *                    Higher values converge faster but are noisier.
 */
void Madgwick_Init(MadgwickFilter_t *filter, float sample_freq, float beta);

/**
 * @brief Update filter with new IMU measurement (6-DOF, no magnetometer).
 *
 * All units must be consistent:
 *   - Gyroscope   : rad/s
 *   - Accelerometer: any consistent unit (will be normalised internally)
 *
 * @param filter  Pointer to filter state.
 * @param gx,gy,gz Gyroscope X/Y/Z in rad/s.
 * @param ax,ay,az Accelerometer X/Y/Z (m/s² or g, normalised internally).
 */
void Madgwick_Update(MadgwickFilter_t *filter,
                     float gx, float gy, float gz,
                     float ax, float ay, float az);

/**
 * @brief Update the sample frequency used by the filter (Hz).
 */
void Madgwick_SetSampleFreq(MadgwickFilter_t *filter, float sample_freq);

/**
 * @brief Convert current quaternion estimate to Euler angles (ZYX convention).
 *
 * @param filter  Pointer to filter state.
 * @param euler   Output Euler angles in degrees.
 */
void Madgwick_GetEuler(const MadgwickFilter_t *filter, EulerAngles_t *euler);

/**
 * @brief Get a copy of the current quaternion.
 *
 * @param filter  Pointer to filter state.
 * @param q       Output quaternion.
 */
void Madgwick_GetQuaternion(const MadgwickFilter_t *filter, Quaternion_t *q);

#ifdef __cplusplus
}
#endif

#endif /* IMU_MADGWICK_H */