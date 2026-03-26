#ifndef ORIENTATION_KALMAN_H
#define ORIENTATION_KALMAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Simple 1D Kalman filter for angle + gyro bias.
 * State: [angle; bias]; Measurement: accel-derived angle (deg).
 */
typedef struct {
    float angle;   /* estimated angle (deg) */
    float bias;    /* estimated gyro bias (deg/s) */
    float rate;    /* unbiased rate (deg/s) */
    float P[2][2]; /* covariance */
    float Q_angle; /* process noise angle */
    float Q_bias;  /* process noise bias */
    float R_meas;  /* measurement noise (accel angle) */
} Kalman1D_t;

void Kalman1D_Init(Kalman1D_t *kf, float q_angle, float q_bias, float r_meas, float init_angle_deg);
float Kalman1D_Update(Kalman1D_t *kf, float new_rate_dps, float meas_angle_deg, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* ORIENTATION_KALMAN_H */
