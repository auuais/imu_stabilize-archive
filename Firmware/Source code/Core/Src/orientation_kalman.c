#include "orientation_kalman.h"
#include <stddef.h>

static void mat_reset(float P[2][2])
{
    P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0.0f;
}

void Kalman1D_Init(Kalman1D_t *kf, float q_angle, float q_bias, float r_meas, float init_angle_deg)
{
    if (kf == NULL) return;
    kf->angle = init_angle_deg;
    kf->bias  = 0.0f;
    kf->rate  = 0.0f;
    kf->Q_angle = q_angle;
    kf->Q_bias  = q_bias;
    kf->R_meas  = r_meas;
    mat_reset(kf->P);
}

float Kalman1D_Update(Kalman1D_t *kf, float new_rate_dps, float meas_angle_deg, float dt_s)
{
    if (kf == NULL || dt_s <= 0.0f) return meas_angle_deg;

    // Predict
    kf->rate = new_rate_dps - kf->bias;
    kf->angle += dt_s * kf->rate;

    // Update covariance
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    float P10 = kf->P[1][0];
    float P11 = kf->P[1][1];

    P00 += dt_s * (dt_s*P11 - P01 - P10 + kf->Q_angle);
    P01 += dt_s * (-P11);
    P10 += dt_s * (-P11);
    P11 += kf->Q_bias * dt_s;

    kf->P[0][0] = P00;
    kf->P[0][1] = P01;
    kf->P[1][0] = P10;
    kf->P[1][1] = P11;

    // Innovation
    float y = meas_angle_deg - kf->angle;

    // Innovation covariance
    float S = P00 + kf->R_meas;
    float K0 = P00 / S;
    float K1 = P10 / S;

    // Update state
    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    // Update covariance
    kf->P[0][0] -= K0 * P00;
    kf->P[0][1] -= K0 * P01;
    kf->P[1][0] -= K1 * P00;
    kf->P[1][1] -= K1 * P01;

    return kf->angle;
}