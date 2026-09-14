#include "kalman_filter.h"

namespace kalman {

void init(KalmanFilter* kf, float Q_angle, float Q_bias, float R_measure) {
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

float update(KalmanFilter* kf, float newAngle, float newRate, float dt) {
    // ---- Prediction ----
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // ---- Update ----
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    float y = newAngle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    // PERTAHANKAN FIX: keempat elemen P dihitung memakai nilai P LAMA
    // (sebelum update), bukan nilai P[0][0]/P[0][1] yang sudah berubah,
    // sesuai koreksi bug di program referensi.
    float P00_old = kf->P[0][0];
    float P01_old = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_old;
    kf->P[0][1] -= K[0] * P01_old;
    kf->P[1][0] -= K[1] * P00_old;
    kf->P[1][1] -= K[1] * P01_old;

    return kf->angle;
}

} // namespace kalman
