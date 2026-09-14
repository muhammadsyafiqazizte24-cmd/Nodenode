#ifndef SHM_KALMAN_FILTER_H
#define SHM_KALMAN_FILTER_H

// ============================================================================
// kalman_filter.h — Kalman Filter 2-state (angle + bias) untuk pitch & roll
//
// PERTAHANKAN: identik dengan implementasi referensi (termasuk fix bug
// kovarian P yang harus dihitung dari nilai P LAMA secara bersamaan, bukan
// berantai). HANYA Kalman Filter yang dipakai di firmware ini — TIDAK ada
// Complementary/Madgwick/Moving Average (sesuai spesifikasi Node SHM).
// ============================================================================

struct KalmanFilter {
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float rate;
    float P[2][2];
};

namespace kalman {

// Inisialisasi filter dengan parameter noise (lihat config.h: KALMAN_Q_ANGLE,
// KALMAN_Q_BIAS, KALMAN_R_MEASURE).
void init(KalmanFilter* kf, float Q_angle, float Q_bias, float R_measure);

// Update satu langkah filter. newAngle = sudut hasil accelerometer (deg),
// newRate = laju sudut dari gyro (deg/s), dt = interval waktu (detik).
// Return: estimasi sudut terbaru (deg).
float update(KalmanFilter* kf, float newAngle, float newRate, float dt);

} // namespace kalman

#endif // SHM_KALMAN_FILTER_H
