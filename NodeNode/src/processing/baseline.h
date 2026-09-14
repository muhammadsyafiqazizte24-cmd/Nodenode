#ifndef SHM_BASELINE_H
#define SHM_BASELINE_H

#include <Arduino.h>
#include "../config/config.h"

// ============================================================================
// baseline.h — Baseline pitch/roll untuk aplikasi SHM
//
// SHM lebih membutuhkan PERUBAHAN sudut (delta) dibanding sudut absolut,
// karena yang ingin dideteksi adalah pergeseran struktur relatif terhadap
// kondisi awal terpasang, bukan orientasi absolut sensor terhadap gravitasi.
// ============================================================================

class BaselineManager {
public:
    BaselineManager() : baseline_pitch(0), baseline_roll(0),
                         sample_count(0), is_calibrated(false),
                         sum_pitch(0), sum_roll(0) {}

    // Dipanggil setiap sample selama proses kalibrasi berlangsung (akumulasi
    // rata-rata). Otomatis menandai is_calibrated=true setelah mencapai
    // CALIBRATION_SAMPLES sample.
    void feedSample(float pitch, float roll) {
        if (is_calibrated) return;
        sum_pitch += pitch;
        sum_roll  += roll;
        sample_count++;
        if (sample_count >= CALIBRATION_SAMPLES) {
            baseline_pitch = sum_pitch / sample_count;
            baseline_roll  = sum_roll  / sample_count;
            is_calibrated = true;
        }
    }

    // Reset untuk memulai ulang kalibrasi (dipicu command "recalibrate").
    void reset() {
        sample_count = 0;
        sum_pitch = 0;
        sum_roll = 0;
        is_calibrated = false;
        // baseline_pitch/roll SENGAJA tidak di-nol-kan di sini supaya nilai
        // lama masih valid dipakai sampai kalibrasi baru selesai (menghindari
        // delta melompat ke nilai mentah saat proses baseline baru berjalan).
    }

    float getDeltaPitch(float pitch) const {
        return is_calibrated ? (pitch - baseline_pitch) : 0.0f;
    }

    float getDeltaRoll(float roll) const {
        return is_calibrated ? (roll - baseline_roll) : 0.0f;
    }

    bool isCalibrated() const { return is_calibrated; }
    uint8_t getSampleCount() const { return sample_count; }
    float getBaselinePitch() const { return baseline_pitch; }
    float getBaselineRoll() const { return baseline_roll; }

    static const uint8_t CALIBRATION_SAMPLES = BASELINE_CALIBRATION_SAMPLES;

private:
    float baseline_pitch, baseline_roll;
    uint8_t sample_count;
    bool is_calibrated;
    float sum_pitch, sum_roll;
};

#endif // SHM_BASELINE_H
