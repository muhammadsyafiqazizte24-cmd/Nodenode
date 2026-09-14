#ifndef SHM_RMS_CALCULATOR_H
#define SHM_RMS_CALCULATOR_H

#include <Arduino.h>
#include "../config/config.h"

// ============================================================================
// rms_calculator.h — RMS getaran dari magnitude accelerometer
//
// Window 1 detik (RMS_WINDOW_SIZE sample @ sampling rate saat ini). Komponen
// gravitasi (1g) dihilangkan sebelum RMS dihitung supaya hasil merefleksikan
// getaran murni, bukan orientasi statis sensor.
// ============================================================================

class RMSCalculator {
public:
    RMSCalculator() : index(0), filled(0), sum_sq(0.0f) {
        memset(buffer, 0, sizeof(buffer));
    }

    // Tambahkan satu sample accel (g). Menghitung magnitude, mengurangi
    // komponen gravitasi, lalu memasukkannya ke circular buffer.
    // Mengembalikan RMS window SAAT INI (bergerak / sliding), sehingga
    // pemanggil bebas memilih memakai nilai tiap sample (sliding) atau
    // hanya mengambil nilainya tiap genap WINDOW_SIZE sample (tumbling,
    // sesuai spesifikasi "hitung RMS tiap 1 detik").
    float addSample(float ax, float ay, float az) {
        float mag = sqrtf(ax * ax + ay * ay + az * az) - GRAVITY_G;

        // Update sum-of-squares secara incremental (buang kontribusi sample
        // lama yang akan tertimpa, tambahkan kontribusi sample baru) supaya
        // RMS tidak perlu menjumlah ulang seluruh buffer tiap panggilan
        // (penting untuk menjaga beban CPU task processing tetap ringan).
        float old_val = buffer[index];
        sum_sq -= (old_val * old_val);
        sum_sq += (mag * mag);

        buffer[index] = mag;
        index = (index + 1) % WINDOW_SIZE;
        if (filled < WINDOW_SIZE) filled++;

        return calculateRMS();
    }

    float calculateRMS() const {
        if (filled == 0) return 0.0f;
        float mean_sq = sum_sq / (float)filled;
        return sqrtf(mean_sq > 0.0f ? mean_sq : 0.0f);
    }

    bool isWindowFull() const { return filled >= WINDOW_SIZE; }

    static const uint16_t WINDOW_SIZE = RMS_WINDOW_SIZE;

private:
    static constexpr float GRAVITY_G = 1.0f;
    float buffer[RMS_WINDOW_SIZE];   // static allocation, circular buffer
    uint16_t index;
    uint16_t filled;
    float sum_sq;
};

#endif // SHM_RMS_CALCULATOR_H
