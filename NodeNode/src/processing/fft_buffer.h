#ifndef SHM_FFT_BUFFER_H
#define SHM_FFT_BUFFER_H

#include <Arduino.h>
#include "../config/config.h"

// ============================================================================
// fft_buffer.h — Ring buffer raw sample untuk dikirim ke backend
//
// ESP32 TIDAK melakukan FFT sama sekali (sesuai spesifikasi) — modul ini
// hanya mengumpulkan magnitude accel axis Z (getaran vertikal) ke ring
// buffer statis lalu menandai `is_ready` saat buffer penuh, supaya FFT
// Buffer Sender task bisa mengambil & mengirimkannya lewat MQTT.
// ============================================================================

class FFTBuffer {
public:
    static const uint16_t BUFFER_SIZE = FFT_BUFFER_SIZE;

    FFTBuffer() : write_index(0), is_ready(false) {
        memset(buffer, 0, sizeof(buffer));
    }

    // Tambahkan satu sample (magnitude accel Z, sudah dikurangi gravitasi
    // dilakukan oleh caller jika diperlukan — modul ini murni ring buffer).
    void addSample(float value) {
        buffer[write_index] = value;
        write_index = (write_index + 1) % BUFFER_SIZE;
        if (write_index == 0) {
            is_ready = true;   // satu putaran penuh selesai
        }
    }

    // Salin isi buffer ke `output` (harus berkapasitas BUFFER_SIZE).
    // Return true & clear flag `is_ready` jika buffer memang sudah penuh.
    // Disalin dalam urutan kronologis (paling lama -> paling baru) supaya
    // backend menerima window yang berurutan meski write_index sedang di
    // tengah buffer saat fungsi ini dipanggil.
    bool getBuffer(float* output) {
        if (!is_ready) return false;

        for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
            uint16_t src = (write_index + i) % BUFFER_SIZE;
            output[i] = buffer[src];
        }
        is_ready = false;
        return true;
    }

    bool ready() const { return is_ready; }

private:
    float buffer[FFT_BUFFER_SIZE];   // static allocation
    uint16_t write_index;
    bool is_ready;
};

#endif // SHM_FFT_BUFFER_H
