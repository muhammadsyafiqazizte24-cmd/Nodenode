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
//
// TAMBAHAN: timestamp awal dan akhir window (epoch millis) untuk
// sinkronisasi FDD antar node.
// ============================================================================

class FFTBuffer {
public:
    static const uint16_t BUFFER_SIZE = FFT_BUFFER_SIZE;

    FFTBuffer() : write_index(0), is_ready(false), window_start_ms(0), window_end_ms(0) {
        memset(buffer, 0, sizeof(buffer));
    }

    // Tambahkan satu sample (magnitude accel Z, sudah dikurangi gravitasi
    // dilakukan oleh caller jika diperlukan — modul ini murni ring buffer).
    // timestamp_ms adalah epoch millis saat sample diambil.
    void addSample(float value, uint64_t timestamp_ms) {
        if (write_index == 0) {
            window_start_ms = timestamp_ms;
        }
        buffer[write_index] = value;
        write_index = (write_index + 1) % BUFFER_SIZE;
        window_end_ms = timestamp_ms;
        if (write_index == 0) {
            is_ready = true;   // satu putaran penuh selesai
        }
    }

    // Salin isi buffer ke `output` (harus berkapasitas BUFFER_SIZE).
    // Return true & clear flag `is_ready` jika buffer memang sudah penuh.
    // Disalin dalam urutan kronologis (paling lama -> paling baru) supaya
    // backend menerima window yang berurutan meski write_index sedang di
    // tengah buffer saat fungsi ini dipanggil.
    // Juga mengembalikan timestamp awal dan akhir window.
    bool getBuffer(float* output, uint64_t& out_start_ms, uint64_t& out_end_ms) {
        if (!is_ready) return false;

        for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
            uint16_t src = (write_index + i) % BUFFER_SIZE;
            output[i] = buffer[src];
        }
        out_start_ms = window_start_ms;
        out_end_ms = window_end_ms;
        is_ready = false;
        return true;
    }

    bool ready() const { return is_ready; }

private:
    float buffer[FFT_BUFFER_SIZE];   // static allocation
    uint16_t write_index;
    bool is_ready;
    uint64_t window_start_ms;
    uint64_t window_end_ms;
};

#endif // SHM_FFT_BUFFER_H
