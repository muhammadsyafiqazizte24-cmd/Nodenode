#ifndef SHM_DATA_STRUCTURES_H
#define SHM_DATA_STRUCTURES_H

#include <Arduino.h>

// ============================================================================
// data_structures.h — Tipe data yang mengalir lewat FreeRTOS Queue antar task
//
// Semua struct di sini didesain POD (plain-old-data) supaya aman dikirim
// lewat xQueueSend/xQueueReceive dengan copy langsung (tanpa pointer/heap),
// konsisten dengan aturan "hindari dynamic memory allocation setelah init".
// ============================================================================

// ---------------------------------------------------------------------------
// Data mentah hasil satu kali sampling MPU9250 (dikirim: Sampling -> Processing)
// ---------------------------------------------------------------------------
struct MPU9250Data {
    // Accelerometer (g)
    float accel_x, accel_y, accel_z;
    // Gyroscope (deg/s), sudah dikurangi bias (kalibrasi + adaptive bias)
    float gyro_x, gyro_y, gyro_z;
    // Magnetometer (uT) - dibaca & disimpan, belum dipakai untuk fusion.
    // Field ini SENGAJA disediakan sejak awal supaya penambahan yaw
    // (magnetometer fusion) di masa depan tidak perlu mengubah struct/queue.
    float mag_x, mag_y, mag_z;
    // Timestamp dari RTC (epoch seconds, lihat rtc_ds3231.cpp)
    uint32_t timestamp;
    // dt aktual (detik) sejak sample sebelumnya - dibutuhkan processing task
    // untuk Kalman/RMS/FFT tanpa perlu akses ulang ke timer sampling.
    float dt;
    // Sequence number untuk tracking pengiriman & sinkronisasi SD->MQTT
    uint32_t sequence;
    // Epoch millis untuk timestamp presisi tinggi (FDD sync)
    uint64_t epoch_ms;
};

// ---------------------------------------------------------------------------
// Hasil processing (Kalman + baseline delta + RMS) siap dikirim ke
// SD Writer maupun MQTT Publisher (dikirim: Processing -> SD/MQTT)
// ---------------------------------------------------------------------------
struct ProcessedData {
    uint32_t sequence;
    uint32_t timestamp;
    float pitch;
    float roll;
    float pitch_delta;     // pitch - baseline_pitch
    float roll_delta;      // roll  - baseline_roll
    float rms_vibration;   // valid hanya setiap genap 200 sample; di luar itu
                            // berisi nilai RMS window terakhir yang selesai
    uint16_t sampling_rate_hz;
    // Raw accel (g) & gyro (deg/s) sample terbaru — diteruskan dari
    // MPU9250Data untuk dashboard "Raw Sensor Data". Tidak dipakai dalam
    // perhitungan Kalman/RMS/FFT (hanya snapshot untuk display).
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    // Calibrated accel (g) — hasil koreksi offset/skala
    bool accel_calibrated;
    float accel_x_cal, accel_y_cal, accel_z_cal;
    // Calibration info untuk payload
    uint8_t calibration_version;
    float accel_offset[3];
    float accel_scale[3];
};

// ---------------------------------------------------------------------------
// Perintah konfigurasi remote (dikirim: MQTT callback -> Config Handler ->
// task terkait via queue masing-masing)
// ---------------------------------------------------------------------------
enum class ConfigCommandType : uint8_t {
    SET_SAMPLING_RATE = 0,
    SET_PUBLISH_INTERVAL,
    SET_RAW_WINDOW_INTERVAL,
    SET_SENSITIVITY_GAIN,
    RECALIBRATE,
    RESTART,
    REQUEST_STATUS,
    CALIBRATE_ACCEL
};

struct ConfigCommand {
    ConfigCommandType type;
    uint32_t value;   // makna tergantung `type` (Hz, ms, atau tidak dipakai)
};

#endif // SHM_DATA_STRUCTURES_H
