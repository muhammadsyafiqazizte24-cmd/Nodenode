#ifndef SHM_CONFIG_H
#define SHM_CONFIG_H

#include <Arduino.h>

// ============================================================================
// config.h — Pin definitions & konstanta umum untuk seluruh firmware
//
// PENTING: Logika pembacaan MPU9250 & RTC DS3231 (register-level) DIPERTAHANKAN
// persis dari program referensi. Yang berubah dari revisi terakhir adalah
// ARSITEKTUR BUS SPI: MPU9250 & SD Card kini berbagi SATU bus SPI global
// (VSPI: SCK=18/MISO=23/MOSI=19), dibedakan lewat pin CS masing-masing.
// Lihat utils/spi_manager.h untuk detail pusat inisialisasi & mutex bus.
// ============================================================================

// ---------------------------------------------------------------------------
// Identitas Node
// ---------------------------------------------------------------------------
// Unique ID per node — nilai default (fallback) di bawah. Saat deployment,
// override lewat build flag per environment di platformio.ini:
//   pio run -e node_01 --target upload   (NODE_ID = node_01)
//   pio run -e node_02 --target upload   (NODE_ID = node_02)
// Gunakan nilai node_01 atau node_02 agar sinkron dengan backend SHMS.
#ifndef NODE_ID
#define NODE_ID "node_01"
#endif

// ---------------------------------------------------------------------------
// SPI bus GLOBAL TUNGGAL — dipakai bersama oleh MPU9250 dan SD Card.
// Hanya SATU bus VSPI di seluruh firmware (lihat utils/spi_manager.h);
// TIDAK ADA SPIClass kedua (HSPI) di project ini. Device dibedakan murni
// lewat pin CS masing-masing.
// ---------------------------------------------------------------------------
#define MPU_SCK_PIN    18
#define MPU_MISO_PIN   23
#define MPU_MOSI_PIN   19
#define MPU_SPI_CLOCK_HZ  1000000UL   // 1 MHz, sama seperti referensi

#define MPU_CS_PIN     5
#define SD_CS_PIN      17

// ---------------------------------------------------------------------------
// RTC DS3231 — I2C (PERTAHANKAN, sama dengan program referensi)
// ---------------------------------------------------------------------------
#define RTC_I2C_ADDR   0x68
#define RTC_SDA_PIN    21
#define RTC_SCL_PIN    22
#define RTC_SYNC_INTERVAL_MS 1000UL

// Sinkronisasi waktu via NTP saat WiFi connect. DS3231 diset ke UTC (offset 0)
// supaya timestamp payload konsisten dengan backend/frontend yang memakai UTC.
#define NTP_SERVER            "pool.ntp.org"
#define NTP_GMT_OFFSET_SEC    0
#define NTP_DAYLIGHT_OFFSET_SEC 0
#define NTP_SYNC_TIMEOUT_MS   15000UL

// ---------------------------------------------------------------------------
// Sampling & filter (PERTAHANKAN nilai kalibrasi/kalman dari referensi;
// SAMPLE_RATE_DEFAULT_HZ di bawah menggantikan SAMPLE_INTERVAL_US 250Hz
// pada referensi, sesuai requirement Node SHM = default 200Hz).
// ---------------------------------------------------------------------------
#define GYRO_CALIBRATION_SAMPLES   500
#define WARMUP_DURATION_MS         5000
#define DT_MAX                     0.05f

#define KALMAN_Q_ANGLE   0.001f
#define KALMAN_Q_BIAS    0.003f
#define KALMAN_R_MEASURE 0.03f

#define GYRO_DEADBAND_DPS      0.6f
#define ACCEL_STATIONARY_TOL   0.03f
#define GYRO_STATIONARY_DPS    1.5f
#define ADAPTIVE_BIAS_ALPHA    0.0005f

// Sampling rate default & batas yang diizinkan lewat remote config
#define SAMPLE_RATE_DEFAULT_HZ   200
#define SAMPLE_RATE_MIN_HZ       50
#define SAMPLE_RATE_MAX_HZ       200

// ---------------------------------------------------------------------------
// Baseline
// ---------------------------------------------------------------------------
#define BASELINE_CALIBRATION_SAMPLES   100

// ---------------------------------------------------------------------------
// RMS vibration — window 1 detik @ 200Hz = 200 sample
// ---------------------------------------------------------------------------
#define RMS_WINDOW_SIZE   200

// ---------------------------------------------------------------------------
// Raw FFT buffer (dikirim mentah ke backend, FFT dilakukan di server)
// ---------------------------------------------------------------------------
#define FFT_BUFFER_SIZE          256
#define FFT_SEND_INTERVAL_DEFAULT_MS  15000UL   // 15 detik (range 10-30s)
#define FFT_SEND_INTERVAL_MIN_MS      10000UL
#define FFT_SEND_INTERVAL_MAX_MS      30000UL

// ---------------------------------------------------------------------------
// MQTT publish periodik (pitch/roll/rms)
// ---------------------------------------------------------------------------
#define MQTT_PUBLISH_INTERVAL_DEFAULT_MS  500UL
#define MQTT_PUBLISH_INTERVAL_MIN_MS      500UL
#define MQTT_PUBLISH_INTERVAL_MAX_MS      10000UL

// ---------------------------------------------------------------------------
// SD Card logging
// ---------------------------------------------------------------------------
#define SD_WRITE_BATCH_SIZE     10       // flush setiap 10 sample terkumpul
#define SD_FLUSH_INTERVAL_MS    500UL    // atau setiap 500ms, mana lebih dulu
#define SD_FILE_ROTATE_INTERVAL_MS  (60UL * 60UL * 1000UL)  // 1 jam
#define SD_FILE_ROTATE_MAX_BYTES    (10UL * 1024UL * 1024UL) // 10MB
#define SD_RETRY_INTERVAL_MS        30000UL                   // re-init tiap 30 dtk saat SD down
#define SD_LOG_DIR               "/shm_logs"
#define SD_CHECKPOINT_FILE       "/shm_logs/checkpoint.dat"

// ---------------------------------------------------------------------------
// WiFi reconnect
// ---------------------------------------------------------------------------
#define WIFI_CHECK_INTERVAL_MS   5000UL
#define WIFI_CONNECT_TIMEOUT_MS  15000UL

// ---------------------------------------------------------------------------
// LED status
// ---------------------------------------------------------------------------
#define LED_WHITE_PIN             27
#define LED_GREEN_PIN             26
#define LED_RED_PIN               25

// ---------------------------------------------------------------------------
// Config reset / AP trigger
// ---------------------------------------------------------------------------
#define CONFIG_RESET_PIN          0
#define CONFIG_RESET_HOLD_MS      5000UL
#define CONFIG_DEFAULT_MQTT_BROKER "192.168.254.126"
#define EEPROM_AT24C256_ADDR      0x53
#define EEPROM_BROKER_OFFSET      0

#define SENSITIVITY_GAIN_DEFAULT   1
// ---------------------------------------------------------------------------
// Watchdog
// ---------------------------------------------------------------------------
#define TASK_WATCHDOG_TIMEOUT_S   10

#endif // SHM_CONFIG_H

