/*
 * ============================================================================
 * Node Structural Health Monitoring (SHM) — Firmware ESP32
 * ============================================================================
 *
 * Node ini HANYA bertugas: akuisisi data sensor (MPU9250 via SPI,
 * register-level), preprocessing ringan (Kalman filter, baseline delta,
 * RMS getaran), logging lokal ke SD Card, dan pengiriman data ke MQTT.
 * FFT TIDAK dilakukan di sini — hanya raw window yang dikirim ke backend.
 *
 * Driver MPU9250 (SPI register-level), RTC DS3231 (I2C register-level), dan
 * Kalman Filter DIPERTAHANKAN persis secara logic dari program referensi;
 * lihat komentar "PERTAHANKAN" di masing-masing modul (src/sensors,
 * src/processing/kalman_filter.*).
 *
 * ============================================================================
 * DIAGRAM ALUR STARTUP
 * ============================================================================
 *
 *   [Power On / Reset]
 *          |
 *          v
 *   Serial.begin(115200)
 *          |
 *          v
 *   spimgr::init()          ----> SATU-SATUNYA SPI.begin() di seluruh
 *   (bus SPI global VSPI)          firmware; set CS MPU & SD ke HIGH dulu,
 *          |                       lalu buat spiMutex (dipakai bersama
 *          |                       MPU9250 & SD Card, lihat spi_manager.h)
 *          v
 *   RTC DS3231 init (I2C)  ----> jika OSF flag set: WARNING waktu tidak
 *          |                      valid, tunggu command "settime" via Serial
 *          v
 *   MPU9250 init (SPI)     ----> jika WHO_AM_I mismatch: HALT + retry loop
 *          |                      (node tidak bisa berfungsi tanpa sensor)
 *          v
 *   Gyro calibration        ----> warm-up 5s + averaging 500 sample
 *   (blocking, sekali saja)        (PERTAHANKAN dari referensi)
 *          |
 *          v
 *   SD Card init (bus SPI GLOBAL sama dgn MPU, CS beda) -> jika gagal:
 *          |                      lanjut "degraded mode" (data tetap
 *          |                      dikirim MQTT, tidak di-log ke SD)
 *          v
 *   Load sync checkpoint dari SD (last_sent_sequence)
 *          |
 *          v
 *   WiFi Manager init (non-blocking, event group)
 *          |
 *          v
 *   MQTT Manager init (server, callback, buffer size)
 *          |
 *          v
 *   taskmgr::createAll()
 *     |-- Queue & Mutex & Semaphore dibuat
 *     |-- 9 task FreeRTOS di-spawn sesuai priority map (freertos_config.h)
 *          |
 *          v
 *   Task Watchdog Timer (TWDT) diinisialisasi, subscribe task kritikal
 *          |
 *          v
 *   setup() selesai -> scheduler FreeRTOS mengambil alih sepenuhnya
 *          |
 *          v
 *   loop() KOSONG (semua kerja ada di task, sesuai prinsip "no blocking,
 *   no delay() di jalur utama" — loop() Arduino di ESP32 sendiri berjalan
 *   sebagai task FreeRTOS berprioritas rendah, dibiarkan idle/yield).
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "esp_task_wdt.h"
#include <nvs_flash.h>

#include "config/config.h"
#include "config/freertos_config.h"
#include "config/mqtt_config.h"

#include "sensors/mpu9250.h"
#include "sensors/rtc_ds3231.h"
#include "utils/spi_manager.h"

#include "communication/wifi_manager.h"
#include "communication/mqtt_manager.h"

#include "storage/sd_logger.h"
#include "sync/sync_manager.h"

#include "scheduler/task_manager.h"

void setup() {
    Serial.begin(115200);
    delay(300);   // jeda singkat sekali-jalan supaya monitor serial siap; BUKAN
                   // bagian dari loop real-time, jadi aman memakai delay() di sini.

    Serial.println("\n=== SHM Node Firmware ===");
    Serial.println("Node ID: " NODE_ID);

    pinMode(LED_WHITE_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    digitalWrite(LED_WHITE_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN, HIGH);

    // ---- SPI bus global (VSPI) — SATU-SATUNYA inisialisasi bus SPI ----
    // Dipakai bersama oleh MPU9250 & SD Card (lihat utils/spi_manager.h).
    // WAJIB dipanggil sebelum mpu9250::init() maupun sdlog::init().
    spimgr::init();
    Serial.println("SPI bus (VSPI, shared MPU9250+SD) siap.");

    // ---- NVS (untuk kalibrasi accel di mpu9250) ----
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // ---- RTC ----
    bool rtc_ok = rtc::init();
    if (!rtc_ok) {
        Serial.println("WARNING: RTC oscillator stop flag set - waktu mungkin tidak akurat.");
        Serial.println("Kirim 'settime HH:MM:SS' via Serial jika diperlukan.");
    }

    // ---- MPU9250 ----
    if (!mpu9250::init()) {
        Serial.println("FATAL: MPU9250 tidak terdeteksi! Periksa wiring SPI.");
        while (true) {
            delay(1000);   // halt loop sekali-jalan, node tidak berfungsi tanpa sensor utama
        }
    }
    Serial.println("MPU9250 OK.");

    // ---- Gyro calibration (blocking sekali-jalan, PERTAHANKAN dari referensi) ----
    Serial.println("Kalibrasi gyroscope (warm-up + averaging)...");
    mpu9250::calibrateGyro(GYRO_CALIBRATION_SAMPLES);
    Serial.println("Kalibrasi gyroscope selesai.");

    // ---- SD Card ----
    if (!sdlog::init()) {
        Serial.println("WARNING: SD Card tidak terdeteksi - berjalan dalam mode degraded "
                        "(logging lokal nonaktif, MQTT tetap berjalan).");
    } else {
        Serial.println("SD Card OK.");
        uint32_t lastSeq = syncmgr::loadCheckpoint();
        Serial.printf("Checkpoint sync terakhir: sequence=%lu\n", lastSeq);
    }

    // ---- WiFi & MQTT (non-blocking init; koneksi aktual ditangani task) ----
    wifimgr::init();
    mqttmgr::init();

    // ---- FreeRTOS: buat semua queue/mutex + spawn semua task ----
    taskmgr::createAll();

    // ---- Task Watchdog Timer ----
    esp_task_wdt_init(TASK_WATCHDOG_TIMEOUT_S, true /* panic -> restart jika timeout */);

    digitalWrite(LED_WHITE_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
    Serial.println("Semua task FreeRTOS aktif. Node SHM berjalan.\n");
}

void loop() {
    // SENGAJA KOSONG: seluruh pekerjaan node (sampling, processing, SD,
    // MQTT, sync, config) berjalan di task FreeRTOS masing-masing (lihat
    // scheduler/task_manager.cpp). loop() Arduino di ESP32 berjalan sebagai
    // task tersendiri berprioritas rendah — diberi delay panjang supaya
    // tidak menyita CPU secara sia-sia (busy-loop kosong).
    vTaskDelay(pdMS_TO_TICKS(100));
    
}
