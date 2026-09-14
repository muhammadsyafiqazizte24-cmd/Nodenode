#ifndef SHM_TASK_MANAGER_H
#define SHM_TASK_MANAGER_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "../utils/data_structures.h"

// ============================================================================
// task_manager.h — Pusat definisi Queue/Semaphore/Task, dan state konfigurasi
// remote yang dipakai lintas task (sampling rate, publish interval, dst.)
//
// File ini adalah "wiring diagram" FreeRTOS firmware: semua primitve
// inter-task communication dideklarasikan di sini (extern), didefinisikan
// di task_manager.cpp, dipakai oleh modul-modul lain (mqtt_manager,
// sd_logger, sync_manager, dll) tanpa saling include silang.
// ============================================================================

// ---------------------------------------------------------------------------
// QUEUES
// ---------------------------------------------------------------------------
extern QueueHandle_t rawDataQueue;        // Sampling -> Processing (MPU9250Data)
extern QueueHandle_t sdWriteQueue;        // Processing -> SD Writer (ProcessedData)
extern QueueHandle_t mqttPublishQueue;    // Processing -> MQTT Publisher (ProcessedData)
extern QueueHandle_t configCommandQueue;  // MQTT callback -> Config Handler (ConfigCommand)

// ---------------------------------------------------------------------------
// SEMAPHORES / MUTEXES
// (spiMutex ada di mpu9250 namespace, i2cMutex ada di rtc namespace —
//  dideklarasikan di modul masing-masing supaya locality jelas. Mutex
//  lintas-modul lain dideklarasikan di sini.)
// ---------------------------------------------------------------------------
extern SemaphoreHandle_t sdFileMutex;     // proteksi akses file SD
extern SemaphoreHandle_t fftBufferMutex;  // proteksi FFTBuffer (ditulis di
                                            // Core0/processing, dibaca di
                                            // Core1/fft_sender)

// ---------------------------------------------------------------------------
// SHARED CONFIG STATE (diubah oleh Config Handler task via command MQTT,
// dibaca oleh task lain). Memakai tipe `volatile` karena hanya berupa nilai
// skalar sederhana yang dibaca/ditulis atomic pada arsitektur 32-bit ESP32;
// tidak butuh mutex terpisah untuk word-aligned read/write tunggal ini.
// ---------------------------------------------------------------------------
extern volatile uint16_t g_samplingRateHz;         // default: SAMPLE_RATE_DEFAULT_HZ
extern volatile uint8_t  g_sensitivityGain;        // default: SENSITIVITY_GAIN_DEFAULT
extern volatile uint32_t g_publishIntervalMs;       // default: MQTT_PUBLISH_INTERVAL_DEFAULT_MS
extern volatile uint32_t g_fftSendIntervalMs;       // default: FFT_SEND_INTERVAL_DEFAULT_MS
extern volatile bool     g_recalibrateRequested;    // di-set true oleh config handler,
                                                      // dibaca & di-clear oleh processing task
extern volatile bool     g_restartRequested;
extern volatile bool     g_statusRequested;

// ---------------------------------------------------------------------------
// STATISTIK (untuk payload status / watchdog monitoring)
// ---------------------------------------------------------------------------
extern volatile uint32_t g_sequenceCounter;   // sequence number global, increment tiap sample
extern volatile uint32_t g_droppedSamples;    // sample yang gagal masuk queue (queue penuh)

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------
namespace taskmgr {

// Membuat semua queue/mutex/event group + men-spawn seluruh task FreeRTOS
// sesuai priority map & core assignment di freertos_config.h.
// Dipanggil sekali dari main.cpp::setup(), setelah semua driver (MPU9250,
// RTC, SD, WiFi) selesai diinisialisasi.
void createAll();

} // namespace taskmgr

#endif // SHM_TASK_MANAGER_H
