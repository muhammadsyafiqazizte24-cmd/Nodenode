#ifndef SHM_FREERTOS_CONFIG_H
#define SHM_FREERTOS_CONFIG_H

// ============================================================================
// freertos_config.h — Task priority map, core assignment, stack sizes
//
// Priority scale mengikuti spesifikasi (10 = tertinggi). Arduino-ESP32 core
// memakai FreeRTOS dengan configMAX_PRIORITIES cukup besar (biasanya 25),
// jadi angka 2-10 di bawah aman dipakai langsung sebagai uxPriority.
// ============================================================================

// ---------------------------------------------------------------------------
// PRIORITAS TASK  (10 = tertinggi)
// ---------------------------------------------------------------------------
#define PRIO_SENSOR_SAMPLING       10   // Core 0 — hard real-time, 200Hz
#define PRIO_DATA_PROCESSING        9   // Core 0 — Kalman, RMS, baseline
#define PRIO_SD_WRITER               8   // Core 1 — logging persistent
#define PRIO_MQTT_PUBLISHER          7   // Core 1 — kirim data periodik
#define PRIO_WIFI_MQTT_RECONNECT     6   // Core 1 — jaga koneksi
#define PRIO_FFT_BUFFER_SENDER       5   // Core 1 — kirim buffer FFT
#define PRIO_CONFIG_HANDLER          4   // Core 1 — terima perintah remote
#define PRIO_SYNC_MANAGER            3   // Core 1 — sinkronisasi data
#define PRIO_SERIAL_DEBUG            2   // Core 1 — monitoring

// ---------------------------------------------------------------------------
// CORE ASSIGNMENT
// ---------------------------------------------------------------------------
// Core 0 didedikasikan HANYA untuk sampling + processing supaya jitter dari
// stack WiFi/MQTT (yang berjalan di Core 1) tidak pernah mengganggu window
// waktu sampling 5ms.
#define CORE_SAMPLING       0
#define CORE_PROCESSING     0
#define CORE_SD_WRITER      1
#define CORE_MQTT_PUBLISHER 1
#define CORE_WIFI_RECONNECT 1
#define CORE_FFT_SENDER     1
#define CORE_CONFIG_HANDLER 1
#define CORE_SYNC_MANAGER   1
#define CORE_SERIAL_DEBUG   1

// ---------------------------------------------------------------------------
// STACK SIZES (dalam WORDS, bukan byte — FreeRTOS pada ESP32 pakai satuan
// word = 4 byte untuk xTaskCreatePinnedToCore. Nilai di bawah sesuai
// spesifikasi: Sampling 4096, Processing 8192, SD Writer 4096, MQTT 6144,
// FFT 8192 words).
// ---------------------------------------------------------------------------
#define STACK_SENSOR_SAMPLING      4096
#define STACK_DATA_PROCESSING      8192
#define STACK_SD_WRITER            4096
#define STACK_MQTT_PUBLISHER       6144
#define STACK_WIFI_RECONNECT       4096
#define STACK_FFT_SENDER           8192
#define STACK_CONFIG_HANDLER       4096
#define STACK_SYNC_MANAGER         4096
#define STACK_SERIAL_DEBUG         2048

// ---------------------------------------------------------------------------
// QUEUE LENGTHS
// ---------------------------------------------------------------------------
#define QLEN_RAW_SENSOR_DATA     40   // ~200ms buffer @200Hz sebelum processing
#define QLEN_PROCESSED_DATA      40
#define QLEN_SD_WRITE             40
#define QLEN_MQTT_PUBLISH          10
#define QLEN_CONFIG_COMMAND        10

// ---------------------------------------------------------------------------
// TASK TIMING (referensi, dipakai di scheduler/task_manager.cpp)
// ---------------------------------------------------------------------------
//  Task                  Interval                 Deadline        Core
//  Sensor Sampling        5ms   (200Hz, remote-cfg) Hard RT          0
//  Data Processing        event-driven (queue)      <2ms             0
//  SD Card Write          100ms batch flush check    <10ms            1
//  MQTT Publish           1s (configurable)          <100ms           1
//  FFT Buffer Send        10-30s (configurable)      <1s              1
//  WiFi/MQTT Reconnect    5s check                   -                1
//  Config Handler         event-driven (MQTT cb)     -                1
//  Sync Manager           setelah reconnect          low priority     1

// ---------------------------------------------------------------------------
// PERFORMANCE ESTIMATE (analisis, lihat juga README.md)
// ---------------------------------------------------------------------------
// Jitter sampling:
//   Target            : ±50us (1% dari periode 5ms @200Hz)
//   Worst-case block   : SPI read (~100us) + I2C RTC sync 1x/detik (~200us,
//                        di luar loop sampling) + xQueueSend (~5us) +
//                        context switch (~15us) = ~120us << 5000us (aman)
//
// CPU usage (estimasi):
//   Core 0 : ~21% (sampling ~6% + processing ~15%)
//   Core 1 : ~17% (SD ~10% + MQTT ~5% + lainnya ~2%)
//   Total  : <40% → aman untuk operasi 24/7 dengan headroom besar
//
// RAM budget (estimasi statis, tanpa heap fragmentation):
//   FFT buffer (256 x float)        : 1,024 bytes
//   RMS buffer (200 x float)        : 800 bytes
//   Queue buffers (5 queue)         : ~2,000 bytes
//   Task stacks (9 task, total words*4): ~44,000 bytes
//   ------------------------------------------------
//   Total                            : ~48 KB dari 520 KB SRAM (~9.2%)
//
// Watchdog:
//   Task Watchdog Timer (TWDT)      : 10 detik timeout
//   Subscriber                       : task Sampling, Processing, SD Writer
//   Aksi jika hang                   : ESP.restart() via panic handler /
//                                       esp_task_wdt callback (lihat main.cpp)

#endif // SHM_FREERTOS_CONFIG_H
