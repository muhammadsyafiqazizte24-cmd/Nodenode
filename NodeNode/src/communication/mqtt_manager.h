#ifndef SHM_MQTT_MANAGER_H
#define SHM_MQTT_MANAGER_H

#include <Arduino.h>
#include "../utils/data_structures.h"

// ============================================================================
// mqtt_manager.h — Klien MQTT (PubSubClient) untuk publish & subscribe
//
// Semua fungsi di sini TIDAK BOLEH dipanggil dari task Sensor Sampling /
// Data Processing (Core 0) — hanya dipanggil dari task-task Core 1
// (MQTT Publisher, FFT Buffer Sender, WiFi/MQTT Reconnect, Config Handler)
// supaya latency jaringan tidak pernah membebani jalur sampling real-time.
// ============================================================================

namespace mqttmgr {

// Inisialisasi client (server, port, callback). Dipanggil sekali dari
// main.cpp / task wifi_mqtt_reconnect sebelum loop dimulai.
void init();

// Coba connect (jika belum) & jalankan mqttClient.loop() untuk memproses
// pesan masuk (callback). Dipanggil periodik dari task
// wifi_mqtt_reconnect. Update bit MQTT_CONNECTED_BIT di netEventGroup.
void loop();

bool isConnected();

// Publish payload periodik (pitch/roll/rms). Return false jika gagal
// (misal MQTT sedang disconnect) — caller (MQTT Publisher task)
// bertanggung jawab tetap menulis ke SD terlepas dari hasil ini.
bool publishPeriodic(const ProcessedData& data);

// Publish raw FFT window.
bool publishFFTWindow(const float* window, uint16_t window_size, uint32_t timestamp);

// Publish balasan request_status.
bool publishStatus(uint32_t uptime_s, uint32_t free_heap, int8_t wifi_rssi,
                    bool sd_ok, float drop_rate_pct);

} // namespace mqttmgr

#endif // SHM_MQTT_MANAGER_H
