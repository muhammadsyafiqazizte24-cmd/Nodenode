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

// Publish payload gabungan (pitch/roll/rms + 200 raw samples ax/ay/az)
// ke topic bridge/<node_id>/data per 1 detik.
bool publishPeriodic(const ProcessedData* batch, uint16_t batch_count,
                      uint32_t boot_id, uint32_t packet_seq);

// Overload untuk single ProcessedData (mis. saat sync replay dari SD)
bool publishPeriodic(const ProcessedData& single);

// Publish raw FFT window dengan timestamp presisi.
bool publishFFTWindow(const float* window, uint16_t window_size,
                      uint16_t sampling_rate_hz,
                      uint64_t window_start_ms, uint64_t window_end_ms);

// Publish balasan request_status.
bool publishStatus(uint32_t uptime_s, uint32_t free_heap, int8_t wifi_rssi,
                    bool sd_ok, float drop_rate_pct);

// Publish konfirmasi perintah konfigurasi (FR-04)
bool publishConfigAck(const char* param, uint32_t requested, uint32_t applied, const char* status);

} // namespace mqttmgr

#endif // SHM_MQTT_MANAGER_H
