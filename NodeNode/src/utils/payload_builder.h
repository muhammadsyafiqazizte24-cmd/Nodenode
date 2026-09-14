#ifndef SHM_PAYLOAD_BUILDER_H
#define SHM_PAYLOAD_BUILDER_H

#include <Arduino.h>
#include "data_structures.h"

// ============================================================================
// payload_builder.h — Bangun payload JSON MQTT sesuai kontrak spesifikasi
//
// Dipisah dari mqtt_manager supaya format payload mudah diaudit/diubah
// tanpa menyentuh logic koneksi/publish MQTT.
// ============================================================================

namespace payload {

// Payload periodik (dikirim tiap 1 detik / sesuai publish_interval):
// { node_id, timestamp, sampling_rate_hz, connection_status, pitch, roll,
//   pitch_delta, roll_delta, rms_vibration,
//   magnetometer:{mag_x,mag_y,mag_z},
//   accelerometer:{x,y,z}, gyroscope:{x,y,z} }
// Hasil ditulis ke `out` (buffer caller-provided), return panjang string
// (atau 0 jika buffer kurang besar) — TANPA alokasi heap dinamis supaya
// aman dipanggil berulang dari task MQTT Publisher 24/7.
size_t buildPeriodicPayload(const ProcessedData& data, bool wifi_connected,
                             char* out, size_t out_size);

// Payload FFT buffer (dikirim tiap 10-30 detik):
// { node_id, timestamp, raw_window: [...], window_size }
size_t buildFFTPayload(const float* window, uint16_t window_size,
                        uint32_t timestamp, char* out, size_t out_size);

// Payload status (balasan request_status):
// { node_id, uptime_s, free_heap, wifi_rssi, sd_ok, mqtt_ok, drop_rate }
size_t buildStatusPayload(uint32_t uptime_s, uint32_t free_heap,
                           int8_t wifi_rssi, bool sd_ok, bool mqtt_ok,
                           float drop_rate_pct, char* out, size_t out_size);

} // namespace payload

#endif // SHM_PAYLOAD_BUILDER_H
