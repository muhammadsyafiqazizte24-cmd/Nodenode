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

// Payload periodik gabungan ke bridge/<node_id>/data (dikirim tiap 1 detik):
// Memuat summary live dashboard (RMS, tilt Kalman, snapshot XYZ)
// DAN 200 raw samples ax/ay/az (@200Hz) dalam satu JSON tunggal.
size_t buildPeriodicPayload(const ProcessedData* batch, uint16_t batch_count,
                             uint32_t boot_id, uint32_t packet_seq,
                             bool wifi_connected, char* out, size_t out_size);

// Payload FFT buffer (dikirim tiap 10-30 detik):
// { node_id, timestamp, raw_accel:[...], window_size, sampling_rate_hz,
//   window_start_ms, window_end_ms }
size_t buildFFTPayload(const float* window, uint16_t window_size,
                        uint16_t sampling_rate_hz,
                        uint64_t window_start_ms, uint64_t window_end_ms,
                        char* out, size_t out_size);

// Payload status (balasan request_status):
// { node_id, uptime_s, free_heap, wifi_rssi, sd_ok, mqtt_ok, drop_rate }
size_t buildStatusPayload(uint32_t uptime_s, uint32_t free_heap,
                           int8_t wifi_rssi, bool sd_ok, bool mqtt_ok,
                           float drop_rate_pct, char* out, size_t out_size);

} // namespace payload

#endif // SHM_PAYLOAD_BUILDER_H
