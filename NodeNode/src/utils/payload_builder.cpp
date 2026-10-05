#include "payload_builder.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include "../config/config.h"
#include "../sensors/rtc_ds3231.h"

namespace payload {

static void setTimestamp(JsonDocument& doc) {
    char ts[30];
    rtc::getDateTimeString(ts, sizeof(ts));   // ISO datetime penuh, biar new Date() valid di frontend
    doc["timestamp"] = ts;
}

size_t buildPeriodicPayload(const ProcessedData* batch, uint16_t batch_count,
                             uint32_t boot_id, uint32_t packet_seq,
                             bool wifi_connected, char* out, size_t out_size) {
    if (batch == nullptr || batch_count == 0) return 0;

    JsonDocument doc;
    const ProcessedData& latest = batch[batch_count - 1];

    doc["schema_version"] = 1;
    doc["node_id"] = NODE_ID;
    setTimestamp(doc);
    doc["sampling_rate_hz"] = latest.sampling_rate_hz;
    doc["connection_status"] = wifi_connected ? "online" : "offline";

    // 1. Live dashboard fields (RMS getaran, Tilt Kalman delta, snapshot XYZ)
    JsonObject vib = doc["vibration"].to<JsonObject>();
    vib["rms"] = latest.rms_vibration;

    JsonObject tilt = doc["tilt"].to<JsonObject>();
    tilt["pitch"] = latest.pitch;
    tilt["roll"] = latest.roll;
    tilt["pitch_delta"] = latest.pitch_delta;
    tilt["roll_delta"] = latest.roll_delta;

    JsonObject mag = doc["magnetometer"].to<JsonObject>();
    mag["mag_x"] = 0.0f;
    mag["mag_y"] = 0.0f;
    mag["mag_z"] = 0.0f;

    // Snapshot raw sensor data terkini untuk kartu dashboard Home
    JsonObject accel = doc["accelerometer"].to<JsonObject>();
    accel["x"] = latest.accel_x;
    accel["y"] = latest.accel_y;
    accel["z"] = latest.accel_z;

    if (latest.accel_calibrated) {
        JsonObject accel_cal = doc["accelerometer_calibrated"].to<JsonObject>();
        accel_cal["x"] = latest.accel_x_cal;
        accel_cal["y"] = latest.accel_y_cal;
        accel_cal["z"] = latest.accel_z_cal;
    }

    JsonObject gyro = doc["gyroscope"].to<JsonObject>();
    gyro["x"] = latest.gyro_x;
    gyro["y"] = latest.gyro_y;
    gyro["z"] = latest.gyro_z;

    if (latest.calibration_version > 0) {
        JsonObject cal = doc["calibration"].to<JsonObject>();
        cal["version"] = latest.calibration_version;
        JsonArray off = cal["accel_offset"].to<JsonArray>();
        off.add(latest.accel_offset[0]);
        off.add(latest.accel_offset[1]);
        off.add(latest.accel_offset[2]);
        JsonArray scl = cal["accel_scale"].to<JsonArray>();
        scl.add(latest.accel_scale[0]);
        scl.add(latest.accel_scale[1]);
        scl.add(latest.accel_scale[2]);
    }

    // 2. Raw time-series 200Hz archive (semua sampel utuh per 1 detik)
    doc["boot_id"] = boot_id;
    doc["packet_seq"] = packet_seq;
    doc["first_sample_seq"] = batch[0].sequence;
    doc["sample_count"] = batch_count;
    // t0_us: epoch-us dari detik RTC (resolusi 1 detik, kasar). TIDAK untuk menyejajarkan node.
    doc["t0_us"] = batch[0].timestamp * 1000000ULL;
    // dt_us mengikuti laju sampling aktif (bisa diubah 50..200 Hz lewat remote config).
    {
        uint16_t hz = latest.sampling_rate_hz ? latest.sampling_rate_hz : 200;
        doc["dt_us"] = 1000000UL / hz;
    }
    // mono_*: waktu monotonik esp_timer, mikrodetik SEJAK BOOT (bukan epoch). Basis waktu
    // berbeda dengan t0_us; jangan dicampur. Selisih (mono_last - mono_first) / (n - 1)
    // = dt nyata antar sampel.
    doc["mono_first_us"] = batch[0].t_us;
    doc["mono_last_us"] = batch[batch_count - 1].t_us;
    // Nomor urut sampel terakhir. Jika (last - first) != (sample_count - 1),
    // ada sampel yang terbuang (queue penuh) di tengah batch ini.
    doc["last_sample_seq"] = batch[batch_count - 1].sequence;

    JsonArray arrAx = doc["ax"].to<JsonArray>();
    JsonArray arrAy = doc["ay"].to<JsonArray>();
    JsonArray arrAz = doc["az"].to<JsonArray>();
    for (uint16_t i = 0; i < batch_count; i++) {
        arrAx.add(batch[i].accel_x);
        arrAy.add(batch[i].accel_y);
        arrAz.add(batch[i].accel_z);
    }

    size_t len = serializeJson(doc, out, out_size);
    return len;
}

size_t buildFFTPayload(const float* window, uint16_t window_size,
                        uint16_t sampling_rate_hz,
                        uint64_t window_start_ms, uint64_t window_end_ms,
                        char* out, size_t out_size) {
    JsonDocument doc;

    doc["node_id"] = NODE_ID;
    setTimestamp(doc);
    doc["window_size"] = window_size;
    doc["sampling_rate_hz"] = sampling_rate_hz;
    doc["window_start_ms"] = window_start_ms;
    doc["window_end_ms"] = window_end_ms;

    JsonArray arr = doc["raw_accel"].to<JsonArray>();
    for (uint16_t i = 0; i < window_size; i++) {
        arr.add(window[i]);
    }

    size_t len = serializeJson(doc, out, out_size);
    return len;
}

size_t buildStatusPayload(uint32_t uptime_s, uint32_t free_heap,
                           int8_t wifi_rssi, bool sd_ok, bool mqtt_ok,
                           float drop_rate_pct, char* out, size_t out_size) {
    JsonDocument doc;

    doc["node_id"] = NODE_ID;
    doc["uptime_s"] = uptime_s;
    doc["free_heap"] = free_heap;
    doc["wifi_rssi"] = wifi_rssi;
    doc["wifi_ssid"] = WiFi.SSID().c_str();
    doc["wifi_ip"] = WiFi.localIP().toString().c_str();
    doc["sd_ok"] = sd_ok;
    doc["mqtt_ok"] = mqtt_ok;
    doc["drop_rate_pct"] = drop_rate_pct;

    size_t len = serializeJson(doc, out, out_size);
    return len;
}

} // namespace payload
