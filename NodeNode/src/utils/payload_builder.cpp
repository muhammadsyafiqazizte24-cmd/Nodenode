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

size_t buildPeriodicPayload(const ProcessedData& data, bool wifi_connected,
                             char* out, size_t out_size) {
    JsonDocument doc;

    doc["node_id"] = NODE_ID;
    setTimestamp(doc);
    doc["sampling_rate_hz"] = data.sampling_rate_hz;
    doc["connection_status"] = wifi_connected ? "online" : "offline";

    JsonObject vib = doc["vibration"].to<JsonObject>();
    vib["rms"] = data.rms_vibration;

    JsonObject tilt = doc["tilt"].to<JsonObject>();
    tilt["pitch"] = data.pitch;
    tilt["roll"] = data.roll;
    tilt["pitch_delta"] = data.pitch_delta;
    tilt["roll_delta"] = data.roll_delta;

    JsonObject mag = doc["magnetometer"].to<JsonObject>();
    mag["mag_x"] = 0.0f;
    mag["mag_y"] = 0.0f;
    mag["mag_z"] = 0.0f;

    // Raw accel/gyro snapshot untuk dashboard "Raw Sensor Data"
    JsonObject accel = doc["accelerometer"].to<JsonObject>();
    accel["x"] = data.accel_x;
    accel["y"] = data.accel_y;
    accel["z"] = data.accel_z;

    // Calibrated accel (jika kalibrasi tersedia)
    if (data.accel_calibrated) {
        JsonObject accel_cal = doc["accelerometer_calibrated"].to<JsonObject>();
        accel_cal["x"] = data.accel_x_cal;
        accel_cal["y"] = data.accel_y_cal;
        accel_cal["z"] = data.accel_z_cal;
    }

    JsonObject gyro = doc["gyroscope"].to<JsonObject>();
    gyro["x"] = data.gyro_x;
    gyro["y"] = data.gyro_y;
    gyro["z"] = data.gyro_z;

    // Calibration info
    if (data.calibration_version > 0) {
        JsonObject cal = doc["calibration"].to<JsonObject>();
        cal["version"] = data.calibration_version;
        JsonArray off = cal["accel_offset"].to<JsonArray>();
        off.add(data.accel_offset[0]);
        off.add(data.accel_offset[1]);
        off.add(data.accel_offset[2]);
        JsonArray scl = cal["accel_scale"].to<JsonArray>();
        scl.add(data.accel_scale[0]);
        scl.add(data.accel_scale[1]);
        scl.add(data.accel_scale[2]);
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
