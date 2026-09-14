#include "payload_builder.h"
#include <ArduinoJson.h>
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

    JsonObject gyro = doc["gyroscope"].to<JsonObject>();
    gyro["x"] = data.gyro_x;
    gyro["y"] = data.gyro_y;
    gyro["z"] = data.gyro_z;

    size_t len = serializeJson(doc, out, out_size);
    return len;
}

size_t buildFFTPayload(const float* window, uint16_t window_size,
                        uint32_t timestamp, char* out, size_t out_size) {
    JsonDocument doc;

    doc["node_id"] = NODE_ID;
    setTimestamp(doc);
    doc["window_size"] = window_size;
    doc["sampling_rate_hz"] = SAMPLE_RATE_DEFAULT_HZ;

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
    doc["sd_ok"] = sd_ok;
    doc["mqtt_ok"] = mqtt_ok;
    doc["drop_rate_pct"] = drop_rate_pct;

    size_t len = serializeJson(doc, out, out_size);
    return len;
}

} // namespace payload
