#include "mqtt_manager.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "wifi_manager.h"
#include "../config/config.h"
#include "../config/mqtt_config.h"
#include "../utils/payload_builder.h"
#include "../scheduler/task_manager.h"
#include "../sync/sync_manager.h"

namespace mqttmgr {

static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static unsigned long last_reconnect_attempt_ms = 0;

// ---------------------------------------------------------------------------
// Callback subscribe: parse topic suffix -> ConfigCommand -> push ke
// configCommandQueue. Callback PubSubClient berjalan di context task
// wifi_mqtt_reconnect (yang memanggil mqttClient.loop()), BUKAN task
// tersendiri — parsing dibuat ringan & non-blocking supaya tidak menunda
// task lain. Eksekusi command sesungguhnya dilakukan oleh Config Handler
// task yang membaca queue ini secara asynchronous.
// ---------------------------------------------------------------------------
static void onMessage(char* topic, byte* payloadBytes, unsigned int length) {
    char payloadStr[32];
    size_t n = min(length, (unsigned int)(sizeof(payloadStr) - 1));
    memcpy(payloadStr, payloadBytes, n);
    payloadStr[n] = '\0';

    String t(topic);
    ConfigCommand cmd;
    cmd.value = 0;
    bool recognized = true;

    if (t == TOPIC_CMD_SAMPLING_RATE) {
        cmd.type = ConfigCommandType::SET_SAMPLING_RATE;
        cmd.value = (uint32_t)atoi(payloadStr);
    } else if (t == TOPIC_CMD_PUBLISH_INTERVAL) {
        cmd.type = ConfigCommandType::SET_PUBLISH_INTERVAL;
        cmd.value = (uint32_t)atol(payloadStr);
    } else if (t == TOPIC_CMD_RAW_WINDOW_INTERVAL) {
        cmd.type = ConfigCommandType::SET_RAW_WINDOW_INTERVAL;
        cmd.value = (uint32_t)atol(payloadStr);
    } else if (t == TOPIC_CMD_RECALIBRATE) {
        cmd.type = ConfigCommandType::RECALIBRATE;
    } else if (t == TOPIC_CMD_CALIBRATE_ACCEL) {
        cmd.type = ConfigCommandType::CALIBRATE_ACCEL;
    } else if (t == TOPIC_CMD_RESTART) {
        cmd.type = ConfigCommandType::RESTART;
    } else if (t == TOPIC_CMD_REQUEST_STATUS) {
        cmd.type = ConfigCommandType::REQUEST_STATUS;
    } else if (t == TOPIC_CMD_SENSITIVITY_GAIN) {
        cmd.type = ConfigCommandType::SET_SENSITIVITY_GAIN;
        cmd.value = (uint32_t)atoi(payloadStr);
    } else {
        recognized = false;
    }

    if (recognized) {
        // Non-blocking send (timeout 0) — jika queue penuh, command di-drop
        // (command berikutnya dari operator biasanya akan diulang secara
        // manual; ini lebih aman daripada memblokir callback MQTT).
        xQueueSend(configCommandQueue, &cmd, 0);
    }
}

void init() {
    mqttClient.setServer(mqttBroker, MQTT_BROKER_PORT);
    mqttClient.setCallback(onMessage);
    mqttClient.setKeepAlive(MQTT_KEEPALIVE_S);
    // Buffer diperbesar supaya cukup untuk payload FFT (256 sample -> ~2-4KB
    // JSON). Default PubSubClient (256 byte) tidak cukup.
    mqttClient.setBufferSize(4096);
}

bool isConnected() {
    return mqttClient.connected();
}

void loop() {
    if (!wifimgr::isConnected()) {
        xEventGroupClearBits(wifimgr::netEventGroup, MQTT_CONNECTED_BIT);
        return;
    }

    if (!mqttClient.connected()) {
        xEventGroupClearBits(wifimgr::netEventGroup, MQTT_CONNECTED_BIT);

        unsigned long now = millis();
        if (now - last_reconnect_attempt_ms < MQTT_RECONNECT_BACKOFF_MS) {
            return;
        }
        last_reconnect_attempt_ms = now;

        // Last-Will: broker menandai node offline otomatis jika koneksi
        // TCP putus tanpa disconnect bersih (mati listrik / crash lapangan).
        bool ok = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD,
                                      TOPIC_LWT, 1, true, "offline");
        if (ok) {
            Serial.printf("[MQTT] Connected to %s\n", mqttBroker);
            mqttClient.publish(TOPIC_LWT, "online", true);
            mqttClient.subscribe(TOPIC_CMD_WILDCARD);
            xEventGroupSetBits(wifimgr::netEventGroup, MQTT_CONNECTED_BIT);
        } else {
            Serial.printf("[MQTT] Reconnect failed to %s, rc=%d\n", mqttBroker, mqttClient.state());

            // Baru saja reconnect -> picu sinkronisasi data SD yang belum
            // terkirim (dieksekusi oleh task Sync Manager sendiri, di sini
            // hanya set event bit / langsung panggil karena syncAfterReconnect()
            // sudah non-blocking secara kooperatif via vTaskDelay).
            syncmgr::syncAfterReconnect();
        }
        return;
    }

    xEventGroupSetBits(wifimgr::netEventGroup, MQTT_CONNECTED_BIT);
    mqttClient.loop();
}

bool publishPeriodic(const ProcessedData& data) {
    if (!mqttClient.connected()) return false;
    static char buf[1024];   // static -> cukup utk JSON + calibration, tidak bebani stack task
    size_t len = payload::buildPeriodicPayload(data, wifimgr::isConnected(), buf, sizeof(buf));
    if (len == 0 || len >= sizeof(buf)) return false;
    return mqttClient.publish(TOPIC_DATA_PERIODIC, (const uint8_t*)buf, len, false);
}

bool publishFFTWindow(const float* window, uint16_t window_size,
                      uint16_t sampling_rate_hz,
                      uint64_t window_start_ms, uint64_t window_end_ms) {
    if (!mqttClient.connected()) return false;
    static char buf[4096];   // static -> tidak membebani stack task 8192-word
    size_t len = payload::buildFFTPayload(window, window_size, sampling_rate_hz, window_start_ms, window_end_ms, buf, sizeof(buf));
    if (len == 0) return false;
    return mqttClient.publish(TOPIC_FFT_BUFFER, (const uint8_t*)buf, len, false);
}

bool publishStatus(uint32_t uptime_s, uint32_t free_heap, int8_t wifi_rssi,
                    bool sd_ok, float drop_rate_pct) {
    if (!mqttClient.connected()) return false;
    char buf[384];
    size_t len = payload::buildStatusPayload(uptime_s, free_heap, wifi_rssi, sd_ok,
                                              mqttClient.connected(), drop_rate_pct,
                                              buf, sizeof(buf));
    if (len == 0) return false;
    return mqttClient.publish(TOPIC_STATUS, (const uint8_t*)buf, len, false);
}

} // namespace mqttmgr
