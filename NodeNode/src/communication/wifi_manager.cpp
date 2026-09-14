#include "wifi_manager.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <Wire.h>
#include "../config/config.h"
#include "../config/mqtt_config.h"
#include "../sensors/rtc_ds3231.h"

char mqttBroker[16] = CONFIG_DEFAULT_MQTT_BROKER;   // default, bisa diubah via portal

namespace wifimgr {

EventGroupHandle_t netEventGroup = nullptr;
static unsigned long last_attempt_ms = 0;

static void saveConfig();

static bool eepromWriteText(uint16_t offset, const char* value) {
    if (value == nullptr) return false;

    if (rtc::i2cMutex != nullptr) {
        if (xSemaphoreTake(rtc::i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            return false;
        }
    }

    uint8_t payload[16] = {0};
    size_t len = strlen(value);
    if (len > sizeof(payload) - 1) {
        len = sizeof(payload) - 1;
    }
    memcpy(payload, value, len);

    Wire.beginTransmission(EEPROM_AT24C256_ADDR);
    Wire.write((uint8_t)((offset >> 8) & 0xFF));
    Wire.write((uint8_t)(offset & 0xFF));
    for (size_t i = 0; i < sizeof(payload); ++i) {
        Wire.write(payload[i]);
    }
    bool ok = (Wire.endTransmission() == 0);

    if (rtc::i2cMutex != nullptr) {
        xSemaphoreGive(rtc::i2cMutex);
    }
    return ok;
}

static bool eepromReadText(uint16_t offset, char* out, size_t out_len) {
    if (out == nullptr || out_len == 0) return false;
    out[0] = '\0';

    if (rtc::i2cMutex != nullptr) {
        if (xSemaphoreTake(rtc::i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            return false;
        }
    }

    uint8_t payload[16] = {0};

    Wire.beginTransmission(EEPROM_AT24C256_ADDR);
    Wire.write((uint8_t)((offset >> 8) & 0xFF));
    Wire.write((uint8_t)(offset & 0xFF));
    if (Wire.endTransmission(false) != 0) {
        if (rtc::i2cMutex != nullptr) xSemaphoreGive(rtc::i2cMutex);
        return false;
    }

    if (Wire.requestFrom((int)EEPROM_AT24C256_ADDR, (int)sizeof(payload)) != sizeof(payload)) {
        if (rtc::i2cMutex != nullptr) xSemaphoreGive(rtc::i2cMutex);
        return false;
    }

    for (size_t i = 0; i < sizeof(payload) && Wire.available(); ++i) {
        payload[i] = Wire.read();
    }

    size_t valid = 0;
    while (valid < sizeof(payload) && payload[valid] != 0xFF && payload[valid] != '\0') {
        valid++;
    }

    if (valid == 0) {
        if (rtc::i2cMutex != nullptr) xSemaphoreGive(rtc::i2cMutex);
        return false;
    }

    if (valid > out_len - 1) {
        valid = out_len - 1;
    }
    memcpy(out, payload, valid);
    out[valid] = '\0';

    if (rtc::i2cMutex != nullptr) {
        xSemaphoreGive(rtc::i2cMutex);
    }
    return true;
}

// Custom parameter WiFiManager untuk MQTT Broker IP
static WiFiManager wm;
static WiFiManagerParameter mqttParam("mqtt", "MQTT Broker IP", mqttBroker, 16);

static void clearBrokerConfig() {
    Preferences prefs;
    prefs.begin("shm-node", false);
    prefs.remove("mqtt_broker");
    prefs.end();
    eepromWriteText(EEPROM_BROKER_OFFSET, CONFIG_DEFAULT_MQTT_BROKER);
    strlcpy(mqttBroker, CONFIG_DEFAULT_MQTT_BROKER, sizeof(mqttBroker));
    mqttParam.setValue(mqttBroker, sizeof(mqttBroker));
    Serial.println("[WiFi] Broker IP config reset ke default.");
}

static bool shouldResetBrokerConfig() {
    pinMode(CONFIG_RESET_PIN, INPUT_PULLUP);

    unsigned long start_ms = millis();
    while (millis() - start_ms < CONFIG_RESET_HOLD_MS) {
        if (digitalRead(CONFIG_RESET_PIN) == LOW) {
            Serial.println("[WiFi] Pin 0 aktif LOW selama 5 detik: reset konfigurasi broker dan masuk AP mode.");
            return true;
        }
        delay(20);
    }
    return false;
}

static void saveConfigCallback() {
    strlcpy(mqttBroker, mqttParam.getValue(), sizeof(mqttBroker));
    eepromWriteText(EEPROM_BROKER_OFFSET, mqttBroker);

    Preferences prefs;
    prefs.begin("shm-node", false);
    prefs.putString("mqtt_broker", mqttBroker);
    prefs.end();
}

void init() {
    if (netEventGroup == nullptr) {
        netEventGroup = xEventGroupCreate();
    }

    pinMode(CONFIG_RESET_PIN, INPUT_PULLUP);

    // Load MQTT broker dari EEPROM AT24C256 terlebih dahulu, lalu fallback ke NVS.
    {
        char eepromBroker[16] = {0};
        if (eepromReadText(EEPROM_BROKER_OFFSET, eepromBroker, sizeof(eepromBroker)) && eepromBroker[0] != '\0') {
            strlcpy(mqttBroker, eepromBroker, sizeof(mqttBroker));
            mqttParam.setValue(mqttBroker, sizeof(mqttBroker));
        }
    }

    {
        Preferences prefs;
        prefs.begin("shm-node", true);
        String saved = prefs.getString("mqtt_broker", "");
        prefs.end();
        if (saved.length() > 0) {
            strlcpy(mqttBroker, saved.c_str(), sizeof(mqttBroker));
            mqttParam.setValue(mqttBroker, 16);
            eepromWriteText(EEPROM_BROKER_OFFSET, mqttBroker);
        }
    }

    wm.setSaveParamsCallback(saveConfigCallback);
    wm.setTitle("SHM Node Config");
    wm.addParameter(&mqttParam);
    wm.setConfigPortalTimeout(180);   // AP mati setelah 3 menit
    wm.setBreakAfterConfig(true);
    wm.setWiFiAutoReconnect(false);

    bool reset_requested = shouldResetBrokerConfig();
    if (reset_requested) {
        wm.resetSettings();
        clearBrokerConfig();
        WiFi.disconnect();
        WiFi.mode(WIFI_STA);
        Serial.println("[WiFi] Memulai portal konfigurasi AP...");
        bool portal_ok = wm.startConfigPortal("SHM-Node", "shm12345");
        if (!portal_ok) {
            Serial.println("[WiFi] Portal AP tidak bisa dibuka. Restarting...");
            ESP.restart();
        }
    } else {
        bool ok = wm.autoConnect("SHM-Node");
        if (!ok) {
            Serial.println("[WiFi] Gagal connect! Memulai AP fallback...");
            wm.resetSettings();
            wm.startConfigPortal("SHM-Node", "shm12345");
        }
    }

    Serial.printf("[WiFi] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
    saveConfigCallback();

    last_attempt_ms = millis();
    xEventGroupSetBits(netEventGroup, WIFI_CONNECTED_BIT);
}

bool isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

void checkAndReconnect() {
    if (isConnected()) {
        xEventGroupSetBits(netEventGroup, WIFI_CONNECTED_BIT);
        return;
    }

    xEventGroupClearBits(netEventGroup, WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT);

    unsigned long now = millis();
    if (now - last_attempt_ms < 5000) return;

    last_attempt_ms = now;
    WiFi.reconnect();
}

} // namespace wifimgr
