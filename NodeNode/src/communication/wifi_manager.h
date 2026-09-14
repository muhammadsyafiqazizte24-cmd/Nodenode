#ifndef SHM_WIFI_MANAGER_H
#define SHM_WIFI_MANAGER_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// ============================================================================
// wifi_manager.h — Koneksi & auto-reconnect WiFi
//
// Status koneksi dipublikasikan lewat Event Group (WIFI_CONNECTED_BIT) supaya
// task lain (MQTT Publisher, Sync Manager) bisa menunggu/poll status tanpa
// polling langsung ke WiFi.status().
// ============================================================================

namespace wifimgr {

extern EventGroupHandle_t netEventGroup;

// Bit definitions pada netEventGroup (dipakai bersama mqtt_manager.h)
#define WIFI_CONNECTED_BIT   (1 << 0)
#define MQTT_CONNECTED_BIT   (1 << 1)
#define SD_READY_BIT         (1 << 2)
#define BASELINE_READY_BIT   (1 << 3)
#define SAMPLING_ACTIVE_BIT  (1 << 4)

// Inisialisasi event group (dipanggil sekali dari main.cpp sebelum task
// dibuat) + mulai koneksi WiFi non-blocking.
void init();

// Dipanggil periodik (5 detik, lihat freertos_config.h) dari task
// wifi_mqtt_reconnect. Melakukan reconnect jika status WiFi terputus, dan
// meng-update bit event group sesuai status terkini.
void checkAndReconnect();

bool isConnected();

} // namespace wifimgr

#endif // SHM_WIFI_MANAGER_H
