#include "sync_manager.h"
#include <SD.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "../config/config.h"
#include "../storage/sd_logger.h"
#include "../communication/mqtt_manager.h"
#include "../utils/data_structures.h"

namespace syncmgr {

static uint32_t last_sent_sequence = 0;
static uint32_t checkpoint_dirty_count = 0;

uint32_t loadCheckpoint() {
    if (!SD.exists(SD_CHECKPOINT_FILE)) {
        last_sent_sequence = 0;
        return 0;
    }

    File f = SD.open(SD_CHECKPOINT_FILE, FILE_READ);
    if (!f) { last_sent_sequence = 0; return 0; }

    uint32_t seq = 0;
    if (f.available() >= (int)sizeof(uint32_t)) {
        f.read((uint8_t*)&seq, sizeof(uint32_t));
    }
    f.close();

    last_sent_sequence = seq;
    return seq;
}

void saveCheckpoint() {
    File f = SD.open(SD_CHECKPOINT_FILE, FILE_WRITE);
    if (!f) return;
    f.seek(0);
    f.write((const uint8_t*)&last_sent_sequence, sizeof(uint32_t));
    f.close();
}

void markAsSent(uint32_t sequence) {
    if (sequence <= last_sent_sequence) return;   // JANGAN mundur / kirim ulang
    last_sent_sequence = sequence;

    // Throttle penulisan checkpoint ke SD supaya tidak menulis flash setiap
    // 1 record (mengurangi wear & overhead I/O) — cukup tiap 20 record atau
    // saat syncAfterReconnect() selesai (lihat pemanggilan eksplisit di bawah).
    if (++checkpoint_dirty_count >= 20) {
        saveCheckpoint();
        checkpoint_dirty_count = 0;
    }
}

uint32_t getLastSentSequence() {
    return last_sent_sequence;
}

void syncAfterReconnect() {
    if (!sdlog::isReady() || !mqttmgr::isConnected()) return;

    SDRecord rec;
    uint32_t synced_count = 0;
    const uint32_t max_batch = 10;
    const uint32_t max_time_ms = 1500UL;
    uint32_t start_ms = millis();

    while (mqttmgr::isConnected() && synced_count < max_batch &&
           (millis() - start_ms) < max_time_ms &&
           sdlog::readNextUnsent(last_sent_sequence, rec)) {

        ProcessedData pd;
        pd.sequence = rec.sequence;
        pd.timestamp = rec.timestamp;
        pd.pitch = rec.pitch;
        pd.roll = rec.roll;
        pd.pitch_delta = rec.pitch_delta;
        pd.roll_delta = rec.roll_delta;
        pd.rms_vibration = rec.rms_vibration;
        pd.sampling_rate_hz = rec.sampling_rate_hz;

        if (mqttmgr::publishPeriodic(pd)) {
           markAsSent(rec.sequence);
           synced_count++;
        } else {
           break;   // MQTT gagal publish (mis. buffer PubSubClient penuh) -> stop, coba lagi nanti
        }

        // Batasi durasi batch agar task lain (SD writer, MQTT publisher,
        // sampling) tetap punya waktu CPU. Tanpa batas ini backlog besar
        // bisa memonopoli Core 1 dan memicu watchdog.
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (synced_count > 0) {
        saveCheckpoint();   // pastikan checkpoint akhir tersimpan walau belum 20 record
    }
}

} // namespace syncmgr
