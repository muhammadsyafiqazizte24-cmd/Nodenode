#include "sd_logger.h"
#include <SPI.h>
#include <SD.h>
#include "../config/config.h"
#include "../utils/spi_manager.h"
#include "../scheduler/task_manager.h"

// ============================================================================
// sd_logger.cpp -- Logging SD Card berbasis sesi pengujian.
// SD Card hanya aktif menulis ketika ada sesi pengujian yang dimulai dari
// server dashboard. Di luar sesi, SD Card IDLE 100% sehingga tidak menyita
// bus SPI yang dipakai bersama dengan MPU9250.
// ============================================================================

namespace sdlog {

static bool sd_ready = false;
static bool sd_mounted = false;
static bool session_active = false;
static uint32_t current_session_id = 0;

static File currentFile;
static char currentFileName[64] = {0};

static SDRecord writeBuffer[SD_WRITE_BATCH_SIZE];
static uint8_t writeBufferCount = 0;
static unsigned long lastFlushMs = 0;

// Helper CS-handling untuk akses SD di bus bersama
static bool beginSDAccess() {
    if (xSemaphoreTake(spimgr::spiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("[SD] SPI mutex timeout; skipping SD access");
        sd_ready = false;
        return false;
    }
    digitalWrite(MPU_CS_PIN, HIGH);   // pastikan MPU9250 deselected
    digitalWrite(SD_CS_PIN, LOW);     // select SD Card
    return true;
}

static void endSDAccess() {
    digitalWrite(SD_CS_PIN, HIGH);    // deselect SD Card setelah selesai
    xSemaphoreGive(spimgr::spiMutex);
}

bool init() {
    if (sdFileMutex == nullptr) {
        sdFileMutex = xSemaphoreCreateMutex();
    }

    if (xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    if (sd_mounted) {
        if (currentFile) currentFile.close();
        currentFile = File();
        currentFileName[0] = '\0';
        writeBufferCount = 0;
        SD.end();
        sd_mounted = false;
    }

    if (!beginSDAccess()) {
        xSemaphoreGive(sdFileMutex);
        return false;
    }

    bool mounted = SD.begin(SD_CS_PIN, SPI, 1000000U);
    sd_mounted = mounted;
    if (mounted && !SD.exists(SD_LOG_DIR)) {
        if (!SD.mkdir(SD_LOG_DIR)) {
            Serial.printf("[SD] mkdir %s gagal\n", SD_LOG_DIR);
        }
    }
    endSDAccess();

    xSemaphoreGive(sdFileMutex);

    sd_ready = mounted;
    session_active = false;
    lastFlushMs = millis();

    if (sd_ready) {
        Serial.printf("[SD] Terdeteksi OK. Menunggu Start Session dari server. Folder: %s\n", SD_LOG_DIR);
    } else {
        Serial.println("[SD] init gagal - mode degraded (MQTT tetap jalan, log SD nonaktif)");
    }
    return sd_ready;
}

void checkRecovery() {
    if (sd_ready) return;
    static unsigned long lastRetryMs = 0;
    unsigned long now = millis();
    if (now - lastRetryMs < SD_RETRY_INTERVAL_MS) return;
    lastRetryMs = now;

    Serial.println("[SD] Mencoba re-init kartu...");
    if (init()) {
        Serial.println("[SD] Pulih - siap untuk sesi.");
    }
}

bool isReady() {
    return sd_ready;
}

bool isSessionActive() {
    return session_active;
}

bool startSession(uint32_t sessionId) {
    if (!sd_ready) return false;

    if (xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return false;
    }

    // Jika sudah ada sesi aktif sebelumnya, tutup dulu
    if (currentFile) {
        flush();
        if (beginSDAccess()) {
            currentFile.close();
            endSDAccess();
        }
    }

    current_session_id = sessionId;
    snprintf(currentFileName, sizeof(currentFileName), "%s/session_%lu.bin", SD_LOG_DIR, (unsigned long)sessionId);

    if (!beginSDAccess()) {
        xSemaphoreGive(sdFileMutex);
        return false;
    }

    currentFile = SD.open(currentFileName, FILE_WRITE);
    endSDAccess();

    session_active = (bool)currentFile;
    writeBufferCount = 0;
    lastFlushMs = millis();

    xSemaphoreGive(sdFileMutex);

    if (session_active) {
        Serial.printf("[SD] Sesi %lu DIMULAI -> menulis ke %s\n", (unsigned long)sessionId, currentFileName);
    } else {
        Serial.printf("[SD] Gagal membuka file sesi: %s\n", currentFileName);
    }
    return session_active;
}

void stopSession() {
    if (!session_active && !currentFile) return;

    if (xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        session_active = false;
        return;
    }

    // Flush sisa buffer yang belum ditulis
    flush();

    if (currentFile) {
        if (beginSDAccess()) {
            currentFile.close();
            endSDAccess();
        }
        currentFile = File();
    }

    session_active = false;
    writeBufferCount = 0;
    xSemaphoreGive(sdFileMutex);

    Serial.printf("[SD] Sesi %lu SELESAI. File ditutup, SD Card IDLE.\n", (unsigned long)current_session_id);
}

void flush() {
    if (!sd_ready || !currentFile || writeBufferCount == 0) return;

    if (!beginSDAccess()) {
        return;
    }
    currentFile.write((const uint8_t*)writeBuffer, sizeof(SDRecord) * writeBufferCount);
    currentFile.flush();
    endSDAccess();

    writeBufferCount = 0;
    lastFlushMs = millis();
}

void writeRecord(const SDRecord& rec) {
    if (!sd_ready || !session_active) return;

    writeBuffer[writeBufferCount++] = rec;

    bool batchFull = (writeBufferCount >= SD_WRITE_BATCH_SIZE);
    bool timeUp = (millis() - lastFlushMs >= SD_FLUSH_INTERVAL_MS);

    if (batchFull || timeUp) {
        if (xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            flush();
            xSemaphoreGive(sdFileMutex);
        }
    }
}

} // namespace sdlog
