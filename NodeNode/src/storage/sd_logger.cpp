#include "sd_logger.h"
#include <SPI.h>
#include <SD.h>
#include "../config/config.h"
#include "../sensors/rtc_ds3231.h"
#include "../utils/spi_manager.h"
#include "../scheduler/task_manager.h"

// ============================================================================
// sd_logger.cpp -- BUS SPI GLOBAL dipakai bersama dengan MPU9250 (satu bus
// VSPI: SCK=18/MISO=23/MOSI=19, lihat utils/spi_manager.h). TIDAK ADA
// SPIClass kedua (HSPI) di sini -- SD Card memakai objek `SPI` global yang
// SAMA, yang sudah diinisialisasi oleh spimgr::init() (dipanggil sekali di
// main.cpp SEBELUM mpu9250::init() maupun sdlog::init()).
//
// SETIAP operasi yang menyentuh SPI (SD.begin/open/exists/mkdir,
// File.read/write/flush/close) WAJIB dibungkus spimgr::spiMutex, dengan
// MPU_CS dipaksa HIGH (deselect) sebelum SD_CS di-LOW-kan. Ini mengikuti
// pola CS-handling yang sama dengan mpu9250.cpp, hanya arah sebaliknya.
//
// sdFileMutex TETAP dipakai sebagai lapisan proteksi TERPISAH: ia menjaga
// STATE internal modul ini (currentFile, writeBuffer, readFile) dari akses
// bersamaan antar-task (SD Writer vs Sync Manager) -- bukan bus fisik.
// Urutan locking selalu: sdFileMutex (outer) dulu, baru spiMutex (inner)
// di dalamnya -- konsisten di semua fungsi untuk menghindari deadlock.
// ============================================================================

namespace sdlog {

static bool sd_ready = false;
static File currentFile;
static char currentFileName[64] = {0};
static unsigned long fileOpenedAtMs = 0;

static SDRecord writeBuffer[SD_WRITE_BATCH_SIZE];
static uint8_t writeBufferCount = 0;
static unsigned long lastFlushMs = 0;

// ---- Read cursor state (dipakai SyncManager) ----
static File readFile;
static bool readCursorActive = false;

// ---------------------------------------------------------------------------
// Helper CS-handling untuk akses SD di bus bersama. Dipanggil SELALU
// berpasangan, mengelilingi SETIAP operasi SD.*/File.* yang menyentuh SPI.
// ---------------------------------------------------------------------------
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

static void buildFileName(char* out, size_t outsize) {
    char dateStr[16];
    rtc::getDateString(dateStr, sizeof(dateStr));
    uint8_t hh = rtc::getHour();
    snprintf(out, outsize, "%s/%s_%s_%02u.bin", SD_LOG_DIR, NODE_ID, dateStr, hh);
}

// Dipanggil HANYA dari dalam blok yang sudah memegang sdFileMutex.
static bool openForAppend() {
    char newName[64];
    buildFileName(newName, sizeof(newName));

    if (strcmp(newName, currentFileName) == 0 && currentFile) {
        return true;   // masih file yang sama, tidak perlu buka ulang
    }

    if (!beginSDAccess()) return false;
    if (currentFile) currentFile.close();
    strncpy(currentFileName, newName, sizeof(currentFileName));
    currentFile = SD.open(currentFileName, FILE_APPEND);
    endSDAccess();

    fileOpenedAtMs = millis();
    return (bool)currentFile;
}

bool init() {
    if (sdFileMutex == nullptr) {
        sdFileMutex = xSemaphoreCreateMutex();
    }

    if (xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    if (!beginSDAccess()) {
        xSemaphoreGive(sdFileMutex);
        return false;
    }
    // SD.begin() memakai objek `SPI` GLOBAL yang sudah di-begin() oleh
    // spimgr::init() -- TIDAK membuat SPIClass baru, TIDAK memanggil
    // SPI.begin() lagi.
    bool mounted = SD.begin(SD_CS_PIN, SPI, 1000000U);
    if (mounted && !SD.exists(SD_LOG_DIR)) {
        SD.mkdir(SD_LOG_DIR);
    }
    endSDAccess();

    xSemaphoreGive(sdFileMutex);

    sd_ready = mounted;
    if (sd_ready) {
        sd_ready = openForAppend();
    }
    lastFlushMs = millis();
    return sd_ready;
}

bool isReady() { return sd_ready; }

void flush() {
    if (!sd_ready || writeBufferCount == 0) return;

    if (xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    if (openForAppend()) {
        if (!beginSDAccess()) {
            xSemaphoreGive(sdFileMutex);
            return;
        }
        currentFile.write((const uint8_t*)writeBuffer, sizeof(SDRecord) * writeBufferCount);
        currentFile.flush();
        endSDAccess();
    }
    xSemaphoreGive(sdFileMutex);

    writeBufferCount = 0;
    lastFlushMs = millis();
}

void writeRecord(const SDRecord& rec) {
    if (!sd_ready) return;

    writeBuffer[writeBufferCount++] = rec;

    bool batchFull = (writeBufferCount >= SD_WRITE_BATCH_SIZE);
    bool timeUp = (millis() - lastFlushMs >= SD_FLUSH_INTERVAL_MS);

    if (batchFull || timeUp) {
        flush();
    }
}

void checkRotation() {
    if (!sd_ready || !currentFile) return;

    bool timeExceeded = (millis() - fileOpenedAtMs >= SD_FILE_ROTATE_INTERVAL_MS);

    if (xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    if (!beginSDAccess()) {
        xSemaphoreGive(sdFileMutex);
        return;
    }
    bool sizeExceeded = (currentFile.size() >= SD_FILE_ROTATE_MAX_BYTES);
    endSDAccess();

    if (timeExceeded || sizeExceeded) {
        if (!beginSDAccess()) {
            xSemaphoreGive(sdFileMutex);
            return;
        }
        currentFile.close();
        endSDAccess();

        currentFileName[0] = '\0';   // paksa openForAppend() membuat/buka file baru
        openForAppend();
    }

    xSemaphoreGive(sdFileMutex);
}

bool readNextUnsent(uint32_t afterSequence, SDRecord& outRec) {
    if (!sd_ready) return false;

    if (xSemaphoreTake(sdFileMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    if (!readCursorActive) {
        if (!beginSDAccess()) {
            xSemaphoreGive(sdFileMutex);
            return false;
        }
        readFile = SD.open(currentFileName, FILE_READ);
        endSDAccess();
        readCursorActive = (bool)readFile;
    }

    if (!readCursorActive) {
        xSemaphoreGive(sdFileMutex);
        return false;
    }

    SDRecord rec;
    bool found = false;

    if (!beginSDAccess()) {
        xSemaphoreGive(sdFileMutex);
        return false;
    }
    while (readFile.available() >= (int)sizeof(SDRecord)) {
        readFile.read((uint8_t*)&rec, sizeof(SDRecord));
        if (rec.sequence > afterSequence) {
            outRec = rec;
            found = true;
            break;
        }
    }
    endSDAccess();

    if (!found) {
        if (!beginSDAccess()) {
            xSemaphoreGive(sdFileMutex);
            return false;
        }
        readFile.close();
        endSDAccess();
        readCursorActive = false;
    }

    xSemaphoreGive(sdFileMutex);
    return found;
}

} // namespace sdlog
