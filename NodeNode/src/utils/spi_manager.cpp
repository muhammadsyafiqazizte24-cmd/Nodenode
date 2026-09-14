#include "spi_manager.h"
#include "../config/config.h"

namespace spimgr {

SemaphoreHandle_t spiMutex = nullptr;

void init() {
    // ---- CS kedua device di-set OUTPUT + HIGH SEBELUM SPI.begin() ----
    // Supaya tidak ada device yang ter-select tanpa sengaja saat bus mulai
    // aktif. Urutan ini penting: pinMode/digitalWrite dulu, baru SPI.begin().
    pinMode(MPU_CS_PIN, OUTPUT);
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(MPU_CS_PIN, HIGH);
    digitalWrite(SD_CS_PIN, HIGH);

    // ---- SPI.begin() — SATU-SATUNYA pemanggilan di seluruh firmware ----
    // Tidak diberi argumen SS di sini karena bus dipakai bersama dua
    // device; CS masing-masing dikendalikan MANUAL oleh setiap driver
    // (lihat mpu9250.cpp / sd_logger.cpp) — bukan diserahkan ke default
    // SS milik SPIClass.
    SPI.begin(MPU_SCK_PIN, MPU_MISO_PIN, MPU_MOSI_PIN);

    // ---- Mutex bus (real FreeRTOS mutex, BUKAN binary semaphore) ----
    // xSemaphoreCreateMutex() mendukung priority inheritance: jika task
    // prioritas rendah (mis. SD Writer, prio 8) sedang memegang spiMutex
    // dan task prioritas tinggi (Sensor Sampling, prio 10) menunggu mutex
    // yang sama, FreeRTOS otomatis menaikkan sementara prioritas si
    // pemegang mutex supaya ia cepat selesai & melepas lock — mengurangi
    // priority inversion dibanding binary semaphore biasa.
    spiMutex = xSemaphoreCreateMutex();
}

} // namespace spimgr
