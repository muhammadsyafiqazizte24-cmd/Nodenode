#ifndef SHM_SPI_MANAGER_H
#define SHM_SPI_MANAGER_H

#include <Arduino.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ============================================================================
// spi_manager.h — Pusat SPI bus tunggal (VSPI) untuk MPU9250 & SD Card
//
// Firmware ini memakai HANYA SATU bus SPI fisik (SCK=18, MISO=23, MOSI=19),
// dipakai bersama oleh MPU9250 (CS=5) dan SD Card (CS=32). TIDAK ADA
// SPIClass kedua (HSPI) di mana pun di project ini — semua driver memakai
// objek global Arduino `SPI`.
//
// Modul ini bertanggung jawab untuk:
//   1. Membuat `spiMutex` global (SATU-SATUNYA mutex bus SPI di seluruh
//      firmware — dipakai oleh mpu9250.cpp DAN sd_logger.cpp).
//   2. Menginisialisasi `SPI.begin()` TEPAT SATU KALI selama boot.
//   3. Men-set kedua pin CS (MPU & SD) ke OUTPUT + HIGH sebelum SPI.begin()
//      dipanggil, supaya tidak ada device yang "nyangkut" aktif di bus.
//
// Driver individual (mpu9250.cpp, sd_logger.cpp) TIDAK BOLEH memanggil
// SPI.begin() lagi, dan TIDAK BOLEH membuat mutex/SPIClass sendiri —
// mereka hanya memakai `spimgr::spiMutex` dan objek `SPI` global ini.
// ============================================================================

namespace spimgr {

// Mutex SATU-SATUNYA untuk seluruh bus SPI. WAJIB dipakai oleh setiap
// fungsi yang melakukan transfer SPI ke MPU9250 maupun SD Card — tidak
// boleh ada transfer SPI tanpa mutex ini.
extern SemaphoreHandle_t spiMutex;

// Dipanggil SEKALI dari main.cpp::setup(), sebelum mpu9250::init() maupun
// sdlog::init(). Menyiapkan pin CS + memulai bus SPI + membuat mutex.
void init();

} // namespace spimgr

#endif // SHM_SPI_MANAGER_H
