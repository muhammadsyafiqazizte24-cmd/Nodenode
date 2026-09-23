#ifndef SHM_MPU9250_H
#define SHM_MPU9250_H

#include <Arduino.h>
#include <SPI.h>

// ============================================================================
// mpu9250.h — Driver SPI register-level untuk MPU9250
//
// PERTAHANKAN: seluruh LOGIKA pembacaan sensor (kalibrasi, adaptive bias,
// deadband) IDENTIK dengan program referensi. Yang berubah HANYA mekanisme
// akses bus SPI: bus kini dipakai BERSAMA dengan SD Card (satu bus VSPI
// global, lihat utils/spi_manager.h), jadi setiap transfer SPI di sini
// WAJIB melalui `spimgr::spiMutex` dan memastikan SD_CS HIGH selama
// komunikasi ke MPU9250.
//
// CATATAN: modul ini TIDAK LAGI memanggil SPI.begin() atau membuat mutex
// sendiri — itu tanggung jawab spimgr::init() (dipanggil sekali dari
// main.cpp SEBELUM mpu9250::init()).
//
// TAMBAHAN: kalibrasi accelerometer (offset + skala per sumbu) untuk
// menghasilkan nilai terkoreksi yang akurat. Offset/skala disimpan di NVS.
// ============================================================================

namespace mpu9250 {

// Inisialisasi register MPU9250 (WHO_AM_I check + power/gyro/accel/DLPF
// config). Return false jika WHO_AM_I tidak sesuai (sensor tidak
// terdeteksi / wiring salah). Mengasumsikan spimgr::init() SUDAH dipanggil
// sebelumnya (bus SPI & mutex sudah siap).
bool init();

// Kalibrasi bias gyro (PERTAHANKAN: warm-up 5 detik + averaging N sample).
// Ini rutin blocking sekali-jalan yang dipanggil saat boot / command
// recalibrate — TIDAK dipanggil dari dalam loop sampling 200Hz.
void calibrateGyro(uint16_t samples);

// Kalibrasi accelerometer (coarse): kalibrasi offset dengan asumsi node
// terpasang DATAR (Z vertikal) saat diam. Menghapus proyeksi gravitasi
// statis (leveling) supaya accel terkalibrasi membaca ~0 pada sumbu X/Y
// dan ~1g pada sumbu Z saat diam. Simpan offset/skala ke NVS.
// CATATAN: ini BUKAN kalibrasi 6-posisi penuh (FR-05). Untuk skala per
// sumbu dan pemisahan bias-tilt yang benar, perlu prosedur 6-posisi.
// Dipanggil dari command handler (bukan loop sampling).
bool calibrateAccel();

// Muat offset/skala accel dari NVS. Dipanggil otomatis di init().
void loadAccelCalibration();

// Apakah kalibrasi accel sudah tersedia/dimuat.
bool isAccelCalibrated();

// Set full-scale akselerometer: gain 1/2/4/8 -> +/-2/+/-4/+/-8/+/-16 g.
// Menulis ulang REG_ACCEL_CONFIG (AFS_SEL) + update accel_scale internal.
void setSensitivityGain(uint8_t gain);

// Baca satu sample accel+gyro (scaled, gyro sudah dikurangi bias +
// deadband + adaptive-bias update). PERTAHANKAN logic dari referensi.
// Setiap register read di dalamnya sudah dilindungi spiMutex secara
// individual (lihat mpu9250.cpp), jadi fungsi ini selalu berhasil
// (blocking sampai bus tersedia — lihat catatan priority-inheritance di
// utils/spi_manager.cpp).
void readAccelGyro(float& ax, float& ay, float& az,
                    float& gx, float& gy, float& gz);

// Baca accel mentah (raw ADC counts) tanpa koreksi offset/skala.
// Dipakai untuk sesi kalibrasi dan dataset raw counts.
void readAccelRaw(int16_t& ax, int16_t& ay, int16_t& az);

// Baca magnetometer mentah (uT). Stub siap-pakai untuk fusion yaw di masa
// depan — lihat penjelasan lebih lengkap di mpu9250.cpp.
bool readMagnetometer(float& mx, float& my, float& mz);

// Ambil offset/skala kalibrasi accel saat ini (untuk payload).
void getAccelCalibration(float& off_x, float& off_y, float& off_z,
                         float& scale_x, float& scale_y, float& scale_z);

// Terapkan koreksi kalibrasi accel ke nilai raw (dipakai internal).
void applyAccelCalibration(float raw_x, float raw_y, float raw_z,
                           float& cal_x, float& cal_y, float& cal_z);

} // namespace mpu9250

#endif // SHM_MPU9250_H
