#include "mpu9250.h"
#include "mpu9250_registers.h"
#include "../config/config.h"
#include "../utils/spi_manager.h"

namespace mpu9250 {

// ---------------------------------------------------------------------------
// State internal modul (PERTAHANKAN nilai & satuan dari referensi:
// gyro_bias_* dalam dps, accel_scale/gyro_scale hasil pembagian range ADC).
// ---------------------------------------------------------------------------
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
static float accel_scale = 16.0f / 32768.0f;   // berubah saat setSensitivityGain
static const float gyro_scale  = 2000.0f / 32768.0f;
static bool calibrating = false;

// ==================== SPI LOW-LEVEL ====================
// Bus SPI dipakai BERSAMA dengan SD Card (satu bus VSPI global, lihat
// utils/spi_manager.h). SETIAP transfer di bawah ini WAJIB:
//   1. Ambil spimgr::spiMutex (portMAX_DELAY -- lihat catatan
//      priority-inheritance di spi_manager.cpp untuk alasan ini aman
//      dipakai blocking tanpa timeout).
//   2. SPI.beginTransaction(...)
//   3. Pastikan SD_CS HIGH (device lain di bus WAJIB deselected).
//   4. MPU_CS LOW -> transfer -> MPU_CS HIGH.
//   5. SPI.endTransaction()
//   6. Lepas spiMutex.
// Pola ini identik untuk readRegister/writeRegister/read16 -- masing-masing
// mengunci & melepas mutex secara independen per transfer (bukan per
// pemanggil level atas), sesuai desain SPI Manager terpusat.
static uint8_t readRegister(uint8_t reg) {
    xSemaphoreTake(spimgr::spiMutex, portMAX_DELAY);

    SPI.beginTransaction(SPISettings(MPU_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(SD_CS_PIN, HIGH);
    digitalWrite(MPU_CS_PIN, LOW);

    SPI.transfer(reg | 0x80);
    uint8_t value = SPI.transfer(0x00);

    digitalWrite(MPU_CS_PIN, HIGH);
    SPI.endTransaction();

    xSemaphoreGive(spimgr::spiMutex);
    return value;
}

static void writeRegister(uint8_t reg, uint8_t data) {
    xSemaphoreTake(spimgr::spiMutex, portMAX_DELAY);

    SPI.beginTransaction(SPISettings(MPU_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(SD_CS_PIN, HIGH);
    digitalWrite(MPU_CS_PIN, LOW);

    SPI.transfer(reg & 0x7F);
    SPI.transfer(data);

    digitalWrite(MPU_CS_PIN, HIGH);
    SPI.endTransaction();

    xSemaphoreGive(spimgr::spiMutex);
}

static int16_t read16(uint8_t regHigh) {
    xSemaphoreTake(spimgr::spiMutex, portMAX_DELAY);

    SPI.beginTransaction(SPISettings(MPU_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(SD_CS_PIN, HIGH);
    digitalWrite(MPU_CS_PIN, LOW);

    SPI.transfer(regHigh | 0x80);
    uint8_t high = SPI.transfer(0x00);
    uint8_t low  = SPI.transfer(0x00);

    digitalWrite(MPU_CS_PIN, HIGH);
    SPI.endTransaction();

    xSemaphoreGive(spimgr::spiMutex);
    return (int16_t)((high << 8) | low);
}

// ==================== INIT (PERTAHANKAN urutan register) ====================
// Mengasumsikan spimgr::init() SUDAH dipanggil (bus SPI & CS pin sudah
// siap) -- modul ini tidak lagi memanggil SPI.begin() atau mengatur pin CS.
bool init() {
    uint8_t whoami = readRegister(REG_WHO_AM_I);
    if (whoami != 0x71) {
        return false;   // sensor tidak terdeteksi / wiring salah
    }

    writeRegister(REG_PWR_MGMT_1, 0x80); delay(100);   // reset
    writeRegister(REG_PWR_MGMT_1, 0x01); delay(10);    // clock source PLL
    writeRegister(REG_GYRO_CONFIG, 0x18); delay(10);   // ±2000 dps
    writeRegister(REG_ACCEL_CONFIG, 0x18); delay(10);  // ±16g
    writeRegister(REG_ACCEL_CONFIG2, 0x03); delay(10); // DLPF accel
    writeRegister(REG_CONFIG, 0x03); delay(10);        // DLPF gyro

    return true;
}

// ==================== GYRO CALIBRATION (PERTAHANKAN) ====================
void calibrateGyro(uint16_t samples) {
    calibrating = true;

    // Warm-up: baca sensor selama beberapa detik sebelum sampling bias,
    // supaya MEMS gyro sudah stabil secara termal (lihat rationale di
    // program referensi asli). read16() sudah mengunci mutex sendiri
    // per panggilan, jadi tidak perlu locking tambahan di sini.
    unsigned long warmup_start = millis();
    while (millis() - warmup_start < WARMUP_DURATION_MS) {
        read16(REG_GYRO_XOUT_H);
        delay(10);
    }

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    for (uint16_t i = 0; i < samples; i++) {
        sum_x += read16(REG_GYRO_XOUT_H)     * gyro_scale;
        sum_y += read16(REG_GYRO_XOUT_H + 2) * gyro_scale;
        sum_z += read16(REG_GYRO_XOUT_H + 4) * gyro_scale;
        delay(2);
    }

    gyro_bias_x = sum_x / samples;
    gyro_bias_y = sum_y / samples;
    gyro_bias_z = sum_z / samples;

    calibrating = false;
}

// ==================== SENSITIVITY GAIN (full-scale akselerometer) ====================
// gain 1/2/4/8 -> AFS_SEL 0/1/2/3 -> +/-2/+/-4/+/-8/+/-16 g. accel_scale diubah
// supaya konversi ADC->g tetap benar (PERTAHANKAN pembagian 32768). Dipanggil
// dari Config Handler task saat menerima command sensitivity_gain.
void setSensitivityGain(uint8_t gain) {
    if (gain != 1 && gain != 2 && gain != 4 && gain != 8) return;
    uint8_t afs = (gain == 1) ? 0 : (gain == 2) ? 1 : (gain == 4) ? 2 : 3;
    writeRegister(REG_ACCEL_CONFIG, (afs << 3) & 0x18);
    accel_scale = (2.0f * gain) / 32768.0f;
}

// ==================== READ ACCEL + GYRO (PERTAHANKAN logic) ====================
void readAccelGyro(float& ax, float& ay, float& az,
                    float& gx, float& gy, float& gz) {
    // Setiap read16() di bawah mengunci & melepas spiMutex secara mandiri
    // (lihat komentar di atas read16()) -- total 6 transaksi mutex per
    // sample, blocking tanpa timeout. Ini aman karena spiMutex adalah
    // real FreeRTOS mutex dengan priority inheritance (lihat
    // spi_manager.cpp), sehingga task SD Writer yang sedang memegang
    // mutex otomatis di-boost prioritasnya sementara jika task Sensor
    // Sampling (prioritas tertinggi) sedang menunggu.
    ax = read16(REG_ACCEL_XOUT_H)     * accel_scale;
    ay = read16(REG_ACCEL_XOUT_H + 2) * accel_scale;
    az = read16(REG_ACCEL_XOUT_H + 4) * accel_scale;

    float raw_gx = read16(REG_GYRO_XOUT_H)     * gyro_scale;
    float raw_gy = read16(REG_GYRO_XOUT_H + 2) * gyro_scale;
    float raw_gz = read16(REG_GYRO_XOUT_H + 4) * gyro_scale;

    // ---- Adaptive gyro bias (PERTAHANKAN) ----
    // Update bias sangat perlahan saat sensor terdeteksi diam, supaya bias
    // instability jangka panjang tetap terkompensasi tanpa mengganggu saat
    // sensor bergerak.
    float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
    bool is_stationary = (fabsf(accel_mag - 1.0f) < ACCEL_STATIONARY_TOL) &&
                          (fabsf(raw_gx) < GYRO_STATIONARY_DPS) &&
                          (fabsf(raw_gy) < GYRO_STATIONARY_DPS) &&
                          (fabsf(raw_gz) < GYRO_STATIONARY_DPS);

    if (is_stationary && !calibrating) {
        gyro_bias_x += ADAPTIVE_BIAS_ALPHA * (raw_gx - gyro_bias_x);
        gyro_bias_y += ADAPTIVE_BIAS_ALPHA * (raw_gy - gyro_bias_y);
        gyro_bias_z += ADAPTIVE_BIAS_ALPHA * (raw_gz - gyro_bias_z);
    }

    gx = raw_gx - gyro_bias_x;
    gy = raw_gy - gyro_bias_y;
    gz = raw_gz - gyro_bias_z;

    // ---- Deadband (PERTAHANKAN) ----
    if (fabsf(gx) < GYRO_DEADBAND_DPS) gx = 0.0f;
    if (fabsf(gy) < GYRO_DEADBAND_DPS) gy = 0.0f;
    if (fabsf(gz) < GYRO_DEADBAND_DPS) gz = 0.0f;
}

// ==================== MAGNETOMETER (stub) ====================
bool readMagnetometer(float& mx, float& my, float& mz) {
    // Belum diimplementasikan (AK8963 internal butuh inisialisasi
    // I2C-master pass-through MPU9250 terpisah, di luar scope revisi ini).
    // Dikembalikan 0 supaya struct MPU9250Data tetap konsisten & pipeline
    // tidak perlu percabangan khusus.
    mx = my = mz = 0.0f;
    return false;
}

} // namespace mpu9250
