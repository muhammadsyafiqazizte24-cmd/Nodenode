#ifndef SHM_RTC_DS3231_H
#define SHM_RTC_DS3231_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ============================================================================
// rtc_ds3231.h — Driver RTC DS3231 register-level via I2C (Wire)
//
// PERTAHANKAN: logic identik dengan program referensi (bcd2dec/dec2bcd,
// rtcSync 1x/detik, interpolasi micros() untuk resolusi ms, oscillator-stop
// flag check). Perubahan struktural saja: dibungkus mutex (i2cMutex) &
// timestamp dikonversi ke epoch uint32_t supaya mudah dipakai sebagai field
// MPU9250Data.timestamp / dikirim lewat MQTT & disimpan di SD.
//
// TAMBAHAN: getEpochMillis() untuk presisi ms (dibutuhkan sinkronisasi FDD).
// ============================================================================

namespace rtc {

extern SemaphoreHandle_t i2cMutex;

// Inisialisasi I2C bus + cek oscillator flag. Return false jika RTC pernah
// kehilangan daya (waktu tidak valid, perlu di-set ulang lewat setTime()).
bool init();

// Set waktu RTC secara manual (dipanggil dari config handler/serial).
void setTime(uint8_t hh, uint8_t mm, uint8_t ss);

// Set tanggal RTC (dibutuhkan untuk penamaan file SD /SHM_001_YYYY-MM-DD_HH.bin)
void setDate(uint16_t year, uint8_t month, uint8_t day);

// Sinkronisasi waktu dari NTP (UTC) ke chip DS3231. Return true bila berhasil.
// Dipanggil sekali setelah WiFi connect; DS3231 lalu menjaga waktu via baterai.
bool syncNtp();

// Sinkronisasi ulang dari chip (dipanggil 1x/detik dari task processing/
// sampling, TIDAK setiap sample, supaya I2C tidak mengganggu timing SPI).
void sync();

// Timestamp sebagai epoch-like seconds-of-day + interpolasi ms internal.
// Dipakai untuk field MPU9250Data.timestamp (detik, resolusi cukup untuk
// keperluan SHM; resolusi sub-detik ada di dt masing-masing sample).
uint32_t getEpochSeconds();

// Timestamp epoch dalam milidetik (untuk sinkronisasi window FDD).
// Kombinasi getEpochSeconds() * 1000 + ms dari interpolasi.
uint64_t getEpochMillis();

// String timestamp "HH:MM:SS.mmm" untuk logging/debug (PERTAHANKAN format).
void getTimestampString(char* buf, size_t buflen);

// String datetime ISO "YYYY-MM-DDTHH:MM:SS.mmmZ" (UTC) untuk payload MQTT.
// Suffix "Z" menandai UTC supaya backend/frontend parse & bandingkan konsisten.
void getDateTimeString(char* buf, size_t buflen);

// String tanggal "YYYY-MM-DD" untuk penamaan file SD.
void getDateString(char* buf, size_t buflen);

// Jam saja (0-23), dipakai untuk deteksi rotasi file per jam.
uint8_t getHour();

} // namespace rtc

#endif // SHM_RTC_DS3231_H
