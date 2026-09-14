#include "rtc_ds3231.h"
#include <Wire.h>
#include <time.h>
#include "../config/config.h"

namespace rtc {

SemaphoreHandle_t i2cMutex = nullptr;

// ---------------------------------------------------------------------------
// State internal (PERTAHANKAN pendekatan: sync 1x/detik dari chip, resolusi
// ms didapat dari interpolasi micros() di antara sync — lihat referensi).
// ---------------------------------------------------------------------------
static uint8_t rtc_hh = 0, rtc_mm = 0, rtc_ss = 0;
static uint16_t rtc_year = 2026;
static uint8_t rtc_month = 1, rtc_day = 1;
static unsigned long rtc_sync_micros = 0;

static uint8_t bcd2dec(uint8_t bcd) { return ((bcd >> 4) * 10) + (bcd & 0x0F); }
static uint8_t dec2bcd(uint8_t dec) { return ((dec / 10) << 4) | (dec % 10); }

static void writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

static uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);  // repeated start, PERTAHANKAN (jangan lepas bus)
    Wire.requestFrom((uint8_t)RTC_I2C_ADDR, (uint8_t)1);
    if (Wire.available()) return Wire.read();
    return 0;
}

bool init() {
    if (i2cMutex == nullptr) {
        i2cMutex = xSemaphoreCreateMutex();
    }

    Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    uint8_t status = readReg(0x0F);
    bool oscillator_ok = !(status & 0x80);   // OSF flag
    xSemaphoreGive(i2cMutex);

    sync();
    return oscillator_ok;
}

void setTime(uint8_t hh, uint8_t mm, uint8_t ss) {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    writeReg(0x00, dec2bcd(ss));  // bit7=0 -> oscillator jalan (CH cleared), PERTAHANKAN
    writeReg(0x01, dec2bcd(mm));
    writeReg(0x02, dec2bcd(hh));  // mode 24 jam (bit6=0)
    xSemaphoreGive(i2cMutex);
    sync();
}

void setDate(uint16_t year, uint8_t month, uint8_t day) {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    writeReg(0x04, dec2bcd(day));
    writeReg(0x05, dec2bcd(month));
    writeReg(0x06, dec2bcd((uint8_t)(year - 2000)));
    xSemaphoreGive(i2cMutex);

    rtc_year = year; rtc_month = month; rtc_day = day;
}

void sync() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    rtc_ss = bcd2dec(readReg(0x00) & 0x7F);
    rtc_mm = bcd2dec(readReg(0x01) & 0x7F);
    rtc_hh = bcd2dec(readReg(0x02) & 0x3F);
    rtc_day   = bcd2dec(readReg(0x04) & 0x3F);
    rtc_month = bcd2dec(readReg(0x05) & 0x1F);
    rtc_year  = 2000 + bcd2dec(readReg(0x06));
    xSemaphoreGive(i2cMutex);

    rtc_sync_micros = micros();
}

// Hitung HH:MM:SS "saat ini" dengan interpolasi micros() sejak sync
// terakhir (PERTAHANKAN pendekatan referensi), dikembalikan lewat
// parameter, dipakai baik oleh getEpochSeconds() maupun getTimestampString().
static void interpolatedTime(uint8_t& hh, uint8_t& mm, uint8_t& ss, uint16_t& ms) {
    unsigned long elapsed_us = micros() - rtc_sync_micros; // aman thd overflow
    unsigned long elapsed_ms = elapsed_us / 1000UL;

    ms = elapsed_ms % 1000UL;
    unsigned long extra_sec_total = elapsed_ms / 1000UL;

    uint8_t s_sum = rtc_ss + (extra_sec_total % 60);
    ss = s_sum % 60;
    uint8_t carry_min = s_sum / 60 + (extra_sec_total / 60) % 60;
    uint8_t m_sum = rtc_mm + carry_min;
    mm = m_sum % 60;
    uint8_t carry_hour = m_sum / 60;
    hh = (rtc_hh + carry_hour) % 24;
}

uint32_t getEpochSeconds() {
    uint8_t hh, mm, ss; uint16_t ms;
    interpolatedTime(hh, mm, ss, ms);

    struct tm t = {};
    t.tm_year = rtc_year - 1900;
    t.tm_mon  = rtc_month - 1;
    t.tm_mday = rtc_day;
    t.tm_hour = hh;
    t.tm_min  = mm;
    t.tm_sec  = ss;
    time_t epoch = mktime(&t);   // newlib mktime, tersedia di framework ESP32
    return (uint32_t)epoch;
}

void getTimestampString(char* buf, size_t buflen) {
    uint8_t hh, mm, ss; uint16_t ms;
    interpolatedTime(hh, mm, ss, ms);
    snprintf(buf, buflen, "%02u:%02u:%02u.%03u", hh, mm, ss, ms);
}

void getDateTimeString(char* buf, size_t buflen) {
    uint8_t hh, mm, ss; uint16_t ms;
    interpolatedTime(hh, mm, ss, ms);
    snprintf(buf, buflen, "%04u-%02u-%02uT%02u:%02u:%02u.%03u",
             rtc_year, rtc_month, rtc_day, hh, mm, ss, ms);
}

void getDateString(char* buf, size_t buflen) {
    snprintf(buf, buflen, "%04u-%02u-%02u", rtc_year, rtc_month, rtc_day);
}

uint8_t getHour() {
    uint8_t hh, mm, ss; uint16_t ms;
    interpolatedTime(hh, mm, ss, ms);
    return hh;
}

} // namespace rtc
