#ifndef SHM_SD_LOGGER_H
#define SHM_SD_LOGGER_H

#include <Arduino.h>
#include "../utils/data_structures.h"

// ============================================================================
// sd_logger.h — Logging persistent ke SD Card, append-only, binary format
//
// Format record binary (fixed-size, mudah di-seek/parse ulang saat sync):
//   uint32_t sequence
//   uint32_t timestamp
//   float    pitch
//   float    roll
//   float    pitch_delta
//   float    roll_delta
//   float    rms_vibration
//   uint16_t sampling_rate_hz
//   -----------------------------
//   total: 26 bytes/record (packed)
//
// Logging TIDAK PERNAH berhenti walau MQTT disconnect — modul ini tidak
// punya dependensi apapun ke WiFi/MQTT.
// ============================================================================

#pragma pack(push, 1)
struct SDRecord {
    uint32_t sequence;
    uint32_t timestamp;
    float pitch;
    float roll;
    float pitch_delta;
    float roll_delta;
    float rms_vibration;
    uint16_t sampling_rate_hz;
};
#pragma pack(pop)

namespace sdlog {

// Inisialisasi SD Card memakai bus SPI GLOBAL yang sama dengan MPU9250.
bool init();

// Re-init berkala saat SD down (dipanggil task_sd_writer) supaya logging
// pulih sendiri tanpa restart node. No-op bila sd_ready masih true.
void checkRecovery();

// Tulis satu record ke buffer internal jika sesi aktif; buffer di-flush otomatis
// setiap SD_WRITE_BATCH_SIZE record ATAU setiap SD_FLUSH_INTERVAL_MS.
void writeRecord(const SDRecord& rec);

// Paksa flush buffer ke file.
void flush();

// Manajemen Sesi SD: Mulai menulis ke file baru (/shm_logs/<node>/session_<id>.bin)
bool startSession(uint32_t sessionId);

// Selesai sesi: flush buffer dan tutup file SD (SD kembali idle total)
void stopSession();

// Cek apakah sesi rekaman sedang aktif
bool isSessionActive();

bool isReady();

} // namespace sdlog

#endif // SHM_SD_LOGGER_H
