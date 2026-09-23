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

// Inisialisasi SD Card memakai bus SPI GLOBAL yang sama dengan MPU9250
// (satu bus VSPI, lihat utils/spi_manager.h) + mount SD Card + buat
// direktori log jika
// belum ada. Return false jika kartu tidak terdeteksi (task SD Writer akan
// tetap berjalan dalam mode "degraded": data di-drop dari sisi SD saja,
// MQTT tetap jalan seperti biasa).
bool init();

// Re-init berkala saat SD down (dipanggil task_sd_writer) supaya logging
// pulih sendiri tanpa restart node. No-op bila sd_ready masih true.
void checkRecovery();

// Tulis satu record ke buffer internal; buffer di-flush ke file otomatis
// setiap SD_WRITE_BATCH_SIZE record ATAU setiap SD_FLUSH_INTERVAL_MS,
// mana yang lebih dulu tercapai (dipanggil dari task_sd_writer).
void writeRecord(const SDRecord& rec);

// Paksa flush buffer ke file (dipanggil task_sd_writer di akhir setiap
// siklusnya, dan otomatis oleh writeRecord() saat batas tercapai).
void flush();

// Cek & lakukan rotasi file jika sudah melewati SD_FILE_ROTATE_INTERVAL_MS
// atau ukuran file sudah melebihi SD_FILE_ROTATE_MAX_BYTES. Nama file:
// /SHM_<node>_<YYYY-MM-DD>_<HH>.bin (lihat rtc_ds3231 untuk sumber waktu).
void checkRotation();

// Baca record dari file (dipakai oleh SyncManager) berdasarkan sequence
// number > `afterSequence`, dipanggil berulang sampai return false (habis).
// `outRec` diisi record berikutnya yang sequence-nya lebih besar.
bool readNextUnsent(uint32_t afterSequence, SDRecord& outRec);

// Baca/tulis file checkpoint sync (sequence terakhir terkirim). WAJIB lewat
// sini (bukan akses SD langsung) supaya terlindungi spiMutex + MPU_CS HIGH —
// sama seperti operasi SD lain di modul ini.
bool readCheckpoint(uint32_t& outSeq);
bool writeCheckpoint(uint32_t seq);

bool isReady();

} // namespace sdlog

#endif // SHM_SD_LOGGER_H
