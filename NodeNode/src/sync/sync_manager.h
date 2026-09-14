#ifndef SHM_SYNC_MANAGER_H
#define SHM_SYNC_MANAGER_H

#include <Arduino.h>

// ============================================================================
// sync_manager.h — Sinkronisasi data SD Card <-> MQTT setelah reconnect
//
// Prinsip: sequence number adalah sumber kebenaran tunggal untuk menentukan
// data mana yang SUDAH terkirim. Checkpoint (last_sent_sequence) disimpan
// persistent di SD (SD_CHECKPOINT_FILE) supaya bertahan lintas reboot.
// ============================================================================

namespace syncmgr {

// Load checkpoint dari SD saat boot. Return 0 jika file checkpoint belum
// ada (node baru pertama kali nyala / SD baru diformat).
uint32_t loadCheckpoint();

// Simpan checkpoint ke SD. Dipanggil setiap kali markAsSent() maju supaya
// crash/reboot mendadak tidak mengulang kirim data yang sama secara masif
// (di-throttle secara internal supaya tidak menulis SD setiap 1 record).
void saveCheckpoint();

// Tandai sequence number tertentu sebagai berhasil terkirim.
void markAsSent(uint32_t sequence);

uint32_t getLastSentSequence();

// Dipanggil dari task Sync Manager setiap kali MQTT baru saja reconnect
// (transisi offline->online). Membaca SD mulai dari
// getLastSentSequence()+1, mem-publish ulang lewat MQTT satu per satu
// (dengan jeda kecil supaya tidak membanjiri broker/CPU), berhenti begitu
// data historis habis atau MQTT terputus lagi.
void syncAfterReconnect();

} // namespace syncmgr

#endif // SHM_SYNC_MANAGER_H
