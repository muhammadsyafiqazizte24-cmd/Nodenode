#include "task_manager.h"
#include "../config/config.h"
#include "../config/freertos_config.h"
#include "../config/mqtt_config.h"
#include "../sensors/mpu9250.h"
#include "../sensors/rtc_ds3231.h"
#include "../processing/kalman_filter.h"
#include "../processing/baseline.h"
#include "../processing/rms_calculator.h"
#include "../processing/fft_buffer.h"
#include "../communication/wifi_manager.h"
#include "../communication/mqtt_manager.h"
#include "../storage/sd_logger.h"
#include "esp_task_wdt.h"
#include <WiFi.h>

// ============================================================================
// task_manager.cpp — Definisi & implementasi seluruh task FreeRTOS
// ============================================================================

// ---------------------------------------------------------------------------
// Definisi handle global (extern di task_manager.h)
// ---------------------------------------------------------------------------
QueueHandle_t rawDataQueue = nullptr;
QueueHandle_t sdWriteQueue = nullptr;
QueueHandle_t mqttPublishQueue = nullptr;
QueueHandle_t configCommandQueue = nullptr;

SemaphoreHandle_t sdFileMutex = nullptr;
SemaphoreHandle_t fftBufferMutex = nullptr;

volatile uint16_t g_samplingRateHz = SAMPLE_RATE_DEFAULT_HZ;
volatile uint8_t  g_sensitivityGain = SENSITIVITY_GAIN_DEFAULT;
volatile uint32_t g_publishIntervalMs = MQTT_PUBLISH_INTERVAL_DEFAULT_MS;
volatile uint32_t g_fftSendIntervalMs = FFT_SEND_INTERVAL_DEFAULT_MS;
volatile bool     g_recalibrateRequested = false;
volatile bool     g_restartRequested = false;
volatile bool     g_statusRequested = false;
volatile bool     g_calibrateAccelRequested = false;

volatile uint32_t g_sequenceCounter = 0;
volatile uint32_t g_droppedSamples = 0;

// Batch publish state (FR-01)
// boot_id dari NVS (naik tiap restart), packet_seq increment tiap publish batch
static const char* NVS_NAMESPACE_BATCH = "batch";
static const char* NVS_KEY_BOOT_ID = "boot_id";
static const char* NVS_KEY_PACKET_SEQ = "packet_seq";

uint32_t g_bootId = 1;
uint32_t g_packetSeq = 0;

// ---------------------------------------------------------------------------
// Objek pemrosesan (dimiliki sepenuhnya oleh task_data_processing, TIDAK
// diakses task lain -> tidak butuh mutex, sesuai desain single-owner).
// ---------------------------------------------------------------------------
static KalmanFilter kalmanPitch, kalmanRoll;
static BaselineManager baseline;
static RMSCalculator rmsCalc;
static FFTBuffer fftBuffer;   // ditulis di sini, dibaca task fft_sender (via fftBufferMutex)

// ============================================================================
// TASK: Sensor Sampling (Core 0, priority 10)
// ============================================================================
static void task_sensor_sampling(void* pv) {
    esp_task_wdt_add(NULL);

    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        // Interval dihitung ulang tiap iterasi supaya perubahan
        // g_samplingRateHz (remote config) langsung berlaku tanpa restart.
        uint16_t hz = g_samplingRateHz;
        if (hz < SAMPLE_RATE_MIN_HZ) hz = SAMPLE_RATE_MIN_HZ;
        if (hz > SAMPLE_RATE_MAX_HZ) hz = SAMPLE_RATE_MAX_HZ;
        TickType_t periodTicks = pdMS_TO_TICKS(1000 / hz);
        if (periodTicks < 1) periodTicks = 1;

        static unsigned long last_us = 0;
        unsigned long now_us = micros();
        float dt = (last_us == 0) ? (1.0f / hz) : ((now_us - last_us) / 1000000.0f);
        if (dt > DT_MAX) dt = DT_MAX;
        last_us = now_us;

        // Re-sync RTC dari chip DS3231 1x/detik. Tanpa ini, interpolasi waktu
        // (micros()) hanya ter-anchor sekali saat boot, lalu drift dari jam
        // DS3231 asli -> timestamp payload tidak sinkron dengan RTC. I2C
        // ~200us sekali per detik tidak mengganggu timing sampling 5ms.
        static unsigned long lastRtcSyncMs = 0;
        unsigned long msNow = millis();
        if (msNow - lastRtcSyncMs >= RTC_SYNC_INTERVAL_MS) {
            if (rtc::isAvailable()) {
                rtc::sync();
            }
            lastRtcSyncMs = msNow;
        }

        MPU9250Data sample;
        mpu9250::readAccelGyro(sample.accel_x, sample.accel_y, sample.accel_z,
                                sample.gyro_x, sample.gyro_y, sample.gyro_z);
        mpu9250::readMagnetometer(sample.mag_x, sample.mag_y, sample.mag_z);
        sample.timestamp = rtc::getEpochSeconds();
        sample.dt = dt;
        sample.sequence = g_sequenceCounter++;
        sample.epoch_ms = rtc::getEpochMillis();

        if (xQueueSend(rawDataQueue, &sample, 0) != pdTRUE) {
            g_droppedSamples++;   // queue penuh -> processing task tertinggal
        }

        esp_task_wdt_reset();
        // vTaskDelayUntil menjaga jitter minimal walau readAccelGyro/queue
        // send memakan waktu variabel (worst-case dianalisis di
        // freertos_config.h: total << periode sampling).
        vTaskDelayUntil(&lastWake, periodTicks);
    }
}

// ============================================================================
// TASK: Data Processing (Core 0, priority 9) — Kalman, baseline, RMS, FFT feed
// ============================================================================
static void task_data_processing(void* pv) {
    esp_task_wdt_add(NULL);
    MPU9250Data sample;

    for (;;) {
        if (xQueueReceive(rawDataQueue, &sample, pdMS_TO_TICKS(1000)) == pdTRUE) {
            esp_task_wdt_reset();

            // ---- Handle recalibrate request (dari Config Handler) ----
            if (g_recalibrateRequested) {
                baseline.reset();
                g_recalibrateRequested = false;
            }

            // ---- Handle accel calibration request ----
            if (g_calibrateAccelRequested) {
                mpu9250::calibrateAccel();
                g_calibrateAccelRequested = false;
            }

            // ---- Orientasi mentah dari accelerometer (simetris, stabil) ----
            float accel_pitch = atan2f(-sample.accel_x,
                                        sqrtf(sample.accel_y * sample.accel_y +
                                              sample.accel_z * sample.accel_z)) * 57.29578f;
            float accel_roll = atan2f(sample.accel_y,
                                       sqrtf(sample.accel_x * sample.accel_x +
                                             sample.accel_z * sample.accel_z)) * 57.29578f;

            // ---- HANYA Kalman Filter (sesuai spesifikasi) ----
            // Konvensi sumbu PERTAHANKAN dari referensi: roll <- gyro X,
            // pitch <- gyro Y.
            float roll  = kalman::update(&kalmanRoll,  accel_roll,  sample.gyro_x, sample.dt);
            float pitch = kalman::update(&kalmanPitch, accel_pitch, sample.gyro_y, sample.dt);

            // ---- Baseline (kalibrasi otomatis di 100 sample pertama) ----
            if (!baseline.isCalibrated()) {
                baseline.feedSample(pitch, roll);
            }
            float pitch_delta = baseline.getDeltaPitch(pitch);
            float roll_delta  = baseline.getDeltaRoll(roll);

            // ---- RMS getaran (sliding window 1 detik) ----
            float rms = rmsCalc.addSample(sample.accel_x, sample.accel_y, sample.accel_z);

            // ---- Raw FFT buffer feed (axis Z untuk getaran vertikal) ----
            if (xSemaphoreTake(fftBufferMutex, 0) == pdTRUE) {
                fftBuffer.addSample(sample.accel_z - 1.0f, sample.epoch_ms);   // gravitasi dihilangkan
                xSemaphoreGive(fftBufferMutex);
            }

            // ---- Ambil kalibrasi accel untuk payload ----
            float accel_off[3] = {0, 0, 0}, accel_scl[3] = {1, 1, 1};
            bool has_cal = mpu9250::isAccelCalibrated();
            if (has_cal) {
                mpu9250::getAccelCalibration(accel_off[0], accel_off[1], accel_off[2],
                                              accel_scl[0], accel_scl[1], accel_scl[2]);
            }

            float ax_cal = 0, ay_cal = 0, az_cal = 0;
            if (has_cal) {
                mpu9250::applyAccelCalibration(sample.accel_x, sample.accel_y, sample.accel_z,
                                               ax_cal, ay_cal, az_cal);
            }

            ProcessedData pd;
            pd.sequence = sample.sequence;
            pd.timestamp = sample.timestamp;
            pd.pitch = pitch;
            pd.roll = roll;
            pd.pitch_delta = pitch_delta;
            pd.roll_delta = roll_delta;
            pd.rms_vibration = rms;
            pd.sampling_rate_hz = g_samplingRateHz;
            // Snapshot raw accel/gyro untuk dashboard "Raw Sensor Data"
            pd.accel_x = sample.accel_x;
            pd.accel_y = sample.accel_y;
            pd.accel_z = sample.accel_z;
            pd.gyro_x = sample.gyro_x;
            pd.gyro_y = sample.gyro_y;
            pd.gyro_z = sample.gyro_z;
            // Calibrated accel
            pd.accel_calibrated = has_cal;
            pd.accel_x_cal = ax_cal;
            pd.accel_y_cal = ay_cal;
            pd.accel_z_cal = az_cal;
            // Calibration info
            pd.calibration_version = has_cal ? 1 : 0;
            pd.accel_offset[0] = accel_off[0];
            pd.accel_offset[1] = accel_off[1];
            pd.accel_offset[2] = accel_off[2];
            pd.accel_scale[0] = accel_scl[0];
            pd.accel_scale[1] = accel_scl[1];
            pd.accel_scale[2] = accel_scl[2];

            // Fan-out: hanya dorong ke SD Writer jika sesi sedang aktif
            if (sdlog::isSessionActive()) {
                xQueueSend(sdWriteQueue, &pd, 0);
            }
            xQueueSend(mqttPublishQueue, &pd, 0);
        }
    }
}

// ============================================================================
// TASK: SD Card Writer (Core 1, priority 8) — Hanya menulis saat sesi aktif
// ============================================================================
static void task_sd_writer(void* pv) {
    esp_task_wdt_add(NULL);
    ProcessedData pd;

    for (;;) {
        if (xQueueReceive(sdWriteQueue, &pd, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (sdlog::isSessionActive()) {
                SDRecord rec;
                rec.sequence = pd.sequence;
                rec.timestamp = pd.timestamp;
                rec.pitch = pd.pitch;
                rec.roll = pd.roll;
                rec.pitch_delta = pd.pitch_delta;
                rec.roll_delta = pd.roll_delta;
                rec.rms_vibration = pd.rms_vibration;
                rec.sampling_rate_hz = pd.sampling_rate_hz;
                sdlog::writeRecord(rec);
            }
        }
        sdlog::checkRecovery();
        esp_task_wdt_reset();
    }
}

// ============================================================================
// TASK: MQTT Publisher (Core 1, priority 7)
// ============================================================================
// Pengiriman gabungan ke bridge/<node_id>/data per 1 detik:
// Mengumpulkan 200 ProcessedData (1 detik @ 200Hz). Begitu terkumpul (atau
// timeout 1s), dikirim sebagai payload tunggal yang berisi summary live
// dashboard (RMS, tilt Kalman, snapshot XYZ) DAN 200 raw samples ax/ay/az
// untuk arsip time-series resolusi penuh.
// ============================================================================
static void task_mqtt_publisher(void* pv) {
    // Batch buffer: 200 sampel (1 detik @ 200Hz)
    static ProcessedData batch[200];
    static uint16_t batchCount = 0;
    static TickType_t batchStartTick = 0;
    static bool batchActive = false;

    ProcessedData pd;
    TickType_t lastPublish = xTaskGetTickCount();

    for (;;) {
        // Drain mqttPublishQueue
        while (xQueueReceive(mqttPublishQueue, &pd, 0) == pdTRUE) {
            if (batchCount < 200) {
                batch[batchCount++] = pd;
                if (!batchActive) {
                    batchActive = true;
                    batchStartTick = xTaskGetTickCount();
                }
            }
        }

        // Kirim payload gabungan setiap 1 detik (atau buffer penuh 200 sampel)
        TickType_t now = xTaskGetTickCount();
        uint32_t elapsedMs = pdMS_TO_TICKS(1000);  // 1 detik target
        if ((batchActive && (batchCount >= 200 || (now - batchStartTick) >= elapsedMs)) &&
            (now - lastPublish) >= pdMS_TO_TICKS(50)) {  // debounce 50ms
            if (mqttmgr::isConnected() && batchCount > 0) {
                mqttmgr::publishPeriodic(batch, batchCount, g_bootId, g_packetSeq);
                g_packetSeq++;
                if (g_packetSeq == 0) g_packetSeq = 1;  // rollover protection
            }
            batchCount = 0;
            batchActive = false;
            lastPublish = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Yield ke task lain
    }
}

// ============================================================================
// TASK: WiFi/MQTT Reconnect (Core 1, priority 6)
// ============================================================================
static void task_wifi_mqtt_reconnect(void* pv) {
    for (;;) {
        wifimgr::checkAndReconnect();
        mqttmgr::loop();   // juga menangani reconnect MQTT + memproses callback
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ============================================================================
// TASK: FFT Buffer Sender (Core 1, priority 5)
// ============================================================================
static void task_fft_buffer_sender(void* pv) {
    static float window[FFTBuffer::BUFFER_SIZE];
    uint64_t lastSlot = 0;

    for (;;) {
        uint32_t interval = g_fftSendIntervalMs;
        if (interval < FFT_SEND_INTERVAL_MIN_MS) interval = FFT_SEND_INTERVAL_MIN_MS;
        if (interval > FFT_SEND_INTERVAL_MAX_MS) interval = FFT_SEND_INTERVAL_MAX_MS;

        // Kirim sekali per slot epoch (now/interval) supaya node_01 & node_02
        // mengirim pada boundary waktu yang sama BILA RTC keduanya sinkron,
        // sehingga window raw lebih mudah disejajarkan untuk FDD.
        uint64_t nowMs = rtc::getEpochMillis();
        uint64_t slot = nowMs / interval;
        if (slot != lastSlot) {
            lastSlot = slot;
            bool got = false;
            uint64_t start_ms = 0, end_ms = 0;
            if (xSemaphoreTake(fftBufferMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                got = fftBuffer.getBuffer(window, start_ms, end_ms);
                xSemaphoreGive(fftBufferMutex);
            }
            if (got && mqttmgr::isConnected()) {
                mqttmgr::publishFFTWindow(window, FFTBuffer::BUFFER_SIZE, g_samplingRateHz, start_ms, end_ms);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ============================================================================
// TASK: Configuration Handler (Core 1, priority 4)
// ============================================================================
static void task_config_handler(void* pv) {
    ConfigCommand cmd;

    for (;;) {
        if (xQueueReceive(configCommandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            switch (cmd.type) {
                case ConfigCommandType::SET_SAMPLING_RATE: {
                    bool valid = (cmd.value >= SAMPLE_RATE_MIN_HZ && cmd.value <= SAMPLE_RATE_MAX_HZ);
                    if (valid) {
                        g_samplingRateHz = (uint16_t)cmd.value;
                        if (xSemaphoreTake(fftBufferMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                            fftBuffer.clear();
                            xSemaphoreGive(fftBufferMutex);
                        }
                    }
                    mqttmgr::publishConfigAck("sampling_rate", cmd.value, g_samplingRateHz, valid ? "applied" : "rejected");
                    break;
                }
                case ConfigCommandType::SET_PUBLISH_INTERVAL: {
                    bool valid = (cmd.value >= MQTT_PUBLISH_INTERVAL_MIN_MS && cmd.value <= MQTT_PUBLISH_INTERVAL_MAX_MS);
                    if (valid) {
                        g_publishIntervalMs = cmd.value;
                    }
                    mqttmgr::publishConfigAck("publish_interval", cmd.value, g_publishIntervalMs, valid ? "applied" : "rejected");
                    break;
                }
                case ConfigCommandType::SET_RAW_WINDOW_INTERVAL: {
                    bool valid = (cmd.value >= FFT_SEND_INTERVAL_MIN_MS && cmd.value <= FFT_SEND_INTERVAL_MAX_MS);
                    if (valid) {
                        g_fftSendIntervalMs = cmd.value;
                    }
                    mqttmgr::publishConfigAck("raw_window_interval", cmd.value, g_fftSendIntervalMs, valid ? "applied" : "rejected");
                    break;
                }
                case ConfigCommandType::SET_SENSITIVITY_GAIN: {
                    bool valid = (cmd.value == 1 || cmd.value == 2 || cmd.value == 4 || cmd.value == 8);
                    if (valid) {
                        g_sensitivityGain = (uint8_t)cmd.value;
                        mpu9250::setSensitivityGain(g_sensitivityGain);
                    }
                    mqttmgr::publishConfigAck("sensitivity_gain", cmd.value, g_sensitivityGain, valid ? "applied" : "rejected");
                    break;
                }
                case ConfigCommandType::RECALIBRATE:
                    g_recalibrateRequested = true;
                    mqttmgr::publishConfigAck("recalibrate", 1, 1, "applied");
                    break;
                case ConfigCommandType::RESTART:
                    g_restartRequested = true;
                    mqttmgr::publishConfigAck("restart", 1, 1, "applied");
                    break;
                case ConfigCommandType::REQUEST_STATUS:
                    g_statusRequested = true;
                    break;
                case ConfigCommandType::CALIBRATE_ACCEL:
                    g_calibrateAccelRequested = true;
                    mqttmgr::publishConfigAck("calibrate_accel", 1, 1, "applied");
                    break;
                case ConfigCommandType::SESSION_START:
                    sdlog::startSession(cmd.value);
                    mqttmgr::publishConfigAck("session_start", cmd.value, cmd.value, "applied");
                    break;
                case ConfigCommandType::SESSION_STOP:
                    sdlog::stopSession();
                    mqttmgr::publishConfigAck("session_stop", 0, 0, "applied");
                    break;
            }
        }

        // Restart dieksekusi di sini (bukan langsung di callback MQTT) supaya
        // ada jeda singkat untuk flush SD/MQTT sebelum reboot.
        if (g_restartRequested) {
            sdlog::flush();
            vTaskDelay(pdMS_TO_TICKS(200));
            ESP.restart();
        }

        if (g_statusRequested) {
            g_statusRequested = false;
            mqttmgr::publishStatus(millis() / 1000, ESP.getFreeHeap(), (int8_t)WiFi.RSSI(),
                                    sdlog::isReady(),
                                    g_droppedSamples > 0
                                        ? (100.0f * g_droppedSamples / (float)(g_sequenceCounter + 1))
                                        : 0.0f);
        }
    }
}

// ============================================================================
// TASK: Serial Debug (Core 1, priority 2)
// ============================================================================
static void task_serial_debug(void* pv) {
    for (;;) {
        if (Serial.available()) {
            String cmdline = Serial.readStringUntil('\n');
            cmdline.trim();
            if (cmdline == "status") {
                Serial.printf("[STATUS] seq=%lu dropped=%lu heap=%u wifi=%d mqtt=%d sd=%d\n",
                              g_sequenceCounter, g_droppedSamples, ESP.getFreeHeap(),
                              wifimgr::isConnected(), mqttmgr::isConnected(), sdlog::isReady());
            } else if (cmdline == "recalibrate") {
                g_recalibrateRequested = true;
                Serial.println("[CMD] recalibrate requested");
            } else if (cmdline == "cal_accel") {
                g_calibrateAccelRequested = true;
                Serial.println("[CMD] accel calibration requested");
            } else if (cmdline.startsWith("rate ")) {
                g_samplingRateHz = (uint16_t)cmdline.substring(5).toInt();
                Serial.printf("[CMD] sampling rate -> %u Hz\n", g_samplingRateHz);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================================
// PUBLIC: createAll()
// ============================================================================
namespace taskmgr {

void createAll() {
    rawDataQueue     = xQueueCreate(QLEN_RAW_SENSOR_DATA, sizeof(MPU9250Data));
    sdWriteQueue      = xQueueCreate(QLEN_SD_WRITE, sizeof(ProcessedData));
    mqttPublishQueue  = xQueueCreate(QLEN_MQTT_PUBLISH, sizeof(ProcessedData));
    configCommandQueue = xQueueCreate(QLEN_CONFIG_COMMAND, sizeof(ConfigCommand));

    fftBufferMutex = xSemaphoreCreateMutex();

    kalman::init(&kalmanPitch, KALMAN_Q_ANGLE, KALMAN_Q_BIAS, KALMAN_R_MEASURE);
    kalman::init(&kalmanRoll,  KALMAN_Q_ANGLE, KALMAN_Q_BIAS, KALMAN_R_MEASURE);

    xTaskCreatePinnedToCore(task_sensor_sampling, "sampling",
        STACK_SENSOR_SAMPLING, nullptr, PRIO_SENSOR_SAMPLING, nullptr, CORE_SAMPLING);

    xTaskCreatePinnedToCore(task_data_processing, "processing",
        STACK_DATA_PROCESSING, nullptr, PRIO_DATA_PROCESSING, nullptr, CORE_PROCESSING);

    xTaskCreatePinnedToCore(task_sd_writer, "sd_writer",
        STACK_SD_WRITER, nullptr, PRIO_SD_WRITER, nullptr, CORE_SD_WRITER);

    xTaskCreatePinnedToCore(task_mqtt_publisher, "mqtt_pub",
        STACK_MQTT_PUBLISHER, nullptr, PRIO_MQTT_PUBLISHER, nullptr, CORE_MQTT_PUBLISHER);

    xTaskCreatePinnedToCore(task_wifi_mqtt_reconnect, "net_reconnect",
        STACK_WIFI_RECONNECT, nullptr, PRIO_WIFI_MQTT_RECONNECT, nullptr, CORE_WIFI_RECONNECT);

    xTaskCreatePinnedToCore(task_fft_buffer_sender, "fft_sender",
        STACK_FFT_SENDER, nullptr, PRIO_FFT_BUFFER_SENDER, nullptr, CORE_FFT_SENDER);

    xTaskCreatePinnedToCore(task_config_handler, "config_handler",
        STACK_CONFIG_HANDLER, nullptr, PRIO_CONFIG_HANDLER, nullptr, CORE_CONFIG_HANDLER);

    xTaskCreatePinnedToCore(task_serial_debug, "serial_debug",
        STACK_SERIAL_DEBUG, nullptr, PRIO_SERIAL_DEBUG, nullptr, CORE_SERIAL_DEBUG);
}

} // namespace taskmgr
