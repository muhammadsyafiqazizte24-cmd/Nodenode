#ifndef SHM_MPU9250_REGISTERS_H
#define SHM_MPU9250_REGISTERS_H

// ============================================================================
// mpu9250_registers.h — Register map MPU9250
// PERTAHANKAN: identik dengan program referensi, hanya dipindah ke file
// terpisah untuk modularitas.
// ============================================================================

#define REG_WHO_AM_I        0x75
#define REG_PWR_MGMT_1      0x6B
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_CONFIG2   0x1D
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_XOUT_H     0x43

// ---------------------------------------------------------------------------
// AK8963 Magnetometer (internal ke MPU9250, diakses via I2C master pass-through
// atau via register bypass). Register disediakan di sini untuk pembacaan
// magnetometer mentah; JANGAN dipakai untuk fusion selain penyimpanan mentah,
// sesuai spesifikasi ("dibaca & disimpan, belum untuk fusion").
// ---------------------------------------------------------------------------
#define REG_INT_PIN_CFG     0x37   // bit1 (BYPASS_EN) untuk akses langsung AK8963
#define REG_USER_CTRL       0x6A
#define AK8963_I2C_ADDR      0x0C
#define AK8963_REG_WIA        0x00
#define AK8963_REG_CNTL1      0x0A
#define AK8963_REG_ST1        0x02
#define AK8963_REG_HXL         0x03
#define AK8963_REG_ASAX        0x10

#endif // SHM_MPU9250_REGISTERS_H
