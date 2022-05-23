#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "mpu_regs.h"

typedef enum invensense_type {
  Invalid_IMU_ID = 0,
  Invensense_MPU6000,
  Invensense_MPU6500,
  Invensense_MPU9250,
  Invensense_ICM20608,
  Invensense_ICM20602,
  Invensense_ICM20601,
  Invensense_ICM20789,
  Invensense_ICM20689,
} invensense_type_e;

enum Gyro_FS_Range {
  GYRO_FS_250DPS = BITS_GYRO_FS_250DPS,
  GYRO_FS_500DPS = BITS_GYRO_FS_500DPS,
  GYRO_FS_1000DPS = BITS_GYRO_FS_1000DPS,
  GYRO_FS_2000DPS = BITS_GYRO_FS_2000DPS,
};

enum Accel_FS_Range {
  ACCEL_FS_2G = BITS_ACCEL_FS_2G,
  ACCEL_FS_4G = BITS_ACCEL_FS_4G,
  ACCEL_FS_8G = BITS_ACCEL_FS_8G,
  ACCEL_FS_16G = BITS_ACCEL_FS_16G,
};

#ifdef __cplusplus
extern "C" {
#endif

bool mpu_init( void );
void mpu_read_imu( int16_t accel[3], int16_t gyro[3], int16_t *temp );
bool mpu_set_fullscale_gyro_range( enum Gyro_FS_Range range );
bool mpu_set_fullscale_accel_range( enum Accel_FS_Range range );

#ifdef __cplusplus
}
#endif