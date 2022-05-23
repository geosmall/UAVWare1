#include "mpu.h"
#include "../uvos/uvos_mpu.h"
#include "../uvos/uvos_spi.h"
#include "../uvos/uvos_time.h"

// Private function prototypes
// void mpu_writereg( uint8_t address, uint8_t value );
// uint8_t mpu_readreg( uint8_t address );
// void mpu_readdata( uint8_t address, uint32_t data[], uint8_t size );

// Last status from register user control
uint8_t _last_stat_user_ctrl;

// buffer for reading from sensor
uint8_t _buffer[14];

static void mpu_writereg( uint8_t address, uint8_t value )
{
	UVOS_MPU_cson();
	UVOS_MPU_sendbyte_slow( address );
	UVOS_MPU_sendbyte_slow( value );
	UVOS_MPU_csoff();
	UVOS_TIME_delay_us( 1 );
}

static uint8_t mpu_readreg( uint8_t address )
{
	UVOS_MPU_cson();
	UVOS_MPU_sendbyte_slow( address | 0x80 );
	uint8_t value = UVOS_MPU_sendzerorecvbyte_slow();
	UVOS_MPU_csoff();
	UVOS_TIME_delay_us( 1 );
	return value;
}

static bool mpu_writereg_and_verify( uint8_t address, uint8_t value )
{
	UVOS_MPU_cson();
	UVOS_MPU_sendbyte_slow( address );
	UVOS_MPU_sendbyte_slow( value );
	UVOS_MPU_csoff();
	UVOS_TIME_delay_us( 10 );
	_buffer[0] = mpu_readreg( address ); // read back the register
	// check the read back register against the written register
	if ( _buffer[0] == value ) {
		return true;
	}
	return false;
}

static void mpu_readdata( uint8_t address, uint8_t data[], uint8_t size )
{
	UVOS_MPU_cson();
	UVOS_MPU_sendbyte( address | 0x80 );
	for ( uint8_t i = 0; i < size; ++i ) {
		data[ i ] = UVOS_MPU_sendzerorecvbyte();
	}
	UVOS_MPU_csoff();
}

/* Write individual bits to MPU register */
static bool mpu_set_registerbits( uint8_t address, uint8_t mask, uint8_t value )
{
	uint8_t temp = mpu_readreg( address );	// get current register contents
	temp &= ~( mask );											// zero out temp bits to be written
	temp |= value;													// bitwise OR bits to be set
	bool r = mpu_writereg_and_verify( address, temp );
	return r;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")

// Check whoami for sensor type
static invensense_type_e mpu_whoami( void )
{
	uint8_t whoami = mpu_readreg(MPUREG_WHOAMI);
	switch (whoami) {
	case MPU_WHOAMI_6000:
		return Invensense_MPU6000;
		break;
	case MPU_WHOAMI_6500:
		return Invensense_MPU6500;
		break;
	case MPU_WHOAMI_MPU9250:
	case MPU_WHOAMI_MPU9255:
		return Invensense_MPU9250;
		break;
	case MPU_WHOAMI_20608D:
	case MPU_WHOAMI_20608G:
		return Invensense_ICM20608;
		break;
	case MPU_WHOAMI_20602:
		return Invensense_ICM20602;
		break;
	case MPU_WHOAMI_20601:
		return Invensense_ICM20601;
		break;
	case MPU_WHOAMI_ICM20789:
	case MPU_WHOAMI_ICM20789_R1:
		return Invensense_ICM20789;
		break;
	case MPU_WHOAMI_ICM20689:
		return Invensense_ICM20689;
		break;
	default :
		return Invalid_IMU_ID;
		break;
	}
}

#pragma GCC pop_options

// Initialize MPU sensor, returns TRUE if recognized sensor found and
// successfully reset, else returns FALSE
#define MAX_MPU_TRIES 5
bool mpu_init( void )
{
	UVOS_MPU_csoff();
	UVOS_MPU_init();
	UVOS_TIME_delay_us( 10000 );
	// MPU POWER ON
	mpu_writereg( MPUREG_PWR_MGMT_1, 0x01 );
	// MPU Gyro and Accel ON
	mpu_writereg( MPUREG_PWR_MGMT_2, 0x00 );
	UVOS_TIME_delay_us( 10000 );

	invensense_type_e id = mpu_whoami();
	if ( ( id == Invalid_IMU_ID ) || ( id != Invensense_MPU6000 && id != Invensense_ICM20602 ) ) {
		return false;
	}

	// Chip reset
	uint8_t tries;
	for ( tries = 0; tries < MAX_MPU_TRIES; tries++ ) {
		_last_stat_user_ctrl = mpu_readreg(MPUREG_USER_CTRL);

		/* First disable the master I2C to avoid hanging the slaves on the
		 * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
		 * is used */
		if (_last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN) {
			_last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
			mpu_writereg(MPUREG_USER_CTRL, _last_stat_user_ctrl);
			UVOS_TIME_delay_us( 10000 );
		}

		/* reset device */
		mpu_writereg(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
		UVOS_TIME_delay_us( 100000 );

		/* Disable I2C bus if SPI selected (Recommended in Datasheet to be
		 * done just after the device is reset) */
		_last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
		mpu_writereg(MPUREG_USER_CTRL, _last_stat_user_ctrl);

		// Wake up device and select GyroZ clock. Note that the  Invensense device
		// starts up in sleep mode, and it can take some time for it to come out of sleep
		mpu_writereg(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
		UVOS_TIME_delay_us( 5000 );

		// check it has woken up
		if (mpu_readreg(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
			break;
		}

		UVOS_TIME_delay_us( 10000 );

		// Return true if Invensense IMU has new data available for reading
		uint8_t status = mpu_readreg(MPUREG_INT_STATUS);
		if ((status & BIT_RAW_RDY_INT) != 0) {
			break;
		}
	}

	if (tries == MAX_MPU_TRIES) {
		// printf("Failed to boot Invensense MAX_MPU_TRIES times\n");
		return false;
	}

	if (id == Invensense_ICM20608 ||
	    id == Invensense_ICM20602 ||
	    id == Invensense_ICM20601) {
		// this avoids a sensor bug for these sensors if used, taken from
		// https://github.com/ArduPilot AP_InertialSensor_Invensense.cpp#L345
		mpu_writereg(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE);
	}

	return true;

}

bool mpu_set_fullscale_gyro_range( enum Gyro_FS_Range range )
{
	return mpu_set_registerbits( MPUREG_GYRO_CONFIG, BITS_GYRO_FS_MASK, range );
}

bool mpu_set_fullscale_accel_range( enum Accel_FS_Range range )
{
	return mpu_set_registerbits( MPUREG_ACCEL_CONFIG, BITS_ACCEL_FS_MASK, range );
}

/**
  * @brief Get accelerometer and gyroscope datas
  * @param  accel: the value of accelerometer
  *         gyro:the value of gyroscope
  *         temp:the pointer to temperature
  * @retval None
  */
void mpu_read_imu( int16_t accel[3], int16_t gyro[3], int16_t *temp )
{
	mpu_readdata( MPUREG_ACCEL_XOUT_H, _buffer, sizeof(_buffer) );
	accel[0] = (_buffer[0] << 8) | _buffer[1];
	accel[1] = (_buffer[2] << 8) | _buffer[3];
	accel[2] = (_buffer[4] << 8) | _buffer[5];
	*temp = (_buffer[6] << 8) | _buffer[7];
	gyro[0] = (_buffer[8] << 8) | _buffer[9];
	gyro[1] = (_buffer[10] << 8) | _buffer[11];
	gyro[2] = (_buffer[12] << 8) | _buffer[13];
}