#include "uvos_mpu.h"
#include "../core/spi.h"
#include "uvos_spi.h"
#include "hardware.h"

void UVOS_MPU_cson( void )
{
  LL_GPIO_ResetOutputPin( SPI_MPU_NSS_GPIO_Port, SPI_MPU_NSS_Pin );
}

void UVOS_MPU_csoff( void )
{
  LL_GPIO_SetOutputPin( SPI_MPU_NSS_GPIO_Port, SPI_MPU_NSS_Pin );
}

void UVOS_MPU_init( void )
{
  UVOS_SPI_init( MPU_SPI );  // MPU_SPI defined in hardware.h
}

void UVOS_MPU_sendbyte_slow( int data )
{
  UVOS_SPI_sendbyte_slow( MPU_SPI, data );
}

void UVOS_MPU_sendbyte( int data )
{
  UVOS_SPI_sendbyte( MPU_SPI, data );
}

int UVOS_MPU_sendzerorecvbyte( void )
{
  return UVOS_SPI_sendzerorecvbyte( MPU_SPI );
}

int UVOS_MPU_sendzerorecvbyte_slow( void )
{
  return UVOS_SPI_sendzerorecvbyte_slow( MPU_SPI );
}