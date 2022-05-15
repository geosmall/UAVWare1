#pragma once

#include <stdint.h>
#include "../core/main.h"

#ifdef __cplusplus
extern "C" {
#endif

void UVOS_MPU_cson( void );
void UVOS_MPU_csoff( void );
void UVOS_MPU_init( void );
void UVOS_MPU_sendbyte_slow( int data );
void UVOS_MPU_sendbyte( int data );
int UVOS_MPU_sendzerorecvbyte( void );
int UVOS_MPU_sendzerorecvbyte_slow( void );

#ifdef __cplusplus
}
#endif