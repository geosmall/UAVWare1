#pragma once

#include <stdint.h>
#include "../core/main.h"

#ifdef __cplusplus
extern "C" {
#endif

void UVOS_SPI_init( SPI_TypeDef *SPIx );
void UVOS_SPI_sendbyte_slow( SPI_TypeDef *SPIx, uint8_t data );
uint8_t UVOS_SPI_sendbyte( SPI_TypeDef *SPIx, uint8_t data );
uint8_t UVOS_SPI_sendzerorecvbyte_slow( SPI_TypeDef *SPIx );
uint8_t UVOS_SPI_sendzerorecvbyte( SPI_TypeDef *SPIx );

#ifdef __cplusplus
}
#endif