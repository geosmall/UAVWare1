#include "uvos_spi.h"
#include "hardware.h"

void UVOS_SPI_init( SPI_TypeDef *SPIx )
{
  SET_BIT( SPIx->CR1, SPI_CR1_SPE ); // Enable the specified SPI peripheral
}

uint8_t UVOS_SPI_sendbyte( SPI_TypeDef *SPIx, uint8_t data )
{
  while ( ( SPIx->SR & SPI_SR_TXE ) == 0 ) {} // Wait if TXE cleared, Tx FIFO is full.
  SPIx->DR = data;
  while ( ( SPIx->SR & SPI_SR_RXNE ) == 0 ) {} // Wait if RNE cleared, Rx FIFO is empty.
  return SPIx->DR;
}

void UVOS_SPI_sendbyte_slow( SPI_TypeDef *SPIx, uint8_t data )
{
  MODIFY_REG( SPIx->CR1, SPI_CR1_BR, BAUDRATE_SLOW_PRESCALER );
  UVOS_SPI_sendbyte( SPIx, data );
  MODIFY_REG( SPIx->CR1, SPI_CR1_BR, BAUDRATE_FAST_PRESCALER );
}

uint8_t UVOS_SPI_sendzerorecvbyte( SPI_TypeDef *SPIx )
{
  while ( ( SPIx->SR & SPI_SR_TXE ) == 0 ) {} // Wait if TXE cleared, Tx FIFO is full.
  SPIx->DR = 0;
  while ( ( SPIx->SR & SPI_SR_RXNE ) == 0 ) {} // Wait if RNE cleared, Rx FIFO is empty.
  return SPIx->DR;
}

uint8_t UVOS_SPI_sendzerorecvbyte_slow( SPI_TypeDef *SPIx )
{
  MODIFY_REG( SPIx->CR1, SPI_CR1_BR, BAUDRATE_SLOW_PRESCALER );
  return UVOS_SPI_sendzerorecvbyte( SPIx );
  MODIFY_REG( SPIx->CR1, SPI_CR1_BR, BAUDRATE_FAST_PRESCALER );
}
