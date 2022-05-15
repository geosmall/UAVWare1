#pragma once

#include "src/uvos/uvos.h"

#ifdef __cplusplus
extern "C" {
#endif

#define W25_CMD_PAGE_PROGRAM  0x02
#define W25_CMD_DATA_READ     0x03

#define W25_CMD_READ_STATUS1  0x05

#define W25_CMD_WRITE_ENABLE  0x06

#define W25_CMD_GET_ID        0x9F
#define W25_ID0_WINBOND       0xEF

#define W25_CMD_GET_SERIAL    0x4B

#define W25_CMD_SECTOR_ERASE_4K 0x20
#define W25_CMD_BLOCK_ERASE_32K 0x52
#define W25_CMD_BLOCK_ERASE_64K 0xD8
#define W25_CMD_CHIP_ERASE      0xC7

#define W25_ERR_OK           0U            /* OK */
#define W25_ERR_OVERFLOW     4U            /* Timer overflow. */
#define W25_ERR_FAILED       27U           /* Requested functionality or process failed. */

#define W25_SERIAL_BUF_SIZE  (8)
uint8_t W25_ReadSerialNumber( uint8_t * buf, size_t bufSize );

#define W25_ID_BUF_SIZE  (3)
uint8_t W25_ReadID( uint8_t * buf, size_t bufSize );

uint8_t W25_ReadStatus1( uint8_t * status );

bool W25_isBusy( void );

void W25_WaitIfBusy( void );

uint8_t W25_Read( uint32_t address, uint8_t * buf, size_t bufSize );

uint8_t W25_EraseAll( void );

uint8_t W25_EraseSector4K( uint32_t address );

uint8_t W25_EraseBlock32K( uint32_t address );

uint8_t W25_EraseBlock64K( uint32_t address );

/*!
 * Program a page with data
 * \param address, should be aligned on page (256 bytes) if programming 256 bytes
 * \param data pointer to data
 * \param dataSize size of data in bytes, max 256
 * \return error code, ERR_OK for no error
 */
uint8_t W25_ProgramPage( uint32_t address, const uint8_t * data, size_t dataSize );

uint8_t W25_GetCapacity( const uint8_t * id, uint32_t * capacity );

uint32_t W25_Init( void );

uint8_t W25_PrintStatus( void );

#ifdef __cplusplus
}
#endif