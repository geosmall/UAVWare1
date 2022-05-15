#pragma once

#include "../core/usart.h"  // header from STM32CubeMX code generate
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* User configuration */
#define IBUS_UART       (huart2)


/* Defines */
#define IBUS_LENGTH         32    // 32 bytes
#define IBUS_COMMAND40      0x40  // Command to set servo or motor speed is always 0x40
#define IBUS_MAX_CHANNLES   14


/* Main Functions */
void ibus_init();
bool ibus_read(uint16_t ibus_channel[], uint8_t ch_num);


/* Sub Functions */
bool ibus_is_valid();
bool ibus_checksum();
void ibus_update(uint16_t ibus_channel[], uint8_t ch_num);
bool is_ibus_lost();

#ifdef __cplusplus
}
#endif