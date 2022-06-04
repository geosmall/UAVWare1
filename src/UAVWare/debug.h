#pragma once

#include "../core/main.h"

#ifdef __cplusplus
extern "C" {
#endif

void debug_init(void);
void debug_putc(uint8_t ch);
void debug_puts( char * data, uint16_t len );
void debug_cls(void);
void debug(const char *data);
void debug_cls(void);
void debug_put_hex8(uint8_t val);
void debug_put_hex16(uint16_t val);
void debug_put_hex32(uint32_t val);
void debug_put_uint8(uint8_t c);
void debug_put_int8(int8_t c);
void debug_put_uint16(uint16_t c);
void debug_put_fixed2(uint16_t c);

#ifdef __cplusplus
}
#endif