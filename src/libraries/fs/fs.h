/*
 * fs.h
 *
 *  Created on: 06.01.2019
 *      Author: Erich Styger
 */

#ifndef SOURCES_FS_H_
#define SOURCES_FS_H_

#include <stdint.h>
#include <stdbool.h>

#define FS_ERR_OK                          0x00U /*!< OK */
#define FS_ERR_FAILED                      0x01U /*!< Requested functionality or process failed. */

#ifdef __cplusplus
extern "C" {
#endif

// #include "CLS1.h"

// uint8_t FS_ParseCommand(const unsigned char* cmd, bool *handled, const CLS1_StdIOType *io);

uint8_t FS_Init(void);

uint8_t FS_Format(void);

uint8_t FS_Mount(void);

uint8_t FS_RunBenchmark(void);

#ifdef __cplusplus
}
#endif

#endif /* SOURCES_FS_H_ */
