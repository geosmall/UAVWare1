#pragma once

// #include "../core/main.h"

// #ifdef __cplusplus
// extern "C" {
// #endif

void RC_TASK_init ( void );
void RC_TASK_update ( void );

void SYS_STATS_TASK_init ( void );
void SYS_STATS_TASK_update( void );

void HEARTBEAT_TASK_init ( void );
void HEARTBEAT_TASK_update ( void );

// #ifdef __cplusplus
// }
// #endif