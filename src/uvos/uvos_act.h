#pragma once

#include "uvos_stm32.h"
#include "stm32yyxx_ll_tim.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	ePWM = 0,	  /* PWM output type (default) */
	eONESHOT,		/* Oneshot output type */
	// eDHOT,		/* DShot output type */
	// eIBUS,		/* IBus output type */
	eInvalid	  /* Used as an 'invalid state' value */
} eOutputType;

struct act_cfg {
	eOutputType 	type;
	stm32_gpio_t	pin;
	TIM_TypeDef * TIM_Inst;
	uint32_t   *  TIM_CCRx;
	uint32_t      TIM_CH;
	uint32_t      TIM_CLK;
};

typedef struct act_cfg act_cfg_t;

void UVOS_ACT_init( void );
void UVOS_ACT_enable( void );
void UVOS_ACT_update_outputs( void );

#ifdef __cplusplus
}
#endif