#include "uvos.h"
#include "uvos_time.h"
#include "hardware.h"


time_tick_callback_t tim_tick_callback = NULL;

void UVOS_TIME_RegisterTickCallback( time_tick_callback_t tim_callback )
{
	tim_tick_callback = tim_callback;
}

#if defined ( SCHEDULER_TIM_IRQHandler )
void SCHEDULER_TIM_IRQHandler( void )
{
	/* Check for DMA Transfer Complete interrupt */
	if ( LL_TIM_IsEnabledIT_UPDATE( SCHEDULER_TIM ) && LL_TIM_IsActiveFlag_UPDATE( SCHEDULER_TIM ) ) {
		/* Clear the update interrupt flag*/
		LL_TIM_ClearFlag_UPDATE( SCHEDULER_TIM );
		if ( tim_tick_callback != NULL ) {
			tim_tick_callback();
		}
	}
}
#endif // defined ( SCHEDULER_TIM_IRQHandler )

void UVOS_TIME_init( void )
{
	/* TIMEBASE_TIM shall be 32 bit free running counter with 1 uSec tick */

	/* Set the pre-scaler value to have TIMBASE_TIM counter clock equal to 1MHz */
	LL_TIM_SetPrescaler( TIMEBASE_TIM, __LL_TIM_CALC_PSC( ( SystemCoreClock ), 1000000 ) );
	/* Enable counter */
	LL_TIM_EnableCounter( TIMEBASE_TIM );
}

void UVOS_TIME_sched_init( const uint32_t tick_hz )
{
	/* SCHEDULER_TIM shall generate scheduler interrupt at LOOP_FREQ_HZ */

	uint32_t InitialAutoreload = 0;

	/* Set the pre-scaler value to have TIMBASE_TIM counter clock equal to 12 MHZ */
	LL_TIM_SetPrescaler( SCHEDULER_TIM, __LL_TIM_CALC_PSC( ( SystemCoreClock ), 12000000 ) );
	// ARR auto-reload reg to deliver TIMEBASE clock frequency
	InitialAutoreload = __LL_TIM_CALC_ARR( SystemCoreClock, LL_TIM_GetPrescaler( SCHEDULER_TIM ), tick_hz );
	LL_TIM_SetAutoReload( SCHEDULER_TIM, InitialAutoreload );

	/* Configure the NVIC to handle SCHEDULER_TIM update interrupt */
	NVIC_SetPriority( SCHEDULER_TIM_IRQn, 0 );
	NVIC_EnableIRQ( SCHEDULER_TIM_IRQn );

	/* Enable the update interrupt */
	LL_TIM_EnableIT_UPDATE( SCHEDULER_TIM );

}

inline uint32_t UVOS_TIME_sched_start( void )
{
	/* Enable counter */
	LL_TIM_EnableCounter( SCHEDULER_TIM );
	/* Force update generation */
	LL_TIM_GenerateEvent_UPDATE( SCHEDULER_TIM );
}

inline uint32_t UVOS_TIME_sched_stop( void )
{
	/* Disable counter */
	LL_TIM_DisableCounter( SCHEDULER_TIM );
}

inline uint32_t UVOS_TIME_gettime_us( void )
{
	return TIMEBASE_TIM->CNT;
}

inline uint32_t UVOS_TIME_gettime_ms( void )
{
	return ( TIMEBASE_TIM->CNT / 1000U );
}

void UVOS_TIME_delay_us( uint32_t us )
{
	const uint32_t wait_until = UVOS_TIME_gettime_us() + us + 1; // +1 to wait at least the specified time
	if ( wait_until < UVOS_TIME_gettime_us() ) { // check for overflow
		while ( UVOS_TIME_gettime_us() > wait_until );
	}
	while ( UVOS_TIME_gettime_us() < wait_until );
}
