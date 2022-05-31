#include "uvos.h"
#include "uvos_act.h"

#include "act_cfg.h"

/* Assumes timer and pin gpio already configured by CubeMX */
void UVOS_ACT_init( void )
{
  for ( uint32_t i = 0; i < ACT_NUM; i++ ) {
    if ( act_cfg_param[i].type == ePWM ) {
      /* CubeMX does not provide a PWM1 mode option so we do it here */
      LL_TIM_OC_SetMode( act_cfg_param[i].TIM_Inst, act_cfg_param[i].TIM_CH, LL_TIM_OCMODE_PWM1 );
      if ( IS_TIM_ADVANCED_INSTANCE( act_cfg_param[i].TIM_Inst ) ) {
        /* Advanced timer TIM1 needs MOE bit set in BDTR register */
        LL_TIM_EnableAllOutputs( act_cfg_param[i].TIM_Inst );
      }
      /* Enable output channel */
      LL_TIM_CC_EnableChannel( act_cfg_param[i].TIM_Inst, act_cfg_param[i].TIM_CH );
      /* Disable counter */
      LL_TIM_DisableCounter( act_cfg_param[i].TIM_Inst );
      /* Generate an update event to reload the Prescaler
         and the repetition counter value (if applicable) immediately */
      LL_TIM_GenerateEvent_UPDATE( act_cfg_param[i].TIM_Inst );
    }
  }
}

void UVOS_ACT_enable( void )
{
  for ( uint32_t i = 0; i < ACT_NUM; i++ ) {
    if ( act_cfg_param[i].type == ePWM ) {
      /* Enable counter */
      LL_TIM_EnableCounter( act_cfg_param[i].TIM_Inst );
    }
  }
}


void UVOS_ACT_update_outputs( void )
{
  for ( uint32_t i = 0; i < ACT_NUM; i++ ) {
    if ( act_cfg_param[i].type == ePWM ) {
    }
  }

  // WRITE_REG( TIM1->CCR1, 4000 );
  // *act_cfg_param[0].TIM_CCRx = 4000;
  WRITE_REG( *act_cfg_param[0].TIM_CCRx, 4000 );
  WRITE_REG( *act_cfg_param[1].TIM_CCRx, 5000 );
  WRITE_REG( *act_cfg_param[2].TIM_CCRx, 6000 );

}