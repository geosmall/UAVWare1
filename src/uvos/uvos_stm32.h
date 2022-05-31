#pragma once

#include "uvos.h"
#include "misc.h"
#include "stm32yyxx_ll_gpio.h"
#include "stm32yyxx_ll_exti.h"
#include "stm32yyxx_ll_dma.h"

struct stm32_irq {
  void     ( *handler )( uint32_t );
  uint32_t flags;
  NVIC_InitTypeDef init;
};
typedef struct stm32_irq stm32_irq_t;

struct stm32_exti {
  LL_EXTI_InitTypeDef init;
};
typedef struct stm32_exti stm32_exti_t;

struct stm32_dma_chan {
  DMA_TypeDef *DMAx;        // DMAx Instance
  uint32_t stream;          // @group DMA_LL_EC_STREAM
  LL_DMA_InitTypeDef init;
};
typedef struct stm32_dma_chan stm32_dma_chan_t;

struct stm32_dma {
  uint32_t ahb_clk;
  struct stm32_irq irq;
  struct stm32_dma_chan rx;
  bool rx_irq_en;
  struct stm32_dma_chan tx;
  bool tx_irq_en;
};
typedef struct stm32_dma stm32_dma_t;


struct stm32_gpio {
  GPIO_TypeDef  *gpio;
  LL_GPIO_InitTypeDef init;
  // uint8_t pin_source;
};
typedef struct stm32_gpio stm32_gpio_t;
