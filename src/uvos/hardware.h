/* See STM32F4yyxx Reference Manual, Section 2.3 Memory map
STM32F1/303xB : HCLK= 72MHz, PCLK1(APB1)=36MHz, PCLK2(APB2)=72MHz
STM32F411xE   : HCLK= 96MHz, PCLK1(APB1)=48MHz, PCLK2(APB2)=96MHz
STM32F405xG   : HCLK=168MHz, PCLK1(APB1)=84MHz, PCLK2(APB2)=168MHz

// Reference STM32 datasheet processor block diagram
For SPI_1, SPI_4, SPI_5 and SPI_6
  - CLK source is PCKL2
For SPI_2 and SPI_3
  - CLK source is PCKL1

F411
For TIM_1, TIM_9, TIM_10, TIM_11
  - CLK source is PCKL2
For TIM_2, TIM_3, TIM_4, TIM_5
  - CLK source is PCKL1

F405
For TIM_1, TIM_8, TIM_9, TIM_10, TIM_11
  - CLK source is PCKL2
For TIM_2, TIM_3, TIM_4, TIM_5, TIM_12, TIM_13, TIM_14
  - CLK source is PCKL1

*/

#define CONCAT2(x, y) x ## y
#define CONCAT(x, y) CONCAT2(x, y)
#define __define_stream_irq( x, y )  void DMA##x##_Stream##y##_IRQHandler ( void )
#define DMA_IRQ_FN( x, y ) __define_stream_irq( x, y )

#if defined(FC_BOARD_OMNIBUS) || defined(FC_BOARD_F4XSD)
	// #define RX_SOFTSPI_4WIRE
	// #define MPU_SPI SPI1
	#define SYS_CLOCK_FREQ_MHZ 168
	#define RX_UART (USART1) // requires Rx DMA
	#define TELEM_UART (USART2)
	#define DEBUG_UART (USART2)
	#define TIMEBASE_TIM (TIM11) // on PCLK2 = 84 MHz 
	#define MPU_SPI (SPI1)       // on PCLK2 = 84 MHz
	#define MX_SPI_INIT_FUNC MX_SPI1_Init
	#define BAUDRATE_FAST_PRESCALER LL_SPI_BAUDRATEPRESCALER_DIV4 // 21 MBit/s
	#define BAUDRATE_SLOW_PRESCALER LL_SPI_BAUDRATEPRESCALER_DIV128 // 0.65625 MBit/s
#elif defined (FC_BOARD_NOXE_V1)
	// #define RX_SPIx SPI2
	// #define MPU_SPI SPI1
	#define SYS_CLOCK_FREQ_MHZ 96
	#define RX_UART (USART2)  // requires Rx DMA
	#define TELEM_UART (USART1)
	#define DEBUG_UART (USART1)
	#define TIMEBASE_TIM (TIM11) // on PCLK2 = 96 MHz
	#define MPU_SPI (SPI1)       // on PCLK2 = 96 MHz
	#define MX_SPI_INIT_FUNC MX_SPI1_Init
	#define BAUDRATE_FAST_PRESCALER LL_SPI_BAUDRATEPRESCALER_DIV4 // 24 MBit/s
	#define BAUDRATE_SLOW_PRESCALER LL_SPI_BAUDRATEPRESCALER_DIV128 // 0.75 MBit/s
#elif defined (ARDUINO_NUCLEO_F411RE)
	#define SYS_CLOCK_FREQ_MHZ 96

	#define RX_USART (USART1) // Rx uses Rx DMA
	#define RX_USART_IRQn (USART1_IRQn)
	#define RX_USART_IRQHandler (USART1_IRQHandler)

	// Per STM32 ref manual, DMA request mapping USART1_RX
  #define RX_DMAx (DMA2)
  #define RX_DMA_STREAMy (LL_DMA_STREAM_2)
	#define RX_DMA_CHANNEL (LL_DMA_CHANNEL_4)
  #define RX_DMA_IRQn (DMA2_Stream2_IRQn)
  #define RX_DMA_STREAM_IRQHandler (DMA2_Stream2_IRQHandler)
	#define RX_DMA_IsActiveFlag_TCx (LL_DMA_IsActiveFlag_TC2)
	#define RX_DMA_ClearFlag_TCx (LL_DMA_ClearFlag_TC2)

	#define TELEM_USART (USART2) // Telem uses Tx DMA
	#define TELEM_USART_IRQn (USART2_IRQn)
	#define TELEM_USART_IRQHandler (USART2_IRQHandler)

	// Per STM32 ref manual, DMA request mapping USART2_TX
  #define TELEM_DMAx (DMA1)
  #define TELEM_DMA_STREAMy (LL_DMA_STREAM_6)
	#define TELEM_DMA_CHANNEL (LL_DMA_CHANNEL_4)
  #define TELEM_DMA_IRQn (DMA1_Stream6_IRQn)
  #define TELEM_DMA_STREAM_IRQHandler (DMA1_Stream6_IRQHandler)
	#define TELEM_DMA_IsActiveFlag_TCx (LL_DMA_IsActiveFlag_TC6)
	#define TELEM_DMA_ClearFlag_TCx (LL_DMA_ClearFlag_TC6)

	#define DEBUG_USART (USART6)

	#define TIMEBASE_TIM (TIM2)   	// 32 bit TIM on PCLK1 = 96 MHz

	#define SCHEDULER_TIM (TIM11)		// 16 bit TIM on PCLK2 = 96 MHz
  #define SCHEDULER_TIM_IRQn (TIM1_TRG_COM_TIM11_IRQn)
  #define SCHEDULER_TIM_IRQHandler (TIM1_TRG_COM_TIM11_IRQHandler)

	#define MPU_SPI (SPI1)       		// on PCLK2 = 96 MHz
	#define FLASH_SPI (SPI2)     		// on PCLK1 = 48 MHz
	#define BAUDRATE_FAST_PRESCALER (LL_SPI_BAUDRATEPRESCALER_DIV8) // 12 MBit/s (slower for wired MPU)
	#define BAUDRATE_SLOW_PRESCALER (LL_SPI_BAUDRATEPRESCALER_DIV128) // 0.75 MBit/s
#else
	#error 'no board defined in hardware.h'
#endif

