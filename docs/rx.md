
RX States
Reset
- Init()
1.) WAIT_FOR_RX_IDLE
2.) DMA_STARTED
3.) DMA_FRAME_RECEIVED


https://www.st.com/resource/en/reference_manual/dm00119316-stm32f411xc-e-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf

19.6.1 Status register (USART_SR)

Bit 4 IDLE: IDLE line detected
This bit is set by hardware when an Idle Line is detected. An interrupt is generated if the 
IDLEIE=1 in the USART_CR1 register. It is cleared by a software sequence (an read to the 
USART_SR register followed by a read to the USART_DR register). 
0: No Idle Line is detected
1: Idle Line is detected
Note: The IDLE bit will not be set again until the RXNE bit has been set itself (i.e. a new idle 
line occurs).


https://stackoverflow.com/questions/43298708/stm32-implementing-uart-in-dma-mode

It is much easier to code DMA transfer (and receive of course) when you use the bare register approach, instead of the juggernaut HAL monster.

Example STM32F446 (assuming reset values in the registers)

DMA1_Stream6 -> NDTR = nTransfers;
DMA1_Stream6 -> PAR = (uint32_t)&(USART2 -> DR);
DMA1_Stream6 -> M0AR = (uint32_t)&dataBuff;
DMA1_Stream6 -> CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE; // you can enable half transfer enable as well

USART2 -> BRR = FCLK / LOWSPEED;
USART2 -> CR3 |= USART_CR3_DMAT;
USART2 -> CR1 = (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);
DMA1_Stream6 -> CR |= DMA_SxCR_EN;

quite easy - isn't it?

void DMA1_Stream6_IRQHandler(void) {  // now it does nothing only clears the flag
    if(DMA1 -> HISR & (DMA_HISR_TCIF6)) {
        DMA1 -> HIFCR |= DMA_HISR_TCIF6;
        while(!(USART2 -> SR & USART_SR_TC));
    }
}

https://www.eevblog.com/forum/microcontrollers/_best_-way-to-load-uart-data-to-ring-buffer-with-stm32hal/


Re: 'Best' way to load UART data to ring buffer with STM32/HAL
« Reply #2 on: November 15, 2016, 11:18:49 am »
Yes, you can use DMA in circular mode to continuously receive UART chars directly into a RAM circular buffer with no CPU time required, you choose how large the buffer is.
I just poll that DMA buffer from the background routine and process incoming chars there (maybe polling not acceptable for your scenario).
You can use hdmarx->Instance->CNDTR to determine where the DMA write pointer is at (and hence the number of chars received)

```
/*
 * The STM32 makes receiving chars into a large circular buffer simple
 * and requires no CPU time. The UART receiver DMA must be setup as CIRCULAR.
 */
#define CIRC_BUF_SZ       64  /* must be power of two */
static uint8_t rx_dma_circ_buf[CIRC_BUF_SZ];
static UART_HandleTypeDef *huart_cobs;
static uint32_t rd_ptr;

#define DMA_WRITE_PTR ( (CIRC_BUF_SZ - huart_cobs->hdmarx->Instance->CNDTR) & (CIRC_BUF_SZ - 1) )

void msgrx_init(UART_HandleTypeDef *huart)
{
    huart_cobs = huart;
    HAL_UART_Receive_DMA(huart_cobs, rx_dma_circ_buf, CIRC_BUF_SZ);
    rd_ptr = 0;
}

static bool msgrx_circ_buf_is_empty(void) {
    if(rd_ptr == DMA_WRITE_PTR) {
        return true;
    }
    return false;
}

static uint8_t msgrx_circ_buf_get(void) {
    uint8_t c = 0;
    if(rd_ptr != DMA_WRITE_PTR) {
        c = rx_dma_circ_buf[rd_ptr++];
        rd_ptr &= (CIRC_BUF_SZ - 1);
    }
    return c;
}
```

You mentioned parity/framing errors: the default HAL handlers just disable the uart reception. I disable those fault interrupts and use checksummed messages to detect errors.

```
/* These uart interrupts halt any ongoing transfer if an error occurs, disable them */
/* Disable the UART Parity Error Interrupt */
 __HAL_UART_DISABLE_IT(&huart1, UART_IT_PE);
/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
__HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);
```

« Last Edit: November 17, 2016, 04:41:02 pm by voltsandjolts »

```
_______________________________________________________________________
Excellent solution! After a small modification it works as intended! Good work.
Only change required because STM has changed the register's name from
huart_cobs->hdmarx->Instance->CNDTR to
huart_cobs->hdmarx->Instance->NDTR.
_______________________________________________________________________
```