/*!
    \file    gd32f3x0_hal_usrt.h
    \brief   definitions for the USRT

    \version 2023-06-01, V1.0.0, firmware for GD32F3x0
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#ifndef GD32F3X0_HAL_USRT_H
#define GD32F3X0_HAL_USRT_H

#include "gd32f3x0_hal.h"

/* usrt struct initialization type enum */
typedef enum {
    HAL_USRT_INIT_STRUCT,           /*!< initialization structure */
    HAL_USRT_DEV_STRUCT,            /*!< device information structure */
    HAL_USRT_USER_CALLBCAK_STRUCT   /*!< user callback struct */
} hal_usrt_struct_type_enum;

/* hal_usrt_init */
/* @PARA: periph */
/* @REFER: USART0 */

/* @PARA: p_init */
/* @STRUCT: hal_usrt_init_struct */
typedef struct {
    uint32_t        baudrate;               /*!< communication baudrate */
    uint32_t        parity;                 /*!< parity mode */
    uint32_t        word_length;            /*!< number of data bits in a frame */
    uint32_t        stop_bit;               /*!< number of stop bits */
    uint32_t        direction;              /*!< communication transfer direction */
    ControlStatus   rx_fifo_en;             /*!< receive FIFO enable */
    uint32_t        clock_polarity;         /*!< clock polarity */
    uint32_t        clock_phase;            /*!< clock phase */
    uint32_t        clock_length_lastbit;   /*!< clock length */
} hal_usrt_init_struct;

/* @STRUCT_MEMBER: baudrate */
/* @=NULL */

/* @STRUCT_MEMBER: parity */
/* @DEFINE: USRT_PARITY_NONE */
#define USRT_PARITY_NONE             USART_PM_NONE    /*!< no parity */
#define USRT_PARITY_EVEN             USART_PM_EVEN    /*!< even parity */
#define USRT_PARITY_ODD              USART_PM_ODD     /*!< odd parity */

/* @STRUCT_MEMBER: word_length */
/* @DEFINE: USRT_WORD_LENGTH_8BIT */
#define USRT_WORD_LENGTH_8BIT        USART_WL_8BIT    /*!< 8 bits word length */
#define USRT_WORD_LENGTH_9BIT        USART_WL_9BIT    /*!< 9 bits word length */

/* @STRUCT_MEMBER: stop_bit */
/* @DEFINE: USRT_STOP_BIT_1 */
#define USRT_STOP_BIT_1              USART_STB_1BIT   /*!< 1 bit stop bit */
#define USRT_STOP_BIT_0_5            USART_STB_0_5BIT /*!< 0.5 bits stop bit */
#define USRT_STOP_BIT_2              USART_STB_2BIT   /*!< 2 bits stop bit */
#define USRT_STOP_BIT_1_5            USART_STB_1_5BIT /*!< 1.5 bits stop bit */

/* @STRUCT_MEMBER: direction */
/* @DEFINE: USRT_DIRECTION_RX_TX */
#define USRT_DIRECTION_RX_TX         (USART_TRANSMIT_ENABLE | USART_RECEIVE_ENABLE)   /*!< RX and TX mode */
#define USRT_DIRECTION_RX_ONLY       (USART_TRANSMIT_DISABLE | USART_RECEIVE_ENABLE)  /*!< RX only mode */
#define USRT_DIRECTION_TX_ONLY       (USART_TRANSMIT_ENABLE | USART_RECEIVE_DISABLE)  /*!< TX only mode */

/* @STRUCT_MEMBER: rx_fifo_en */
/* @ENUM: ControlStatus */;

/* @STRUCT_MEMBER: clock_polarity */
/* @DEFINE: USRT_CLOCK_POLARITY_LOW */
#define USRT_CLOCK_POLARITY_LOW      USART_CPL_LOW    /*!< steady low value on CK pin */
#define USRT_CLOCK_POLARITY_HIGH     USART_CPL_HIGH   /*!< steady high value on CK pin */

/* @STRUCT_MEMBER: clock_phase */
/* @DEFINE: USRT_CLOCK_PHASE_1CK */
#define USRT_CLOCK_PHASE_1CK         USART_CPH_1CK    /*!< the first clock transition is the first data capture edge */
#define USRT_CLOCK_PHASE_2CK         USART_CPH_2CK    /*!< the second clock transition is the first data capture edge */

/* @STRUCT_MEMBER: clock_length_lastbit */
/* @DEFINE: USRT_LAST_BIT_NOT_OUTPUT */
#define USRT_LAST_BIT_NOT_OUTPUT     USART_CLEN_NONE  /*!< the clock pulse of the last data bit (MSB) is not output to the CK pin */
#define USRT_LAST_BIT_OUTPUT         USART_CLEN_EN    /*!< the clock pulse of the last data bit (MSB) is output to the CK pin */

/* usrt device interrupt callback function pointer structure */
typedef struct {
    __IO hal_irq_handle_cb receive_complete_handle;   /*!< receive complete callback function */  
    __IO hal_irq_handle_cb transmit_ready_handle;     /*!< transmit ready callback function */
    __IO hal_irq_handle_cb transmit_complete_handle;  /*!< transmit complete callback function */
    __IO hal_irq_handle_cb error_handle;              /*!< error callback function */
} hal_usrt_irq_struct;

/* usrt transfer buffer structure */
typedef struct {
    __IO uint8_t *buffer;           /*!< pointer to transfer buffer */
    __IO uint32_t length;           /*!< transfer length */
    __IO uint32_t pos;              /*!< transfer position */
} usrt_buffer_struct;

/* usrt run state enum */
typedef enum {
    USRT_STATE_FREE,                /*!< ready for use */
    USRT_STATE_BUSY,                /*!< busy state */
    USRT_STATE_BUSY_TX_RX,          /*!< busy for Tx Rx state */
} hal_usrt_run_state_enum;

/* usrt device information structure */
typedef struct {
    uint32_t                      periph;        /*!< usart port */
    hal_usrt_irq_struct           usrt_irq;      /*!< device interrupt callback function pointer */
    hal_dma_dev_struct            *p_dma_rx;     /*!< DMA receive pointer */
    hal_dma_dev_struct            *p_dma_tx;     /*!< DMA transmit pointer */
    usrt_buffer_struct            txbuffer;      /*!< transmit buffer */
    usrt_buffer_struct            rxbuffer;      /*!< receive buffer */
    uint16_t                      data_bit_mask; /*!< mask bit of data */
    __IO uint16_t                 last_error;    /*!< the last error code */
    __IO uint16_t                 error_state;   /*!< error state */
    __IO hal_usrt_run_state_enum  tx_state;      /*!< transmit state */
    __IO hal_usrt_run_state_enum  rx_state;      /*!< receive state */
    void                          *rx_callback;  /*!< receive callback function pointer */
    void                          *tx_callback;  /*!< transmit callback function pointer */
    hal_mutex_enum                mutex;
    void                          *priv;         /*!< private pointer */
} hal_usrt_dev_struct;

typedef void (*hal_usrt_user_cb)(hal_usrt_dev_struct *usrt);

/* usrt user callback struct */
typedef struct{
    hal_usrt_user_cb complete_func;      /*!< transfer complete callback function */
    hal_usrt_user_cb error_func;         /*!< error callback function */
}hal_usrt_user_callback_struct;

/* usrt error code */
#define HAL_USRT_ERROR_NONE         0U               /*!< no error */
#define HAL_USRT_ERROR_PERR         BIT(0)           /*!< parity error */
#define HAL_USRT_ERROR_NERR         BIT(1)           /*!< noise error */
#define HAL_USRT_ERROR_FERR         BIT(2)           /*!< frame error */
#define HAL_USRT_ERROR_ORERR        BIT(3)           /*!< overrun error */
#define HAL_USRT_ERROR_DMATX        BIT(4)           /*!< DMA Tx error */
#define HAL_USRT_ERROR_DMARX        BIT(5)           /*!< DMA Rx error */

/* function declarations */
/* initialization functions */
/* initialize the usrt structure with the default values */
void hal_usrt_struct_init(hal_usrt_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize the usrt */
void hal_usrt_deinit(hal_usrt_dev_struct *usrt);
/* @FUNCTION: initialize the usrt with specified values */
/* initialize usrt */
int32_t hal_usrt_init(hal_usrt_dev_struct *usrt, uint32_t periph, hal_usrt_init_struct *p_init);
/* @END */

/* usrt interrput handle functions */
/* usrt interrupt handler content function,which is merely used in usrt_handler */
void hal_usrt_irq(hal_usrt_dev_struct *usrt);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_usrt_irq_handle_set(hal_usrt_dev_struct *usrt, hal_usrt_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_usrt_irq_handle_all_reset(hal_usrt_dev_struct *usrt);

/* transmit or receive functions */
/* transmit amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_usrt_transmit_poll(hal_usrt_dev_struct *usrt, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms);
/* receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_usrt_receive_poll(hal_usrt_dev_struct *usrt, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms);
/* transmit & receive amounts of data, poll transfer process and completed status */
/* the function is blocking */
int32_t hal_usrt_transmit_receive_poll(hal_usrt_dev_struct *usrt, uint8_t *p_tx_buffer, uint8_t *p_rx_buffer, uint32_t length, uint32_t timeout_ms);
/* transmit amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_usrt_transmit_interrupt(hal_usrt_dev_struct *usrt, uint8_t *p_buffer, uint32_t length, hal_usrt_user_cb p_user_func);
/* receive amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_usrt_receive_interrupt(hal_usrt_dev_struct *usrt, uint8_t *p_buffer, uint32_t length, hal_usrt_user_cb p_user_func);
/* transmit & receive amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_usrt_transmit_receive_interrupt(hal_usrt_dev_struct *usrt, uint8_t *p_tx_buffer, uint8_t *p_rx_buffer, uint32_t length, hal_usrt_user_cb p_user_func);
/* transmit amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_usrt_transmit_dma(hal_usrt_dev_struct *usrt, uint8_t *p_buffer, uint16_t length, hal_usrt_user_callback_struct *p_func);
/* transmit amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_usrt_receive_dma(hal_usrt_dev_struct *usrt, uint8_t *p_buffer, uint16_t length, hal_usrt_user_callback_struct *p_func);
/* transmit & receive amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_usrt_transmit_receive_dma(hal_usrt_dev_struct *usrt, uint8_t *p_tx_buffer, uint8_t *p_rx_buffer, uint16_t length, hal_usrt_user_callback_struct *p_func);

/* transfer control functions */
/* pause usrt DMA transfer during transmission process */
int32_t hal_usrt_dma_pause(hal_usrt_dev_struct *usrt);
/* resume usrt DMA transfer during transmission process */
int32_t hal_usrt_dma_resume(hal_usrt_dev_struct *usrt);
/* stop usrt transmit and receive transfer */
/* the function is blocking */
int32_t hal_usrt_transfer_stop(hal_usrt_dev_struct *usrt);


/* function declarations */
/* initialization functions */
/* reset usrt */
void hals_usrt_deinit(uint32_t usrt_periph);
/* configure usrt baud rate value */
void hals_usrt_baudrate_set(uint32_t usrt_periph, uint32_t baudval);
/* configure usrt parity */
void hals_usrt_parity_config(uint32_t usrt_periph, uint32_t paritycfg);
/* configure usrt word length */
void hals_usrt_word_length_set(uint32_t usrt_periph, uint32_t wlen);
/* configure usrt stop bit length */
void hals_usrt_stop_bit_set(uint32_t usrt_periph, uint32_t stblen);
/* enable usrt */
void hals_usrt_enable(uint32_t usrt_periph);
/* disable usrt */
void hals_usrt_disable(uint32_t usrt_periph);
/* configure usrt transmitter */
void hals_usrt_transmit_config(uint32_t usrt_periph, uint32_t txconfig);
/* configure usrt receiver */
void hals_usrt_receive_config(uint32_t usrt_periph, uint32_t rxconfig);

/* usrt normal mode communication */
/* data is transmitted/received with the LSB/MSB first */
void hals_usrt_data_first_config(uint32_t usrt_periph, uint32_t msbf);
/* configure usrt inversion */
void hals_usrt_invert_config(uint32_t usrt_periph, usart_invert_enum invertpara);
/* enable the usrt overrun function */
void hals_usrt_overrun_enable(uint32_t usrt_periph);
/* disable the usrt overrun function */
void hals_usrt_overrun_disable(uint32_t usrt_periph);
/* configure the usrt oversample mode */
void hals_usrt_oversample_config(uint32_t usrt_periph, uint32_t oversamp);
/* configure the sample bit method */
void hals_usrt_sample_bit_config(uint32_t usrt_periph, uint32_t osb);
/* usrt transmit data function */
void hals_usrt_data_transmit(uint32_t usrt_periph, uint16_t data);
/* usrt receive data function */
uint16_t hals_usrt_data_receive(uint32_t usrt_periph);
/* enable usrt command */
void hals_usrt_command_enable(uint32_t usrt_periph, uint32_t cmdtype);

/* synchronous communication */
/* enable usrt clock */
void hals_usrt_clock_enable(uint32_t usrt_periph);
/* disable usrt clock */
void hals_usrt_clock_disable(uint32_t usrt_periph);
/* configure usrt synchronous mode parameters */
void hals_usrt_synchronous_clock_config(uint32_t usrt_periph, uint32_t clen, uint32_t cph, uint32_t cpl);

/* usrt DMA */
/* configure usrt DMA reception */
void hals_usrt_dma_receive_config(uint32_t usrt_periph, uint8_t dmacmd);
/* configure usrt DMA transmission */
void hals_usrt_dma_transmit_config(uint32_t usrt_periph, uint8_t dmacmd);
/* enable DMA on reception error */
void hals_usrt_reception_error_dma_enable(uint32_t usrt_periph);
/* disable DMA on reception error */
void hals_usrt_reception_error_dma_disable(uint32_t usrt_periph);

/* usrt receive FIFO */
/* enable receive FIFO */
void hals_usrt_receive_fifo_enable(uint32_t usrt_periph);
/* disable receive FIFO */
void hals_usrt_receive_fifo_disable(uint32_t usrt_periph);
/* read receive FIFO counter number */
uint8_t hals_usrt_receive_fifo_counter_number(uint32_t usrt_periph);

/* flag & interrupt functions */
/* get usrt status */
FlagStatus hals_usrt_flag_get(uint32_t usrt_periph, usart_flag_enum flag);
/* clear usrt status */
void hals_usrt_flag_clear(uint32_t usrt_periph, usart_flag_enum flag);
/* enable usrt interrupt */
void hals_usrt_interrupt_enable(uint32_t usrt_periph, usart_interrupt_enum interrupt);
/* disable usrt interrupt */
void hals_usrt_interrupt_disable(uint32_t usrt_periph, usart_interrupt_enum interrupt);
/* get usrt interrupt flag status */
FlagStatus hals_usrt_interrupt_flag_get(uint32_t usrt_periph, usart_interrupt_flag_enum int_flag);
/* clear usrt interrupt flag */
void hals_usrt_interrupt_flag_clear(uint32_t usrt_periph, usart_interrupt_flag_enum int_flag);

#endif /* GD32F3X0_HAL_USRT_H */
