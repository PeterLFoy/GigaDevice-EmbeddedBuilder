/*!
    \file    gd32f3x0_hal_smartcard.h
    \brief   definitions for the SMARTCARD

    \version 2023-08-01, V1.0.0, HAL firmware for GD32F3x0
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

#ifndef GD32F3X0_HAL_SMARTCARD_H
#define GD32F3X0_HAL_SMARTCARD_H

#include "gd32f3x0_hal.h"

/* smartcard structure type enum */
typedef enum {
    HAL_SMARTCARD_INIT_STRUCT,                           /*!< smartcard initialization structure */
    HAL_SMARTCARD_DEV_STRUCT,                            /*!< smartcard device information structure */
    HAL_SMARTCARD_USER_CALLBCAK_STRUCT,                  /*!< smartcard user callback structure */
    HAL_SMARTCARD_IRQ_INIT_STRUCT,                       /*!< smartcard interrupt callback initialization structure */
} hal_smartcard_struct_type_enum;

/* hal_smartcard_init */
/* @PARA: periph */
/* @REFER: USART0 */

/* @PARA: p_init */
/* @STRUCT: smartcard initialization structure */
typedef struct {
    uint32_t        baudrate;             /*!< smartcard communication baud rate */
    uint32_t        parity;               /*!< parity mode */
    uint32_t        word_length;          /*!< number of data bits in a frame */
    uint32_t        stop_bit;             /*!< number of stop bits */
    uint32_t        direction;            /*!< communication transfer direction */
    
    uint32_t        clock_polarity;       /*!< the state of serial colck */
    uint32_t        clock_phase;          /*!< the clock transition on which the bit capture is made */
    uint32_t        clock_length_lastbit; /*!< whether the clock pulse corresponding to the last transmitted data bit */
    
    uint32_t        prescaler;            /*!< smartcard prescaler */
    uint32_t        guard_time;           /*!< smartcard guard time */
    uint32_t        nack_state;           /*!< NACK transmission is enabled or disabled */
    ControlStatus   early_nack;           /*!< NACK occurs 1/16 bit time earlier when parity error is detected. */
    ControlStatus   rx_fifo_en;           /*!< receive FIFO enable */
    ControlStatus   timeout_enable;       /*!< the receiver timeout is enabled */
    uint32_t        timeout_value;        /*!< the receiver timeout value */
    uint32_t        sample_method;        /*!< select sample method */
    uint32_t        block_length;         /*!< the smartcard block length in T=1 reception mode */
    uint32_t        auto_retry_count;     /*!< the smartcard auto-retry count */
    
    ControlStatus   first_bit_msb;        /*!< MSB is sent first on communication line */
    ControlStatus   tx_rx_swap;           /*!< Tx and Rx pins are swapped */
    ControlStatus   rx_level_invert;      /*!< the Rx pin active level is inverted */
    ControlStatus   tx_level_invert;      /*!< the Tx pin active level is inverted */
    ControlStatus   data_bit_invert;      /*!< data is inverted */
    ControlStatus   overrun_disable;      /*!< the reception overrun detection is disabled */
    ControlStatus   rx_error_dma_stop;    /*!< the DMA is disabled in case of reception error */
} hal_smartcard_init_struct;

/* @STRUCT_MEMBER: baudrate */
/* @=NULL */

/* @STRUCT_MEMBER: parity */
/* @DEFINE: SMARTCARD_PARITY_EVEN */
#define SMARTCARD_PARITY_EVEN                  USART_PM_EVEN    /*!< even parity */
#define SMARTCARD_PARITY_ODD                   USART_PM_ODD     /*!< odd parity */

/* @STRUCT_MEMBER: word_length */
/* @DEFINE: SMARTCARD_WORD_LENGTH_9BIT */
#define SMARTCARD_WORD_LENGTH_9BIT             USART_WL_9BIT    /*!< 9 bits word length */

/* @STRUCT_MEMBER: stop_bit */
/* @DEFINE: SMARTCARD_STOP_BIT_0_5 */
#define SMARTCARD_STOP_BIT_0_5                 USART_STB_0_5BIT /*!< 0.5 bit */
#define SMARTCARD_STOP_BIT_1_5                 USART_STB_1_5BIT /*!< 1.5 bits */

/* @STRUCT_MEMBER: direction */
/* @DEFINE: SMARTCARD_DIRECTION_RX_TX */
#define SMARTCARD_DIRECTION_RX_TX              (USART_TRANSMIT_ENABLE | USART_RECEIVE_ENABLE)     /*!< RX and TX mode */
#define SMARTCARD_DIRECTION_RX_ONLY            (USART_TRANSMIT_DISABLE | USART_RECEIVE_ENABLE)    /*!< RX mode */
#define SMARTCARD_DIRECTION_TX_ONLY            (USART_TRANSMIT_ENABLE | USART_RECEIVE_DISABLE)    /*!< TX mode */

/* @STRUCT_MEMBER: clock_polarity */
/* @DEFINE: SMARTCARD_CLOCK_POLARITY_LOW */
#define SMARTCARD_CLOCK_POLARITY_LOW           USART_CPL_LOW    /*!< low polarity */
#define SMARTCARD_CLOCK_POLARITY_HIGH          USART_CPL_HIGH   /*!< high polarity */

/* @STRUCT_MEMBER: clock_phase */
/* @DEFINE: SMARTCARD_CLOCK_PHASE_1CK */
#define SMARTCARD_CLOCK_PHASE_1CK              USART_CPH_1CK    /*!< frame phase on first clock transition */
#define SMARTCARD_CLOCK_PHASE_2CK              USART_CPH_2CK    /*!< frame phase on second clock transition */

/* @STRUCT_MEMBER: clock_length_lastbit */
/* @DEFINE: SMARTCARD_LAST_BIT_NOT_OUTPUT */
#define SMARTCARD_LAST_BIT_NOT_OUTPUT          USART_CLEN_NONE  /*!< frame last data bit clock pulse not output to SCLK pin */
#define SMARTCARD_LAST_BIT_OUTPUT              USART_CLEN_EN    /*!< frame last data bit clock pulse output to SCLK pin */

/* @STRUCT_MEMBER: prescaler */
/* @=NULL */

/* @STRUCT_MEMBER: guard_time */
/* @=NULL */

/* @STRUCT_MEMBER: nack_state */
/* @DEFINE: SMARTCARD_NACK_ENABLE */
#define SMARTCARD_NACK_ENABLE                  USART_CTL2_NKEN  /*!< NACK transmission enabled */
#define SMARTCARD_NACK_DISABLE                 0x0U             /*!< NACK transmission disabled  */

/* @STRUCT_MEMBER: early_nack */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: rx_fifo_en */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: timeout_enable */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: timeout_value */
/* @=NULL */

/* @STRUCT_MEMBER: sample_method */
/* @DEFINE: SMARTCARD_THREE_SAMPLE_BIT */
#define SMARTCARD_THREE_SAMPLE_BIT             USART_OSB_3BIT   /*!< frame three-bit sample method */
#define SMARTCARD_ONE_SAMPLE_BIT               USART_OSB_1BIT   /*!< frame one-bit sample method */

/* @STRUCT_MEMBER: block_length */
/* @=NULL */

/* @STRUCT_MEMBER: auto_retry_count */
/* @=NULL */

/* @STRUCT_MEMBER: first_bit_msb */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: tx_rx_swap */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: rx_level_invert */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: tx_level_invert */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: data_bit_invert */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: overrun_disable */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: rx_error_dma_stop */
/* @REFER: ControlStatus */

/* smartcard transfer buffer structure */
typedef struct {
    __IO uint8_t *buffer;                                /*!< pointer to smartcard transfer buffer */
    __IO uint32_t length;                                /*!< smartcard transfer length */
    __IO uint32_t pos;                                   /*!< smartcard transfer position */
} smartcard_buffer_struct;

/* smartcard device interrupt callback function pointer structure */
typedef struct {
    hal_irq_handle_cb receive_complete_handle;           /*!< smartcard receive complete callback function */
    hal_irq_handle_cb transmit_ready_handle;             /*!< smartcard transmit ready callback function */
    hal_irq_handle_cb transmit_complete_handle;          /*!< smartcard transmit complete callback function */
    hal_irq_handle_cb error_handle;                      /*!< smartcard transfer error callback function */
} hal_smartcard_irq_struct;

/* smartcard run state enum */
typedef enum {
    SMARTCARD_STATE_FREE,                                /*!< smartcard ready for use */
    SMARTCARD_STATE_BUSY,                                /*!< smartcard in busy state */
} hal_smartcard_run_state_enum;

/* smartcard device information structure */
typedef struct {
    uint32_t                             periph;         /*!< usart port */
    hal_smartcard_irq_struct             smartcard_irq;  /*!< smartcard device interrupt callback function pointer */
    hal_dma_dev_struct                   *p_dma_rx;      /*!< DMA receive pointer */
    hal_dma_dev_struct                   *p_dma_tx;      /*!< DMA transmit pointer */
    smartcard_buffer_struct              txbuffer;       /*!< transmit buffer */
    smartcard_buffer_struct              rxbuffer;       /*!< receive buffer */
    __IO uint16_t                        last_error;     /*!< the last error code */
    __IO uint16_t                        error_state;    /*!< smartcard error state */
    __IO hal_smartcard_run_state_enum    tx_state;       /*!< transmit state */
    __IO hal_smartcard_run_state_enum    rx_state;       /*!< receive state */
    void                                 *rx_callback;   /*!< receive callback function pointer */
    void                                 *tx_callback;   /*!< transmit callback function pointer */
    hal_mutex_enum                       mutex;
    void                                 *priv;          /*!< private pointer */
} hal_smartcard_dev_struct;

/* smartcard device user callback function pointer */
typedef void (*hal_smartcard_user_cb)(hal_smartcard_dev_struct *smartcard);

/* smartcard callback structure */
typedef struct{
    hal_smartcard_user_cb complete_func;                 /*!< user-defined transfer complete callback function */
    hal_smartcard_user_cb error_func;                    /*!< user-defined transfer error callback function */
} hal_smartcard_user_callback_struct;

/* smartcard error code */
#define HAL_SMARTCARD_ERROR_NONE               0U       /*!< no error */
#define HAL_SMARTCARD_ERROR_PERR               BIT(0)   /*!< parity error */
#define HAL_SMARTCARD_ERROR_NERR               BIT(1)   /*!< noise error */
#define HAL_SMARTCARD_ERROR_FERR               BIT(2)   /*!< frame error */
#define HAL_SMARTCARD_ERROR_ORERR              BIT(3)   /*!< overrun error */
#define HAL_SMARTCARD_ERROR_DMATX              BIT(4)   /*!< overrun error */
#define HAL_SMARTCARD_ERROR_DMARX              BIT(5)   /*!< overrun error */
#define HAL_SMARTCARD_ERROR_RTF                BIT(11)  /*!< Receiver TimeOut error */

/* function declarations */
/* initialize the smartcard structure with the default values */
void hal_smartcard_struct_init(hal_smartcard_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize smartcard */
void hal_smartcard_deinit(hal_smartcard_dev_struct *smartcard);
/* @FUNCTION: initialize the smartcard with specified values */
/* initialize smartcard */
int32_t hal_smartcard_init(hal_smartcard_dev_struct *smartcard, uint32_t periph, hal_smartcard_init_struct *p_init);
/* @END */

/* smartcard interrput handle functions */
/* smartcard interrupt handler content function,which is merely used in smartcard_handler */
void hal_smartcard_irq(hal_smartcard_dev_struct *smartcard);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_smartcard_irq_handle_set(hal_smartcard_dev_struct *smartcard, hal_smartcard_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_smartcard_irq_handle_all_reset(hal_smartcard_dev_struct *smartcard);

/* transmit or receive functions */
/* transmit amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_smartcard_transmit_poll(hal_smartcard_dev_struct *smartcard, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms);
/* receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_smartcard_receive_poll(hal_smartcard_dev_struct *smartcard, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms);
/* transmit amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_smartcard_transmit_interrupt(hal_smartcard_dev_struct *smartcard, uint8_t *p_buffer, uint32_t length, hal_smartcard_user_cb p_user_func);
/* receive amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_smartcard_receive_interrupt(hal_smartcard_dev_struct *smartcard, uint8_t *p_buffer, uint32_t length, hal_smartcard_user_cb p_user_func);
/* transmit amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_smartcard_transmit_dma(hal_smartcard_dev_struct *smartcard, uint8_t *p_buffer, uint16_t length, hal_smartcard_user_callback_struct *p_user_func);
/* receive amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_smartcard_receive_dma(hal_smartcard_dev_struct *smartcard, uint8_t *p_buffer, uint16_t length, hal_smartcard_user_callback_struct *p_user_func);

/* transfer control functions */
/* stop smartcard transmit transfer */
/* the function is blocking */
int32_t hal_smartcard_transmit_stop(hal_smartcard_dev_struct *smartcard);
/* stop smartcard receive transfer */
/* the function is blocking */
int32_t hal_smartcard_receive_stop(hal_smartcard_dev_struct *smartcard);


/* function declarations */
/* initialization functions */
/* reset smartcard */
void hals_smartcard_deinit(uint32_t smartcard_periph);
/* configure smartcard baud rate value */
void hals_smartcard_baudrate_set(uint32_t smartcard_periph, uint32_t baudval);
/* configure smartcard parity */
void hals_smartcard_parity_config(uint32_t smartcard_periph, uint32_t paritycfg);
/* configure smartcard word length */
void hals_smartcard_word_length_set(uint32_t smartcard_periph, uint32_t wlen);
/* configure smartcard stop bit length */
void hals_smartcard_stop_bit_set(uint32_t smartcard_periph, uint32_t stblen);
/* enable smartcard */
void hals_smartcard_enable(uint32_t smartcard_periph);
/* disable smartcard */
void hals_smartcard_disable(uint32_t smartcard_periph);
/* configure smartcard transmitter */
void hals_smartcard_transmit_config(uint32_t smartcard_periph, uint32_t txconfig);
/* configure smartcard receiver */
void hals_smartcard_receive_config(uint32_t smartcard_periph, uint32_t rxconfig);

/* smartcard normal mode communication */
/* data is transmitted/received with the LSB/MSB first */
void hals_smartcard_data_first_config(uint32_t smartcard_periph, uint32_t msbf);
/* configure smartcard inversion */
void hals_smartcard_invert_config(uint32_t smartcard_periph, usart_invert_enum invertpara);
/* enable the smartcard overrun function */
void hals_smartcard_overrun_enable(uint32_t smartcard_periph);
/* disable the smartcard overrun function */
void hals_smartcard_overrun_disable(uint32_t smartcard_periph);
/* configure the smartcard oversample mode */
void hals_smartcard_oversample_config(uint32_t smartcard_periph, uint32_t oversamp);
/* configure the sample bit method */
void hals_smartcard_sample_bit_config(uint32_t smartcard_periph, uint32_t osb);
/* enable receiver timeout */
void hals_smartcard_receiver_timeout_enable(uint32_t smartcard_periph);
/* disable receiver timeout */
void hals_smartcard_receiver_timeout_disable(uint32_t smartcard_periph);
/* configure receiver timeout threshold */
void hals_smartcard_receiver_timeout_threshold_config(uint32_t smartcard_periph, uint32_t rtimeout);
/* smartcard transmit data function */
void hals_smartcard_data_transmit(uint32_t smartcard_periph, uint16_t data);
/* smartcard receive data function */
uint16_t hals_smartcard_data_receive(uint32_t smartcard_periph);
/* enable smartcard command */
void hals_smartcard_command_enable(uint32_t smartcard_periph, uint32_t cmdtype);

/* synchronous communication */
/* enable smartcard clock */
void hals_smartcard_clock_enable(uint32_t smartcard_periph);
/* disable smartcard clock */
void hals_smartcard_clock_disable(uint32_t smartcard_periph);
/* configure smartcard synchronous mode parameters */
void hals_smartcard_synchronous_clock_config(uint32_t smartcard_periph, uint32_t clen, uint32_t cph, uint32_t cpl);

/* smartcard communication */
/* configure guard time value in smartcard mode */
void hals_smartcard_guard_time_config(uint32_t smartcard_periph, uint32_t guat);
/* enable smartcard mode */
void hals_smartcard_mode_enable(uint32_t smartcard_periph);
/* disable smartcard mode */
void hals_smartcard_mode_disable(uint32_t smartcard_periph);
/* enable NACK in smartcard mode */
void hals_smartcard_mode_nack_enable(uint32_t smartcard_periph);
/* disable NACK in smartcard mode */
void hals_smartcard_mode_nack_disable(uint32_t smartcard_periph);
/* enable early NACK in smartcard mode */
void hals_smartcard_mode_early_nack_enable(uint32_t smartcard_periph);
/* disable early NACK in smartcard mode */
void hals_smartcard_mode_early_nack_disable(uint32_t smartcard_periph);
/* configure smartcard auto-retry number */
void hals_smartcard_autoretry_config(uint32_t smartcard_periph, uint32_t scrtnum);
/* configure block length */
void hals_smartcard_block_length_config(uint32_t smartcard_periph, uint32_t bl);

/* smartcard DMA */
/* configure smartcard DMA reception */
void hals_smartcard_dma_receive_config(uint32_t smartcard_periph, uint8_t dmacmd);
/* configure smartcard DMA transmission */
void hals_smartcard_dma_transmit_config(uint32_t smartcard_periph, uint8_t dmacmd);
/* enable DMA on reception error */
void hals_smartcard_reception_error_dma_enable(uint32_t smartcard_periph);
/* disable DMA on reception error */
void hals_smartcard_reception_error_dma_disable(uint32_t smartcard_periph);

/* smartcard receive FIFO */
/* enable receive FIFO */
void hals_smartcard_receive_fifo_enable(uint32_t smartcard_periph);
/* disable receive FIFO */
void hals_smartcard_receive_fifo_disable(uint32_t smartcard_periph);
/* read receive FIFO counter number */
uint8_t hals_smartcard_receive_fifo_counter_number(uint32_t smartcard_periph);

/* flag & interrupt functions */
/* get smartcard status */
FlagStatus hals_smartcard_flag_get(uint32_t smartcard_periph, usart_flag_enum flag);
/* clear smartcard status */
void hals_smartcard_flag_clear(uint32_t smartcard_periph, usart_flag_enum flag);
/* enable smartcard interrupt */
void hals_smartcard_interrupt_enable(uint32_t smartcard_periph, usart_interrupt_enum interrupt);
/* disable smartcard interrupt */
void hals_smartcard_interrupt_disable(uint32_t smartcard_periph, usart_interrupt_enum interrupt);
/* get smartcard interrupt flag status */
FlagStatus hals_smartcard_interrupt_flag_get(uint32_t smartcard_periph, usart_interrupt_flag_enum int_flag);
/* clear smartcard interrupt flag */
void hals_smartcard_interrupt_flag_clear(uint32_t smartcard_periph, usart_interrupt_flag_enum int_flag);

#endif /* GD32F3X0_HAL_SMARTCARD_H */
