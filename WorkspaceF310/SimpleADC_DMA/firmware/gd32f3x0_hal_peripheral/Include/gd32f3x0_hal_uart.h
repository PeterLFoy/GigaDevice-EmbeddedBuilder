/*!
    \file    gd32f3x0_hal_uart.h
    \brief   definitions for the UART

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

#ifndef GD32F3X0_HAL_UART_H
#define GD32F3X0_HAL_UART_H

#include "gd32f3x0_hal.h"

/* uart struct initialization type enum */
typedef enum {
    HAL_UART_INIT_STRUCT,                /*!< initialization structure */
    HAL_UART_DEV_STRUCT,                 /*!< device information structure */
    HAL_UART_USER_CALLBCAK_STRUCT,       /*!< user callback struct */
    HAL_UART_IRQ_INIT_STRUCT             /*!< interrupt callback initialization structure */
} hal_uart_struct_type_enum;

/* @STRUCT_MEMBER: work_mode */
/* @ENUM: hal_uart_work_mode_enum */
typedef enum {
    UART_WORK_MODE_ASYN = 0,                            /*!< asynchronous communication mode */
    UART_WORK_MODE_SINGLE_WIRE,                         /*!< single wire(half-duplex) communication mode */
    UART_WORK_MODE_MULTIPROCESSCOR,                     /*!< multiprocessor communication mode */
    UART_WORK_MODE_LIN                                  /*!< LIN mode */
} hal_uart_work_mode_enum;

/* hal_uart_init */
/* @PARA: periph */
/* @REFER: USART0 */

/* @PARA: p_init */
/* @STRUCT: hal_uart_init_struct */
typedef struct {
    hal_uart_work_mode_enum     work_mode;              /*!< work mode */
    uint32_t                    baudrate;               /*!< communication baudrate */
    uint32_t                    parity;                 /*!< parity mode */
    uint32_t                    word_length;            /*!< number of data bits in a frame */
    uint32_t                    stop_bit;               /*!< number of stop bits */
    uint32_t                    direction;              /*!< communication transfer direction */
    uint32_t                    over_sample;            /*!< oversample mode */
    uint32_t                    sample_method;          /*!< one sample bit method */
    uint32_t                    hardware_flow;          /*!< hardware flow control */
    ControlStatus               rx_fifo_en;             /*!< receive FIFO enable */
    ControlStatus               timeout_enable;         /*!< the receiver timeout is enabled */
    uint32_t                    timeout_value;          /*!< the receiver timeout value */
    ControlStatus               first_bit_msb;          /*!< MSB is sent first on communication line */
    ControlStatus               tx_rx_swap;             /*!< Tx and Rx pins are swapped */
    ControlStatus               rx_level_invert;        /*!< the Rx pin active level is inverted */
    ControlStatus               tx_level_invert;        /*!< the Tx pin active level is inverted */
    ControlStatus               data_bit_invert;        /*!< data is inverted */
    ControlStatus               overrun_disable;        /*!< the reception overrun detection is disabled */
    ControlStatus               rx_error_dma_stop;      /*!< the DMA is disabled in case of reception error */
    /*!< LIN mode */
    uint32_t break_frame_length;   /*!< LIN break frame length */
    /*!< RS485 mode */
    ControlStatus   rs485_mode;           /*!< rs485 mode */
    uint32_t        de_polarity;          /*!< driver enable polarity */
    uint32_t        de_assertion_time;    /*!< driver enable assertion time */
    uint32_t        de_deassertion_time;  /*!< driver enable deassertion time */
    /*!< multi-processor mode */
    uint32_t    wakeup_mode;    /*!< wakeup mode */
    uint8_t     address;        /*!< wakeup address */
    uint32_t    addr_length;    /*!< address length */
} hal_uart_init_struct;

/* @STRUCT_MEMBER: work_mode */
/* @ENUM: hal_uart_work_mode_enum */
/* @REFER: hal_uart_work_mode_enum */

/* @STRUCT_MEMBER: baudrate */
/* @=NULL */

/* @STRUCT_MEMBER: parity */
/* @DEFINE: UART_PARITY_NONE */
#define UART_PARITY_NONE            USART_PM_NONE     /*!< no parity */
#define UART_PARITY_EVEN            USART_PM_EVEN     /*!< even parity */
#define UART_PARITY_ODD             USART_PM_ODD      /*!< odd parity */

/* @STRUCT_MEMBER: word_length */
/* @DEFINE: UART_WORD_LENGTH_8BIT */
#define UART_WORD_LENGTH_8BIT       USART_WL_8BIT     /*!< 8 bits word length */
#define UART_WORD_LENGTH_9BIT       USART_WL_9BIT     /*!< 9 bits word length */

/* @STRUCT_MEMBER: stop_bit */
/* @DEFINE: UART_STOP_BIT_1 */
#define UART_STOP_BIT_1             USART_STB_1BIT    /*!< 1 bit stop bit */
#define UART_STOP_BIT_0_5           USART_STB_0_5BIT  /*!< 0.5 bit stop bit */
#define UART_STOP_BIT_2             USART_STB_2BIT    /*!< 2 bits stop bit */
#define UART_STOP_BIT_1_5           USART_STB_1_5BIT  /*!< 1.5 bits stop bit */

/* @STRUCT_MEMBER: direction */
/* @DEFINE: UART_DIRECTION_RX_TX */
#define UART_DIRECTION_RX_TX        (USART_TRANSMIT_ENABLE | USART_RECEIVE_ENABLE)  /*!< RX and TX mode */
#define UART_DIRECTION_RX_ONLY      (USART_TRANSMIT_DISABLE | USART_RECEIVE_ENABLE) /*!< RX only mode */
#define UART_DIRECTION_TX_ONLY      (USART_TRANSMIT_ENABLE | USART_RECEIVE_DISABLE) /*!< TX only mode */

/* @STRUCT_MEMBER: over_sample */
/* @DEFINE: UART_OVER_SAMPLE_8 */
#define UART_OVER_SAMPLE_16             USART_OVSMOD_16   /*!< oversampling by 16 */
#define UART_OVER_SAMPLE_8              USART_OVSMOD_8    /*!< oversampling by 8 */

/* @STRUCT_MEMBER: sample_method */
/* @DEFINE: UART_THREE_SAMPLE_BIT */
#define UART_THREE_SAMPLE_BIT           USART_OSB_3BIT    /*!< three sample bit method */
#define UART_ONE_SAMPLE_BIT             USART_OSB_1BIT    /*!< one sample bit method */

/* @STRUCT_MEMBER: hardware_flow */
/* @DEFINE: UART_HARDWARE_FLOW_NONE */
#define UART_HARDWARE_FLOW_NONE         (USART_RTS_DISABLE | USART_CTS_DISABLE) /*!< hardware flow none */
#define UART_HARDWARE_FLOW_RTS          (USART_RTS_ENABLE | USART_CTS_DISABLE)  /*!< RTS only */
#define UART_HARDWARE_FLOW_CTS          (USART_RTS_DISABLE | USART_CTS_ENABLE)  /*!< CTS only */
#define UART_HARDWARE_FLOW_RTS_CTS      (USART_RTS_ENABLE | USART_CTS_ENABLE)  /*!< RTS and CTS */

/* @STRUCT_MEMBER: rx_fifo_en */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: timeout_enable */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: timeout_value */
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

/* @STRUCT_MEMBER: break_frame_length */
/* @DEFINE: UART_LIN_BREAK_DETECTION_10BIT */
#define UART_LIN_BREAK_DETECTION_10BIT         USART_LBLEN_10B    /*!< 10 bits break detection */
#define UART_LIN_BREAK_DETECTION_11BIT         USART_LBLEN_11B    /*!< 11 bits break detection */

/* @STRUCT_MEMBER: rs485_mode */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: de_polarity */
/* @DEFINE: UART_RS485_DE_POLARITY_HIGH */
#define UART_RS485_DE_POLARITY_HIGH     USART_DEP_HIGH     /*!< DE signal is active high */
#define UART_RS485_DE_POLARITY_LOW      USART_DEP_LOW      /*!< DE signal is active low */

/* @STRUCT_MEMBER: de_assertion_time */
/* @=NULL */

/* @STRUCT_MEMBER: de_deassertion_time */
/* @=NULL */

/* @STRUCT_MEMBER: wakeup_mode */
/* @DEFINE: UART_MULTIPROCESSOR_WAKEUP_IDLE */
#define UART_MULTIPROCESSOR_WAKEUP_IDLE        USART_WM_IDLE      /*!< wakeup from deep-sleep by idle line */
#define UART_MULTIPROCESSOR_WAKEUP_ADDRESS     USART_WM_ADDR      /*!< wakeup from deep-sleep by address mark */

/* @STRUCT_MEMBER: addr_length */
/* @DEFINE: UART_MULTIPROCESSOR_ADDRESS_4BIT */
#define UART_MULTIPROCESSOR_ADDRESS_4BIT       USART_ADDM_4BIT     /*!< 4-bit address detection */
#define UART_MULTIPROCESSOR_ADDRESS_FULLBIT    USART_ADDM_FULLBIT  /*!< full-bit address detection */

/* uart device interrupt callback function pointer structure */
typedef struct {
    __IO hal_irq_handle_cb receive_complete_handle;   /*!< receive complete callback function */
    __IO hal_irq_handle_cb receive_timeout_handle;    /*!< receive timeout callback function */
    __IO hal_irq_handle_cb transmit_ready_handle;     /*!< transmit ready callback function */
    __IO hal_irq_handle_cb transmit_complete_handle;  /*!< transmit complete callback function */
    __IO hal_irq_handle_cb error_handle;              /*!< error callback function */
    __IO hal_irq_handle_cb wakeup_handle;             /*!< wakeup callback function */
    __IO hal_irq_handle_cb idle_line_detected_handle; /*!< idle line detected callback function */
    __IO hal_irq_handle_cb address_match_handle;      /*!< address match callback function */
    __IO hal_irq_handle_cb lin_break_detected_handle; /*!< LIN break detected callback function */
    __IO hal_irq_handle_cb cts_change_handle;         /*!< CTS change callback function */
} hal_uart_irq_struct;

/* uart transfer buffer structure */
typedef struct {
    __IO uint8_t *buffer;           /*!< pointer to transfer buffer */
    __IO uint32_t length;           /*!< transfer length */
    __IO uint32_t pos;              /*!< transfer position */
} uart_buffer_struct;

/* uart run state enum */
typedef enum {
    UART_STATE_FREE = 0,        /*!< ready for use */
    UART_STATE_BUSY             /*!< busy state */
} hal_uart_run_state_enum;

/* uart device information structure */
typedef struct {
    uint32_t                      periph;        /*!< usart port */
    hal_uart_irq_struct           uart_irq;      /*!< device interrupt callback function pointer */
    hal_dma_dev_struct            *p_dma_rx;     /*!< DMA receive pointer */
    hal_dma_dev_struct            *p_dma_tx;     /*!< DMA transmit pointer */
    uart_buffer_struct            txbuffer;      /*!< transmit buffer */
    uart_buffer_struct            rxbuffer;      /*!< receive buffer */
    uint16_t                      data_bit_mask; /*!< mask bit of data */
    __IO uint16_t                 last_error;    /*!< the last error code */
    __IO uint16_t                 error_state;   /*!< error state */
    __IO hal_uart_run_state_enum  tx_state;      /*!< transmit state */
    __IO hal_uart_run_state_enum  rx_state;      /*!< receive state */
    void                          *rx_callback;  /*!< receive callback function pointer */
    void                          *tx_callback;  /*!< transmit callback function pointer */
    hal_mutex_enum                mutex;
    void                          *priv;         /*!< private pointer */
} hal_uart_dev_struct;

typedef void (*hal_uart_user_cb)(hal_uart_dev_struct *uart);

/* uart user callback struct */
typedef struct {
    hal_uart_user_cb complete_func;      /*!< transfer complete callback function */
    hal_uart_user_cb error_func;         /*!< error callback function */
} hal_uart_user_callback_struct;

/* uart error code */
#define HAL_UART_ERROR_NONE                    0U                /*!< no error */
#define HAL_UART_ERROR_PERR                    BIT(0)            /*!< parity error */
#define HAL_UART_ERROR_NERR                    BIT(1)            /*!< noise error */
#define HAL_UART_ERROR_FERR                    BIT(2)            /*!< frame error */
#define HAL_UART_ERROR_ORERR                   BIT(3)            /*!< overrun error */
#define HAL_UART_ERROR_DMATX                   BIT(4)            /*!< DMA Tx error */
#define HAL_UART_ERROR_DMARX                   BIT(5)            /*!< DMA Rx error */

/* function declarations */
/* initialization functions */
/* initialize the uart structure with the default values */
void hal_uart_struct_init(hal_uart_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize uart */
void hal_uart_deinit(hal_uart_dev_struct *uart);
/* @FUNCTION: initialize uart */
int32_t hal_uart_init(hal_uart_dev_struct *uart, uint32_t periph, hal_uart_init_struct *p_init);
/* @END */

/* uart interrput handle functions */
/* uart interrupt handler content function,which is merely used in uart_handler */
void hal_uart_irq(hal_uart_dev_struct *uart);
/* set user-defined interrupt callback function,
which will be registered and called when corresponding interrupt be triggered */
void hal_uart_irq_handle_set(hal_uart_dev_struct *uart, hal_uart_irq_struct *p_irq);
/* reset all user-defined interrupt callback function,
which will be registered and called when corresponding interrupt be triggered */
void hal_uart_irq_handle_all_reset(hal_uart_dev_struct *uart);

/* transmit or receive functions */
/* transmit amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_uart_transmit_poll(hal_uart_dev_struct *uart, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms);
/* receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_uart_receive_poll(hal_uart_dev_struct *uart, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms);
/* transmit amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_uart_transmit_interrupt(hal_uart_dev_struct *uart, uint8_t *p_buffer, uint32_t length, hal_uart_user_cb p_user_func);
/* receive amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_uart_receive_interrupt(hal_uart_dev_struct *uart, uint8_t *p_buffer, uint32_t length, hal_uart_user_cb p_user_func);
/* transmit amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_uart_transmit_dma(hal_uart_dev_struct *uart, uint8_t *p_buffer, uint16_t length, hal_uart_user_callback_struct *p_func);
/* receive amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_uart_receive_dma(hal_uart_dev_struct *uart, uint8_t *p_buffer, uint16_t length, hal_uart_user_callback_struct *p_func);

/* transfer control functions */
/* pause uart DMA transfer during transmission process */
int32_t hal_uart_dma_pause(hal_uart_dev_struct *uart);
/* resume USRT DMA transfer during transmission process */
int32_t hal_uart_dma_resume(hal_uart_dev_struct *uart);
/* stop USRT transmit and receive transfer */
int32_t hal_uart_transmit_stop(hal_uart_dev_struct *uart);
/* stop uart transmit transfer */
int32_t hal_uart_receive_stop(hal_uart_dev_struct *uart);


/* function declarations */
/* initialization functions */
/* reset uart */
void hals_uart_deinit(uint32_t uart_periph);
/* configure uart baud rate value */
void hals_uart_baudrate_set(uint32_t uart_periph, uint32_t baudval);
/* configure uart parity */
void hals_uart_parity_config(uint32_t uart_periph, uint32_t paritycfg);
/* configure uart word length */
void hals_uart_word_length_set(uint32_t uart_periph, uint32_t wlen);
/* configure uart stop bit length */
void hals_uart_stop_bit_set(uint32_t uart_periph, uint32_t stblen);
/* enable uart */
void hals_uart_enable(uint32_t uart_periph);
/* disable uart */
void hals_uart_disable(uint32_t uart_periph);
/* configure uart transmitter */
void hals_uart_transmit_config(uint32_t uart_periph, uint32_t txconfig);
/* configure uart receiver */
void hals_uart_receive_config(uint32_t uart_periph, uint32_t rxconfig);

/* uart normal mode communication */
/* data is transmitted/received with the LSB/MSB first */
void hals_uart_data_first_config(uint32_t uart_periph, uint32_t msbf);
/* configure uart inversion */
void hals_uart_invert_config(uint32_t uart_periph, usart_invert_enum invertpara);
/* enable the uart overrun function */
void hals_uart_overrun_enable(uint32_t uart_periph);
/* disable the uart overrun function */
void hals_uart_overrun_disable(uint32_t uart_periph);
/* configure the uart oversample mode */
void hals_uart_oversample_config(uint32_t uart_periph, uint32_t oversamp);
/* configure the sample bit method */
void hals_uart_sample_bit_config(uint32_t uart_periph, uint32_t osb);
/* enable receiver timeout */
void hals_uart_receiver_timeout_enable(uint32_t uart_periph);
/* disable receiver timeout */
void hals_uart_receiver_timeout_disable(uint32_t uart_periph);
/* configure receiver timeout threshold */
void hals_uart_receiver_timeout_threshold_config(uint32_t uart_periph, uint32_t rtimeout);
/* uart transmit data function */
void hals_uart_data_transmit(uint32_t uart_periph, uint16_t data);
/* uart receive data function */
uint16_t hals_uart_data_receive(uint32_t uart_periph);
/* enable uart command */
void hals_uart_command_enable(uint32_t uart_periph, uint32_t cmdtype);

/* multi-processor communication */
/* configure the address of the uart in wake up by address match mode */
void hals_uart_address_config(uint32_t uart_periph, uint8_t addr);
/* configure address detection mode */
void hals_uart_address_detection_mode_config(uint32_t uart_periph, uint32_t addmod);
/* enable mute mode */
void hals_uart_mute_mode_enable(uint32_t uart_periph);
/* disable mute mode */
void hals_uart_mute_mode_disable(uint32_t uart_periph);
/* configure wakeup method in mute mode */
void hals_uart_mute_mode_wakeup_config(uint32_t uart_periph, uint32_t wmethod);

/* LIN mode communication */
/* enable LIN mode */
void hals_uart_lin_mode_enable(uint32_t uart_periph);
/* disable LIN mode */
void hals_uart_lin_mode_disable(uint32_t uart_periph);
/* configure LIN break frame length */
void hals_uart_lin_break_detection_length_config(uint32_t uart_periph, uint32_t lblen);

/* half-duplex communication */
/* enable half-duplex mode */
void hals_uart_halfduplex_enable(uint32_t uart_periph);
/* disable half-duplex mode */
void hals_uart_halfduplex_disable(uint32_t uart_periph);

/* hardware flow communication */
/* configure hardware flow control RTS */
void hals_uart_hardware_flow_rts_config(uint32_t uart_periph, uint32_t rtsconfig);
/* configure hardware flow control CTS */
void hals_uart_hardware_flow_cts_config(uint32_t uart_periph, uint32_t ctsconfig);

/* enable RS485 driver */
void hals_uart_rs485_driver_enable(uint32_t uart_periph);
/* disable RS485 driver */
void hals_uart_rs485_driver_disable(uint32_t uart_periph);
/* configure driver enable assertion time */
void hals_uart_driver_assertime_config(uint32_t uart_periph, uint32_t deatime);
/* configure driver enable de-assertion time */
void hals_uart_driver_deassertime_config(uint32_t uart_periph, uint32_t dedtime);
/* configure driver enable polarity mode */
void hals_uart_depolarity_config(uint32_t uart_periph, uint32_t dep);

/* uart DMA */
/* configure uart DMA reception */
void hals_uart_dma_receive_config(uint32_t uart_periph, uint8_t dmacmd);
/* configure uart DMA transmission */
void hals_uart_dma_transmit_config(uint32_t uart_periph, uint8_t dmacmd);
/* enable DMA on reception error */
void hals_uart_reception_error_dma_enable(uint32_t uart_periph);
/* disable DMA on reception error */
void hals_uart_reception_error_dma_disable(uint32_t uart_periph);

/* uart wakeup function */
/* enable uart to wakeup the MCU from deep-sleep mode */
void hals_uart_wakeup_enable(uint32_t uart_periph);
/* disable uart to wakeup the MCU from deep-sleep mode */
void hals_uart_wakeup_disable(uint32_t uart_periph);
/* configure the uart wakeup mode from deep-sleep mode */
void hals_uart_wakeup_mode_config(uint32_t uart_periph, uint32_t wum);

/* uart receive FIFO */
/* enable receive FIFO */
void hals_uart_receive_fifo_enable(uint32_t uart_periph);
/* disable receive FIFO */
void hals_uart_receive_fifo_disable(uint32_t uart_periph);
/* read receive FIFO counter number */
uint8_t hals_uart_receive_fifo_counter_number(uint32_t uart_periph);

/* flag & interrupt functions */
/* get uart status */
FlagStatus hals_uart_flag_get(uint32_t uart_periph, usart_flag_enum flag);
/* clear uart status */
void hals_uart_flag_clear(uint32_t uart_periph, usart_flag_enum flag);
/* enable uart interrupt */
void hals_uart_interrupt_enable(uint32_t uart_periph, usart_interrupt_enum interrupt);
/* disable uart interrupt */
void hals_uart_interrupt_disable(uint32_t uart_periph, usart_interrupt_enum interrupt);
/* get uart interrupt flag status */
FlagStatus hals_uart_interrupt_flag_get(uint32_t uart_periph, usart_interrupt_flag_enum int_flag);
/* clear uart interrupt flag */
void hals_uart_interrupt_flag_clear(uint32_t uart_periph, usart_interrupt_flag_enum int_flag);

#endif /* GD32F3X0_HAL_UART_H */
