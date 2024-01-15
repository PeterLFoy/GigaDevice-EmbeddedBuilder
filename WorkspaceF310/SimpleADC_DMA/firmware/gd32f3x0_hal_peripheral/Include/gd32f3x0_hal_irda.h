/*!
    \file    gd32f3x0_hal_irda.h
    \brief   definitions for the IRDA

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

#ifndef GD32F3X0_HAL_IRDA_H
#define GD32F3X0_HAL_IRDA_H

#include "gd32f3x0_hal.h"

/* irda struct initialization type enum */
typedef enum {
    HAL_IRDA_INIT_STRUCT,               /*!< initialization structure */
    HAL_IRDA_DEV_STRUCT,                /*!< device information structure */
    HAL_IRDA_USER_CALLBCAK_STRUCT       /*!< user callback struct */
} hal_irda_struct_type_enum;

/* hal_irda_init */
/* @PARA: periph */
/* @REFER: USART0 */

/* @PARA: p_init */
/* @STRUCT: irda initialization structure */
typedef struct {
    uint32_t        baudrate;       /*!< communication baudrate */
    uint32_t        parity;         /*!< parity mode */
    uint32_t        word_length;    /*!< number of data bits in a frame */
    uint32_t        direction;      /*!< communication transfer direction */
    ControlStatus   rx_fifo_en;     /*!< receive FIFO enable */
    uint32_t        mode;           /*!< power mode */
    uint8_t         prescaler;      /*!< prescaler */
} hal_irda_init_struct;

/* @STRUCT_MEMBER: baudrate */
/* @=NULL */

/* @STRUCT_MEMBER: parity */
/* @DEFINE: IRDA_PARITY_NONE */
#define IRDA_PARITY_NONE               USART_PM_NONE     /*!< no parity */
#define IRDA_PARITY_EVEN               USART_PM_EVEN     /*!< even parity */
#define IRDA_PARITY_ODD                USART_PM_ODD      /*!< odd parity */

/* @STRUCT_MEMBER: word_length */
/* @DEFINE: IRDA_WORD_LENGTH_8BIT */
#define IRDA_WORD_LENGTH_8BIT          USART_WL_8BIT     /*!< 8 bits word length */
#define IRDA_WORD_LENGTH_9BIT          USART_WL_9BIT     /*!< 9 bits word length */

/* @STRUCT_MEMBER: direction */
/* @DEFINE: IRDA_DIRECTION_RX_TX */
#define IRDA_DIRECTION_RX_TX           (USART_TRANSMIT_ENABLE | USART_RECEIVE_ENABLE)  /*!< RX and TX mode */
#define IRDA_DIRECTION_RX_ONLY         (USART_TRANSMIT_DISABLE | USART_RECEIVE_ENABLE) /*!< RX only mode */
#define IRDA_DIRECTION_TX_ONLY         (USART_TRANSMIT_ENABLE | USART_RECEIVE_DISABLE) /*!< TX only mode */

/* @STRUCT_MEMBER: rx_fifo_en */
/* @REFER: ControlStatus */

/* @STRUCT_MEMBER: mode */
/* @DEFINE: IRDA_NORMAL_MODE */
#define IRDA_NORMAL_MODE               USART_IRLP_NORMAL /* normal mode */
#define IRDA_LOW_POWER_MODE            USART_IRLP_LOW    /* low-power mode */

/* @STRUCT_MEMBER: prescaler */
/* @=NULL */

/* irda device interrupt callback function pointer structure */
typedef struct {
    hal_irq_handle_cb receive_complete_handle;   /*!< receive complete callback function */
    hal_irq_handle_cb transmit_ready_handle;     /*!< transmit ready callback function */
    hal_irq_handle_cb transmit_complete_handle;  /*!< transmit complete callback function */
    hal_irq_handle_cb error_handle;              /*!< error callback function */
} hal_irda_irq_struct;

/* IRAD transfer buffer structure */
typedef struct {
    __IO uint8_t *buffer;                        /*!< pointer to transfer buffer */
    __IO uint32_t length;                        /*!< transfer length */
    __IO uint32_t pos;                           /*!< transfer position */
} irda_buffer_struct;

/* irda run state enum */
typedef enum {
    IRDA_STATE_FREE,                             /*!< ready for use */
    IRDA_STATE_BUSY,                             /*!< busy state */
} hal_irda_run_state_enum;

/* irda device information structure */
typedef struct {
    uint32_t                      periph;        /*!< usart port */
    hal_irda_irq_struct           irda_irq;      /*!< device interrupt callback function pointer */
    hal_dma_dev_struct            *p_dma_rx;     /*!< DMA receive pointer */
    hal_dma_dev_struct            *p_dma_tx;     /*!< DMA transmit pointer */
    irda_buffer_struct            txbuffer;      /*!< transmit buffer */
    irda_buffer_struct            rxbuffer;      /*!< receive buffer */
    uint16_t                      data_bit_mask; /*!< mask bit of data */
    __IO uint16_t                 last_error;    /*!< the last error code */
    __IO uint16_t                 error_state;   /*!< error state */
    __IO hal_irda_run_state_enum  tx_state;      /*!< transmit state */
    __IO hal_irda_run_state_enum  rx_state;      /*!< receive state */
    void                          *rx_callback;  /*!< receive callback function pointer */
    void                          *tx_callback;  /*!< transmit callback function pointer */
    hal_mutex_enum                mutex;
    void                          *priv;         /*!< private pointer */
} hal_irda_dev_struct;

typedef void (*hal_irda_user_cb)(hal_irda_dev_struct *irda);

/* irda user callback struct */
typedef struct{
    __IO hal_irda_user_cb complete_func;     /*!< transfer complete callback function */
    __IO hal_irda_user_cb error_func;        /*!< error callback function */
}hal_irda_user_callback_struct;

/* IRDA error code */
#define HAL_IRDA_ERROR_NONE           0U                /*!< no error */
#define HAL_IRDA_ERROR_PERR           BIT(0)            /*!< parity error */
#define HAL_IRDA_ERROR_NERR           BIT(1)            /*!< noise error */
#define HAL_IRDA_ERROR_FERR           BIT(2)            /*!< frame error */
#define HAL_IRDA_ERROR_ORERR          BIT(3)            /*!< overrun error */
#define HAL_IRDA_ERROR_DMATX          BIT(4)            /*!< DMA Tx error */
#define HAL_IRDA_ERROR_DMARX          BIT(5)            /*!< DMA Rx error */

/* function declarations */
/* initialization functions */
/* initialize the irda structure with the default values */
void hal_irda_struct_init(hal_irda_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize irda */
void hal_irda_deinit(hal_irda_dev_struct *irda);
/* @FUNCTION: initialize the irda with specified values */
/* initialize irda */
int32_t hal_irda_init(hal_irda_dev_struct *irda, uint32_t periph, hal_irda_init_struct *p_init);
/* @END */

/* irda interrput handle functions */
/* irda interrupt handler content function,which is merely used in irda_handler */
void hal_irda_irq(hal_irda_dev_struct *irda);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_irda_irq_handle_set(hal_irda_dev_struct *irda, hal_irda_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_irda_irq_handle_all_reset(hal_irda_dev_struct *irda);

/* transmit or receive functions */
/* transmit amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_irda_transmit_poll(hal_irda_dev_struct *irda, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms);
/* receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_irda_receive_poll(hal_irda_dev_struct *irda, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms);
/* transmit amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_irda_transmit_interrupt(hal_irda_dev_struct *irda, uint8_t *p_buffer, uint32_t length, hal_irda_user_cb p_user_func);
/* receive amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_irda_receive_interrupt(hal_irda_dev_struct *irda, uint8_t *p_buffer, uint32_t length, hal_irda_user_cb p_user_func);
/* transmit amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_irda_transmit_dma(hal_irda_dev_struct *irda, uint8_t *p_buffer, uint16_t length, hal_irda_user_callback_struct *p_func);
/* receive amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_irda_receive_dma(hal_irda_dev_struct *irda, uint8_t *p_buffer, uint16_t length, hal_irda_user_callback_struct *p_func);

/* transfer control functions */
/* pause irda DMA transfer during transmission process */
int32_t hal_irda_dma_pause(hal_irda_dev_struct *irda);
/* resume irda DMA transfer during transmission process */
int32_t hal_irda_dma_resume(hal_irda_dev_struct *irda);
/* stop irda transmit transfer */
/* the function is blocking */
int32_t hal_irda_transmit_stop(hal_irda_dev_struct *irda);
/* stop irda receive transfer */
/* the function is blocking */
int32_t hal_irda_receive_stop(hal_irda_dev_struct *irda);


/* hals function declarations */
/* initialization functions */
/* reset irda */
void hals_irda_deinit(uint32_t irda_periph);
/* configure irda baud rate value */
void hals_irda_baudrate_set(uint32_t irda_periph, uint32_t baudval);
/* configure irda parity */
void hals_irda_parity_config(uint32_t irda_periph, uint32_t paritycfg);
/* configure irda word length */
void hals_irda_word_length_set(uint32_t irda_periph, uint32_t wlen);
/* configure irda stop bit length */
void hals_irda_stop_bit_set(uint32_t irda_periph, uint32_t stblen);
/* enable irda */
void hals_irda_enable(uint32_t irda_periph);
/* disable irda */
void hals_irda_disable(uint32_t irda_periph);
/* configure irda transmitter */
void hals_irda_transmit_config(uint32_t irda_periph, uint32_t txconfig);
/* configure irda receiver */
void hals_irda_receive_config(uint32_t irda_periph, uint32_t rxconfig);

/* irda transmit data function */
void hals_irda_data_transmit(uint32_t irda_periph, uint16_t data);
/* smartcard receive data function */
uint16_t hals_irda_data_receive(uint32_t irda_periph);

/* irda communication */
/* enable irda mode */
void hals_irda_mode_enable(uint32_t irda_periph);
/* disable irda mode */
void hals_irda_mode_disable(uint32_t irda_periph);
/* configure the peripheral clock prescaler in irda irda low-power or smartcard mode */
void hals_irda_prescaler_config(uint32_t irda_periph, uint32_t psc);
/* configure irda low-power */
void hals_irda_lowpower_config(uint32_t irda_periph, uint32_t irlp);

/* irda DMA */
/* configure irda DMA reception */
void hals_irda_dma_receive_config(uint32_t irda_periph, uint8_t dmacmd);
/* configure irda DMA transmission */
void hals_irda_dma_transmit_config(uint32_t irda_periph, uint8_t dmacmd);
/* enable DMA on reception error */
void hals_irda_reception_error_dma_enable(uint32_t irda_periph);
/* disable DMA on reception error */
void hals_irda_reception_error_dma_disable(uint32_t irda_periph);

/* flag & interrupt functions */
/* get irda status */
FlagStatus hals_irda_flag_get(uint32_t irda_periph, usart_flag_enum flag);
/* clear irda status */
void hals_irda_flag_clear(uint32_t irda_periph, usart_flag_enum flag);
/* enable irda interrupt */
void hals_irda_interrupt_enable(uint32_t irda_periph, usart_interrupt_enum interrupt);
/* disable irda interrupt */
void hals_irda_interrupt_disable(uint32_t irda_periph, usart_interrupt_enum interrupt);
/* get irda interrupt flag status */
FlagStatus hals_irda_interrupt_flag_get(uint32_t irda_periph, usart_interrupt_flag_enum int_flag);
/* clear irda interrupt flag */
void hals_irda_interrupt_flag_clear(uint32_t irda_periph, usart_interrupt_flag_enum int_flag);

#endif /* GD32F3X0_HAL_IRDA_H */
