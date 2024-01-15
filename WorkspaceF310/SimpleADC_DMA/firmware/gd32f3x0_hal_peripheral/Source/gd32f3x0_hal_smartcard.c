/*!
    \file    gd32f3x0_hal_smartcard.c
    \brief   SMARTCARD driver

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

#include "gd32f3x0_hal.h"

/* smartcard private function */
static FlagStatus _smartcard_error_flag_get(hal_smartcard_dev_struct *smartcard);
static void _smartcard_transmit_complete_interrupt(void *smartcard);
static void _smartcard_transmit_interrupt(void *smartcard);
static void _smartcard_receive_interrupt(void *smartcard);
static void _smartcard_transmit_dma(void *dma);
static void _smartcard_receive_dma(void *dma);
static void _smartcard_dma_error(void *dma);

/*!
    \brief      initialize the smartcard structure with the default values
                note: this function must be called after the structure is created
    \param[in]  hal_struct_type: smartcard structure type
      \arg        HAL_SMARTCARD_INIT_STRUCT: initialization structure
      \arg        HAL_SMARTCARD_INIT_EX_STRUCT: initialization extend structure
      \arg        HAL_SMARTCARD_DEV_STRUCT: device information structure
      \arg        HAL_SMARTCARD_USER_CALLBCAK_STRUCT: user callback structure
      \arg        HAL_SMARTCARD_IRQ_INIT_STRUCT: interrupt callback initialization structure
    \param[in]  p_struct: init structure pointer
    \param[out] none
    \retval     none
*/
void hal_smartcard_struct_init(hal_smartcard_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1U == HAL_PARAMETER_CHECK)
    if(NULL == p_struct) {
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_SMARTCARD_INIT_STRUCT:
        /* initialize smartcard initialization structure with the default values */
        ((hal_smartcard_init_struct *)p_struct)->baudrate                = 115200U;
        ((hal_smartcard_init_struct *)p_struct)->parity                  = SMARTCARD_PARITY_EVEN;
        ((hal_smartcard_init_struct *)p_struct)->word_length             = SMARTCARD_WORD_LENGTH_9BIT;
        ((hal_smartcard_init_struct *)p_struct)->stop_bit                = SMARTCARD_STOP_BIT_1_5;
        ((hal_smartcard_init_struct *)p_struct)->direction               = SMARTCARD_DIRECTION_RX_TX;
        ((hal_smartcard_init_struct *)p_struct)->clock_polarity          = SMARTCARD_CLOCK_POLARITY_LOW;
        ((hal_smartcard_init_struct *)p_struct)->clock_phase             = SMARTCARD_CLOCK_PHASE_1CK;
        ((hal_smartcard_init_struct *)p_struct)->clock_length_lastbit    = SMARTCARD_LAST_BIT_NOT_OUTPUT;
        ((hal_smartcard_init_struct *)p_struct)->prescaler               = 1U;
        ((hal_smartcard_init_struct *)p_struct)->guard_time              = 0U;
        ((hal_smartcard_init_struct *)p_struct)->nack_state              = SMARTCARD_NACK_DISABLE;
        ((hal_smartcard_init_struct *)p_struct)->early_nack              = DISABLE;
        ((hal_smartcard_init_struct *)p_struct)->rx_fifo_en              = DISABLE;
        ((hal_smartcard_init_struct *)p_struct)->timeout_enable          = DISABLE;
        ((hal_smartcard_init_struct *)p_struct)->timeout_value           = 0U;
        ((hal_smartcard_init_struct *)p_struct)->sample_method           = SMARTCARD_THREE_SAMPLE_BIT;
        ((hal_smartcard_init_struct *)p_struct)->block_length            = 0U;
        ((hal_smartcard_init_struct *)p_struct)->auto_retry_count        = 0U;
        ((hal_smartcard_init_struct *)p_struct)->first_bit_msb           = DISABLE;
        ((hal_smartcard_init_struct *)p_struct)->tx_rx_swap              = DISABLE;
        ((hal_smartcard_init_struct *)p_struct)->rx_level_invert         = DISABLE;
        ((hal_smartcard_init_struct *)p_struct)->tx_level_invert         = DISABLE;
        ((hal_smartcard_init_struct *)p_struct)->data_bit_invert         = DISABLE;
        ((hal_smartcard_init_struct *)p_struct)->overrun_disable         = DISABLE;
        ((hal_smartcard_init_struct *)p_struct)->rx_error_dma_stop       = DISABLE;
        break;

    case HAL_SMARTCARD_DEV_STRUCT:
        /* initialize smartcard device information structure with the default values */
        ((hal_smartcard_dev_struct *)p_struct)->periph                                 = 0U;
        ((hal_smartcard_dev_struct *)p_struct)->smartcard_irq.error_handle             = NULL;
        ((hal_smartcard_dev_struct *)p_struct)->smartcard_irq.receive_complete_handle  = NULL;
        ((hal_smartcard_dev_struct *)p_struct)->smartcard_irq.transmit_complete_handle = NULL;
        ((hal_smartcard_dev_struct *)p_struct)->smartcard_irq.transmit_ready_handle    = NULL;
        ((hal_smartcard_dev_struct *)p_struct)->p_dma_rx                               = NULL;
        ((hal_smartcard_dev_struct *)p_struct)->p_dma_tx                               = NULL;
        ((hal_smartcard_dev_struct *)p_struct)->txbuffer.buffer                        = NULL;
        ((hal_smartcard_dev_struct *)p_struct)->txbuffer.length                        = 0U;
        ((hal_smartcard_dev_struct *)p_struct)->txbuffer.pos                           = 0U;
        ((hal_smartcard_dev_struct *)p_struct)->rxbuffer.buffer                        = NULL;
        ((hal_smartcard_dev_struct *)p_struct)->rxbuffer.length                        = 0U;
        ((hal_smartcard_dev_struct *)p_struct)->rxbuffer.pos                           = 0U;
        ((hal_smartcard_dev_struct *)p_struct)->last_error                             = HAL_SMARTCARD_ERROR_NONE;
        ((hal_smartcard_dev_struct *)p_struct)->error_state                            = HAL_SMARTCARD_ERROR_NONE;
        ((hal_smartcard_dev_struct *)p_struct)->tx_state                               = SMARTCARD_STATE_FREE;
        ((hal_smartcard_dev_struct *)p_struct)->rx_state                               = SMARTCARD_STATE_FREE;
        ((hal_smartcard_dev_struct *)p_struct)->rx_callback                            = NULL;
        ((hal_smartcard_dev_struct *)p_struct)->tx_callback                            = NULL;
        ((hal_smartcard_dev_struct *)p_struct)->mutex                                  = HAL_MUTEX_UNLOCKED;
        ((hal_smartcard_dev_struct *)p_struct)->priv                                   = NULL;
        break;

    case HAL_SMARTCARD_USER_CALLBCAK_STRUCT:
        /* initialize user callback structure with the default values */
        ((hal_smartcard_user_callback_struct *)p_struct)->complete_func = NULL;
        ((hal_smartcard_user_callback_struct *)p_struct)->error_func = NULL;
        break;

    case HAL_SMARTCARD_IRQ_INIT_STRUCT:
        /* initialize interrupt callback structure with the default values */
        ((hal_smartcard_irq_struct *)p_struct)->error_handle = NULL;
        ((hal_smartcard_irq_struct *)p_struct)->receive_complete_handle = NULL;
        ((hal_smartcard_irq_struct *)p_struct)->transmit_complete_handle = NULL;
        ((hal_smartcard_irq_struct *)p_struct)->transmit_ready_handle = NULL;
        break;

    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize smartcard
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smartcard_deinit(hal_smartcard_dev_struct *smartcard)
{
    uint32_t periph;
    periph = smartcard->periph;

    if((USART0 == periph)) {
        /* deinitialize the peripheral */
        hals_smartcard_deinit(periph);
        /* initialize smartcard extend initialization structure */
        hal_smartcard_struct_init(HAL_SMARTCARD_DEV_STRUCT, smartcard);
        smartcard->periph = periph;
    } else {
        HAL_DEBUGE("parameter [smartcard->periph] value is invalid");
    }
}

/*!
    \brief      initialize smartcard
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which SMARTCARD is initialized
                    this parameter can only be USART0 for gd32f3x0
    \param[in]  p_init: the initialization data needed to initialize smartcard
                  baudrate: communication baudrate
                  parity: SMARTCARD_PARITY_EVEN, SMARTCARD_PARITY_ODD
                  word_length: SMARTCARD_WORD_LENGTH_9BIT
                  stop_bit: SMARTCARD_STOP_BIT_0_5, SMARTCARD_STOP_BIT_1_5
                  direction: SMARTCARD_DIRECTION_RX_TX, SMARTCARD_DIRECTION_RX_ONLY, SMARTCARD_DIRECTION_TX_ONLY
                  clock_polarity: SMARTCARD_CLOCK_POLARITY_LOW, SMARTCARD_CLOCK_POLARITY_HIGH
                  clock_phase: SMARTCARD_CLOCK_PHASE_1CK, SMARTCARD_CLOCK_PHASE_2CK
                  clock_length_lastbit: SMARTCARD_LAST_BIT_NOT_OUTPUT, SMARTCARD_LAST_BIT_OUTPUT
                  prescaler: 2,4,6 - 62
                  guard_time: 0 - 255
                  nack_state: SMARTCARD_NACK_DISABLE, SMARTCARD_NACK_ENABLE
                  early_nack: DISABLE, ENABLE
                  rx_fifo_en: DISABLE, ENABLE
                  timeout_enable: DISABLE, ENABLE
                  timeout_value: 0 - (2^24-1)
                  sample_method: SMARTCARD_THREE_SAMPLE_BIT, SMARTCARD_ONE_SAMPLE_BIT
                  block_length: 0 - 255
                  auto_retry_count: 0 - 7
                  first_bit_msb: DISABLE, ENABLE
                  tx_rx_swap: DISABLE, ENABLE
                  rx_level_invert: DISABLE, ENABLE
                  tx_level_invert: DISABLE, ENABLE
                  data_bit_invert: DISABLE, ENABLE
                  overrun_disable: DISABLE, ENABLE
                  rx_error_dma_stop: DISABLE, ENABLE
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_smartcard_init(hal_smartcard_dev_struct *smartcard, uint32_t periph, hal_smartcard_init_struct *p_init)
{
    uint32_t reg_temp;

#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer and p_init address */
    if((NULL == smartcard) || (NULL == p_init)) {
        HAL_DEBUGE("pointer [smartcard] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check periph parameter */
    if(USART0 != periph) {
        HAL_DEBUGE("parameter [periph] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check periph value from smartcard device struct */
    if(0U != smartcard->periph) {
        HAL_DEBUGI("periph value from smartcard device struct has been rewrite");
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* lock smartcard */
    HAL_LOCK(smartcard);

    smartcard->periph   = periph;

    /* disable the peripheral */
    hals_smartcard_disable(periph);

    /* configure CTL0 register */
    reg_temp = USART_CTL0(periph);
    reg_temp &= ~(USART_CTL0_PM | USART_CTL0_PCEN | USART_CTL0_WL | USART_CTL0_OVSMOD | \
                  USART_CTL0_REN | USART_CTL0_TEN);
    reg_temp |= (p_init->direction | p_init->parity | p_init->word_length);
    USART_CTL0(periph) = reg_temp;

    /* configure CTL1 register */
    reg_temp = USART_CTL1(periph);
    reg_temp &= ~(USART_CTL1_RTEN | USART_CTL1_CLEN | USART_CTL1_CPH | USART_CTL1_CPL | \
                  USART_CTL1_CKEN | USART_CTL1_STB);
    /* Synchronous mode is activated by default */
    reg_temp |= (USART_CTL1_CKEN | p_init->clock_polarity | p_init->clock_phase | \
                 p_init->clock_length_lastbit | p_init->stop_bit);
    USART_CTL1(periph) = reg_temp;

    /* configure timeout enable */
    if(ENABLE == p_init->timeout_enable) {
        USART_CTL1(periph) |= USART_CTL1_RTEN;
    } else {
        USART_CTL1(periph) &= ~(USART_CTL1_RTEN);
    }
    /* configure data inversion */
    if(ENABLE == p_init->data_bit_invert) {
        USART_CTL1(periph) |= USART_CTL1_DINV;
    } else {
        USART_CTL1(periph) &= ~(USART_CTL1_DINV);
    }
    /* configure Rx pin active level inversion */
    if(ENABLE == p_init->rx_level_invert) {
        USART_CTL1(periph) |= USART_CTL1_RINV;
    } else {
        USART_CTL1(periph) &= ~(USART_CTL1_RINV);
    }
    /* configure Tx pin active level inversion */
    if(ENABLE == p_init->tx_level_invert) {
        USART_CTL1(periph) |= USART_CTL1_TINV;
    } else {
        USART_CTL1(periph) &= ~(USART_CTL1_TINV);
    }
    /* configure Rx/Tx pins swap */
    if(ENABLE == p_init->tx_rx_swap) {
        USART_CTL1(periph) |= USART_CTL1_STRP;
    } else {
        USART_CTL1(periph) &= ~(USART_CTL1_STRP);
    }
    /* configure MSB first on communication line */
    if(ENABLE == p_init->first_bit_msb) {
        USART_CTL1(periph) |= USART_CTL1_MSBF;
    } else {
        USART_CTL1(periph) &= ~(USART_CTL1_MSBF);
    }

    /* configure CTL2 register */
    reg_temp = USART_CTL2(periph);
    reg_temp &= ~(USART_CTL2_OSB | USART_CTL2_NKEN | USART_CTL2_SCRTNUM);
    reg_temp |= (p_init->sample_method | p_init->nack_state | (p_init->auto_retry_count << 17U));
    USART_CTL2(periph) = reg_temp;
    /* configure Rx overrun detection disabling */
    if(ENABLE == p_init->overrun_disable) {
        USART_CTL2(periph) |= USART_CTL2_OVRD;
    } else {
        USART_CTL2(periph) &= ~(USART_CTL2_OVRD);
    }

    /* configure DMA disabling on reception error */
    if(ENABLE == p_init->rx_error_dma_stop) {
        USART_CTL2(periph) |= USART_CTL2_DDRE;
    } else {
        USART_CTL2(periph) &= ~(USART_CTL2_DDRE);
    }

    /* configure GP register */
    reg_temp = USART_GP(periph);
    reg_temp &= ~(USART_GP_PSC | USART_GP_GUAT);
    reg_temp |= ((p_init->guard_time << 8U) | (p_init->prescaler));
    USART_GP(periph) = reg_temp;

    /* configure RT register */
    reg_temp = USART_RT(periph);
    reg_temp &= ~(USART_RT_RT | USART_RT_BL);
    reg_temp |= ((p_init->block_length << 24U));
    if(ENABLE == p_init->timeout_enable) {
        reg_temp |= p_init->timeout_value;
    }
    USART_RT(periph) = reg_temp;

    /* configure RFCS register */
    if(ENABLE == p_init->early_nack) {
        USART_RFCS(periph) |= USART_RFCS_ELNACK;
    } else {
        USART_RFCS(periph) &= ~USART_RFCS_ELNACK;
    }
    if(ENABLE == p_init->rx_fifo_en) {
        USART_RFCS(periph) |= USART_RFCS_RFEN;
    } else {
        USART_RFCS(periph) &= ~USART_RFCS_RFEN;
    }

    /* configure baud rate */
    hals_smartcard_baudrate_set(periph, p_init->baudrate);

    /* clear LMEN, HDEN, IREN */
    hals_uart_halfduplex_disable(periph);
    hals_irda_mode_disable(periph);
    hals_uart_lin_mode_disable(periph);

    /* enable smartcard */
    hals_smartcard_mode_enable(periph);

    /* initialize Tx and Rx state */
    smartcard->tx_state = SMARTCARD_STATE_FREE;
    smartcard->rx_state = SMARTCARD_STATE_FREE;

    /* enable usart */
    hals_smartcard_enable(smartcard->periph);

    /* unlock smartcard */
    HAL_UNLOCK(smartcard);

    return HAL_ERR_NONE;
}

/*!
    \brief      smartcard interrupt handler content function,which is merely used in smartcard_handler
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smartcard_irq(hal_smartcard_dev_struct *smartcard)
{
    /* if no error occurs */
    if(RESET == _smartcard_error_flag_get(smartcard)) {
        /* if SMARTCARD is in receiver mode */
        if(RESET != hals_smartcard_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_RBNE)) {
            if(NULL != smartcard->smartcard_irq.receive_complete_handle) {
                smartcard->smartcard_irq.receive_complete_handle(smartcard);
            }
            return;
        }

        /* if some errors occur */
    } else {
        /* smartcard parity error interrupt occurred */
        if(RESET != hals_smartcard_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_PERR)) {
            hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_PERR);
            smartcard->error_state |= HAL_SMARTCARD_ERROR_PERR;
            smartcard->last_error = HAL_SMARTCARD_ERROR_PERR;
        }

        /* smartcard noise error interrupt occurred */
        if(RESET != hals_smartcard_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_ERR_NERR)) {
            hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_NERR);
            smartcard->error_state |= HAL_SMARTCARD_ERROR_NERR;
            smartcard->last_error = HAL_SMARTCARD_ERROR_NERR;
        }

        /* smartcard frame error interrupt occurred */
        if(RESET != hals_smartcard_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_ERR_FERR)) {
            hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_FERR);
            smartcard->error_state |= HAL_SMARTCARD_ERROR_FERR;
            smartcard->last_error = HAL_SMARTCARD_ERROR_FERR;
        }

        /* smartcard overrun interrupt occurred */
        if(RESET != hals_smartcard_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_ERR_ORERR)) {
            hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_ORERR);
            smartcard->error_state |= HAL_SMARTCARD_ERROR_ORERR;
            smartcard->last_error = HAL_SMARTCARD_ERROR_ORERR;
        }

        /* smartcard read data buffer not empty and overrun error overrun interrupt occurred */
        if(RESET != hals_smartcard_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_RBNE_ORERR)) {
            hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_RBNE_ORERR);
            smartcard->error_state |= HAL_SMARTCARD_ERROR_ORERR;
            smartcard->last_error = HAL_SMARTCARD_ERROR_ORERR;
        }

        /* smartcard receiver timeout interrupt occurred */
        if(RESET != hals_smartcard_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_RT)) {
            hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_RT);
            smartcard->error_state |= HAL_SMARTCARD_ERROR_RTF;
            smartcard->last_error = HAL_SMARTCARD_ERROR_RTF;
        }

        if(HAL_SMARTCARD_ERROR_NONE != smartcard->error_state) {
            /* call error callback  */
            if(NULL != smartcard->smartcard_irq.error_handle) {
                smartcard->smartcard_irq.error_handle(smartcard);
                smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;
            }
            return;
        }
    }

    /* transmitter buffer empty interrupt handle */
    if(RESET != hals_smartcard_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_TBE)) {
        if(smartcard->smartcard_irq.transmit_ready_handle != NULL) {
            smartcard->smartcard_irq.transmit_ready_handle(smartcard);
        }

        return;
    }

    /* transmission complete interrupt handle */
    if(RESET != hals_smartcard_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_TC)) {
        hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_TC);

        if(NULL != smartcard->smartcard_irq.transmit_complete_handle) {
            smartcard->smartcard_irq.transmit_complete_handle(smartcard);
        }

        return;
    }
}

/*!
    \brief      set user-defined interrupt callback function
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_irq: smartcard interrupt callback function pointer
    \param[out] none
    \retval     none
*/
void hal_smartcard_irq_handle_set(hal_smartcard_dev_struct *smartcard, hal_smartcard_irq_struct *p_irq)
{
    /* initialize smartcard error callback */
    if(NULL != p_irq->error_handle) {
        smartcard->smartcard_irq.error_handle = p_irq->error_handle;
        hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_ERR);
        hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_PERR);
    } else {
        smartcard->smartcard_irq.error_handle = NULL;
        hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_ERR);
        hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_PERR);
    }

    /* initialize smartcard receive completed callback */
    if(NULL != p_irq->receive_complete_handle) {
        smartcard->smartcard_irq.receive_complete_handle = p_irq->receive_complete_handle;
        hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_RBNE);
    } else {
        smartcard->smartcard_irq.receive_complete_handle = NULL;
        hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_RBNE);
    }

    /* initialize smartcard transmit completed callback */
    if(NULL != p_irq->transmit_complete_handle) {
        smartcard->smartcard_irq.transmit_complete_handle = p_irq->transmit_complete_handle;
        hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_TC);
    } else {
        smartcard->smartcard_irq.transmit_complete_handle = NULL;
        hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_TC);
    }

    /* initialize smartcard transmit ready callback */
    if(NULL != p_irq->transmit_ready_handle) {
        smartcard->smartcard_irq.transmit_ready_handle = p_irq->transmit_ready_handle;
        hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_TBE);
    } else {
        smartcard->smartcard_irq.transmit_ready_handle = NULL;
        hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_TBE);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smartcard_irq_handle_all_reset(hal_smartcard_dev_struct *smartcard)
{
    /* configure interrupt callback function to NULL */
    smartcard->smartcard_irq.error_handle = NULL;
    smartcard->smartcard_irq.receive_complete_handle = NULL;
    smartcard->smartcard_irq.transmit_complete_handle = NULL;
    smartcard->smartcard_irq.transmit_ready_handle = NULL;
}

/*!
    \brief      transmit amounts of data by poll method
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY, HAL_ERR_TIMEOUT details refer to gd32f3x0_hal.h
*/
int32_t hal_smartcard_transmit_poll(hal_smartcard_dev_struct *smartcard, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms)
{
    uint32_t tick_start;

#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer, p_buffer address and length */
    if((NULL == smartcard) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [usart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* check whether the Tx state is in ready */
    if(SMARTCARD_STATE_BUSY == smartcard->tx_state) {
        HAL_DEBUGE("usart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock smartcard */
    HAL_LOCK(smartcard);

    /* disable receive and enable transmit */
    hals_smartcard_receive_config(smartcard->periph, USART_RECEIVE_DISABLE);
    hals_smartcard_command_enable(smartcard->periph, USART_CMD_RXFCMD);
    hals_smartcard_transmit_config(smartcard->periph, USART_TRANSMIT_ENABLE);

    /* initialize transmit parameter */
    smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;
    smartcard->tx_state = SMARTCARD_STATE_BUSY;
    smartcard->txbuffer.buffer  = (uint8_t *)p_buffer;
    smartcard->txbuffer.length  = length;
    smartcard->txbuffer.pos     = 0U;

    /* configure timeout */
    tick_start = hal_sys_basetick_count_get();

    while(smartcard->txbuffer.pos < smartcard->txbuffer.length) {
        /* wait for transmit buffer empty */
        while(RESET == hals_smartcard_flag_get(smartcard->periph, USART_FLAG_TBE)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    HAL_DEBUGW("smartcard transmit timeout");
                    smartcard->tx_state = SMARTCARD_STATE_FREE;
                    /* unlock smartcard */
                    HAL_UNLOCK(smartcard);

                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* write the data to be transmitted */
        hals_smartcard_data_transmit(smartcard->periph, (*smartcard->txbuffer.buffer & (uint8_t)0xFFU));
        smartcard->txbuffer.buffer++;
        /* change the transmit pointer */
        smartcard->txbuffer.pos++;
    }

    /* wait for transmit complete */
    while(RESET == hals_smartcard_flag_get(smartcard->periph, USART_FLAG_TC)) {
        if(HAL_TIMEOUT_FOREVER != timeout_ms) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                HAL_DEBUGW("smartcard transmit timeout");
                smartcard->tx_state = SMARTCARD_STATE_FREE;
                /* unlock smartcard */
                HAL_UNLOCK(smartcard);
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    /* re-enable receive */
    hals_smartcard_receive_config(smartcard->periph, USART_RECEIVE_ENABLE);

    /* change the Tx state to free */
    smartcard->tx_state = SMARTCARD_STATE_FREE;

    /* unlock smartcard */
    HAL_UNLOCK(smartcard);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by poll method
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY, HAL_ERR_TIMEOUT details refer to gd32f3x0_hal.h
*/
int32_t hal_smartcard_receive_poll(hal_smartcard_dev_struct *smartcard, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms)
{
    uint32_t tick_start;

#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer, p_buffer address and length */
    if((NULL == smartcard) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [usart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* check whether the Rx state is in ready */
    if(SMARTCARD_STATE_BUSY == smartcard->rx_state) {
        HAL_DEBUGE("usart rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock smartcard */
    HAL_LOCK(smartcard);

    smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;
    smartcard->rx_state = SMARTCARD_STATE_BUSY;

    /* initialize receive parameter */
    smartcard->rxbuffer.buffer  = (uint8_t *)p_buffer;
    smartcard->rxbuffer.length  = length;
    smartcard->rxbuffer.pos     = 0U;

    /* configure timeout */
    tick_start = hal_sys_basetick_count_get();

    while(smartcard->rxbuffer.pos < smartcard->rxbuffer.length) {
        /* wait for read data buffer not empty */
        while(RESET == hals_smartcard_flag_get(smartcard->periph, USART_FLAG_RBNE)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    HAL_DEBUGW("usart receive timeout");
                    smartcard->rx_state = SMARTCARD_STATE_FREE;
                    /* unlock smartcard */
                    HAL_UNLOCK(smartcard);

                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* read data from data register */
        *smartcard->rxbuffer.buffer = (uint8_t)(hals_smartcard_data_receive(smartcard->periph) & 0xFFU);
        smartcard->rxbuffer.buffer++;

        /* change the receive pointer */
        smartcard->rxbuffer.pos++;
    }

    /* change the Rx state to free */
    smartcard->rx_state = SMARTCARD_STATE_FREE;

    /* unlock smartcard */
    HAL_UNLOCK(smartcard);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by interrupt method
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, details refer to gd32f3x0_hal.h
*/
int32_t hal_smartcard_transmit_interrupt(hal_smartcard_dev_struct *smartcard, uint8_t *p_buffer, uint32_t length, hal_smartcard_user_cb p_user_func)
{
#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer, p_buffer address and length */
    if((NULL == smartcard) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [usart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* check whether the Tx state is in ready */
    if(SMARTCARD_STATE_BUSY == smartcard->tx_state) {
        HAL_DEBUGE("usart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock smartcard */
    HAL_LOCK(smartcard);

    smartcard->tx_state = SMARTCARD_STATE_BUSY;
    smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;

    /* disable receive and enable transmit */
    hals_smartcard_receive_config(smartcard->periph, USART_RECEIVE_DISABLE);
    hals_smartcard_command_enable(smartcard->periph, USART_CMD_RXFCMD);
    hals_smartcard_transmit_config(smartcard->periph, USART_TRANSMIT_ENABLE);

    /* initialize transmit parameter */
    smartcard->txbuffer.buffer  = (uint8_t *)p_buffer;
    smartcard->txbuffer.length  = length;
    smartcard->txbuffer.pos     = 0U;
    smartcard->tx_callback      = (void *)p_user_func;
    smartcard->smartcard_irq.transmit_ready_handle = _smartcard_transmit_interrupt;
    smartcard->smartcard_irq.transmit_complete_handle = _smartcard_transmit_complete_interrupt;

    /* clear SMARTCARD TC interrupt flag */
    hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_TC);

    /* enable ERR(frame error) and TBE interrupt */
    hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_ERR);
    hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_TBE);

    /* unlock smartcard */
    HAL_UNLOCK(smartcard);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by interrupt method
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be sent received
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, details refer to gd32f3x0_hal.h
*/
int32_t hal_smartcard_receive_interrupt(hal_smartcard_dev_struct *smartcard, uint8_t *p_buffer, uint32_t length, hal_smartcard_user_cb p_user_func)
{
#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer, p_buffer address and length */
    if((NULL == smartcard) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [usart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* check whether the Rx state is in ready */
    if(SMARTCARD_STATE_BUSY == smartcard->rx_state) {
        HAL_DEBUGE("usart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock smartcard */
    HAL_LOCK(smartcard);

    smartcard->rx_state = SMARTCARD_STATE_BUSY;
    smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;

    /* initialize receive parameter */
    smartcard->rxbuffer.buffer  = (uint8_t *)p_buffer;
    smartcard->rxbuffer.length  = length;
    smartcard->rxbuffer.pos     = 0U;
    smartcard->rx_callback      = (void *)p_user_func;
    smartcard->smartcard_irq.receive_complete_handle = _smartcard_receive_interrupt;

    /* enable interrupt */
    hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_PERR);
    hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_ERR);
    hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_RBNE);

    /* unlock smartcard */
    HAL_UNLOCK(smartcard);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by dma method
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, details refer to gd32f3x0_hal.h
*/
int32_t hal_smartcard_transmit_dma(hal_smartcard_dev_struct *smartcard, uint8_t *p_buffer, uint16_t length, hal_smartcard_user_callback_struct *p_user_func)
{
    hal_dma_irq_struct dma_irq;

#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer, p_buffer address and length */
    if((NULL == smartcard) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [usart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* check whether the Tx state is in ready */
    if(SMARTCARD_STATE_BUSY == smartcard->tx_state) {
        HAL_DEBUGE("usart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* check whether the pointer of DMA Tx is NULL */
    if(NULL == smartcard->p_dma_tx) {
        HAL_DEBUGE("parameter [smartcard->p_dma_tx] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* lock smartcard */
    HAL_LOCK(smartcard);

    smartcard->tx_state = SMARTCARD_STATE_BUSY;
    smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;

    /* disable receive and enable transmit */
    hals_smartcard_receive_config(smartcard->periph, USART_RECEIVE_DISABLE);
    hals_smartcard_command_enable(smartcard->periph, USART_CMD_RXFCMD);
    hals_smartcard_transmit_config(smartcard->periph, USART_TRANSMIT_ENABLE);

    /* initialize transmit parameter */
    smartcard->txbuffer.buffer  = (uint8_t *)p_buffer;
    smartcard->txbuffer.length  = length;
    smartcard->txbuffer.pos     = 0U;

    if(NULL != p_user_func) {
        smartcard->tx_callback = (void *)p_user_func->complete_func;
        smartcard->smartcard_irq.error_handle = (hal_irq_handle_cb)p_user_func->error_func;
    } else {
        smartcard->tx_callback = NULL;
        smartcard->smartcard_irq.error_handle = NULL;
    }
    smartcard->smartcard_irq.transmit_complete_handle = _smartcard_transmit_complete_interrupt;

    /* configure DMA interrupt callback function */
    dma_irq.full_finish_handle = _smartcard_transmit_dma;
    dma_irq.error_handle = _smartcard_dma_error;
    if(NULL != smartcard->p_dma_tx->dma_irq.half_finish_handle) {
        dma_irq.half_finish_handle = smartcard->p_dma_tx->dma_irq.half_finish_handle;
    } else {
        dma_irq.half_finish_handle = NULL;
    }

    /* start DMA interrupt mode transfer */
    if(HAL_ERR_NONE != hal_dma_start_interrupt(smartcard->p_dma_tx,
            (uint32_t)smartcard->txbuffer.buffer, (uint32_t)&USART_TDATA(smartcard->periph),
            smartcard->txbuffer.length, &dma_irq)) {
        smartcard->tx_state = SMARTCARD_STATE_FREE;
        smartcard->error_state = HAL_SMARTCARD_ERROR_DMATX;
        /* unlock uart */
        HAL_UNLOCK(smartcard);
    }

    /* clear SMARTCARD TC interrupt flag */
    hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_TC);

    /* enable ERR interrupt(frame error) */
    hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_ERR);

    /* DMA enable for smartcard transmission */
    hals_smartcard_dma_transmit_config(smartcard->periph, USART_TRANSMIT_DMA_ENABLE);

    /* unlock smartcard */
    HAL_UNLOCK(smartcard);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by dma method
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be sent received
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, details refer to gd32f3x0_hal.h
*/
int32_t hal_smartcard_receive_dma(hal_smartcard_dev_struct *smartcard, uint8_t *p_buffer, uint16_t length, hal_smartcard_user_callback_struct *p_user_func)
{
    hal_dma_irq_struct dma_irq;

#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer, p_buffer address and length */
    if((NULL == smartcard) || (NULL == p_buffer) || (0 == length)) {
        HAL_DEBUGE("parameter [usart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* check whether the Rx state is in ready */
    if(SMARTCARD_STATE_BUSY == smartcard->rx_state) {
        HAL_DEBUGE("usart rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* check whether the pointer of DMA Rx is NULL */
    if(NULL == smartcard->p_dma_rx) {
        HAL_DEBUGE("parameter [smartcard->p_dma_rx] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* lock smartcard */
    HAL_LOCK(smartcard);

    smartcard->rx_state = SMARTCARD_STATE_BUSY;
    smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;

    /* initialize receive parameter */
    smartcard->rxbuffer.buffer  = (uint8_t *)p_buffer;
    smartcard->rxbuffer.length  = length;
    smartcard->rxbuffer.pos     = 0U;

    if(NULL != p_user_func) {
        smartcard->rx_callback = (void *)p_user_func->complete_func;
        smartcard->smartcard_irq.error_handle = (hal_irq_handle_cb)p_user_func->error_func;
    } else {
        smartcard->rx_callback = NULL;
        smartcard->smartcard_irq.error_handle = NULL;
    }

    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _smartcard_receive_dma;
    dma_irq.error_handle = _smartcard_dma_error;
    if(NULL != smartcard->p_dma_rx->dma_irq.half_finish_handle) {
        dma_irq.half_finish_handle = smartcard->p_dma_rx->dma_irq.half_finish_handle;
    } else {
        dma_irq.half_finish_handle = NULL;
    }

    /* start DMA interrupt mode transfer */
    if(HAL_ERR_NONE != hal_dma_start_interrupt(smartcard->p_dma_rx,
            (uint32_t)&USART_RDATA(smartcard->periph), (uint32_t)smartcard->rxbuffer.buffer,
            smartcard->rxbuffer.length, &dma_irq)) {
        smartcard->rx_state = SMARTCARD_STATE_FREE;
        smartcard->error_state = HAL_SMARTCARD_ERROR_DMARX;
        /* unlock uart */
        HAL_UNLOCK(smartcard);
    }

    /* enable the usart parity error interrupt */
    hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_PERR);

    /* enable the usart error interrupt: (frame error, noise error, overrun error) */
    hals_smartcard_interrupt_enable(smartcard->periph, USART_INT_ERR);

    /* DMA enable for smartcard reception */
    hals_smartcard_dma_receive_config(smartcard->periph, USART_RECEIVE_DMA_ENABLE);

    /* unlock smartcard */
    HAL_UNLOCK(smartcard);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop smartcard transmit transfer
                the function is blocking
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_smartcard_transmit_stop(hal_smartcard_dev_struct *smartcard)
{
#if (1U == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == smartcard) {
        HAL_DEBUGE("parameter [smartcard] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* lock smartcard */
    HAL_LOCK(smartcard);

    /* disable the TBE and TC interrupt */
    hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_TBE);
    hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_TC);

    /* check receive state, if free then disable ERR interrupt */
    if(SMARTCARD_STATE_FREE == smartcard->rx_state) {
        hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_ERR);
    }

    /* disable DMA transimt and stop DMA */
    hals_smartcard_dma_transmit_config(smartcard->periph, USART_TRANSMIT_DMA_DISABLE);
    hal_dma_stop(smartcard->p_dma_tx);

    /* reset the position and state */
    smartcard->txbuffer.pos = 0U;
    smartcard->tx_state = SMARTCARD_STATE_FREE;

    /* clear interrupt error flags */
    hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_FERR);

    /* unlock smartcard */
    HAL_UNLOCK(smartcard);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop smartcard receive transfer
                the function is blocking
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_smartcard_receive_stop(hal_smartcard_dev_struct *smartcard)
{
#if (1U == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == smartcard) {
        HAL_DEBUGE("parameter [smartcard] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* lock smartcard */
    HAL_LOCK(smartcard);

    /* disable the RBNE, PERR, ERR, EB and RT interrupt */
    hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_RBNE);
    hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_PERR);
    hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_EB);
    hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_RT);

    /* check transmit state, if free then disable ERR interrupt */
    if(SMARTCARD_STATE_FREE == smartcard->tx_state) {
        hals_smartcard_interrupt_disable(smartcard->periph, USART_INT_ERR);
    }

    /* disable DMA receive and stop DMA */
    hals_smartcard_dma_receive_config(smartcard->periph, USART_RECEIVE_DMA_DISABLE);
    hal_dma_stop(smartcard->p_dma_rx);

    /* reset the position and state */
    smartcard->rxbuffer.pos = 0U;
    smartcard->rx_state = SMARTCARD_STATE_FREE;

    /* clear interrupt error flags */
    hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_PERR);
    hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_FERR);
    hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_NERR);
    hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_ORERR);
    hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_EB);
    hals_smartcard_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_RT);

    /* unlock smartcard */
    HAL_UNLOCK(smartcard);

    return HAL_ERR_NONE;
}

/*!
    \brief      reset SMARTCARD
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_smartcard_deinit(uint32_t smartcard_periph)
{
    switch(smartcard_periph) {
    case USART0:
        /* reset USART0 */
        hal_rcu_periph_reset_enable(RCU_USART0RST);
        hal_rcu_periph_reset_disable(RCU_USART0RST);
        break;
    case USART1:
        /* reset USART1 */
        hal_rcu_periph_reset_enable(RCU_USART1RST);
        hal_rcu_periph_reset_disable(RCU_USART1RST);
        break;
    default:
        break;
    }
}

/*!
    \brief      configure SMARTCARD baud rate value
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  baudval: baud rate value
    \param[out] none
    \retval     none
*/
void hals_smartcard_baudrate_set(uint32_t smartcard_periph, uint32_t baudval)
{
    uint32_t uclk = 0U, intdiv = 0U, fradiv = 0U, udiv = 0U;
    switch(smartcard_periph) {
    /* get clock frequency */
    case USART0:
        /* get USART0 clock */
        uclk = hals_rcu_clock_freq_get(CK_USART);
        break;
    case USART1:
        /* get USART1 clock */
        uclk = hals_rcu_clock_freq_get(CK_APB1);
        break;
    default:
        break;
    }
    if(USART_CTL0(smartcard_periph) & USART_CTL0_OVSMOD) {
        /* oversampling by 8, configure the value of USART_BAUD */
        udiv = ((2U * uclk) + (baudval / 2U)) / baudval;
        intdiv = udiv & 0x0000fff0U;
        fradiv = (udiv >> 1U) & 0x00000007U;
        USART_BAUD(smartcard_periph) = ((USART_BAUD_FRADIV | USART_BAUD_INTDIV) & (intdiv | fradiv));
    } else {
        /* oversampling by 16, configure the value of USART_BAUD */
        udiv = (uclk + (baudval / 2U)) / baudval;
        intdiv = udiv & 0x0000fff0U;
        fradiv = udiv & 0x0000000fU;
        USART_BAUD(smartcard_periph) = ((USART_BAUD_FRADIV | USART_BAUD_INTDIV) & (intdiv | fradiv));
    }
}

/*!
    \brief      configure SMARTCARD parity
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  paritycfg: SMARTCARD parity configure
                only one parameter can be selected which is shown as below:
      \arg        USART_PM_NONE: no parity
      \arg        USART_PM_ODD: odd parity
      \arg        USART_PM_EVEN: even parity
    \param[out] none
    \retval     none
*/
void hals_smartcard_parity_config(uint32_t smartcard_periph, uint32_t paritycfg)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
    /* clear USART_CTL0 PM,PCEN bits */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_PM | USART_CTL0_PCEN);
    /* configure SMARTCARD parity mode */
    USART_CTL0(smartcard_periph) |= paritycfg;
}

/*!
    \brief      configure SMARTCARD word length
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  wlen: SMARTCARD word length configure
                only one parameter can be selected which is shown as below:
      \arg        USART_WL_8BIT: 8 bits
      \arg        USART_WL_9BIT: 9 bits
    \param[out] none
    \retval     none
*/
void hals_smartcard_word_length_set(uint32_t smartcard_periph, uint32_t wlen)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
    /* clear USART_CTL0 WL bit */
    USART_CTL0(smartcard_periph) &= ~USART_CTL0_WL;
    /* configure SMARTCARD word length */
    USART_CTL0(smartcard_periph) |= wlen;
}

/*!
    \brief      configure SMARTCARD stop bit length
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  stblen: SMARTCARD stop bit configure
                only one parameter can be selected which is shown as below:
      \arg        USART_STB_1BIT: 1 bit
      \arg        USART_STB_0_5BIT: 0.5bit
      \arg        USART_STB_2BIT: 2 bits
      \arg        USART_STB_1_5BIT: 1.5bit
    \param[out] none
    \retval     none
*/
void hals_smartcard_stop_bit_set(uint32_t smartcard_periph, uint32_t stblen)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
    /* clear USART_CTL1 STB bits */
    USART_CTL1(smartcard_periph) &= ~USART_CTL1_STB;
    USART_CTL1(smartcard_periph) |= stblen;
}

/*!
    \brief      enable SMARTCARD
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_smartcard_enable(uint32_t smartcard_periph)
{
    USART_CTL0(smartcard_periph) |= USART_CTL0_UEN;
}

/*!
    \brief      disable SMARTCARD
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_smartcard_disable(uint32_t smartcard_periph)
{
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
}

/*!
    \brief      configure SMARTCARD transmitter
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  txconfig: enable or disable SMARTCARD transmitter
                only one parameter can be selected which is shown as below:
      \arg        USART_TRANSMIT_ENABLE: enable SMARTCARD transmission
      \arg        USART_TRANSMIT_DISABLE: enable SMARTCARD transmission
    \param[out] none
    \retval     none
*/
void hals_smartcard_transmit_config(uint32_t smartcard_periph, uint32_t txconfig)
{
    USART_CTL0(smartcard_periph) &= ~USART_CTL0_TEN;
    /* configure transfer mode */
    USART_CTL0(smartcard_periph) |= txconfig;
}

/*!
    \brief      configure SMARTCARD receiver
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  rxconfig: enable or disable SMARTCARD receiver
                only one parameter can be selected which is shown as below:
      \arg        USART_RECEIVE_ENABLE: enable SMARTCARD reception
      \arg        USART_RECEIVE_DISABLE: disable SMARTCARD reception
    \param[out] none
    \retval     none
*/
void hals_smartcard_receive_config(uint32_t smartcard_periph, uint32_t rxconfig)
{
    USART_CTL0(smartcard_periph) &= ~USART_CTL0_REN;
    /* configure receiver mode */
    USART_CTL0(smartcard_periph) |= rxconfig;
}

/*!
    \brief      data is transmitted/received with the LSB/MSB first
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  msbf: LSB/MSB
                only one parameter can be selected which is shown as below:
      \arg        USART_MSBF_LSB: LSB first
      \arg        USART_MSBF_MSB: MSB first
    \param[out] none
    \retval     none
*/
void hals_smartcard_data_first_config(uint32_t smartcard_periph, uint32_t msbf)
{
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
    /* configure LSB or MSB first */
    USART_CTL1(smartcard_periph) &= ~(USART_CTL1_MSBF);
    USART_CTL1(smartcard_periph) |= msbf;
}

/*!
    \brief      configure SMARTCARD inversion
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  invertpara: refer to usart_invert_enum
                only one parameter can be selected which is shown as below:
      \arg        USART_DINV_ENABLE: data bit level inversion
      \arg        USART_DINV_DISABLE: data bit level not inversion
      \arg        USART_TXPIN_ENABLE: TX pin level inversion
      \arg        USART_TXPIN_DISABLE: TX pin level not inversion
      \arg        USART_RXPIN_ENABLE: RX pin level inversion
      \arg        USART_RXPIN_DISABLE: RX pin level not inversion
      \arg        USART_SWAP_ENABLE: swap TX/RX pins
      \arg        USART_SWAP_DISABLE: not swap TX/RX pins
    \param[out] none
    \retval     none
*/
void hals_smartcard_invert_config(uint32_t smartcard_periph, usart_invert_enum invertpara)
{
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
    /* inverted or not the specified signal */
    switch(invertpara) {
    case USART_DINV_ENABLE:
        USART_CTL1(smartcard_periph) |= USART_CTL1_DINV;
        break;
    case USART_DINV_DISABLE:
        USART_CTL1(smartcard_periph) &= ~(USART_CTL1_DINV);
        break;
    case USART_TXPIN_ENABLE:
        USART_CTL1(smartcard_periph) |= USART_CTL1_TINV;
        break;
    case USART_TXPIN_DISABLE:
        USART_CTL1(smartcard_periph) &= ~(USART_CTL1_TINV);
        break;
    case USART_RXPIN_ENABLE:
        USART_CTL1(smartcard_periph) |= USART_CTL1_RINV;
        break;
    case USART_RXPIN_DISABLE:
        USART_CTL1(smartcard_periph) &= ~(USART_CTL1_RINV);
        break;
    case USART_SWAP_ENABLE:
        USART_CTL1(smartcard_periph) |= USART_CTL1_STRP;
        break;
    case USART_SWAP_DISABLE:
        USART_CTL1(smartcard_periph) &= ~(USART_CTL1_STRP);
        break;
    default:
        break;
    }
}

/*!
    \brief      enable the SMARTCARD overrun function
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_smartcard_overrun_enable(uint32_t smartcard_periph)
{
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
    /* enable overrun function */
    USART_CTL2(smartcard_periph) &= ~(USART_CTL2_OVRD);
}

/*!
    \brief      disable the SMARTCARD overrun function
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_smartcard_overrun_disable(uint32_t smartcard_periph)
{
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
    /* disable overrun function */
    USART_CTL2(smartcard_periph) |= USART_CTL2_OVRD;
}

/*!
    \brief      configure the SMARTCARD oversample mode
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  oversamp: oversample value
                only one parameter can be selected which is shown as below:
      \arg        USART_OVSMOD_8: oversampling by 8
      \arg        USART_OVSMOD_16: oversampling by 16
    \param[out] none
    \retval     none
*/
void hals_smartcard_oversample_config(uint32_t smartcard_periph, uint32_t oversamp)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
    /* clear OVSMOD bit */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_OVSMOD);
    USART_CTL0(smartcard_periph) |= oversamp;
}

/*!
    \brief      configure the sample bit method
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  osb: sample bit
                only one parameter can be selected which is shown as below:
      \arg        USART_OSB_1BIT: 1 bit
      \arg        USART_OSB_3BIT: 3 bits
    \param[out] none
    \retval     none
*/
void hals_smartcard_sample_bit_config(uint32_t smartcard_periph, uint32_t osb)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(smartcard_periph) &= ~(USART_CTL2_OSB);
    USART_CTL2(smartcard_periph) |= osb;
}

/*!
    \brief      enable receiver timeout
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_smartcard_receiver_timeout_enable(uint32_t smartcard_periph)
{
    USART_CTL1(smartcard_periph) |= USART_CTL1_RTEN;
}

/*!
    \brief      disable receiver timeout
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_smartcard_receiver_timeout_disable(uint32_t smartcard_periph)
{
    USART_CTL1(smartcard_periph) &= ~(USART_CTL1_RTEN);
}

/*!
    \brief      configure receiver timeout threshold
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[in]  rtimeout: 0x00000000-0x00FFFFFF, receiver timeout value in terms of number of baud clocks
    \param[out] none
    \retval     none
*/
void hals_smartcard_receiver_timeout_threshold_config(uint32_t smartcard_periph, uint32_t rtimeout)
{
    USART_RT(smartcard_periph) &= ~(USART_RT_RT);
    USART_RT(smartcard_periph) |= rtimeout;
}

/*!
    \brief      SMARTCARD transmit data function
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  data: data of transmission
    \param[out] none
    \retval     none
*/
void hals_smartcard_data_transmit(uint32_t smartcard_periph, uint16_t data)
{
    USART_TDATA(smartcard_periph) = (USART_TDATA_TDATA & (uint32_t)data);
}

/*!
    \brief      SMARTCARD receive data function
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[out] none
    \retval     data of received
*/
uint16_t hals_smartcard_data_receive(uint32_t smartcard_periph)
{
    return (uint16_t)(GET_BITS(USART_RDATA(smartcard_periph), 0U, 8U));
}

/*!
    \brief      enable SMARTCARD command
    \param[in]  smartcard_periph: USARTx(x=0,1,2)
    \param[in]  cmdtype: command type
                only one parameter can be selected which is shown as below:
      \arg        USART_CMD_SBKCMD: send break command
      \arg        USART_CMD_MMCMD: mute mode command
      \arg        USART_CMD_RXFCMD: receive data flush command
      \arg        USART_CMD_TXFCMD: transmit data flush request
    \param[out] none
    \retval     none
*/
void hals_smartcard_command_enable(uint32_t smartcard_periph, uint32_t cmdtype)
{
    USART_CMD(smartcard_periph) |= (cmdtype);
}

/*!
    \brief      enable SMARTCARD clock
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_smartcard_clock_enable(uint32_t smartcard_periph)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);

    USART_CTL1(smartcard_periph) |= USART_CTL1_CKEN;
}

/*!
    \brief      disable SMARTCARD clock
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_smartcard_clock_disable(uint32_t smartcard_periph)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);

    USART_CTL1(smartcard_periph) &= ~(USART_CTL1_CKEN);
}

/*!
    \brief      configure SMARTCARD synchronous mode parameters
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  clen: last bit clock pulse
                only one parameter can be selected which is shown as below:
      \arg        USART_CLEN_NONE: clock pulse of the last data bit (MSB) is not output to the CK pin
      \arg        USART_CLEN_EN: clock pulse of the last data bit (MSB) is output to the CK pin
    \param[in]  cph: clock phase
                only one parameter can be selected which is shown as below:
      \arg        USART_CPH_1CK: first clock transition is the first data capture edge
      \arg        USART_CPH_2CK: second clock transition is the first data capture edge
    \param[in]  cpl: clock polarity
                only one parameter can be selected which is shown as below:
      \arg        USART_CPL_LOW: steady low value on CK pin
      \arg        USART_CPL_HIGH: steady high value on CK pin
    \param[out] none
    \retval     none
*/
void hals_smartcard_synchronous_clock_config(uint32_t smartcard_periph, uint32_t clen, uint32_t cph,
        uint32_t cpl)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
    /* reset USART_CTL1 CLEN,CPH,CPL bits */
    USART_CTL1(smartcard_periph) &= ~(USART_CTL1_CLEN | USART_CTL1_CPH | USART_CTL1_CPL);

    USART_CTL1(smartcard_periph) |= (USART_CTL1_CLEN & clen);
    USART_CTL1(smartcard_periph) |= (USART_CTL1_CPH & cph);
    USART_CTL1(smartcard_periph) |= (USART_CTL1_CPL & cpl);
}

/*!
    \brief      configure guard time value in smartcard mode
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[in]  guat: 0x00-0xFF
    \param[out] none
    \retval     none
*/
void hals_smartcard_guard_time_config(uint32_t smartcard_periph, uint32_t guat)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);

    USART_GP(smartcard_periph) &= ~(USART_GP_GUAT);
    USART_GP(smartcard_periph) |= (USART_GP_GUAT & ((guat) << GP_GUAT_OFFSET));
}

/*!
    \brief      enable smartcard mode
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_smartcard_mode_enable(uint32_t smartcard_periph)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(smartcard_periph) |= USART_CTL2_SCEN;
}

/*!
    \brief      disable smartcard mode
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_smartcard_mode_disable(uint32_t smartcard_periph)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(smartcard_periph) &= ~(USART_CTL2_SCEN);
}

/*!
    \brief      enable NACK in smartcard mode
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_smartcard_mode_nack_enable(uint32_t smartcard_periph)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(smartcard_periph) |= USART_CTL2_NKEN;
}

/*!
    \brief      disable NACK in smartcard mode
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_smartcard_mode_nack_disable(uint32_t smartcard_periph)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(smartcard_periph) &= ~(USART_CTL2_NKEN);
}

/*!
    \brief      enable early NACK in smartcard mode
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_smartcard_mode_early_nack_enable(uint32_t smartcard_periph)
{
    USART_RFCS(smartcard_periph) |= USART_RFCS_ELNACK;
}

/*!
    \brief      disable early NACK in smartcard mode
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_smartcard_mode_early_nack_disable(uint32_t smartcard_periph)
{
    USART_RFCS(smartcard_periph) &= ~USART_RFCS_ELNACK;
}

/*!
    \brief      configure smartcard auto-retry number
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[in]  scrtnum: 0x00000000-0x00000007, smartcard auto-retry number
    \param[out] none
    \retval     none
*/
void hals_smartcard_autoretry_config(uint32_t smartcard_periph, uint32_t scrtnum)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
    USART_CTL2(smartcard_periph) &= ~(USART_CTL2_SCRTNUM);
    USART_CTL2(smartcard_periph) |= (USART_CTL2_SCRTNUM & (scrtnum << CTL2_SCRTNUM_OFFSET));
}

/*!
    \brief      configure block length
    \param[in]  smartcard_periph: USARTx(x=0)
    \param[in]  bl: 0x00000000-0x000000FF
    \param[out] none
    \retval     none
*/
void hals_smartcard_block_length_config(uint32_t smartcard_periph, uint32_t bl)
{
    USART_RT(smartcard_periph) &= ~(USART_RT_BL);
    USART_RT(smartcard_periph) |= (USART_RT_BL & ((bl) << RT_BL_OFFSET));
}

/*!
    \brief      configure SMARTCARD DMA reception
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  dmacmd: SMARTCARD DMA mode
                only one parameter can be selected which is shown as below:
      \arg        USART_RECEIVE_DMA_ENABLE: enable SMARTCARD DMA for reception
      \arg        USART_RECEIVE_DMA_DISABLE: disable SMARTCARD DMA for reception
    \param[out] none
    \retval     none
*/
void hals_smartcard_dma_receive_config(uint32_t smartcard_periph, uint8_t dmacmd)
{
    USART_CTL2(smartcard_periph) &= ~USART_CTL2_DENR;
    USART_CTL2(smartcard_periph) |= (USART_CTL2_DENR & dmacmd);
}

/*!
    \brief      configure SMARTCARD DMA transmission
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  dmacmd: SMARTCARD DMA mode
                only one parameter can be selected which is shown as below:
      \arg        USART_TRANSMIT_DMA_ENABLE: enable SMARTCARD DMA for transmission
      \arg        USART_TRANSMIT_DMA_DISABLE: disable SMARTCARD DMA for transmission
    \param[out] none
    \retval     none
*/
void hals_smartcard_dma_transmit_config(uint32_t smartcard_periph, uint8_t dmacmd)
{
    USART_CTL2(smartcard_periph) &= ~USART_CTL2_DENT;
    USART_CTL2(smartcard_periph) |= (USART_CTL2_DENT & dmacmd);
}

/*!
    \brief      enable DMA on reception error
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_smartcard_reception_error_dma_enable(uint32_t smartcard_periph)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
    USART_CTL2(smartcard_periph) &= ~(USART_CTL2_DDRE);
}

/*!
    \brief      disable DMA on reception error
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_smartcard_reception_error_dma_disable(uint32_t smartcard_periph)
{
    /* disable SMARTCARD */
    USART_CTL0(smartcard_periph) &= ~(USART_CTL0_UEN);
    USART_CTL2(smartcard_periph) |= USART_CTL2_DDRE;
}

/*!
    \brief      enable receive FIFO
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_smartcard_receive_fifo_enable(uint32_t smartcard_periph)
{
    USART_RFCS(smartcard_periph) |= USART_RFCS_RFEN;
}

/*!
    \brief      disable receive FIFO
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_smartcard_receive_fifo_disable(uint32_t smartcard_periph)
{
    USART_RFCS(smartcard_periph) &= ~(USART_RFCS_RFEN);
}

/*!
    \brief      read receive FIFO counter number
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[out] none
    \retval     receive FIFO counter number
*/
uint8_t hals_smartcard_receive_fifo_counter_number(uint32_t smartcard_periph)
{
    return (uint8_t)(GET_BITS(USART_RFCS(smartcard_periph), 12U, 14U));
}

/*!
    \brief      get SMARTCARD status
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  flag: flag type
                only one parameter can be selected which is shown as below:
      \arg        USART_FLAG_PERR: parity error flag
      \arg        USART_FLAG_FERR: frame error flag
      \arg        USART_FLAG_NERR: noise error flag
      \arg        USART_FLAG_ORERR: overrun error flag
      \arg        USART_FLAG_IDLE: idle line detected flag
      \arg        USART_FLAG_RBNE: read data buffer not empty
      \arg        USART_FLAG_TC: transmission complete flag
      \arg        USART_FLAG_TBE: transmit data register empty
      \arg        USART_FLAG_LBD: LIN break detected flag
      \arg        USART_FLAG_CTSF: CTS change flag
      \arg        USART_FLAG_CTS: CTS level
      \arg        USART_FLAG_RT: receiver timeout flag
      \arg        USART_FLAG_EB: end of block flag
      \arg        USART_FLAG_BSY: busy flag
      \arg        USART_FLAG_AM: address match flag
      \arg        USART_FLAG_SB: send break flag
      \arg        USART_FLAG_RWU: receiver wakeup from mute mode.
      \arg        USART_FLAG_WU: wakeup from deep-sleep mode flag
      \arg        USART_FLAG_TEA: transmit enable acknowledge flag
      \arg        USART_FLAG_REA: receive enable acknowledge flag
      \arg        USART_FLAG_EPERR: early parity error flag
      \arg        USART_FLAG_RFE: receive FIFO empty flag
      \arg        USART_FLAG_RFF: receive FIFO full flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_smartcard_flag_get(uint32_t smartcard_periph, usart_flag_enum flag)
{
    if(RESET != (USART_REG_VAL(smartcard_periph, flag) & BIT(USART_BIT_POS(flag)))) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear SMARTCARD status
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  flag: flag type
                only one parameter can be selected which is shown as below:
      \arg        USART_FLAG_PERR: parity error flag
      \arg        USART_FLAG_FERR: frame error flag
      \arg        USART_FLAG_NERR: noise error flag
      \arg        USART_FLAG_ORERR: overrun error flag
      \arg        USART_FLAG_IDLE: idle line detected flag
      \arg        USART_FLAG_TC: transmission complete flag
      \arg        USART_FLAG_LBD: LIN break detected flag
      \arg        USART_FLAG_CTSF: CTS change flag
      \arg        USART_FLAG_RT: receiver timeout flag
      \arg        USART_FLAG_EB: end of block flag
      \arg        USART_FLAG_AM: address match flag
      \arg        USART_FLAG_WU: wakeup from deep-sleep mode flag
      \arg        USART_FLAG_EPERR: early parity error flag
    \param[out] none
    \retval     none
*/
void hals_smartcard_flag_clear(uint32_t smartcard_periph, usart_flag_enum flag)
{
    USART_INTC(smartcard_periph) |= BIT(USART_BIT_POS(flag));
}

/*!
    \brief      enable SMARTCARD interrupt
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  interrupt: interrupt
                only one parameter can be selected which is shown as below:
      \arg        USART_INT_IDLE: idle line detected interrupt
      \arg        USART_INT_RBNE: read data buffer not empty interrupt and overrun error interrupt
      \arg        USART_INT_TC: transmission complete interrupt
      \arg        USART_INT_TBE: transmitter buffer empty interrupt
      \arg        USART_INT_PERR: parity error interrupt
      \arg        USART_INT_AM: address match interrupt
      \arg        USART_INT_RT: receiver timeout interrupt
      \arg        USART_INT_EB: end of block interrupt
      \arg        USART_INT_LBD: LIN break detection interrupt
      \arg        USART_INT_ERR: error interrupt
      \arg        USART_INT_CTS: CTS interrupt
      \arg        USART_INT_WU: wakeup from deep-sleep mode interrupt
      \arg        USART_INT_RFF: receive FIFO full interrupt
    \param[out] none
    \retval     none
*/
void hals_smartcard_interrupt_enable(uint32_t smartcard_periph, usart_interrupt_enum interrupt)
{
    USART_REG_VAL(smartcard_periph, interrupt) |= BIT(USART_BIT_POS(interrupt));
}

/*!
    \brief      disable SMARTCARD interrupt
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  interrupt: interrupt
                only one parameter can be selected which is shown as below:
      \arg        USART_INT_IDLE: idle interrupt
      \arg        USART_INT_RBNE: read data buffer not empty interrupt and overrun error interrupt
      \arg        USART_INT_TC: transmission complete interrupt
      \arg        USART_INT_TBE: transmitter buffer empty interrupt
      \arg        USART_INT_PERR: parity error interrupt
      \arg        USART_INT_AM: address match interrupt
      \arg        USART_INT_RT: receiver timeout interrupt
      \arg        USART_INT_EB: end of block interrupt
      \arg        USART_INT_LBD: LIN break detection interrupt
      \arg        USART_INT_ERR: error interrupt
      \arg        USART_INT_CTS: CTS interrupt
      \arg        USART_INT_WU: wakeup from deep-sleep mode interrupt
      \arg        USART_INT_RFF: receive FIFO full interrupt
    \param[out] none
    \retval     none
*/
void hals_smartcard_interrupt_disable(uint32_t smartcard_periph, usart_interrupt_enum interrupt)
{
    USART_REG_VAL(smartcard_periph, interrupt) &= ~BIT(USART_BIT_POS(interrupt));
}

/*!
    \brief      get SMARTCARD interrupt flag status
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  int_flag: interrupt and flag type, refer to usart_interrupt_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        USART_INT_FLAG_EB: end of block interrupt flag
      \arg        USART_INT_FLAG_RT: receiver timeout interrupt flag
      \arg        USART_INT_FLAG_AM: address match interrupt flag
      \arg        USART_INT_FLAG_PERR: parity error interrupt flag
      \arg        USART_INT_FLAG_TBE: transmitter buffer empty interrupt flag
      \arg        USART_INT_FLAG_TC: transmission complete interrupt flag
      \arg        USART_INT_FLAG_RBNE: read data buffer not empty interrupt flag
      \arg        USART_INT_FLAG_RBNE_ORERR: overrun error interrupt flag
      \arg        USART_INT_FLAG_IDLE: idle line detected interrupt flag
      \arg        USART_INT_FLAG_LBD: LIN break detected interrupt flag
      \arg        USART_INT_FLAG_WU: wakeup from deep-sleep mode interrupt flag
      \arg        USART_INT_FLAG_CTS: CTS interrupt flag
      \arg        USART_INT_FLAG_ERR_NERR: noise error interrupt flag
      \arg        USART_INT_FLAG_ERR_ORERR: overrun error interrupt flag
      \arg        USART_INT_FLAG_ERR_FERR: frame error interrupt flag
      \arg        USART_INT_FLAG_RFF: receive FIFO full interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_smartcard_interrupt_flag_get(uint32_t smartcard_periph,
        usart_interrupt_flag_enum int_flag)
{
    uint32_t intenable = 0U, flagstatus = 0U;
    /* get the interrupt enable bit status */
    intenable = (USART_REG_VAL(smartcard_periph, int_flag) & BIT(USART_BIT_POS(int_flag)));
    /* get the corresponding flag bit status */
    flagstatus = (USART_REG_VAL2(smartcard_periph, int_flag) & BIT(USART_BIT_POS2(int_flag)));

    if(flagstatus && intenable) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear SMARTCARD interrupt flag
    \param[in]  smartcard_periph: USARTx(x=0,1)
    \param[in]  flag: SMARTCARD interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        USART_INT_FLAG_EB: end of block interrupt flag
      \arg        USART_INT_FLAG_RT: receiver timeout interrupt flag
      \arg        USART_INT_FLAG_AM: address match interrupt flag
      \arg        USART_INT_FLAG_PERR: parity error interrupt flag
      \arg        USART_INT_FLAG_TC: transmission complete interrupt flag
      \arg        USART_INT_FLAG_RBNE_ORERR: overrun error interrupt flag
      \arg        USART_INT_FLAG_IDLE: idle line detected interrupt flag
      \arg        USART_INT_FLAG_LBD: LIN break detected interrupt flag
      \arg        USART_INT_FLAG_WU: wakeup from deep-sleep mode interrupt flag
      \arg        USART_INT_FLAG_CTS: CTS change interrupt flag
      \arg        USART_INT_FLAG_ERR_FERR: frame error interrupt flag
      \arg        USART_INT_FLAG_ERR_NERR: noise detected interrupt flag
      \arg        USART_INT_FLAG_ERR_ORERR: overrun error interrupt flag
      \arg        USART_INT_FLAG_RFF: receive FIFO full interrupt flag
    \param[out] none
    \retval     none
*/
void hals_smartcard_interrupt_flag_clear(uint32_t smartcard_periph,
        usart_interrupt_flag_enum int_flag)
{
    if(USART_INT_FLAG_RFF == int_flag) {
        USART_RFCS(smartcard_periph) &= (uint32_t)(~USART_RFCS_RFFINT);
    } else {
        USART_INTC(smartcard_periph) |= BIT(USART_BIT_POS2(int_flag));
    }
}

/*!
    \brief      get smartcard error status
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error status: RESET or SET
*/
static FlagStatus _smartcard_error_flag_get(hal_smartcard_dev_struct *smartcard)
{
    if(0U == (USART_STAT(smartcard->periph) & (uint32_t)(USART_STAT_PERR | USART_STAT_FERR | \
              USART_STAT_ORERR | USART_STAT_NERR | USART_STAT_RTF))) {
        return RESET;
    } else {
        return SET;
    }
}

/*!
    \brief      smartcard transmit complete interrupt handler
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _smartcard_transmit_complete_interrupt(void *smartcard)
{
    hal_smartcard_dev_struct *p_smartcard = smartcard;
    hal_smartcard_user_cb p_func = (hal_smartcard_user_cb)p_smartcard->tx_callback;

    /* disable usart transmit complete interrupt */
    hals_smartcard_interrupt_disable(p_smartcard->periph, USART_INT_TC);

    /* check receive state, if free then disable ERR interrupt */
    if(SMARTCARD_STATE_FREE == p_smartcard->rx_state) {
        hals_smartcard_interrupt_disable(p_smartcard->periph, USART_INT_ERR);
    }

    /* clear transmit complete callback pointer */
    p_smartcard->smartcard_irq.transmit_complete_handle = NULL;

    /* re-enable receive */
    hals_smartcard_receive_config(p_smartcard->periph, USART_RECEIVE_ENABLE);

    /* change the Tx state to free */
    p_smartcard->tx_state = SMARTCARD_STATE_FREE;

    if(NULL != p_func) {
        p_func(p_smartcard);
    }
}

/*!
    \brief      smartcard transmit interrupt handler
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _smartcard_transmit_interrupt(void *smartcard)
{
    hal_smartcard_dev_struct *p_smartcard = smartcard;
    uint32_t length = p_smartcard->txbuffer.length;

    /* Check the remain data to be transmitted */
    if(p_smartcard->txbuffer.pos < length) {
        hals_smartcard_data_transmit(p_smartcard->periph, (*p_smartcard->txbuffer.buffer & (uint8_t)0xFFU));
        p_smartcard->txbuffer.buffer++;
        p_smartcard->txbuffer.pos++;
    } else {
        /* disable TBE interrupt and enable TC interrupt */
        hals_smartcard_interrupt_disable(p_smartcard->periph, USART_INT_TBE);
        hals_smartcard_interrupt_enable(p_smartcard->periph, USART_INT_TC);

        p_smartcard->smartcard_irq.transmit_ready_handle = NULL;
    }
}

/*!
    \brief      smartcard receive interrupt handler
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _smartcard_receive_interrupt(void *smartcard)
{
    hal_smartcard_dev_struct *p_smartcard = smartcard;
    uint32_t length = p_smartcard->rxbuffer.length;

    /* read data from data register */
    *p_smartcard->rxbuffer.buffer = (uint8_t)(hals_smartcard_data_receive(p_smartcard->periph) & 0xFFU);
    p_smartcard->rxbuffer.buffer++;
    p_smartcard->rxbuffer.pos++;

    if(p_smartcard->rxbuffer.pos == length) {
        hal_smartcard_user_cb p_func = (hal_smartcard_user_cb)p_smartcard->rx_callback;

        /* disable PERR and RBNE interrupt */
        hals_smartcard_interrupt_disable(p_smartcard->periph, USART_INT_PERR);
        hals_smartcard_interrupt_disable(p_smartcard->periph, USART_INT_RBNE);

        /* check transmit state, if free then disable ERR interrupt */
        if(SMARTCARD_STATE_FREE == p_smartcard->tx_state) {
            hals_smartcard_interrupt_disable(p_smartcard->periph, USART_INT_ERR);
        }

        /* reset receive_complete_handle */
        p_smartcard->smartcard_irq.receive_complete_handle = NULL;

        /* change the Rx state to free */
        p_smartcard->rx_state = SMARTCARD_STATE_FREE;

        if(NULL != p_func) {
            p_func(p_smartcard);
        }
    }
}

/*!
    \brief      smartcard DMA transmit handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _smartcard_transmit_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_smartcard_dev_struct *p_smartcard;

    p_dma = (hal_dma_dev_struct *)dma;
    p_smartcard = (hal_smartcard_dev_struct *)p_dma->p_periph;

    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        p_smartcard->txbuffer.pos = p_smartcard->txbuffer.length;

        /* disable the dma transfer for transmit request */
        hals_smartcard_dma_transmit_config(p_smartcard->periph, USART_TRANSMIT_DMA_DISABLE);

        /* enable TC interrupt */
        hals_smartcard_interrupt_enable(p_smartcard->periph, USART_INT_TC);
    }
}

/*!
    \brief      smartcard DMA receive handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _smartcard_receive_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_smartcard_dev_struct *p_smartcard;
    hal_smartcard_user_cb p_func;

    p_dma = (hal_dma_dev_struct *)dma;
    p_smartcard = (hal_smartcard_dev_struct *)p_dma->p_periph;
    p_func = (hal_smartcard_user_cb)p_smartcard->rx_callback;

    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        p_smartcard->rxbuffer.pos = p_smartcard->rxbuffer.length;

        /* disable the dma transfer for the receiver request */
        hals_smartcard_dma_receive_config(p_smartcard->periph, USART_RECEIVE_DMA_DISABLE);

        /* disable PERR and ERR(frame error, noise error, overrun error) interrupts */
        hals_smartcard_interrupt_disable(p_smartcard->periph, USART_INT_PERR);
        hals_smartcard_interrupt_disable(p_smartcard->periph, USART_INT_ERR);

        /* change the Rx state to free */
        p_smartcard->rx_state = SMARTCARD_STATE_FREE;
    }

    if(NULL != p_func) {
        p_func(p_smartcard);
    }
}

/*!
    \brief      smartcard dma communication error
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _smartcard_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_smartcard_dev_struct *p_smartcard;

    p_dma = (hal_dma_dev_struct *)dma;
    p_smartcard = (hal_smartcard_dev_struct *)p_dma->p_periph;

    if(SMARTCARD_STATE_BUSY == p_smartcard->tx_state) {
        p_smartcard->error_state |= HAL_SMARTCARD_ERROR_DMATX;
        p_smartcard->last_error = HAL_SMARTCARD_ERROR_DMATX;
        p_smartcard->txbuffer.pos = p_smartcard->txbuffer.length;

        /* disable the dma transmit */
        hals_smartcard_dma_transmit_config(p_smartcard->periph, USART_TRANSMIT_DMA_DISABLE);

        /* disable and ERR(frame error) interrupt */
        hals_smartcard_interrupt_disable(p_smartcard->periph, USART_INT_ERR);

        /* change the Tx state to free */
        p_smartcard->tx_state = SMARTCARD_STATE_FREE;
    } else if(SMARTCARD_STATE_BUSY == p_smartcard->rx_state) {
        p_smartcard->error_state |= HAL_SMARTCARD_ERROR_DMARX;
        p_smartcard->last_error = HAL_SMARTCARD_ERROR_DMARX;
        p_smartcard->rxbuffer.pos = p_smartcard->rxbuffer.length;

        /* disable the dma receive */
        hals_smartcard_dma_receive_config(p_smartcard->periph, USART_RECEIVE_DMA_DISABLE);

        /* disable PERR and ERR(frame error, noise error, overrun error) interrupts */
        hals_smartcard_interrupt_disable(p_smartcard->periph, USART_INT_PERR);
        hals_smartcard_interrupt_disable(p_smartcard->periph, USART_INT_ERR);

        /* change the Rx state to free */
        p_smartcard->rx_state = SMARTCARD_STATE_FREE;
    } else {
        HAL_DEBUGE("smartcard processor fatal error: dma error exception due to run state");
    }

    if(p_smartcard->smartcard_irq.error_handle != NULL) {
        p_smartcard->smartcard_irq.error_handle(p_smartcard);

        /* change the error state to none */
        p_smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;
    }
}
