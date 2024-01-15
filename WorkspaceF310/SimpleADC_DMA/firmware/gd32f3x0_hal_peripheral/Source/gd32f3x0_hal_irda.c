/*!
    \file    gd32f3x0_hal_irda.c
    \brief   IRDA driver

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

static uint16_t _irda_data_bit_mask_get(hal_irda_dev_struct *irda);
static FlagStatus _irda_error_flag_get(hal_irda_dev_struct *irda);
static void _irda_transmit_complete_interrupt(void *irda);
static void _irda_transmit_interrupt(void *irda);
static void _irda_receive_interrupt(void *irda);
static void _irda_transmit_dma(void *dma);
static void _irda_receive_dma(void *dma);
static void _irda_dma_error(void *dma);

/*!
    \brief      initialize the irda structure with the default values
                note: this function must be called after the structure is created
    \param[in]  hal_struct_type: type of irda structure for initialization
      \arg        HAL_IRDA_INIT_STRUCT: initialization structure
      \arg        HAL_IRDA_DEV_STRUCT: device information structure
      \arg        HAL_IRDA_USER_CALLBCAK_STRUCT: user callback structure
    \param[in]  p_struct: structure pointer
    \param[out] none
    \retval     none
*/
void hal_irda_struct_init(hal_irda_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct) {
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_IRDA_INIT_STRUCT:
        /* initialize irda initialization structure with the default values */
        ((hal_irda_init_struct *)p_struct)->baudrate     = 115200U;
        ((hal_irda_init_struct *)p_struct)->parity       = IRDA_PARITY_NONE;
        ((hal_irda_init_struct *)p_struct)->word_length  = IRDA_WORD_LENGTH_8BIT;
        ((hal_irda_init_struct *)p_struct)->direction    = IRDA_DIRECTION_RX_TX;
        ((hal_irda_init_struct *)p_struct)->rx_fifo_en   = DISABLE;
        ((hal_irda_init_struct *)p_struct)->mode         = IRDA_NORMAL_MODE;
        ((hal_irda_init_struct *)p_struct)->prescaler    = 1U;
        break;

    case HAL_IRDA_DEV_STRUCT:
        /* initialize irda device information structure with the default values */
        ((hal_irda_dev_struct *)p_struct)->periph = 0U;
        ((hal_irda_dev_struct *)p_struct)->irda_irq.error_handle = NULL;
        ((hal_irda_dev_struct *)p_struct)->irda_irq.receive_complete_handle = NULL;
        ((hal_irda_dev_struct *)p_struct)->irda_irq.transmit_complete_handle = NULL;
        ((hal_irda_dev_struct *)p_struct)->irda_irq.transmit_ready_handle = NULL;
        ((hal_irda_dev_struct *)p_struct)->p_dma_rx          = NULL;
        ((hal_irda_dev_struct *)p_struct)->p_dma_tx          = NULL;
        ((hal_irda_dev_struct *)p_struct)->txbuffer.buffer   = NULL;
        ((hal_irda_dev_struct *)p_struct)->txbuffer.length   = 0U;
        ((hal_irda_dev_struct *)p_struct)->txbuffer.pos      = 0U;
        ((hal_irda_dev_struct *)p_struct)->rxbuffer.buffer   = NULL;
        ((hal_irda_dev_struct *)p_struct)->rxbuffer.length   = 0U;
        ((hal_irda_dev_struct *)p_struct)->rxbuffer.pos      = 0U;
        ((hal_irda_dev_struct *)p_struct)->data_bit_mask      = 0U;
        ((hal_irda_dev_struct *)p_struct)->last_error        = HAL_IRDA_ERROR_NONE;
        ((hal_irda_dev_struct *)p_struct)->error_state       = HAL_IRDA_ERROR_NONE;
        ((hal_irda_dev_struct *)p_struct)->tx_state          = IRDA_STATE_FREE;
        ((hal_irda_dev_struct *)p_struct)->rx_state          = IRDA_STATE_FREE;
        ((hal_irda_dev_struct *)p_struct)->rx_callback       = NULL;
        ((hal_irda_dev_struct *)p_struct)->tx_callback       = NULL;
        ((hal_irda_dev_struct *)p_struct)->mutex             = HAL_MUTEX_UNLOCKED;
        ((hal_irda_dev_struct *)p_struct)->priv              = NULL;

    case HAL_IRDA_USER_CALLBCAK_STRUCT:
        /* initialize user callback structure with the default values */
        ((hal_irda_user_callback_struct *)p_struct)->complete_func = NULL;
        ((hal_irda_user_callback_struct *)p_struct)->error_func = NULL;
        break;

    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize irda
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_irda_deinit(hal_irda_dev_struct *irda)
{
    uint32_t periph;

    periph = irda->periph;
    if(USART0 == periph) {
        /* deinitialize the periph and the device information sturcture */
        hals_irda_deinit(periph);
        hal_irda_struct_init(HAL_IRDA_DEV_STRUCT, irda);
        irda->periph = periph;
    } else {
        HAL_DEBUGE("parameter [irda->periph] value is invalid");
    }
}

/*!
    \brief      initialize irda
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which IRDA is initialized, USART0 only
    \param[in]  p_init: the initialization data needed to initialize irda
                  baudrate: communication baudrate
                  parity: IRDA_PARITY_NONE, IRDA_PARITY_EVEN, IRDA_PARITY_ODD
                  word_length: IRDA_WORD_LENGTH_8BIT, IRDA_WORD_LENGTH_9BIT
                  direction: IRDA_DIRECTION_RX_TX, IRDA_DIRECTION_RX_ONLY, IRDA_DIRECTION_TX_ONLY
                  mode: IRDA_NORMAL_MODE, IRDA_LOW_POWER_MODE
                  prescaler: 1 - 255
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32f3x0_hal.h
*/
int32_t hal_irda_init(hal_irda_dev_struct *irda, uint32_t periph, hal_irda_init_struct *p_init)
{
    uint32_t reg_temp;

#if (1 == HAL_PARAMETER_CHECK)
    /* check irda pointer and p_init address */
    if((NULL == irda) || (NULL == p_init)) {
        HAL_DEBUGE("pointer [irda] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check periph parameter */
    if(USART0 != periph) {
        HAL_DEBUGE("parameter [periph] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check periph value from irda device struct */
    if(0U != irda->periph) {
        HAL_DEBUGI("periph value from irda device struct has been rewrite");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock irda */
    HAL_LOCK(irda);

    irda->periph = periph;

    /* disable irda */
    hals_irda_disable(periph);

    /* configure CTL0 register */
    reg_temp = USART_CTL0(periph);
    reg_temp &= ~(USART_CTL0_PM | USART_CTL0_PCEN | USART_CTL0_WL | USART_CTL0_REN | USART_CTL0_TEN);
    reg_temp |= (p_init->direction | p_init->parity | p_init->word_length);
    USART_CTL0(periph) = reg_temp;

    /* configure GP register */
    reg_temp = USART_GP(periph);
    reg_temp &= ~USART_GP_PSC;
    reg_temp |= p_init->prescaler;
    USART_GP(periph) = reg_temp;

    /* configure RFCS register */
    if(ENABLE == p_init->rx_fifo_en) {
        USART_RFCS(periph) |= USART_RFCS_RFEN;
    } else {
        USART_RFCS(periph) &= ~USART_RFCS_RFEN;
    }

    /* configure baudrate */
    hals_irda_baudrate_set(periph, p_init->baudrate);
    irda->data_bit_mask = _irda_data_bit_mask_get(irda);

    /* reset the Rx and Tx state */
    irda->tx_state = IRDA_STATE_FREE;
    irda->rx_state = IRDA_STATE_FREE;

    /* configure irda in low power mode */
    hals_irda_lowpower_config(periph, p_init->mode);
    hals_irda_mode_enable(periph);

    /* enable usart */
    hals_irda_enable(irda->periph);

    /* unlock irda */
    HAL_UNLOCK(irda);

    return HAL_ERR_NONE;
}

/*!
    \brief      irda interrupt handler content function,which is merely used in irda_handler
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_irda_irq(hal_irda_dev_struct *irda)
{
    if(RESET == _irda_error_flag_get(irda)) {
        /* check whether IRDA is in receiver mode or not */
        if(RESET != hals_irda_interrupt_flag_get(irda->periph, USART_INT_FLAG_RBNE)) {
            if(NULL != irda->irda_irq.receive_complete_handle) {
                irda->irda_irq.receive_complete_handle(irda);
            }
            return;
        }
    } else {
        /* check whether the PERR flag is set or not */
        if(RESET != hals_irda_interrupt_flag_get(irda->periph, USART_INT_FLAG_PERR)) {
            hals_irda_interrupt_flag_clear(irda->periph, USART_INT_FLAG_PERR);
            irda->error_state |= HAL_IRDA_ERROR_PERR;
            irda->last_error = HAL_IRDA_ERROR_PERR;
        }

        /* check whether the NERR flag is set or not */
        if(RESET != hals_irda_interrupt_flag_get(irda->periph, USART_INT_FLAG_ERR_NERR)) {
            hals_irda_interrupt_flag_clear(irda->periph, USART_INT_FLAG_ERR_NERR);
            irda->error_state |= HAL_IRDA_ERROR_NERR;
            irda->last_error = HAL_IRDA_ERROR_NERR;
        }

        /* check whether the FERR flag is set or not */
        if(RESET != hals_irda_interrupt_flag_get(irda->periph, USART_INT_FLAG_ERR_FERR)) {
            hals_irda_interrupt_flag_clear(irda->periph, USART_INT_FLAG_ERR_FERR);
            irda->error_state |= HAL_IRDA_ERROR_FERR;
            irda->last_error = HAL_IRDA_ERROR_FERR;
        }

        /* check whether the ERR ORERR is set or not */
        if(RESET != hals_irda_interrupt_flag_get(irda->periph, USART_INT_FLAG_ERR_ORERR)) {
            hals_irda_interrupt_flag_clear(irda->periph, USART_INT_FLAG_ERR_ORERR);
            irda->error_state |= HAL_IRDA_ERROR_ORERR;
            irda->last_error = HAL_IRDA_ERROR_ORERR;
        }

        /* check whether RBNE ORERR is set or not */
        if(RESET != hals_irda_interrupt_flag_get(irda->periph, USART_INT_FLAG_RBNE_ORERR)) {
            hals_irda_interrupt_flag_clear(irda->periph, USART_INT_FLAG_RBNE_ORERR);
            irda->error_state |= HAL_IRDA_ERROR_ORERR;
            irda->last_error = HAL_IRDA_ERROR_ORERR;
        }

        /* check whether error state is none or not */
        if(HAL_IRDA_ERROR_NONE != irda->error_state) {
            if(NULL != irda->irda_irq.error_handle) {
                irda->irda_irq.error_handle(irda);
                irda->error_state = HAL_IRDA_ERROR_NONE;
            }
            return;
        }
    }

    /* transmitter buffer empty interrupt handle */
    if(RESET != hals_irda_interrupt_flag_get(irda->periph, USART_INT_FLAG_TBE)) {
        if(NULL != irda->irda_irq.transmit_ready_handle) {
            irda->irda_irq.transmit_ready_handle(irda);
        }
        return;
    }

    /* transmission complete interrupt handle */
    if(RESET != hals_irda_interrupt_flag_get(irda->periph, USART_INT_FLAG_TC)) {
        hals_irda_interrupt_flag_clear(irda->periph, USART_INT_FLAG_TC);

        if(NULL != irda->irda_irq.transmit_complete_handle) {
            irda->irda_irq.transmit_complete_handle(irda);
        }
        return;
    }
}

/*!
    \brief      set user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  irda: irda device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to irda interrupt callback functions structure
                  The structure member can be assigned as following parameters:
      \arg        hal_irq_handle_cb function pointer: the function is user-defined,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
      \arg        NULL: The corresponding callback mechanism is out of use, and
                    disable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_irda_irq_handle_set(hal_irda_dev_struct *irda, hal_irda_irq_struct *p_irq)
{
    /* set user-defined error interrupt callback */
    if(NULL != p_irq->error_handle) {
        irda->irda_irq.error_handle = p_irq->error_handle;
        hals_irda_interrupt_enable(irda->periph, USART_INT_ERR);
        hals_irda_interrupt_enable(irda->periph, USART_INT_PERR);
    } else {
        irda->irda_irq.error_handle = NULL;
        hals_irda_interrupt_disable(irda->periph, USART_INT_ERR);
        hals_irda_interrupt_disable(irda->periph, USART_INT_PERR);
    }

    /* set user-defined receive complete interrupt callback */
    if(NULL != p_irq->receive_complete_handle) {
        irda->irda_irq.receive_complete_handle = p_irq->receive_complete_handle;
        hals_irda_interrupt_enable(irda->periph, USART_INT_RBNE);
    } else {
        irda->irda_irq.receive_complete_handle = NULL;
        hals_irda_interrupt_disable(irda->periph, USART_INT_RBNE);
    }

    /* set user-defined transmit complete interrupt callback */
    if(NULL != p_irq->transmit_complete_handle) {
        irda->irda_irq.transmit_complete_handle = p_irq->transmit_complete_handle;
        hals_irda_interrupt_enable(irda->periph, USART_INT_TC);
    } else {
        irda->irda_irq.transmit_complete_handle = NULL;
        hals_irda_interrupt_disable(irda->periph, USART_INT_TC);
    }

    /* set user-defined transmit ready interrupt callback */
    if(NULL != p_irq->transmit_ready_handle) {
        irda->irda_irq.transmit_ready_handle = p_irq->transmit_ready_handle;
        hals_irda_interrupt_enable(irda->periph, USART_INT_TBE);
    } else {
        irda->irda_irq.transmit_ready_handle = NULL;
        hals_irda_interrupt_disable(irda->periph, USART_INT_TBE);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_irda_irq_handle_all_reset(hal_irda_dev_struct *irda)
{
    /* configure interrupt callback function to NULL */
    irda->irda_irq.error_handle = NULL;
    irda->irda_irq.receive_complete_handle = NULL;
    irda->irda_irq.transmit_complete_handle = NULL;
    irda->irda_irq.transmit_ready_handle = NULL;
}

/*!
    \brief      transmit amounts of data, poll transmit process and completed status
                the function is blocking
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY, HAL_ERR_TIMEOUT details refer to gd32f3x0_hal.h
*/
int32_t hal_irda_transmit_poll(hal_irda_dev_struct *irda, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms)
{
    uint8_t data_length;
    uint32_t tick_start;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == irda) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [irda] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the tx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->tx_state) {
        HAL_DEBUGE("irda tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock irda */
    HAL_LOCK(irda);

    irda->error_state = HAL_IRDA_ERROR_NONE;
    irda->tx_state = IRDA_STATE_BUSY;

    /* initialize transmit parameters */
    irda->txbuffer.buffer   = (uint8_t *)p_buffer;
    irda->txbuffer.length   = length;
    irda->txbuffer.pos      = 0U;

    /* calculate the data length */
    data_length = 1U;
    if(RESET != (USART_CTL0(irda->periph) & USART_CTL0_WL)) {
        if(RESET == (USART_CTL0(irda->periph) & USART_CTL0_PCEN)) {
            data_length = 2U;
        }
    }

    /* configure timeout */
    tick_start = hal_sys_basetick_count_get();

    while(irda->txbuffer.pos < irda->txbuffer.length) {
        /* wait for transmit buffer empty */
        while(RESET == hals_irda_flag_get(irda->periph, USART_FLAG_TBE)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    HAL_DEBUGW("irda transmit timeout");
                    /* reset the state */
                    irda->rx_state = IRDA_STATE_FREE;
                    /* unlock irda */
                    HAL_UNLOCK(irda);
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* send the data to be transmitted */
        if(2U == data_length) {
            hals_irda_data_transmit(irda->periph, (*(uint16_t *)irda->txbuffer.buffer & (uint16_t)0x1FFU));
            irda->txbuffer.buffer += 2;
        } else {
            hals_irda_data_transmit(irda->periph, (*irda->txbuffer.buffer & (uint8_t)0xFFU));
            irda->txbuffer.buffer++;
        }

        /* change the transmit pointer */
        irda->txbuffer.pos++;
    }

    /* wait for transmit complete */
    while(RESET == hals_irda_flag_get(irda->periph, USART_FLAG_TC)) {
        if(HAL_TIMEOUT_FOREVER != timeout_ms) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                HAL_DEBUGW("irda transmit timeout");
                /* reset the state */
                irda->rx_state = IRDA_STATE_FREE;
                /* unlock irda */
                HAL_UNLOCK(irda);
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    /* change the Tx state to free */
    irda->tx_state = IRDA_STATE_FREE;

    /* unlock irda */
    HAL_UNLOCK(irda);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data, poll receive process and completed status
                the function is blocking
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY, HAL_ERR_TIMEOUT details refer to gd32f3x0_hal.h
*/
int32_t hal_irda_receive_poll(hal_irda_dev_struct *irda, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms)
{
    uint8_t data_length;
    uint32_t tick_start;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == irda) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [irda] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the rx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->rx_state) {
        HAL_DEBUGE("irda rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock irda */
    HAL_LOCK(irda);

    irda->error_state = HAL_IRDA_ERROR_NONE;
    irda->rx_state = IRDA_STATE_BUSY;

    /* initialize receive parameters */

    irda->rxbuffer.buffer   = (uint8_t *)p_buffer;
    irda->rxbuffer.length   = length;
    irda->rxbuffer.pos      = 0U;
    irda->data_bit_mask     = _irda_data_bit_mask_get(irda);

    /* calculate the data length */
    data_length = 1U;
    if(RESET != (USART_CTL0(irda->periph) & USART_CTL0_WL)) {
        if(RESET == (USART_CTL0(irda->periph) & USART_CTL0_PCEN)) {
            data_length = 2U;
        }
    }

    /* configure timeout */
    tick_start = hal_sys_basetick_count_get();

    while(irda->rxbuffer.pos < irda->rxbuffer.length) {
        /* wait for read data buffer not empty */
        while(RESET == hals_irda_flag_get(irda->periph, USART_FLAG_RBNE)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    HAL_DEBUGW("irda receive timeout");
                    /* reset the state */
                    irda->rx_state = IRDA_STATE_FREE;
                    /* unlock irda */
                    HAL_UNLOCK(irda);

                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* read data from data register */
        if(2U == data_length) {
            *(uint16_t *)irda->rxbuffer.buffer = (hals_irda_data_receive(irda->periph) & irda->data_bit_mask);
            irda->rxbuffer.buffer += 2;
        } else {
            *irda->rxbuffer.buffer = (uint8_t)(hals_irda_data_receive(irda->periph) & irda->data_bit_mask);
            irda->rxbuffer.buffer++;
        }

        /* change the receive pointer */
        irda->rxbuffer.pos++;
    }

    /* change the Rx state to free */
    irda->rx_state = IRDA_STATE_FREE;

    /* unlock irda */
    HAL_UNLOCK(irda);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by interrupt method
                the function is non-blocking
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_irda_transmit_interrupt(hal_irda_dev_struct *irda, uint8_t *p_buffer, uint32_t length,
                                    hal_irda_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == irda) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [irda] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the tx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->tx_state) {
        HAL_DEBUGE("irda tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock irda */
    HAL_LOCK(irda);

    irda->tx_state = IRDA_STATE_BUSY;
    irda->error_state = HAL_IRDA_ERROR_NONE;

    /* initialize transmit parameters */
    irda->txbuffer.buffer   = (uint8_t *)p_buffer;
    irda->txbuffer.length   = length;
    irda->txbuffer.pos      = 0U;
    irda->tx_callback       = (void *)p_user_func;

    /* configure the transmit ready and complete callback as the function implemented */
    irda->irda_irq.transmit_ready_handle = _irda_transmit_interrupt;
    irda->irda_irq.transmit_complete_handle = _irda_transmit_complete_interrupt;

    /* clear IRDA TC interrupt flag */
    hals_irda_interrupt_flag_clear(irda->periph, USART_INT_FLAG_TC);

    /* enable the TBE interrupt */
    hals_irda_interrupt_enable(irda->periph, USART_INT_TBE);

    /* unlock irda */
    HAL_UNLOCK(irda);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by interrupt method
                the function is non-blocking
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_irda_receive_interrupt(hal_irda_dev_struct *irda, uint8_t *p_buffer, uint32_t length,
                                   hal_irda_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == irda) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [irda] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the rx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->rx_state) {
        HAL_DEBUGE("irda rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock irda */
    HAL_LOCK(irda);

    irda->rx_state = IRDA_STATE_BUSY;
    irda->error_state = HAL_IRDA_ERROR_NONE;

    /* initialize receive parameters */
    irda->rxbuffer.buffer   = (uint8_t *)p_buffer;
    irda->rxbuffer.length   = length;
    irda->rxbuffer.pos      = 0U;
    irda->rx_callback       = (void *)p_user_func;
    irda->irda_irq.receive_complete_handle = _irda_receive_interrupt;
    irda->data_bit_mask = _irda_data_bit_mask_get(irda);

    /* enable PERR, ERR, RBNE interrupt */
    hals_irda_interrupt_enable(irda->periph, USART_INT_PERR);
    hals_irda_interrupt_enable(irda->periph, USART_INT_ERR);
    hals_irda_interrupt_enable(irda->periph, USART_INT_RBNE);

    /* unlock irda */
    HAL_UNLOCK(irda);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by dma method
                the function is non-blocking
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_irda_transmit_dma(hal_irda_dev_struct *irda, uint8_t *p_buffer, uint16_t length, hal_irda_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == irda) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [irda] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check the parameter */
    if(NULL == irda->p_dma_tx) {
        HAL_DEBUGE("parameter [irda->p_dma_tx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the tx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->tx_state) {
        HAL_DEBUGE("irda tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock irda */
    HAL_LOCK(irda);

    irda->tx_state = IRDA_STATE_BUSY;
    irda->error_state = HAL_IRDA_ERROR_NONE;

    /* initialize transmit parameters */
    irda->txbuffer.buffer   = (uint8_t *)p_buffer;
    irda->txbuffer.length   = length;
    irda->txbuffer.pos      = 0U;

    if(NULL != p_func) {
        irda->tx_callback = (void *)p_func->complete_func;
        irda->irda_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    } else {
        irda->tx_callback = NULL;
        irda->irda_irq.error_handle = NULL;
    }
    irda->irda_irq.transmit_complete_handle = _irda_transmit_complete_interrupt;

    /* configure DMA interrupt callback function */
    dma_irq.full_finish_handle = _irda_transmit_dma;
    dma_irq.error_handle = _irda_dma_error;
    if(NULL != irda->p_dma_tx->dma_irq.half_finish_handle) {
        dma_irq.half_finish_handle = irda->p_dma_tx->dma_irq.half_finish_handle;
    } else {
        dma_irq.half_finish_handle = NULL;
    }

    /* start DMA interrupt mode transfer */
    if(HAL_ERR_NONE != hal_dma_start_interrupt(irda->p_dma_tx, (uint32_t)irda->txbuffer.buffer,
            (uint32_t)&USART_TDATA(irda->periph), irda->txbuffer.length, &dma_irq)) {
        irda->tx_state = IRDA_STATE_FREE;
        irda->error_state = HAL_IRDA_ERROR_DMATX;
        /* unlock uart */
        HAL_UNLOCK(irda);
    }

    /* clear IRDA TC interrupt flag */
    hals_irda_interrupt_flag_clear(irda->periph, USART_INT_FLAG_TC);

    /* DMA enable for transmission */
    hals_irda_dma_transmit_config(irda->periph, USART_TRANSMIT_DMA_ENABLE);

    /* unlock irda */
    HAL_UNLOCK(irda);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by dma method
                the function is non-blocking
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_irda_receive_dma(hal_irda_dev_struct *irda, uint8_t *p_buffer, uint16_t length, hal_irda_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == irda) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [irda] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check the parameter */
    if(NULL == irda->p_dma_rx) {
        HAL_DEBUGE("parameter [irda->p_dma_rx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the rx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->rx_state) {
        HAL_DEBUGE("irda rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock irda */
    HAL_LOCK(irda);

    irda->rx_state = IRDA_STATE_BUSY;
    irda->error_state = HAL_IRDA_ERROR_NONE;

    /* initialize receive parameters */
    irda->rxbuffer.buffer   = (uint8_t *)p_buffer;
    irda->rxbuffer.length   = length;
    irda->rxbuffer.pos      = 0U;

    if(NULL != p_func) {
        irda->rx_callback = (void *)p_func->complete_func;
        irda->irda_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    } else {
        irda->rx_callback = NULL;
        irda->irda_irq.error_handle = NULL;
    }
    irda->data_bit_mask = _irda_data_bit_mask_get(irda);

    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _irda_receive_dma;
    dma_irq.error_handle = _irda_dma_error;
    if(NULL != irda->p_dma_rx->dma_irq.half_finish_handle) {
        dma_irq.half_finish_handle = irda->p_dma_rx->dma_irq.half_finish_handle;
    } else {
        dma_irq.half_finish_handle = NULL;
    }

    /* start DMA interrupt mode transfer */
    if(HAL_ERR_NONE != hal_dma_start_interrupt(irda->p_dma_rx, (uint32_t)&USART_RDATA(irda->periph),
            (uint32_t)irda->rxbuffer.buffer, irda->rxbuffer.length, &dma_irq)) {
        irda->rx_state = IRDA_STATE_FREE;
        irda->error_state = HAL_IRDA_ERROR_DMARX;
        /* unlock uart */
        HAL_UNLOCK(irda);
    }

    /* enable the usart parity error and error interrupt: (frame error, noise error, overrun error)  */
    hals_irda_interrupt_enable(irda->periph, USART_INT_PERR);
    hals_irda_interrupt_enable(irda->periph, USART_INT_ERR);

    /* DMA enable for reception */
    hals_irda_dma_receive_config(irda->periph, USART_RECEIVE_DMA_ENABLE);

    /* unlock irda */
    HAL_UNLOCK(irda);

    return HAL_ERR_NONE;
}

/*!
    \brief      pause irda DMA transfer during transmission process
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32f3x0_hal.h
*/
int32_t hal_irda_dma_pause(hal_irda_dev_struct *irda)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == irda) {
        HAL_DEBUGE("parameter [irda] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock irda */
    HAL_LOCK(irda);

    /* check the tx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->tx_state) {
        /* disable DMA transimt */
        hals_irda_dma_transmit_config(irda->periph, USART_TRANSMIT_DMA_DISABLE);
    }

    /* check the rx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->rx_state) {
        /* disable the PERR and ERR interrupt */
        hals_irda_interrupt_disable(irda->periph, USART_INT_PERR);
        hals_irda_interrupt_disable(irda->periph, USART_INT_ERR);

        /* disable DMA receive */
        hals_irda_dma_receive_config(irda->periph, USART_RECEIVE_DMA_DISABLE);
    }

    /* unlock irda */
    HAL_UNLOCK(irda);

    return HAL_ERR_NONE;
}

/*!
    \brief      resume irda DMA transfer during transmission process
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32f3x0_hal.h
*/
int32_t hal_irda_dma_resume(hal_irda_dev_struct *irda)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == irda) {
        HAL_DEBUGE("parameter [irda] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock irda */
    HAL_LOCK(irda);

    /* check the tx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->tx_state) {
        /* enable DMA transimt */
        hals_irda_dma_transmit_config(irda->periph, USART_TRANSMIT_DMA_ENABLE);
    }

    /* check the rx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->rx_state) {
        /* enable the PERR and ERR interrupt */
        hals_irda_interrupt_enable(irda->periph, USART_INT_PERR);
        hals_irda_interrupt_enable(irda->periph, USART_INT_ERR);

        /* enable DMA receive */
        hals_irda_dma_receive_config(irda->periph, USART_RECEIVE_DMA_ENABLE);
    }

    /* unlock irda */
    HAL_UNLOCK(irda);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop irda transmit transfer
                the function is blocking
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32f3x0_hal.h
*/
int32_t hal_irda_transmit_stop(hal_irda_dev_struct *irda)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == irda) {
        HAL_DEBUGE("parameter [irda] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock irda */
    HAL_LOCK(irda);

    /* disable the TBE and TC interrupt */
    hals_irda_interrupt_disable(irda->periph, USART_INT_TBE);
    hals_irda_interrupt_disable(irda->periph, USART_INT_TC);

    /* disable DMA transimt and stop DMA */
    hals_irda_dma_transmit_config(irda->periph, USART_TRANSMIT_DMA_DISABLE);
    hal_dma_stop(irda->p_dma_tx);

    /* reset the position and state */
    irda->txbuffer.pos = 0;
    irda->tx_state = IRDA_STATE_FREE;

    /* unlock irda */
    HAL_UNLOCK(irda);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop irda receive transfer
                the function is blocking
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32f3x0_hal.h
*/
int32_t hal_irda_receive_stop(hal_irda_dev_struct *irda)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == irda) {
        HAL_DEBUGE("parameter [irda] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock irda */
    HAL_LOCK(irda);

    /* disable the RBNE, PERR and ERR interrupt */
    hals_irda_interrupt_disable(irda->periph, USART_INT_RBNE);
    hals_irda_interrupt_disable(irda->periph, USART_INT_PERR);
    hals_irda_interrupt_disable(irda->periph, USART_INT_ERR);

    /* disable DMA receive and stop DMA */
    hals_irda_dma_receive_config(irda->periph, USART_RECEIVE_DMA_DISABLE);
    hal_dma_stop(irda->p_dma_rx);

    /* reset the position and state */
    irda->rxbuffer.pos = 0;
    irda->rx_state = IRDA_STATE_FREE;

    /* clear interrupt error flags */
    hals_irda_interrupt_flag_clear(irda->periph, USART_INT_FLAG_PERR);
    hals_irda_interrupt_flag_clear(irda->periph, USART_INT_FLAG_ERR_FERR);
    hals_irda_interrupt_flag_clear(irda->periph, USART_INT_FLAG_ERR_NERR);
    hals_irda_interrupt_flag_clear(irda->periph, USART_INT_FLAG_ERR_ORERR);

    /* unlock irda */
    HAL_UNLOCK(irda);

    return HAL_ERR_NONE;
}

/*!
    \brief      reset IRDA
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_irda_deinit(uint32_t irda_periph)
{
    switch(irda_periph) {
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
    \brief      configure IRDA baud rate value
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[in]  baudval: baud rate value
    \param[out] none
    \retval     none
*/
void hals_irda_baudrate_set(uint32_t irda_periph, uint32_t baudval)
{
    uint32_t uclk = 0U, intdiv = 0U, fradiv = 0U, udiv = 0U;
    switch(irda_periph) {
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
    if(USART_CTL0(irda_periph) & USART_CTL0_OVSMOD) {
        /* oversampling by 8, configure the value of USART_BAUD */
        udiv = ((2U * uclk) + (baudval / 2U)) / baudval;
        intdiv = udiv & 0x0000fff0U;
        fradiv = (udiv >> 1U) & 0x00000007U;
        USART_BAUD(irda_periph) = ((USART_BAUD_FRADIV | USART_BAUD_INTDIV) & (intdiv | fradiv));
    } else {
        /* oversampling by 16, configure the value of USART_BAUD */
        udiv = (uclk + (baudval / 2U)) / baudval;
        intdiv = udiv & 0x0000fff0U;
        fradiv = udiv & 0x0000000fU;
        USART_BAUD(irda_periph) = ((USART_BAUD_FRADIV | USART_BAUD_INTDIV) & (intdiv | fradiv));
    }
}

/*!
    \brief      configure IRDA parity
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[in]  paritycfg: IRDA parity configure
                only one parameter can be selected which is shown as below:
      \arg        USART_PM_NONE: no parity
      \arg        USART_PM_ODD: odd parity
      \arg        USART_PM_EVEN: even parity
    \param[out] none
    \retval     none
*/
void hals_irda_parity_config(uint32_t irda_periph, uint32_t paritycfg)
{
    /* disable IRDA */
    USART_CTL0(irda_periph) &= ~(USART_CTL0_UEN);
    /* clear USART_CTL0 PM,PCEN bits */
    USART_CTL0(irda_periph) &= ~(USART_CTL0_PM | USART_CTL0_PCEN);
    /* configure IRDA parity mode */
    USART_CTL0(irda_periph) |= paritycfg;
}

/*!
    \brief      configure IRDA word length
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[in]  wlen: IRDA word length configure
                only one parameter can be selected which is shown as below:
      \arg        USART_WL_8BIT: 8 bits
      \arg        USART_WL_9BIT: 9 bits
    \param[out] none
    \retval     none
*/
void hals_irda_word_length_set(uint32_t irda_periph, uint32_t wlen)
{
    /* disable IRDA */
    USART_CTL0(irda_periph) &= ~(USART_CTL0_UEN);
    /* clear USART_CTL0 WL bit */
    USART_CTL0(irda_periph) &= ~USART_CTL0_WL;
    /* configure IRDA word length */
    USART_CTL0(irda_periph) |= wlen;
}

/*!
    \brief      configure IRDA stop bit length
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[in]  stblen: IRDA stop bit configure
                only one parameter can be selected which is shown as below:
      \arg        USART_STB_1BIT: 1 bit
      \arg        USART_STB_0_5BIT: 0.5bit
      \arg        USART_STB_2BIT: 2 bits
      \arg        USART_STB_1_5BIT: 1.5bit
    \param[out] none
    \retval     none
*/
void hals_irda_stop_bit_set(uint32_t irda_periph, uint32_t stblen)
{
    /* disable IRDA */
    USART_CTL0(irda_periph) &= ~(USART_CTL0_UEN);
    /* clear USART_CTL1 STB bits */
    USART_CTL1(irda_periph) &= ~USART_CTL1_STB;
    USART_CTL1(irda_periph) |= stblen;
}

/*!
    \brief      enable IRDA
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_irda_enable(uint32_t irda_periph)
{
    USART_CTL0(irda_periph) |= USART_CTL0_UEN;
}

/*!
    \brief      disable IRDA
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_irda_disable(uint32_t irda_periph)
{
    USART_CTL0(irda_periph) &= ~(USART_CTL0_UEN);
}

/*!
    \brief      configure IRDA transmitter
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[in]  txconfig: enable or disable IRDA transmitter
                only one parameter can be selected which is shown as below:
      \arg        USART_TRANSMIT_ENABLE: enable IRDA transmission
      \arg        USART_TRANSMIT_DISABLE: enable IRDA transmission
    \param[out] none
    \retval     none
*/
void hals_irda_transmit_config(uint32_t irda_periph, uint32_t txconfig)
{
    USART_CTL0(irda_periph) &= ~USART_CTL0_TEN;
    /* configure transfer mode */
    USART_CTL0(irda_periph) |= txconfig;
}

/*!
    \brief      configure IRDA receiver
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[in]  rxconfig: enable or disable IRDA receiver
                only one parameter can be selected which is shown as below:
      \arg        USART_RECEIVE_ENABLE: enable IRDA reception
      \arg        USART_RECEIVE_DISABLE: disable IRDA reception
    \param[out] none
    \retval     none
*/
void hals_irda_receive_config(uint32_t irda_periph, uint32_t rxconfig)
{
    USART_CTL0(irda_periph) &= ~USART_CTL0_REN;
    /* configure receiver mode */
    USART_CTL0(irda_periph) |= rxconfig;
}

/*!
    \brief      IRDA transmit data function
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[in]  data: data of transmission
    \param[out] none
    \retval     none
*/
void hals_irda_data_transmit(uint32_t irda_periph, uint16_t data)
{
    USART_TDATA(irda_periph) = (USART_TDATA_TDATA & (uint32_t)data);
}

/*!
    \brief      IRDA receive data function
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[out] none
    \retval     data of received
*/
uint16_t hals_irda_data_receive(uint32_t irda_periph)
{
    return (uint16_t)(GET_BITS(USART_RDATA(irda_periph), 0U, 8U));
}

/*!
    \brief      enable irda mode
    \param[in]  irda_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_irda_mode_enable(uint32_t irda_periph)
{
    /* disable IRDA */
    USART_CTL0(irda_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(irda_periph) |= USART_CTL2_IREN;
}

/*!
    \brief      disable irda mode
    \param[in]  irda_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_irda_mode_disable(uint32_t irda_periph)
{
    /* disable IRDA */
    USART_CTL0(irda_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(irda_periph) &= ~(USART_CTL2_IREN);
}

/*!
    \brief      configure the peripheral clock prescaler in IRDA irda low-power or smartcard mode
    \param[in]  irda_periph: USARTx(x=0)
    \param[in]  psc: 0x00000000-0x000000FF
    \param[out] none
    \retval     none
*/
void hals_irda_prescaler_config(uint32_t irda_periph, uint32_t psc)
{
    /* disable IRDA */
    USART_CTL0(irda_periph) &= ~(USART_CTL0_UEN);
    USART_GP(irda_periph) &= ~(USART_GP_PSC);
    USART_GP(irda_periph) |= psc;
}

/*!
    \brief      configure irda low-power
    \param[in]  irda_periph: USARTx(x=0)
    \param[in]  irlp: irda low-power or normal
                only one parameter can be selected which is shown as below:
      \arg        USART_IRLP_LOW:    low-power
      \arg        USART_IRLP_NORMAL: normal
    \param[out] none
    \retval     none
*/
void hals_irda_lowpower_config(uint32_t irda_periph, uint32_t irlp)
{
    /* disable IRDA */
    USART_CTL0(irda_periph) &= ~(USART_CTL0_UEN);
    USART_CTL2(irda_periph) &= ~(USART_CTL2_IRLP);
    USART_CTL2(irda_periph) |= (USART_CTL2_IRLP & irlp);
}

/*!
    \brief      configure IRDA DMA reception
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[in]  dmacmd: IRDA DMA mode
                only one parameter can be selected which is shown as below:
      \arg        USART_RECEIVE_DMA_ENABLE: enable IRDA DMA for reception
      \arg        USART_RECEIVE_DMA_DISABLE: disable IRDA DMA for reception
    \param[out] none
    \retval     none
*/
void hals_irda_dma_receive_config(uint32_t irda_periph, uint8_t dmacmd)
{
    USART_CTL2(irda_periph) &= ~USART_CTL2_DENR;
    USART_CTL2(irda_periph) |= (USART_CTL2_DENR & dmacmd);
}

/*!
    \brief      configure IRDA DMA transmission
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[in]  dmacmd: IRDA DMA mode
                only one parameter can be selected which is shown as below:
      \arg        USART_TRANSMIT_DMA_ENABLE: enable IRDA DMA for transmission
      \arg        USART_TRANSMIT_DMA_DISABLE: disable IRDA DMA for transmission
    \param[out] none
    \retval     none
*/
void hals_irda_dma_transmit_config(uint32_t irda_periph, uint8_t dmacmd)
{
    USART_CTL2(irda_periph) &= ~USART_CTL2_DENT;
    USART_CTL2(irda_periph) |= (USART_CTL2_DENT & dmacmd);
}

/*!
    \brief      enable DMA on reception error
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_irda_reception_error_dma_enable(uint32_t irda_periph)
{
    /* disable IRDA */
    USART_CTL0(irda_periph) &= ~(USART_CTL0_UEN);
    USART_CTL2(irda_periph) &= ~(USART_CTL2_DDRE);
}

/*!
    \brief      disable DMA on reception error
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_irda_reception_error_dma_disable(uint32_t irda_periph)
{
    /* disable IRDA */
    USART_CTL0(irda_periph) &= ~(USART_CTL0_UEN);
    USART_CTL2(irda_periph) |= USART_CTL2_DDRE;
}

/*!
    \brief      get IRDA status
    \param[in]  irda_periph: USARTx(x=0,1)
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
FlagStatus hals_irda_flag_get(uint32_t irda_periph, usart_flag_enum flag)
{
    if(RESET != (USART_REG_VAL(irda_periph, flag) & BIT(USART_BIT_POS(flag)))) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear IRDA status
    \param[in]  irda_periph: USARTx(x=0,1)
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
void hals_irda_flag_clear(uint32_t irda_periph, usart_flag_enum flag)
{
    USART_INTC(irda_periph) |= BIT(USART_BIT_POS(flag));
}

/*!
    \brief      enable IRDA interrupt
    \param[in]  irda_periph: USARTx(x=0,1)
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
void hals_irda_interrupt_enable(uint32_t irda_periph, usart_interrupt_enum interrupt)
{
    USART_REG_VAL(irda_periph, interrupt) |= BIT(USART_BIT_POS(interrupt));
}

/*!
    \brief      disable IRDA interrupt
    \param[in]  irda_periph: USARTx(x=0,1)
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
void hals_irda_interrupt_disable(uint32_t irda_periph, usart_interrupt_enum interrupt)
{
    USART_REG_VAL(irda_periph, interrupt) &= ~BIT(USART_BIT_POS(interrupt));
}

/*!
    \brief      get IRDA interrupt flag status
    \param[in]  irda_periph: USARTx(x=0,1)
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
FlagStatus hals_irda_interrupt_flag_get(uint32_t irda_periph, usart_interrupt_flag_enum int_flag)
{
    uint32_t intenable = 0U, flagstatus = 0U;
    /* get the interrupt enable bit status */
    intenable = (USART_REG_VAL(irda_periph, int_flag) & BIT(USART_BIT_POS(int_flag)));
    /* get the corresponding flag bit status */
    flagstatus = (USART_REG_VAL2(irda_periph, int_flag) & BIT(USART_BIT_POS2(int_flag)));

    if(flagstatus && intenable) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear IRDA interrupt flag
    \param[in]  irda_periph: USARTx(x=0,1)
    \param[in]  flag: IRDA interrupt flag
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
void hals_irda_interrupt_flag_clear(uint32_t irda_periph, usart_interrupt_flag_enum int_flag)
{
    if(USART_INT_FLAG_RFF == int_flag) {
        USART_RFCS(irda_periph) &= (uint32_t)(~USART_RFCS_RFFINT);
    } else {
        USART_INTC(irda_periph) |= BIT(USART_BIT_POS2(int_flag));
    }
}

/*!
    \brief      get the mask of data bit
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the mask of data bit(0x7F, 0xFF, 0x1FF)
*/
static uint16_t _irda_data_bit_mask_get(hal_irda_dev_struct *irda)
{
    uint16_t reval;

    if(RESET != (USART_CTL0(irda->periph) & USART_CTL0_WL)) {
        /* check whether the PCEN is enabled */
        if(RESET != (USART_CTL0(irda->periph) & USART_CTL0_PCEN)) {
            reval = 0xFFU;
        } else {
            reval = 0x1FFU;
        }
    } else {
        /* check whether the PCEN is enabled */
        if(RESET != (USART_CTL0(irda->periph) & USART_CTL0_PCEN)) {
            reval = 0x7FU;
        } else {
            reval = 0xFFU;
        }
    }

    return reval;
}

/*!
    \brief      get irda error flag
    \param[in]  irda: irda device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     SET or RESET
*/
static FlagStatus _irda_error_flag_get(hal_irda_dev_struct *irda)
{
    if(0U == (USART_STAT(irda->periph) & (uint32_t)(USART_STAT_PERR | USART_STAT_FERR | \
              USART_STAT_ORERR | USART_STAT_NERR))) {
        return RESET;
    } else {
        return SET;
    }
}

/*!
    \brief      handle the transmit complete interrupt
    \param[in]  irda: pointer to a irda device information structure
    \param[out] none
    \retval     none
*/
static void _irda_transmit_complete_interrupt(void *irda)
{
    hal_irda_dev_struct *p_irda = irda;
    hal_irda_user_cb p_func = (hal_irda_user_cb)p_irda->tx_callback;

    /* disable the transmit complete interrupt */
    hals_irda_interrupt_disable(p_irda->periph, USART_INT_TC);
    /* reset transmit_complete_handle and tx_state */
    p_irda->irda_irq.transmit_complete_handle = NULL;
    p_irda->tx_state = IRDA_STATE_FREE;

    if(NULL != p_func) {
        /* if there is a user transmit complete callback */
        p_func(p_irda);
    }
}

/*!
    \brief      handle the transmit interrupt
    \param[in]  irda: pointer to a irda device information structure
    \param[out] none
    \retval     none
*/
static void _irda_transmit_interrupt(void *irda)
{
    uint32_t temp_val;
    hal_irda_dev_struct *p_irda = irda;

    temp_val = p_irda->txbuffer.pos;
    if(temp_val < p_irda->txbuffer.length) {
        if((RESET != (USART_CTL0(p_irda->periph) & USART_CTL0_WL)) && \
                (RESET == (USART_CTL0(p_irda->periph) & USART_CTL0_PCEN))) {
            /* 9-bit data, none parity */
            hals_irda_data_transmit(p_irda->periph, (*(uint16_t *)p_irda->txbuffer.buffer & (uint16_t)0x1FFU));
            p_irda->txbuffer.buffer += 2U;
        } else {
            /* 9-bit data, with parity or 8-bit data */
            hals_irda_data_transmit(p_irda->periph, (*p_irda->txbuffer.buffer & (uint8_t)0xFFU));
            p_irda->txbuffer.buffer++;
        }
        p_irda->txbuffer.pos++;
    } else {
        /* disable the TBE interrupt, enable the TC interrupt and reset the transmit_ready_handle */
        hals_irda_interrupt_disable(p_irda->periph, USART_INT_TBE);
        hals_irda_interrupt_enable(p_irda->periph, USART_INT_TC);
        p_irda->irda_irq.transmit_ready_handle = NULL;
    }
}

/*!
    \brief      handle the receive interrupt
    \param[in]  irda: pointer to a irda device information structure
    \param[out] none
    \retval     none
*/
static void _irda_receive_interrupt(void *irda)
{
    hal_irda_dev_struct *p_irda = irda;
    hal_irda_user_cb p_func = (hal_irda_user_cb)p_irda->rx_callback;

    /* store the received data */
    if(0x1FFU == p_irda->data_bit_mask) {
        *(uint16_t *)p_irda->rxbuffer.buffer = (hals_irda_data_receive(p_irda->periph) &
                                                p_irda->data_bit_mask);
        p_irda->rxbuffer.buffer += 2U;
    } else {
        *p_irda->rxbuffer.buffer = (uint8_t)(hals_irda_data_receive(p_irda->periph) &
                                             p_irda->data_bit_mask);
        p_irda->rxbuffer.buffer++;
    }
    p_irda->rxbuffer.pos++;

    if(p_irda->rxbuffer.pos == p_irda->rxbuffer.length) {
        /* disable PERR, ERR, RBNE interrupt */
        hals_irda_interrupt_disable(p_irda->periph, USART_INT_PERR);
        hals_irda_interrupt_disable(p_irda->periph, USART_INT_ERR);
        hals_irda_interrupt_disable(p_irda->periph, USART_INT_RBNE);

        /* reset receive_complete_handle and rx_state */
        p_irda->irda_irq.receive_complete_handle = NULL;
        p_irda->rx_state = IRDA_STATE_FREE;

        if(NULL != p_func) {
            /* if there is a user receive complete callback */
            p_func(p_irda);
        }
    }
}

/*!
    \brief      handle the irda DMA transmit process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _irda_transmit_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_irda_dev_struct *p_irda;

    p_dma = (hal_dma_dev_struct *)dma;
    p_irda = (hal_irda_dev_struct *)p_dma->p_periph;

    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        p_irda->txbuffer.pos = p_irda->txbuffer.length;
        hals_irda_dma_transmit_config(p_irda->periph, USART_TRANSMIT_DMA_DISABLE);
        /* enable TC interrupt */
        hals_irda_interrupt_enable(p_irda->periph, USART_INT_TC);
    }
}

/*!
    \brief      handle the irda DMA receive process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _irda_receive_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_irda_dev_struct *p_irda;
    hal_irda_user_cb p_func;

    p_dma = (hal_dma_dev_struct *)dma;
    p_irda = (hal_irda_dev_struct *)p_dma->p_periph;
    p_func = (hal_irda_user_cb)p_irda->rx_callback;

    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        p_irda->rxbuffer.pos = p_irda->rxbuffer.length;
        /* disable DMA receive, PERR and ERR interrupt */
        hals_irda_dma_receive_config(p_irda->periph, USART_RECEIVE_DMA_DISABLE);
        hals_irda_interrupt_disable(p_irda->periph, USART_INT_PERR);
        hals_irda_interrupt_disable(p_irda->periph, USART_INT_ERR);
        /* reset rx_state */
        p_irda->rx_state = IRDA_STATE_FREE;
    }

    if(NULL != p_func) {
        /* if there is a user receive complete callback */
        p_func(p_irda);
    }
}

/*!
    \brief      handle the irda DMA error process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _irda_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_irda_dev_struct *p_irda;

    p_dma = (hal_dma_dev_struct *)dma;
    p_irda = (hal_irda_dev_struct *)p_dma->p_periph;

    if(IRDA_STATE_BUSY == p_irda->tx_state) {
        /* transmit state is busy */
        p_irda->error_state |= HAL_SMARTCARD_ERROR_DMATX;
        p_irda->last_error = HAL_SMARTCARD_ERROR_DMATX;
        p_irda->txbuffer.pos = p_irda->txbuffer.length;

        /* disable DMA transmit and reset tx_state */
        hals_irda_dma_transmit_config(p_irda->periph, USART_TRANSMIT_DMA_DISABLE);
        p_irda->tx_state = IRDA_STATE_FREE;
    } else if(IRDA_STATE_BUSY == p_irda->rx_state) {
        /* receive state is busy */
        p_irda->error_state |= HAL_SMARTCARD_ERROR_DMARX;
        p_irda->last_error = HAL_SMARTCARD_ERROR_DMARX;
        p_irda->rxbuffer.pos = p_irda->rxbuffer.length;

        /* disable DMA receive, PERR, ERR interrupt */
        hals_irda_dma_receive_config(p_irda->periph, USART_RECEIVE_DMA_DISABLE);
        hals_irda_interrupt_disable(p_irda->periph, USART_INT_PERR);
        hals_irda_interrupt_disable(p_irda->periph, USART_INT_ERR);
        /* reset rx_state */
        p_irda->rx_state = IRDA_STATE_FREE;
    } else {
        HAL_DEBUGE("irda processor fatal error: dma error exception due to run state");
    }

    if(p_irda->irda_irq.error_handle != NULL) {
        /* if there is a user error callback */
        p_irda->irda_irq.error_handle(p_irda);
        p_irda->error_state = HAL_IRDA_ERROR_NONE;
    }
}
