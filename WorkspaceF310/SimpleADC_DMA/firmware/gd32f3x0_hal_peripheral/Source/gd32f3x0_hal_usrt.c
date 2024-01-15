/*!
    \file    gd32f3x0_hal_usrt.c
    \brief   USRT driver

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

#define USRT_DUMMY_DATA            ((uint16_t) 0xFFFFU)

static uint16_t _usrt_data_bit_mask_get(hal_usrt_dev_struct *usrt);
static FlagStatus _usrt_error_flag_get(hal_usrt_dev_struct *usrt);
static void _usrt_transmit_complete_interrupt(void *usrt);
static void _usrt_transmit_interrupt(void *usrt);
static void _usrt_receive_interrupt(void *usrt);
static void _usrt_tx_rx_interrupt(void *usrt);
static void _usrt_transmit_dma(void *dma);
static void _usrt_receive_dma(void *dma);
static void _usrt_dma_error(void *dma);

/*!
    \brief      initialize the usrt structure with the default values
                note: this function must be called after the structure is created
    \param[in]  hal_struct_type: type of usrt structure for initialization
      \arg        HAL_USRT_INIT_STRUCT: initialization structure
      \arg        HAL_USRT_DEV_STRUCT: device information structure
      \arg        HAL_USRT_USER_CALLBCAK_STRUCT: user callback structure
    \param[in]  p_struct: structure pointer
    \param[out] none
    \retval     none
*/
void hal_usrt_struct_init(hal_usrt_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct) {
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_USRT_INIT_STRUCT:
        /* initialize usrt initialization structure with the default values */
        ((hal_usrt_init_struct *)p_struct)->baudrate             = 115200U;
        ((hal_usrt_init_struct *)p_struct)->parity               = USRT_PARITY_NONE;
        ((hal_usrt_init_struct *)p_struct)->word_length          = USRT_WORD_LENGTH_8BIT;
        ((hal_usrt_init_struct *)p_struct)->stop_bit             = USRT_STOP_BIT_1;
        ((hal_usrt_init_struct *)p_struct)->direction            = USRT_DIRECTION_RX_TX;
        ((hal_usrt_init_struct *)p_struct)->rx_fifo_en           = DISABLE;
        ((hal_usrt_init_struct *)p_struct)->clock_polarity       = USRT_CLOCK_POLARITY_LOW;
        ((hal_usrt_init_struct *)p_struct)->clock_phase          = USRT_CLOCK_PHASE_1CK;
        ((hal_usrt_init_struct *)p_struct)->clock_length_lastbit = USRT_LAST_BIT_NOT_OUTPUT;
        break;

    case HAL_USRT_DEV_STRUCT:
        /* initialize usrt device information structure with the default values */
        ((hal_usrt_dev_struct *)p_struct)->periph                            = 0U;
        ((hal_usrt_dev_struct *)p_struct)->usrt_irq.error_handle             = NULL;
        ((hal_usrt_dev_struct *)p_struct)->usrt_irq.receive_complete_handle  = NULL;
        ((hal_usrt_dev_struct *)p_struct)->usrt_irq.transmit_complete_handle = NULL;
        ((hal_usrt_dev_struct *)p_struct)->usrt_irq.transmit_ready_handle    = NULL;
        ((hal_usrt_dev_struct *)p_struct)->p_dma_rx                          = NULL;
        ((hal_usrt_dev_struct *)p_struct)->p_dma_tx                          = NULL;
        ((hal_usrt_dev_struct *)p_struct)->txbuffer.buffer                   = NULL;
        ((hal_usrt_dev_struct *)p_struct)->txbuffer.length                   = 0U;
        ((hal_usrt_dev_struct *)p_struct)->txbuffer.pos                      = 0U;
        ((hal_usrt_dev_struct *)p_struct)->rxbuffer.buffer                   = NULL;
        ((hal_usrt_dev_struct *)p_struct)->rxbuffer.length                   = 0U;
        ((hal_usrt_dev_struct *)p_struct)->rxbuffer.pos                      = 0U;
        ((hal_usrt_dev_struct *)p_struct)->data_bit_mask                     = 0U;
        ((hal_usrt_dev_struct *)p_struct)->last_error                        = HAL_USRT_ERROR_NONE;
        ((hal_usrt_dev_struct *)p_struct)->error_state                       = HAL_USRT_ERROR_NONE;
        ((hal_usrt_dev_struct *)p_struct)->tx_state                          = USRT_STATE_FREE;
        ((hal_usrt_dev_struct *)p_struct)->rx_state                          = USRT_STATE_FREE;
        ((hal_usrt_dev_struct *)p_struct)->rx_callback                       = NULL;
        ((hal_usrt_dev_struct *)p_struct)->tx_callback                       = NULL;
        ((hal_usrt_dev_struct *)p_struct)->mutex                             = HAL_MUTEX_UNLOCKED;
        ((hal_usrt_dev_struct *)p_struct)->priv                              = NULL;
        break;

    case HAL_USRT_USER_CALLBCAK_STRUCT:
        /* initialize user callback structure with the default values */
        ((hal_usrt_user_callback_struct *)p_struct)->complete_func = NULL;
        ((hal_usrt_user_callback_struct *)p_struct)->error_func = NULL;
        break;

    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize the usrt
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_usrt_deinit(hal_usrt_dev_struct *usrt)
{
    uint32_t periph;

    periph = usrt->periph;
    if((USART0 == periph) || (USART1 == periph)) {
        /* deinitialize the periph and the device information sturcture */
        hals_usrt_deinit(periph);
        hal_usrt_struct_init(HAL_USRT_DEV_STRUCT, usrt);
        usrt->periph = periph;
    } else {
        HAL_DEBUGE("parameter [usrt->periph] value is invalid");
    }
}

/*!
    \brief      initialize the usrt with specified values
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which USRT is initialized
    \param[in]  p_init: the initialization data needed to initialize usrt
                  baudrate: communication baudrate
                  parity: USRT_PARITY_NONE, USRT_PARITY_EVEN, USRT_PARITY_ODD
                  word_length: USRT_WORD_LENGTH_8BIT, USRT_WORD_LENGTH_9BIT
                  stop_bit: USRT_STOP_BIT_1, USRT_STOP_BIT_2, USRT_STOP_BIT_1_5
                  direction: USRT_DIRECTION_RX_TX, USRT_DIRECTION_RX_ONLY, USRT_DIRECTION_TX_ONLY
                  rx_fifo_en：DISABLE, ENABLE
                  clock_polarity: USRT_CLOCK_POLARITY_LOW, USRT_CLOCK_POLARITY_HIGH
                  clock_phase: USRT_CLOCK_PHASE_1CK, USRT_CLOCK_PHASE_2CK
                  clock_length_lastbit: USRT_LAST_BIT_NOT_OUTPUT, USRT_LAST_BIT_OUTPUT
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_init(hal_usrt_dev_struct *usrt, uint32_t periph, hal_usrt_init_struct *p_init)
{
    uint32_t reg_temp;

#if (1 == HAL_PARAMETER_CHECK)
    /* check usrt pointer and p_init address */
    if((NULL == usrt) || (NULL == p_init)) {
        HAL_DEBUGE("pointer [usrt] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check periph parameter */
    if((USART0 != periph) && (USART1 != periph)) {
        HAL_DEBUGE("parameter [periph] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check periph value from usrt device struct */
    if(0U != usrt->periph) {
        HAL_DEBUGI("periph value from usrt device struct has been rewrite");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock uart */
    HAL_LOCK(usrt);

    usrt->periph = periph;

    /* disable usrt */
    hals_usrt_disable(periph);

    /* configure CTL0 register */
    reg_temp = USART_CTL0(periph);
    reg_temp &= ~(USART_CTL0_PM | USART_CTL0_PCEN | USART_CTL0_WL | USART_CTL0_REN | USART_CTL0_TEN);
    reg_temp |= (p_init->direction | p_init->parity | p_init->word_length);
    USART_CTL0(periph) = reg_temp;

    /* configure CTL1 register */
    reg_temp = USART_CTL1(periph);
    reg_temp &= ~(USART_CTL1_STB | USART_CTL1_CLEN | USART_CTL1_CPH | USART_CTL1_CPL);
    reg_temp |= (p_init->stop_bit | p_init->clock_length_lastbit | p_init->clock_phase | p_init->clock_polarity | USART_CTL1_CKEN);
    USART_CTL1(periph) = reg_temp;

    /* configure RFCS register */
    if(ENABLE == p_init->rx_fifo_en) {
        USART_RFCS(periph) |= USART_RFCS_RFEN;
    } else {
        USART_RFCS(periph) &= ~USART_RFCS_RFEN;
    }

    /* configure baudrate */
    hals_usrt_baudrate_set(periph, p_init->baudrate);
    usrt->data_bit_mask = _usrt_data_bit_mask_get(usrt);

    /* reset the Rx and Tx state */
    usrt->tx_state = USRT_STATE_FREE;
    usrt->rx_state = USRT_STATE_FREE;

    /* enable usart */
    hals_usrt_enable(usrt->periph);

    /* unlock usrt */
    HAL_UNLOCK(usrt);

    return HAL_ERR_NONE;
}

/*!
    \brief      usrt interrupt handler content function,which is merely used in usrt_handler
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_usrt_irq(hal_usrt_dev_struct *usrt)
{
    if(RESET == _usrt_error_flag_get(usrt)) {
        /* check whether USRT is in receiver mode or not */
        if(RESET != hals_usrt_interrupt_flag_get(usrt->periph, USART_INT_FLAG_RBNE)) {
            if(NULL != usrt->usrt_irq.receive_complete_handle) {
                usrt->usrt_irq.receive_complete_handle(usrt);
            }
            return;
        }
    } else {
        /* check whether the PERR flag is set or not */
        if(RESET != hals_usrt_interrupt_flag_get(usrt->periph, USART_INT_FLAG_PERR)) {
            hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_PERR);
            usrt->error_state |= HAL_USRT_ERROR_PERR;
            usrt->last_error = HAL_USRT_ERROR_PERR;
        }

        /* check whether the NERR flag is set or not */
        if(RESET != hals_usrt_interrupt_flag_get(usrt->periph,
                USART_INT_FLAG_ERR_NERR)) {
            hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_ERR_NERR);
            usrt->error_state |= HAL_USRT_ERROR_NERR;
            usrt->last_error = HAL_USRT_ERROR_NERR;
        }

        /* check whether the FERR flag is set or not */
        if(RESET != hals_usrt_interrupt_flag_get(usrt->periph,
                USART_INT_FLAG_ERR_FERR)) {
            hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_ERR_FERR);
            usrt->error_state |= HAL_USRT_ERROR_FERR;
            usrt->last_error = HAL_USRT_ERROR_FERR;
        }

        /* check whether the ERR ORERR is set or not */
        if(RESET != hals_usrt_interrupt_flag_get(usrt->periph,
                USART_INT_FLAG_ERR_ORERR)) {
            hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_ERR_ORERR);
            usrt->error_state |= HAL_USRT_ERROR_ORERR;
            usrt->last_error = HAL_USRT_ERROR_ORERR;
        }

        /* check whether RBNE ORERR is set or not */
        if(RESET != hals_usrt_interrupt_flag_get(usrt->periph,
                USART_INT_FLAG_RBNE_ORERR)) {
            hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_RBNE_ORERR);
            usrt->error_state |= HAL_USRT_ERROR_ORERR;
            usrt->last_error = HAL_USRT_ERROR_ORERR;
        }

        /* check whether error state is none or not */
        if(HAL_USRT_ERROR_NONE != usrt->error_state) {
            if(NULL != usrt->usrt_irq.error_handle) {
                usrt->usrt_irq.error_handle(usrt);
                usrt->error_state = HAL_USRT_ERROR_NONE;
            }
            return;
        }
    }

    /* transmitter buffer empty interrupt handle */
    if(RESET != hals_usrt_interrupt_flag_get(usrt->periph, USART_INT_FLAG_TBE)) {
        if(NULL != usrt->usrt_irq.transmit_ready_handle) {
            usrt->usrt_irq.transmit_ready_handle(usrt);
        }
        return;
    }

    /* transmission complete interrupt handle */
    if(RESET != hals_usrt_interrupt_flag_get(usrt->periph, USART_INT_FLAG_TC)) {
        hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_TC);

        if(NULL != usrt->usrt_irq.transmit_complete_handle) {
            usrt->usrt_irq.transmit_complete_handle(usrt);
        }
        return;
    }
}

/*!
    \brief      set user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  usrt: usrt device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to usrt interrupt callback functions structure
                  The structure member can be assigned as following parameters:
      \arg        hal_irq_handle_cb function pointer: the function is user-defined,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
      \arg        NULL: The corresponding callback mechanism is out of use, and
                    disable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_usrt_irq_handle_set(hal_usrt_dev_struct *usrt,
                             hal_usrt_irq_struct *p_irq)
{
    /* set user-defined error interrupt callback */
    if(NULL != p_irq->error_handle) {
        usrt->usrt_irq.error_handle = p_irq->error_handle;
        hals_usrt_interrupt_enable(usrt->periph, USART_INT_ERR);
        hals_usrt_interrupt_enable(usrt->periph, USART_INT_PERR);
    } else {
        usrt->usrt_irq.error_handle = NULL;
        hals_usrt_interrupt_disable(usrt->periph, USART_INT_ERR);
        hals_usrt_interrupt_disable(usrt->periph, USART_INT_PERR);
    }

    /* set user-defined receive complete interrupt callback */
    if(NULL != p_irq->receive_complete_handle) {
        usrt->usrt_irq.receive_complete_handle = p_irq->receive_complete_handle;
        hals_usrt_interrupt_enable(usrt->periph, USART_INT_RBNE);
    } else {
        usrt->usrt_irq.receive_complete_handle = NULL;
        hals_usrt_interrupt_disable(usrt->periph, USART_INT_RBNE);
    }

    /* set user-defined transmit complete interrupt callback */
    if(NULL != p_irq->transmit_complete_handle) {
        usrt->usrt_irq.transmit_complete_handle = p_irq->transmit_complete_handle;
        hals_usrt_interrupt_enable(usrt->periph, USART_INT_TC);
    } else {
        usrt->usrt_irq.transmit_complete_handle = NULL;
        hals_usrt_interrupt_disable(usrt->periph, USART_INT_TC);
    }

    /* set user-defined transmit ready interrupt callback */
    if(NULL != p_irq->transmit_ready_handle) {
        usrt->usrt_irq.transmit_ready_handle = p_irq->transmit_ready_handle;
        hals_usrt_interrupt_enable(usrt->periph, USART_INT_TBE);
    } else {
        usrt->usrt_irq.transmit_ready_handle = NULL;
        hals_usrt_interrupt_disable(usrt->periph, USART_INT_TBE);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_usrt_irq_handle_all_reset(hal_usrt_dev_struct *usrt)
{
    usrt->usrt_irq.error_handle             = NULL;
    usrt->usrt_irq.receive_complete_handle  = NULL;
    usrt->usrt_irq.transmit_complete_handle = NULL;
    usrt->usrt_irq.transmit_ready_handle    = NULL;
}

/*!
    \brief      transmit amounts of data, poll transmit process and completed status
                the function is blocking
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY, HAL_ERR_TIMEOUT details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_transmit_poll(hal_usrt_dev_struct *usrt, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms)
{
    uint8_t data_length;
    uint32_t tick_start;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [usrt] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the tx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->tx_state) {
        HAL_DEBUGE("usrt tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock usrt */
    HAL_LOCK(usrt);

    usrt->error_state       = HAL_USRT_ERROR_NONE;
    usrt->tx_state          = USRT_STATE_BUSY;

    /* initialize transmit parameters */
    usrt->txbuffer.buffer   = (uint8_t *)p_buffer;
    usrt->txbuffer.length   = length;
    usrt->txbuffer.pos      = 0U;

    /* calculate the data length */
    data_length = 1U;
    if(RESET != (USART_CTL0(usrt->periph) & USART_CTL0_WL)) {
        if(RESET == (USART_CTL0(usrt->periph) & USART_CTL0_PCEN)) {
            data_length = 2U;
        }
    }

    /* configure timeout */
    tick_start = hal_sys_basetick_count_get();

    while(usrt->txbuffer.pos < usrt->txbuffer.length) {
        /* wait for transmit buffer empty */
        while(RESET == hals_usrt_flag_get(usrt->periph, USART_FLAG_TBE)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    HAL_DEBUGW("usrt transmit timeout");
                    /* reset the state */
                    usrt->rx_state = USRT_STATE_FREE;
                    /* unlock usrt */
                    HAL_UNLOCK(usrt);
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* send the data to be transmitted */
        if(2U == data_length) {
            hals_usrt_data_transmit(usrt->periph, (*(uint16_t *)usrt->txbuffer.buffer & (uint16_t)0x1FFU));
            usrt->txbuffer.buffer += 2;
        } else {
            hals_usrt_data_transmit(usrt->periph, (*usrt->txbuffer.buffer & (uint8_t)0xFFU));
            usrt->txbuffer.buffer++;
        }

        /* change the transmit pointer */
        usrt->txbuffer.pos++;
    }

    /* wait for transmit complete */
    while(RESET == hals_usrt_flag_get(usrt->periph, USART_FLAG_TC)) {
        if(HAL_TIMEOUT_FOREVER != timeout_ms) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                HAL_DEBUGW("usrt transmit timeout");
                /* reset the state */
                usrt->rx_state = USRT_STATE_FREE;
                /* unlock usrt */
                HAL_UNLOCK(usrt);
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    /* change the Tx state to free */
    usrt->tx_state = USRT_STATE_FREE;

    /* unlock usrt */
    HAL_UNLOCK(usrt);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data, poll receive process and completed status
                the function is blocking
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY, HAL_ERR_TIMEOUT details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_receive_poll(hal_usrt_dev_struct *usrt, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms)
{
    uint8_t data_length;
    uint32_t tick_start;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [usrt] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the rx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->rx_state) {
        HAL_DEBUGE("usrt rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock usrt */
    HAL_LOCK(usrt);

    usrt->rx_state = USRT_STATE_BUSY;
    usrt->error_state = HAL_USRT_ERROR_NONE;

    /* initialize receive parameters */
    usrt->rxbuffer.buffer   = (uint8_t *)p_buffer;
    usrt->rxbuffer.length   = length;
    usrt->rxbuffer.pos      = 0U;
    usrt->data_bit_mask     = _usrt_data_bit_mask_get(usrt);

    /* calculate the data length */
    data_length = 1U;
    if(RESET != (USART_CTL0(usrt->periph) & USART_CTL0_WL)) {
        if(RESET == (USART_CTL0(usrt->periph) & USART_CTL0_PCEN)) {
            data_length = 2U;
        }
    }

    /* configure timeout */
    tick_start = hal_sys_basetick_count_get();

    while(usrt->rxbuffer.pos < usrt->rxbuffer.length) {
        /* send dummy byte to generate clock for the slave to send data */
        while(RESET == hals_usrt_flag_get(usrt->periph, USART_FLAG_TC)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    HAL_DEBUGW("usrt transmit timeout");
                    /* reset the state */
                    usrt->rx_state = USRT_STATE_FREE;
                    /* unlock usrt */
                    HAL_UNLOCK(usrt);
                    return HAL_ERR_TIMEOUT;
                }
                hals_usrt_data_transmit(usrt->periph, USRT_DUMMY_DATA);
            }
        }

        /* reconfigure the timeout */
        tick_start = hal_sys_basetick_count_get();
        /* wait for read data buffer not empty */
        while(RESET == hals_usrt_flag_get(usrt->periph, USART_FLAG_RBNE)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    HAL_DEBUGW("usrt receive timeout");
                    /* reset the state */
                    usrt->rx_state = USRT_STATE_FREE;
                    /* unlock usrt */
                    HAL_UNLOCK(usrt);
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* read data from data register */
        if(2U == data_length) {
            *usrt->rxbuffer.buffer = (uint8_t)(hals_usrt_data_receive(usrt->periph) & usrt->data_bit_mask);
            usrt->rxbuffer.buffer++;
        } else {
            *(uint16_t *)usrt->rxbuffer.buffer = (hals_usrt_data_receive(usrt->periph) & usrt->data_bit_mask);
            usrt->rxbuffer.buffer += 2;
        }
        /* change the receive pointer */
        usrt->rxbuffer.pos++;
    }

    /* change the Rx state to free */
    usrt->rx_state = USRT_STATE_FREE;

    /* unlock usrt */
    HAL_UNLOCK(usrt);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit & receive amounts of data, poll transfer process and completed status
                the function is blocking
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_tx_buffer: pointer to Tx data buffer
    \param[in]  p_rx_buffer: pointer to Rx data buffer
    \param[in]  length: number of data to be received
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_transmit_receive_poll(hal_usrt_dev_struct *usrt, uint8_t *p_tx_buffer, uint8_t *p_rx_buffer, uint32_t length, uint32_t timeout_ms)
{
    uint8_t data_length;
    uint32_t tick_start;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_tx_buffer) || (NULL == p_rx_buffer) ||
            (0U == length)) {
        HAL_DEBUGE("parameter [usrt] or [p_tx_buffer] or [p_rx_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* 9-bit transfer with no parity, if tx or rx buffer address is not aligned on uint16_t, return error */
    if((RESET != (USART_CTL0(usrt->periph) & USART_CTL0_WL)) && \
            (RESET == (USART_CTL0(usrt->periph) & USART_CTL0_PCEN))) {
        if(RESET != (((uint32_t)p_tx_buffer || (uint32_t)p_rx_buffer) & 1U)) {
            return HAL_ERR_ADDRESS;
        }
    }

    /* check the rx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->rx_state) {
        HAL_DEBUGE("usrt rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock usrt */
    HAL_LOCK(usrt);

    /* initialize receive parameters */
    usrt->error_state = HAL_USRT_ERROR_NONE;
    usrt->rx_state = USRT_STATE_BUSY;
    usrt->rxbuffer.buffer   = (uint8_t *)p_rx_buffer;
    usrt->rxbuffer.length   = length;
    usrt->rxbuffer.pos      = 0U;
    usrt->data_bit_mask     = _usrt_data_bit_mask_get(usrt);

    /* initialize transmit parameters */
    usrt->tx_state = USRT_STATE_BUSY;
    usrt->txbuffer.buffer = (uint8_t *)p_tx_buffer;
    usrt->txbuffer.length = length;

    /* calculate the data length */
    data_length = 1;
    if(RESET != (USART_CTL0(usrt->periph) & USART_CTL0_WL)) {
        if(RESET == (USART_CTL0(usrt->periph) & USART_CTL0_PCEN)) {
            data_length = 2U;
        }
    }

    /* configure timeout */
    tick_start = hal_sys_basetick_count_get();

    while(usrt->rxbuffer.pos < usrt->rxbuffer.length) {
        /* wait the TBE flag is set */
        while(RESET == hals_usrt_flag_get(usrt->periph, USART_FLAG_TBE)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    HAL_DEBUGW("usrt TBE flag set timeout");
                    /* reset the state */
                    usrt->tx_state = USRT_STATE_FREE;
                    /* unlock usrt */
                    HAL_UNLOCK(usrt);
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* send the data to be transmitted */
        if(2U == data_length) {
            hals_usrt_data_transmit(usrt->periph, (*(uint16_t *)usrt->txbuffer.buffer & usrt->data_bit_mask));
            usrt->txbuffer.buffer += 2;
        } else {
            hals_usrt_data_transmit(usrt->periph, (*usrt->txbuffer.buffer & (uint8_t)usrt->data_bit_mask));
            usrt->txbuffer.buffer++;
        }

        /* wait for transmit complete */
        while(RESET == hals_usrt_flag_get(usrt->periph, USART_FLAG_TC)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    HAL_DEBUGW("usrt transmit timeout");
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* wait the RBNE flag is set */
        while(RESET == hals_usrt_flag_get(usrt->periph, USART_FLAG_RBNE)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    HAL_DEBUGW("usrt RBNE flag set timeout");
                    /* reset the state */
                    usrt->rx_state = USRT_STATE_FREE;
                    /* unlock usrt */
                    HAL_UNLOCK(usrt);
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* read data from data register */
        if(2U == data_length) {
            *(uint16_t *)usrt->rxbuffer.buffer = (hals_usrt_data_receive(usrt->periph) & usrt->data_bit_mask);
            usrt->rxbuffer.buffer += 2;
        } else {
            *usrt->rxbuffer.buffer = (uint8_t)(hals_usrt_data_receive(usrt->periph) & usrt->data_bit_mask);
            usrt->rxbuffer.buffer++;
        }

        /* change the receive pointer */
        usrt->rxbuffer.pos++;
    }

    /* change the Tx and Rx state to free */
    usrt->tx_state = USRT_STATE_FREE;
    usrt->rx_state = USRT_STATE_FREE;

    /* unlock usrt */
    HAL_UNLOCK(usrt);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by interrupt method
                the function is non-blocking
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_transmit_interrupt(hal_usrt_dev_struct *usrt, uint8_t *p_buffer, uint32_t length, hal_usrt_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [usrt] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the tx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->tx_state) {
        HAL_DEBUGE("usrt tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock usrt */
    HAL_LOCK(usrt);

    usrt->tx_state = USRT_STATE_BUSY;
    usrt->error_state = HAL_USRT_ERROR_NONE;

    /* initialize transmit parameters */
    usrt->txbuffer.buffer   = (uint8_t *)p_buffer;
    usrt->txbuffer.length   = length;
    usrt->txbuffer.pos      = 0U;
    usrt->tx_callback       = (void *)p_user_func;

    /* configure the transmit ready and complete callback as the function implemented */
    usrt->usrt_irq.transmit_ready_handle = _usrt_transmit_interrupt;
    usrt->usrt_irq.transmit_complete_handle = _usrt_transmit_complete_interrupt;

    /* clear USRT TC interrupt flag */
    hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_TC);

    /* enable the TBE interrupt */
    hals_usrt_interrupt_enable(usrt->periph, USART_INT_TBE);

    /* unlock uart */
    HAL_UNLOCK(usrt);

    hals_uart_enable(usrt->periph);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by interrupt method
                the function is non-blocking
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_receive_interrupt(hal_usrt_dev_struct *usrt, uint8_t *p_buffer, uint32_t length, hal_usrt_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [usrt] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the rx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->rx_state) {
        HAL_DEBUGE("usrt rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock usrt */
    HAL_LOCK(usrt);

    usrt->rx_state = USRT_STATE_BUSY;
    usrt->error_state = HAL_USRT_ERROR_NONE;

    /* initialize receive parameters */
    usrt->rxbuffer.buffer   = (uint8_t *)p_buffer;
    usrt->rxbuffer.length   = length;
    usrt->rxbuffer.pos      = 0U;
    usrt->data_bit_mask     = _usrt_data_bit_mask_get(usrt);
    usrt->rx_callback       = (void *)p_user_func;
    usrt->usrt_irq.receive_complete_handle = _usrt_receive_interrupt;

    /* enable PERR, ERR, RBNE interrupt */
    hals_usrt_interrupt_enable(usrt->periph, USART_INT_PERR);
    hals_usrt_interrupt_enable(usrt->periph, USART_INT_ERR);
    hals_usrt_interrupt_enable(usrt->periph, USART_INT_RBNE);

    /* unlock usrt */
    HAL_UNLOCK(usrt);

    /* send dummy byte to generate clock for the slave to send data */
    hals_usrt_data_transmit(usrt->periph, USRT_DUMMY_DATA & _usrt_data_bit_mask_get(usrt));

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit & receive amounts of data by interrupt method
                the function is non-blocking
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_tx_buffer: pointer to Tx data buffer
    \param[in]  p_rx_buffer: pointer to Rx data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_transmit_receive_interrupt(hal_usrt_dev_struct *usrt, uint8_t *p_tx_buffer, uint8_t *p_rx_buffer, uint32_t length, hal_usrt_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_tx_buffer) || (NULL == p_rx_buffer) ||
            (0U == length)) {
        HAL_DEBUGE("parameter [usrt] or [p_tx_buffer] or [p_rx_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* 9-bit transfer with no parity, if tx or rx buffer address is not aligned on uint16_t, return error */
    if((RESET != (USART_CTL0(usrt->periph) & USART_CTL0_WL)) && \
            (RESET == (USART_CTL0(usrt->periph) & USART_CTL0_PCEN))) {
        if(RESET != (((uint32_t)p_tx_buffer || (uint32_t)p_rx_buffer) & 1U)) {
            return HAL_ERR_ADDRESS;
        }
    }

    /* check the rx_state wheher is in busy Tx Rx or not */
    if(USRT_STATE_BUSY_TX_RX == usrt->rx_state) {
        HAL_DEBUGE("usrt tx or rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock usrt */
    HAL_LOCK(usrt);

    usrt->error_state = HAL_USRT_ERROR_NONE;
    usrt->tx_state = USRT_STATE_BUSY_TX_RX;
    usrt->rx_state = USRT_STATE_BUSY_TX_RX;

    /* initialize transmit parameters */
    usrt->txbuffer.buffer   = (uint8_t *)p_tx_buffer;
    usrt->txbuffer.length   = length;
    usrt->txbuffer.pos      = 0U;
    usrt->usrt_irq.transmit_ready_handle = _usrt_tx_rx_interrupt;
    usrt->usrt_irq.transmit_complete_handle = _usrt_transmit_complete_interrupt;

    /* initialize receive parameters */
    usrt->rxbuffer.buffer   = (uint8_t *)p_rx_buffer;
    usrt->rxbuffer.length   = length;
    usrt->rxbuffer.pos      = 0U;
    usrt->data_bit_mask     = _usrt_data_bit_mask_get(usrt);
    usrt->rx_callback       = (void *)p_user_func;
    usrt->usrt_irq.receive_complete_handle = _usrt_tx_rx_interrupt;

    /* clear USRT TC interrupt flag */
    hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_TC);

    /* enable PERR, ERR, RBNE, TBE interrupt */
    hals_usrt_interrupt_enable(usrt->periph, USART_INT_PERR);
    hals_usrt_interrupt_enable(usrt->periph, USART_INT_ERR);
    hals_usrt_interrupt_enable(usrt->periph, USART_INT_RBNE);
    hals_usrt_interrupt_enable(usrt->periph, USART_INT_TBE);

    /* unlock uart */
    HAL_UNLOCK(usrt);

    hals_uart_enable(usrt->periph);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by dma method
                the function is non-blocking
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_transmit_dma(hal_usrt_dev_struct *usrt, uint8_t *p_buffer, uint16_t length, hal_usrt_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [usrt] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check the parameter */
    if(NULL == usrt->p_dma_tx) {
        HAL_DEBUGE("parameter [usrt->p_dma_tx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the tx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->tx_state) {
        HAL_DEBUGE("usrt tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock usrt */
    HAL_LOCK(usrt);

    usrt->tx_state = USRT_STATE_BUSY;
    usrt->error_state = HAL_USRT_ERROR_NONE;

    /* initialize transmit parameters */
    usrt->txbuffer.buffer   = (uint8_t *)p_buffer;
    usrt->txbuffer.length   = length;
    usrt->txbuffer.pos      = 0U;

    if(NULL != p_func) {
        usrt->tx_callback = (void *)p_func->complete_func;
        usrt->usrt_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    } else {
        usrt->tx_callback = NULL;
        usrt->usrt_irq.error_handle = NULL;
    }
    usrt->usrt_irq.transmit_complete_handle = _usrt_transmit_complete_interrupt;

    /* configure DMA interrupt callback function */
    dma_irq.full_finish_handle = _usrt_transmit_dma;
    dma_irq.error_handle = _usrt_dma_error;
    if(NULL != usrt->p_dma_tx->dma_irq.half_finish_handle) {
        dma_irq.half_finish_handle = usrt->p_dma_tx->dma_irq.half_finish_handle;
    } else {
        dma_irq.half_finish_handle = NULL;
    }

    /* start DMA interrupt mode transfer */
    if(HAL_ERR_NONE != hal_dma_start_interrupt(usrt->p_dma_tx,
            (uint32_t)usrt->txbuffer.buffer,
            (uint32_t)&USART_TDATA(usrt->periph), usrt->txbuffer.length, &dma_irq)) {
        usrt->tx_state = USRT_STATE_FREE;
        usrt->error_state = HAL_USRT_ERROR_DMATX;
        /* unlock uart */
        HAL_UNLOCK(usrt);
    }

    /* clear USRT TC interrupt flag */
    hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_TC);

    /* DMA enable for transmission */
    hals_usrt_dma_transmit_config(usrt->periph, USART_TRANSMIT_DMA_ENABLE);

    /* unlock uart */
    HAL_UNLOCK(usrt);

    hals_uart_enable(usrt->periph);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by dma method
                the function is non-blocking
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_receive_dma(hal_usrt_dev_struct *usrt, uint8_t *p_buffer, uint16_t length, hal_usrt_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [usrt] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check the parameter */
    if(NULL == usrt->p_dma_rx) {
        HAL_DEBUGE("parameter [usrt->p_dma_rx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the rx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->rx_state) {
        HAL_DEBUGE("usrt rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock usrt */
    HAL_LOCK(usrt);

    usrt->rx_state = USRT_STATE_BUSY;
    usrt->error_state = HAL_USRT_ERROR_NONE;

    /* initialize receive parameters */
    usrt->rxbuffer.buffer   = (uint8_t *)p_buffer;
    usrt->rxbuffer.length   = length;
    usrt->rxbuffer.pos      = 0U;
    usrt->data_bit_mask     = _usrt_data_bit_mask_get(usrt);

    if(NULL != p_func) {
        usrt->rx_callback = (void *)p_func->complete_func;
        usrt->usrt_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    } else {
        usrt->rx_callback = NULL;
        usrt->usrt_irq.error_handle = NULL;
    }

    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _usrt_receive_dma;
    dma_irq.error_handle = _usrt_dma_error;
    if(NULL != usrt->p_dma_rx->dma_irq.half_finish_handle) {
        dma_irq.half_finish_handle = usrt->p_dma_rx->dma_irq.half_finish_handle;
    }

    /* start DMA interrupt mode transfer */
    if(HAL_ERR_NONE != hal_dma_start_interrupt(usrt->p_dma_rx,
            (uint32_t)&USART_RDATA(usrt->periph),
            (uint32_t)usrt->rxbuffer.buffer, length, &dma_irq)) {
        usrt->rx_state = USRT_STATE_FREE;
        usrt->error_state = HAL_USRT_ERROR_DMARX;
        /* lock uart */
        HAL_LOCK(usrt);
    }

    /* enable the usart parity error and error interrupt: (frame error, noise error, overrun error) */
    hals_usrt_interrupt_enable(usrt->periph, USART_INT_PERR);
    hals_usrt_interrupt_enable(usrt->periph, USART_INT_ERR);

    /* usrt transmit data by DMA to generate clock */
    dma_irq.full_finish_handle = NULL;
    dma_irq.half_finish_handle = NULL;
    dma_irq.error_handle = NULL;

    /* start transmit DMA interrupt mode transfer */
    if(HAL_ERR_NONE != hal_dma_start_interrupt(usrt->p_dma_tx, (uint32_t)usrt->txbuffer.buffer, (uint32_t)&USART_TDATA(usrt->periph), usrt->txbuffer.length, &dma_irq)) {
        usrt->rx_state = USRT_STATE_FREE;
        usrt->error_state = HAL_USRT_ERROR_DMATX;
        /* unlock uart */
        HAL_UNLOCK(usrt);
    }

    /* DMA enable for reception and transmission */
    hals_usrt_dma_receive_config(usrt->periph, USART_RECEIVE_DMA_ENABLE);
    hals_usrt_dma_transmit_config(usrt->periph, USART_TRANSMIT_DMA_ENABLE);

    /* unlock uart */
    HAL_UNLOCK(usrt);

    hals_uart_enable(usrt->periph);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit & receive amounts of data by dma method
                the function is non-blocking
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_tx_buffer: pointer to Tx data buffer
    \param[in]  p_rx_buffer: pointer to Rx data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_transmit_receive_dma(hal_usrt_dev_struct *usrt, uint8_t *p_tx_buffer, uint8_t *p_rx_buffer, uint16_t length, hal_usrt_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_tx_buffer) || (NULL == p_rx_buffer) ||
            (0U == length)) {
        HAL_DEBUGE("parameter [usrt] or [p_tx_buffer] or [p_rx_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check the parameter */
    if(NULL == usrt->p_dma_tx) {
        HAL_DEBUGE("parameter [usrt->p_dma_tx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the tx_state wheher is busy Tx Rx or not */
    if(USRT_STATE_BUSY_TX_RX == usrt->tx_state) {
        HAL_DEBUGE("usrt tx or rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock usrt */
    HAL_LOCK(usrt);

    usrt->rx_state      = USRT_STATE_BUSY_TX_RX;
    usrt->tx_state      = USRT_STATE_BUSY_TX_RX;
    usrt->error_state   = HAL_USRT_ERROR_NONE;

    /* initialize receive parameters */
    usrt->rxbuffer.buffer   = (uint8_t *)p_rx_buffer;
    usrt->rxbuffer.length   = length;
    usrt->rxbuffer.pos      = 0U;
    usrt->data_bit_mask     = _usrt_data_bit_mask_get(usrt);

    if(NULL != p_func) {
        usrt->rx_callback = (void *)p_func->complete_func;
        usrt->usrt_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    } else {
        usrt->rx_callback = NULL;
        usrt->usrt_irq.error_handle = NULL;
    }

    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _usrt_receive_dma;
    dma_irq.error_handle = _usrt_dma_error;
    if(NULL != usrt->p_dma_rx->dma_irq.half_finish_handle) {
        dma_irq.half_finish_handle = usrt->p_dma_rx->dma_irq.half_finish_handle;
    } else {
        dma_irq.half_finish_handle = NULL;
    }

    /* start DMA interrupt mode transfer */
    if(HAL_ERR_NONE != hal_dma_start_interrupt(usrt->p_dma_rx,
            (uint32_t)&USART_RDATA(usrt->periph),
            (uint32_t)usrt->rxbuffer.buffer, length, &dma_irq)) {
        usrt->rx_state = USRT_STATE_FREE;
        usrt->error_state = HAL_USRT_ERROR_DMARX;
        /* lock uart */
        HAL_LOCK(usrt);
    }

    /* enable the usart parity error and error interrupt: (frame error, noise error, overrun error) */
    hals_usrt_interrupt_enable(usrt->periph, USART_INT_PERR);
    hals_usrt_interrupt_enable(usrt->periph, USART_INT_ERR);

    /* initialize transmit parameters */
    usrt->txbuffer.buffer   = (uint8_t *)p_tx_buffer;
    usrt->txbuffer.length   = length;
    usrt->txbuffer.pos      = 0U;

    if(NULL != p_func) {
        usrt->tx_callback = (void *)p_func->complete_func;
    } else {
        usrt->tx_callback = NULL;
    }
    usrt->usrt_irq.transmit_complete_handle = _usrt_transmit_complete_interrupt;

    /* configure DMA interrupt callback function */
    dma_irq.full_finish_handle = _usrt_transmit_dma;
    dma_irq.error_handle = _usrt_dma_error;
    if(NULL != usrt->p_dma_tx->dma_irq.half_finish_handle) {
        dma_irq.half_finish_handle = usrt->p_dma_tx->dma_irq.half_finish_handle;
    } else {
        dma_irq.half_finish_handle = NULL;
    }

    /* start transmit DMA interrupt mode transfer */
    if(HAL_ERR_NONE != hal_dma_start_interrupt(usrt->p_dma_tx, (uint32_t)usrt->txbuffer.buffer, (uint32_t)&USART_TDATA(usrt->periph), usrt->txbuffer.length, &dma_irq)) {
        usrt->rx_state = USRT_STATE_FREE;
        usrt->error_state = HAL_USRT_ERROR_DMATX;
        /* unlock uart */
        HAL_UNLOCK(usrt);
    }

    /* clear USRT TC flag */
    hals_usrt_flag_clear(usrt->periph, USART_FLAG_TC);

    /* DMA enable for reception and transmission */
    hals_usrt_dma_receive_config(usrt->periph, USART_RECEIVE_DMA_ENABLE);
    hals_usrt_dma_transmit_config(usrt->periph, USART_TRANSMIT_DMA_ENABLE);

    /* lock uart */
    HAL_LOCK(usrt);

    hals_uart_enable(usrt->periph);

    return HAL_ERR_NONE;
}

/*!
    \brief      pause usrt DMA transfer during transmission process
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_dma_pause(hal_usrt_dev_struct *usrt)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == usrt) {
        HAL_DEBUGE("parameter [usrt] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock usrt */
    HAL_LOCK(usrt);

    /* check the tx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->tx_state) {
        /* disable DMA transimt */
        hals_usrt_dma_transmit_config(usrt->periph, USART_TRANSMIT_DMA_DISABLE);
    }

    /* check the rx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->rx_state) {
        /* disable the PERR and ERR interrupt */
        hals_usrt_interrupt_disable(usrt->periph, USART_INT_PERR);
        hals_usrt_interrupt_disable(usrt->periph, USART_INT_ERR);

        /* disable DMA receive */
        hals_usrt_dma_receive_config(usrt->periph, USART_RECEIVE_DMA_DISABLE);
    }

    /* unlock usrt */
    HAL_UNLOCK(usrt);

    return HAL_ERR_NONE;
}

/*!
    \brief      resume usrt DMA transfer during transmission process
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_dma_resume(hal_usrt_dev_struct *usrt)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == usrt) {
        HAL_DEBUGE("parameter [usrt] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock usrt */
    HAL_LOCK(usrt);

    /* check the tx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->tx_state) {
        /* enable DMA transimt */
        hals_usrt_dma_transmit_config(usrt->periph, USART_TRANSMIT_DMA_ENABLE);
    }

    /* check the rx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->rx_state) {
        /* enable the PERR and ERR interrupt */
        hals_usrt_interrupt_enable(usrt->periph, USART_INT_PERR);
        hals_usrt_interrupt_enable(usrt->periph, USART_INT_ERR);

        /* enable DMA receive */
        hals_usrt_dma_receive_config(usrt->periph, USART_RECEIVE_DMA_ENABLE);
    }

    /* unlock usrt */
    HAL_UNLOCK(usrt);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop usrt transmit and receive transfer
                the function is blocking
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32f3x0_hal.h
*/
int32_t hal_usrt_transfer_stop(hal_usrt_dev_struct *usrt)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == usrt) {
        HAL_DEBUGE("parameter [usrt] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock usrt */
    HAL_LOCK(usrt);

    /* disable the TBE and TC interrupt */
    hals_usrt_interrupt_disable(usrt->periph, USART_INT_TBE);
    hals_usrt_interrupt_disable(usrt->periph, USART_INT_TC);

    /* disable the RBNE, PERR and ERR interrupt */
    hals_usrt_interrupt_disable(usrt->periph, USART_INT_RBNE);
    hals_usrt_interrupt_disable(usrt->periph, USART_INT_PERR);
    hals_usrt_interrupt_disable(usrt->periph, USART_INT_ERR);

    /* disable DMA transimt and stop DMA */
    hals_usrt_dma_transmit_config(usrt->periph, USART_TRANSMIT_DMA_DISABLE);
    hal_dma_stop(usrt->p_dma_tx);

    /* disable DMA receive and stop DMA */
    hals_usrt_dma_receive_config(usrt->periph, USART_RECEIVE_DMA_DISABLE);
    hal_dma_stop(usrt->p_dma_rx);

    /* reset the position and state */
    usrt->txbuffer.pos = 0;
    usrt->tx_state = USRT_STATE_FREE;
    usrt->rxbuffer.pos = 0;
    usrt->rx_state = USRT_STATE_FREE;

    /* clear interrupt error flags */
    hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_PERR);
    hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_ERR_FERR);
    hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_ERR_NERR);
    hals_usrt_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_ERR_ORERR);

    /* unlock usrt */
    HAL_UNLOCK(usrt);

    return HAL_ERR_NONE;
}

/*!
    \brief      reset USRT
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_usrt_deinit(uint32_t usrt_periph)
{
    switch(usrt_periph) {
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
    \brief      configure USRT baud rate value
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  baudval: baud rate value
    \param[out] none
    \retval     none
*/
void hals_usrt_baudrate_set(uint32_t usrt_periph, uint32_t baudval)
{
    uint32_t uclk = 0U, intdiv = 0U, fradiv = 0U, udiv = 0U;
    switch(usrt_periph) {
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
    if(USART_CTL0(usrt_periph) & USART_CTL0_OVSMOD) {
        /* oversampling by 8, configure the value of USART_BAUD */
        udiv = ((2U * uclk) + (baudval / 2U)) / baudval;
        intdiv = udiv & 0x0000fff0U;
        fradiv = (udiv >> 1U) & 0x00000007U;
        USART_BAUD(usrt_periph) = ((USART_BAUD_FRADIV | USART_BAUD_INTDIV) & (intdiv | fradiv));
    } else {
        /* oversampling by 16, configure the value of USART_BAUD */
        udiv = (uclk + (baudval / 2U)) / baudval;
        intdiv = udiv & 0x0000fff0U;
        fradiv = udiv & 0x0000000fU;
        USART_BAUD(usrt_periph) = ((USART_BAUD_FRADIV | USART_BAUD_INTDIV) & (intdiv | fradiv));
    }
}

/*!
    \brief      configure USRT parity
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  paritycfg: USRT parity configure
                only one parameter can be selected which is shown as below:
      \arg        USART_PM_NONE: no parity
      \arg        USART_PM_ODD: odd parity
      \arg        USART_PM_EVEN: even parity
    \param[out] none
    \retval     none
*/
void hals_usrt_parity_config(uint32_t usrt_periph, uint32_t paritycfg)
{
    /* disable USRT */
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);
    /* clear USART_CTL0 PM,PCEN bits */
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_PM | USART_CTL0_PCEN);
    /* configure USRT parity mode */
    USART_CTL0(usrt_periph) |= paritycfg;
}

/*!
    \brief      configure USRT word length
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  wlen: USRT word length configure
                only one parameter can be selected which is shown as below:
      \arg        USART_WL_8BIT: 8 bits
      \arg        USART_WL_9BIT: 9 bits
    \param[out] none
    \retval     none
*/
void hals_usrt_word_length_set(uint32_t usrt_periph, uint32_t wlen)
{
    /* disable USRT */
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);
    /* clear USART_CTL0 WL bit */
    USART_CTL0(usrt_periph) &= ~USART_CTL0_WL;
    /* configure USRT word length */
    USART_CTL0(usrt_periph) |= wlen;
}

/*!
    \brief      configure USRT stop bit length
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  stblen: USRT stop bit configure
                only one parameter can be selected which is shown as below:
      \arg        USART_STB_1BIT: 1 bit
      \arg        USART_STB_0_5BIT: 0.5bit
      \arg        USART_STB_2BIT: 2 bits
      \arg        USART_STB_1_5BIT: 1.5bit
    \param[out] none
    \retval     none
*/
void hals_usrt_stop_bit_set(uint32_t usrt_periph, uint32_t stblen)
{
    /* disable USRT */
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);
    /* clear USART_CTL1 STB bits */
    USART_CTL1(usrt_periph) &= ~USART_CTL1_STB;
    USART_CTL1(usrt_periph) |= stblen;
}

/*!
    \brief      enable USRT
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_usrt_enable(uint32_t usrt_periph)
{
    USART_CTL0(usrt_periph) |= USART_CTL0_UEN;
}

/*!
    \brief      disable USRT
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_usrt_disable(uint32_t usrt_periph)
{
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);
}

/*!
    \brief      configure USRT transmitter
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  txconfig: enable or disable USRT transmitter
                only one parameter can be selected which is shown as below:
      \arg        USART_TRANSMIT_ENABLE: enable USRT transmission
      \arg        USART_TRANSMIT_DISABLE: enable USRT transmission
    \param[out] none
    \retval     none
*/
void hals_usrt_transmit_config(uint32_t usrt_periph, uint32_t txconfig)
{
    USART_CTL0(usrt_periph) &= ~USART_CTL0_TEN;
    /* configure transfer mode */
    USART_CTL0(usrt_periph) |= txconfig;
}

/*!
    \brief      configure USRT receiver
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  rxconfig: enable or disable USRT receiver
                only one parameter can be selected which is shown as below:
      \arg        USART_RECEIVE_ENABLE: enable USRT reception
      \arg        USART_RECEIVE_DISABLE: disable USRT reception
    \param[out] none
    \retval     none
*/
void hals_usrt_receive_config(uint32_t usrt_periph, uint32_t rxconfig)
{
    USART_CTL0(usrt_periph) &= ~USART_CTL0_REN;
    /* configure receiver mode */
    USART_CTL0(usrt_periph) |= rxconfig;
}

/*!
    \brief      data is transmitted/received with the LSB/MSB first
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  msbf: LSB/MSB
                only one parameter can be selected which is shown as below:
      \arg        USART_MSBF_LSB: LSB first
      \arg        USART_MSBF_MSB: MSB first
    \param[out] none
    \retval     none
*/
void hals_usrt_data_first_config(uint32_t usrt_periph, uint32_t msbf)
{
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);
    /* configure LSB or MSB first */
    USART_CTL1(usrt_periph) &= ~(USART_CTL1_MSBF);
    USART_CTL1(usrt_periph) |= msbf;
}

/*!
    \brief      configure USRT inversion
    \param[in]  usrt_periph: USARTx(x=0,1)
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
void hals_usrt_invert_config(uint32_t usrt_periph, usart_invert_enum invertpara)
{
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);
    /* inverted or not the specified signal */
    switch(invertpara) {
    case USART_DINV_ENABLE:
        USART_CTL1(usrt_periph) |= USART_CTL1_DINV;
        break;
    case USART_DINV_DISABLE:
        USART_CTL1(usrt_periph) &= ~(USART_CTL1_DINV);
        break;
    case USART_TXPIN_ENABLE:
        USART_CTL1(usrt_periph) |= USART_CTL1_TINV;
        break;
    case USART_TXPIN_DISABLE:
        USART_CTL1(usrt_periph) &= ~(USART_CTL1_TINV);
        break;
    case USART_RXPIN_ENABLE:
        USART_CTL1(usrt_periph) |= USART_CTL1_RINV;
        break;
    case USART_RXPIN_DISABLE:
        USART_CTL1(usrt_periph) &= ~(USART_CTL1_RINV);
        break;
    case USART_SWAP_ENABLE:
        USART_CTL1(usrt_periph) |= USART_CTL1_STRP;
        break;
    case USART_SWAP_DISABLE:
        USART_CTL1(usrt_periph) &= ~(USART_CTL1_STRP);
        break;
    default:
        break;
    }
}

/*!
    \brief      enable the USRT overrun function
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_usrt_overrun_enable(uint32_t usrt_periph)
{
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);
    /* enable overrun function */
    USART_CTL2(usrt_periph) &= ~(USART_CTL2_OVRD);
}

/*!
    \brief      disable the USRT overrun function
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_usrt_overrun_disable(uint32_t usrt_periph)
{
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);
    /* disable overrun function */
    USART_CTL2(usrt_periph) |= USART_CTL2_OVRD;
}

/*!
    \brief      configure the USRT oversample mode
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  oversamp: oversample value
                only one parameter can be selected which is shown as below:
      \arg        USART_OVSMOD_8: oversampling by 8
      \arg        USART_OVSMOD_16: oversampling by 16
    \param[out] none
    \retval     none
*/
void hals_usrt_oversample_config(uint32_t usrt_periph, uint32_t oversamp)
{
    /* disable USRT */
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);
    /* clear OVSMOD bit */
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_OVSMOD);
    USART_CTL0(usrt_periph) |= oversamp;
}

/*!
    \brief      configure the sample bit method
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  osb: sample bit
                only one parameter can be selected which is shown as below:
      \arg        USART_OSB_1BIT: 1 bit
      \arg        USART_OSB_3BIT: 3 bits
    \param[out] none
    \retval     none
*/
void hals_usrt_sample_bit_config(uint32_t usrt_periph, uint32_t osb)
{
    /* disable USRT */
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(usrt_periph) &= ~(USART_CTL2_OSB);
    USART_CTL2(usrt_periph) |= osb;
}


/*!
    \brief      USRT transmit data function
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  data: data of transmission
    \param[out] none
    \retval     none
*/
void hals_usrt_data_transmit(uint32_t usrt_periph, uint16_t data)
{
    USART_TDATA(usrt_periph) = (USART_TDATA_TDATA & (uint32_t)data);
}

/*!
    \brief      USRT receive data function
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[out] none
    \retval     data of received
*/
uint16_t hals_usrt_data_receive(uint32_t usrt_periph)
{
    return (uint16_t)(GET_BITS(USART_RDATA(usrt_periph), 0U, 8U));
}

/*!
    \brief      enable USRT command
    \param[in]  usrt_periph: USARTx(x=0,1,2)
    \param[in]  cmdtype: command type
                only one parameter can be selected which is shown as below:
      \arg        USART_CMD_SBKCMD: send break command
      \arg        USART_CMD_MMCMD: mute mode command
      \arg        USART_CMD_RXFCMD: receive data flush command
      \arg        USART_CMD_TXFCMD: transmit data flush request
    \param[out] none
    \retval     none
*/
void hals_usrt_command_enable(uint32_t usrt_periph, uint32_t cmdtype)
{
    USART_CMD(usrt_periph) |= (cmdtype);
}

/*!
    \brief      enable USRT clock
    \param[in]  usrt_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_usrt_clock_enable(uint32_t usrt_periph)
{
    /* disable USRT */
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);

    USART_CTL1(usrt_periph) |= USART_CTL1_CKEN;
}

/*!
    \brief      disable USRT clock
    \param[in]  usrt_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_usrt_clock_disable(uint32_t usrt_periph)
{
    /* disable USRT */
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);

    USART_CTL1(usrt_periph) &= ~(USART_CTL1_CKEN);
}

/*!
    \brief      configure USART synchronous mode parameters
    \param[in]  usart_periph: USARTx(x=0,1)
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
void hals_usrt_synchronous_clock_config(uint32_t usrt_periph, uint32_t clen, uint32_t cph, uint32_t cpl)
{
    /* disable USART */
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);
    /* reset USART_CTL1 CLEN,CPH,CPL bits */
    USART_CTL1(usrt_periph) &= ~(USART_CTL1_CLEN | USART_CTL1_CPH | USART_CTL1_CPL);

    USART_CTL1(usrt_periph) |= (USART_CTL1_CLEN & clen);
    USART_CTL1(usrt_periph) |= (USART_CTL1_CPH & cph);
    USART_CTL1(usrt_periph) |= (USART_CTL1_CPL & cpl);
}

/*!
    \brief      configure USRT DMA reception
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  dmacmd: USRT DMA mode
                only one parameter can be selected which is shown as below:
      \arg        USART_RECEIVE_DMA_ENABLE: enable USRT DMA for reception
      \arg        USART_RECEIVE_DMA_DISABLE: disable USRT DMA for reception
    \param[out] none
    \retval     none
*/
void hals_usrt_dma_receive_config(uint32_t usrt_periph, uint8_t dmacmd)
{
    USART_CTL2(usrt_periph) &= ~USART_CTL2_DENR;
    USART_CTL2(usrt_periph) |= (USART_CTL2_DENR & dmacmd);
}

/*!
    \brief      configure USRT DMA transmission
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  dmacmd: USRT DMA mode
                only one parameter can be selected which is shown as below:
      \arg        USART_TRANSMIT_DMA_ENABLE: enable USRT DMA for transmission
      \arg        USART_TRANSMIT_DMA_DISABLE: disable USRT DMA for transmission
    \param[out] none
    \retval     none
*/
void hals_usrt_dma_transmit_config(uint32_t usrt_periph, uint8_t dmacmd)
{
    USART_CTL2(usrt_periph) &= ~USART_CTL2_DENT;
    USART_CTL2(usrt_periph) |= (USART_CTL2_DENT & dmacmd);
}

/*!
    \brief      enable DMA on reception error
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_usrt_reception_error_dma_enable(uint32_t usrt_periph)
{
    /* disable USRT */
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);
    USART_CTL2(usrt_periph) &= ~(USART_CTL2_DDRE);
}

/*!
    \brief      disable DMA on reception error
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_usrt_reception_error_dma_disable(uint32_t usrt_periph)
{
    /* disable USRT */
    USART_CTL0(usrt_periph) &= ~(USART_CTL0_UEN);
    USART_CTL2(usrt_periph) |= USART_CTL2_DDRE;
}

/*!
    \brief      enable receive FIFO
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_usrt_receive_fifo_enable(uint32_t usrt_periph)
{
    USART_RFCS(usrt_periph) |= USART_RFCS_RFEN;
}

/*!
    \brief      disable receive FIFO
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_usrt_receive_fifo_disable(uint32_t usrt_periph)
{
    USART_RFCS(usrt_periph) &= ~(USART_RFCS_RFEN);
}

/*!
    \brief      read receive FIFO counter number
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[out] none
    \retval     receive FIFO counter number
*/
uint8_t hals_usrt_receive_fifo_counter_number(uint32_t usrt_periph)
{
    return (uint8_t)(GET_BITS(USART_RFCS(usrt_periph), 12U, 14U));
}

/*!
    \brief      get USRT status
    \param[in]  usrt_periph: USARTx(x=0,1)
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
FlagStatus hals_usrt_flag_get(uint32_t usrt_periph, usart_flag_enum flag)
{
    if(RESET != (USART_REG_VAL(usrt_periph, flag) & BIT(USART_BIT_POS(flag)))) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear USRT status
    \param[in]  usrt_periph: USARTx(x=0,1)
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
void hals_usrt_flag_clear(uint32_t usrt_periph, usart_flag_enum flag)
{
    USART_INTC(usrt_periph) |= BIT(USART_BIT_POS(flag));
}

/*!
    \brief      enable USRT interrupt
    \param[in]  usrt_periph: USARTx(x=0,1)
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
void hals_usrt_interrupt_enable(uint32_t usrt_periph,
                                usart_interrupt_enum interrupt)
{
    USART_REG_VAL(usrt_periph, interrupt) |= BIT(USART_BIT_POS(interrupt));
}

/*!
    \brief      disable USRT interrupt
    \param[in]  usrt_periph: USARTx(x=0,1)
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
void hals_usrt_interrupt_disable(uint32_t usrt_periph,
                                 usart_interrupt_enum interrupt)
{
    USART_REG_VAL(usrt_periph, interrupt) &= ~BIT(USART_BIT_POS(interrupt));
}

/*!
    \brief      get USRT interrupt flag status
    \param[in]  usrt_periph: USARTx(x=0,1)
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
FlagStatus hals_usrt_interrupt_flag_get(uint32_t usrt_periph,
                                        usart_interrupt_flag_enum int_flag)
{
    uint32_t intenable = 0U, flagstatus = 0U;
    /* get the interrupt enable bit status */
    intenable = (USART_REG_VAL(usrt_periph,
                               int_flag) & BIT(USART_BIT_POS(int_flag)));
    /* get the corresponding flag bit status */
    flagstatus = (USART_REG_VAL2(usrt_periph,
                                 int_flag) & BIT(USART_BIT_POS2(int_flag)));

    if(flagstatus && intenable) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear USRT interrupt flag
    \param[in]  usrt_periph: USARTx(x=0,1)
    \param[in]  flag: USRT interrupt flag
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
void hals_usrt_interrupt_flag_clear(uint32_t usrt_periph,
                                    usart_interrupt_flag_enum int_flag)
{
    if(USART_INT_FLAG_RFF == int_flag) {
        USART_RFCS(usrt_periph) &= (uint32_t)(~USART_RFCS_RFFINT);
    } else {
        USART_INTC(usrt_periph) |= BIT(USART_BIT_POS2(int_flag));
    }
}

/*!
    \brief      get the mask of data bit
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the mask of data bit(0x7F, 0xFF, 0x1FF)
*/
static uint16_t _usrt_data_bit_mask_get(hal_usrt_dev_struct *usrt)
{
    uint16_t reval;

    if(RESET != (USART_CTL0(usrt->periph) & USART_CTL0_WL)) {
        /* check whether the PCEN is enabled */
        if(RESET != (USART_CTL0(usrt->periph) & USART_CTL0_PCEN)) {
            reval = 0xFFU;
        } else {
            reval = 0x1FFU;
        }
    } else {
        /* check whether the PCEN is enabled */
        if(RESET != (USART_CTL0(usrt->periph) & USART_CTL0_PCEN)) {
            reval = 0x7FU;
        } else {
            reval = 0xFFU;
        }
    }

    return reval;
}

/*!
    \brief      get usrt error flag
    \param[in]  usrt: usrt device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     SET or RESET
*/
static FlagStatus _usrt_error_flag_get(hal_usrt_dev_struct *usrt)
{
    if(0U == (USART_STAT(usrt->periph) & (uint32_t)(USART_STAT_PERR |
              USART_STAT_FERR | \
              USART_STAT_ORERR | USART_STAT_NERR))) {
        return RESET;
    } else {
        return SET;
    }
}

/*!
    \brief      handle the transmit complete interrupt
    \param[in]  usrt: pointer to a UART device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_transmit_complete_interrupt(void *usrt)
{
    hal_usrt_dev_struct *p_usrt = usrt;
    hal_usrt_user_cb p_func = (hal_usrt_user_cb)p_usrt->tx_callback;
    if(USRT_STATE_BUSY_TX_RX == p_usrt->tx_state) {
        /* in Tx Rx state, the callback function is called when receive complete event occurs */
        p_func = NULL;
    }

    /* disable the transmit complete interrupt */
    hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_TC);
    /* reset transmit_complete_handle and tx_state */
    p_usrt->usrt_irq.transmit_complete_handle = NULL;
    p_usrt->tx_state = USRT_STATE_FREE;

    if(NULL != p_func) {
        /* if there is a user transmit complete callback */
        p_func(p_usrt);
    }
}

/*!
    \brief      handle the transmit interrupt
    \param[in]  usrt: pointer to a UART device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_transmit_interrupt(void *usrt)
{
    uint32_t temp_val;
    hal_usrt_dev_struct *p_usrt = usrt;

    temp_val = p_usrt->txbuffer.pos;
    if(temp_val < p_usrt->txbuffer.length) {
        if((RESET != (USART_CTL0(p_usrt->periph) & USART_CTL0_WL)) && \
                (RESET == (USART_CTL0(p_usrt->periph) & USART_CTL0_PCEN))) {
            /* 9-bit data, none parity */
            hals_usrt_data_transmit(p_usrt->periph,
                                    (*(uint16_t *)p_usrt->txbuffer.buffer & (uint16_t)0x1FFU));
            p_usrt->txbuffer.buffer += 2U;
        } else {
            /* 9-bit data, with parity or 8-bit data */
            hals_usrt_data_transmit(p_usrt->periph,
                                    (*p_usrt->txbuffer.buffer & (uint8_t)0xFFU));
            p_usrt->txbuffer.buffer++;
        }
        p_usrt->txbuffer.pos++;
    } else {
        /* disable the TBE interrupt, enable the TC interrupt and reset the transmit_ready_handle */
        hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_TBE);
        hals_usrt_interrupt_enable(p_usrt->periph, USART_INT_TC);
        p_usrt->usrt_irq.transmit_ready_handle = NULL;
    }
}

/*!
    \brief      handle the receive interrupt
    \param[in]  usrt: pointer to a UART device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_receive_interrupt(void *usrt)
{
    hal_usrt_dev_struct *p_usrt = usrt;
    hal_usrt_user_cb p_func = (hal_usrt_user_cb)p_usrt->rx_callback;

    if(0x1FFU == p_usrt->data_bit_mask) {
        /* store the received data */
        *(uint16_t *)p_usrt->rxbuffer.buffer = (hals_usrt_data_receive(p_usrt->periph) &
                                                p_usrt->data_bit_mask);
        p_usrt->rxbuffer.buffer += 2U;
    } else {
        /* store the received data */
        *p_usrt->rxbuffer.buffer = (uint8_t)(hals_usrt_data_receive(p_usrt->periph) &
                                             p_usrt->data_bit_mask);
        p_usrt->rxbuffer.buffer++;
    }
    p_usrt->rxbuffer.pos++;

    if(p_usrt->rxbuffer.pos == p_usrt->rxbuffer.length) {
        /* disable PERR, ERR, RBNE interrupt */
        hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_PERR);
        hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_ERR);
        hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_RBNE);

        /* reset receive_complete_handle and rx_state */
        p_usrt->usrt_irq.receive_complete_handle = NULL;
        p_usrt->rx_state = USRT_STATE_FREE;

        if(NULL != p_func) {
            /* if there is a user receive complete callback */
            p_func(p_usrt);
        }
    }
}

/*!
    \brief      handle the transmit and receive interrupt
    \param[in]  usrt: pointer to a UART device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_tx_rx_interrupt(void *usrt)
{
    uint16_t recv_data;
    hal_usrt_dev_struct *p_usrt = usrt;

    if(USRT_STATE_BUSY_TX_RX == p_usrt->tx_state) {
        /* check whether the TBE is set or not */
        if(RESET != hals_usrt_flag_get(p_usrt->periph, USART_FLAG_TBE)) {
            if(p_usrt->txbuffer.pos < p_usrt->txbuffer.length) {
                if((RESET != (USART_CTL0(p_usrt->periph) & USART_CTL0_WL)) && \
                        (RESET == (USART_CTL0(p_usrt->periph) & USART_CTL0_PCEN))) {
                    /* 9-bit data, none parity */
                    hals_usrt_data_transmit(p_usrt->periph,
                                            (*(uint16_t *)p_usrt->txbuffer.buffer & (uint16_t)0x1FFU));
                    p_usrt->txbuffer.buffer += 2U;
                } else {
                    /* 9-bit data, with parity or 8-bit data */
                    hals_usrt_data_transmit(p_usrt->periph,
                                            (*p_usrt->txbuffer.buffer & (uint8_t)0xFFU));
                    p_usrt->txbuffer.buffer++;
                }
                p_usrt->txbuffer.pos++;
            } else {
                /* disable the TBE interrupt */
                hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_TBE);
                hals_usrt_interrupt_enable(p_usrt->periph, USART_INT_TC);
                p_usrt->usrt_irq.transmit_ready_handle = NULL;
            }
        }
    }
    if(USRT_STATE_BUSY_TX_RX == p_usrt->rx_state) {
        /* check whether the RBNE is set or not */
        if(RESET != hals_usrt_flag_get(p_usrt->periph, USART_FLAG_RBNE)) {
            recv_data = (hals_usrt_data_receive(p_usrt->periph) & p_usrt->data_bit_mask);
            if(0x1FFU == p_usrt->data_bit_mask) {
                /* 9-bit data, none parity */
                *(uint16_t *)p_usrt->rxbuffer.buffer = recv_data;
                p_usrt->rxbuffer.buffer += 2U;
            } else {
                /* 9-bit data, with parity or 8-bit data */
                *p_usrt->rxbuffer.buffer = (uint8_t)recv_data;
                p_usrt->rxbuffer.buffer++;
            }
            p_usrt->rxbuffer.pos++;
        }

        if(p_usrt->rxbuffer.pos == p_usrt->rxbuffer.length) {
            hal_usrt_user_cb p_func = (hal_usrt_user_cb)p_usrt->rx_callback;
            /* disable PERR, ERR, RBNE interrupt */
            hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_PERR);
            hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_ERR);
            hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_RBNE);
            /* reset receive_complete_handle and rx_state */
            p_usrt->usrt_irq.receive_complete_handle = NULL;
            p_usrt->rx_state = USRT_STATE_FREE;

            if(NULL != p_func) {
                /* if there is a user Tx Rx complete callback */
                p_func(p_usrt);
            }
        }
    }
}

/*!
    \brief      handle the usrt DMA transmit process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_transmit_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_usrt_dev_struct *p_usrt;

    p_dma = (hal_dma_dev_struct *)dma;
    p_usrt = (hal_usrt_dev_struct *)p_dma->p_periph;

    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        p_usrt->txbuffer.pos = p_usrt->txbuffer.length;
        hals_usrt_dma_transmit_config(p_usrt->periph, USART_TRANSMIT_DMA_DISABLE);
        /* enable TC interrupt */
        hals_usrt_interrupt_enable(p_usrt->periph, USART_INT_TC);
    }
}

/*!
    \brief      handle the usrt DMA receive process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_receive_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_usrt_dev_struct *p_usrt;
    hal_usrt_user_cb p_func;

    p_dma = (hal_dma_dev_struct *)dma;
    p_usrt = (hal_usrt_dev_struct *)p_dma->p_periph;
    p_func = (hal_usrt_user_cb)p_usrt->rx_callback;

    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        p_usrt->rxbuffer.pos = p_usrt->rxbuffer.length;
        /* disable DMA receive, PERR and ERR interrupt */
        hals_usrt_dma_receive_config(p_usrt->periph, USART_RECEIVE_DMA_DISABLE);
        hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_PERR);
        hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_ERR);
        /* reset rx_state */
        p_usrt->rx_state = USRT_STATE_FREE;
    }

    if(NULL != p_func) {
        /* if there is a user receive complete callback */
        p_func(p_usrt);
    }
}

/*!
    \brief      handle the UART DMA error process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_usrt_dev_struct *p_usrt;

    p_dma = (hal_dma_dev_struct *)dma;
    p_usrt = (hal_usrt_dev_struct *)p_dma->p_periph;

    if(USRT_STATE_BUSY == p_usrt->tx_state) {
        /* transmit state is busy */
        p_usrt->error_state |= HAL_USRT_ERROR_DMATX;
        p_usrt->last_error = HAL_USRT_ERROR_DMATX;
        p_usrt->txbuffer.pos = p_usrt->txbuffer.length;

        /* disable DMA transmit and reset tx_state */
        hals_usrt_dma_transmit_config(p_usrt->periph, USART_TRANSMIT_DMA_DISABLE);
        p_usrt->tx_state = USRT_STATE_FREE;
    } else if(USRT_STATE_BUSY == p_usrt->rx_state) {
        /* receive state is busy */
        p_usrt->error_state |= HAL_USRT_ERROR_DMARX;
        p_usrt->last_error = HAL_USRT_ERROR_DMARX;
        p_usrt->rxbuffer.pos = p_usrt->rxbuffer.length;

        /* disable DMA receive, PERR, ERR interrupt */
        hals_usrt_dma_receive_config(p_usrt->periph, USART_RECEIVE_DMA_DISABLE);
        hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_PERR);
        hals_usrt_interrupt_disable(p_usrt->periph, USART_INT_ERR);

        /* reset rx_state */
        p_usrt->rx_state = USRT_STATE_FREE;
    } else {
        HAL_DEBUGE("usrt processor fatal error: dma error exception due to run state");
    }

    if(p_usrt->usrt_irq.error_handle != NULL) {
        /* if there is a user error callback */
        p_usrt->usrt_irq.error_handle(p_usrt);
        p_usrt->error_state = HAL_USRT_ERROR_NONE;
    }
}
