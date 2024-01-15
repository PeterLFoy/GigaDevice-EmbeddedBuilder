/*!
    \file    gd32f3x0_hal_uart.c
    \brief   UART driver

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

static uint16_t _uart_data_bit_mask_get(hal_uart_dev_struct *uart);
static FlagStatus _uart_error_flag_get(hal_uart_dev_struct *uart);
static void _uart_transmit_complete_interrupt(void *uart);
static void _uart_transmit_interrupt(void *uart);
static void _uart_receive_interrupt(void *uart);
static void _uart_transmit_dma(void *dma);
static void _uart_receive_dma(void *dma);
static void _uart_dma_error(void *dma);

/*!
    \brief      initialize the uart structure with the default values
                note: this function must be called after the structure is created
    \param[in]  hal_struct_type: type of uart structure for initialization
      \arg        HAL_UART_INIT_STRUCT: initialization structure
      \arg        HAL_UART_DEV_STRUCT: device information structure
      \arg        HAL_UART_USER_CALLBCAK_STRUCT: user callback structure
      \arg        HAL_UART_IRQ_INIT_STRUCT: interrupt callback initialization structure
    \param[in]  p_struct: structure pointer
    \param[out] none
    \retval     none
*/
void hal_uart_struct_init(hal_uart_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct) {
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_UART_INIT_STRUCT:
        /* initialize uart initialization structure with the default values */
        ((hal_uart_init_struct *)p_struct)->work_mode = UART_WORK_MODE_ASYN;
        ((hal_uart_init_struct *)p_struct)->baudrate = 115200U;
        ((hal_uart_init_struct *)p_struct)->parity = UART_PARITY_NONE;
        ((hal_uart_init_struct *)p_struct)->word_length = UART_WORD_LENGTH_8BIT;
        ((hal_uart_init_struct *)p_struct)->stop_bit = UART_STOP_BIT_1;
        ((hal_uart_init_struct *)p_struct)->direction = UART_DIRECTION_RX_TX;
        ((hal_uart_init_struct *)p_struct)->over_sample = UART_OVER_SAMPLE_16;
        ((hal_uart_init_struct *)p_struct)->sample_method = UART_THREE_SAMPLE_BIT;
        ((hal_uart_init_struct *)p_struct)->hardware_flow = UART_HARDWARE_FLOW_NONE;
        ((hal_uart_init_struct *)p_struct)->rx_fifo_en = DISABLE;
        ((hal_uart_init_struct *)p_struct)->timeout_enable = DISABLE;
        ((hal_uart_init_struct *)p_struct)->timeout_value = 0U;
        ((hal_uart_init_struct *)p_struct)->first_bit_msb     = DISABLE;
        ((hal_uart_init_struct *)p_struct)->tx_rx_swap        = DISABLE;
        ((hal_uart_init_struct *)p_struct)->rx_level_invert   = DISABLE;
        ((hal_uart_init_struct *)p_struct)->tx_level_invert   = DISABLE;
        ((hal_uart_init_struct *)p_struct)->data_bit_invert   = DISABLE;
        ((hal_uart_init_struct *)p_struct)->overrun_disable   = DISABLE;
        ((hal_uart_init_struct *)p_struct)->rx_error_dma_stop = DISABLE;
        ((hal_uart_init_struct *)p_struct)->break_frame_length = UART_LIN_BREAK_DETECTION_10BIT;
        ((hal_uart_init_struct *)p_struct)->rs485_mode = DISABLE;
        ((hal_uart_init_struct *)p_struct)->de_polarity = UART_RS485_DE_POLARITY_HIGH;
        ((hal_uart_init_struct *)p_struct)->de_assertion_time = 0U;
        ((hal_uart_init_struct *)p_struct)->de_deassertion_time = 0U;
        ((hal_uart_init_struct *)p_struct)->wakeup_mode = UART_MULTIPROCESSOR_WAKEUP_IDLE;
        ((hal_uart_init_struct *)p_struct)->address = 0U;
        ((hal_uart_init_struct *)p_struct)->addr_length = UART_MULTIPROCESSOR_ADDRESS_4BIT;
        break;

    case HAL_UART_DEV_STRUCT:
        /* initialize uart device information structure with the default values */
        ((hal_uart_dev_struct *)p_struct)->periph = 0U;
        ((hal_uart_dev_struct *)p_struct)->uart_irq.receive_complete_handle = NULL;
        ((hal_uart_dev_struct *)p_struct)->uart_irq.receive_timeout_handle = NULL;
        ((hal_uart_dev_struct *)p_struct)->uart_irq.transmit_ready_handle = NULL;
        ((hal_uart_dev_struct *)p_struct)->uart_irq.transmit_complete_handle = NULL;
        ((hal_uart_dev_struct *)p_struct)->uart_irq.error_handle = NULL;
        ((hal_uart_dev_struct *)p_struct)->uart_irq.wakeup_handle = NULL;
        ((hal_uart_dev_struct *)p_struct)->uart_irq.idle_line_detected_handle = NULL;
        ((hal_uart_dev_struct *)p_struct)->uart_irq.address_match_handle = NULL;
        ((hal_uart_dev_struct *)p_struct)->uart_irq.lin_break_detected_handle = NULL;
        ((hal_uart_dev_struct *)p_struct)->uart_irq.cts_change_handle = NULL;
        ((hal_uart_dev_struct *)p_struct)->p_dma_rx = NULL;
        ((hal_uart_dev_struct *)p_struct)->p_dma_tx = NULL;
        ((hal_uart_dev_struct *)p_struct)->txbuffer.buffer = NULL;
        ((hal_uart_dev_struct *)p_struct)->txbuffer.length = 0U;
        ((hal_uart_dev_struct *)p_struct)->txbuffer.pos = 0U;
        ((hal_uart_dev_struct *)p_struct)->rxbuffer.buffer = NULL;
        ((hal_uart_dev_struct *)p_struct)->rxbuffer.length = 0U;
        ((hal_uart_dev_struct *)p_struct)->rxbuffer.pos = 0U;
        ((hal_uart_dev_struct *)p_struct)->data_bit_mask = 0U;
        ((hal_uart_dev_struct *)p_struct)->last_error = HAL_UART_ERROR_NONE;
        ((hal_uart_dev_struct *)p_struct)->error_state = HAL_UART_ERROR_NONE;
        ((hal_uart_dev_struct *)p_struct)->tx_state = UART_STATE_FREE;
        ((hal_uart_dev_struct *)p_struct)->rx_state = UART_STATE_FREE;
        ((hal_uart_dev_struct *)p_struct)->rx_callback = NULL;
        ((hal_uart_dev_struct *)p_struct)->tx_callback = NULL;
        ((hal_uart_dev_struct *)p_struct)->mutex = HAL_MUTEX_UNLOCKED;
        ((hal_uart_dev_struct *)p_struct)->priv = NULL;
        break;

    case HAL_UART_USER_CALLBCAK_STRUCT:
        /* initialize user callback structure with the default values */
        ((hal_uart_user_callback_struct *)p_struct)->complete_func = NULL;
        ((hal_uart_user_callback_struct *)p_struct)->error_func = NULL;
        break;

    case HAL_UART_IRQ_INIT_STRUCT:
        /* initialize interrupt callback structure with the default values */
        ((hal_uart_irq_struct *)p_struct)->address_match_handle = NULL;
        ((hal_uart_irq_struct *)p_struct)->cts_change_handle = NULL;
        ((hal_uart_irq_struct *)p_struct)->error_handle = NULL;
        ((hal_uart_irq_struct *)p_struct)->idle_line_detected_handle = NULL;
        ((hal_uart_irq_struct *)p_struct)->lin_break_detected_handle = NULL;
        ((hal_uart_irq_struct *)p_struct)->receive_complete_handle = NULL;
        ((hal_uart_irq_struct *)p_struct)->receive_timeout_handle = NULL;
        ((hal_uart_irq_struct *)p_struct)->transmit_complete_handle = NULL;
        ((hal_uart_irq_struct *)p_struct)->transmit_ready_handle = NULL;
        ((hal_uart_irq_struct *)p_struct)->wakeup_handle = NULL;
        break;
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize the uart
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_uart_deinit(hal_uart_dev_struct *uart)
{
    uint32_t periph;

    periph = uart->periph;
    if((USART0 == periph) || (USART1 == periph)) {
        /* deinitialize the periph and the device information sturcture */
        hals_uart_deinit(periph);
        hal_uart_struct_init(HAL_UART_DEV_STRUCT, uart);
        uart->periph = periph;
    } else {
        HAL_DEBUGE("parameter [uart->periph] value is invalid");
    }
}

/*!
    \brief      initialize uart
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which UART is initialized
    \param[in]  p_init: the initialization data needed to initialize uart
                  work_mode: UART_WORK_MODE_ASYN, UART_WORK_MODE_SINGLE_WIRE, UART_WORK_MODE_MULTIPROCESSCOR
                             UART_WORK_MODE_LIN
                  baudrate: communication baudrate
                  parity: UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD
                  word_length: UART_WORD_LENGTH_8BIT, UART_WORD_LENGTH_9BIT
                  stop_bit: UART_STOP_BIT_1, UART_STOP_BIT_0_5, UART_STOP_BIT_2, UART_STOP_BIT_1_5
                  over_sample: UART_OVER_SAMPLE_8, UART_OVER_SAMPLE_16
                  sample_method: UART_THREE_SAMPLE_BIT, UART_ONE_SAMPLE_BIT
                  direction: UART_DIRECTION_RX_TX, UART_DIRECTION_RX_ONLY, UART_DIRECTION_TX_ONLY
                  hardware_flow: UART_HARDWARE_FLOW_NONE, UART_HARDWARE_FLOW_RTS_ONLY, UART_HARDWARE_FLOW_CTS_ONLY
                                UART_HARDWARE_FLOW_RTS_CTS
                  rx_fifo_en：DISABLE, ENABLE
                  timeout_enable: DISABLE, ENABLE
                  timeout_value: 0 - (2^24-1)
                  first_bit_msb: DISABLE, ENABLE
                  tx_rx_swap: DISABLE, ENABLE
                  rx_level_invert: DISABLE, ENABLE
                  tx_level_invert: DISABLE, ENABLE
                  data_bit_invert: DISABLE, ENABLE
                  overrun_disable: DISABLE, ENABLE
                  rx_error_dma_stop: DISABLE, ENABLE
                  lin_mode: (USART0 only)
                    break_frame_length: UART_LIN_BREAK_DETECTION_10BIT, UART_LIN_BREAK_DETECTION_11BIT
                  rs485_mode:
                    rs485_mode: DISABLE, ENABLE
                    de_polarity: UART_RS485_DE_POLARITY_HIGH, UART_RS485_DE_POLARITY_LOW
                    de_assertion_time: 0 - 31
                    de_deassertion_time: 0 - 31
                  multiprocessor_mode:
                    wakeup_mode: UART_MULTIPROCESSOR_WAKEUP_IDLE, UART_MULTIPROCESSOR_WAKEUP_ADDRESS
                    address: 0 - 15(4-bit address detection), 0 - 255(full-bit address detection)
                    addr_length: UART_MULTIPROCESSOR_ADDRESS_4BIT, UART_MULTIPROCESSOR_ADDRESS_FULLBIT
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32f3x0_hal.h
*/
int32_t hal_uart_init(hal_uart_dev_struct *uart, uint32_t periph, hal_uart_init_struct *p_init)
{
    uint32_t reg_temp;

#if (1 == HAL_PARAMETER_CHECK)
    /* check uart pointer and p_init address */
    if((NULL == uart) || (NULL == p_init)) {
        HAL_DEBUGE("pointer [uart] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check periph parameter */
    if((USART0 != periph) && (USART1 != periph)) {
        HAL_DEBUGE("parameter [periph] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check periph value from uart device struct */
    if(0 != uart->periph) {
        HAL_DEBUGI("periph value from uart device struct has been rewrite");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock uart */
    HAL_LOCK(uart);

    uart->periph    = periph;

    /* disable uart */
    hals_uart_disable(periph);

    /* configure CTL0 register */
    reg_temp = USART_CTL0(periph);
    reg_temp &= ~(USART_CTL0_PM | USART_CTL0_PCEN | USART_CTL0_WL | USART_CTL0_OVSMOD | USART_CTL0_REN | USART_CTL0_TEN);
    reg_temp |= (p_init->direction | p_init->over_sample | p_init->parity | p_init->word_length);
    USART_CTL0(periph) = reg_temp;

    /* configure CTL1 register */
    reg_temp = USART_CTL1(periph);
    reg_temp &= ~(USART_CTL1_STB);
    reg_temp |= (p_init->stop_bit);
    USART_CTL1(periph) = reg_temp;

    /* configure timeout enable */
    if(ENABLE == p_init->timeout_enable) {
        USART_CTL1(periph) |= USART_CTL1_RTEN;
    } else {
        USART_CTL1(periph) &= ~(USART_CTL1_RTEN);
    }
    /* configure the data bit invert function */
    if(ENABLE == p_init->data_bit_invert) {
        USART_CTL1(periph) |= USART_CTL1_DINV;
    } else {
        USART_CTL1(periph) &= ~(USART_CTL1_DINV);
    }
    /* configure the rx level invert function */
    if(ENABLE == p_init->rx_level_invert) {
        USART_CTL1(periph) |= USART_CTL1_RINV;
    } else {
        USART_CTL1(periph) &= ~(USART_CTL1_RINV);
    }
    /* configure the tx level invert function */
    if(ENABLE == p_init->tx_level_invert) {
        USART_CTL1(periph) |= USART_CTL1_TINV;
    } else {
        USART_CTL1(periph) &= ~(USART_CTL1_TINV);
    }
    /* configure the tx rx swap function */
    if(ENABLE == p_init->tx_rx_swap) {
        USART_CTL1(periph) |= USART_CTL1_STRP;
    } else {
        USART_CTL1(periph) &= ~(USART_CTL1_STRP);
    }
    /* configure the first bit MSB function */
    if(ENABLE == p_init->first_bit_msb) {
        USART_CTL1(periph) |= USART_CTL1_MSBF;
    } else {
        USART_CTL1(periph) &= ~(USART_CTL1_MSBF);
    }

    /* configure CTL2 register */
    reg_temp = USART_CTL2(periph);
    reg_temp &= ~(USART_CTL2_CTSEN | USART_CTL2_RTSEN | USART_CTL2_OSB);
    reg_temp |= (p_init->hardware_flow | p_init->sample_method);
    USART_CTL2(periph) = reg_temp;
    /* configure the overrun function */
    if(ENABLE == p_init->overrun_disable) {
        USART_CTL2(periph) |= USART_CTL2_OVRD;
    } else {
        USART_CTL2(periph) &= ~(USART_CTL2_OVRD);
    }
    /* configure the rx error DMA stop function */
    if(ENABLE == p_init->rx_error_dma_stop) {
        USART_CTL2(periph) |= USART_CTL2_DDRE;
    } else {
        USART_CTL2(periph) &= ~(USART_CTL2_DDRE);
    }

    /* configure RT register */
    reg_temp = USART_RT(periph);
    reg_temp &= ~USART_RT_RT;
    if(ENABLE == p_init->timeout_enable) {
        reg_temp |= p_init->timeout_value;
    }
    USART_RT(periph) = reg_temp;

    /* configure RFCS register */
    if(ENABLE == p_init->rx_fifo_en) {
        USART_RFCS(periph) |= USART_RFCS_RFEN;
    } else {
        USART_RFCS(periph) &= ~USART_RFCS_RFEN;
    }

    /* configure baudrate */
    hals_uart_baudrate_set(periph, p_init->baudrate);

    /* get the data bit mask */
    uart->data_bit_mask = _uart_data_bit_mask_get(uart);

    /* disable the SMARTCARD mode, Half-Duplex mode, IRDA mode, LIN mode and clock */
    hals_smartcard_mode_disable(periph);
    hals_uart_halfduplex_disable(periph);
    hals_irda_mode_disable(periph);
    hals_usrt_clock_disable(periph);
    hals_uart_lin_mode_disable(periph);

    if(UART_WORK_MODE_ASYN != p_init->work_mode) {
        switch(p_init->work_mode) {
        case UART_WORK_MODE_MULTIPROCESSCOR:
            /* multiprocessor mode configure */
            hals_uart_mute_mode_wakeup_config(periph, p_init->wakeup_mode);
            if(UART_MULTIPROCESSOR_WAKEUP_IDLE != p_init->wakeup_mode) {
                hals_uart_address_detection_mode_config(periph, p_init->addr_length);
                hals_uart_address_config(periph, p_init->address);
            }
            /* enable mute mode */
            hals_uart_mute_mode_enable(periph);
            break;
        case UART_WORK_MODE_LIN:
            /* LIN mode configure */
            hals_uart_lin_break_detection_length_config(periph, p_init->break_frame_length);
            hals_uart_lin_mode_enable(periph);
            break;

        case UART_WORK_MODE_SINGLE_WIRE:
            /* enable single wire(half-duplex) mode */
            hals_uart_halfduplex_enable(periph);
            break;
        default:
            break;
        }
    }

    if(ENABLE == p_init->rs485_mode) {
        p_init->de_assertion_time &= BITS(0, 4);
        p_init->de_deassertion_time &= BITS(0, 4);

        /* RS485 mode configure */
        hals_uart_driver_assertime_config(periph, p_init->de_assertion_time);
        hals_uart_driver_deassertime_config(periph, p_init->de_deassertion_time);
        hals_uart_depolarity_config(periph, p_init->de_polarity);
        /* enable RS485 driver */
        hals_uart_rs485_driver_enable(periph);
    }

    /* reset the Rx and Tx state */
    uart->tx_state = UART_STATE_FREE;
    uart->rx_state = UART_STATE_FREE;

    /* enable usart */
    hals_uart_enable(uart->periph);

    /* unlock uart */
    HAL_UNLOCK(uart);

    return HAL_ERR_NONE;
}

/*!
    \brief      uart interrupt handler content function,which is merely used in uart_handler
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_uart_irq(hal_uart_dev_struct *uart)
{
    if(RESET == _uart_error_flag_get(uart)) {
        /* check whether UART is in receiver mode or not */
        if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_RBNE)) {
            if(NULL != uart->uart_irq.receive_complete_handle) {
                uart->uart_irq.receive_complete_handle(uart);
            }
            return;
        }
        /* check whether UART is in receiver timeout or not */
        if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_RT)) {
            hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_RT);
            if(NULL != uart->uart_irq.receive_timeout_handle) {
                uart->uart_irq.receive_timeout_handle(uart);
            }
            return;
        }
    } else {
        /* check whether the PERR flag is set or not */
        if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_PERR)) {
            hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_PERR);
            uart->error_state |= HAL_UART_ERROR_PERR;
            uart->last_error = HAL_UART_ERROR_PERR;
        }

        /* check whether the NERR flag is set or not */
        if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_ERR_NERR)) {
            hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_ERR_NERR);
            uart->error_state |= HAL_UART_ERROR_NERR;
            uart->last_error = HAL_UART_ERROR_NERR;
        }

        /* check whether the FERR flag is set or not */
        if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_ERR_FERR)) {
            hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_ERR_FERR);
            uart->error_state |= HAL_UART_ERROR_FERR;
            uart->last_error = HAL_UART_ERROR_FERR;
        }

        /* check whether the ERR ORERR is set or not */
        if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_ERR_ORERR)) {
            hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_ERR_ORERR);
            uart->error_state |= HAL_UART_ERROR_ORERR;
            uart->last_error = HAL_UART_ERROR_ORERR;
        }

        /* check whether RBNE ORERR is set or not */
        if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_RBNE_ORERR)) {
            hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_RBNE_ORERR);
            uart->error_state |= HAL_UART_ERROR_ORERR;
            uart->last_error = HAL_UART_ERROR_ORERR;
        }

        /* check whether error state is none or not */
        if(HAL_UART_ERROR_NONE != uart->error_state) {
            if(uart->uart_irq.error_handle != NULL) {
                uart->uart_irq.error_handle(uart);
                uart->error_state = HAL_UART_ERROR_NONE;
            }
            return;
        }
    }

    /* multi-processor mode interrupt handle */
    if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_IDLE)) {
        hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_IDLE);
        if(NULL != uart->uart_irq.idle_line_detected_handle) {
            uart->uart_irq.idle_line_detected_handle(uart);
        }
    }

    /* address match interrput handle */
    if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_AM)) {
        hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_AM);
        if(NULL != uart->uart_irq.address_match_handle) {
            uart->uart_irq.address_match_handle(uart);
        }
    }

    /* LIN mode interrupt handle */
    if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_LBD)) {
        hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_LBD);
        if(NULL != uart->uart_irq.lin_break_detected_handle) {
            uart->uart_irq.lin_break_detected_handle(uart);
        }
    }

    /* wakeup from deepsleep mode interrupt handle */
    if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_WU)) {
        hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_WU);
        uart->rx_state = UART_STATE_FREE;
        uart->tx_state = UART_STATE_FREE;
        if(NULL != uart->uart_irq.wakeup_handle) {
            uart->uart_irq.wakeup_handle(uart);
        }
    }

    /* hardware flow mode interrupt handle */
    if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_CTS)) {
        hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_CTS);
        if(NULL != uart->uart_irq.cts_change_handle) {
            uart->uart_irq.cts_change_handle(uart);
        }
    }

    /* transmitter buffer empty interrupt handle */
    if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_TBE)) {
        if(NULL != uart->uart_irq.transmit_ready_handle) {
            uart->uart_irq.transmit_ready_handle(uart);
        }
        return;
    }

    /* transmission complete interrupt handle */
    if(RESET != hals_uart_interrupt_flag_get(uart->periph, USART_INT_FLAG_TC)) {
        hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_TC);
        if(NULL != uart->uart_irq.transmit_complete_handle) {
            uart->uart_irq.transmit_complete_handle(uart);
        }
        return;
    }
}

/*!
    \brief      set user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  uart: uart device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to uart interrupt callback functions structure
                  The structure member can be assigned as following parameters:
      \arg        hal_irq_handle_cb function pointer: the function is user-defined,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
      \arg        NULL: The corresponding callback mechanism is out of use, and
                    disable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_uart_irq_handle_set(hal_uart_dev_struct *uart, hal_uart_irq_struct *p_irq)
{
    /* set user-defined address match interrupt callback */
    if(NULL != p_irq->address_match_handle) {
        uart->uart_irq.address_match_handle = p_irq->address_match_handle;
        hals_uart_interrupt_enable(uart->periph, USART_INT_AM);
    } else {
        uart->uart_irq.address_match_handle = NULL;
        hals_uart_interrupt_disable(uart->periph, USART_INT_AM);
    }

    /* set user-defined CTS change interrupt callback */
    if(NULL != p_irq->cts_change_handle) {
        uart->uart_irq.cts_change_handle = p_irq->cts_change_handle;
        hals_uart_interrupt_enable(uart->periph, USART_INT_CTS);
    } else {
        uart->uart_irq.cts_change_handle = NULL;
        hals_uart_interrupt_disable(uart->periph, USART_INT_CTS);
    }

    /* set user-defined error interrupt callback */
    if(NULL != p_irq->error_handle) {
        uart->uart_irq.error_handle = p_irq->error_handle;
        hals_uart_interrupt_enable(uart->periph, USART_INT_ERR);
        hals_uart_interrupt_enable(uart->periph, USART_INT_PERR);
    } else {
        uart->uart_irq.error_handle = NULL;
        hals_uart_interrupt_disable(uart->periph, USART_INT_ERR);
        hals_uart_interrupt_disable(uart->periph, USART_INT_PERR);
    }

    /* set user-defined idle line detected interrupt callback */
    if(NULL != p_irq->idle_line_detected_handle) {
        uart->uart_irq.idle_line_detected_handle = p_irq->idle_line_detected_handle;
        hals_uart_interrupt_enable(uart->periph, USART_INT_IDLE);
    } else {
        uart->uart_irq.idle_line_detected_handle = NULL;
        hals_uart_interrupt_disable(uart->periph, USART_INT_IDLE);
    }

    /* set user-defined LIN break detected interrupt callback */
    if(NULL != p_irq->lin_break_detected_handle) {
        uart->uart_irq.lin_break_detected_handle = p_irq->lin_break_detected_handle;
        hals_uart_interrupt_enable(uart->periph, USART_INT_LBD);
    } else {
        uart->uart_irq.lin_break_detected_handle = NULL;
        hals_uart_interrupt_disable(uart->periph, USART_INT_LBD);
    }

    /* set user-defined receive complete interrupt callback */
    if(NULL != p_irq->receive_complete_handle) {
        uart->uart_irq.receive_complete_handle = p_irq->receive_complete_handle;
        hals_uart_interrupt_enable(uart->periph, USART_INT_RBNE);
    } else {
        uart->uart_irq.receive_complete_handle = NULL;
        hals_uart_interrupt_disable(uart->periph, USART_INT_RBNE);
    }

    /* set user-defined receive timeout interrupt callback */
    if(NULL != p_irq->receive_timeout_handle) {
        uart->uart_irq.receive_timeout_handle = p_irq->receive_timeout_handle;
        hals_uart_interrupt_enable(uart->periph, USART_INT_RT);
    } else {
        uart->uart_irq.receive_timeout_handle = NULL;
        hals_uart_interrupt_disable(uart->periph, USART_INT_RT);
    }

    /* set user-defined transmit complete interrupt callback */
    if(NULL != p_irq->transmit_complete_handle) {
        uart->uart_irq.transmit_complete_handle = p_irq->transmit_complete_handle;
        hals_uart_interrupt_enable(uart->periph, USART_INT_TC);
    } else {
        uart->uart_irq.transmit_complete_handle = NULL;
        hals_uart_interrupt_disable(uart->periph, USART_INT_TC);
    }

    /* set user-defined transmit ready interrupt callback */
    if(NULL != p_irq->transmit_ready_handle) {
        uart->uart_irq.transmit_ready_handle = p_irq->transmit_ready_handle;
        hals_uart_interrupt_enable(uart->periph, USART_INT_TBE);
    } else {
        uart->uart_irq.transmit_ready_handle = NULL;
        hals_uart_interrupt_disable(uart->periph, USART_INT_TBE);
    }

    /* set user-defined wakeup interrupt callback */
    if(NULL != p_irq->wakeup_handle) {
        uart->uart_irq.wakeup_handle = p_irq->wakeup_handle;
        hals_uart_interrupt_enable(uart->periph, USART_INT_WU);
    } else {
        uart->uart_irq.wakeup_handle = NULL;
        hals_uart_interrupt_disable(uart->periph, USART_INT_WU);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_uart_irq_handle_all_reset(hal_uart_dev_struct *uart)
{
    /* configure interrupt callback function to NULL */
    uart->uart_irq.address_match_handle         = NULL;
    uart->uart_irq.cts_change_handle            = NULL;
    uart->uart_irq.error_handle                 = NULL;
    uart->uart_irq.idle_line_detected_handle    = NULL;
    uart->uart_irq.lin_break_detected_handle    = NULL;
    uart->uart_irq.receive_complete_handle      = NULL;
    uart->uart_irq.receive_timeout_handle      = NULL;
    uart->uart_irq.transmit_complete_handle     = NULL;
    uart->uart_irq.transmit_ready_handle        = NULL;
    uart->uart_irq.wakeup_handle                = NULL;
}

/*!
    \brief      transmit amounts of data, poll transmit process and completed status
                the function is blocking
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY, HAL_ERR_TIMEOUT details refer to gd32f3x0_hal.h
*/
int32_t hal_uart_transmit_poll(hal_uart_dev_struct *uart, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms)
{
    uint8_t data_length;
    uint32_t tick_start;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [uart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the tx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->tx_state) {
        HAL_DEBUGE("uart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock uart */
    HAL_LOCK(uart);

    /* initialize transmit parameters */
    uart->error_state       = HAL_UART_ERROR_NONE;
    uart->txbuffer.buffer   = (uint8_t *)p_buffer;
    uart->txbuffer.length   = length;
    uart->txbuffer.pos      = 0U;
    uart->tx_state          = UART_STATE_BUSY;

    /* calculate the data length */
    data_length = 1U;
    if(RESET != (USART_CTL0(uart->periph) & USART_CTL0_WL)) {
        if(RESET == (USART_CTL0(uart->periph) & USART_CTL0_PCEN)) {
            data_length = 2U;
        }
    }

    /* configure timeout */
    tick_start = hal_sys_basetick_count_get();

    while(uart->txbuffer.pos < uart->txbuffer.length) {
        /* wait for transmit buffer empty */
        while(RESET == hals_uart_flag_get(uart->periph, USART_FLAG_TBE)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    HAL_DEBUGW("uart transmit timeout");
                    /* reset the state */
                    uart->rx_state = UART_STATE_FREE;
                    /* unlock uart */
                    HAL_UNLOCK(uart);
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* send data: in case of 9bit&no parity, uint16_t data will be transmitted */
        if(2U == data_length) {
            hals_uart_data_transmit(uart->periph, (*(uint16_t *)uart->txbuffer.buffer & (uint16_t)0x1FFU));
            uart->txbuffer.buffer += 2;
        } else {
            hals_uart_data_transmit(uart->periph, (*uart->txbuffer.buffer & (uint8_t)0xFFU));
            uart->txbuffer.buffer++;
        }
        /* change the transmit pointer */
        uart->txbuffer.pos++;
    }

    /* wait for transmit complete */
    while(RESET == hals_uart_flag_get(uart->periph, USART_FLAG_TC)) {
        if(HAL_TIMEOUT_FOREVER != timeout_ms) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                HAL_DEBUGW("uart transmit timeout");
                /* reset the state */
                uart->rx_state = UART_STATE_FREE;
                /* unlock uart */
                HAL_UNLOCK(uart);
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    /* change the Tx state to free */
    uart->tx_state = UART_STATE_FREE;

    /* unlock uart */
    HAL_UNLOCK(uart);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data, poll receive process and completed status
                the function is blocking
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY, HAL_ERR_TIMEOUT details refer to gd32f3x0_hal.h
*/
int32_t hal_uart_receive_poll(hal_uart_dev_struct *uart, uint8_t *p_buffer, uint32_t length, uint32_t timeout_ms)
{
    uint8_t data_length;
    uint32_t tick_start;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [uart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the rx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->rx_state) {
        HAL_DEBUGE("uart rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock uart */
    HAL_LOCK(uart);

    uart->error_state       = HAL_UART_ERROR_NONE;
    uart->rx_state          = UART_STATE_BUSY;

    /* initialize receive parameters */
    uart->rxbuffer.buffer   = (uint8_t *)p_buffer;
    uart->rxbuffer.length   = length;
    uart->rxbuffer.pos      = 0U;
    uart->data_bit_mask     = _uart_data_bit_mask_get(uart);

    /* calculate the data length */
    data_length = 1U;
    if(RESET != (USART_CTL0(uart->periph) & USART_CTL0_WL)) {
        if(RESET == (USART_CTL0(uart->periph) & USART_CTL0_PCEN)) {
            data_length = 2U;
        }
    }

    /* configure timeout */
    tick_start = hal_sys_basetick_count_get();

    while(uart->rxbuffer.pos < uart->rxbuffer.length) {
        /* wait for read data buffer not empty */
        while(RESET == hals_uart_flag_get(uart->periph, USART_FLAG_RBNE)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    HAL_DEBUGW("uart receive timeout");
                    /* reset the state */
                    uart->rx_state = UART_STATE_FREE;
                    /* unlock uart */
                    HAL_UNLOCK(uart);
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* read data: in case of 9bit&no parity, uint16_t data will be receive */
        if(2U == data_length) {
            *(uint16_t *)uart->rxbuffer.buffer = (hals_uart_data_receive(uart->periph) & uart->data_bit_mask);
            uart->rxbuffer.buffer += 2;
        } else {
            *uart->rxbuffer.buffer = (uint8_t)(hals_uart_data_receive(uart->periph) & uart->data_bit_mask);
            uart->rxbuffer.buffer++;
        }
        uart->rxbuffer.pos++;
    }

    /* change the Rx state to free */
    uart->rx_state = UART_STATE_FREE;

    /* unlock uart */
    HAL_UNLOCK(uart);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by interrupt method
                the function is non-blocking
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_uart_transmit_interrupt(hal_uart_dev_struct *uart, uint8_t *p_buffer, uint32_t length,
                                    hal_uart_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [uart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the tx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->tx_state) {
        HAL_DEBUGE("uart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock uart */
    HAL_LOCK(uart);

    uart->tx_state          = UART_STATE_BUSY;
    uart->error_state       = HAL_UART_ERROR_NONE;

    /* initialize transmit parameters */
    uart->txbuffer.buffer   = (uint8_t *)p_buffer;
    uart->txbuffer.length   = length;
    uart->txbuffer.pos      = 0U;
    uart->tx_callback       = (void *)p_user_func;

    /* configure the transmit ready and complete callback as the function implemented */
    uart->uart_irq.transmit_ready_handle    = _uart_transmit_interrupt;
    uart->uart_irq.transmit_complete_handle = _uart_transmit_complete_interrupt;

    /* clear UART TC interrupt flag */
    hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_TC);

    /* enable the TBE interrupt */
    hals_uart_interrupt_enable(uart->periph, USART_INT_TBE);

    /* unlock uart */
    HAL_UNLOCK(uart);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by interrupt method
                the function is non-blocking
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_uart_receive_interrupt(hal_uart_dev_struct *uart, uint8_t *p_buffer, uint32_t length,
                                   hal_uart_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [uart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the rx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->rx_state) {
        HAL_DEBUGE("uart rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock uart */
    HAL_LOCK(uart);

    uart->rx_state = UART_STATE_BUSY;
    uart->error_state = HAL_UART_ERROR_NONE;

    /* initialize receive parameters */
    uart->rxbuffer.buffer   = (uint8_t *)p_buffer;
    uart->rxbuffer.length   = length;
    uart->rxbuffer.pos      = 0U;
    uart->data_bit_mask     = _uart_data_bit_mask_get(uart);
    uart->rx_callback       = (void *)p_user_func;

    /* configure rx interrupt hander */
    uart->uart_irq.receive_complete_handle = _uart_receive_interrupt;

    /* enable PERR, ERR, RBNE interrupt */
    hals_uart_interrupt_enable(uart->periph, USART_INT_PERR);
    hals_uart_interrupt_enable(uart->periph, USART_INT_ERR);
    hals_uart_interrupt_enable(uart->periph, USART_INT_RBNE);

    /* unlock uart */
    HAL_UNLOCK(uart);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by dma method
                the function is non-blocking
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_uart_transmit_dma(hal_uart_dev_struct *uart, uint8_t *p_buffer, uint16_t length, hal_uart_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [uart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check the parameter */
    if(NULL == uart->p_dma_tx) {
        HAL_DEBUGE("parameter [uart->p_dma_tx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the tx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->tx_state) {
        HAL_DEBUGE("uart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock uart */
    HAL_LOCK(uart);

    uart->tx_state = UART_STATE_BUSY;
    uart->error_state = HAL_UART_ERROR_NONE;

    /* initialize transmit parameters */
    uart->txbuffer.buffer   = (uint8_t *)p_buffer;
    uart->txbuffer.length   = length;
    uart->txbuffer.pos      = 0U;

    if(NULL != p_func) {
        uart->tx_callback = (void *)p_func->complete_func;
        uart->uart_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    } else {
        uart->tx_callback = NULL;
        uart->uart_irq.error_handle = NULL;
    }
    uart->uart_irq.transmit_complete_handle = _uart_transmit_complete_interrupt;

    /* configure DMA interrupt callback function */
    dma_irq.full_finish_handle  = _uart_transmit_dma;
    dma_irq.error_handle        = _uart_dma_error;
    if(NULL != uart->p_dma_tx->dma_irq.half_finish_handle) {
        dma_irq.half_finish_handle = uart->p_dma_tx->dma_irq.half_finish_handle;
    } else {
        dma_irq.half_finish_handle = NULL;
    }

    /* start DMA interrupt mode transfer */
    if(HAL_ERR_NONE != hal_dma_start_interrupt(uart->p_dma_tx, (uint32_t)uart->txbuffer.buffer,
            (uint32_t)&USART_TDATA(uart->periph), uart->txbuffer.length, &dma_irq)) {
        uart->tx_state = UART_STATE_FREE;
        uart->error_state = HAL_UART_ERROR_DMATX;
        /* unlock uart */
        HAL_UNLOCK(uart);

        return HAL_ERR_NONE;
    }

    /* clear UART TC interrupt flag */
    hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_TC);

    /* DMA enable for transmission */
    hals_uart_dma_transmit_config(uart->periph, USART_TRANSMIT_DMA_ENABLE);

    /* unlock uart */
    HAL_UNLOCK(uart);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by dma method
                the function is non-blocking
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_uart_receive_dma(hal_uart_dev_struct *uart, uint8_t *p_buffer, uint16_t length, hal_uart_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_buffer) || (0U == length)) {
        HAL_DEBUGE("parameter [uart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check the parameter */
    if(NULL == uart->p_dma_rx) {
        HAL_DEBUGE("parameter [uart->p_dma_rx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the rx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->rx_state) {
        HAL_DEBUGE("uart rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* lock uart */
    HAL_LOCK(uart);

    uart->rx_state      = UART_STATE_BUSY;
    uart->error_state   = HAL_UART_ERROR_NONE;

    /* initialize receive parameters */
    uart->rxbuffer.buffer   = (uint8_t *)p_buffer;
    uart->rxbuffer.length   = length;
    uart->rxbuffer.pos      = 0U;
    uart->data_bit_mask     = _uart_data_bit_mask_get(uart);

    if(NULL != p_func) {
        uart->rx_callback = (void *)p_func->complete_func;
        uart->uart_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    } else {
        uart->rx_callback = NULL;
        uart->uart_irq.error_handle = NULL;
    }

    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle  = _uart_receive_dma;
    dma_irq.error_handle        = _uart_dma_error;
    if(NULL != uart->p_dma_rx->dma_irq.half_finish_handle) {
        dma_irq.half_finish_handle = uart->p_dma_rx->dma_irq.half_finish_handle;
    } else {
        dma_irq.half_finish_handle = NULL;
    }

    /* start DMA interrupt mode transfer */
    if(HAL_ERR_NONE != hal_dma_start_interrupt(uart->p_dma_rx, (uint32_t)&USART_RDATA(uart->periph),
            (uint32_t)uart->rxbuffer.buffer, length, &dma_irq)) {
        uart->tx_state = UART_STATE_FREE;
        uart->error_state = HAL_UART_ERROR_DMARX;
        /* unlock uart */
        HAL_UNLOCK(uart);

        return HAL_ERR_NONE;
    }

    /* enable the usart parity error and error interrupt: (frame error, noise error, overrun error) */
    hals_uart_interrupt_enable(uart->periph, USART_INT_PERR);
    hals_uart_interrupt_enable(uart->periph, USART_INT_ERR);

    /* DMA enable for reception */
    hals_uart_dma_receive_config(uart->periph, USART_RECEIVE_DMA_ENABLE);

    /* lock uart */
    HAL_UNLOCK(uart);

    return HAL_ERR_NONE;
}

/*!
    \brief      pause uart DMA transfer during transmission process
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32f3x0_hal.h
*/
int32_t hal_uart_dma_pause(hal_uart_dev_struct *uart)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart) {
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock uart */
    HAL_LOCK(uart);

    /* check the tx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->tx_state) {
        /* disable DMA transimt */
        hals_uart_dma_transmit_config(uart->periph, USART_TRANSMIT_DMA_DISABLE);
    }

    /* check the rx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->rx_state) {
        /* disable the PERR and ERR interrupt */
        hals_uart_interrupt_disable(uart->periph, USART_INT_PERR);
        hals_uart_interrupt_disable(uart->periph, USART_INT_ERR);

        /* disable DMA receive */
        hals_uart_dma_receive_config(uart->periph, USART_RECEIVE_DMA_DISABLE);
    }

    /* unlock uart */
    HAL_UNLOCK(uart);

    return HAL_ERR_NONE;
}

/*!
    \brief      resume uart DMA transfer during transmission process
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32f3x0_hal.h
*/
int32_t hal_uart_dma_resume(hal_uart_dev_struct *uart)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart) {
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock uart */
    HAL_LOCK(uart);

    /* check the tx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->tx_state) {
        /* enable DMA transimt */
        hals_uart_dma_transmit_config(uart->periph, USART_TRANSMIT_DMA_ENABLE);
    }

    /* check the rx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->rx_state) {
        /* enable the PERR and ERR interrupt */
        hals_uart_interrupt_enable(uart->periph, USART_INT_PERR);
        hals_uart_interrupt_enable(uart->periph, USART_INT_ERR);

        /* enable DMA receive */
        hals_uart_dma_receive_config(uart->periph, USART_RECEIVE_DMA_ENABLE);
    }

    /* unlock uart */
    HAL_UNLOCK(uart);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop uart transmit transfer
                the function is blocking
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32f3x0_hal.h
*/
int32_t hal_uart_transmit_stop(hal_uart_dev_struct *uart)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart) {
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock uart */
    HAL_LOCK(uart);

    /* disable the TBE and TC interrupt */
    hals_uart_interrupt_disable(uart->periph, USART_INT_TBE);
    hals_uart_interrupt_disable(uart->periph, USART_INT_TC);

    /* disable DMA transimt and stop DMA */
    hals_uart_dma_transmit_config(uart->periph, USART_TRANSMIT_DMA_DISABLE);
    hal_dma_stop(uart->p_dma_tx);

    /* reset the position and state */
    uart->txbuffer.pos = 0;
    uart->tx_state = UART_STATE_FREE;

    /* unlock uart */
    HAL_UNLOCK(uart);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop uart receive transfer
                the function is blocking
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32f3x0_hal.h
*/
int32_t hal_uart_receive_stop(hal_uart_dev_struct *uart)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart) {
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock uart */
    HAL_LOCK(uart);

    /* disable the RBNE, PERR and ERR interrupt */
    hals_uart_interrupt_disable(uart->periph, USART_INT_RBNE);
    hals_uart_interrupt_disable(uart->periph, USART_INT_PERR);
    hals_uart_interrupt_disable(uart->periph, USART_INT_ERR);

    /* disable DMA receive and stop DMA */
    hals_uart_dma_receive_config(uart->periph, USART_RECEIVE_DMA_DISABLE);
    hal_dma_stop(uart->p_dma_rx);

    /* reset the position and state */
    uart->rxbuffer.pos = 0;
    uart->rx_state = UART_STATE_FREE;

    /* clear interrupt error flags */
    hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_PERR);
    hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_ERR_FERR);
    hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_ERR_NERR);
    hals_uart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_ERR_ORERR);

    /* unlock uart */
    HAL_UNLOCK(uart);

    return HAL_ERR_NONE;
}

/*!
    \brief      reset UART
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_deinit(uint32_t uart_periph)
{
    switch(uart_periph) {
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
    \brief      configure UART baud rate value
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  baudval: baud rate value
    \param[out] none
    \retval     none
*/
void hals_uart_baudrate_set(uint32_t uart_periph, uint32_t baudval)
{
    uint32_t uclk = 0U, intdiv = 0U, fradiv = 0U, udiv = 0U;
    switch(uart_periph) {
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
    if(USART_CTL0(uart_periph) & USART_CTL0_OVSMOD) {
        /* oversampling by 8, configure the value of USART_BAUD */
        udiv = ((2U * uclk) + (baudval / 2U)) / baudval;
        intdiv = udiv & 0x0000fff0U;
        fradiv = (udiv >> 1U) & 0x00000007U;
        USART_BAUD(uart_periph) = ((USART_BAUD_FRADIV | USART_BAUD_INTDIV) & (intdiv | fradiv));
    } else {
        /* oversampling by 16, configure the value of USART_BAUD */
        udiv = (uclk + (baudval / 2U)) / baudval;
        intdiv = udiv & 0x0000fff0U;
        fradiv = udiv & 0x0000000fU;
        USART_BAUD(uart_periph) = ((USART_BAUD_FRADIV | USART_BAUD_INTDIV) & (intdiv | fradiv));
    }
}

/*!
    \brief      configure UART parity
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  paritycfg: UART parity configure
                only one parameter can be selected which is shown as below:
      \arg        USART_PM_NONE: no parity
      \arg        USART_PM_ODD: odd parity
      \arg        USART_PM_EVEN: even parity
    \param[out] none
    \retval     none
*/
void hals_uart_parity_config(uint32_t uart_periph, uint32_t paritycfg)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    /* clear USART_CTL0 PM,PCEN bits */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_PM | USART_CTL0_PCEN);
    /* configure UART parity mode */
    USART_CTL0(uart_periph) |= paritycfg;
}

/*!
    \brief      configure UART word length
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  wlen: UART word length configure
                only one parameter can be selected which is shown as below:
      \arg        USART_WL_8BIT: 8 bits
      \arg        USART_WL_9BIT: 9 bits
    \param[out] none
    \retval     none
*/
void hals_uart_word_length_set(uint32_t uart_periph, uint32_t wlen)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    /* clear USART_CTL0 WL bit */
    USART_CTL0(uart_periph) &= ~USART_CTL0_WL;
    /* configure UART word length */
    USART_CTL0(uart_periph) |= wlen;
}

/*!
    \brief      configure UART stop bit length
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  stblen: UART stop bit configure
                only one parameter can be selected which is shown as below:
      \arg        USART_STB_1BIT: 1 bit
      \arg        USART_STB_0_5BIT: 0.5bit
      \arg        USART_STB_2BIT: 2 bits
      \arg        USART_STB_1_5BIT: 1.5bit
    \param[out] none
    \retval     none
*/
void hals_uart_stop_bit_set(uint32_t uart_periph, uint32_t stblen)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    /* clear USART_CTL1 STB bits */
    USART_CTL1(uart_periph) &= ~USART_CTL1_STB;
    USART_CTL1(uart_periph) |= stblen;
}

/*!
    \brief      enable UART
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_enable(uint32_t uart_periph)
{
    USART_CTL0(uart_periph) |= USART_CTL0_UEN;
}

/*!
    \brief      disable UART
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_disable(uint32_t uart_periph)
{
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
}

/*!
    \brief      configure UART transmitter
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  txconfig: enable or disable UART transmitter
                only one parameter can be selected which is shown as below:
      \arg        USART_TRANSMIT_ENABLE: enable UART transmission
      \arg        USART_TRANSMIT_DISABLE: enable UART transmission
    \param[out] none
    \retval     none
*/
void hals_uart_transmit_config(uint32_t uart_periph, uint32_t txconfig)
{
    USART_CTL0(uart_periph) &= ~USART_CTL0_TEN;
    /* configure transfer mode */
    USART_CTL0(uart_periph) |= txconfig;
}

/*!
    \brief      configure UART receiver
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  rxconfig: enable or disable UART receiver
                only one parameter can be selected which is shown as below:
      \arg        USART_RECEIVE_ENABLE: enable UART reception
      \arg        USART_RECEIVE_DISABLE: disable UART reception
    \param[out] none
    \retval     none
*/
void hals_uart_receive_config(uint32_t uart_periph, uint32_t rxconfig)
{
    USART_CTL0(uart_periph) &= ~USART_CTL0_REN;
    /* configure receiver mode */
    USART_CTL0(uart_periph) |= rxconfig;
}

/*!
    \brief      data is transmitted/received with the LSB/MSB first
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  msbf: LSB/MSB
                only one parameter can be selected which is shown as below:
      \arg        USART_MSBF_LSB: LSB first
      \arg        USART_MSBF_MSB: MSB first
    \param[out] none
    \retval     none
*/
void hals_uart_data_first_config(uint32_t uart_periph, uint32_t msbf)
{
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    /* configure LSB or MSB first */
    USART_CTL1(uart_periph) &= ~(USART_CTL1_MSBF);
    USART_CTL1(uart_periph) |= msbf;
}

/*!
    \brief      configure UART inversion
    \param[in]  uart_periph: USARTx(x=0,1)
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
void hals_uart_invert_config(uint32_t uart_periph, usart_invert_enum invertpara)
{
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    /* inverted or not the specified signal */
    switch(invertpara) {
    case USART_DINV_ENABLE:
        USART_CTL1(uart_periph) |= USART_CTL1_DINV;
        break;
    case USART_DINV_DISABLE:
        USART_CTL1(uart_periph) &= ~(USART_CTL1_DINV);
        break;
    case USART_TXPIN_ENABLE:
        USART_CTL1(uart_periph) |= USART_CTL1_TINV;
        break;
    case USART_TXPIN_DISABLE:
        USART_CTL1(uart_periph) &= ~(USART_CTL1_TINV);
        break;
    case USART_RXPIN_ENABLE:
        USART_CTL1(uart_periph) |= USART_CTL1_RINV;
        break;
    case USART_RXPIN_DISABLE:
        USART_CTL1(uart_periph) &= ~(USART_CTL1_RINV);
        break;
    case USART_SWAP_ENABLE:
        USART_CTL1(uart_periph) |= USART_CTL1_STRP;
        break;
    case USART_SWAP_DISABLE:
        USART_CTL1(uart_periph) &= ~(USART_CTL1_STRP);
        break;
    default:
        break;
    }
}

/*!
    \brief      enable the UART overrun function
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_overrun_enable(uint32_t uart_periph)
{
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    /* enable overrun function */
    USART_CTL2(uart_periph) &= ~(USART_CTL2_OVRD);
}

/*!
    \brief      disable the UART overrun function
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_overrun_disable(uint32_t uart_periph)
{
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    /* disable overrun function */
    USART_CTL2(uart_periph) |= USART_CTL2_OVRD;
}

/*!
    \brief      configure the UART oversample mode
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  oversamp: oversample value
                only one parameter can be selected which is shown as below:
      \arg        USART_OVSMOD_8: oversampling by 8
      \arg        USART_OVSMOD_16: oversampling by 16
    \param[out] none
    \retval     none
*/
void hals_uart_oversample_config(uint32_t uart_periph, uint32_t oversamp)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    /* clear OVSMOD bit */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_OVSMOD);
    USART_CTL0(uart_periph) |= oversamp;
}

/*!
    \brief      configure the sample bit method
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  osb: sample bit
                only one parameter can be selected which is shown as below:
      \arg        USART_OSB_1BIT: 1 bit
      \arg        USART_OSB_3BIT: 3 bits
    \param[out] none
    \retval     none
*/
void hals_uart_sample_bit_config(uint32_t uart_periph, uint32_t osb)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(uart_periph) &= ~(USART_CTL2_OSB);
    USART_CTL2(uart_periph) |= osb;
}

/*!
    \brief      enable receiver timeout
    \param[in]  uart_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_uart_receiver_timeout_enable(uint32_t uart_periph)
{
    USART_CTL1(uart_periph) |= USART_CTL1_RTEN;
}

/*!
    \brief      disable receiver timeout
    \param[in]  uart_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_uart_receiver_timeout_disable(uint32_t uart_periph)
{
    USART_CTL1(uart_periph) &= ~(USART_CTL1_RTEN);
}

/*!
    \brief      configure receiver timeout threshold
    \param[in]  uart_periph: USARTx(x=0)
    \param[in]  rtimeout: 0x00000000-0x00FFFFFF, receiver timeout value in terms of number of baud clocks
    \param[out] none
    \retval     none
*/
void hals_uart_receiver_timeout_threshold_config(uint32_t uart_periph, uint32_t rtimeout)
{
    USART_RT(uart_periph) &= ~(USART_RT_RT);
    USART_RT(uart_periph) |= rtimeout;
}

/*!
    \brief      UART transmit data function
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  data: data of transmission
    \param[out] none
    \retval     none
*/
void hals_uart_data_transmit(uint32_t uart_periph, uint16_t data)
{
    USART_TDATA(uart_periph) = (USART_TDATA_TDATA & (uint32_t)data);
}

/*!
    \brief      UART receive data function
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     data of received
*/
uint16_t hals_uart_data_receive(uint32_t uart_periph)
{
    return (uint16_t)(GET_BITS(USART_RDATA(uart_periph), 0U, 8U));
}

/*!
    \brief      enable UART command
    \param[in]  uart_periph: USARTx(x=0,1,2)
    \param[in]  cmdtype: command type
                only one parameter can be selected which is shown as below:
      \arg        USART_CMD_SBKCMD: send break command
      \arg        USART_CMD_MMCMD: mute mode command
      \arg        USART_CMD_RXFCMD: receive data flush command
      \arg        USART_CMD_TXFCMD: transmit data flush request
    \param[out] none
    \retval     none
*/
void hals_uart_command_enable(uint32_t uart_periph, uint32_t cmdtype)
{
    USART_CMD(uart_periph) |= (cmdtype);
}

/*!
    \brief      configure the address of the UART in wake up by address match mode
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  addr: 0x00-0xFF, address of UART terminal
    \param[out] none
    \retval     none
*/
void hals_uart_address_config(uint32_t uart_periph, uint8_t addr)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL1(uart_periph) &= ~(USART_CTL1_ADDR);
    USART_CTL1(uart_periph) |= (USART_CTL1_ADDR & (((uint32_t)addr) << CTL1_ADDR_OFFSET));
}

/*!
    \brief      configure address detection mode
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  addmod: address detection mode
                only one parameter can be selected which is shown as below:
      \arg        USART_ADDM_4BIT: 4 bits
      \arg        USART_ADDM_FULLBIT: full bits
    \param[out] none
    \retval     none
*/
void hals_uart_address_detection_mode_config(uint32_t uart_periph, uint32_t addmod)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL1(uart_periph) &= ~(USART_CTL1_ADDM);
    USART_CTL1(uart_periph) |= USART_CTL1_ADDM & (addmod);
}

/*!
    \brief      enable mute mode
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_mute_mode_enable(uint32_t uart_periph)
{
    USART_CTL0(uart_periph) |= USART_CTL0_MEN;
}

/*!
    \brief      disable mute mode
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_mute_mode_disable(uint32_t uart_periph)
{
    USART_CTL0(uart_periph) &= ~(USART_CTL0_MEN);
}

/*!
    \brief      configure wakeup method in mute mode
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  wmethod: two methods be used to enter or exit the mute mode
                only one parameter can be selected which is shown as below:
      \arg        USART_WM_IDLE: idle line
      \arg        USART_WM_ADDR: address match
    \param[out] none
    \retval     none
*/
void hals_uart_mute_mode_wakeup_config(uint32_t uart_periph, uint32_t wmethod)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL0(uart_periph) &= ~(USART_CTL0_WM);
    USART_CTL0(uart_periph) |= wmethod;
}

/*!
    \brief      enable LIN mode
    \param[in]  uart_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_uart_lin_mode_enable(uint32_t uart_periph)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL1(uart_periph) |= USART_CTL1_LMEN;
}

/*!
    \brief      disable LIN mode
    \param[in]  uart_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_uart_lin_mode_disable(uint32_t uart_periph)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL1(uart_periph) &= ~(USART_CTL1_LMEN);
}

/*!
    \brief      configure LIN break frame length
    \param[in]  uart_periph: USARTx(x=0)
    \param[in]  lblen: LIN break detection length
                only one parameter can be selected which is shown as below:
      \arg        USART_LBLEN_10B: 10 bits break detection
      \arg        USART_LBLEN_11B: 11 bits break detection
    \param[out] none
    \retval     none
*/
void hals_uart_lin_break_detection_length_config(uint32_t uart_periph, uint32_t lblen)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    USART_CTL1(uart_periph) &= ~(USART_CTL1_LBLEN);
    USART_CTL1(uart_periph) |= USART_CTL1_LBLEN & (lblen);
}

/*!
    \brief      enable half-duplex mode
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_halfduplex_enable(uint32_t uart_periph)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(uart_periph) |= USART_CTL2_HDEN;
}

/*!
    \brief      disable half-duplex mode
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_halfduplex_disable(uint32_t uart_periph)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(uart_periph) &= ~(USART_CTL2_HDEN);
}

/*!
    \brief      configure hardware flow control RTS
    \param[in]  usart_periph: USARTx(x=0,1)
    \param[in]  rtsconfig: enable or disable RTS
                only one parameter can be selected which is shown as below:
      \arg        USART_RTS_ENABLE:  enable RTS
      \arg        USART_RTS_DISABLE: disable RTS
    \param[out] none
    \retval     none
*/
void hals_uart_hardware_flow_rts_config(uint32_t usart_periph, uint32_t rtsconfig)
{
    /* disable UART */
    USART_CTL0(usart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(usart_periph) &= ~(USART_CTL2_RTSEN);
    USART_CTL2(usart_periph) |= rtsconfig;
}

/*!
    \brief      configure hardware flow control CTS
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  ctsconfig:  enable or disable CTS
                only one parameter can be selected which is shown as below:
      \arg        USART_CTS_ENABLE:  enable CTS
      \arg        USART_CTS_DISABLE: disable CTS
    \param[out] none
    \retval     none
*/
void hals_uart_hardware_flow_cts_config(uint32_t uart_periph, uint32_t ctsconfig)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(uart_periph) &= ~USART_CTL2_CTSEN;
    USART_CTL2(uart_periph) |= ctsconfig;
}

/*!
    \brief      enable RS485 driver
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_rs485_driver_enable(uint32_t uart_periph)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(uart_periph) |= USART_CTL2_DEM;
}

/*!
    \brief      disable RS485 driver
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_rs485_driver_disable(uint32_t uart_periph)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL2(uart_periph) &= ~(USART_CTL2_DEM);
}

/*!
    \brief      configure driver enable assertion time
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  deatime: 0x00000000-0x0000001F
    \param[out] none
    \retval     none
*/
void hals_uart_driver_assertime_config(uint32_t uart_periph, uint32_t deatime)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL0(uart_periph) &= ~(USART_CTL0_DEA);
    USART_CTL0(uart_periph) |= (USART_CTL0_DEA & ((deatime) << CTL0_DEA_OFFSET));
}

/*!
    \brief      configure driver enable de-assertion time
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  dedtime: 0x00000000-0x0000001F
    \param[out] none
    \retval     none
*/
void hals_uart_driver_deassertime_config(uint32_t uart_periph, uint32_t dedtime)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);

    USART_CTL0(uart_periph) &= ~(USART_CTL0_DED);
    USART_CTL0(uart_periph) |= (USART_CTL0_DED & ((dedtime) << CTL0_DED_OFFSET));
}

/*!
    \brief      configure driver enable polarity mode
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  dep: DE signal
                only one parameter can be selected which is shown as below:
      \arg        USART_DEP_HIGH: DE signal is active high
      \arg        USART_DEP_LOW: DE signal is active low
    \param[out] none
    \retval     none
*/
void hals_uart_depolarity_config(uint32_t uart_periph, uint32_t dep)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    /* reset DEP bit */
    USART_CTL2(uart_periph) &= ~(USART_CTL2_DEP);
    USART_CTL2(uart_periph) |= (USART_CTL2_DEP & dep);
}

/*!
    \brief      configure UART DMA reception
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  dmacmd: UART DMA mode
                only one parameter can be selected which is shown as below:
      \arg        USART_RECEIVE_DMA_ENABLE: enable UART DMA for reception
      \arg        USART_RECEIVE_DMA_DISABLE: disable UART DMA for reception
    \param[out] none
    \retval     none
*/
void hals_uart_dma_receive_config(uint32_t uart_periph, uint8_t dmacmd)
{
    USART_CTL2(uart_periph) &= ~USART_CTL2_DENR;
    USART_CTL2(uart_periph) |= (USART_CTL2_DENR & dmacmd);
}

/*!
    \brief      configure UART DMA transmission
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  dmacmd: UART DMA mode
                only one parameter can be selected which is shown as below:
      \arg        USART_TRANSMIT_DMA_ENABLE: enable UART DMA for transmission
      \arg        USART_TRANSMIT_DMA_DISABLE: disable UART DMA for transmission
    \param[out] none
    \retval     none
*/
void hals_uart_dma_transmit_config(uint32_t uart_periph, uint8_t dmacmd)
{
    USART_CTL2(uart_periph) &= ~USART_CTL2_DENT;
    USART_CTL2(uart_periph) |= (USART_CTL2_DENT & dmacmd);
}

/*!
    \brief      enable DMA on reception error
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_reception_error_dma_enable(uint32_t uart_periph)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    USART_CTL2(uart_periph) &= ~(USART_CTL2_DDRE);
}

/*!
    \brief      disable DMA on reception error
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_reception_error_dma_disable(uint32_t uart_periph)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    USART_CTL2(uart_periph) |= USART_CTL2_DDRE;
}

/*!
    \brief      enable UART to wakeup the MCU from deep-sleep mode
    \param[in]  uart_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_uart_wakeup_enable(uint32_t uart_periph)
{
    USART_CTL0(uart_periph) |= USART_CTL0_UESM;
}

/*!
    \brief      disable UART to wakeup the MCU from deep-sleep mode
    \param[in]  uart_periph: USARTx(x=0)
    \param[out] none
    \retval     none
*/
void hals_uart_wakeup_disable(uint32_t uart_periph)
{
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UESM);
}

/*!
    \brief      configure the UART wakeup mode from deep-sleep mode
    \param[in]  uart_periph: USARTx(x=0)
    \param[in]  wum: wakeup mode
                only one parameter can be selected which is shown as below:
      \arg        USART_WUM_ADDR: WUF active on address match
      \arg        USART_WUM_STARTB: WUF active on start bit
      \arg        USART_WUM_RBNE: WUF active on RBNE
    \param[out] none
    \retval     none
*/
void hals_uart_wakeup_mode_config(uint32_t uart_periph, uint32_t wum)
{
    /* disable UART */
    USART_CTL0(uart_periph) &= ~(USART_CTL0_UEN);
    /* reset WUM bit */
    USART_CTL2(uart_periph) &= ~(USART_CTL2_WUM);
    USART_CTL2(uart_periph) |= USART_CTL2_WUM & (wum);
}

/*!
    \brief      enable receive FIFO
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_receive_fifo_enable(uint32_t uart_periph)
{
    USART_RFCS(uart_periph) |= USART_RFCS_RFEN;
}

/*!
    \brief      disable receive FIFO
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     none
*/
void hals_uart_receive_fifo_disable(uint32_t uart_periph)
{
    USART_RFCS(uart_periph) &= ~(USART_RFCS_RFEN);
}

/*!
    \brief      read receive FIFO counter number
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[out] none
    \retval     receive FIFO counter number
*/
uint8_t hals_uart_receive_fifo_counter_number(uint32_t uart_periph)
{
    return (uint8_t)(GET_BITS(USART_RFCS(uart_periph), 12U, 14U));
}

/*!
    \brief      get UART status
    \param[in]  uart_periph: USARTx(x=0,1)
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
FlagStatus hals_uart_flag_get(uint32_t uart_periph, usart_flag_enum flag)
{
    if(RESET != (USART_REG_VAL(uart_periph, flag) & BIT(USART_BIT_POS(flag)))) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear UART status
    \param[in]  uart_periph: USARTx(x=0,1)
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
void hals_uart_flag_clear(uint32_t uart_periph, usart_flag_enum flag)
{
    USART_INTC(uart_periph) |= BIT(USART_BIT_POS(flag));
}

/*!
    \brief      enable UART interrupt
    \param[in]  uart_periph: USARTx(x=0,1)
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
void hals_uart_interrupt_enable(uint32_t uart_periph, usart_interrupt_enum interrupt)
{
    USART_REG_VAL(uart_periph, interrupt) |= BIT(USART_BIT_POS(interrupt));
}

/*!
    \brief      disable UART interrupt
    \param[in]  uart_periph: USARTx(x=0,1)
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
void hals_uart_interrupt_disable(uint32_t uart_periph, usart_interrupt_enum interrupt)
{
    USART_REG_VAL(uart_periph, interrupt) &= ~BIT(USART_BIT_POS(interrupt));
}

/*!
    \brief      get UART interrupt flag status
    \param[in]  uart_periph: USARTx(x=0,1)
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
FlagStatus hals_uart_interrupt_flag_get(uint32_t uart_periph, usart_interrupt_flag_enum int_flag)
{
    uint32_t intenable = 0U, flagstatus = 0U;
    /* get the interrupt enable bit status */
    intenable = (USART_REG_VAL(uart_periph, int_flag) & BIT(USART_BIT_POS(int_flag)));
    /* get the corresponding flag bit status */
    flagstatus = (USART_REG_VAL2(uart_periph, int_flag) & BIT(USART_BIT_POS2(int_flag)));

    if(flagstatus && intenable) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear UART interrupt flag
    \param[in]  uart_periph: USARTx(x=0,1)
    \param[in]  flag: UART interrupt flag
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
void hals_uart_interrupt_flag_clear(uint32_t uart_periph, usart_interrupt_flag_enum int_flag)
{
    if(USART_INT_FLAG_RFF == int_flag) {
        USART_RFCS(uart_periph) &= (uint32_t)(~USART_RFCS_RFFINT);
    } else {
        USART_INTC(uart_periph) |= BIT(USART_BIT_POS2(int_flag));
    }
}

/*!
    \brief      get the mask of data bit
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the mask of data bit(0x7F, 0xFF, 0x1FF)
*/
static uint16_t _uart_data_bit_mask_get(hal_uart_dev_struct *uart)
{
    uint16_t reval;

    if(RESET != (USART_CTL0(uart->periph) & USART_CTL0_WL)) {
        /* check whether the PCEN is enabled */
        if(RESET != (USART_CTL0(uart->periph) & USART_CTL0_PCEN)) {
            reval = 0xFFU;
        } else {
            reval = 0x1FFU;
        }
    } else {
        /* check whether the PCEN is enabled */
        if(RESET != (USART_CTL0(uart->periph) & USART_CTL0_PCEN)) {
            reval = 0x7FU;
        } else {
            reval = 0xFFU;
        }
    }

    return reval;
}

/*!
    \brief      get uart error flag
    \param[in]  uart: uart device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     SET or RESET
*/
static FlagStatus _uart_error_flag_get(hal_uart_dev_struct *uart)
{
    if(0U == (USART_STAT(uart->periph) & (uint32_t)(USART_STAT_PERR | USART_STAT_FERR | \
              USART_STAT_ORERR | USART_STAT_NERR))) {
        return RESET;
    } else {
        return SET;
    }
}

/*!
    \brief      handle the transmit complete interrupt
    \param[in]  uart: pointer to a uart device information structure
    \param[out] none
    \retval     none
*/
static void _uart_transmit_complete_interrupt(void *uart)
{
    hal_uart_dev_struct *p_uart = uart;
    hal_uart_user_cb p_func = (hal_uart_user_cb)p_uart->tx_callback;

    /* disable the transmit complete interrupt */
    hals_uart_interrupt_disable(p_uart->periph, USART_INT_TC);

    /* reset transmit_complete_handle and tx_state */
    p_uart->uart_irq.transmit_complete_handle = NULL;
    p_uart->tx_state = UART_STATE_FREE;

    if(NULL != p_func) {
        /* if there is a user transmit complete callback */
        p_func(p_uart);
    }
}

/*!
    \brief      handle the transmit interrupt
    \param[in]  uart: pointer to a uart device information structure
    \param[out] none
    \retval     none
*/
static void _uart_transmit_interrupt(void *uart)
{
    hal_uart_dev_struct *p_uart = uart;

    if(p_uart->txbuffer.pos < p_uart->txbuffer.length) {
        /* send data: in case of 9bit&no parity, uint16_t data will be transmitted */
        if((RESET != (USART_CTL0(p_uart->periph) & USART_CTL0_WL)) && \
                (RESET == (USART_CTL0(p_uart->periph) & USART_CTL0_PCEN))) {
            /* 9-bit data, none parity */
            hals_uart_data_transmit(p_uart->periph, (*(uint16_t *)p_uart->txbuffer.buffer & (uint16_t)0x1FFU));
            p_uart->txbuffer.buffer += 2;
        } else {
            /* 9-bit data, with parity or 8-bit data */
            hals_uart_data_transmit(p_uart->periph, (*p_uart->txbuffer.buffer & (uint8_t)0xFFU));
            p_uart->txbuffer.buffer++;
        }
        p_uart->txbuffer.pos++;
    } else {
        /* disable the TBE interrupt, enable the TC interrupt and reset the transmit_ready_handle */
        hals_uart_interrupt_disable(p_uart->periph, USART_INT_TBE);
        hals_uart_interrupt_enable(p_uart->periph, USART_INT_TC);
        p_uart->uart_irq.transmit_ready_handle = NULL;
    }
}

/*!
    \brief      handle the receive interrupt
    \param[in]  uart: pointer to a uart device information structure
    \param[out] none
    \retval     none
*/
static void _uart_receive_interrupt(void *uart)
{
    hal_uart_dev_struct *p_uart = uart;
    hal_uart_user_cb p_func = (hal_uart_user_cb)p_uart->rx_callback;

    /* store the received data */
    if(0x1FFU == p_uart->data_bit_mask) {
        *(uint16_t *)p_uart->rxbuffer.buffer = (hals_uart_data_receive(p_uart->periph) &
                                                p_uart->data_bit_mask);
        p_uart->rxbuffer.buffer += 2U;
    } else {
        *p_uart->rxbuffer.buffer = (uint8_t)(hals_uart_data_receive(p_uart->periph) &
                                             p_uart->data_bit_mask);
        p_uart->rxbuffer.buffer++;
    }
    p_uart->rxbuffer.pos++;

    /* receive is finished */
    if(p_uart->rxbuffer.pos == p_uart->rxbuffer.length) {
        /* disable PERR, ERR, RBNE interrupt */
        hals_uart_interrupt_disable(p_uart->periph, USART_INT_PERR);
        hals_uart_interrupt_disable(p_uart->periph, USART_INT_ERR);
        hals_uart_interrupt_disable(p_uart->periph, USART_INT_RBNE);

        /* reset receive_complete_handle and rx_state */
        p_uart->uart_irq.receive_complete_handle = NULL;
        p_uart->rx_state = UART_STATE_FREE;

        if(NULL != p_func) {
            /* if there is a user receive complete callback */
            p_func(p_uart);
        }
    }
}

/*!
    \brief      handle the uart DMA transmit process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _uart_transmit_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_uart_dev_struct *p_uart;
    hal_uart_user_cb p_func;

    p_dma = (hal_dma_dev_struct *)dma;
    p_uart = (hal_uart_dev_struct *)p_dma->p_periph;
    p_func = (hal_uart_user_cb)p_uart->tx_callback;

    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        p_uart->txbuffer.pos = p_uart->txbuffer.length;
        hals_uart_dma_transmit_config(p_uart->periph, USART_TRANSMIT_DMA_DISABLE);
        /* enable TC interrupt */
        hals_uart_interrupt_enable(p_uart->periph, USART_INT_TC);
        /* reset rx_state */
        p_uart->tx_state = UART_STATE_FREE;
    }

    if(NULL != p_func) {
        /* if there is a user receive complete callback */
        p_func(p_uart);
    }
}

/*!
    \brief      handle the uart DMA receive process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _uart_receive_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_uart_dev_struct *p_uart;
    hal_uart_user_cb p_func;

    p_dma = (hal_dma_dev_struct *)dma;
    p_uart = (hal_uart_dev_struct *)p_dma->p_periph;
    p_func = (hal_uart_user_cb)p_uart->rx_callback;

    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        p_uart->rxbuffer.pos = p_uart->rxbuffer.length;
        /* disable DMA receive, PERR and ERR interrupt */
        hals_uart_dma_receive_config(p_uart->periph, USART_RECEIVE_DMA_DISABLE);
        hals_uart_interrupt_disable(p_uart->periph, USART_INT_PERR);
        hals_uart_interrupt_disable(p_uart->periph, USART_INT_ERR);
        /* reset rx_state */
        p_uart->rx_state = UART_STATE_FREE;
    }

    if(NULL != p_func) {
        /* if there is a user receive complete callback */
        p_func(p_uart);
    }
}

/*!
    \brief      handle the uart DMA error process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _uart_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_uart_dev_struct *p_uart;

    p_dma = (hal_dma_dev_struct *)dma;
    p_uart = (hal_uart_dev_struct *)p_dma->p_periph;

    if(UART_STATE_BUSY == p_uart->tx_state) {
        /* transmit state is busy */
        p_uart->error_state |= HAL_UART_ERROR_DMATX;
        p_uart->last_error = HAL_UART_ERROR_DMATX;
        p_uart->txbuffer.pos = p_uart->txbuffer.length;
        /* disable DMA transmit and reset tx_state */
        hals_uart_dma_transmit_config(p_uart->periph, USART_TRANSMIT_DMA_DISABLE);
        p_uart->tx_state = UART_STATE_FREE;
    } else if(UART_STATE_BUSY == p_uart->rx_state) {
        /* receive state is busy */
        p_uart->error_state |= HAL_UART_ERROR_DMARX;
        p_uart->last_error = HAL_UART_ERROR_DMARX;
        p_uart->rxbuffer.pos = p_uart->rxbuffer.length;
        /* disable DMA receive, PERR, ERR interrupt */
        hals_uart_dma_receive_config(p_uart->periph, USART_RECEIVE_DMA_DISABLE);
        hals_uart_interrupt_disable(p_uart->periph, USART_INT_PERR);
        hals_uart_interrupt_disable(p_uart->periph, USART_INT_ERR);
        /* reset rx_state */
        p_uart->rx_state = UART_STATE_FREE;
    } else {
        HAL_DEBUGE("uart processor fatal error: dma error exception due to run state");
    }

    if(p_uart->uart_irq.error_handle != NULL) {
        /* if there is a user error callback */
        p_uart->uart_irq.error_handle(p_uart);
        p_uart->error_state = HAL_UART_ERROR_NONE;
    }
}
