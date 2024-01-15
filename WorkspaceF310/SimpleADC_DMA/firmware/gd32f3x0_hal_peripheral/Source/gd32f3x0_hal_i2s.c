/*!
    \file    gd32f3x0_hal_i2s.c
    \brief   I2S driver

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

#if (defined(GD32F350) || defined(GD32F310))
#define I2S_INIT_MASK                   ((uint32_t)0x0000F047U)  /*!< I2S parameter initialization mask */
#define SPI_I2SPSC_RESET                ((uint32_t)0x00000002U)  /*!< default value of SPI_I2SPSC register */

/* I2S private function */
static void _i2s_deinit(uint32_t i2s_periph);
static void _i2s_transmit_complete_dma(void *dma);
static void _i2s_receive_complete_dma(void *dma);
static void _i2s_dma_error(void *dma);
static void _i2s_transmit_interrupt(void *i2s);
static void _i2s_receive_interrupt(void *i2s);
static FlagStatus _i2s_flag_get(uint32_t periph, uint32_t flag);
static FlagStatus _i2s_interrupt_flag_get(uint32_t periph, uint8_t interrupt);

/*!
    \brief      deinitialize I2S
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_deinit(hal_i2s_dev_struct *i2s)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == i2s) {
        HAL_DEBUGE("parameter [*i2s] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(SPI0 == i2s->periph) {
        i2s->state = HAL_I2S_STATE_BUSY;
        /* reset I2S */
        _i2s_deinit(i2s->periph);
        hal_i2s_struct_init(HAL_I2S_DEV_STRUCT, i2s);
        i2s->state = HAL_I2S_STATE_READY;
    } else {
        HAL_DEBUGE("parameter [i2c->periph] value is invalid");
    }
}

/*!
    \brief      initialize the I2S structure
    \param[in]  hal_struct_type: refer to hal_i2s_struct_type_enum
    \param[in]  p_struct: point to I2S structure that contains the configuration information
    \param[out] none
    \retval     none
*/
void hal_i2s_struct_init(hal_i2s_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct) {
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_I2S_INIT_STRUCT:
        /* initialize I2S initialization structure with the default values */
        ((hal_i2s_init_struct *)p_struct)->mode        = I2S_MODE_MASTERTX ;
        ((hal_i2s_init_struct *)p_struct)->standard    = I2S_STD_MSB ;
        ((hal_i2s_init_struct *)p_struct)->frameformat = I2S_FRAMEFORMAT_DT32B_CH32B;
        ((hal_i2s_init_struct *)p_struct)->mckout      = I2S_MCKOUT_DISABLE;
        ((hal_i2s_init_struct *)p_struct)->audiosample = I2S_AUDIOSAMPLE_8K;
        ((hal_i2s_init_struct *)p_struct)->ckpl        = I2S_CKPL_HIGH;
        break;

    case HAL_I2S_DEV_STRUCT:
        /* initialize I2S device information structure with the default values */
        ((hal_i2s_dev_struct *)p_struct)->periph                   = SPI0;
        ((hal_i2s_dev_struct *)p_struct)->i2s_irq.error_handle     = NULL;
        ((hal_i2s_dev_struct *)p_struct)->i2s_irq.receive_handler  = NULL;
        ((hal_i2s_dev_struct *)p_struct)->i2s_irq.transmit_handler = NULL;
        ((hal_i2s_dev_struct *)p_struct)->p_dma_rx                 = NULL;
        ((hal_i2s_dev_struct *)p_struct)->p_dma_tx                 = NULL;
        ((hal_i2s_dev_struct *)p_struct)->rx_callback              = NULL;
        ((hal_i2s_dev_struct *)p_struct)->tx_callback              = NULL;
        ((hal_i2s_dev_struct *)p_struct)->error_callback           = NULL;
        ((hal_i2s_dev_struct *)p_struct)->txbuffer.buffer          = NULL;
        ((hal_i2s_dev_struct *)p_struct)->txbuffer.length          = 0U;
        ((hal_i2s_dev_struct *)p_struct)->txbuffer.pos             = 0U;
        ((hal_i2s_dev_struct *)p_struct)->rxbuffer.buffer          = NULL;
        ((hal_i2s_dev_struct *)p_struct)->rxbuffer.length          = 0U;
        ((hal_i2s_dev_struct *)p_struct)->rxbuffer.pos             = 0U;
        ((hal_i2s_dev_struct *)p_struct)->state                    = HAL_I2S_STATE_READY;
        ((hal_i2s_dev_struct *)p_struct)->error_code               = HAL_I2S_ERROR_NONE;
        ((hal_i2s_dev_struct *)p_struct)->mutex                    = HAL_MUTEX_UNLOCKED;
        break;

    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      initialize I2S
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which I2S is initialized
                    this parameter can only be I2S0 for gd32f3x0
    \param[in]  p_init: the initialization data needed to initialize I2S
                  mode: I2S_MODE_SLAVETX, I2S_MODE_SLAVERX, I2S_MODE_MASTERTX, I2S_MODE_MASTERRX
                  standard: the argument could be selected from enumeration <hal_i2s_standard_enum>
                  frameformat: I2S_FRAMEFORMAT_DT16B_CH16B, I2S_FRAMEFORMAT_DT16B_CH32B, I2S_FRAMEFORMAT_DT24B_CH32B, I2S_FRAMEFORMAT_DT32B_CH32B
                  mckout: I2S_MCKOUT_DISABLE, I2S_MCKOUT_ENABLE
                  audiosample: the argument could be selected from enumeration <hal_i2s_audiosample_enum>
                  ckpl:I2S_CKPL_LOW, I2S_CKPL_HIGH
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS,HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_i2s_init(hal_i2s_dev_struct *i2s, uint32_t periph, hal_i2s_init_struct *p_init)
{
    uint32_t reg = 0U;
    uint32_t i2sdiv = 2U, i2sof = 0U;
    uint32_t clks = 0U;
    uint32_t i2sclock = 0U;

#if (1 == HAL_PARAMETER_CHECK)
    /* check I2S pointer and p_init address */
    if((NULL == i2s) && (NULL == p_init)) {
        return HAL_ERR_ADDRESS;
    }

    /* check periph parameter */
    if(SPI0 != periph) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    i2s->state = HAL_I2S_STATE_BUSY;
    i2s->periph = periph;
    /* disable the peripheral */
    __HAL_SPI_DISABLE(periph);

    /* init I2S */
    reg = SPI_I2SCTL(periph);
    reg &= I2S_INIT_MASK;

    /* enable I2S mode */
    reg |= (uint32_t)SPI_I2SCTL_I2SSEL;
    /* select I2S mode */
    reg |= (uint32_t)p_init->mode;
    /* select I2S standard */
    reg |= (uint32_t)p_init->standard;
    /* select I2S polarity */
    reg |= (uint32_t)p_init->ckpl;

    /* write to SPI_I2SCTL register */
    SPI_I2SCTL(periph) = (uint32_t)reg;

    /* configure I2S prescaler */
    /* deinitialize SPI_I2SPSC register */
    SPI_I2SPSC(periph) = SPI_I2SPSC_RESET;

    /* get system clock */
    i2sclock = hals_rcu_clock_freq_get(CK_SYS);

    /* configure the prescaler depending on the mclk output state, the frame format and audio sample rate */
    if(I2S_MCKOUT_ENABLE == p_init->mckout) {
        clks = (uint32_t)(((i2sclock / 256U) * 10U) / p_init->audiosample);
    } else {
        if(I2S_FRAMEFORMAT_DT16B_CH16B == p_init->frameformat) {
            clks = (uint32_t)(((i2sclock / 32U) * 10U) / p_init->audiosample);
        } else {
            clks = (uint32_t)(((i2sclock / 64U) * 10U) / p_init->audiosample);
        }
    }

    /* remove the floating point */
    clks = (clks + 5U) / 10U;
    i2sof = (clks & 0x00000001U);
    i2sdiv = ((clks - i2sof) / 2U);
    i2sof  = (i2sof << 8U);

    /* set the default values */
    if((i2sdiv < 2U) || (i2sdiv > 255U)) {
        i2sdiv = 2U;
        i2sof = 0U;
    }

    /* configure SPI_I2SPSC */
    SPI_I2SPSC(periph) = (uint32_t)(i2sdiv | i2sof | p_init->mckout);

    /* clear SPI_I2SCTL_DTLEN and SPI_I2SCTL_CHLEN bits */
    SPI_I2SCTL(periph) &= (uint32_t)(~(SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN));
    /* configure data frame format */
    SPI_I2SCTL(periph) |= (uint32_t)p_init->frameformat;

    i2s->state = HAL_I2S_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data, poll transmit process and completed status, the function is blocking
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer
    \param[in]  length: length of data to be sent
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_i2s_transmit_poll(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, uint32_t timeout_ms)
{
    __IO uint32_t tmp = 0x0U;
    uint32_t tmp1 = 0U;
    uint32_t tick_start = 0;

#if (1 == HAL_PARAMETER_CHECK)
    /* check input parameter */
    if((NULL == i2s) || (NULL == p_buffer) || (0U == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock I2S */
    HAL_LOCK(i2s);

    if(HAL_I2S_STATE_READY == i2s->state) {
        tmp1 = SPI_I2SCTL(i2s->periph) & ((SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN));
        /* check I2S frameformat */
        if((I2S_FRAMEFORMAT_DT24B_CH32B == tmp1) || (I2S_FRAMEFORMAT_DT32B_CH32B == tmp1)) {
            i2s->txbuffer.length = (length << 1U);
            i2s->txbuffer.pos = (length << 1U);
        } else {
            i2s->txbuffer.length = length;
            i2s->txbuffer.pos = length;
        }

        i2s->error_code = HAL_I2S_ERROR_NONE;
        i2s->state = HAL_I2S_STATE_BUSY_TX;

        /* enable I2S peripheral */
        __HAL_I2S_ENABLE(i2s->periph);

        /* transmit data */
        while(i2s->txbuffer.pos > 0U) {
            SPI_DATA(i2s->periph) = (*p_buffer++);
            i2s->txbuffer.pos--;

            /* wait TBE flag is set */
            tick_start = hal_sys_basetick_count_get();
            while(RESET == _i2s_flag_get(i2s->periph, I2S_FLAG_TBE)) {
                if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                    if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }
            if(SET == _i2s_flag_get(i2s->periph, I2S_FLAG_TXURERR)) {
                /* clear underrun flag */
                tmp = SPI_STAT((i2s)->periph);

                i2s->state = HAL_I2S_STATE_READY;
                i2s->error_code = HAL_I2S_ERROR_UNDERRUN;
                return HAL_ERR_HARDWARE;
            }
        }
        i2s->state = HAL_I2S_STATE_READY;

        /* unlock I2S */
        HAL_UNLOCK(i2s);

        return HAL_ERR_NONE;
    } else {

        /* unlock I2S */
        HAL_UNLOCK(i2s);

        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      receive amounts of data, poll receive process and completed status, the function is blocking
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer
    \param[in]  length: length of data to be sent
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_i2s_receive_poll(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, uint32_t timeout_ms)
{
    uint32_t tmp1 = 0U;
    __IO uint32_t tmp = 0x00U;
    uint32_t tick_start = 0;

#if (1 == HAL_PARAMETER_CHECK)
    /* check input parameter */
    if((NULL == i2s) || (NULL == p_buffer) || (0U == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* lock I2S */
    HAL_LOCK(i2s);

    if(HAL_I2S_STATE_READY == i2s->state) {
        /* check I2S frameformat */
        tmp1 = SPI_I2SCTL(i2s->periph) & (SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN);
        if((tmp1 == I2S_FRAMEFORMAT_DT24B_CH32B) || (tmp1 == I2S_FRAMEFORMAT_DT32B_CH32B)) {
            i2s->rxbuffer.length = (length << 1U);
            i2s->rxbuffer.pos = (length << 1U);
        } else {
            i2s->rxbuffer.length = length;
            i2s->rxbuffer.pos = length;
        }

        i2s->error_code = HAL_ERR_NONE;
        i2s->state = HAL_I2S_STATE_BUSY_RX;

        if(SPI_I2SCTL_I2SEN != (SPI_I2SCTL(i2s->periph) &SPI_I2SCTL_I2SEN)) {
            /* enable I2S peripheral */
            __HAL_I2S_ENABLE(i2s->periph);
        }

        /* receive data */
        while(i2s->rxbuffer.pos > 0U) {
            tick_start = hal_sys_basetick_count_get();
            while(RESET == _i2s_flag_get(i2s->periph, I2S_FLAG_RBNE)) {
                if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                    if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }

            if(SET == _i2s_flag_get(i2s->periph, I2S_FLAG_RXORERR)) {
                /* clear overrun error */
                tmp = SPI_DATA(i2s->periph);
                tmp = SPI_STAT(i2s->periph);
                /* set the I2S state ready */
                i2s->state = HAL_I2S_STATE_READY;
                i2s->error_code = HAL_I2S_ERROR_OVERRUN;
                return HAL_ERR_HARDWARE;
            }

            (*p_buffer++) = SPI_DATA(i2s->periph);
            i2s->rxbuffer.pos--;
        }

        i2s->state = HAL_I2S_STATE_READY;

        /* unlock I2S */
        HAL_UNLOCK(i2s);

        return HAL_ERR_NONE;
    } else {

        /* unlock I2S */
        HAL_UNLOCK(i2s);

        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      transmit amounts of data by interrupt method, the function is non-blocking
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer
    \param[in]  length: length of data to be sent
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_i2s_transmit_interrupt(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length,
                                   hal_i2s_user_callback_struct *p_user_func)
{
    uint32_t tmp1 = 0U;

#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == i2s) || (NULL == p_buffer) || (0U == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* lock I2S */
    HAL_LOCK(i2s);

    if(i2s->state == HAL_I2S_STATE_READY) {
        i2s->txbuffer.buffer = p_buffer;
        /* check I2S frameformat */
        tmp1 = SPI_I2SCTL(i2s->periph) & (SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN);

        if((I2S_FRAMEFORMAT_DT24B_CH32B == tmp1) || (I2S_FRAMEFORMAT_DT32B_CH32B == tmp1)) {
            i2s->txbuffer.length = (length << 1U);
            i2s->txbuffer.pos = (length << 1U);
        } else {
            i2s->txbuffer.length = length;
            i2s->txbuffer.pos = length;
        }
        i2s->i2s_irq.transmit_handler = _i2s_transmit_interrupt;
        i2s->tx_callback = (void *)p_user_func->complete_func;
        i2s->error_callback = (void *)p_user_func->error_func;

        i2s->state = HAL_I2S_STATE_BUSY_TX;

        /* enable TBE and ERR interrupt */
        __HAL_I2S_INT_ENABLE(i2s->periph, I2S_INT_TBE);
        __HAL_I2S_INT_ENABLE(i2s->periph, I2S_INT_ERR);

        if(SPI_I2SCTL_I2SEN != (SPI_I2SCTL(i2s->periph) &SPI_I2SCTL_I2SEN)) {
            /* enable I2S peripheral */
            __HAL_I2S_ENABLE(i2s->periph);
        }

        /* unlock I2S */
        HAL_UNLOCK(i2s);

        return HAL_ERR_NONE;
    } else {

        /* unlock I2S */
        HAL_UNLOCK(i2s);

        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      receive amounts of data by interrupt method, the function is non-blocking
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer
    \param[in]  length: length of data to be sent
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_i2s_receive_interrupt(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length,
                                  hal_i2s_user_callback_struct *p_user_func)
{
    uint32_t tmp1 = 0U;

#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == i2s) || (NULL == p_buffer) || (0U == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* lock I2S */
    HAL_LOCK(i2s);

    if(i2s->state == HAL_I2S_STATE_READY) {
        i2s->rxbuffer.buffer = p_buffer;
        /* check I2S frameformat */
        tmp1 = SPI_I2SCTL(i2s->periph) & (SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN);

        if((I2S_FRAMEFORMAT_DT24B_CH32B == tmp1) || (I2S_FRAMEFORMAT_DT32B_CH32B == tmp1)) {
            i2s->rxbuffer.length = (length << 1U);
            i2s->rxbuffer.pos = (length << 1U);
        } else {
            i2s->rxbuffer.length = length;
            i2s->rxbuffer.pos = length;
        }

        i2s->i2s_irq.receive_handler = _i2s_receive_interrupt;
        i2s->rx_callback = (void *)p_user_func->complete_func;
        i2s->error_callback = (void *)p_user_func->error_func;
        i2s->state     = HAL_I2S_STATE_BUSY_RX;

        /* enable RBNE and ERR interrupt */
        __HAL_I2S_INT_ENABLE(i2s->periph, I2S_INT_RBNE);
        __HAL_I2S_INT_ENABLE(i2s->periph, I2S_INT_ERR);

        if(SPI_I2SCTL_I2SEN != (SPI_I2SCTL(i2s->periph) &SPI_I2SCTL_I2SEN)) {
            /* enable I2S peripheral */
            __HAL_I2S_ENABLE(i2s->periph);
        }

        /* unlock I2S */
        HAL_UNLOCK(i2s);

        return HAL_ERR_NONE;
    } else {

        /* unlock I2S */
        HAL_UNLOCK(i2s);

        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      transmit amounts of data by DMA method, the function is non-blocking
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer
    \param[in]  length: length of data to be sent
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_i2s_transmit_dma(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length,
                             hal_i2s_user_callback_struct *p_user_func)
{
    uint32_t *p_tmp = NULL;
    uint32_t tmp1 = 0U;
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == i2s) || (NULL == p_buffer) || (0U == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* lock I2S */
    HAL_LOCK(i2s);

    if(HAL_I2S_STATE_READY == i2s->state) {
        i2s->txbuffer.buffer = p_buffer;
        /* check I2S frameformat */
        tmp1 = SPI_I2SCTL(i2s->periph) & (SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN);
        if((I2S_FRAMEFORMAT_DT24B_CH32B == tmp1) || (I2S_FRAMEFORMAT_DT32B_CH32B == tmp1)) {
            i2s->txbuffer.length = (length << 1U);
            i2s->txbuffer.pos = (length << 1U);
        } else {
            i2s->txbuffer.length = length;
            i2s->txbuffer.pos = length;
        }
        i2s->state = HAL_I2S_STATE_BUSY_TX;
        /* I2S callback function */
        i2s->tx_callback = (void *)p_user_func->complete_func;
        i2s->error_callback = (void *)p_user_func->error_func;

        /* I2S DMA transmit info*/
        hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
        dma_irq.half_finish_handle = NULL;
        dma_irq.full_finish_handle = _i2s_transmit_complete_dma;
        dma_irq.error_handle = _i2s_dma_error;
        p_tmp = (uint32_t *)&p_buffer;
        hal_dma_start_interrupt(i2s->p_dma_tx, *(uint32_t *)p_tmp, (uint32_t)&SPI_DATA(i2s->periph), length, &dma_irq);

        /* enable I2S peripheral */
        __HAL_I2S_ENABLE(i2s->periph);

        /* enable transmit DMA request */
        __HAL_I2S_DMA_ENABLE(i2s->periph, SPI_DMA_TRANSMIT);

        /* unlock I2S */
        HAL_UNLOCK(i2s);

        return HAL_ERR_NONE;
    } else {

        /* unlock I2S */
        HAL_UNLOCK(i2s);

        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      receive amounts of data by DMA method, the function is non-blocking
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer
    \param[in]  length: length of data to be sent
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL details refer to gd32f3x0_hal.h
*/
int32_t hal_i2s_receive_dma(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length,
                            hal_i2s_user_callback_struct *p_user_func)
{
    uint32_t *p_tmp = NULL;
    __IO uint32_t tmp1 = 0U;
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == i2s) || (NULL == p_buffer) || (0U == length)) {
        return  HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* lock I2S */
    HAL_LOCK(i2s);

    if(HAL_I2S_STATE_READY == i2s->state) {
        i2s->rxbuffer.buffer = p_buffer;
        /* check I2S frameformat */
        tmp1 = SPI_I2SCTL(i2s->periph) & (SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN);

        if((I2S_FRAMEFORMAT_DT24B_CH32B == tmp1) ||
                (I2S_FRAMEFORMAT_DT32B_CH32B == tmp1)) {
            i2s->rxbuffer.length = (length << 1U);
            i2s->rxbuffer.pos = (length << 1U);
        } else {
            i2s->rxbuffer.length = length;
            i2s->rxbuffer.pos = length;
        }

        i2s->state = HAL_I2S_STATE_BUSY_RX;

        /* I2S callback function */
        i2s->rx_callback = (void *)p_user_func->complete_func;
        i2s->error_callback = (void *)p_user_func->error_func;

        /* I2S DMA receive info*/
        hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
        dma_irq.half_finish_handle = NULL;
        dma_irq.full_finish_handle = _i2s_receive_complete_dma;
        dma_irq.error_handle = _i2s_dma_error;
        p_tmp = (uint32_t *)&p_buffer;
        hal_dma_start_interrupt(i2s->p_dma_rx, (uint32_t)&SPI_DATA(i2s->periph), *(uint32_t *)p_tmp, length, &dma_irq);

        __HAL_I2S_ENABLE(i2s->periph);
        /* enable receive DMA request */
        __HAL_I2S_DMA_ENABLE(i2s->periph, SPI_DMA_RECEIVE);

        /* unlock I2S */
        HAL_UNLOCK(i2s);

        return HAL_ERR_NONE;
    } else {

        /* unlock I2S */
        HAL_UNLOCK(i2s);

        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      I2S interrupt handler content function,which is merely used in i2s_handler
    \param[in]  i2s: I2S device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_irq(hal_i2s_dev_struct *i2s)
{
    __IO uint32_t tmp = 0x00U;
    hal_i2s_user_cb p_func;

#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == i2s) {
        HAL_DEBUGE("parameter [*i2s] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(HAL_I2S_STATE_BUSY_RX == i2s->state) {
        if(RESET != _i2s_interrupt_flag_get(i2s->periph, I2S_INT_FLAG_RBNE)) {
            i2s->i2s_irq.receive_handler(i2s);
        }

        /* I2S overrun error interrupt occured */
        if(RESET != _i2s_interrupt_flag_get(i2s->periph, I2S_INT_FLAG_RXORERR)) {
            /* disable RBNE and ERR interrupt */
            __HAL_I2S_INT_DISABLE(i2s->periph, I2S_INT_RBNE);
            __HAL_I2S_INT_DISABLE(i2s->periph, I2S_INT_ERR);

            /* clear overrun error */
            tmp = SPI_DATA(i2s->periph);
            tmp = SPI_STAT(i2s->periph);

            i2s->state = HAL_I2S_STATE_READY;
            /* set the error code and execute error callback */
            i2s->error_code = HAL_I2S_ERROR_OVERRUN;
            p_func = (hal_i2s_user_cb)i2s->error_callback;
            if(NULL != p_func) {
                p_func(i2s);
            }
        }
    }

    if(HAL_I2S_STATE_BUSY_TX == i2s->state) {
        if(RESET != _i2s_interrupt_flag_get(i2s->periph, I2S_INT_FLAG_TBE)) {
            i2s->i2s_irq.transmit_handler(i2s);
        }

        /* I2S underrun error interrupt occurred */
        if(RESET != _i2s_interrupt_flag_get(i2s->periph, I2S_INT_FLAG_TXURERR)) {
            /* disable TBE and ERR interrupt */
            __HAL_I2S_INT_DISABLE(i2s->periph, I2S_INT_TBE);
            __HAL_I2S_INT_DISABLE(i2s->periph, I2S_INT_ERR);

            /* clear underrun flag */
            tmp = SPI_STAT((i2s)->periph);

            i2s->state = HAL_I2S_STATE_READY;
            /* set the error code and execute error callback */
            i2s->error_code = HAL_I2S_ERROR_UNDERRUN;
            p_func = (hal_i2s_user_cb)i2s->error_callback;
            if(NULL != p_func) {
                p_func(i2s);
            }
        }
    }
}

/*!
    \brief      set user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to I2S interrupt callback function pointer structure
                  The structure member can be assigned as following parameters:
      \arg        hal_irq_handle_cb function pointer: the function is user-defined,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
      \arg        NULL: the corresponding callback mechanism is out of use, and
                     disable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_i2s_irq_handle_set(hal_i2s_dev_struct *i2s, hal_i2s_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == i2s) {
        HAL_DEBUGE("parameter [*i2s] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(NULL != p_irq->error_handle) {
        i2s->i2s_irq.error_handle = p_irq->error_handle;
        __HAL_I2S_INT_ENABLE(i2s->periph, I2S_INT_ERR);
    } else {
        i2s->i2s_irq.error_handle = NULL;
        __HAL_I2S_INT_DISABLE(i2s->periph, I2S_INT_ERR);
    }

    if(NULL != p_irq->receive_handler) {
        i2s->i2s_irq.receive_handler = p_irq->receive_handler;
        __HAL_I2S_INT_ENABLE(i2s->periph, I2S_INT_RBNE);
    } else {
        i2s->i2s_irq.receive_handler = NULL;
        __HAL_I2S_INT_DISABLE(i2s->periph, I2S_INT_RBNE);
    }

    if(NULL != p_irq->transmit_handler) {
        i2s->i2s_irq.transmit_handler = p_irq->transmit_handler;
        __HAL_I2S_INT_ENABLE(i2s->periph, I2S_INT_TBE);
    } else {
        i2s->i2s_irq.transmit_handler = NULL;
        __HAL_I2S_INT_DISABLE(i2s->periph, I2S_INT_TBE);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_irq_handle_all_reset(hal_i2s_dev_struct *i2s)
{
    i2s->i2s_irq.error_handle = NULL;
    i2s->i2s_irq.transmit_handler = NULL;
    i2s->i2s_irq.receive_handler = NULL;
}

/*!
    \brief      start I2S module function
    \param[in]  i2s: i2s device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_start(hal_i2s_dev_struct *i2s)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == i2s) {
        HAL_DEBUGE("parameter [*i2s] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    __HAL_I2S_ENABLE(i2s->periph);
}

/*!
    \brief      stop I2S module function
    \param[in]  i2s: I2S device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_stop(hal_i2s_dev_struct *i2s)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == i2s) {
        HAL_DEBUGE("parameter [*i2s] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    __HAL_I2S_DISABLE(i2s->periph);
}

/*!
    \brief      pause I2S DMA
    \param[in]  i2s: i2s device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_dma_pause(hal_i2s_dev_struct *i2s)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == i2s) {
        HAL_DEBUGE("parameter [*i2s] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(HAL_I2S_STATE_BUSY_TX == i2s->state) {
        /* disable the I2S DMA transmit request */
        __HAL_I2S_DMA_DISABLE(i2s->periph, SPI_DMA_TRANSMIT);
    } else {
        if(HAL_I2S_STATE_BUSY_RX == i2s->state) {
            /* disable the I2S DMA receive request */
            __HAL_I2S_DMA_DISABLE(i2s->periph, SPI_DMA_RECEIVE);
        }
    }
}

/*!
    \brief      II2S DMA resume function
    \param[in]  i2s: i2s device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_dma_resume(hal_i2s_dev_struct *i2s)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == i2s) {
        HAL_DEBUGE("parameter [*i2s] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(HAL_I2S_STATE_BUSY_TX == i2s->state) {
        /* disable the I2S DMA transmit request */
        __HAL_I2S_DMA_ENABLE(i2s->periph, SPI_DMA_TRANSMIT);
    } else {
        if(HAL_I2S_STATE_BUSY_RX == i2s->state) {
            /* disable the I2S DMA receive request */
            __HAL_I2S_DMA_ENABLE(i2s->periph, SPI_DMA_RECEIVE);
        }
    }

    if(SPI_I2SCTL_I2SEN != (SPI_I2SCTL(i2s->periph) &SPI_I2SCTL_I2SEN)) {
        /* enable I2S peripheral */
        __HAL_I2S_ENABLE(i2s->periph);
    }
}

/*!
    \brief      I2S DMA stop function
    \param[in]  i2s: i2s device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_dma_stop(hal_i2s_dev_struct *i2s)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == i2s) {
        HAL_DEBUGE("parameter [*i2s] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(HAL_I2S_STATE_BUSY_TX == i2s->state) {
        /* disable the I2S DMA transmit request */
        __HAL_I2S_DMA_DISABLE(i2s->periph, SPI_DMA_TRANSMIT);

        /* disable the I2S DMA channel */
        hal_dma_stop(i2s->p_dma_tx);
    } else {
        if(HAL_I2S_STATE_BUSY_RX == i2s->state) {
            /* disable the I2S DMA receive request */
            __HAL_I2S_DMA_DISABLE(i2s->periph, SPI_DMA_RECEIVE);

            /* disable the I2S DMA channel */
            hal_dma_stop(i2s->p_dma_rx);
        }
    }
    /* disable I2S peripheral */
    __HAL_SPI_DISABLE(i2s->periph);
    i2s->state = HAL_I2S_STATE_READY;
}

/*!
    \brief      reset I2S
    \param[in]  i2s_periph: SPI0
    \param[out] none
    \retval     none
*/
static void _i2s_deinit(uint32_t i2s_periph)
{
    /* reset I2S0 */
    hal_rcu_periph_reset_enable(RCU_SPI0RST);
    hal_rcu_periph_reset_disable(RCU_SPI0RST);
}

/*!
    \brief      I2S DMA transmit handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _i2s_transmit_complete_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_i2s_dev_struct *p_i2s;
    hal_i2s_user_cb p_func;

    p_dma = (hal_dma_dev_struct *)dma;
    p_i2s = (hal_i2s_dev_struct *)p_dma->p_periph;
    p_func = (hal_i2s_user_cb)p_i2s->tx_callback;
    /* DMA normal mode */
    if(SET != (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        __HAL_I2S_DMA_DISABLE(p_i2s->periph, SPI_DMA_TRANSMIT);
        p_i2s->txbuffer.length = 0;
        p_i2s->state = HAL_I2S_STATE_READY;
    }
    /* DMA transmit complete callback */
    if(NULL != p_func) {
        p_func(p_i2s);
    }
}

/*!
    \brief      I2S DMA receive handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _i2s_receive_complete_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_i2s_dev_struct *p_i2s;
    hal_i2s_user_cb p_func;

    p_dma = (hal_dma_dev_struct *)dma;
    p_i2s = (hal_i2s_dev_struct *)p_dma->p_periph;
    p_func = (hal_i2s_user_cb)p_i2s->rx_callback;
    /* DMA normal mode */
    if(SET != (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        __HAL_I2S_DMA_DISABLE(p_i2s->periph, SPI_DMA_RECEIVE);
        p_i2s->rxbuffer.length = 0;
        p_i2s->state = HAL_I2S_STATE_READY;
    }
    /* DMA receive complete callback */
    if(NULL != p_func) {
        p_func(p_i2s);
    }
}

/*!
    \brief      I2S DMA communication error
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _i2s_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_i2s_dev_struct *p_i2s;
    hal_i2s_user_cb p_func;

    p_dma = (hal_dma_dev_struct *)dma;
    p_i2s = (hal_i2s_dev_struct *)p_dma->p_periph;
    p_func = (hal_i2s_user_cb)p_i2s->error_callback;

    /* disable SPI transmit and receive DMA */
    __HAL_I2S_DMA_DISABLE(p_i2s->periph, SPI_DMA_RECEIVE);
    __HAL_I2S_DMA_DISABLE(p_i2s->periph, SPI_DMA_TRANSMIT);
    p_i2s->txbuffer.length = 0;
    p_i2s->rxbuffer.length = 0;
    p_i2s->state = HAL_I2S_STATE_READY ;

    /* error callback */
    if(NULL != p_func) {
        p_func(p_i2s);
    }
}

/*!
    \brief      I2S transmit interrupt handler
    \param[in]  i2s: I2S device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _i2s_transmit_interrupt(void *i2s)
{
    hal_i2s_dev_struct *p_i2s = i2s;
    hal_i2s_user_cb p_func = (hal_i2s_user_cb)p_i2s->tx_callback;
    /* transmit data */
    SPI_DATA(p_i2s->periph) = (*p_i2s->txbuffer.buffer++);
    p_i2s->txbuffer.pos--;

    if(0U == p_i2s->txbuffer.pos) {
        /* disable TBE and ERR interrupt */
        __HAL_I2S_INT_DISABLE(p_i2s->periph, I2S_INT_TBE);
        __HAL_I2S_INT_DISABLE(p_i2s->periph, I2S_INT_ERR);
        p_i2s->state = HAL_I2S_STATE_READY;

        if(NULL != p_func) {
            p_func(p_i2s);
        }
    }
}

/*!
    \brief      I2S receive interrupt handler
    \param[in]  i2s: I2S device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _i2s_receive_interrupt(void *i2s)
{
    hal_i2s_dev_struct *p_i2s = i2s;
    hal_i2s_user_cb p_func = (hal_i2s_user_cb)p_i2s->rx_callback;

    /* receive data */
    (*p_i2s->rxbuffer.buffer++) = SPI_DATA(p_i2s->periph);
    p_i2s->rxbuffer.pos--;

    if(0U == p_i2s->rxbuffer.pos) {
        /* disable RBNE and ERR interrupt */
        __HAL_I2S_INT_DISABLE(p_i2s->periph, I2S_INT_RBNE);
        __HAL_I2S_INT_DISABLE(p_i2s->periph, I2S_INT_ERR);

        p_i2s->state = HAL_I2S_STATE_READY;

        if(NULL != p_func) {
            p_func(p_i2s);
        }
    }
}

/*!
    \brief      get I2S flag status
    \param[in]  spi_periph: SPI0
    \param[in]  flag: I2S flag status
                only one parameter can be selected which are shown as below:
      \arg        I2S_FLAG_TBE: transmit buffer empty flag
      \arg        I2S_FLAG_RBNE: receive buffer not empty flag
      \arg        I2S_FLAG_TRANS: transmit on-going flag
      \arg        I2S_FLAG_RXORERR: overrun error flag
      \arg        I2S_FLAG_TXURERR: underrun error flag
      \arg        I2S_FLAG_CH: channel side flag
      \arg        I2S_FLAG_FERR: I2S format error interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
static FlagStatus _i2s_flag_get(uint32_t periph, uint32_t flag)
{
    if(RESET != (SPI_STAT(periph) & flag)) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      get I2S interrupt flag status
    \param[in]  spi_periph: SPI0
    \param[in]  interrupt: I2S interrupt flag status
                only one parameter can be selected which is shown as below:
      \arg        I2S_INT_FLAG_TBE: transmit buffer empty interrupt flag
      \arg        I2S_INT_FLAG_RBNE: receive buffer not empty interrupt flag
      \arg        I2S_INT_FLAG_RXORERR: overrun interrupt flag
      \arg        I2S_INT_FLAG_TXURERR: underrun error interrupt flag
      \arg        I2S_INT_FLAG_FERR: format error interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
static FlagStatus _i2s_interrupt_flag_get(uint32_t periph, uint8_t interrupt)
{
    uint32_t reg1 = SPI_STAT(periph);
    uint32_t reg2 = SPI_CTL1(periph);

    switch(interrupt) {
    /* I2S transmit buffer empty interrupt */
    case I2S_INT_FLAG_TBE:
        reg1 = reg1 & SPI_STAT_TBE;
        reg2 = reg2 & SPI_CTL1_TBEIE;
        break;
    /* I2S receive buffer not empty interrupt */
    case I2S_INT_FLAG_RBNE:
        reg1 = reg1 & SPI_STAT_RBNE;
        reg2 = reg2 & SPI_CTL1_RBNEIE;
        break;
    /* I2S overrun interrupt */
    case I2S_INT_FLAG_RXORERR:
        reg1 = reg1 & SPI_STAT_RXORERR;
        reg2 = reg2 & SPI_CTL1_ERRIE;
        break;
    /* I2S underrun error interrupt */
    case I2S_INT_FLAG_TXURERR:
        reg1 = reg1 & SPI_STAT_TXURERR;
        reg2 = reg2 & SPI_CTL1_ERRIE;
        break;
    /* I2S format error interrupt */
    case I2S_INT_FLAG_FERR:
        reg1 = reg1 & SPI_STAT_FERR;
        reg2 = reg2 & SPI_CTL1_ERRIE;
        break;
    default :
        break;
    }
    /*get I2S interrupt flag status */
    if((0U != reg1) && (0U != reg2)) {
        return SET;
    } else {
        return RESET;
    }
}
#endif /* GD32F350 and GD32F310 */
