/*!
    \file    gd32f3x0_hal_spi.c
    \brief   SPI driver

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

#define SPI_INIT_CTL0_MASK                   ((uint32_t)0x00001040U)  /*!< SPI_CTL0 parameter initialization mask */
#define SPI_INIT_CTL1_MASK                   ((uint32_t)0x000000E3U)  /*!< SPI_CTL1 parameter initialization mask */
#define SPI_INIT_QCTL_MASK                   ((uint32_t)0x00000000U)  /*!< SPI_QCTL parameter initialization mask */

#define SPI_TIMEOUT_VALUE  200

/* SPI private function */
static void _spi_deinit(uint32_t periph);

static void _spi_transmit_interrupt(void *spi);
static void _spi_receive_interrupt(void *spi);
static void _spi_2lines_receive_interrupt(void *spi);
static void _spi_2lines_transmit_interrupt(void *spi);

static void _spi_transmit_compelete_dma(void *dma);
static void _spi_receive_compelete_dma(void *dma);
static void _spi_transmit_receive_compelete_dma(void *dma);
static void _spi_dma_error(void *dma);

static void _spi_stop_receive_interrupt(void *spi);
static void _spi_stop_transmit_interrupt(void *hspi);
static void _spi_close_receive_interrupt(hal_spi_dev_struct *spi);
static void _spi_close_transmit_interrupt(hal_spi_dev_struct *spi);
static void _spi_close_transmit_receive_interrupt(hal_spi_dev_struct *spi);

static int32_t _spi_end_transmit_receive(hal_spi_dev_struct *spi, uint32_t timeout_ms);
static int32_t _spi_end_receive(hal_spi_dev_struct *spi, uint32_t timeout_ms);

static void _spi_clear_error_flag(uint32_t periph, uint32_t error_flag);

static FlagStatus _spi_flag_get(uint32_t periph, uint32_t flag);
static FlagStatus _spi_interrupt_flag_get(uint32_t periph, uint8_t interrupt);

/*!
    \brief      deinitialize SPI
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_spi_deinit(hal_spi_dev_struct *spi)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == spi) {
        HAL_DEBUGE("parameter [*spi] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if((SPI0 == spi->periph) || (SPI1 == spi->periph)) {
        spi->state = HAL_SPI_STATE_BUSY;
        /* reset SPI */
        _spi_deinit(spi->periph);
        hal_spi_struct_init(HAL_SPI_DEV_STRUCT, spi);
        spi->state = HAL_SPI_STATE_READY;
    } else {
        HAL_DEBUGE("parameter [spi->periph] value is invalid");
    }
}

/*!
    \brief      initialize the SPI structure
    \param[in]  struct_type: refer to hal_spi_struct_type_enum
    \param[in]  p_struct: point to SPI structure that contains the configuration information
    \param[out] none
    \retval     none
*/
void hal_spi_struct_init(hal_spi_struct_type_enum struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct) {
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(struct_type) {
    case HAL_SPI_INIT_STRUCT:
        /* initialize SPI initialization structure with the default values */
        ((hal_spi_init_struct *)p_struct)->device_mode          = SPI_MASTER ;
        ((hal_spi_init_struct *)p_struct)->trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
        ((hal_spi_init_struct *)p_struct)->frame_size           = SPI_FRAMESIZE_8BIT;
        ((hal_spi_init_struct *)p_struct)->nss                  = SPI_NSS_SOFT;
        ((hal_spi_init_struct *)p_struct)->clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
        ((hal_spi_init_struct *)p_struct)->prescaler            = SPI_PSC_16;
        ((hal_spi_init_struct *)p_struct)->crc_calculation      = SPI_CRC_DISABLE;
        ((hal_spi_init_struct *)p_struct)->crc_poly             = 0x07U;
        ((hal_spi_init_struct *)p_struct)->endian               = SPI_ENDIAN_MSB;
        ((hal_spi_init_struct *)p_struct)->nssp_mode            = SPI_NSSP_DISABLE;
        ((hal_spi_init_struct *)p_struct)->ti_mode              = SPI_TIMODE_DISABLE;
        break;

    case HAL_SPI_DEV_STRUCT:
        /* initialize SPI device information structure with the default values */
        ((hal_spi_dev_struct *)p_struct)->periph                   = 0U;
        ((hal_spi_dev_struct *)p_struct)->spi_irq.error_handler    = NULL;
        ((hal_spi_dev_struct *)p_struct)->spi_irq.receive_handler  = NULL;
        ((hal_spi_dev_struct *)p_struct)->spi_irq.transmit_handler = NULL;
        ((hal_spi_dev_struct *)p_struct)->p_dma_rx                 = NULL;
        ((hal_spi_dev_struct *)p_struct)->p_dma_tx                 = NULL;
        ((hal_spi_dev_struct *)p_struct)->txbuffer.buffer          = NULL;
        ((hal_spi_dev_struct *)p_struct)->txbuffer.length          = 0U;
        ((hal_spi_dev_struct *)p_struct)->txbuffer.pos             = 0U;
        ((hal_spi_dev_struct *)p_struct)->rxbuffer.buffer          = NULL;
        ((hal_spi_dev_struct *)p_struct)->rxbuffer.length          = 0U;
        ((hal_spi_dev_struct *)p_struct)->rxbuffer.pos             = 0U;
        ((hal_spi_dev_struct *)p_struct)->rx_callback              = NULL;
        ((hal_spi_dev_struct *)p_struct)->tx_callback              = NULL;
        ((hal_spi_dev_struct *)p_struct)->tx_rx_callback           = NULL;
        ((hal_spi_dev_struct *)p_struct)->error_callback           = NULL;
        ((hal_spi_dev_struct *)p_struct)->state                    = HAL_SPI_STATE_READY;
        ((hal_spi_dev_struct *)p_struct)->error_code               = HAL_SPI_ERROR_NONE;
        ((hal_spi_dev_struct *)p_struct)->mutex                    = HAL_MUTEX_UNLOCKED;
        break;

    default:
        HAL_DEBUGW("parameter [struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      initialize SPI
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  periph: SPI peripheral address
    \param[in]  p_init: the initialization data needed to initialize SPI
                  device_mode: SPI_MASTER, SPI_SLAVE
                  trans_mode: the argument could be selected from enumeration <hal_hal_spi_trans_mode_enum>
                  frame_size: SPI_FRAMESIZE_16BIT, SPI_FRAMESIZE_8BIT
                  nss: SPI_NSS_SOFT, SPI_NSS_HARD
                  endian: SPI_ENDIAN_MSB, SPI_ENDIAN_LSB
                  clock_polarity_phase: SPI_CK_PL_LOW_PH_1EDGE, SPI_CK_PL_HIGH_PH_1EDGE, SPI_CK_PL_LOW_PH_2EDGE, SPI_CK_PL_HIGH_PH_2EDGE
                  prescaler: the argument could be selected from enumeration <hal_hal_spi_prescaler_enum>
                  crc_calculation: SPI_CRC_DISABLE, SPI_CRC_ENABLE
                  crc_poly: 0x00 ~ 0xFF
                  nssp_mode: SPI_NSSP_ENABLE, SPI_NSSP_DISABLE
                  ti_mode: SPI_TIMODE_DISABLE, SPI_TIMODE_ENABLE
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_spi_init(hal_spi_dev_struct *spi, uint32_t periph, hal_spi_init_struct *p_init)
{
    uint32_t reg0 = 0U;
    uint32_t reg1 = 0U;
    uint32_t reg3 = 0U;

#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi) || (NULL == p_init)) {
        return HAL_ERR_ADDRESS;
    }
    if((SPI0 == periph) && ((SPI_TRANSMODE_QUADRECEIVE == p_init->trans_mode) ||
                            (SPI_TRANSMODE_QUADTRANSMIT == p_init->trans_mode))) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    spi->periph = periph;

    spi->state = HAL_SPI_STATE_BUSY;
    __HAL_SPI_DISABLE(periph);

    /* SPI init */
    reg0 = SPI_CTL0(periph);
    reg0 &= SPI_INIT_CTL0_MASK;

    if((SPI1 == periph) && ((SPI_TRANSMODE_QUADRECEIVE == p_init->trans_mode) ||
                            (SPI_TRANSMODE_QUADTRANSMIT == p_init->trans_mode))) {
        /* SPI1 quad mode */
        reg3 = SPI_QCTL(periph);
        reg3 &= SPI_INIT_QCTL_MASK;
        reg3 |= p_init->trans_mode;
        /* write to SPI_QCTL register */
        SPI_QCTL(periph) = (uint32_t)reg3;

        p_init->endian = SPI_ENDIAN_MSB;
        p_init->crc_calculation = SPI_CRC_DISABLE;
        p_init->frame_size = SPI_FRAMESIZE_8BIT;
    } else {
        reg0 |= p_init->trans_mode;
    }
    reg0 |= (p_init->frame_size | p_init->device_mode | p_init->crc_calculation | p_init->nss | p_init->endian |
             p_init->clock_polarity_phase | p_init->prescaler);
    /* write to SPI_CTL0 register */
    SPI_CTL0(periph) = (uint32_t)reg0;

    reg1 = SPI_CTL1(periph);
    reg1 &= SPI_INIT_CTL1_MASK;
    reg1 |= p_init->ti_mode | p_init->nssp_mode;
    if((SPI_MASTER ==  p_init->device_mode) && (SPI_NSS_HARD == p_init->nss)) {
        reg1 |= SPI_CTL1_NSSDRV;
    }
    /* write to SPI_CTL1 register */
    SPI_CTL1(periph) = (uint32_t)reg1;
    if(SPI_CRC_ENABLE == p_init->crc_calculation) {
        SPI_CRCPOLY(periph) = (uint32_t)(p_init->crc_poly);
    }

    /* select SPI mode */
    SPI_I2SCTL(periph) &= (uint32_t)(~SPI_I2SCTL_I2SSEL);

    spi->state = HAL_SPI_STATE_READY;
    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data, poll transmit process and completed status, the function is blocking
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_txbuffer: pointer to txbuffer
    \param[in]  length: length of data to be sent
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, HAL_ERR_BUSY,HAL_ERR_TIMEOUT details refer to gd32f3x0_hal.h
*/
int32_t hal_spi_transmit_poll(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint32_t length, uint32_t timeout_ms)
{
    uint32_t tick_start;

#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi) || (NULL == p_txbuffer) || (0 == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(spi->state != HAL_SPI_STATE_READY) {
        return HAL_ERR_BUSY;
    }

    /* lock SPI */
    HAL_LOCK(spi);

    /* set the transaction information */
    spi->state = HAL_SPI_STATE_BUSY_TX;
    spi->error_code = HAL_ERR_NONE;
    spi->txbuffer.buffer  = (uint8_t *)p_txbuffer;
    spi->txbuffer.length  = length;
    spi->txbuffer.pos     = length;

    /* init field not used to zero */
    spi->rxbuffer.buffer  = (uint8_t *)NULL;
    spi->rxbuffer.length  = 0U;
    spi->rxbuffer.pos     = 0U;
    spi->tx_callback      = NULL;
    spi->rx_callback      = NULL;

    /* 1 line transmit */
    if((SPI_CTL0(spi->periph)) & (SPI_CTL0_BDEN)) {
        SPI_CTL0(spi->periph) |= SPI_CTL0_BDOEN;
    }

    if(__HAL_SPI_GET_CRC_USED(spi->periph) == SPI_CRC_ENABLE) {
        /* reset CRC */
        __HAL_SPI_CRC_OFF(spi->periph);
        __HAL_SPI_CRC_ON(spi->periph);
    }
    __HAL_SPI_ENABLE(spi->periph);

    if(__HAL_SPI_GET_FRAME_SIZE(spi->periph) == SPI_FRAMESIZE_16BIT) {
        if((SPI_SLAVE == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) || (0x01U == spi->txbuffer.pos)) {
            SPI_DATA(spi->periph) = *((uint16_t *)p_txbuffer);
            p_txbuffer += sizeof(uint16_t);
            spi->txbuffer.pos--;
        }

        /* transmit 16 bit data*/
        while(spi->txbuffer.pos > 0U) {
            /* wait TBE set */
            tick_start = hal_sys_basetick_count_get();
            while(RESET == _spi_flag_get(spi->periph, SPI_FLAG_TBE)) {
                if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                    if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }
            /* transmit data */
            SPI_DATA(spi->periph) = *((uint16_t *)p_txbuffer);
            p_txbuffer += sizeof(uint16_t);
            spi->txbuffer.pos--;
        }

    } else { /* transmit 8 bit data */
        if((SPI_SLAVE == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) ||
                (0x01U == spi->txbuffer.pos)) {
            SPI_DATA(spi->periph) = (*p_txbuffer);
            p_txbuffer += sizeof(uint8_t);
            spi->txbuffer.pos--;
        }
        while(spi->txbuffer.pos > 0U) {
            /* wait until TBE is set */
            tick_start = hal_sys_basetick_count_get();
            while(RESET == _spi_flag_get(spi->periph, SPI_FLAG_TBE)) {
                if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                    if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }

            *(__IO uint8_t *)&SPI_DATA(spi->periph) = (*p_txbuffer++);
            spi->txbuffer.pos--;
        }
    }

    if(__HAL_SPI_GET_CRC_USED(spi->periph) == SPI_CRC_ENABLE) {
        /* next data is crc */
        __HAL_SPI_CRC_NEXT(spi->periph);
    }

    /* check the end of the transaction */
    if(HAL_ERR_NONE != _spi_end_transmit_receive(spi, SPI_TIMEOUT_VALUE)) {
        return HAL_ERR_TIMEOUT ;
    }
    spi->state = HAL_SPI_STATE_READY;

    /* unlock SPI */
    HAL_UNLOCK(spi);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data, poll receive process and completed status, the function is blocking
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_rxbuffer: pointer to rxbuffer
    \param[in]  length: length of data to be sent
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_spi_receive_poll(hal_spi_dev_struct *spi, uint8_t *p_rxbuffer, uint32_t length, uint32_t timeout_ms)
{
    __IO uint16_t tmp_crc = 0U;
    uint32_t tick_start = 0;

#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi) || (NULL == p_rxbuffer) || (0 == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if((SPI_MASTER == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) &&
            (SPI_TRANSMODE_FULLDUPLEX == __HAL_SPI_GET_TRANS_MODE(spi->periph))) {
        spi->state = HAL_SPI_STATE_READY;
        /* call transmit-receive function to send dummy data generate clock */
        return hal_spi_transmit_receive_poll(spi, p_rxbuffer, p_rxbuffer, length, timeout_ms);
    }

    if(HAL_SPI_STATE_READY != spi->state) {
        return HAL_ERR_BUSY;
    }

    /* lock SPI */
    HAL_LOCK(spi);

    spi->state            = HAL_SPI_STATE_BUSY_RX;
    spi->error_code       = HAL_ERR_NONE;
    /* init field not used to zero */
    spi->txbuffer.buffer  = (uint8_t *)NULL;
    spi->txbuffer.length  = 0U;
    spi->txbuffer.pos     = 0U;
    spi->tx_callback      = NULL;
    spi->rx_callback      = NULL;
    spi->tx_rx_callback   = NULL;
    spi->error_callback   = NULL;
    /* set the receiver information */
    spi->rxbuffer.buffer  = (uint8_t *)p_rxbuffer;
    spi->rxbuffer.length  = length;
    spi->rxbuffer.pos     = length;

    /* 1 line receive */
    if((SPI_CTL0(spi->periph)) & (SPI_CTL0_BDEN)) {
        SPI_CTL0(spi->periph) &= ~SPI_CTL0_BDOEN;
    }

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)) {
        /* reset CRC */
        __HAL_SPI_CRC_OFF(spi->periph);
        __HAL_SPI_CRC_ON(spi->periph);
        /* set crc next bit before the latest data */
        spi->rxbuffer.pos--;
    }

    __HAL_SPI_ENABLE(spi->periph);

    /* receive 8 bit data */
    if(__HAL_SPI_GET_FRAME_SIZE(spi->periph) == SPI_FRAMESIZE_8BIT) {
        while(spi->rxbuffer.pos > 0U) {
            /* wait uintl RBNE is set */
            tick_start = hal_sys_basetick_count_get();
            while(RESET == _spi_flag_get(spi->periph, SPI_FLAG_RBNE)) {
                if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                    if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }
            (*(uint8_t *)p_rxbuffer++) = SPI_DATA(spi->periph);
            spi->rxbuffer.pos--;

        }
    } else { /* receive 16 bit data */
        while(spi->rxbuffer.pos > 0U) {
            /* wait uintl RBNE is set */
            tick_start = hal_sys_basetick_count_get();
            while(RESET == _spi_flag_get(spi->periph, SPI_FLAG_RBNE)) {
                if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                    if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }

            /* receive data */
            (*(uint16_t *)p_rxbuffer) = SPI_DATA(spi->periph);
            p_rxbuffer += sizeof(uint16_t);
            spi->rxbuffer.pos--;
        }
    }

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)) {
        __HAL_SPI_CRC_NEXT(spi->periph);
        /* wait until RBNE is set */
        tick_start = hal_sys_basetick_count_get();
        while(RESET == _spi_flag_get(spi->periph, SPI_FLAG_RBNE)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* receive last data */
        if(SPI0 == spi->periph) {
            if(__HAL_SPI_GET_FRAME_SIZE(spi->periph) == SPI_FRAMESIZE_16BIT) {
                *((uint16_t *)p_rxbuffer) = SPI_DATA(spi->periph);
            } else {
                (*(uint8_t *)p_rxbuffer) = *(__IO uint8_t *)&SPI_DATA(spi->periph);
            }
        }

        /* wait until RBNE is set */
        tick_start = hal_sys_basetick_count_get();
        while(RESET == _spi_flag_get(spi->periph, SPI_FLAG_RBNE)) {
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        tmp_crc = SPI_DATA(spi->periph);
        if(SET == _spi_flag_get(spi->periph, SPI_FLAG_CRCERR)) {
            _spi_clear_error_flag(spi->periph, SPI_ERROR_FLAG_CRCERR);
            spi->state = HAL_SPI_STATE_READY;
            return HAL_ERR_HARDWARE;
        }
    }
    /* check the end of the transaction */
    if(HAL_ERR_NONE != _spi_end_transmit_receive(spi, SPI_TIMEOUT_VALUE)) {
        spi->state = HAL_SPI_STATE_READY;
        return HAL_ERR_TIMEOUT;
    }

    spi->state = HAL_SPI_STATE_READY;

    /* unlock SPI */
    HAL_UNLOCK(spi);

    return HAL_ERR_NONE;

}

/*!
    \brief      transmit and receive amounts of data, poll receive process and completed status, the function is blocking
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_txbuffer: pointer to txbuffer
    \param[in]  p_rxbuffer: pointer to rxbuffer
    \param[in]  length: length of data to be sent
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_spi_transmit_receive_poll(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint8_t *p_rxbuffer,
                                      uint32_t length, uint32_t timeout_ms)
{
    /* alternate Rx and Tx during transfer */
    uint32_t tx_flag = 1U;
    __IO uint16_t tmp_crc = 0U;
    uint32_t tick_start = 0;

#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi) || (NULL == p_rxbuffer) || (NULL == p_txbuffer) || (0 == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(HAL_SPI_STATE_READY != spi->state) {
        return HAL_ERR_BUSY;
    }

    /* lock SPI */
    HAL_LOCK(spi);

    /* set transmit information */
    spi->state            = HAL_SPI_STATE_BUSY_TX_RX;
    spi->error_code       = HAL_ERR_NONE;
    spi->txbuffer.buffer  = (uint8_t *)p_txbuffer;
    spi->txbuffer.length  = length;
    spi->txbuffer.pos     = length;

    /* set receive information*/
    spi->rxbuffer.buffer  = (uint8_t *)p_rxbuffer;
    spi->rxbuffer.length  = length;
    spi->rxbuffer.pos     = length;

    spi->tx_callback      = NULL;
    spi->rx_callback      = NULL;

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)) {
        /* reset CRC */
        __HAL_SPI_CRC_OFF(spi->periph);
        __HAL_SPI_CRC_ON(spi->periph);
    }

    __HAL_SPI_ENABLE(spi->periph);

    /* transmit and receive 16 bit data */
    if(__HAL_SPI_GET_FRAME_SIZE(spi->periph) == SPI_FRAMESIZE_16BIT) {
        if((SPI_SLAVE == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) ||
                (0x01U == spi->txbuffer.pos)) {
            SPI_DATA(spi->periph) = *((uint16_t *)p_txbuffer);
            p_txbuffer += sizeof(uint16_t);
            spi->txbuffer.pos--;
        }

        while(spi->txbuffer.pos > 0U || spi->rxbuffer.pos > 0U) {
            if(tx_flag && (spi->txbuffer.pos > 0U)) {
                tick_start = hal_sys_basetick_count_get();
                while(RESET == _spi_flag_get(spi->periph, SPI_FLAG_TBE)) {
                    if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                        if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                            return HAL_ERR_TIMEOUT;
                        }
                    }
                }

                SPI_DATA(spi->periph) = *((uint16_t *)p_txbuffer);
                p_txbuffer += sizeof(uint16_t);
                spi->txbuffer.pos--;
                tx_flag = 0U;

                /* enable CRC transmission */
                if((0U == spi->txbuffer.pos) &&
                        (SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph))) {
                    __HAL_SPI_CRC_NEXT(spi->periph);
                }
            }
            /* check RBNE flag */
            if((spi->rxbuffer.pos > 0U)) {
                tick_start = hal_sys_basetick_count_get();
                while(RESET == _spi_flag_get(spi->periph, SPI_FLAG_RBNE)) {
                    if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                        if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                            return HAL_ERR_TIMEOUT;
                        }
                    }
                }
                *((uint16_t *)p_rxbuffer) = SPI_DATA(spi->periph);
                p_rxbuffer += sizeof(uint16_t);
                spi->rxbuffer.pos--;
                /* next data is a transmission */
                tx_flag = 1U;
            }
        }
    } else { /* transmit and receive 8 bit data */
        if((SPI_SLAVE == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) ||
                (0x01U == spi->txbuffer.pos)) {
            SPI_DATA(spi->periph) = (*p_txbuffer);
            p_txbuffer += sizeof(uint8_t);
            spi->txbuffer.pos--;
        }
        while(spi->txbuffer.pos > 0U || spi->rxbuffer.pos > 0U) {
            if(tx_flag && (spi->txbuffer.pos > 0U)) {
                tick_start = hal_sys_basetick_count_get();
                /* wait until TBE is set */
                while(RESET == _spi_flag_get(spi->periph, SPI_FLAG_TBE)) {
                    if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                        if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                            return HAL_ERR_TIMEOUT;
                        }
                    }
                }

                *(__IO uint8_t *)&SPI_DATA(spi->periph) = (*p_txbuffer++);
                spi->txbuffer.pos--;
                tx_flag = 0U;

            }
            if((spi->txbuffer.pos == 0U) &&
                    (SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph))) {
                __HAL_SPI_CRC_NEXT(spi->periph);
            }
            if(spi->rxbuffer.pos > 0U) {
                tick_start = hal_sys_basetick_count_get();
                while(RESET == _spi_flag_get(spi->periph, SPI_FLAG_RBNE)) {
                    if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                        if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                            return HAL_ERR_TIMEOUT;
                        }
                    }
                }

                (*(uint8_t *)p_rxbuffer++) = SPI_DATA(spi->periph);
                spi->rxbuffer.pos--;
                tx_flag = 1U;
            }
        }
    }

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)) {
        while(RESET == _spi_flag_get(spi->periph, SPI_FLAG_RBNE)) {
            tick_start = hal_sys_basetick_count_get();
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    spi->state = HAL_SPI_STATE_READY;
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
        tmp_crc = SPI_DATA(spi->periph);
        if(SET == _spi_flag_get(spi->periph, SPI_FLAG_CRCERR)) {
            _spi_clear_error_flag(spi->periph, SPI_ERROR_FLAG_CRCERR);
            spi->state = HAL_SPI_STATE_READY;
            return HAL_ERR_HARDWARE;
        }
    }

    /* check the end of the transaction */
    if(HAL_ERR_NONE != _spi_end_transmit_receive(spi, SPI_TIMEOUT_VALUE)) {
        spi->state = HAL_SPI_STATE_READY;
        return HAL_ERR_TIMEOUT ;
    }
    spi->state = HAL_SPI_STATE_READY;

    /* unlock SPI */
    HAL_UNLOCK(spi);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by interrupt method, the function is non-blocking
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_txbuffer: pointer to txbuffer
    \param[in]  length: length of data to be sent
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_spi_transmit_interrupt(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint32_t length,
                                   hal_spi_user_callback_struct *p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi) || (NULL == p_txbuffer) || (0 == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(HAL_SPI_STATE_READY != spi->state) {
        return HAL_ERR_BUSY;
    }

    /* lock SPI */
    HAL_LOCK(spi);

    /* set the transmit information */
    spi->state = HAL_SPI_STATE_BUSY_TX;
    spi->error_code = HAL_SPI_ERROR_NONE;
    spi->txbuffer.buffer = (uint8_t *)p_txbuffer;
    spi->txbuffer.length = length;
    spi->txbuffer.pos = length;

    /* init field not used to zero */
    spi->rxbuffer.buffer  = (uint8_t *)NULL;
    spi->rxbuffer.length  = 0U;
    spi->rxbuffer.pos = 0U;

    spi->rx_callback = NULL;
    spi->tx_rx_callback = NULL;
    spi->tx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;

    spi->spi_irq.transmit_handler = _spi_transmit_interrupt;

    if((SPI_CTL0(spi->periph)) & (SPI_CTL0_BDEN)) {
        SPI_CTL0(spi->periph) |= SPI_CTL0_BDOEN;
    }
    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)) {
        __HAL_SPI_CRC_OFF(spi->periph);
        __HAL_SPI_CRC_ON(spi->periph);
    }

    /* enable TBE and ERR interrupt */
    __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_TBE);
    __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_ERR);

    __HAL_SPI_ENABLE(spi->periph);

    /* unlock SPI */
    HAL_UNLOCK(spi);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by interrupt method, the function is non-blocking
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_rxbuffer: pointer to rxbuffer
    \param[in]  length: length of data to be sent
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_spi_receive_interrupt(hal_spi_dev_struct *spi, uint8_t *p_rxbuffer, uint32_t length,
                                  hal_spi_user_callback_struct *p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi) || (NULL == p_rxbuffer) || (0 == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if((SPI_MASTER == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) &&
            (SPI_TRANSMODE_FULLDUPLEX == __HAL_SPI_GET_TRANS_MODE(spi->periph))) {
        spi->state = HAL_SPI_STATE_READY;
        /* call transmit-receive function to send dummy data generate clock */
        return hal_spi_transmit_receive_interrupt(spi, p_rxbuffer, p_rxbuffer, length,
                p_user_func);
    }

    if(HAL_SPI_STATE_READY != spi->state) {
        return HAL_ERR_BUSY;
    }

    /* lock SPI */
    HAL_LOCK(spi);

    spi->state = HAL_SPI_STATE_BUSY_RX;
    /* init field not used to zero */
    spi->txbuffer.buffer = (uint8_t *)NULL;
    spi->txbuffer.length = 0;
    spi->txbuffer.pos = 0;

    /* set the receiption information */
    spi->rxbuffer.buffer = (uint8_t *)p_rxbuffer;
    spi->rxbuffer.length = length;
    spi->rxbuffer.pos = length;

    spi->tx_callback = NULL;
    spi->rx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;

    spi->spi_irq.receive_handler = _spi_receive_interrupt;

    if((SPI_CTL0(spi->periph)) & (SPI_CTL0_BDEN)) {
        SPI_CTL0(spi->periph) &= ~SPI_CTL0_BDOEN;
    }

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)) {
        __HAL_SPI_CRC_OFF(spi->periph);
        __HAL_SPI_CRC_ON(spi->periph);
    }

    /* enable RBNE and ERR interrupt */
    __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_RBNE);
    __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_ERR);

    __HAL_SPI_ENABLE(spi->periph);

    /* unlock SPI */
    HAL_UNLOCK(spi);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit and receive amounts of data by interrupt method, the function is non-blocking
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_txbuffer: pointer to txbuffer
    \param[in]  p_rxbuffer: pointer to rxbuffer
    \param[in]  length: length of data to be sent
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32f3x0_hal.h
*/
int32_t hal_spi_transmit_receive_interrupt(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint8_t *p_rxbuffer,
        uint32_t length, hal_spi_user_callback_struct *p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi) || (NULL == p_rxbuffer) || (NULL == p_txbuffer) ||
            (0 == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(spi->state != HAL_SPI_STATE_READY) {
        return HAL_ERR_BUSY;
    }

    /* lock SPI */
    HAL_LOCK(spi);

    __HAL_SPI_DISABLE(spi->periph);
    /* set transmit information */
    spi->state       = HAL_SPI_STATE_BUSY_TX_RX;
    spi->txbuffer.buffer  = (uint8_t *)p_txbuffer;
    spi->txbuffer.length  = length;
    spi->txbuffer.pos = length;

    /* set receiption information */
    spi->rxbuffer.buffer  = (uint8_t *)p_rxbuffer;
    spi->rxbuffer.length  = length;
    spi->rxbuffer.pos     = length;

    spi->tx_callback = NULL;
    spi->rx_callback = NULL;
    spi->tx_rx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;

    spi->spi_irq.receive_handler = _spi_2lines_receive_interrupt;
    spi->spi_irq.transmit_handler = _spi_2lines_transmit_interrupt;

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)) {
        __HAL_SPI_CRC_OFF(spi->periph);
        __HAL_SPI_CRC_ON(spi->periph);
    }

    /* enable TBE, RBNE and ERR interrupt */
    __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_TBE);
    __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_RBNE);
    __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_ERR);

    __HAL_SPI_ENABLE(spi->periph);

    /* unlock SPI */
    HAL_UNLOCK(spi);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by DMA method, the function is non-blocking
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_txbuffer: pointer to txbuffer
    \param[in]  length: length of data to be sent
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_BUSY, HAL_ERR_VAL details refer to gd32f3x0_hal.h
*/
int32_t hal_spi_transmit_dma(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint16_t length,
                             hal_spi_user_callback_struct *p_user_func)
{
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi) || (NULL == p_txbuffer) || (0 == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(spi->state != HAL_SPI_STATE_READY) {
        return HAL_ERR_BUSY;
    }

    /* lock SPI */
    HAL_LOCK(spi);

    /* set the transaction information */
    spi->state = HAL_SPI_STATE_BUSY_TX;
    spi->txbuffer.buffer = (uint8_t *)p_txbuffer;
    spi->txbuffer.length = length;
    spi->txbuffer.pos = length;

    /* Init field not used to zero */
    spi->rxbuffer.buffer = (uint8_t *)NULL;
    spi->rxbuffer.length = 0U;
    spi->rxbuffer.pos = 0U;
    spi->rx_callback = NULL;
    spi->tx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;

    /* 1 lines transmit */
    if((SPI_CTL0(spi->periph)) & (SPI_CTL0_BDEN)) {
        SPI_CTL0(spi->periph) |= SPI_CTL0_BDOEN;
    }
    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)) {
        /* reset CRC */
        __HAL_SPI_CRC_OFF(spi->periph);
        __HAL_SPI_CRC_ON(spi->periph);
    }
    spi->p_dma_tx->dma_irq.half_finish_handle = NULL;
    spi->p_dma_tx->dma_irq.full_finish_handle = _spi_transmit_compelete_dma;
    spi->p_dma_tx->dma_irq.error_handle = _spi_dma_error;

    dma_irq.half_finish_handle = NULL;
    dma_irq.full_finish_handle = _spi_transmit_compelete_dma;
    dma_irq.error_handle = _spi_dma_error;

    hal_dma_start_interrupt(spi->p_dma_tx, (uint32_t)spi->txbuffer.buffer,
                            (uint32_t)&SPI_DATA(spi->periph), spi->txbuffer.pos, &dma_irq);
    __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_ERR);
    __HAL_SPI_ENABLE(spi->periph);

    /* enable SPI DMA transmit */
    __HAL_SPI_DMA_ENABLE(spi->periph, SPI_DMA_TRANSMIT);

    /* unlock SPI */
    HAL_UNLOCK(spi);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by DMA method, the function is non-blocking
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_rxbuffer: pointer to rxbuffer
    \param[in]  length: length of data to be sent
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_spi_receive_dma(hal_spi_dev_struct *spi, uint8_t *p_rxbuffer, uint16_t length,
                            hal_spi_user_callback_struct *p_user_func)
{
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi) || (NULL == p_rxbuffer) || (0 == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if((SPI_MASTER == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) &&
            (SPI_TRANSMODE_FULLDUPLEX == __HAL_SPI_GET_TRANS_MODE(spi->periph))) {
        spi->state = HAL_SPI_STATE_READY;
        /* call transmit-receive function to send dummy data generate clock on CLK line */
        return hal_spi_transmit_receive_dma(spi, p_rxbuffer, p_rxbuffer, length,
                                            p_user_func);
    }

    if(spi->state != HAL_SPI_STATE_READY) {
        return HAL_ERR_BUSY;
    }

    /* lock SPI */
    HAL_LOCK(spi);

    spi->state = HAL_SPI_STATE_BUSY_RX;
    spi->error_code = HAL_ERR_NONE;
    spi->txbuffer.buffer  = (uint8_t *)NULL;
    spi->txbuffer.length  = 0;
    spi->txbuffer.pos = 0;

    /* set the receive information */
    spi->rxbuffer.buffer  = (uint8_t *)p_rxbuffer;
    spi->rxbuffer.length  = length;
    spi->rxbuffer.pos     = length;

    spi->rx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;
    spi->tx_callback = NULL;

    /* 1 line receive */
    if((SPI_CTL0(spi->periph)) & (SPI_CTL0_BDEN)) {
        SPI_CTL0(spi->periph) &= ~SPI_CTL0_BDOEN;
    }
    /* DMA receive complete callback */
    spi->p_dma_rx->dma_irq.half_finish_handle = NULL;
    spi->p_dma_rx->dma_irq.full_finish_handle = _spi_receive_compelete_dma;
    spi->p_dma_rx->dma_irq.error_handle = _spi_dma_error;

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)) {
        /* reset CRC */
        __HAL_SPI_CRC_OFF(spi->periph);
        __HAL_SPI_CRC_ON(spi->periph);
    }

    spi->tx_callback = NULL;
    spi->tx_rx_callback = NULL;
    spi->rx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;

    /* DMA receive complete callback */
    dma_irq.half_finish_handle = NULL;
    dma_irq.full_finish_handle = _spi_receive_compelete_dma;
    dma_irq.error_handle = _spi_dma_error;

    hal_dma_start_interrupt(spi->p_dma_rx, (uint32_t)&SPI_DATA(spi->periph), (uint32_t)spi->rxbuffer.buffer,
                            spi->rxbuffer.pos, &dma_irq);

    __HAL_SPI_ENABLE(spi->periph);

    __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_ERR);

    /* enable SPI DMA receive */
    __HAL_SPI_DMA_ENABLE(spi->periph, SPI_DMA_RECEIVE);

    /* unlock SPI */
    HAL_UNLOCK(spi);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit and receive amounts of data by DMA method, the function is non-blocking
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_txbuffer: pointer to txbuffer
    \param[in]  p_rxbuffer: pointer to rxbuffer
    \param[in]  length: length of data to be sent
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_spi_transmit_receive_dma(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint8_t *p_rxbuffer, uint16_t length,
                                     hal_spi_user_callback_struct *p_user_func)
{
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi) || (NULL == p_rxbuffer) || (NULL == p_txbuffer) ||
            (0 == length)) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(HAL_SPI_STATE_READY != spi->state) {
        return HAL_ERR_BUSY;
    }

    /* lock SPI */
    HAL_LOCK(spi);

    /* set the transaction information */
    spi->state = HAL_SPI_STATE_BUSY_TX_RX;
    spi->txbuffer.buffer  = (uint8_t *)p_txbuffer;
    spi->txbuffer.length  = length;
    spi->txbuffer.pos = length;

    /* set the reception information */
    spi->rxbuffer.buffer = (uint8_t *)p_rxbuffer;
    spi->rxbuffer.length = length;
    spi->rxbuffer.pos = length;

    spi->tx_rx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)) {
        /* reset CRC */
        __HAL_SPI_CRC_OFF(spi->periph);
        __HAL_SPI_CRC_ON(spi->periph);
    }

    spi->p_dma_tx->dma_irq.half_finish_handle = NULL;
    spi->p_dma_tx->dma_irq.full_finish_handle = NULL;
    spi->p_dma_tx->dma_irq.error_handle = NULL;

    spi->p_dma_rx->dma_irq.half_finish_handle = NULL;
    spi->p_dma_rx->dma_irq.full_finish_handle = _spi_transmit_receive_compelete_dma;
    spi->p_dma_rx->dma_irq.error_handle = _spi_dma_error;

    dma_irq.half_finish_handle = NULL;
    if(HAL_SPI_STATE_BUSY_RX == spi->state) {
        dma_irq.full_finish_handle = _spi_receive_compelete_dma;
    } else {
        dma_irq.full_finish_handle = _spi_transmit_receive_compelete_dma;
    }

    dma_irq.error_handle = _spi_dma_error;

    hal_dma_start_interrupt(spi->p_dma_rx, (uint32_t)&SPI_DATA(spi->periph), (uint32_t)spi->rxbuffer.buffer,
                            spi->rxbuffer.pos, &dma_irq);
    /* enable SPI DMA receive */
    __HAL_SPI_DMA_ENABLE(spi->periph, SPI_DMA_RECEIVE);

    hal_dma_start_interrupt(spi->p_dma_tx, (uint32_t)spi->txbuffer.buffer, (uint32_t)&SPI_DATA(spi->periph),
                            spi->txbuffer.pos, &dma_irq);

    __HAL_SPI_ENABLE(spi->periph);
    __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_ERR);
    /* enable SPI DMA transmit */
    __HAL_SPI_DMA_ENABLE(spi->periph, SPI_DMA_TRANSMIT);

    /* unlock SPI */
    HAL_UNLOCK(spi);

    return HAL_ERR_NONE;
}

/*!
    \brief      SPI interrupt handler content function, which is merely used in spi_handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_spi_irq(hal_spi_dev_struct *spi)
{
    __IO uint32_t tmp = 0x00U;

#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == spi) {
        HAL_DEBUGE("parameter [*spi] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if((_spi_interrupt_flag_get(spi->periph, SPI_INT_FLAG_TBE) != RESET)) {
        spi->spi_irq.transmit_handler(spi);
    }

    if(((SPI_STAT(spi->periph) & SPI_STAT_RXORERR) == RESET) &&
            (_spi_interrupt_flag_get(spi->periph, SPI_INT_FLAG_RBNE) != RESET)) {
        spi->spi_irq.receive_handler(spi);
    }

    if((((SPI_STAT(spi->periph) & (SPI_STAT_RXORERR | SPI_STAT_CONFERR | SPI_STAT_FERR))) != RESET) &&
            ((SPI_CTL1(spi->periph) & SPI_CTL1_ERRIE) != RESET)) {
        /* SPI overrun error interrupt occurred */
        if(_spi_interrupt_flag_get(spi->periph, SPI_INT_FLAG_RXORERR) != RESET) {
            _spi_clear_error_flag(spi->periph, SPI_ERROR_FLAG_RXORERR);
        }

        /* SPI mode error interrupt occurred */
        if(_spi_interrupt_flag_get(spi->periph, SPI_INT_FLAG_CONFERR) != RESET) {
            _spi_clear_error_flag(spi->periph, SPI_ERROR_FLAG_CONF);
        }

        /* SPI frame error interrupt occurred */
        if(_spi_interrupt_flag_get(spi->periph, SPI_INT_FLAG_FERR) != RESET) {
            _spi_clear_error_flag(spi->periph, SPI_ERROR_FLAG_FERR);
        }

        /* disable all interrupts */
        __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_TBE);
        __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_RBNE);
        __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_ERR);
        spi->state = HAL_SPI_STATE_READY;

        /* disable the SPI DMA requests if enabled */
        hal_dma_stop(spi->p_dma_tx);
        hal_dma_stop(spi->p_dma_rx);

    }
}

/*!
    \brief      set user-defined interrupt callback function
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to SPI interrupt callback function pointer structure
                  The structure member can be assigned as following parameters:
      \arg        hal_irq_handle_cb function pointer: the function is user-defined,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
      \arg        NULL: the corresponding callback mechanism is out of use, and
                     disable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_spi_irq_handle_set(hal_spi_dev_struct *spi, hal_spi_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi) || (NULL == p_irq)) {
        HAL_DEBUGE("parameter [*spi] or [*p_irq] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(NULL != p_irq->error_handler) {
        spi->spi_irq.error_handler = p_irq->error_handler;
        __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_ERR);
    } else {
        spi->spi_irq.error_handler = NULL;
        __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_ERR);
    }

    if(NULL != p_irq->receive_handler) {
        spi->spi_irq.receive_handler = p_irq->receive_handler;
        __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_RBNE);
    } else {
        spi->spi_irq.receive_handler = NULL;
        __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_RBNE);
    }

    if(NULL != p_irq->transmit_handler) {
        spi->spi_irq.transmit_handler = p_irq->transmit_handler;
        __HAL_SPI_INT_ENABLE(spi->periph, SPI_INT_TBE);
    } else {
        spi->spi_irq.transmit_handler = NULL;
        __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_TBE);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_spi_irq_handle_all_reset(hal_spi_dev_struct *spi)
{
    spi->spi_irq.error_handler = NULL;
    spi->spi_irq.transmit_handler = NULL;
    spi->spi_irq.receive_handler = NULL;
    spi->spi_irq.transmit_receive_handler = NULL;
}

/*!
    \brief      start SPI module function
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/void hal_spi_start(hal_spi_dev_struct *spi)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == spi) {
        HAL_DEBUGE("parameter [*spi] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    __HAL_SPI_ENABLE(spi->periph);
}
/*!
    \brief      stop SPI module function
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/void hal_spi_stop(hal_spi_dev_struct *spi)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == spi) {
        HAL_DEBUGE("parameter [*spi] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    __HAL_SPI_DISABLE(spi->periph);
}

/*!
    \brief      SPI abort function
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_spi_abort(hal_spi_dev_struct *spi)
{
    uint32_t tick_start = 0;
    int32_t errorcode;
    errorcode = HAL_ERR_NONE;
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi)) {
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(SPI_CTL1(spi->periph) & SPI_CTL1_TBEIE) {
        spi->spi_irq.transmit_handler = _spi_stop_transmit_interrupt;
        tick_start = hal_sys_basetick_count_get();
        while(spi->state != HAL_SPI_ERROR_NONE) {
            if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)) {
                    spi->error_code = HAL_SPI_ERROR_ABORT;
                    break;
                }
            }
        }
    }

    if(SPI_CTL1(spi->periph) & SPI_CTL1_RBNEIE) {
        spi->spi_irq.receive_handler = _spi_stop_receive_interrupt;
        tick_start = hal_sys_basetick_count_get();
        while(spi->state != HAL_SPI_ERROR_NONE) {
            if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)) {
                    spi->error_code = HAL_SPI_ERROR_ABORT;
                    break;
                }
            }
        }
    }

    __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_ERR);
    if((SPI_CTL1(spi->periph)&SPI_CTL1_DMATEN) ||
            (SPI_CTL1(spi->periph)&SPI_CTL1_DMAREN)) {
        if(spi->p_dma_tx != NULL) {
            hal_dma_stop(spi->p_dma_tx);

            spi->error_code = HAL_SPI_ERROR_NONE;
            __HAL_SPI_DMA_DISABLE(spi->periph, SPI_DMA_TRANSMIT);
            if(HAL_ERR_NONE != _spi_end_transmit_receive(spi, SPI_TIMEOUT_VALUE)) {
                spi->error_code = HAL_SPI_ERROR_ABORT;
            }
            __HAL_SPI_DISABLE(spi->periph);
        }
        if(spi->p_dma_rx != NULL) {
            hal_dma_stop(spi->p_dma_rx);

            spi->error_code = HAL_SPI_ERROR_ABORT;
            __HAL_SPI_DISABLE(spi->periph);

            tick_start = hal_sys_basetick_count_get();
            while(RESET != _spi_flag_get(spi->periph, SPI_FLAG_TRANS)) {
                if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE) {
                    if(SET == hal_sys_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)) {
                        spi->error_code = HAL_SPI_ERROR_ABORT;
                    }
                }
            }
            __HAL_SPI_DMA_DISABLE(spi->periph, SPI_DMA_RECEIVE);
        }
    }
    spi->rxbuffer.pos = 0U;
    spi->txbuffer.pos = 0U;

    if(HAL_SPI_ERROR_ABORT == spi->error_code) {
        errorcode = HAL_ERR_NONE ;
    } else {
        spi->error_code = HAL_SPI_ERROR_NONE;
    }

    /* clear the rx overrun error flag */
    _spi_clear_error_flag(spi->periph, SPI_ERROR_FLAG_RXORERR);

    spi->state = HAL_SPI_STATE_READY;

    return errorcode;
}

/*!
    \brief      SPI DMA pause function
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_spi_dma_pause(hal_spi_dev_struct *spi)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == spi) {
        HAL_DEBUGE("parameter [*spi] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    __HAL_SPI_DMA_DISABLE(spi->periph, SPI_DMA_TRANSMIT);
    __HAL_SPI_DMA_DISABLE(spi->periph, SPI_DMA_RECEIVE);
}

/*!
    \brief      SPI DMA resume function
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_spi_dma_resume(hal_spi_dev_struct *spi)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == spi) {
        HAL_DEBUGE("parameter [*spi] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    __HAL_SPI_DMA_ENABLE(spi->periph, SPI_DMA_TRANSMIT);
    __HAL_SPI_DMA_ENABLE(spi->periph, SPI_DMA_RECEIVE);
}

/*!
    \brief      SPI DMA stop function
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_spi_dma_stop(hal_spi_dev_struct *spi)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == spi) {
        HAL_DEBUGE("parameter [*spi] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(NULL != spi->p_dma_rx) {
        hal_dma_stop(spi->p_dma_rx);
    }
    if(NULL != spi->p_dma_tx) {
        hal_dma_stop(spi->p_dma_tx);
    }

    __HAL_SPI_DMA_DISABLE(spi->periph, SPI_DMA_TRANSMIT);
    __HAL_SPI_DMA_DISABLE(spi->periph, SPI_DMA_RECEIVE);
    spi->state = HAL_SPI_STATE_READY;
}

/*!
    \brief      reset SPI
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[out] none
    \retval     none
*/
static void _spi_deinit(uint32_t periph)
{
    switch(periph) {
    case SPI0:
        /* reset SPI0 */
        hal_rcu_periph_reset_enable(RCU_SPI0RST);
        hal_rcu_periph_reset_disable(RCU_SPI0RST);
        break;
    case SPI1:
        /* reset SPI1 */
        hal_rcu_periph_reset_enable(RCU_SPI1RST);
        hal_rcu_periph_reset_disable(RCU_SPI1RST);
        break;
    default :
        break;
    }
}

/*!
    \brief      SPI 8 bit transmit interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_transmit_interrupt(void *spi)
{
    hal_spi_dev_struct *p_spi = spi;
    if(__HAL_SPI_GET_FRAME_SIZE(p_spi->periph) == SPI_FRAMESIZE_16BIT) {
        SPI_DATA(p_spi->periph) = *((uint16_t *)p_spi->txbuffer.buffer);
        p_spi->txbuffer.buffer += sizeof(uint16_t);
        p_spi->txbuffer.pos--;
    } else {
        SPI_DATA(p_spi->periph) = *(p_spi->txbuffer.buffer);
        p_spi->txbuffer.buffer++;
        p_spi->txbuffer.pos--;
    }

    if(0U == p_spi->txbuffer.pos) {
        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)) {
            __HAL_SPI_CRC_NEXT(p_spi->periph);
        }
        _spi_close_transmit_interrupt(p_spi);
    }
}

/*!
    \brief      SPI receive interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_receive_interrupt(void *spi)
{
    hal_spi_dev_struct *p_spi = spi;

    if(__HAL_SPI_GET_FRAME_SIZE(p_spi->periph) == SPI_FRAMESIZE_16BIT) {
        *((uint16_t *)p_spi->rxbuffer.buffer) = SPI_DATA(p_spi->periph);
        p_spi->rxbuffer.buffer += sizeof(uint16_t);
        p_spi->rxbuffer.pos--;
    } else {
        *(p_spi->rxbuffer.buffer) = SPI_DATA(p_spi->periph);
        p_spi->rxbuffer.buffer++;
        p_spi->rxbuffer.pos--;

    }
    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph) &&
            (1U == p_spi->rxbuffer.pos)) {
        __HAL_SPI_CRC_NEXT(p_spi->periph);
    }

    if(0U == p_spi->rxbuffer.pos) {
        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)) {
            __IO uint32_t tmp = 0U;
            tmp = SPI_DATA(p_spi->periph);
        }
        _spi_close_receive_interrupt(p_spi);
    }
}

/*!
    \brief      SPI 8 bit 2 lines receive interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_2lines_receive_interrupt(void *spi)
{
    hal_spi_dev_struct *p_spi = spi;

    if(__HAL_SPI_GET_FRAME_SIZE(p_spi->periph) == SPI_FRAMESIZE_16BIT) {

        *((uint16_t *)p_spi->rxbuffer.buffer) = (uint16_t)(SPI_DATA(p_spi->periph));
        p_spi->rxbuffer.buffer += sizeof(uint16_t);
        p_spi->rxbuffer.pos--;

    } else { /* receive data in 8 bit mode */
        *(p_spi->rxbuffer.buffer) = *((__IO uint8_t *)&SPI_DATA(p_spi->periph));
        p_spi->rxbuffer.buffer++;
        p_spi->rxbuffer.pos--;
    }

    if(0U == p_spi->rxbuffer.pos) {
        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)) {
            __IO uint32_t tmp = 0;
            tmp = SPI_DATA(p_spi->periph);
        }
        /* disable RBNE and ERR interrupt */
        __HAL_SPI_INT_DISABLE(p_spi->periph, SPI_INT_RBNE);
        __HAL_SPI_INT_DISABLE(p_spi->periph, SPI_INT_ERR);
        if(0U == p_spi->txbuffer.pos) {
            _spi_close_transmit_receive_interrupt(p_spi);
        }
    }

}

/*!
    \brief      SPI 8 bit 2 lines transmit interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_2lines_transmit_interrupt(void *spi)
{
    hal_spi_dev_struct *p_spi = spi;

    if(__HAL_SPI_GET_FRAME_SIZE(p_spi->periph) == SPI_FRAMESIZE_16BIT) {
        SPI_DATA(p_spi->periph) = *((uint16_t *)p_spi->txbuffer.buffer);
        p_spi->txbuffer.buffer += sizeof(uint16_t);
        p_spi->txbuffer.pos--;

    } else { /* transmit data in 8 bit mode */
        SPI_DATA(p_spi->periph) = *(p_spi->txbuffer.buffer);
        p_spi->txbuffer.buffer++;
        p_spi->txbuffer.pos--;
    }

    /* check end of the reception */
    if(0U == p_spi->txbuffer.pos) {
        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)) {
            __HAL_SPI_CRC_NEXT(p_spi->periph);
            __HAL_SPI_INT_DISABLE(p_spi->periph, SPI_INT_TBE);
            return;
        }
        /* disable RBNE and ERR interrupt */
        __HAL_SPI_INT_DISABLE(p_spi->periph, SPI_INT_TBE);

        if(0U == p_spi->rxbuffer.pos) {
            _spi_close_transmit_receive_interrupt(p_spi);
        }
    }
}

/*!
    \brief      SPI DMA transmit handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _spi_transmit_compelete_dma(void *dma)
{
    hal_spi_user_cb p_func = NULL;
    hal_spi_user_cb p_func_err = NULL;

    hal_dma_dev_struct *p_dma;
    hal_spi_dev_struct *p_spi;

    p_dma = (hal_dma_dev_struct *)dma;
    p_spi = (hal_spi_dev_struct *)p_dma->p_periph;
    p_func = (hal_spi_user_cb)p_spi->tx_callback;
    p_func_err = (hal_spi_user_cb)p_spi->error_callback;

    /* DMA normal Mode */
    if(SET != (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        /* disable ERR interrupt */
        __HAL_SPI_INT_DISABLE(p_spi->periph, SPI_INT_ERR);
        /* disable Tx DMA Request */
        __HAL_SPI_DMA_DISABLE(p_spi->periph, SPI_DMA_TRANSMIT);

        /* check the end of the transaction */
        if(HAL_ERR_NONE != _spi_end_transmit_receive(p_spi, SPI_TIMEOUT_VALUE)) {
            p_spi->error_code = HAL_SPI_ERROR_TIMEOUT;
        }

        /* clear overrun flag in 2 Lines communication mode because received data is not read */
        if(SPI_TRANSMODE_FULLDUPLEX == __HAL_SPI_GET_DEVICE_MODE(p_spi->periph)) {
            if(SET == _spi_flag_get(p_spi->periph, SPI_FLAG_RXORERR)) {
                _spi_clear_error_flag(p_spi->periph, SPI_ERROR_FLAG_RXORERR);
                p_spi->error_code = HAL_SPI_ERROR_CRC;
            }
        }

        p_spi->txbuffer.pos = 0U;
        p_spi->state = HAL_SPI_STATE_READY;

        if(p_spi->error_code != HAL_ERR_NONE) {
            if(NULL != p_func_err) {
                p_func_err(p_spi);
                return;
            }
        }
    }
    if(NULL != p_func) {
        p_func(p_spi);
    }

}

/*!
    \brief      SPI DMA receive handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _spi_receive_compelete_dma(void *dma)
{
    __IO uint16_t tmpcrc = 0U;
    hal_spi_user_cb p_func = NULL;
    hal_spi_user_cb p_func_err = NULL;

    hal_dma_dev_struct *p_dma;
    hal_spi_dev_struct *p_spi;

    p_dma = (hal_dma_dev_struct *)dma;
    p_spi = (hal_spi_dev_struct *)p_dma->p_periph;
    p_func = (hal_spi_user_cb)p_spi->rx_callback;
    p_func_err = (hal_spi_user_cb)p_spi->error_callback;

    /* DMA Normal Mode */
    if(SET != (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        /* Disable ERR interrupt */
        __HAL_SPI_INT_DISABLE(p_spi->periph, SPI_INT_ERR);
        /* disable Rx/Tx DMA Request */
        __HAL_SPI_DMA_DISABLE(p_spi->periph, SPI_DMA_TRANSMIT);
        __HAL_SPI_DMA_DISABLE(p_spi->periph, SPI_DMA_RECEIVE);
        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)) {
            tmpcrc = SPI_DATA(p_spi->periph);
        }

        /* check the end of the transaction */
        if(_spi_end_receive(p_spi, 200) != HAL_ERR_NONE) {
            p_spi->error_code = HAL_SPI_ERROR_TIMEOUT;
        }

        p_spi->rxbuffer.pos = 0U;
        p_spi->state = HAL_SPI_STATE_READY;

        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)) {
            if(SET == _spi_flag_get(p_spi->periph, SPI_FLAG_CRCERR)) {
                _spi_clear_error_flag(p_spi->periph, SPI_ERROR_FLAG_CRCERR);
                p_spi->error_code = HAL_SPI_ERROR_CRC;
            }
        }

        if(p_spi->error_code != HAL_ERR_NONE) {
            if(NULL != p_func_err) {
                p_func_err(p_spi);
                return;
            }
        }
    }
    if(NULL != p_func) {
        p_func(p_spi);
    }
}

/*!
    \brief      SPI DMA transmit and receive handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _spi_transmit_receive_compelete_dma(void *dma)
{
    uint32_t tick_start = 0;
    __IO uint16_t tmpcrc = 0U;
    hal_spi_user_cb p_func = NULL;
    hal_spi_user_cb p_func_err = NULL;

    hal_dma_dev_struct *p_dma;
    hal_spi_dev_struct *p_spi;

    p_dma = (hal_dma_dev_struct *)dma;
    p_spi = (hal_spi_dev_struct *)p_dma->p_periph;
    p_func = (hal_spi_user_cb)p_spi->tx_rx_callback;
    p_func_err = (hal_spi_user_cb)p_spi->error_callback;

    /* DMA normal mode */
    if(SET != (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)) {
        /* disable ERR interrupt */
        __HAL_SPI_INT_DISABLE(p_spi->periph, SPI_INT_ERR);

        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)) {
            tick_start = hal_sys_basetick_count_get();
            while(RESET == _spi_flag_get(p_spi->periph, SPI_FLAG_RBNE)) {
                if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE) {
                    if(SET == hal_sys_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)) {
                        __HAL_SPI_CRC_OFF(p_spi->periph);
                        __HAL_SPI_CRC_ON(p_spi->periph);
                        break;
                    }
                }
            }
            tmpcrc = SPI_DATA(p_spi->periph);
        }

        /* check the end of the transaction */
        if(_spi_end_transmit_receive(p_spi, SPI_TIMEOUT_VALUE) != HAL_ERR_NONE) {
            p_spi->error_code = HAL_SPI_ERROR_TIMEOUT;
        }

        /* disable Rx and Tx DMA */
        __HAL_SPI_DMA_DISABLE(p_spi->periph, SPI_DMA_TRANSMIT);
        __HAL_SPI_DMA_DISABLE(p_spi->periph, SPI_DMA_RECEIVE);

        p_spi->rxbuffer.pos = 0U;
        p_spi->txbuffer.pos = 0U;
        p_spi->state = HAL_SPI_STATE_READY;

        if(p_spi->error_code != HAL_ERR_NONE) {
            if(NULL != p_func_err) {
                p_func_err(p_spi);
                return;
            }
        }
    }
    if(NULL != p_func) {
        p_func(p_spi);
    }

}

/*!
    \brief      SPI DMA error handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _spi_dma_error(void *dma)
{
    hal_spi_user_cb p_func_err = NULL;

    hal_dma_dev_struct *p_dma;
    hal_spi_dev_struct *p_spi;

    p_dma = (hal_dma_dev_struct *)dma;
    p_spi = (hal_spi_dev_struct *)p_dma->p_periph;
    p_func_err = (hal_spi_user_cb)p_spi->error_callback;

    __HAL_SPI_DMA_DISABLE(p_spi->periph, SPI_DMA_RECEIVE);
    __HAL_SPI_DMA_DISABLE(p_spi->periph, SPI_DMA_TRANSMIT);

    p_spi->error_code = HAL_SPI_ERROR_DMA;
    p_spi->state = HAL_SPI_STATE_READY;

    if(NULL != p_func_err) {
        p_func_err(p_spi);
    }
}

/*!
    \brief      SPI stop receive  interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_stop_receive_interrupt(void *spi)
{
    uint32_t tick_start = 0;
    hal_spi_dev_struct *p_spi = spi;
    /* disable SPI Peripheral */
    __HAL_SPI_DISABLE(p_spi->periph);

    /* disable TBEIE, RBNEIE and ERRIE interrupts */
    __HAL_SPI_INT_DISABLE(p_spi->periph, SPI_INT_TBE);
    __HAL_SPI_INT_DISABLE(p_spi->periph, SPI_INT_RBNE);
    __HAL_SPI_INT_DISABLE(p_spi->periph, SPI_INT_ERR);

    /* check RBNEIE is diabled */
    tick_start = hal_sys_basetick_count_get();
    while((SPI_CTL1(p_spi->periph) & SPI_CTL1_RBNEIE)) {
        if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)) {
                p_spi->error_code = HAL_SPI_ERROR_ABORT;
            }
        }
    }

    tick_start = hal_sys_basetick_count_get();
    while(RESET != _spi_flag_get(p_spi->periph, SPI_FLAG_TRANS)) {
        if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)) {
                p_spi->error_code = HAL_SPI_ERROR_ABORT;
            }
        }
    }

    p_spi->state = HAL_SPI_STATE_ABORT;
}

/*!
    \brief      SPI stop transmit interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_stop_transmit_interrupt(void *spi)
{
    hal_spi_dev_struct *p_spi = spi;

    /* disable TBEIE, ERRIE(mode fault event, overrun error, TI frame error) interrupts */
    __HAL_SPI_INT_DISABLE(p_spi->periph, SPI_INT_TBE);
    __HAL_SPI_INT_DISABLE(p_spi->periph, SPI_INT_ERR);

    /* disable SPI peripheral */
    __HAL_SPI_DISABLE(p_spi->periph);
}

/*!
    \brief      SPI close transmit and  receive interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_close_transmit_receive_interrupt(hal_spi_dev_struct *spi)
{
    hal_spi_user_cb p_func = NULL;

    /* disable ERR interrupt */
    __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_ERR);
    /* check the end of the transaction */
    if(_spi_end_transmit_receive(spi, SPI_TIMEOUT_VALUE) != HAL_ERR_NONE) {
        spi->error_code = HAL_SPI_ERROR_TIMEOUT;
    }
    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)) {
        if(RESET != _spi_flag_get(spi->periph, SPI_FLAG_CRCERR)) {
            _spi_clear_error_flag(spi->periph, SPI_ERROR_FLAG_CRCERR);
            p_func = (hal_spi_user_cb)spi->error_callback;
            if(NULL != p_func) {
                p_func(spi);
            }
            return;
        }
    }
    if(HAL_ERR_NONE == spi->error_code) {
        if(HAL_SPI_STATE_BUSY_RX == spi->state) {
            spi->state = HAL_SPI_STATE_READY;
            p_func = (hal_spi_user_cb)spi->tx_callback;
            if(NULL != p_func) {
                p_func(spi);
            }
        } else {
            spi->state = HAL_SPI_STATE_READY;
            p_func = (hal_spi_user_cb)spi->tx_rx_callback;
            if(NULL != p_func) {
                p_func(spi);
            }
        }
    } else {
        spi->state = HAL_SPI_STATE_READY;
        p_func = (hal_spi_user_cb)spi->error_callback;
        if(NULL != p_func) {
            p_func(spi);
        }
    }

}

/*!
    \brief      SPI close receive interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_close_receive_interrupt(hal_spi_dev_struct *spi)
{
    hal_spi_user_cb p_func = NULL;

    /* disable ERR interrupt */
    __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_ERR);
    __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_RBNE);

    if(HAL_ERR_NONE != _spi_end_receive(spi, SPI_TIMEOUT_VALUE)) {
        spi->error_code = HAL_SPI_ERROR_TIMEOUT;
    }

    spi->state = HAL_SPI_STATE_READY;

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)) {
        if(RESET != _spi_flag_get(spi->periph, SPI_FLAG_CRCERR)) {
            _spi_clear_error_flag(spi->periph, SPI_ERROR_FLAG_CRCERR);
            p_func = (hal_spi_user_cb)spi->error_callback;
            if(NULL != p_func) {
                p_func(spi);
            }
            return;
        }
    }
    if(HAL_SPI_ERROR_NONE == spi->error_code) {
        p_func = (hal_spi_user_cb)spi->rx_callback;
        if(NULL != p_func) {
            p_func(spi);
        }
    } else {
        p_func = (hal_spi_user_cb)spi->error_callback;
        if(NULL != p_func) {
            p_func(spi);
        }
    }
}

/*!
    \brief      SPI close transmit interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_close_transmit_interrupt(hal_spi_dev_struct *spi)
{
    hal_spi_user_cb p_func = NULL;

    /* disable TBE and ERR interrupt */
    __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_ERR);
    __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_TBE);

    /* check the end of the transaction */
    if(_spi_end_transmit_receive(spi, SPI_TIMEOUT_VALUE) != HAL_ERR_NONE) {
        p_func = (hal_spi_user_cb)spi->error_callback;
        if(NULL != p_func) {
            p_func(spi);
        }
    } else {
        p_func = (hal_spi_user_cb)spi->tx_callback;
        spi->state = HAL_SPI_STATE_READY;
        if(NULL != p_func) {
            p_func(spi);
        }
    }

}

/*!
    \brief      check SPI receive end
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
static int32_t _spi_end_receive(hal_spi_dev_struct *spi,  uint32_t timeout_ms)
{
    uint32_t tick_start = 0;
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == spi) {
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if((SPI_MASTER == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) &&
            (SPI_TRANSMODE_FULLDUPLEX != __HAL_SPI_GET_DEVICE_MODE(spi->periph))) {
        /* disable SPI peripheral */
        __HAL_SPI_DISABLE(spi->periph);
    }

    tick_start = hal_sys_basetick_count_get();
    while(RESET != _spi_flag_get(spi->periph, SPI_FLAG_TRANS)) {
        if(HAL_TIMEOUT_FOREVER != timeout_ms) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      check SPI transmit and receive end
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
static int32_t _spi_end_transmit_receive(hal_spi_dev_struct *spi,
        uint32_t timeout_ms)
{
    uint32_t tick_start = 0;
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == spi) {
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check if the SPI trans bit is SET */
    tick_start = hal_sys_basetick_count_get();
    while(RESET != _spi_flag_get(spi->periph, SPI_FLAG_TRANS)) {
        if(HAL_TIMEOUT_FOREVER != timeout_ms) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_TBE);
    __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_RBNE);
    __HAL_SPI_INT_DISABLE(spi->periph, SPI_INT_ERR);
    __HAL_SPI_DISABLE(spi->periph);

    return HAL_ERR_NONE;
}

/*!
    \brief      check SPI transmit and receive end
    \param[in]  periph: SPIx(x=0,1)
    \param[in]  error_flag: error flag
                only one parameter can be selected which is shown as below:
      \arg        SPI_ERROR_FLAG_CONF: SPI configuration error
      \arg        SPI_ERROR_FLAG_FERR: SPI format error
      \arg        SPI_ERROR_FLAG_CRCERR: SPI CRC error
      \arg        SPI_ERROR_FLAG_RXORERR: SPI reception overrun error
    \param[out] none
    \retval     none
*/
static void _spi_clear_error_flag(uint32_t periph, uint32_t error_flag)
{
    __IO uint32_t temp = 0;
    switch(error_flag) {
    case SPI_ERROR_FLAG_CONF:
        temp = SPI_STAT(periph);
        SPI_CTL0(periph) &= (uint32_t)(~SPI_CTL0_SPIEN);
        break;
    case SPI_ERROR_FLAG_RXORERR:
        temp = SPI_DATA(periph);
        temp = SPI_STAT(periph);
        break;
    case SPI_ERROR_FLAG_FERR:
        temp = SPI_STAT(periph);
        break;
    case SPI_ERROR_FLAG_CRCERR:
        SPI_STAT(periph) &= (uint32_t)(~SPI_FLAG_CRCERR);
        break;
    default:
        break;
    }
}

/*!
    \brief      get SPI flag status
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  flag: SPI flag status
                only one parameter can be selected which are shown as below:
      \arg        SPI_FLAG_TBE: transmit buffer empty flag
      \arg        SPI_FLAG_RBNE: receive buffer not empty flag
      \arg        SPI_FLAG_TRANS: transmit on-going flag
      \arg        SPI_FLAG_RXORERR: receive overrun error flag
      \arg        SPI_FLAG_CONFERR: mode config error flag
      \arg        SPI_FLAG_CRCERR: CRC error flag
      \arg        SPI_FLAG_FERR: SPI format error interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
static FlagStatus _spi_flag_get(uint32_t periph, uint32_t flag)
{
    if(RESET != (SPI_STAT(periph) & flag)) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      get SPI interrupt flag status
    \param[in]  spi_periph: SPIx(x=0,1)
    \param[in]  interrupt: SPI interrupt flag status
                only one parameter can be selected which is shown as below:
      \arg        SPI_INT_FLAG_TBE: transmit buffer empty interrupt flag
      \arg        SPI_INT_FLAG_RBNE: receive buffer not empty interrupt flag
      \arg        SPI_INT_FLAG_RXORERR: overrun interrupt flag
      \arg        SPI_INT_FLAG_CONFERR: config error interrupt flag
      \arg        SPI_INT_FLAG_CRCERR: CRC error interrupt flag
      \arg        SPI_INT_FLAG_FERR: format error interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
static FlagStatus _spi_interrupt_flag_get(uint32_t periph, uint8_t interrupt)
{
    uint32_t reg1 = SPI_STAT(periph);
    uint32_t reg2 = SPI_CTL1(periph);

    switch(interrupt) {
    /* SPI transmit buffer empty interrupt */
    case SPI_INT_FLAG_TBE:
        reg1 = reg1 & SPI_STAT_TBE;
        reg2 = reg2 & SPI_CTL1_TBEIE;
        break;
    /* SPI receive buffer not empty interrupt */
    case SPI_INT_FLAG_RBNE:
        reg1 = reg1 & SPI_STAT_RBNE;
        reg2 = reg2 & SPI_CTL1_RBNEIE;
        break;
    /* SPI overrun interrupt */
    case SPI_INT_FLAG_RXORERR:
        reg1 = reg1 & SPI_STAT_RXORERR;
        reg2 = reg2 & SPI_CTL1_ERRIE;
        break;
    /* SPI config error interrupt */
    case SPI_INT_FLAG_CONFERR:
        reg1 = reg1 & SPI_STAT_CONFERR;
        reg2 = reg2 & SPI_CTL1_ERRIE;
        break;
    /* SPI CRC error interrupt */
    case SPI_INT_FLAG_CRCERR:
        reg1 = reg1 & SPI_STAT_CRCERR;
        reg2 = reg2 & SPI_CTL1_ERRIE;
        break;
    /* SPI format error interrupt */
    case SPI_INT_FLAG_FERR:
        reg1 = reg1 & SPI_STAT_FERR;
        reg2 = reg2 & SPI_CTL1_ERRIE;
        break;
    default :
        break;
    }
    /*get SPI interrupt flag status */
    if((0U != reg1) && (0U != reg2)) {
        return SET;
    } else {
        return RESET;
    }
}
