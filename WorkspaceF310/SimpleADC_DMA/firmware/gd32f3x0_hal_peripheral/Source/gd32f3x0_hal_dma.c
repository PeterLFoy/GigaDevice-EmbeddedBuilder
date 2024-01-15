/*!
    \file    gd32f3x0_hal_dma.h
    \brief   DMA driver

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

#define _DEV_VALID_ADDRESS           ((uint32_t)0x68000000)

/*!
    \brief      initialize DMA channel
    \param[in]  dma_dev: DMA device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channelx:specify which DMA channel is initialized.
                  the argument could be selected from enumeration <dma_channel_enum>
                  \arg        DMA_CHx(x=0..6)
    \param[in]  dma: the pointer of DMA init structure
                  direction: DMA_DIR_PERIPH_TO_MEMORY, DMA_DIR_MEMORY_TO_PERIPH, DMA_DIR_MEMORY_TO_MEMORY
                  priority: DMA_PRIORITY_LEVEL_LOW, DMA_PRIORITY_LEVEL_MEDIUM, DMA_PRIORITY_LEVEL_HIGH, \
                            DMA_PRIORITY_LEVEL_ULTRA_HIGH:
                  mode: DMA_NORMAL_MODE, DMA_CIRCULAR_MODE
                  periph_inc: ENABLE, DISABLE
                  memory_inc: ENABLE, DISABLE
                  periph_width: DMA_PERIPH_SIZE_8BITS,DMA_PERIPH_SIZE_16BITS,DMA_PERIPH_SIZE_32BITS
                  memory_width: DMA_MEMORY_SIZE_8BITS,DMA_MEMORY_SIZE_16BITS,DMA_MEMORY_SIZE_32BITS
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE,HAL_ERR_VAL details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_init(hal_dma_dev_struct *dma_dev, dma_channel_enum channelx, \
                     hal_dma_init_struct *dma)
{
    uint32_t ctl;
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == dma_dev) || (NULL == dma)) {
        HAL_DEBUGE("pointer [dma] or [dma_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check DMA mode and direction parameters */
    if((DMA_CIRCULAR_MODE == dma->mode) && (DMA_DIR_MEMORY_TO_MEMORY  \
        == dma->direction)) {
        HAL_DEBUGI("circular mode is invalid due to 'memory to memory' has been configured");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* change DMA device state */
    dma_dev->state = HAL_DMA_STATE_BUSY;

    dma_dev->channel = channelx;
    /* disable dma channelx */
    hals_dma_channel_disable(dma_dev->channel);

    /* configure peripheral transfer width,memory transfer width,channel priority */
    ctl = DMA_CHCTL(channelx);
    ctl &= ~(DMA_CHXCTL_PWIDTH | DMA_CHXCTL_MWIDTH | DMA_CHXCTL_PRIO | \
             DMA_CHXCTL_M2M | DMA_CHXCTL_DIR | DMA_CHXCTL_CMEN);
    ctl |= (dma->periph_width | dma->memory_width | dma->priority | \
            dma->direction | dma->mode);
    DMA_CHCTL(channelx) = ctl;
    /* configure peripheral increasing mode */
    if(ENABLE == dma->periph_inc) {
        DMA_CHCTL(channelx) |= DMA_CHXCTL_PNAGA;
    } else {
        DMA_CHCTL(channelx) &= ~DMA_CHXCTL_PNAGA;
    }

    /* configure memory increasing mode */
    if(ENABLE == dma->memory_inc) {
        DMA_CHCTL(channelx) |= DMA_CHXCTL_MNAGA;
    } else {
        DMA_CHCTL(channelx) &= ~DMA_CHXCTL_MNAGA;
    }
    /* change DMA device state */
    dma_dev->state = HAL_DMA_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      initialize the DMA structure with the default values
    \param[in]  struct_type: the argument could be selected from enumeration <hal_dma_struct_type_enum>
    \param[in]  p_struct: pointer to DMA structure that contains the configuration information
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE,HAL_ERR_VAL details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_struct_init(hal_dma_struct_type_enum struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct) {
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(struct_type) {
    case HAL_DMA_INIT_STRUCT:
        /* initialize DMA initialization structure with the default values */
        ((hal_dma_init_struct *)p_struct)->direction = DMA_DIR_PERIPH_TO_MEMORY;
        ((hal_dma_init_struct *)p_struct)->priority = DMA_PRIORITY_LEVEL_LOW;
        ((hal_dma_init_struct *)p_struct)->mode = DMA_NORMAL_MODE;
        ((hal_dma_init_struct *)p_struct)->periph_inc = DISABLE;
        ((hal_dma_init_struct *)p_struct)->memory_inc = ENABLE;
        ((hal_dma_init_struct *)p_struct)->periph_width = DMA_PERIPH_SIZE_8BITS;
        ((hal_dma_init_struct *)p_struct)->memory_width = DMA_MEMORY_SIZE_8BITS;

        break;

    case HAL_DMA_DEV_STRUCT:
        /* initialize DMA device information structure with the default values */
        ((hal_dma_dev_struct *)p_struct)->channel = DMA_CH0;
        ((hal_dma_dev_struct *)p_struct)->dma_irq.error_handle = NULL;
        ((hal_dma_dev_struct *)p_struct)->dma_irq.half_finish_handle = NULL;
        ((hal_dma_dev_struct *)p_struct)->dma_irq.full_finish_handle = NULL;
        ((hal_dma_dev_struct *)p_struct)->error_state = HAL_DMA_ERROR_NONE;
        ((hal_dma_dev_struct *)p_struct)->state = HAL_DMA_STATE_NONE;
        ((hal_dma_dev_struct *)p_struct)->p_periph = NULL;
        ((hal_dma_dev_struct *)p_struct)->mutex = HAL_MUTEX_UNLOCKED;

        break;

    case HAL_DMA_IRQ_STRUCT:
        /* initialize DMA device interrupt callback function structure with the default values */
        ((hal_dma_irq_struct *)p_struct)->error_handle = NULL;
        ((hal_dma_irq_struct *)p_struct)->full_finish_handle = NULL;
        ((hal_dma_irq_struct *)p_struct)->half_finish_handle = NULL;

        break;
    default:
        HAL_DEBUGE("parameter [struct_type] value is invalid");
        return HAL_ERR_VAL;
    }

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      deinitialize DMA device structure and init structure
    \param[in]  dma_dev: DMA device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_deinit(hal_dma_dev_struct *dma_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == dma_dev) {
        HAL_DEBUGE("parameter [*dma_dev] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    /* deinitialize DMA a channel registers */
    /* disable DMA a channel */
    DMA_CHCTL(dma_dev->channel) &= ~DMA_CHXCTL_CHEN;
    /* reset DMA channel registers */
    DMA_CHCTL(dma_dev->channel) = DMA_CHCTL_RESET_VALUE;
    DMA_CHCNT(dma_dev->channel) = DMA_CHCNT_RESET_VALUE;
    DMA_CHPADDR(dma_dev->channel) = DMA_CHPADDR_RESET_VALUE;
    DMA_CHMADDR(dma_dev->channel) = DMA_CHMADDR_RESET_VALUE;
    DMA_INTC |= DMA_FLAG_ADD(DMA_CHINTF_RESET_VALUE, dma_dev->channel);
    /* reset DMA devices */
    hal_dma_struct_init(HAL_DMA_DEV_STRUCT, dma_dev);
    /* reset DMA device interrupt callback functions */
    dma_dev->dma_irq.error_handle = NULL;
    dma_dev->dma_irq.full_finish_handle = NULL;
    dma_dev->dma_irq.half_finish_handle = NULL;
    dma_dev->error_state = HAL_DMA_ERROR_NONE;
    dma_dev->state = HAL_DMA_STATE_RESET;
    dma_dev->p_periph = NULL;
    dma_dev->mutex = HAL_MUTEX_UNLOCKED;

    return HAL_ERR_NONE;
}

/*!
    \brief      start DMA transfering
    \param[in]  dma_dev: DMA device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  src_addr: the source memory buffer address
    \param[in]  dst_addr: the destination memory buffer address
    \param[in]  length: the number of data to be transferred from source to destination
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_BUSY, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_start(hal_dma_dev_struct *dma_dev, uint32_t src_addr, \
                      uint32_t dst_addr, uint16_t length)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check DMA pointer address */
    if(NULL == dma_dev) {
        HAL_DEBUGE("pointer [dma_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(0 == (src_addr & _DEV_VALID_ADDRESS)) {
        HAL_DEBUGE("parameter [src_addr] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    if(0 == (dst_addr & _DEV_VALID_ADDRESS)) {
        HAL_DEBUGE("parameter [dst_addr] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    /* lock DMA */
    HAL_LOCK(dma_dev);
    if(HAL_DMA_STATE_READY == dma_dev->state) {

        dma_dev->state = HAL_DMA_STATE_BUSY ;
        dma_dev->error_state =  HAL_DMA_ERROR_NONE;
        /* disable DMA channel */
        hals_dma_channel_disable(dma_dev->channel);

        if(RESET != (DMA_CHCTL(dma_dev->channel) & DMA_CHXCTL_DIR)) {
            /* configure the transfer destination and source address */
            hals_dma_memory_address_config(dma_dev->channel, src_addr);
            hals_dma_periph_address_config(dma_dev->channel, dst_addr);
        } else {
            /* configure the transfer destination and source address */
            hals_dma_memory_address_config(dma_dev->channel, dst_addr);
            hals_dma_periph_address_config(dma_dev->channel, src_addr);
        }
        /* configure the transfer number */
        hals_dma_transfer_number_config(dma_dev->channel, length);
        /* enable DMA channel */
        hals_dma_channel_enable(dma_dev->channel);
    } else {
        /* unlock DMA */
        HAL_UNLOCK(dma_dev);
        
        return HAL_ERR_BUSY;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      stop DMA transfering
    \param[in]  dma_dev: DMA device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE,HAL_ERR_ALREADY_DONE details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_stop(hal_dma_dev_struct *dma_dev)
{   
#if (1 == HAL_PARAMETER_CHECK)
    /* check DMA pointer address */
    if(NULL == dma_dev) {
        HAL_DEBUGE("pointer [dma_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    if(HAL_DMA_STATE_BUSY != dma_dev->state) {
        /* dma not in transfer state */
        dma_dev->error_state = HAL_DMA_ERROR_NOTRANSFER;
        /* unlock DMA */
        HAL_UNLOCK(dma_dev);

        return HAL_ERR_ALREADY_DONE;

    } else {

        /* disable DMA channel */
        hals_dma_channel_disable(dma_dev->channel);
        /* clear all flags */
        hals_dma_flag_clear(dma_dev->channel, DMA_INTF_GIF);

    }
    dma_dev->state = HAL_DMA_STATE_READY ;
    /* unlock DMA */
    HAL_UNLOCK(dma_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      DMA interrupt handler content function,which is merely used in dma_handler
    \param[in]  dma_dev: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_irq(hal_dma_dev_struct *dma_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the DMA pointer address and the number length parameter */
    if(NULL == dma_dev) {
        HAL_DEBUGE("parameter [dma_dev] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    /* full transfer finish interrupt handler */
    if(SET == hals_dma_interrupt_flag_get(dma_dev->channel, DMA_INTF_FTFIF)) {
        hals_dma_interrupt_flag_clear(dma_dev->channel, DMA_INTF_FTFIF);
        if(dma_dev->dma_irq.full_finish_handle != NULL) {
            dma_dev->dma_irq.full_finish_handle(dma_dev);
            /* unlock DMA */
            dma_dev->state = HAL_DMA_STATE_READY;
            HAL_UNLOCK(dma_dev);
        }
    }

    /* half transfer finish interrupt handler */
    if(SET == hals_dma_interrupt_flag_get(dma_dev->channel, DMA_INTF_HTFIF)) {
        /* if DMA not in circular mode */
        if(0 == (DMA_CHCTL(dma_dev->channel) & DMA_CHXCTL_CMEN)) {
            hals_dma_interrupt_disable(dma_dev->channel, DMA_INT_HTF);
                    }
        hals_dma_interrupt_flag_clear(dma_dev->channel, DMA_INTF_HTFIF);
        if(dma_dev->dma_irq.half_finish_handle != NULL) {
            dma_dev->dma_irq.half_finish_handle(dma_dev);
        }


    }

    /* error interrupt handler */
    if(SET == hals_dma_interrupt_flag_get(dma_dev->channel, DMA_INTF_ERRIF)) {
        hals_dma_interrupt_flag_clear(dma_dev->channel, DMA_INTF_GIF);
        dma_dev->state = HAL_DMA_STATE_READY;
        dma_dev->error_state = HAL_DMA_ERROR_TRANSFER;
        /* unlock DMA */
        HAL_UNLOCK(dma_dev);
        if(dma_dev->dma_irq.error_handle != NULL) {
            dma_dev->dma_irq.error_handle(dma_dev);
        }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      set user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to DMA interrupt callback function pointer structure
                  The structure member can be assigned as following parameters:
      \arg        hal_irq_handle_cb function pointer: the function is user-defined,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_irq_handle_set(hal_dma_dev_struct *dma_dev, hal_dma_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the DMA pointer address and the number length parameter */
    if(NULL == dma_dev) {
        HAL_DEBUGE("parameter [dma_dev] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    /* lock DMA */
    HAL_LOCK(dma_dev);
    /* disable DMA channel */
    hals_dma_channel_disable(dma_dev->channel);
    /* set the handler pointer  */
    if(NULL != p_irq) {
        dma_dev->dma_irq.error_handle = p_irq->error_handle;
        dma_dev->dma_irq.full_finish_handle = p_irq->full_finish_handle;
        dma_dev->dma_irq.half_finish_handle = p_irq->half_finish_handle;
    }

    /* enable the channel error interrupt */
    if(NULL != dma_dev->dma_irq.error_handle) {
        hals_dma_interrupt_enable(dma_dev->channel, DMA_INT_ERR);
    } else {
        hals_dma_interrupt_disable(dma_dev->channel, DMA_INT_ERR);
        HAL_DEBUGE("parameter [error_handle] value is invalid");
    }

    /* set the full finish handler pointer and enable the channel full transfer finish  interrupt */
    if(NULL != dma_dev->dma_irq.full_finish_handle) {
        hals_dma_interrupt_enable(dma_dev->channel, DMA_INT_FTF);
    } else {
        hals_dma_interrupt_disable(dma_dev->channel, DMA_INT_FTF);
        HAL_DEBUGE("parameter [full_finish_handle] value is invalid");
    }

    /* set the half finish handler pointer and enable the channel half transfer finish interrupt */
    if(NULL != dma_dev->dma_irq.half_finish_handle) {
        hals_dma_interrupt_enable(dma_dev->channel, DMA_INT_HTF);
    } else {
        hals_dma_interrupt_disable(dma_dev->channel, DMA_INT_HTF);
        HAL_DEBUGE("parameter [half_finish_handle] value is invalid");
    }
    /* enable DMA channel */
    hals_dma_channel_enable(dma_dev->channel);
    /* unlock DMA */
    HAL_UNLOCK(dma_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_irq_handle_all_reset(hal_dma_dev_struct *dma_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the DMA pointer address and the number length parameter */
    if(NULL == dma_dev) {
        HAL_DEBUGE("parameter [dma_dev] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    /* lock DMA */
    HAL_LOCK(dma_dev);
    /* DMA interrupt handler reset */
    dma_dev->dma_irq.error_handle = NULL;
    dma_dev->dma_irq.half_finish_handle = NULL;
    dma_dev->dma_irq.full_finish_handle = NULL;
    /* unlock DMA */
    HAL_UNLOCK(dma_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      start transfering amounts of data, poll transmit process and completed status
    \param[in]  dma_dev: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  transfer_state: refer to hal_dma_transfer_state_enum
    \param[in]  timeout_ms: time out duration
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, HAL_ERR_NONE, HAL_ERR_NO_SUPPORT,  \
                HAL_ERR_ALREADY_DONE, HAL_ERR_TIMEOUT  details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_start_poll(hal_dma_dev_struct *dma_dev, hal_dma_transfer_state_enum \
                           transfer_state, uint32_t timeout_ms)
{
    uint32_t flag, tick_start = 0;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == dma_dev) {
        HAL_DEBUGE("parameter [dma_dev] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check the cirulation mode of the data transfer */
    if(RESET != (DMA_CHCTL(dma_dev->channel) & DMA_CHXCTL_CMEN)) {
        /* polling mode is not supported in circular mode */
        HAL_DEBUGE("DMA poll function is invalid in circulation mode");
        
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(HAL_DMA_STATE_BUSY != dma_dev->state) {
        /* no transfer */
        dma_dev->error_state = HAL_DMA_ERROR_NOTRANSFER;
        HAL_UNLOCK(dma_dev);

        return HAL_ERR_ALREADY_DONE;
    }
    /* get the transfer flag */
    if(HAL_DMA_TARNSFER_HALF == transfer_state) {
        flag = DMA_FLAG_ADD(DMA_INTF_HTFIF, dma_dev->channel);
    } else {
        flag = DMA_FLAG_ADD(DMA_INTF_FTFIF, dma_dev->channel);
    }

    /* set timeout */
    tick_start = hal_sys_basetick_count_get();

    while(RESET == (DMA_INTF & flag)) {
        if(SET == hals_dma_flag_get(dma_dev->channel, DMA_INTF_ERRIF)) {
            /* when error occurs, clear all flags and output error message */
            dma_dev->error_state = HAL_DMA_ERROR_TRANSFER;
            dma_dev->state = HAL_DMA_STATE_READY;
            hals_dma_flag_clear(dma_dev->channel, DMA_INTF_GIF);
            HAL_UNLOCK(dma_dev);
            HAL_DEBUGE("dma transfer error, poll stop");

            return HAL_ERR_HARDWARE;
        }
        /* check for the timeout */
        if(HAL_TIMEOUT_FOREVER != timeout_ms) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)){
            
                dma_dev->error_state = HAL_DMA_ERROR_TIMEOUT;
                dma_dev->state = HAL_DMA_STATE_TIMEOUT;
                /* when timeout occurs, output timeout warning message */
                HAL_DEBUGW("dma transfer state poll timeout");
                HAL_UNLOCK(dma_dev);

                return HAL_ERR_TIMEOUT;
                
            }
        }
    }

    hals_dma_flag_clear(dma_dev->channel, DMA_INTF_GIF);

    if(HAL_DMA_TARNSFER_FULL == transfer_state) {
        dma_dev->state = HAL_DMA_STATE_READY;
    }
    /* unlock DMA */
    HAL_UNLOCK(dma_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      start transfering amounts of data by interrupt method,the function is non-blocking
    \param[in]  dma_dev: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  src_addr: the source memory buffer address
    \param[in]  dst_addr: the destination memory buffer address
    \param[in]  length: the number of data to be transferred from source to destination
    \param[in]  p_irq: device interrupt callback function structure
                  error_handle: channel error interrupt handler pointer
                  half_finish_handle: channel half transfer finish interrupt handler pointer
                  full_finish_handle: channel full transfer finish interrupt handler pointer
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_start_interrupt(hal_dma_dev_struct *dma_dev, uint32_t src_addr, \
                                uint32_t dst_addr, uint16_t length, \
                                hal_dma_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the DMA pointer address and the number length parameter */
    if((NULL == dma_dev) || (0 == length)) {
        HAL_DEBUGE("parameter [dma_dev] and [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check src_addr parameter */
    if(0 == (src_addr & _DEV_VALID_ADDRESS)) {
        HAL_DEBUGE("parameter [src_addr] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check dst_addr parameter */
    if(0 == (dst_addr & _DEV_VALID_ADDRESS)) {
        HAL_DEBUGE("parameter [dst_addr] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    /* lock DMA */
    HAL_LOCK(dma_dev);

    if(HAL_DMA_STATE_READY == dma_dev->state) {

        dma_dev->state = HAL_DMA_STATE_BUSY;
        dma_dev->error_state = HAL_DMA_ERROR_NONE;
        /* set the handler pointer  */
        hals_dma_channel_disable(dma_dev->channel);
        /* set the handler pointer  */
        if(NULL != p_irq) {
            dma_dev->dma_irq.error_handle = p_irq->error_handle;
            dma_dev->dma_irq.full_finish_handle = p_irq->full_finish_handle;
            dma_dev->dma_irq.half_finish_handle = p_irq->half_finish_handle;
        }

        /* enable the channel error interrupt */
        if(NULL != dma_dev->dma_irq.error_handle) {
            hals_dma_interrupt_enable(dma_dev->channel, DMA_INT_ERR);
        } else {
            hals_dma_interrupt_disable(dma_dev->channel, DMA_INT_ERR);
            HAL_DEBUGE("parameter [error_handle] value is invalid");

        }

        /* set the full finish handler pointer and enable the channel full transfer finish  interrupt */
        if(NULL != dma_dev->dma_irq.full_finish_handle) {
            hals_dma_interrupt_enable(dma_dev->channel, DMA_INT_FTF);
        } else {
            hals_dma_interrupt_disable(dma_dev->channel, DMA_INT_FTF);
            HAL_DEBUGE("parameter [full_finish_handle] value is invalid");

        }

        /* set the half finish handler pointer and enable the channel half transfer finish interrupt */
        if(NULL != dma_dev->dma_irq.half_finish_handle) {
            hals_dma_interrupt_enable(dma_dev->channel, DMA_INT_HTF);
        } else {
            hals_dma_interrupt_disable(dma_dev->channel, DMA_INT_HTF);
            HAL_DEBUGE("parameter [half_finish_handle] value is invalid");

        }

        /* clear all flags */
        hals_dma_flag_clear(dma_dev->channel, DMA_INTF_GIF);

        /* check the direction of the data transfer */
        if(RESET != (DMA_CHCTL(dma_dev->channel) & DMA_CHXCTL_DIR)) {
            /* configure the transfer destination and source address */
            hals_dma_memory_address_config(dma_dev->channel, src_addr);
            hals_dma_periph_address_config(dma_dev->channel, dst_addr);
        } else {
            /* configure the transfer destination and source address */
            hals_dma_memory_address_config(dma_dev->channel, dst_addr);
            hals_dma_periph_address_config(dma_dev->channel, src_addr);
        }
        /* configure the transfer number */
        hals_dma_transfer_number_config(dma_dev->channel, length);

        /* enable DMA channel */
        hals_dma_channel_enable(dma_dev->channel);
    } else {
        /* unlock DMA */
        HAL_UNLOCK(dma_dev);
        return HAL_ERR_BUSY;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      stop transfering amounts of data by interrupt method,the function is non-blocking
    \param[in]  dma_dev: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, HAL_ERR_ALREADY_DONE \
                details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_stop_interrupt(hal_dma_dev_struct *dma_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check DMA pointer address */
    if(NULL == dma_dev) {
        HAL_DEBUGE("pointer [dma_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(HAL_DMA_STATE_BUSY != dma_dev->state) {
        /* no transfer ongoing */
        dma_dev->error_state = HAL_DMA_ERROR_NOTRANSFER;
        /* unlock DMA */
        HAL_UNLOCK(dma_dev);

        return HAL_ERR_ALREADY_DONE;
    } else {
        /* lock DMA */
        HAL_LOCK(dma_dev);
        /* disable DMA channel */
        hals_dma_channel_disable(dma_dev->channel);
        /* disable DMA IT */
        hals_dma_interrupt_disable(dma_dev->channel, DMA_INT_ERR | \
                                   DMA_INT_HTF | DMA_INT_FTF);
        dma_dev->state = HAL_DMA_STATE_READY;
        /* reset the interrupt handle */
        dma_dev->dma_irq.error_handle = NULL;
        dma_dev->dma_irq.full_finish_handle = NULL;
        dma_dev->dma_irq.half_finish_handle = NULL;
        /* clear all flags */
        hals_dma_flag_clear(dma_dev->channel, DMA_INTF_GIF);
        /* unlock DMA */
        HAL_UNLOCK(dma_dev);
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      enable the DMA channels remapping
    \param[in]  dma_remap: specify the DMA channels to remap
                one or more parameters can be selected which are shown as below:
      \arg        DMA_REMAP_TIMER16_CH0_UP: remap TIMER16 channel0 and UP DMA requests to channel1(defaut channel0)
      \arg        DMA_REMAP_TIMER15_CH0_UP: remap TIMER15 channel2 and UP DMA requests to channel3(defaut channel2)
      \arg        DMA_REMAP_USART0_RX: remap USART0 Rx DMA request to channel4(default channel2)
      \arg        DMA_REMAP_USART0_TX: remap USART0 Tx DMA request to channel3(default channel1)
      \arg        DMA_REMAP_ADC: remap ADC DMA requests from channel0 to channel1
    \param[out] none
    \retval     int32_t: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_channel_remap_enable(uint32_t dma_remap){

    #if (1 == HAL_PARAMETER_CHECK)
        /* check syscfg_dma_remap parameter */
        if((DMA_REMAP_TIMER16_CH0_UP != dma_remap)  && \
        (DMA_REMAP_TIMER15_CH0_UP != dma_remap)  && \
        (DMA_REMAP_USART0_RX != dma_remap) && \
        (DMA_REMAP_USART0_TX != dma_remap) && \
        (DMA_REMAP_ADC != dma_remap)){
            HAL_DEBUGE("parameter [dma_remap] value is invalid");
            return HAL_ERR_VAL;
        }
    #endif /* 1 == HAL_PARAMETER_CHECK */

    SYSCFG_CFG0 |= dma_remap;

    return HAL_ERR_NONE;
}

/*!
    \brief      disable the DMA channels remapping
    \param[in]  dma_remap: specify the DMA channels to remap
                one or more parameters can be selected which are shown as below:
      \arg        DMA_REMAP_TIMER16_CH0_UP: remap TIMER16 channel0 and UP DMA requests to channel1(defaut channel0)
      \arg        DMA_REMAP_TIMER15_CH0_UP: remap TIMER15 channel2 and UP DMA requests to channel3(defaut channel2)
      \arg        DMA_REMAP_USART0_RX: remap USART0 Rx DMA request to channel4(default channel2)
      \arg        DMA_REMAP_USART0_TX: remap USART0 Tx DMA request to channel3(default channel1)
      \arg        DMA_REMAP_ADC: remap ADC DMA requests from channel0 to channel1
    \param[out] none
    \retval     int32_t: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dma_channel_remap_disable(uint32_t dma_remap){

    #if (1 == HAL_PARAMETER_CHECK)
        /* check syscfg_dma_remap parameter */
        if((DMA_REMAP_TIMER16_CH0_UP != dma_remap)  && \
        (DMA_REMAP_TIMER15_CH0_UP != dma_remap)  && \
        (DMA_REMAP_USART0_RX != dma_remap) && \
        (DMA_REMAP_USART0_TX != dma_remap) && \
        (DMA_REMAP_ADC != dma_remap)){
            HAL_DEBUGE("parameter [dma_remap] value is invalid");
            return HAL_ERR_VAL;
        }
    #endif /* 1 == HAL_PARAMETER_CHECK */

    SYSCFG_CFG0 &= ~dma_remap;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      enable DMA circulation mode
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[out] none
    \retval     none
*/
void hals_dma_circulation_enable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) |= DMA_CHXCTL_CMEN;
}

/*!
    \brief      disable DMA circulation mode
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[out] none
    \retval     none
*/
void hals_dma_circulation_disable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) &= ~DMA_CHXCTL_CMEN;
}

/*!
    \brief      enable memory to memory mode
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[out] none
    \retval     none
*/
void hals_dma_memory_to_memory_enable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) |= DMA_CHXCTL_M2M;
}

/*!
    \brief      disable memory to memory mode
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[out] none
    \retval     none
*/
void hals_dma_memory_to_memory_disable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) &= ~DMA_CHXCTL_M2M;
}

/*!
    \brief      enable DMA channel
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[out] none
    \retval     none
*/
void hals_dma_channel_enable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) |= DMA_CHXCTL_CHEN;
}

/*!
    \brief      disable DMA channel
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[out] none
    \retval     none
*/
void hals_dma_channel_disable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) &= ~DMA_CHXCTL_CHEN;
}

/*!
    \brief      set DMA peripheral base address
    \param[in]  channelx: specify which DMA channel to set peripheral base address
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  address: peripheral base address
    \param[out] none
    \retval     none
*/
void hals_dma_periph_address_config(dma_channel_enum channelx, uint32_t address)
{
    DMA_CHPADDR(channelx) = address;
}

/*!
    \brief      set DMA memory base address
    \param[in]  channelx: specify which DMA channel to set memory base address
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  address: memory base address
    \param[out] none
    \retval     none
*/
void hals_dma_memory_address_config(dma_channel_enum channelx, uint32_t address)
{
    DMA_CHMADDR(channelx) = address;
}

/*!
    \brief      set the number of remaining data to be transferred by the DMA
    \param[in]  channelx: specify which DMA channel to set number
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  number: the number of remaining data to be transferred by the DMA
    \param[out] none
    \retval     none
*/
void hals_dma_transfer_number_config(dma_channel_enum channelx, uint16_t number)
{
    DMA_CHCNT(channelx) = (number & DMA_CHANNEL_CNT_MASK);
}

/*!
    \brief      get the number of remaining data to be transferred by the DMA
    \param[in]  channelx: specify which DMA channel to set number
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[out] none
    \retval     the number of remaining data to be transferred by the DMA
*/
uint32_t hals_dma_transfer_number_get(dma_channel_enum channelx)
{
    return (uint32_t)DMA_CHCNT(channelx);
}

/*!
    \brief      configure priority level of DMA channel
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  priority: priority level of this channel
                only one parameter can be selected which is shown as below:
      \arg        DMA_PRIORITY_LEVEL_LOW: low priority
      \arg        DMA_PRIORITY_LEVEL_MEDIUM: medium priority
      \arg        DMA_PRIORITY_LEVEL_HIGH: high priority
      \arg        DMA_PRIORITY_LEVEL_ULTRA_HIGH: ultra high priority
    \param[out] none
    \retval     none
*/
void hals_dma_priority_config(dma_channel_enum channelx, uint32_t priority)
{
    uint32_t ctl;

    /* acquire DMA_CHxCTL register */
    ctl = DMA_CHCTL(channelx);
    /* assign regiser */
    ctl &= ~DMA_CHXCTL_PRIO;
    ctl |= priority;
    DMA_CHCTL(channelx) = ctl;
}

/*!
    \brief      configure transfer data width of memory
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  mwidth: transfer data width of memory
                only one parameter can be selected which is shown as below:
      \arg        DMA_MEMORY_SIZE_8BITS: transfer data width of memory is 8-bit
      \arg        DMA_MEMORY_SIZE_16BITS: transfer data width of memory is 16-bit
      \arg        DMA_MEMORY_SIZE_32BITS: transfer data width of memory is 32-bit
    \param[out] none
    \retval     none
*/
void hals_dma_memory_width_config(dma_channel_enum channelx, uint32_t mwidth)
{
    uint32_t ctl;

    /* acquire DMA_CHxCTL register */
    ctl = DMA_CHCTL(channelx);
    /* assign regiser */
    ctl &= ~DMA_CHXCTL_MWIDTH;
    ctl |= mwidth;
    DMA_CHCTL(channelx) = ctl;
}

/*!
    \brief      configure transfer data width of peripheral
    \param[in]  channelx: specify which DMA channel
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  pwidth: transfer data width of peripheral
                only one parameter can be selected which is shown as below:
      \arg        DMA_PERIPH_SIZE_8BITS: transfer data width of peripheral is 8-bit
      \arg        DMA_PERIPH_SIZE_16BITS: transfer data width of peripheral is 16-bit
      \arg        DMA_PERIPH_SIZE_32BITS: transfer data width of peripheral is 32-bit
    \param[out] none
    \retval     none
*/
void hasl_dma_periph_width_config(dma_channel_enum channelx, uint32_t pwidth)
{
    uint32_t ctl;

    /* acquire DMA_CHxCTL register */
    ctl = DMA_CHCTL(channelx);
    /* assign regiser */
    ctl &= ~DMA_CHXCTL_PWIDTH;
    ctl |= pwidth;
    DMA_CHCTL(channelx) = ctl;
}

/*!
    \brief      enable next address increasement algorithm of memory
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[out] none
    \retval     none
*/
void hals_dma_memory_increase_enable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) |= DMA_CHXCTL_MNAGA;
}

/*!
    \brief      disable next address increasement algorithm of memory
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[out] none
    \retval     none
*/
void dma_memory_increase_disable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) &= ~DMA_CHXCTL_MNAGA;
}

/*!
    \brief      enable next address increasement algorithm of peripheral
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[out] none
    \retval     none
*/
void hals_dma_periph_increase_enable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) |= DMA_CHXCTL_PNAGA;
}

/*!
    \brief      disable next address increasement algorithm of peripheral
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[out] none
    \retval     none
*/
void dma_periph_increase_disable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) &= ~DMA_CHXCTL_PNAGA;
}

/*!
    \brief      configure the direction of data transfer on the channel
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  direction: specify the direction of  data transfer
                only one parameter can be selected which is shown as below:
      \arg        DMA_PERIPHERAL_TO_MEMORY: read from peripheral and write to memory
      \arg        DMA_MEMORY_TO_PERIPHERAL: read from memory and write to peripheral
    \param[out] none
    \retval     none
*/
void hals_dma_transfer_direction_config(dma_channel_enum channelx, uint32_t direction)
{
    if(DMA_DIR_PERIPH_TO_MEMORY == direction) {
        DMA_CHCTL(channelx) &= ~DMA_CHXCTL_DIR;
    } else {
        DMA_CHCTL(channelx) |= DMA_CHXCTL_DIR;
    }
}

/*!
    \brief      check DMA flag is set or not
    \param[in]  channelx: specify which DMA channel to get flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  flag: specify get which flag
                only one parameter can be selected which is shown as below:
      \arg        HAL_DMA_FLAG_G: global interrupt flag of channel
      \arg        HAL_DMA_FLAG_FTF: full transfer finish flag of channel
      \arg        HAL_DMA_FLAG_HTF: half transfer finish flag of channel
      \arg        HAL_DMA_FLAG_ERR: error flag of channel
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_dma_flag_get(dma_channel_enum channelx, uint32_t flag)
{
    FlagStatus reval;

    if(RESET != (DMA_INTF & DMA_FLAG_ADD(flag, channelx))) {
        reval = SET;
    } else {
        reval = RESET;
    }

    return reval;
}

/*!
    \brief      clear DMA a channel flag
    \param[in]  channelx: specify which DMA channel to clear flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  flag: specify get which flag
                only one parameter can be selected which is shown as below:
      \arg        HAL_DMA_FLAG_G: global interrupt flag of channel
      \arg        HAL_DMA_FLAG_FTF: full transfer finish flag of channel
      \arg        HAL_DMA_FLAG_HTF: half transfer finish flag of channel
      \arg        HAL_DMA_FLAG_ERR: error flag of channel
    \param[out] none
    \retval     none
*/
void hals_dma_flag_clear(dma_channel_enum channelx, uint32_t flag)
{
    DMA_INTC |= DMA_FLAG_ADD(flag, channelx);
}

/*!
    \brief      enable DMA interrupt
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  source: specify which interrupt to enable
                only one parameter can be selected which is shown as below:
      \arg        DMA_INT_ERR: channel error interrupt
      \arg        DMA_INT_HTF: channel half transfer finish interrupt
      \arg        DMA_INT_FTF: channel full transfer finish interrupt
    \param[out] none
    \retval     none
*/
void hals_dma_interrupt_enable(dma_channel_enum channelx, uint32_t source)
{
    DMA_CHCTL(channelx) |= source;
}

/*!
    \brief      disable DMA interrupt
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  source: specify which interrupt to disable
                only one parameter can be selected which is shown as below:
      \arg        DMA_INT_ERR: channel error interrupt
      \arg        DMA_INT_HTF: channel half transfer finish interrupt
      \arg        DMA_INT_FTF: channel full transfer finish interrupt
    \param[out] none
    \retval     none
*/
void hals_dma_interrupt_disable(dma_channel_enum channelx, uint32_t source)
{
    DMA_CHCTL(channelx) &= ~source;
}

/*!
    \brief      check DMA flag and interrupt enable bit is set or not
    \param[in]  channelx: specify which DMA channel to get flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  flag: specify get which flag
                only one parameter can be selected which is shown as below:
      \arg        HAL_DMA_INT_FLAG_FTF: transfer finish flag of channel
      \arg        HAL_DMA_INT_FLAG_HTF: half transfer finish flag of channel
      \arg        HAL_DMA_INT_FLAG_ERR: error flag of channel
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_dma_interrupt_flag_get(dma_channel_enum channelx, uint32_t flag)
{
    uint32_t interrupt_enable = 0U, interrupt_flag = 0U;

    switch(flag) {
    case DMA_INTF_FTFIF:
        interrupt_flag = DMA_INTF & DMA_FLAG_ADD(flag, channelx);
        interrupt_enable = DMA_CHCTL(channelx) & DMA_CHXCTL_FTFIE;
        break;
    case DMA_INTF_HTFIF:
        interrupt_flag = DMA_INTF & DMA_FLAG_ADD(flag, channelx);
        interrupt_enable = DMA_CHCTL(channelx) & DMA_CHXCTL_HTFIE;
        break;
    case DMA_INTF_ERRIF:
        interrupt_flag = DMA_INTF & DMA_FLAG_ADD(flag, channelx);
        interrupt_enable = DMA_CHCTL(channelx) & DMA_CHXCTL_ERRIE;
        break;
    default:
        break;
    }

    if(interrupt_flag && interrupt_enable) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear DMA a channel interrupt flag
    \param[in]  channelx: specify which DMA channel to clear flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..6)
    \param[in]  flag: specify get which flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_INT_FLAG_G: global interrupt flag of channel
      \arg        HAL_DMA_INT_FLAG_FTF: transfer finish flag of channel
      \arg        HAL_DMA_INT_FLAG_HTF: half transfer finish flag of channel
      \arg        HAL_DMA_INT_FLAG_ERR: error flag of channel
    \param[out] none
    \retval     none
*/
void hals_dma_interrupt_flag_clear(dma_channel_enum channelx, uint32_t flag)
{
    DMA_INTC |= DMA_FLAG_ADD(flag, channelx);
}
