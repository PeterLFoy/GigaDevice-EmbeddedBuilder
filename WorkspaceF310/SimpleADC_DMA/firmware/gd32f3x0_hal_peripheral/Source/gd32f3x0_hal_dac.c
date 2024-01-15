/*!
    \file    gd32f3x0_hal_dac.c
    \brief   DAC driver

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

#ifdef GD32F350
#include "gd32f3x0_hal.h"

/* DAC DMA full transmission complete callback */
static void _dac_dma_full_transfer_complete(void *dma);
/* DAC DMA half transmission complete callback */
static void _dac_dma_half_transfer_complete(void *dma);
/* DAC DMA error callback */
static void _dac_dma_error(void *dma);


/*!
    \brief      initialize DAC
    \param[in]  dac_dev: DAC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  dac: the pointer of DAC init structure
                  output_waveform_selection:
                  only one parameter can be selected which is shown as below:
      \arg          DAC_WAVE_DISABLE: wave disable
      \arg          DAC_WAVE_MODE_LFSR: LFSR noise mode
      \arg          DAC_WAVE_MODE_TRIANGLE: triangle noise mode
                  noise_wave_bit_width:
                  only one parameter can be selected which is shown as below:
      \arg          DAC_WAVE_BIT_WIDTH_1: bit width of the wave signal is 1
      \arg          DAC_WAVE_BIT_WIDTH_2: bit width of the wave signal is 2
      \arg          DAC_WAVE_BIT_WIDTH_3: bit width of the wave signal is 3
      \arg          DAC_WAVE_BIT_WIDTH_4: bit width of the wave signal is 4
      \arg          DAC_WAVE_BIT_WIDTH_5: bit width of the wave signal is 5
      \arg          DAC_WAVE_BIT_WIDTH_6: bit width of the wave signal is 6
      \arg          DAC_WAVE_BIT_WIDTH_7: bit width of the wave signal is 7
      \arg          DAC_WAVE_BIT_WIDTH_8: bit width of the wave signal is 8
      \arg          DAC_WAVE_BIT_WIDTH_9: bit width of the wave signal is 9
      \arg          DAC_WAVE_BIT_WIDTH_10: bit width of the wave signal is 10
      \arg          DAC_WAVE_BIT_WIDTH_11: bit width of the wave signal is 11
      \arg          DAC_WAVE_BIT_WIDTH_12: bit width of the wave signal is 12
                trigger_enable:
                only one parameter can be selected which is shown as below:
      \arg          ENABLE: enable trigger mode
      \arg          DISABLE: disable trigger mode
                trigger_selection:
                only one parameter can be selected which is shown as below:
      \arg          DAC_TRIGGER_T5_TRGO: TIMER5 TRGO
      \arg          DAC_TRIGGER_T2_TRGO: TIMER2 TRGO
      \arg          DAC_TRIGGER_T14_TRGO: TIMER14 TRGO
      \arg          DAC_TRIGGER_T1_TRGO: TIMER1 TRGO
      \arg          DAC_TRIGGER_EXTI_9: EXTI interrupt line9 event
      \arg          DAC_TRIGGER_SOFTWARE: software trigger
                aligned_mode:
                only one parameter can be selected which is shown as below:
      \arg          DAC_R12_ALIGNED_MODE: 12-bit right-aligned
      \arg          DAC_L12_ALIGNED_MODE: 12-bit left-aligned
      \arg          DAC_R8_ALIGNED_MODE: 8-bit right-aligned
                output_value: output_value to be loaded
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dac_init(hal_dac_dev_struct *dac_dev, hal_dac_init_struct *dac)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == dac_dev) || (NULL == dac)) {
        HAL_DEBUGE("pointer [*dac_dev] or pointer [*dac] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    dac_dev->state = DAC_STATE_BUSY;

    /* configure DAC wave mode */
    hals_dac_wave_mode_config(dac->output_waveform_selection);
    /* configure DAC wave bit width */
    hals_dac_wave_bit_width_config(dac->noise_wave_bit_width);
    /* enable or disable DAC output buffer */
    hals_dac_output_buffer_config(dac->output_buffer_enable);
    /* enable or disable DAC trigger */
    hals_dac_trigger_config(dac->trigger_enable);
    /* configure DAC trigger source */
    hals_dac_trigger_source_config(dac->trigger_selection);
    /* set DAC data holding register value */
    hals_dac_output_value_set(dac->aligned_mode, dac->output_value);
    /* change DAC error state */
    dac_dev->error_state = DAC_ERROR_NONE;
    /* change DAC state */
    dac_dev->state = DAC_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      deinitialize DAC device structure and init structure
    \param[in]  dac_dev: DAC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dac_deinit(hal_dac_dev_struct *dac_dev)
{

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == dac_dev) {
        HAL_DEBUGE("pointer [dac_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    dac_dev->state = DAC_STATE_BUSY;
    /* deinit DAC */
    hal_rcu_periph_reset_enable(RCU_DACRST);
    hal_rcu_periph_reset_enable(RCU_DACRST);

    /* change DAC error state and state */
    dac_dev->error_state = DAC_ERROR_NONE;
    dac_dev->state = DAC_STATE_RESET;
    /* change DAC state */
    dac_dev->state = DAC_STATE_READY;
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize the DAC structure with the default values
    \param[in]  hal_struct_type: the argument could be selected from enumeration <hal_dac_struct_type_enum>
    \param[in]  p_struct: pointer to DAC structure that contains the configuration information
    \param[out] none
    \retval     none
*/
void hal_dac_struct_init(hal_dac_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer [*p_struct] value is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_DAC_INIT_STRUCT:
        /* initialize DAC initialization structure with the default values */
        ((hal_dac_init_struct *)p_struct)->output_waveform_selection                        = DAC_WAVE_DISABLE;
        ((hal_dac_init_struct *)p_struct)->noise_wave_bit_width                             = DAC_WAVE_BIT_WIDTH_1;
        ((hal_dac_init_struct *)p_struct)->trigger_enable                                   = DISABLE;
        ((hal_dac_init_struct *)p_struct)->output_buffer_enable                             = DISABLE;
        ((hal_dac_init_struct *)p_struct)->trigger_selection                                = DAC_TRIGGER_T5_TRGO;
        ((hal_dac_init_struct *)p_struct)->aligned_mode                                     = DAC_R12_ALIGNED_MODE;
        ((hal_dac_init_struct *)p_struct)->output_value                                     = 0U;
        break;
    case HAL_DAC_IRQ_STRUCT:
        /* initialize DAC initialization structure with the default values */
        ((hal_dac_irq_struct *)p_struct)->dac_underflow_handle                              = NULL;
        break;
    case HAL_DAC_DMA_HANDLE_CB_STRUCT:
        /* initialize DAC initialization structure with the default values */
        ((hal_dac_dma_handle_cb_struct *)p_struct)->full_transcom_handle                    = NULL;
        ((hal_dac_dma_handle_cb_struct *)p_struct)->half_transcom_handle                    = NULL;
        ((hal_dac_dma_handle_cb_struct *)p_struct)->error_handle                            = NULL;
        break;
    case HAL_DAC_DEV_STRUCT:
        /* initialize DAC initialization structure with the default values */
        ((hal_dac_dev_struct *)p_struct)->dac_irq.dac_underflow_handle                      = NULL;
        ((hal_dac_dev_struct *)p_struct)->p_dma_dac                                         = NULL;
        ((hal_dac_dev_struct *)p_struct)->dac_dma.full_transcom_handle                      = NULL;
        ((hal_dac_dev_struct *)p_struct)->dac_dma.half_transcom_handle                      = NULL;
        ((hal_dac_dev_struct *)p_struct)->dac_dma.error_handle                              = NULL;
        ((hal_dac_dev_struct *)p_struct)->error_state                                       = DAC_ERROR_NONE;
        ((hal_dac_dev_struct *)p_struct)->state                                             = DAC_STATE_NONE;
        ((hal_dac_dev_struct *)p_struct)->mutex                                             = HAL_MUTEX_UNLOCKED;
        ((hal_dac_dev_struct *)p_struct)->priv                                              = NULL;
        break;
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      start DAC module function
    \param[in]  dac_dev: DAC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dac_start(hal_dac_dev_struct *dac_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == dac_dev) {
        HAL_DEBUGE("pointer [dac_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    dac_dev->state = DAC_STATE_BUSY;
    hals_dac_enable();

    if(SET == (DAC_CTL & (DAC_CTL_DTEN | DAC_CTL_DTSEL))) {
        hals_dac_software_trigger_enable();
    }
    dac_dev->state = DAC_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      stop DAC module function
    \param[in]  dac_dev: DAC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dac_stop(hal_dac_dev_struct *dac_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == dac_dev) {
        HAL_DEBUGE("pointer [dac_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    hals_dac_disable();

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      start DAC under error interrupt
    \param[in]  dac_dev: DAC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: DAC under error interrupt callback function structure
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dac_start_interrupt(hal_dac_dev_struct *dac_dev, hal_dac_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == dac_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [dac_dev] or pointer [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* lock DAC */
    HAL_LOCK(dac_dev);
    dac_dev->state = DAC_STATE_BUSY;
    dac_dev->dac_irq.dac_underflow_handle = p_irq->dac_underflow_handle;
    /* clear the specified DAC interrupt flag */
    hals_dac_interrupt_flag_clear();
    /*  enable DAC interrupt(DAC DMA underrun interrupt) */
    hals_dac_interrupt_enable();
    /* enable DAC */
    hals_dac_enable();
    /* unlock DAC */
    HAL_UNLOCK(dac_dev);
    /* change DAC state */
    dac_dev->state = DAC_STATE_READY;
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      stop DAC under error interrupt
    \param[in]  dac_dev: DAC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dac_stop_interrupt(hal_dac_dev_struct *dac_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == dac_dev) {
        HAL_DEBUGE("pointer [dac_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* lock DAC */
    HAL_LOCK(dac_dev);
    dac_dev->state = DAC_STATE_BUSY;
    dac_dev->dac_irq.dac_underflow_handle = NULL;
    /* clear the specified DAC interrupt flag */
    hals_dac_interrupt_flag_clear();
    /*  disable DAC interrupt(DAC DMA underrun interrupt) */
    hals_dac_interrupt_disable();
    /* unlock DAC */
    HAL_UNLOCK(dac_dev);
    /* change DAC state */
    dac_dev->state = DAC_STATE_READY;
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      start DAC DMA request
    \param[in]  dac_dev: DAC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  pdata: the source memory buffer address
    \param[in]  length: the number of data to be transferred from source to destination
    \param[in]  aligned_mode:
                only one parameter can be selected which is shown as below:
      \arg          DAC_R12_ALIGNED_MODE: 12-bit right-aligned
      \arg          DAC_L12_ALIGNED_MODE: 12-bit left-aligned
      \arg          DAC_R8_ALIGNED_MODE: 8-bit right-aligned
    \param[in]  dmacb: DAC DMA transfer complete interrupt callback function structure
                  full_transcom_handle: DAC DMA full transfer complete interrupt handler
                  half_transcom_handle: DAC DMA half transfer complete interrupt handler
                  underflow_handle: DAC DMA underflow interrupt handler
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dac_start_dma(hal_dac_dev_struct *dac_dev, uint32_t *pdata, uint32_t length, uint32_t aligned_mode, hal_dac_dma_handle_cb_struct *dmacb)
{
    volatile uint32_t *dac_value_adress;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == dac_dev) || (NULL == pdata) || (NULL == dmacb)) {
        HAL_DEBUGE("pointer [dac_dev] or pointer [pdata] or pointer [dmacb] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* lock DAC */
    HAL_LOCK(dac_dev);
    dac_dev->state = DAC_STATE_BUSY;

    dac_dev->dac_dma.full_transcom_handle = dmacb->full_transcom_handle;
    dac_dev->dac_dma.half_transcom_handle = dmacb->half_transcom_handle;
    dac_dev->dac_dma.error_handle = dmacb->error_handle;

    dac_dev->p_dma_dac->dma_irq.full_finish_handle = _dac_dma_full_transfer_complete;
    dac_dev->p_dma_dac->dma_irq.half_finish_handle = _dac_dma_half_transfer_complete;
    dac_dev->p_dma_dac->dma_irq.error_handle = _dac_dma_error;

    switch(aligned_mode) {
    /* data right 12b alignment */
    case DAC_R12_ALIGNED_MODE:
        dac_value_adress = &DAC_R12DH;
        break;
    /* data left 12b alignment */
    case DAC_L12_ALIGNED_MODE:
        dac_value_adress = &DAC_L12DH;
        break;
    /* data right 8b alignment */
    case DAC_R8_ALIGNED_MODE:
        dac_value_adress = &DAC_R8DH;
        break;
    default:
        break;
    }
    /* enable the selected DAC channel DMA request */
    hals_dac_dma_enable();
    /* start the DMA channel */
    hal_dma_start_interrupt(dac_dev->p_dma_dac, (uint32_t)pdata, (uint32_t)dac_value_adress, length, NULL);
    hals_dac_enable();
    dac_dev->state = DAC_STATE_READY;
    /* unlock DAC */
    HAL_UNLOCK(dac_dev);

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      stop DAC DMA request
    \param[in]  dac_dev: DAC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dac_stop_dma(hal_dac_dev_struct *dac_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == dac_dev) {
        HAL_DEBUGE("pointer [dac_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* disable DAC DMA mode */
    hals_dac_dma_disable();
    /* disable the DMA channel */
    hal_dma_stop(dac_dev->p_dma_dac);
    /* disable DAC */
    hals_dac_disable();
#if defined(DAC_CTL_DDUDRIE)
    /* clear interrupt flag */
    hals_dac_interrupt_flag_clear();
    /* disable interrupt */
    hals_dac_interrupt_disable();
#endif /* DAC_CR_DMAUDRIE2 */
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      DAC interrupt handler content function,which is merely used in dac_handler
    \param[in]  dac_dev: DAC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_dac_irq(hal_dac_dev_struct *dac_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == dac_dev) {
        HAL_DEBUGE("pointer [dac_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* check dac underflow interrupt control bit */
    /* check dac underflow interrupt state bit */
    if(SET == hals_dac_interrupt_flag_get()) {
        /* change DAC state to error state */
        dac_dev->state = DAC_STATE_ERROR;
        /* set DAC error state to DMA underflow error */
        dac_dev->error_state = DAC_ERROR_DMA_UNDERFLOW;
        /* clear interrupt flag */
        hals_dac_interrupt_flag_clear();

        /* error callback */
        if(NULL != (dac_dev->dac_irq.dac_underflow_handle)) {
            dac_dev->dac_irq.dac_underflow_handle(dac_dev);
        }
    }
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      set user-defined interrupt callback function
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  dac_dev: DAC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to DAC interrupt callback functions structure
                  hal_irq_handle_cb: the function is user-defined,
    \param[out] none
    \retval     none
*/
int32_t hal_dac_irq_handle_set(hal_dac_dev_struct *dac_dev, hal_dac_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == dac_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [dac_dev] or pointer [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* DAC underflow interrupt handler set */
    if(NULL != p_irq->dac_underflow_handle) {
        dac_dev->dac_irq.dac_underflow_handle = p_irq->dac_underflow_handle;
        hals_dac_interrupt_enable();
    } else {
        dac_dev->dac_irq.dac_underflow_handle = NULL;
        hals_dac_interrupt_disable();
    }
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  dac_dev: DAC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
int32_t hal_dac_irq_handle_all_reset(hal_dac_dev_struct *dac_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == dac_dev) {
        HAL_DEBUGE("pointer [dac_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* DAC underflow interrupt handler set */
    dac_dev->dac_irq.dac_underflow_handle = NULL;
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      configure DAC wave mode
    \param[in]  wave_mode
                only one parameter can be selected which is shown as below:
      \arg        DAC_WAVE_DISABLE: wave disable
      \arg        DAC_WAVE_MODE_LFSR: LFSR noise mode
      \arg        DAC_WAVE_MODE_TRIANGLE: triangle noise mode
    \param[out] none
    \retval     none
*/
void hals_dac_wave_mode_config(uint32_t wave_mode)
{
    DAC_CTL &= ~DAC_CTL_DWM;
    DAC_CTL |= wave_mode;
}

/*!
    \brief      configure DAC wave bit width
    \param[in]  bit_width
                only one parameter can be selected which is shown as below:
      \arg        DAC_WAVE_BIT_WIDTH_1: bit width of the wave signal is 1
      \arg        DAC_WAVE_BIT_WIDTH_2: bit width of the wave signal is 2
      \arg        DAC_WAVE_BIT_WIDTH_3: bit width of the wave signal is 3
      \arg        DAC_WAVE_BIT_WIDTH_4: bit width of the wave signal is 4
      \arg        DAC_WAVE_BIT_WIDTH_5: bit width of the wave signal is 5
      \arg        DAC_WAVE_BIT_WIDTH_6: bit width of the wave signal is 6
      \arg        DAC_WAVE_BIT_WIDTH_7: bit width of the wave signal is 7
      \arg        DAC_WAVE_BIT_WIDTH_8: bit width of the wave signal is 8
      \arg        DAC_WAVE_BIT_WIDTH_9: bit width of the wave signal is 9
      \arg        DAC_WAVE_BIT_WIDTH_10: bit width of the wave signal is 10
      \arg        DAC_WAVE_BIT_WIDTH_11: bit width of the wave signal is 11
      \arg        DAC_WAVE_BIT_WIDTH_12: bit width of the wave signal is 12
    \param[out] none
    \retval     none
*/
void hals_dac_wave_bit_width_config(uint32_t bit_width)
{
    DAC_CTL &= ~DAC_CTL_DWBW;
    DAC_CTL |= bit_width;
}

/*!
    \brief      enable or disable DAC trigger
    \param[in]  trigger_enable: enable DAC trigger
                only one parameter can be selected which is shown as below:
      \arg        ENABLE: trigger enable
      \arg        DISABLE: trigger disable
    \param[out] none
    \retval     none
*/
void hals_dac_trigger_config(uint32_t trigger_enable)
{
    if(trigger_enable == ENABLE) {
        DAC_CTL |= DAC_CTL_DTEN;
    } else {
        DAC_CTL &= ~DAC_CTL_DTEN;
    }
}

/*!
    \brief      enable or disable DAC output buffer
    \param[in]  output_buffer: enable DAC output buffer
                only one parameter can be selected which is shown as below:
      \arg        ENABLE: output buffer enable
      \arg        DISABLE: output buffer disable
    \param[out] none
    \retval     none
*/
void hals_dac_output_buffer_config(uint32_t output_buffer)
{
    if(output_buffer == ENABLE) {
        DAC_CTL &= ~DAC_CTL_DBOFF;
    } else {
        DAC_CTL |= DAC_CTL_DBOFF;
    }
}

/*!
    \brief      configure DAC trigger source
    \param[in]  triggersource: external triggers of DAC
                only one parameter can be selected which is shown as below:
      \arg        DAC_TRIGGER_T1_TRGO: trigger source is TIMER1 TRGO
      \arg        DAC_TRIGGER_T2_TRGO: trigger source is TIMER2 TRGO
      \arg        DAC_TRIGGER_T5_TRGO: trigger source is TIMER5 TRGO
      \arg        DAC_TRIGGER_T14_TRGO: trigger source is TIMER14 TRGO
      \arg        DAC_TRIGGER_EXTI_9: trigger source is EXTI interrupt line9 event
      \arg        DAC_TRIGGER_SOFTWARE: software trigger
    \param[out] none
    \retval     none
*/
void hals_dac_trigger_source_config(uint32_t triggersource)
{
    DAC_CTL &= ~DAC_CTL_DTSEL;
    DAC_CTL |= triggersource;
}

/*!
    \brief      enable DAC software trigger
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_dac_software_trigger_enable(void)
{
    DAC_SWT |= DAC_SWT_SWTR;
}

/*!
    \brief      disable DAC software trigger
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_dac_software_trigger_disable(void)
{
    DAC_SWT &= ~DAC_SWT_SWTR;
}

/*!
    \brief      set DAC data holding register value
    \param[in]  dac_align
                only one parameter can be selected which is shown as below:
      \arg        DAC_R12_ALIGNED_MODE: data right 12b alignment
      \arg        DAC_L12_ALIGNED_MODE: data left 12b alignment
      \arg        DAC_R8_ALIGNED_MODE: data right 8b alignment
    \param[in]  data: data to be loaded
    \param[out] none
    \retval     none
*/
void hals_dac_output_value_set(uint32_t dac_align, uint16_t data)
{
    switch(dac_align) {
    /* data right 12b alignment */
    case DAC_R12_ALIGNED_MODE:
        DAC_R12DH = data;
        break;
    /* data left 12b alignment */
    case DAC_L12_ALIGNED_MODE:
        DAC_L12DH = data << 4;
        break;
    /* data right 8b alignment */
    case DAC_R8_ALIGNED_MODE:
        DAC_R8DH = data;
        break;
    default:
        break;
    }
}

/*!
    \brief      enable DAC
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_dac_enable(void)
{
    DAC_CTL |= DAC_CTL_DEN;
}

/*!
    \brief      disable DAC
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_dac_disable(void)
{
    DAC_CTL &= ~DAC_CTL_DEN;
}

/*!
    \brief      enable DAC DMA
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_dac_dma_enable(void)
{
    DAC_CTL |= DAC_CTL_DDMAEN;
}

/*!
    \brief      disable DAC DMA
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_dac_dma_disable(void)
{
    DAC_CTL &= ~DAC_CTL_DDMAEN;
}

/*!
    \brief      enable DAC interrupt(DAC DMA underrun interrupt)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_dac_interrupt_enable(void)
{
    DAC_CTL |= DAC_CTL_DDUDRIE;
}

/*!
    \brief      disable DAC interrupt(DAC DMA underrun interrupt)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_dac_interrupt_disable(void)
{
    DAC_CTL &= ~DAC_CTL_DDUDRIE;
}

/*!
    \brief      get the specified DAC interrupt flag(DAC DMA underrun interrupt flag)
    \param[in]  none
    \param[out] none
    \retval     the state of DAC interrupt flag(SET or RESET)
*/
FlagStatus hals_dac_interrupt_flag_get(void)
{
    FlagStatus temp_flag = RESET;
    uint32_t ddudr_flag = 0U, ddudrie_flag = 0U;
    /* check the DMA underrun flag and DAC DMA underrun interrupt enable flag */
    ddudr_flag = DAC_STAT & DAC_STAT_DDUDR;
    ddudrie_flag = DAC_CTL & DAC_CTL_DDUDRIE;
    if((RESET != ddudr_flag) && (RESET != ddudrie_flag)) {
        temp_flag = SET;
    }
    return temp_flag;
}

/*!
    \brief      clear the specified DAC interrupt flag(DAC DMA underrun interrupt flag)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_dac_interrupt_flag_clear(void)
{
    DAC_STAT |= DAC_STAT_DDUDR;
}

/*!
    \brief      DAC DMA full transmission complete callback
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _dac_dma_full_transfer_complete(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_dac_dev_struct *dac_dev;
    p_dma = (hal_dma_dev_struct *)dma;
    dac_dev = (hal_dac_dev_struct *)(p_dma->p_periph);

    if(NULL != (dac_dev->dac_dma.full_transcom_handle)) {
        dac_dev->dac_dma.full_transcom_handle(dac_dev);
    }
    dac_dev->state = DAC_STATE_READY;
}

/*!
    \brief      DAC DMA full transmission complete callback
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _dac_dma_half_transfer_complete(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_dac_dev_struct *dac_dev;
    p_dma = (hal_dma_dev_struct *)dma;
    dac_dev = (hal_dac_dev_struct *)(p_dma->p_periph);

    if(NULL != (dac_dev->dac_dma.half_transcom_handle)) {
        dac_dev->dac_dma.half_transcom_handle(dac_dev);
    }
    dac_dev->state = DAC_STATE_READY;
}

/*!
    \brief      DAC DMA underflow error callback
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _dac_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_dac_dev_struct *dac_dev;
    p_dma = (hal_dma_dev_struct *)dma;
    dac_dev = (hal_dac_dev_struct *)(p_dma->p_periph);

    if(NULL != (dac_dev->dac_dma.error_handle)) {
        dac_dev->dac_dma.error_handle(dac_dev);
    }
    dac_dev->state = DAC_STATE_READY;
}
#endif /* GD32F350 */
