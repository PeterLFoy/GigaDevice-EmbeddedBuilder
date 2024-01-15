/*!
    \file    gd32f3x0_hal_ctc.c
    \brief   CTC driver

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
#define CTC_FLAG_MASK            ((uint32_t)0x00000700U)

/*!
    \brief      reset CTC peripheral
    \param[in]  ctc_dev: CTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_ctc_deinit(hal_ctc_dev_struct *ctc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == ctc_dev) {
        HAL_DEBUGE("pointer [*ctc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    ctc_dev->state = HAL_CTC_STATE_BUSY;
    /* reset CTC */
    hal_rcu_periph_reset_enable(RCU_CTCRST);
    hal_rcu_periph_reset_disable(RCU_CTCRST);

    /* change CTC error state and state */
    ctc_dev->error_state = HAL_CTC_ERROR_NONE;
    ctc_dev->state = HAL_CTC_STATE_RESET;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize CTC init structure
    \param[in]  ctc_dev: CTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] ctc_init: initialize the CTC parameter struct
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_ctc_init(hal_ctc_dev_struct *ctc_dev, hal_ctc_init_struct *ctc_init)
{
    uint16_t freq_value = 0U;
    uint8_t temp_cklim_value = 0U;
    uint16_t temp_reload_value = 0x00;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == ctc_dev) || (NULL == ctc_init)) {
        HAL_DEBUGE("pointer [ctc_dev] or pointer [ctc_init] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    ctc_dev->state = HAL_CTC_STATE_BUSY;
    if(ctc_init->source == CTC_REFSOURCE_LXTAL) {
        ctc_init->frequency = 32768;
    }
    if(ctc_init->cal_style == CTC_VALUE_AUTO_CAL) {
        switch(ctc_init->prescaler) {
        case CTC_REFSOURCE_PSC_OFF:
            freq_value = ctc_init->frequency;
            temp_reload_value = CRS_CAL_RELOADVALUE(48000000, freq_value);
            temp_cklim_value = CRS_CAL_CKLIM(48000000, freq_value);
            break;
        case CTC_REFSOURCE_PSC_DIV2:
            freq_value = (int)ctc_init->frequency / 2.0;
            temp_reload_value = CRS_CAL_RELOADVALUE(48000000, freq_value);
            temp_cklim_value = CRS_CAL_CKLIM(48000000, freq_value);
            break;
        case CTC_REFSOURCE_PSC_DIV4:
            freq_value = (int)ctc_init->frequency / 4.0;
            temp_reload_value = CRS_CAL_RELOADVALUE(48000000, freq_value);
            temp_cklim_value = CRS_CAL_CKLIM(48000000, freq_value);
            break;
        case CTC_REFSOURCE_PSC_DIV8:
            freq_value = (int)ctc_init->frequency / 8.0;
            temp_reload_value = CRS_CAL_RELOADVALUE(48000000, freq_value);
            temp_cklim_value = CRS_CAL_CKLIM(48000000, freq_value);
            break;
        case CTC_REFSOURCE_PSC_DIV16:
            freq_value = (int)ctc_init->frequency / 16.0;
            temp_reload_value = CRS_CAL_RELOADVALUE(48000000, freq_value);
            temp_cklim_value = CRS_CAL_CKLIM(48000000, freq_value);
            break;
        case CTC_REFSOURCE_PSC_DIV32:
            freq_value = (int)ctc_init->frequency / 32.0;
            temp_reload_value = CRS_CAL_RELOADVALUE(48000000, freq_value);
            temp_cklim_value = CRS_CAL_CKLIM(48000000, freq_value);
            break;
        case CTC_REFSOURCE_PSC_DIV64:
            freq_value = (int)ctc_init->frequency / 64.0;
            temp_reload_value = CRS_CAL_RELOADVALUE(48000000, freq_value);
            temp_cklim_value = CRS_CAL_CKLIM(48000000, freq_value);
            break;
        case CTC_REFSOURCE_PSC_DIV128:
            freq_value = (int)ctc_init->frequency / 128.0;
            temp_reload_value = CRS_CAL_RELOADVALUE(48000000, freq_value);
            temp_cklim_value = CRS_CAL_CKLIM(48000000, freq_value);
            break;
        default:
            HAL_DEBUGW("parameter [ctc_init->prescaler] value is undefine");
            break;
        }

        /* configure clock trim base limit value */
        hals_ctc_clock_limit_value_config(temp_cklim_value);
        /* configure CTC counter reload value */
        hals_ctc_counter_reload_value_config(temp_reload_value);
    } else {
        /* configure clock trim base limit value */
        hals_ctc_clock_limit_value_config(ctc_init->limit_value);
        /* configure CTC counter reload value */
        hals_ctc_counter_reload_value_config(ctc_init->reload_value);
    }

    /* select reference signal source */
    hals_ctc_refsource_signal_select(ctc_init->source);
    /* configure reference signal source polarity */
    hals_ctc_refsource_polarity_config(ctc_init->polarity);
    /* configure reference signal source prescaler */
    hals_ctc_refsource_prescaler_config(ctc_init->prescaler);
    /* configure hardware automatically trim mode */
    hals_ctc_hardware_trim_mode_config(CTC_HARDWARE_TRIM_MODE_ENABLE);

    /* change CTC error state and state */
    ctc_dev->error_state = HAL_CTC_ERROR_NONE;
    ctc_dev->state = HAL_CTC_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize the CTC device structure with the default values
    \param[in]  hal_struct_type: the argument could be selected from enumeration <hal_ctc_struct_type_enum>
    \param[in]  p_struct: pointer to CTC structure that contains the configuration information
    \param[out] none
    \retval     none
*/
void hal_ctc_struct_init(hal_ctc_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer [p_struct] value is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_CTC_INIT_STRUCT:
        /* initialize CTC initialization structure with the default values */
        ((hal_ctc_init_struct *)p_struct)->cal_style     = CTC_VALUE_AUTO_CAL;
        ((hal_ctc_init_struct *)p_struct)->source        = CTC_REFSOURCE_LXTAL;
        ((hal_ctc_init_struct *)p_struct)->polarity      = CTC_REFSOURCE_POLARITY_FALLING;
        ((hal_ctc_init_struct *)p_struct)->prescaler     = CTC_REFSOURCE_PSC_OFF;
        ((hal_ctc_init_struct *)p_struct)->limit_value   = 34;
        ((hal_ctc_init_struct *)p_struct)->reload_value  = 0;
        ((hal_ctc_init_struct *)p_struct)->frequency  = 1;
        break;
    case HAL_CTC_IRQ_STRUCT:
        /* initialize CTC IRQ structure with the default values */
        ((hal_ctc_irq_struct *)p_struct)->ctc_ckok_handle             = NULL;
        ((hal_ctc_irq_struct *)p_struct)->ctc_ckwarn_handle           = NULL;
        ((hal_ctc_irq_struct *)p_struct)->ctc_err_handle              = NULL;
        ((hal_ctc_irq_struct *)p_struct)->ctc_eref_handle             = NULL;
        break;
    case HAL_CTC_DEV_STRUCT:
        /* initialize CTC device structure with the default values */
        ((hal_ctc_dev_struct *)p_struct)->ctc_irq.ctc_ckok_handle    = NULL;
        ((hal_ctc_dev_struct *)p_struct)->ctc_irq.ctc_ckwarn_handle  = NULL;
        ((hal_ctc_dev_struct *)p_struct)->ctc_irq.ctc_err_handle     = NULL;
        ((hal_ctc_dev_struct *)p_struct)->ctc_irq.ctc_eref_handle    = NULL;
        ((hal_ctc_dev_struct *)p_struct)->error_state                = HAL_CTC_ERROR_NONE;
        ((hal_ctc_dev_struct *)p_struct)->state                      = HAL_CTC_STATE_NONE;
        ((hal_ctc_dev_struct *)p_struct)->mutex                      = HAL_MUTEX_UNLOCKED;
        ((hal_ctc_dev_struct *)p_struct)->priv                       = NULL;
        break;
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      start CTC peripheral
    \param[in]  ctc_dev: CTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_ctc_start(hal_ctc_dev_struct *ctc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == ctc_dev) {
        HAL_DEBUGE("pointer [ctc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    ctc_dev->state = HAL_CTC_STATE_BUSY;

    /* enable CTC trim counter */
    hals_ctc_counter_enable();

    ctc_dev->state = HAL_CTC_STATE_READY;
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      stop CTC peripheral
    \param[in]  ctc_dev: CTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_ctc_stop(hal_ctc_dev_struct *ctc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == ctc_dev) {
        HAL_DEBUGE("pointer [ctc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    ctc_dev->state = HAL_CTC_STATE_BUSY;

    /* disable CTC trim counter */
    hals_ctc_counter_disable();

    ctc_dev->state = HAL_CTC_STATE_READY;
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      configure the IRC48M trim value
    \param[in]  ctc_dev: CTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  trim_value: 6-bit IRC48M trim value
      \arg        0x00-0x3F
    \param[out] none
    \retval     none
*/
void hal_ctc_irc48m_trim_value_config(hal_ctc_dev_struct *ctc_dev, uint8_t trim_value)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == ctc_dev) {
        HAL_DEBUGE("pointer [*ctc_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    hals_ctc_irc48m_trim_value_config(trim_value);
}

/*!
    \brief      CTC interrupt handler content function,which is merely used in ctc_handler
    \param[in]  ctc_dev: CTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_ctc_irq(hal_ctc_dev_struct *ctc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == ctc_dev) {
        HAL_DEBUGE("pointer [ctc_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* check interrupt state bit */
    if(SET == hals_ctc_interrupt_flag_get(CTC_INT_FLAG_CKOK)) {
        /* clear interrupt flag */
        hals_ctc_interrupt_flag_clear(CTC_INT_FLAG_CKOK);
        /* ok callback */
        if(NULL != (ctc_dev->ctc_irq.ctc_ckok_handle)) {
            ctc_dev->ctc_irq.ctc_ckok_handle(ctc_dev);
        }
    }

    /* check interrupt state bit */
    if(SET == hals_ctc_interrupt_flag_get(CTC_INT_FLAG_CKWARN)) {
        /* clear interrupt flag */
        hals_ctc_interrupt_flag_clear(CTC_INT_FLAG_CKWARN);
        /* warning callback */
        if(NULL != (ctc_dev->ctc_irq.ctc_ckwarn_handle)) {
            ctc_dev->ctc_irq.ctc_ckwarn_handle(ctc_dev);
        }
    }

    /* check interrupt state bit */
    if(SET == hals_ctc_interrupt_flag_get(CTC_INT_FLAG_ERR)) {
        /* clear interrupt flag */
        hals_ctc_interrupt_flag_clear(CTC_INT_FLAG_ERR);
        /* error callback */
        if(NULL != (ctc_dev->ctc_irq.ctc_err_handle)) {
            ctc_dev->ctc_irq.ctc_err_handle(ctc_dev);
        }
    }

    /* check interrupt state bit */
    if(SET == hals_ctc_interrupt_flag_get(CTC_INT_FLAG_EREF)) {
        /* clear interrupt flag */
        hals_ctc_interrupt_flag_clear(CTC_INT_FLAG_EREF);
        /* expect reference callback */
        if(NULL != (ctc_dev->ctc_irq.ctc_eref_handle)) {
            ctc_dev->ctc_irq.ctc_eref_handle(ctc_dev);
        }
    }
}

/*!
    \brief      CTC interrupt handler content function,which is merely used in ctc_handler
    \param[in]  ctc_dev: CTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to CTC interrupt callback functions structure
    \param[out] none
    \retval     none
*/
void hal_ctc_irq_handle_set(hal_ctc_dev_struct *ctc_dev, hal_ctc_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == ctc_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [ctc_dev] or pointer [p_irq] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* clock trim OK interrupt handler set */
    if(NULL != p_irq->ctc_ckok_handle) {
        ctc_dev->ctc_irq.ctc_ckok_handle = p_irq->ctc_ckok_handle;
    } else {
        ctc_dev->ctc_irq.ctc_ckok_handle = NULL;
    }

    /* clock trim warning interrupt handler set */
    if(NULL != p_irq->ctc_ckwarn_handle) {
        ctc_dev->ctc_irq.ctc_ckwarn_handle = p_irq->ctc_ckwarn_handle;
    } else {
        ctc_dev->ctc_irq.ctc_ckwarn_handle = NULL;
    }

    /* error interrupt handler set */
    if(NULL != p_irq->ctc_err_handle) {
        ctc_dev->ctc_irq.ctc_err_handle = p_irq->ctc_err_handle;
    } else {
        ctc_dev->ctc_irq.ctc_err_handle = NULL;
    }

    /* expect reference interrupt handler set */
    if(NULL != p_irq->ctc_eref_handle) {
        ctc_dev->ctc_irq.ctc_eref_handle = p_irq->ctc_eref_handle;
    } else {
        ctc_dev->ctc_irq.ctc_eref_handle = NULL;
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  ctc_dev: CTC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_ctc_irq_handle_all_reset(hal_ctc_dev_struct *ctc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == ctc_dev) {
        HAL_DEBUGE("pointer [ctc_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* configure interrupt callback function to NULL */
    ctc_dev->ctc_irq.ctc_ckok_handle   = NULL;
    ctc_dev->ctc_irq.ctc_ckwarn_handle = NULL;
    ctc_dev->ctc_irq.ctc_err_handle    = NULL;
    ctc_dev->ctc_irq.ctc_eref_handle   = NULL;
}

/*!
    \brief      start CTC device with interrupt
    \param[in]  ctc_dev: CTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: the callback handler of CTC interrupt
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_ctc_start_interrupt(hal_ctc_dev_struct *ctc_dev, hal_ctc_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == ctc_dev) {
        HAL_DEBUGE("pointer [ctc_dev] or pointer [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* lock CTC */
    HAL_LOCK(ctc_dev);

    if(NULL != p_irq->ctc_ckok_handle) {
        ctc_dev->ctc_irq.ctc_ckok_handle = p_irq->ctc_ckok_handle;
        hals_ctc_interrupt_flag_clear(CTC_INT_CKOK);
        hals_ctc_interrupt_enable(CTC_INT_CKOK);
    } else{
        hals_ctc_interrupt_disable(CTC_INT_CKOK);
        hals_ctc_interrupt_flag_clear(CTC_INT_CKOK);
    }
    if(NULL != p_irq->ctc_ckwarn_handle) {
        ctc_dev->ctc_irq.ctc_ckwarn_handle = p_irq->ctc_ckwarn_handle;
        hals_ctc_interrupt_flag_clear(CTC_INT_CKWARN);
        hals_ctc_interrupt_enable(CTC_INT_CKWARN);
    }else{
        hals_ctc_interrupt_disable(CTC_INT_CKWARN);
        hals_ctc_interrupt_flag_clear(CTC_INT_CKWARN);
    }
    if(NULL != p_irq->ctc_err_handle) {
        ctc_dev->ctc_irq.ctc_err_handle = p_irq->ctc_err_handle;
        hals_ctc_interrupt_flag_clear(CTC_INT_ERR);
        hals_ctc_interrupt_enable(CTC_INT_ERR);
    }else{
        hals_ctc_interrupt_disable(CTC_INT_ERR);
        hals_ctc_interrupt_flag_clear(CTC_INT_ERR);
    }
    if(NULL != p_irq->ctc_eref_handle) {
        ctc_dev->ctc_irq.ctc_eref_handle = p_irq->ctc_eref_handle;
        hals_ctc_interrupt_flag_clear(CTC_INT_EREF);
        hals_ctc_interrupt_enable(CTC_INT_EREF);
    }else{
        hals_ctc_interrupt_disable(CTC_INT_EREF);
        hals_ctc_interrupt_flag_clear(CTC_INT_EREF);
    }
    /* enable CTC trim counter */
    hals_ctc_counter_enable();
    /* unlock CTC */
    HAL_UNLOCK(ctc_dev);

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      stop CTC with interrupt
    \param[in]  ctc_dev: CTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_ctc_stop_interrupt(hal_ctc_dev_struct *ctc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == ctc_dev) {
        HAL_DEBUGE("pointer [ctc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* configure interrupt callback function to NULL */
    ctc_dev->ctc_irq.ctc_ckok_handle   = NULL;
    ctc_dev->ctc_irq.ctc_ckwarn_handle = NULL;
    ctc_dev->ctc_irq.ctc_err_handle    = NULL;
    ctc_dev->ctc_irq.ctc_eref_handle   = NULL;

    /* disable the CTC interrupt*/
    hals_ctc_interrupt_disable(CTC_INT_CKOK);
    hals_ctc_interrupt_flag_clear(CTC_INT_CKOK);
    hals_ctc_interrupt_disable(CTC_INT_CKWARN);
    hals_ctc_interrupt_flag_clear(CTC_INT_CKWARN);
    hals_ctc_interrupt_disable(CTC_INT_ERR);
    hals_ctc_interrupt_flag_clear(CTC_INT_ERR);
    hals_ctc_interrupt_disable(CTC_INT_EREF);
    hals_ctc_interrupt_flag_clear(CTC_INT_EREF);

    /* disable CTC trim counter */
    hals_ctc_counter_disable();
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      enable CTC trim counter
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_ctc_counter_enable(void)
{
    CTC_CTL0 |= (uint32_t)CTC_CTL0_CNTEN;
}

/*!
    \brief      disable CTC trim counter
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_ctc_counter_disable(void)
{
    CTC_CTL0 &= (uint32_t)(~CTC_CTL0_CNTEN);
}

/*!
    \brief      configure reference signal source prescaler
    \param[in]  prescaler:
                only one parameter can be selected which is shown as below:
      \arg        CTC_REFSOURCE_PSC_OFF: reference signal not divided
      \arg        CTC_REFSOURCE_PSC_DIV2: reference signal divided by 2
      \arg        CTC_REFSOURCE_PSC_DIV4: reference signal divided by 4
      \arg        CTC_REFSOURCE_PSC_DIV8: reference signal divided by 8
      \arg        CTC_REFSOURCE_PSC_DIV16: reference signal divided by 16
      \arg        CTC_REFSOURCE_PSC_DIV32: reference signal divided by 32
      \arg        CTC_REFSOURCE_PSC_DIV64: reference signal divided by 64
      \arg        CTC_REFSOURCE_PSC_DIV128: reference signal divided by 128
    \param[out] none
    \retval     none
*/
void hals_ctc_refsource_prescaler_config(uint32_t prescaler)
{
    CTC_CTL1 &= (uint32_t)(~CTC_CTL1_REFPSC);
    CTC_CTL1 |= (uint32_t)prescaler;
}

/*!
    \brief      select reference signal source
    \param[in]  refs:
                only one parameter can be selected which is shown as below:
      \arg        CTC_REFSOURCE_GPIO: GPIO is selected
      \arg        CTC_REFSOURCE_LXTAL: LXTAL is selected
    \param[out] none
    \retval     none
*/
void hals_ctc_refsource_signal_select(uint32_t refs)
{
    CTC_CTL1 &= (uint32_t)(~CTC_CTL1_REFSEL);
    CTC_CTL1 |= (uint32_t)refs;
}


/*!
    \brief      configure reference signal source polarity
    \param[in]  polarity:
                only one parameter can be selected which is shown as below:
      \arg        CTC_REFSOURCE_POLARITY_FALLING: reference signal source polarity is falling edge
      \arg        CTC_REFSOURCE_POLARITY_RISING: reference signal source polarity is rising edge
    \param[out] none
    \retval     none
*/
void hals_ctc_refsource_polarity_config(uint32_t polarity)
{
    CTC_CTL1 &= (uint32_t)(~CTC_CTL1_REFPOL);
    CTC_CTL1 |= (uint32_t)polarity;
}

/*!
    \brief      configure clock trim base limit value
    \param[in]  limit_value: 8-bit clock trim base limit value
      \arg        0x00 - 0xFF
    \param[out] none
    \retval     none
*/
void hals_ctc_clock_limit_value_config(uint8_t limit_value)
{
    CTC_CTL1 &= (uint32_t)(~CTC_CTL1_CKLIM);
    CTC_CTL1 |= CTL1_CKLIM(limit_value);
}

/*!
    \brief      configure CTC counter reload value
    \param[in]  reload_value: 16-bit CTC counter reload value
      \arg        0x0000-0xFFFF
    \param[out] none
    \retval     none
*/
void hals_ctc_counter_reload_value_config(uint16_t reload_value)
{
    CTC_CTL1 &= (uint32_t)(~CTC_CTL1_RLVALUE);
    CTC_CTL1 |= (uint32_t)reload_value;
}

/*!
    \brief      configure hardware automatically trim mode
    \param[in]  hardmode:
                only one parameter can be selected which is shown as below:
      \arg        CTC_HARDWARE_TRIM_MODE_ENABLE: hardware automatically trim mode enable
      \arg        CTC_HARDWARE_TRIM_MODE_DISABLE: hardware automatically trim mode disable
    \param[out] none
    \retval     none
*/
void hals_ctc_hardware_trim_mode_config(uint32_t hardmode)
{
    CTC_CTL0 &= (uint32_t)(~CTC_CTL0_AUTOTRIM);
    CTC_CTL0 |= (uint32_t)hardmode;
}

/*!
    \brief      generate software reference source sync pulse
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_ctc_software_refsource_pulse_generate(void)
{
    CTC_CTL0 |= (uint32_t)CTC_CTL0_SWREFPUL;
}

/*!
    \brief      configure the IRC48M trim value
    \param[in]  trim_value: 6-bit IRC48M trim value
      \arg        0x00-0x3F
    \param[out] none
    \retval     none
*/
void hals_ctc_irc48m_trim_value_config(uint8_t trim_value)
{
    /* clear TRIMVALUE bits */
    CTC_CTL0 &= (~(uint32_t)CTC_CTL0_TRIMVALUE);
    /* set TRIMVALUE bits */
    CTC_CTL0 |= CTL0_TRIMVALUE(trim_value);
}

/*!
    \brief      read CTC counter capture value when reference sync pulse occurred
    \param[in]  none
    \param[out] none
    \retval     the 16-bit CTC counter capture value
*/
uint16_t hals_ctc_counter_capture_value_read(void)
{
    uint16_t capture_value = 0U;
    capture_value = (uint16_t)GET_STAT_REFCAP(CTC_STAT);
    return (capture_value);
}

/*!
    \brief      read CTC trim counter direction when reference sync pulse occurred
    \param[in]  none
    \param[out] none
    \retval     FlagStatus: SET or RESET
      \arg        SET: CTC trim counter direction is down-counting
      \arg        RESET: CTC trim counter direction is up-counting
*/
FlagStatus hals_ctc_counter_direction_read(void)
{
    FlagStatus ret_status = RESET;
    if(RESET != (CTC_STAT & CTC_STAT_REFDIR)) {
        ret_status = SET;
    }
    return ret_status;
}

/*!
    \brief      read CTC counter reload value
    \param[in]  none
    \param[out] none
    \retval     the 16-bit CTC counter reload value
*/
uint16_t hals_ctc_counter_reload_value_read(void)
{
    uint16_t reload_value = 0U;
    reload_value = (uint16_t)(CTC_CTL1 & CTC_CTL1_RLVALUE);
    return (reload_value);
}

/*!
    \brief      read the IRC48M trim value
    \param[in]  none
    \param[out] none
    \retval     the 6-bit IRC48M trim value
*/
uint8_t hals_ctc_irc48m_trim_value_read(void)
{
    uint8_t trim_value = 0U;
    trim_value = (uint8_t)GET_CTL0_TRIMVALUE(CTC_CTL0);
    return (trim_value);
}

/*!
    \brief      get CTC flag
    \param[in]  flag: the CTC flag
                only one parameter can be selected which is shown as below:
      \arg        CTC_FLAG_CKOK: clock trim OK flag
      \arg        CTC_FLAG_CKWARN: clock trim warning flag
      \arg        CTC_FLAG_ERR: error flag
      \arg        CTC_FLAG_EREF: expect reference flag
      \arg        CTC_FLAG_CKERR: clock trim error bit
      \arg        CTC_FLAG_REFMISS: reference sync pulse miss
      \arg        CTC_FLAG_TRIMERR: trim value error bit
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_ctc_flag_get(uint32_t flag)
{
    FlagStatus ret_status = RESET;

    if(RESET != (CTC_STAT & flag)) {
        ret_status = SET;
    }
    return ret_status;
}

/*!
    \brief      clear CTC flag
    \param[in]  flag: the CTC flag
                only one parameter can be selected which is shown as below:
      \arg        CTC_FLAG_CKOK: clock trim OK flag
      \arg        CTC_FLAG_CKWARN: clock trim warning flag
      \arg        CTC_FLAG_ERR: error flag
      \arg        CTC_FLAG_EREF: expect reference flag
      \arg        CTC_FLAG_CKERR: clock trim error bit
      \arg        CTC_FLAG_REFMISS: reference sync pulse miss
      \arg        CTC_FLAG_TRIMERR: trim value error bit
    \param[out] none
    \retval     none
*/
void hals_ctc_flag_clear(uint32_t flag)
{
    if(RESET != (flag & CTC_FLAG_MASK)) {
        CTC_INTC |= CTC_INTC_ERRIC;
    } else {
        CTC_INTC |= flag;
    }
}

/*!
    \brief      enable the CTC interrupt
    \param[in]  interrupt: CTC interrupt enable
                one or more parameters can be selected which are shown as below:
      \arg        CTC_INT_CKOK: clock trim OK interrupt enable
      \arg        CTC_INT_CKWARN: clock trim warning interrupt enable
      \arg        CTC_INT_ERR: error interrupt enable
      \arg        CTC_INT_EREF: expect reference interrupt enable
    \param[out] none
    \retval     none
*/
void hals_ctc_interrupt_enable(uint32_t interrupt)
{
    CTC_CTL0 |= (uint32_t)interrupt;
}

/*!
    \brief      disable the CTC interrupt
    \param[in]  interrupt: CTC interrupt enable source
                one or more parameters can be selected which are shown as below:
      \arg        CTC_INT_CKOK: clock trim OK interrupt enable
      \arg        CTC_INT_CKWARN: clock trim warning interrupt enable
      \arg        CTC_INT_ERR: error interrupt enable
      \arg        CTC_INT_EREF: expect reference interrupt enable
    \param[out] none
    \retval     none
*/
void hals_ctc_interrupt_disable(uint32_t interrupt)
{
    CTC_CTL0 &= (uint32_t)(~interrupt);
}

/*!
    \brief      get CTC interrupt flag
    \param[in]  int_flag: the CTC interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        CTC_INT_FLAG_CKOK: clock trim OK interrupt
      \arg        CTC_INT_FLAG_CKWARN: clock trim warning interrupt
      \arg        CTC_INT_FLAG_ERR: error interrupt
      \arg        CTC_INT_FLAG_EREF: expect reference interrupt
      \arg        CTC_FLAG_CKERR: clock trim error bit
      \arg        CTC_FLAG_REFMISS: reference sync pulse miss
      \arg        CTC_FLAG_TRIMERR: trim value error
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_ctc_interrupt_flag_get(uint32_t int_flag)
{
    uint32_t interrupt_flag = 0U, intenable = 0U;
    FlagStatus ret_status = RESET;

    /* check whether the interrupt is enabled */
    if(RESET != (int_flag & CTC_FLAG_MASK)) {
        intenable = CTC_CTL0 & CTC_INT_ERR;
    } else {
        intenable = CTC_CTL0 & int_flag;
    }
    interrupt_flag = CTC_STAT & int_flag;

    if(interrupt_flag && intenable) {
        ret_status = SET;
    }
    return ret_status;
}

/*!
    \brief      clear CTC interrupt flag
    \param[in]  int_flag: the CTC interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        CTC_INT_FLAG_CKOK: clock trim OK interrupt
      \arg        CTC_INT_FLAG_CKWARN: clock trim warning interrupt
      \arg        CTC_INT_FLAG_ERR: error interrupt
      \arg        CTC_INT_FLAG_EREF: expect reference interrupt
      \arg        CTC_FLAG_CKERR: clock trim error bit interrupt
      \arg        CTC_FLAG_REFMISS: reference sync pulse miss interrupt
      \arg        CTC_FLAG_TRIMERR: trim value error interrupt
    \param[out] none
    \retval     none
*/
void hals_ctc_interrupt_flag_clear(uint32_t int_flag)
{
    if(RESET != (int_flag & CTC_FLAG_MASK)) {
        CTC_INTC |= CTC_INTC_ERRIC;
    } else {
        CTC_INTC |= int_flag;
    }
}
