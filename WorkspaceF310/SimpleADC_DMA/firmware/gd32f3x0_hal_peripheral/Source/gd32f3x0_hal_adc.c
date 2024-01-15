/*!
    \file    gd32f3x0_hal_adc.c
    \brief   ADC driver

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

/* delaytime for ADC enable and disable, in milliseconds */
#define ADC_ENABLE_DELAYTIME                    ((uint32_t) 2U)
#define ADC_DISABLE_DELAYTIME                   ((uint32_t) 2U)
/* delaytime for temperature sensor stabilization time, in microseconds */
#define ADC_TEMPSENSOR_DELAYTIME                ((uint32_t) 10U)

/* enable ADC */
/* enable the ADC */
static int32_t _adc_enable(hal_adc_dev_struct *adc_dev);
/*disable the ADC */
static int32_t _adc_disable(hal_adc_dev_struct *adc_dev);
/* get the ADC enable state */
static FlagStatus _adc_enable_state_get(void);

/* DMA callback function */
/* ADC DMA full transmission complete callback */
static void _adc_dma_full_transfer_complete(void *dma);
/* ADC DMA half transmission complete callback */
static void _adc_dma_half_transfer_complete(void *dma);
/* ADC DMA error callback */
static void _adc_dma_error(void *dma);

/* get ADC state and error */
/* set ADC state */
static void _adc_state_set(hal_adc_dev_struct *adc_dev, hal_adc_state_enum adc_state);
/* clear ADC state */
static void _adc_state_clear(hal_adc_dev_struct *adc_dev, hal_adc_state_enum adc_state);
/* set ADC error */
static void _adc_error_set(hal_adc_dev_struct *adc_dev, hal_adc_error_enum adc_error);
/* clear ADC error */
static void _adc_error_clear(hal_adc_dev_struct *adc_dev, hal_adc_error_enum adc_error);

/*!
    \brief      initialize the ADC structure with the default values
    \param[in]  hal_struct_type: the argument could be selected from enumeration <hal_adc_struct_type_enum>
    \param[in]  p_struct: pointer to ADC structure that contains the configuration information
    \param[out] none
    \retval     none
*/
void hal_adc_struct_init(hal_adc_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer [*p_struct] value is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_ADC_INIT_STRUCT:
        /* initialize ADC initialization structure with the default values */
        ((hal_adc_init_struct *)p_struct)->resolution                  = ADC_RESOLUTION_12B;
        ((hal_adc_init_struct *)p_struct)->data_alignment              = ADC_LSB_ALIGNMENT;
        ((hal_adc_init_struct *)p_struct)->scan_mode                   = DISABLE;
        ((hal_adc_init_struct *)p_struct)->hardware_oversampling       = DISABLE;
        ((hal_adc_init_struct *)p_struct)->oversample_trigger_mode     = ADC_OVERSAMPLING_ALL_CONVERT;
        ((hal_adc_init_struct *)p_struct)->oversampling_shift          = ADC_OVERSAMPLE_SHIFT_NONE;
        ((hal_adc_init_struct *)p_struct)->oversampling_ratio          = ADC_OVERSAMPLE_RATIO_MUL2;

        break;

    case HAL_ADC_ROUTINE_CONFIG_STRUCT:
        /* initialize ADC routine channel initialization structure with the default values */
        ((hal_adc_routine_config_struct *)p_struct)->routine_sequence_conversions                = DISABLE;
        ((hal_adc_routine_config_struct *)p_struct)->routine_sequence_length                     = 0U;
        ((hal_adc_routine_config_struct *)p_struct)->routine_sequence_external_trigger_select    = ADC_EXTTRIG_ROUTINE_NONE;
        ((hal_adc_routine_config_struct *)p_struct)->continuous_mode                             = DISABLE;
        ((hal_adc_routine_config_struct *)p_struct)->discontinuous_mode                          = DISABLE;
        ((hal_adc_routine_config_struct *)p_struct)->number_of_conversions_in_discontinuous_mode = 0U;
        break;

    case HAL_ADC_ROUTINE_RANK_CONFIG_STRUCT:
        /* initialize ADC routine channel configuration structure with the default values */
        ((hal_adc_routine_rank_config_struct *)p_struct)->channel          = 0U;
        ((hal_adc_routine_rank_config_struct *)p_struct)->routine_sequence = ADC_ROUTINE_SEQUENCE_0;
        ((hal_adc_routine_rank_config_struct *)p_struct)->sampling_time    = ADC_SAMPLETIME_1POINT5;
        break;

    case HAL_ADC_INSERTED_CONFIG_STRUCT:
        /* initialize ADC inserted channel initialization structure with the default values */
        ((hal_adc_inserted_config_struct *)p_struct)->inserted_sequence_conversions             = DISABLE;
        ((hal_adc_inserted_config_struct *)p_struct)->inserted_sequence_length                  = 0U;
        ((hal_adc_inserted_config_struct *)p_struct)->inserted_sequence_external_trigger_select = ADC_EXTTRIG_ROUTINE_NONE;
        ((hal_adc_inserted_config_struct *)p_struct)->auto_convert                              = DISABLE;
        ((hal_adc_inserted_config_struct *)p_struct)->discontinuous_mode                        = DISABLE;
        break;

    case HAL_ADC_INSERTED_RANK_CONFIG_STRUCT:
        /* initialize ADC inserted channel configuration structure with the default values */
        ((hal_adc_inserted_rank_config_struct *)p_struct)->channel           = 0U;
        ((hal_adc_inserted_rank_config_struct *)p_struct)->inserted_sequence = ADC_INSERTED_SEQUENCE_0;
        ((hal_adc_inserted_rank_config_struct *)p_struct)->sampling_time     = ADC_SAMPLETIME_1POINT5;
        ((hal_adc_inserted_rank_config_struct *)p_struct)->data_offset       = 0U;
        break;

    case HAL_ADC_IRQ_STRUCT:
        /* initialize ADC device interrupt callback function pointer structure with the default values */
        ((hal_adc_irq_struct *)p_struct)->adc_watchdog_handle = NULL;
        ((hal_adc_irq_struct *)p_struct)->adc_eoc_handle      = NULL;
        ((hal_adc_irq_struct *)p_struct)->adc_eoic_handle     = NULL;
        break;

    case HAL_ADC_DMA_HANDLE_CB_STRUCT:
        /* initialize ADC DMA callback function pointer structure with the default values */
        ((hal_adc_dma_handle_cb_struct *)p_struct)->full_transcom_handle = NULL;
        ((hal_adc_dma_handle_cb_struct *)p_struct)->half_transcom_handle = NULL;
        ((hal_adc_dma_handle_cb_struct *)p_struct)->error_handle         = NULL;
        break;

    case HAL_ADC_WATCHDOG_CONFIG_STRUCT:
        /* initialize ADC watchdog configuration structure with the default values */
        ((hal_adc_watchdog_config_struct *)p_struct)->routine_sequence_analog_watchdog  = DISABLE;
        ((hal_adc_watchdog_config_struct *)p_struct)->inserted_sequence_analog_watchdog = DISABLE;
        ((hal_adc_watchdog_config_struct *)p_struct)->analog_watchdog_mode              = ADC_OVERSAMPLING_ALL_CONVERT;
        ((hal_adc_watchdog_config_struct *)p_struct)->analog_watchdog_channel_select    = ADC_CHANNEL_0;
        ((hal_adc_watchdog_config_struct *)p_struct)->analog_watchdog_high_threshold    = 0U;
        ((hal_adc_watchdog_config_struct *)p_struct)->analog_watchdog_low_threshold     = 0U;
        break;

    case HAL_ADC_DEV_STRUCT:
        /* initialize ADC device information structure with the default values */
        ((hal_adc_dev_struct *)p_struct)->adc_irq.adc_eoc_handle       = NULL;
        ((hal_adc_dev_struct *)p_struct)->adc_irq.adc_eoic_handle      = NULL;
        ((hal_adc_dev_struct *)p_struct)->adc_irq.adc_watchdog_handle  = NULL;
        ((hal_adc_dev_struct *)p_struct)->p_dma_adc                    = NULL;
        ((hal_adc_dev_struct *)p_struct)->adc_dma.full_transcom_handle = NULL;
        ((hal_adc_dev_struct *)p_struct)->adc_dma.half_transcom_handle = NULL;
        ((hal_adc_dev_struct *)p_struct)->adc_dma.error_handle         = NULL;
        ((hal_adc_dev_struct *)p_struct)->error_state                  = HAL_ADC_ERROR_NONE;
        ((hal_adc_dev_struct *)p_struct)->state                        = HAL_ADC_STATE_RESET;
        break;

    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize ADC device structure and init structure
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_deinit(hal_adc_dev_struct *adc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    _adc_state_set(adc_dev, HAL_ADC_STATE_BUSY_SYSTEM);

    /* if ADC is disabled */
    if(HAL_ERR_NONE == _adc_disable(adc_dev)) {
        /* deinit ADC */
        hals_adc_deinit();
        adc_dev->error_state = HAL_ADC_ERROR_NONE;
        adc_dev->state = HAL_ADC_STATE_RESET;
    } else {
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      initialize ADC
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_init: the pointer of ADC init structure
                  resolution_select:
                  only one parameter can be selected which is shown as below:
      \arg          ADC_RESOLUTION_12B: 12-bit ADC resolution
      \arg          ADC_RESOLUTION_10B: 10-bit ADC resolution
      \arg          ADC_RESOLUTION_8B : 8-bit ADC resolution
      \arg          ADC_RESOLUTION_6B : 6-bit ADC resolution
                  data_alignment:
                  only one parameter can be selected which is shown as below:
      \arg          ADC_LSB_ALIGNMENT: LSB alignment
      \arg          ADC_MSB_ALIGNMENT: MSB alignment
                  scan_mode: ENABLE, DISABLE
                  only one parameter can be selected which is shown as below:
      \arg          ENABLE: enable scan mode
      \arg          DISABLE: disable scan mode
                  oversample_config:
                    the argument could be selected from structure <hal_adc_oversample_struct>
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_init(hal_adc_dev_struct *adc_dev, hal_adc_init_struct *p_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc_dev) || (NULL == p_init)) {
        HAL_DEBUGE("pointer [adc_dev] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    if(HAL_ADC_STATE_RESET == (hal_adc_state_get(adc_dev))) {
        adc_dev->error_state = HAL_ADC_ERROR_NONE;
    }

    if((HAL_ERR_NONE == _adc_disable(adc_dev)) &&
            (RESET == ((hal_adc_error_get(adc_dev)) & HAL_ADC_ERROR_SYSTEM))) {
        /* set ADC state  */
        _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);
        _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
        _adc_state_set(adc_dev, HAL_ADC_STATE_BUSY_SYSTEM);

        /* init ADC */
        /* 1.ADC data resolution select */
        /* ADC data resolution select can be updated only when ADC is disabled*/
        if(RESET == _adc_enable_state_get()) {
            hals_adc_resolution_config(p_init->resolution);
        }
        /* 2.ADC data alignment config */
        hals_adc_data_alignment_config(p_init->data_alignment);

        /* 3.ADC scan mode config */
        hals_adc_special_function_config(ADC_SCAN_MODE, p_init->scan_mode);

        /* 4.ADC oversampling mode config */
        if(ENABLE == p_init->hardware_oversampling) {
            hals_adc_oversample_mode_config(p_init->oversample_trigger_mode, p_init->oversampling_shift,
                                            p_init->oversampling_ratio);
            /* enable ADC oversample mode */
            hals_adc_oversample_mode_enable();
        } else {
            if(SET == __HAL_ADC_GET_OVERSAMPLE_ENABLE) {
                /* disable ADC oversample mode */
                hals_adc_oversample_mode_disable();
            }
        }
    } else {
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      ADC calibration
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_calibration_start(hal_adc_dev_struct *adc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is disabled */
    if(HAL_ERR_NONE == _adc_disable(adc_dev)) {
        /* set ADC state  */
        _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);
        _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
        _adc_state_set(adc_dev, HAL_ADC_STATE_BUSY_SYSTEM);

        /* enable the ADC */
        _adc_enable(adc_dev);
        /* ADC calibration and reset calibration */
        hals_adc_calibration_enable();

        /* set ADC state  */
        _adc_state_clear(adc_dev, HAL_ADC_STATE_BUSY_SYSTEM);
        _adc_state_set(adc_dev, HAL_ADC_STATE_READY);
    } else {
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      initialize ADC routine channel
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_rchannel: the pointer of ADC routine sequence init structure
                  routine_sequence_conversions:
                  only one parameter can be selected which is shown as below:
      \arg          ENABLE: enable external trigger
      \arg          DISABLE: disable external trigger
                  routine_sequence_length: 1-16
                  routine_sequence_external_trigger_select:
                  only one parameter can be selected which is shown as below:
      \arg          ADC_EXTTRIG_ROUTINE_T0_CH0: TIMER0 CH0 event select
      \arg          ADC_EXTTRIG_ROUTINE_T0_CH1: TIMER0 CH1 event select
      \arg          ADC_EXTTRIG_ROUTINE_T0_CH2: TIMER0 CH2 event select
      \arg          ADC_EXTTRIG_ROUTINE_T1_CH1: TIMER1 CH1 event select
      \arg          ADC_EXTTRIG_ROUTINE_T2_TRGO: TIMER2 TRGO event select
      \arg          ADC_EXTTRIG_ROUTINE_T14_CH0: TIMER14 CH0 event select
      \arg          ADC_EXTTRIG_ROUTINE_EXTI_11: external interrupt line 11
      \arg          ADC_EXTTRIG_ROUTINE_NONE: software trigger
                  continuous_mode:
                  only one parameter can be selected which is shown as below:
      \arg          ENABLE: enable continuous mode
      \arg          DISABLE: disable continuous mode
                  discontinuous_mode:
                  only one parameter can be selected which is shown as below:
      \arg          ENABLE: enable continuous
      \arg          DISABLE: disable continuous
                  number_of_conversions_in_discontinuous_mode: 1-8
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_routine_channel_config(hal_adc_dev_struct *adc_dev, hal_adc_routine_config_struct *p_rchannel)
{
    uint32_t reg_temp;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc_dev) || (NULL == p_rchannel)) {
        HAL_DEBUGE("pointer [adc_dev] or [p_rinit] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    if(HAL_ADC_STATE_RESET == (hal_adc_state_get(adc_dev))) {
        adc_dev->error_state = HAL_ADC_ERROR_NONE;
    }

    if((HAL_ERR_NONE == _adc_disable(adc_dev)) &&
            (RESET == ((hal_adc_error_get(adc_dev)) & HAL_ADC_ERROR_SYSTEM)) &&
            (ENABLE == (p_rchannel->routine_sequence_conversions))) {
        /* set ADC state  */
        _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);
        _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
        _adc_state_set(adc_dev, HAL_ADC_STATE_BUSY_SYSTEM);
        /* init ADC routine channel */
        /* 1.ADC channel length config */
        hals_adc_channel_length_config(ADC_ROUTINE_CHANNEL, p_rchannel->routine_sequence_length);
        /* 2.ADC routine channel external trigger config */
        hals_adc_external_trigger_source_config(ADC_ROUTINE_CHANNEL,
                                                p_rchannel->routine_sequence_external_trigger_select);

        /* 3.ADC continuous mode config */
        hals_adc_special_function_config(ADC_CONTINUOUS_MODE, p_rchannel->continuous_mode);
        /* 4.ADC discontinuous mode config */
        if(ENABLE == p_rchannel->discontinuous_mode) {
            if(DISABLE == p_rchannel->continuous_mode) {
                reg_temp = ADC_CTL0;
                /* clear DISRC bit and DISNUM bit */
                reg_temp &= ~((uint32_t)ADC_CTL0_DISRC);
                reg_temp &= ~((uint32_t)ADC_CTL0_DISNUM);
                reg_temp |= CTL0_DISNUM(((uint32_t)(p_rchannel->number_of_conversions_in_discontinuous_mode) - 1U));
                reg_temp |= (uint32_t)ADC_CTL0_DISRC;
                ADC_CTL0 = reg_temp;
            } else {
                ADC_CTL0 &= ~((uint32_t)ADC_CTL0_DISRC);
                _adc_error_set(adc_dev, HAL_ADC_ERROR_CONFIG);
                _adc_error_set(adc_dev, HAL_ADC_ERROR_SYSTEM);
            }
        } else {
            reg_temp = ADC_CTL0;
            reg_temp &= ~(uint32_t)ADC_CTL0_DISRC;
            ADC_CTL0 = reg_temp;
        }
    } else {
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      configure ADC routine channel
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_rrank: the pointer of ADC routine channel configuration structure
                  channel: ADC_CHANNEL_x(x=0..18)
                  routine_sequence:
                    the argument could be selected from enumeration <hal_adc_routine_sequence_enum>
                  sampling_time:
        \arg        ADC_SAMPLETIME_1POINT5: 1.5 cycles
        \arg        ADC_SAMPLETIME_7POINT5: 7.5 cycles
        \arg        ADC_SAMPLETIME_13POINT5: 13.5 cycles
        \arg        ADC_SAMPLETIME_28POINT5: 28.5 cycles
        \arg        ADC_SAMPLETIME_41POINT5: 41.5 cycles
        \arg        ADC_SAMPLETIME_55POINT5: 55.5 cycles
        \arg        ADC_SAMPLETIME_71POINT5: 71.5 cycles
        \arg        ADC_SAMPLETIME_239POINT5: 239.5 cycles
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_routine_rank_config(hal_adc_dev_struct *adc_dev, hal_adc_routine_rank_config_struct *p_rrank)
{
    __IO uint32_t wait_loop = 0;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc_dev) || (NULL == p_rrank)) {
        HAL_DEBUGE("pointer [adc_dev] or [p_rchannel] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* configure ADC routine channel */
    hals_adc_routine_channel_config(p_rrank->routine_sequence, p_rrank->channel,
                                    p_rrank->sampling_time);

    /* enable temperature sensor and VREFINT measurement */
    if((ADC_CHANNEL_TEMPSENSOR == p_rrank->channel) ||
            (ADC_CHANNEL_VREFINT == p_rrank->channel)) {
        if(RESET == __HAL_ADC_GET_TEMPVREF_ENABLE) {
            /* enable the temperature sensor and Vrefint channel */
            ADC_CTL1 |= ADC_CTL1_TSVREN;
            if(ADC_CHANNEL_TEMPSENSOR == p_rrank->channel) {
                /* compute number of CPU cycles to wait for */
                wait_loop = (ADC_TEMPSENSOR_DELAYTIME * (g_systemcoreclock / 1000000));
                /* delay for temperature sensor stabilization time */
                while(wait_loop != 0) {
                    wait_loop--;
                }
            }
        }
    }
    /* enable backup battery voltage measurement */
    if(ADC_CHANNEL_BATTERY == p_rrank->channel) {
        if(RESET == __HAL_ADC_GET_BATTERY_ENABLE) {
            /* enable the temperature sensor and Vrefint channel */
            ADC_CTL1 |= ADC_CTL1_VBETEN;
        }
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      enable ADC and start the conversion of routine sequence
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_start(hal_adc_dev_struct *adc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is enabled */
    if(HAL_ERR_NONE == _adc_enable(adc_dev)) {
        _adc_state_clear(adc_dev, HAL_ADC_STATE_READY);
        _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_EOC);
        _adc_state_set(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);

        /* if ADC configured in automatic inserted mode */
        if(SET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV) {
            _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_EOC);
            _adc_state_set(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
        }

        if(SET == ((hal_adc_state_get(adc_dev)) & HAL_ADC_STATE_INSERTED_BUSY)) {
            _adc_error_clear(adc_dev, HAL_ADC_ERROR_DMA);
        } else {
            adc_dev->error_state = HAL_ADC_ERROR_NONE;
        }

        /* clear ADC flag */
        hals_adc_flag_clear(ADC_FLAG_EOC);

        /* enable conversion of routine sequence */
        if(ADC_EXTTRIG_ROUTINE_NONE == __HAL_ADC_GET_ROUTINECH_EXTTRIGGER) {
            /* enable software trigger */
            hals_adc_external_trigger_config(ADC_ROUTINE_CHANNEL, ENABLE);
            hals_adc_software_trigger_enable(ADC_ROUTINE_CHANNEL);
        } else {
            /* enable external trigger */
            hals_adc_external_trigger_config(ADC_ROUTINE_CHANNEL, ENABLE);
        }
    } else {
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      stop the conversion of routine sequence and disable ADC
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_stop(hal_adc_dev_struct *adc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is disabled */
    if(HAL_ERR_NONE == _adc_disable(adc_dev)) {
        _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);
        _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
        _adc_state_set(adc_dev, HAL_ADC_STATE_READY);
    } else {
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      polling for ADC routine sequence conversion, this function is just for single conversion
                (scan mode disabled or scan mode enabled with the length of routine channel is 1)
    \param[in]  adc_dev: ADC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: time out duration in milliscond
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_TIMEOUT, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_routine_conversion_poll(hal_adc_dev_struct *adc_dev, uint32_t timeout_ms)
{
    uint32_t tick_start = 0;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* lock ADC */
    HAL_LOCK(adc_dev);

    /* if ADC configured in DMA mode */
    if(SET == __HAL_ADC_GET_DMA_MODE) {
        _adc_error_set(adc_dev, HAL_ADC_ERROR_CONFIG);
        return HAL_ERR_NO_SUPPORT;
    }

    /* set timeout */
    tick_start = hal_sys_basetick_count_get();

    /* if single conversion: scan mode disabled or enabled with length = 1 */
    if((RESET == __HAL_ADC_GET_SCAN_MODE) && (RESET == __HAL_ADC_GET_ROUTINECH_LENGTH)) {
        /* wait until end of conversion flag is raised */
        while(RESET == hals_adc_flag_get(ADC_FLAG_EOC)) {
            /* check for the timeout */
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    _adc_state_set(adc_dev, HAL_ADC_STATE_TIMEOUT);
                    /* when timeout occurs, output timeout warning message */
                    HAL_DEBUGW("adc_dev routine conversion poll timeout");
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
    } else {
        return HAL_ERR_NO_SUPPORT;
    }

    /* clear ADC EOC flag and STRC flag */
    hals_adc_flag_clear(ADC_FLAG_EOC);
    hals_adc_flag_clear(ADC_FLAG_STRC);

    /* set ADC states */
    _adc_state_set(adc_dev, HAL_ADC_STATE_ROUTINE_EOC);
    if((ADC_EXTTRIG_ROUTINE_NONE == __HAL_ADC_GET_ROUTINECH_EXTTRIGGER) &&
            (DISABLE == __HAL_ADC_GET_CONTINUOUS_MODE)) {
        _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);
        if(RESET == ((hal_adc_state_get(adc_dev)) & HAL_ADC_STATE_INSERTED_BUSY)) {
            _adc_state_set(adc_dev, HAL_ADC_STATE_READY);
        }
    }

    /* unlock ADC */
    HAL_UNLOCK(adc_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      enable ADC routine sequence software trigger
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_adc_routine_software_trigger_enable(hal_adc_dev_struct *adc_dev)
{
    hals_adc_software_trigger_enable(ADC_ROUTINE_CHANNEL);
}

/*!
    \brief      start ADC EOC interrupt
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: EOC interrupt callback function structure
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_start_interrupt(hal_adc_dev_struct *adc_dev, hal_adc_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [adc_dev] or [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* lock ADC */
    HAL_LOCK(adc_dev);

    /* if ADC is enabled */
    if(HAL_ERR_NONE == _adc_enable(adc_dev)) {
        /* set ADC state */
        _adc_state_clear(adc_dev, HAL_ADC_STATE_READY);
        _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_EOC);
        _adc_state_set(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);

        /* if ADC configured in automatic inserted mode */
        if(SET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV) {
            _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_EOC);
            _adc_state_set(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
        }
        /* check if an inserted conversion is ongoing */
        if(SET == ((hal_adc_state_get(adc_dev)) & HAL_ADC_STATE_INSERTED_BUSY)) {
            _adc_error_clear(adc_dev, HAL_ADC_ERROR_DMA);
        } else {
            adc_dev->error_state = HAL_ADC_ERROR_NONE;
        }

        adc_dev->adc_irq.adc_eoc_handle = p_irq->adc_eoc_handle;

        /* clear ADC interrupt flag */
        hals_adc_interrupt_flag_clear(ADC_INT_FLAG_EOC);
        /* enable ADC end of sequence conversion flag */
        hals_adc_interrupt_enable(ADC_INT_EOC);
        /* enable the conversion of routine sequence */
        if(ADC_EXTTRIG_ROUTINE_NONE == __HAL_ADC_GET_ROUTINECH_EXTTRIGGER) {
            /* enable software trigger */
            hals_adc_external_trigger_config(ADC_ROUTINE_CHANNEL, ENABLE);
            hals_adc_software_trigger_enable(ADC_ROUTINE_CHANNEL);
        } else {
            /* enable external trigger */
            hals_adc_external_trigger_config(ADC_ROUTINE_CHANNEL, ENABLE);
        }
    } else {
        return HAL_ERR_HARDWARE;
    }

    /* unlock ADC */
    HAL_UNLOCK(adc_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop ADC EOC interrupt
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_stop_interrupt(hal_adc_dev_struct *adc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is disabled */
    if(HAL_ERR_NONE == _adc_disable(adc_dev)) {
        /* end of sequence conversion interrupt */
        hals_adc_interrupt_disable(ADC_INT_EOC);
        adc_dev->adc_irq.adc_eoc_handle = NULL;

        /* set ADC state */
        _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);
        _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
        _adc_state_set(adc_dev, HAL_ADC_STATE_READY);
    } else {
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      enable ADC and start the conversion of routine sequence with DMA
    \param[in]  adc_dev: ADC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  pdata: the source memory buffer address
    \param[in]  length: the number of data to be transferred from source to destination
    \param[in]  dmacb: ADC DMA transfer complete interrupt callback function structure
                  transcom_handle: DMA transfer complete interrupt handler
                  error_handle: ADC DMA error interrupt handler
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_HARDWARE, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_start_dma(hal_adc_dev_struct *adc_dev, uint32_t *pdata, uint32_t length, hal_adc_dma_handle_cb_struct *dmacb)
{
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc_dev) || (NULL == pdata) || (NULL == dmacb)) {
        HAL_DEBUGE("pointer [adc_dev] or [pdata] or [dmacb] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(0U == length) {
        HAL_DEBUGE("parameter [length] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* lock ADC */
    HAL_LOCK(adc_dev);

    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);

    /* if ADC is enabled */
    if(HAL_ERR_NONE == _adc_enable(adc_dev)) {
        /* set ADC state */
        _adc_state_clear(adc_dev, HAL_ADC_STATE_READY);
        _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_EOC);
        _adc_state_set(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);

        /* if conversions on routine sequence are also triggering inserted sequence */
        if(SET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV) {
            _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_EOC);
            _adc_state_set(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
        }
        /* check if an inserted conversion is ongoing */
        if(SET == ((hal_adc_state_get(adc_dev)) & HAL_ADC_STATE_INSERTED_BUSY)) {
            _adc_error_clear(adc_dev, HAL_ADC_ERROR_DMA);
        } else {
            adc_dev->error_state = HAL_ADC_ERROR_NONE;
        }

        /* clear ADC flag */
        hals_adc_flag_clear(ADC_FLAG_EOC);

        adc_dev->adc_dma.full_transcom_handle = dmacb->full_transcom_handle;
        adc_dev->adc_dma.half_transcom_handle = dmacb->half_transcom_handle;
        adc_dev->adc_dma.error_handle    = dmacb->error_handle;

        /* set the DMA transfer complete callback */
        dma_irq.full_finish_handle = _adc_dma_full_transfer_complete;
        dma_irq.half_finish_handle = _adc_dma_half_transfer_complete;
        dma_irq.error_handle       = _adc_dma_error;

        /* enable ADC DMA mode */
        hals_adc_dma_mode_enable();
        /* start the DMA channel */
        hal_dma_start_interrupt(adc_dev->p_dma_adc, (uint32_t)&ADC_RDATA, (uint32_t)pdata, length,
                                &dma_irq);

        /* enable the conversion of routine sequence */
        if(ADC_EXTTRIG_ROUTINE_NONE == __HAL_ADC_GET_ROUTINECH_EXTTRIGGER) {
            /* enable software trigger */
            hals_adc_external_trigger_config(ADC_ROUTINE_CHANNEL, ENABLE);
            hals_adc_software_trigger_enable(ADC_ROUTINE_CHANNEL);

        } else {
            /* enable external trigger */
            hals_adc_external_trigger_config(ADC_ROUTINE_CHANNEL, ENABLE);
        }
    } else {
        return HAL_ERR_HARDWARE;
    }

    /* unlock ADC */
    HAL_UNLOCK(adc_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop the conversion of routine sequence, disable ADC DMA mode and disable ADC
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_stop_dma(hal_adc_dev_struct *adc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is enabled */
    if(HAL_ERR_NONE == _adc_disable(adc_dev)) {
        /* disable ADC DMA mode */
        hals_adc_dma_mode_disable();
        /* disable the DMA channel */
        hal_dma_stop(adc_dev->p_dma_adc);

        /* reset DMA callback function interrupt handler */
        adc_dev->adc_dma.full_transcom_handle = NULL;
        adc_dev->adc_dma.half_transcom_handle = NULL;
        adc_dev->adc_dma.error_handle = NULL;

        /* set ADC state */
        _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);
        _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
        _adc_state_set(adc_dev, HAL_ADC_STATE_READY);
    } else {
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      initialize ADC inserted channel
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_ichannel: the pointer of ADC inserted sequence init structure
                  inserted_sequence_conversions: ENABLE, DISABLE
                  inserted_sequence_length: 1-4
                  inserted_sequence_external_trigger_select: inserted sequence trigger source
                  only one parameter can be selected which is shown as below:
      \arg          ADC_EXTTRIG_INSERTED_T0_TRGO: TIMER0 TRGO event select
      \arg          ADC_EXTTRIG_INSERTED_T0_CH3: TIMER0 CH3 event select
      \arg          ADC_EXTTRIG_INSERTED_T1_TRGO: TIMER1 TRGO event select
      \arg          ADC_EXTTRIG_INSERTED_T1_CH0: TIMER1 CH0 event select
      \arg          ADC_EXTTRIG_INSERTED_T2_CH3: TIMER2 CH3 event select
      \arg          ADC_EXTTRIG_INSERTED_T14_TRGO: TIMER14 TRGO event select
      \arg          ADC_EXTTRIG_INSERTED_EXTI_15: external interrupt line 15
      \arg          ADC_EXTTRIG_INSERTED_NONE: software trigger
                  auto_convert:
      \arg          ENABLE: enable auto convert
      \arg          DISABLE: disable auto convert
                  discontinuous_mode:
      \arg          ENABLE: enable discontinuous mode
      \arg          DISABLE: disable discontinuous mode
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_inserted_channel_config(hal_adc_dev_struct *adc_dev, hal_adc_inserted_config_struct *p_ichannel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc_dev) || (NULL == p_ichannel)) {
        HAL_DEBUGE("pointer [adc_dev] or [p_iinit] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* ADC inserted channel initialize */
    if(ENABLE == p_ichannel->inserted_sequence_conversions) {
        /* ADC inserted channel length config */
        hals_adc_channel_length_config(ADC_INSERTED_CHANNEL, p_ichannel->inserted_sequence_length);

        /* ADC inserted channel external trigger config */
        hals_adc_external_trigger_source_config(ADC_INSERTED_CHANNEL,
                                                p_ichannel->inserted_sequence_external_trigger_select);

        /* ADC inserted channel sequence convert automatically config */
        if(ENABLE == p_ichannel->auto_convert) {
            if(ADC_EXTTRIG_INSERTED_NONE == p_ichannel->inserted_sequence_external_trigger_select) {
                hals_adc_special_function_config(ADC_INSERTED_CHANNEL_AUTO, p_ichannel->auto_convert);
            } else {
                _adc_error_set(adc_dev, HAL_ADC_ERROR_CONFIG);
                return HAL_ERR_NO_SUPPORT;
            }
        } else {
            ADC_CTL0 &= ~((uint32_t)ADC_CTL0_ICA);
        }

        /* ADC discontinuous mode config */
        if(ENABLE == p_ichannel->discontinuous_mode) {
            /* inserted discontinuous can be enabled only if auto-inserted mode is disabled.   */
            if(DISABLE == p_ichannel->auto_convert) {
                ADC_CTL0 |= (uint32_t)ADC_CTL0_DISIC;
            } else {
                _adc_error_set(adc_dev, HAL_ADC_ERROR_CONFIG);
                return HAL_ERR_NO_SUPPORT;
            }
        } else {
            ADC_CTL0 &= ~((uint32_t)ADC_CTL0_DISIC);
        }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      configure ADC inserted rank
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irank: the pointer of ADC inserted channel configuration structure
                  channel: ADC_CHANNEL_x(x=0..18)
                  inserted_sequence:
                    the argument could be selected from enumeration <hal_adc_inserted_sequence_enum>
                  sampling_time: the sample time value
                  only one parameter can be selected which is shown as below:
      \arg          ADC_SAMPLETIME_1POINT5: 1.5 sampling cycles
      \arg          ADC_SAMPLETIME_7POINT5: 7.5 sampling cycles
      \arg          ADC_SAMPLETIME_13POINT5: 13.5 sampling cycles
      \arg          ADC_SAMPLETIME_28POINT5: 28.5 sampling cycles
      \arg          ADC_SAMPLETIME_41POINT5: 41.5 sampling cycles
      \arg          ADC_SAMPLETIME_55POINT5: 55.5 sampling cycles
      \arg          ADC_SAMPLETIME_71POINT5: 71.5 sampling cycles
      \arg          ADC_SAMPLETIME_239POINT5: 239.5 sampling cycles
                  data_offset: the offset data(0-0x0FFF)
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_inserted_rank_config(hal_adc_dev_struct *adc_dev, hal_adc_inserted_rank_config_struct *p_irank)
{
    uint32_t reg_temp;
    uint32_t length_temp;
    __IO uint32_t wait_loop = 0;
    uint8_t inserted_length;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc_dev) || (NULL == p_irank)) {
        HAL_DEBUGE("pointer [adc_dev] or [p_ichannel] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* ADC inserted channel length */
    length_temp = ((ADC_ISQ & ADC_ISQ_IL) >> 20U) + 1U;

    if(RESET == __HAL_ADC_GET_SCAN_MODE) {
        if(ADC_INSERTED_SEQUENCE_0 == p_irank->inserted_sequence) {
            /* ADC inserted channel config */
            hals_adc_inserted_channel_config(ADC_INSERTED_SEQUENCE_0, p_irank->channel, p_irank->sampling_time);
        } else {
            _adc_error_set(adc_dev, HAL_ADC_ERROR_CONFIG);
            return HAL_ERR_NO_SUPPORT;
        }
    } else {
        if(p_irank->inserted_sequence <= (length_temp - 1U)) {
            /* ADC inserted channel config */
            hals_adc_inserted_channel_config(p_irank->inserted_sequence, p_irank->channel,
                                             p_irank->sampling_time);
        } else {
            /* clear parameters */
            ADC_ISQ &= ~((uint32_t)ADC_ISQ_IL);

            inserted_length = (uint8_t)(length_temp - 1U);
            reg_temp = ADC_ISQ;
            reg_temp &= ~((uint32_t)(ADC_ISQ_ISQN << (15U - (inserted_length - (p_irank->inserted_sequence)) *
                                     5U)));
            ADC_ISQ = reg_temp;
        }
    }

    /* ADC inserted channel offset config */
    hals_adc_inserted_channel_offset_config(p_irank->inserted_sequence, p_irank->data_offset);

    /* enable temperature sensor and VREFINT measurement */
    if((ADC_CHANNEL_TEMPSENSOR == p_irank->channel) || (ADC_CHANNEL_VREFINT == p_irank->channel)) {
        if(RESET == __HAL_ADC_GET_TEMPVREF_ENABLE) {
            ADC_CTL1 |= ADC_CTL1_TSVREN;
            if(ADC_CHANNEL_TEMPSENSOR == p_irank->channel) {
                /* delay for temperature sensor stabilization time */
                /* compute number of CPU cycles to wait for */
                wait_loop = (ADC_TEMPSENSOR_DELAYTIME * (g_systemcoreclock / 1000000));
                while(wait_loop != 0) {
                    wait_loop--;
                }
            }
        }
    }
    /* enable backup battery voltage measurement */
    if(ADC_CHANNEL_BATTERY == p_irank->channel) {
        if(RESET == __HAL_ADC_GET_BATTERY_ENABLE) {
            /* enable the temperature sensor and Vrefint channel */
            ADC_CTL1 |= ADC_CTL1_VBETEN;
        }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      enable ADC and start the conversion of inserted sequence
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_inserted_start(hal_adc_dev_struct *adc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is enabled */
    if(HAL_ERR_NONE == _adc_enable(adc_dev)) {
        /* set ADC state  */
        _adc_state_clear(adc_dev, HAL_ADC_STATE_READY);
        _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_EOC);
        _adc_state_set(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
        /* check if a routine conversion is ongoing */
        if(RESET == ((hal_adc_state_get(adc_dev)) & HAL_ADC_STATE_ROUTINE_BUSY)) {
            adc_dev->error_state = HAL_ADC_ERROR_NONE;
        }

        /* clear ADC flag */
        hals_adc_flag_clear(ADC_FLAG_EOIC);

        if(RESET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV) {
            if(ADC_EXTTRIG_INSERTED_NONE == __HAL_ADC_GET_INSERTEDCH_EXTTRIGGER) {
                /* enable software trigger */
                hals_adc_external_trigger_config(ADC_INSERTED_CHANNEL, ENABLE);
                hals_adc_software_trigger_enable(ADC_INSERTED_CHANNEL);
            } else {
                /* enable external trigger */
                hals_adc_external_trigger_config(ADC_INSERTED_CHANNEL, ENABLE);
            }
        }
    } else {
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      stop the conversion of inserted sequence and disable ADC
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_inserted_stop(hal_adc_dev_struct *adc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* routine sequence is not busy and automatic convert is set */
    if((RESET == ((hal_adc_state_get(adc_dev)) & HAL_ADC_STATE_ROUTINE_BUSY)) &&
            (SET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV)) {
        /* if ADC is disabled */
        if(HAL_ERR_NONE == _adc_disable(adc_dev)) {
            /* set ADC state  */
            _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);
            _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
            _adc_state_set(adc_dev, HAL_ADC_STATE_READY);
        }
    } else {
        _adc_error_set(adc_dev, HAL_ADC_ERROR_CONFIG);
        return HAL_ERR_NO_SUPPORT;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      polling for ADC inserted sequence conversion, this function is just for single conversion
                (scan mode disabled or scan mode enabled with the length of inserted channel is 1).
    \param[in]  adc_dev: ADC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: time out duration in milliscond
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, HAL_ERR_TIMEOUT, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_inserted_conversion_poll(hal_adc_dev_struct *adc_dev, uint32_t timeout_ms)
{
    uint32_t tick_start = 0;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* lock ADC */
    HAL_LOCK(adc_dev);

    /* set timeout */
    tick_start = hal_sys_basetick_count_get();

    /* if single conversion: scan mode disabled or enabled with length = 1 */
    if((RESET == __HAL_ADC_GET_SCAN_MODE) && (RESET == __HAL_ADC_GET_INSERTEDCH_LENGTH)) {
        /* wait until end of inserted channel conversion flag is raised */
        while(RESET == hals_adc_flag_get(ADC_FLAG_EOIC)) {
            /* check for the timeout */
            if(HAL_TIMEOUT_FOREVER != timeout_ms) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                    _adc_state_set(adc_dev, HAL_ADC_STATE_TIMEOUT);
                    /* when timeout occurs, output timeout warning message */
                    HAL_DEBUGW("adc_dev inserted conversion poll timeout");
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
    } else {
        return HAL_ERR_NO_SUPPORT;
    }

    /* clear ADC EOIC/STIC/EOC flag */
    hals_adc_flag_clear(ADC_FLAG_EOIC);
    hals_adc_flag_clear(ADC_FLAG_STIC);
    hals_adc_flag_clear(ADC_FLAG_EOC);

    /* set ADC state */
    _adc_state_set(adc_dev, HAL_ADC_STATE_INSERTED_EOC);

    if((ADC_EXTTRIG_INSERTED_NONE == __HAL_ADC_GET_INSERTEDCH_EXTTRIGGER) ||
            ((RESET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV) \
             && ((ADC_EXTTRIG_ROUTINE_NONE == __HAL_ADC_GET_ROUTINECH_EXTTRIGGER) &&
                 (RESET == __HAL_ADC_GET_CONTINUOUS_MODE)))) {
        /* set ADC state */
        _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
        if(RESET == ((hal_adc_state_get(adc_dev)) & HAL_ADC_STATE_ROUTINE_BUSY)) {
            _adc_state_set(adc_dev, HAL_ADC_STATE_READY);
        }
    }
    /* unlock ADC */
    HAL_UNLOCK(adc_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      enable ADC inserted sequence software trigger
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_adc_inserted_software_trigger_enable(hal_adc_dev_struct *adc_dev)
{
    hals_adc_software_trigger_enable(ADC_INSERTED_CHANNEL);
}

/*!
    \brief      start ADC EOIC interrupt
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: EOIC interrupt callback function structure
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_inserted_start_interrupt(hal_adc_dev_struct *adc_dev, hal_adc_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [adc_dev] or [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* lock ADC */
    HAL_LOCK(adc_dev);

    /* if ADC is enabled */
    if(HAL_ERR_NONE == _adc_enable(adc_dev)) {
        _adc_state_clear(adc_dev, HAL_ADC_STATE_READY);
        _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_EOC);
        _adc_state_set(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);

        /* check if an inserted conversion is ongoing */
        if(RESET == ((hal_adc_state_get(adc_dev)) & HAL_ADC_STATE_ROUTINE_BUSY)) {
            adc_dev->error_state = HAL_ADC_ERROR_NONE;
        }

        adc_dev->adc_irq.adc_eoic_handle = p_irq->adc_eoic_handle;
        /* clear ADC interrupt flag */
        hals_adc_interrupt_flag_clear(ADC_INT_FLAG_EOIC);
        /* enable ADC end of sequence conversion flag */
        hals_adc_interrupt_enable(ADC_INT_EOIC);

        /* check if inserted sequence external trigger is enable*/
        if(RESET == ((ADC_CTL1) & (ADC_CTL1_ETEIC))) {
            /* enable software trigger */
            hals_adc_external_trigger_config(ADC_INSERTED_CHANNEL, ENABLE);
        }
        if(RESET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV) {
            if(ADC_EXTTRIG_INSERTED_NONE == __HAL_ADC_GET_INSERTEDCH_EXTTRIGGER) {
                /* enable software trigger */
                hals_adc_software_trigger_enable(ADC_INSERTED_CHANNEL);
            }
        }
    } else {
        return HAL_ERR_HARDWARE;
    }

    /* unlock ADC */
    HAL_UNLOCK(adc_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop the conversion of inserted sequence and disable ADC
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_inserted_stop_interrupt(hal_adc_dev_struct *adc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* routine sequence is not busy and automatic convert is set */
    if((RESET == ((hal_adc_state_get(adc_dev)) & HAL_ADC_STATE_ROUTINE_BUSY)) &&
            (SET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV)) {
        /* if ADC is disabled */
        if(HAL_ERR_NONE == _adc_disable(adc_dev)) {
            /* disable end of inserted sequence conversion interrupt */
            hals_adc_interrupt_disable(ADC_INT_EOIC);
            adc_dev->adc_irq.adc_eoic_handle = NULL;

            /* set ADC state */;
            _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);
            _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
            _adc_state_set(adc_dev, HAL_ADC_STATE_READY);
        }
    } else {
        _adc_error_set(adc_dev, HAL_ADC_ERROR_CONFIG);
        return HAL_ERR_NO_SUPPORT;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      configure watchdog
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_watchdog: the pointer of ADC watchdog configuration structure
                  routine_sequence_analog_watchdog:
      \arg          ENABLE: enable routine sequence analog watchdog
      \arg          DISABLE: disable routine sequence analog watchdog
                  inserted_sequence_analog_watchdog:
      \arg          ENABLE: enable inserted sequence analog watchdog
      \arg          DISABLE: disable inserted sequence analog watchdog
                  analog_watchdog_mode: ADC_WATCHDOG_MODE_ALL_CHANNELS, ADC_WATCHDOG_MODE_SINGLE_CHANNEL
      \arg          ADC_WATCHDOG_MODE_ALL_CHANNELS: Analog watchdog is effective on all channels
      \arg          ADC_WATCHDOG_MODE_SINGLE_CHANNEL: Analog watchdog is effective on a single channel
                  analog_watchdog_channel_select: ADC_CHANNEL_x(x=0..18)
                  analog_watchdog_low_threshold: analog watchdog low threshold, 0..4095
                  analog_watchdog_high_threshold: analog watchdog high threshold, 0..4095
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_watchdog_config(hal_adc_dev_struct *adc_dev, hal_adc_watchdog_config_struct *p_watchdog)
{
    uint32_t reg_temp;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc_dev) || (NULL == p_watchdog)) {
        HAL_DEBUGE("pointer [adc_dev] or [p_watchdog] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    reg_temp = ADC_CTL0;
    reg_temp &= (uint32_t)~(ADC_CTL0_RWDEN | ADC_CTL0_IWDEN | ADC_CTL0_WDSC);

    if(ENABLE == p_watchdog->routine_sequence_analog_watchdog) {
        reg_temp |= ADC_CTL0_RWDEN;
    }
    if(ENABLE == p_watchdog->inserted_sequence_analog_watchdog) {
        reg_temp |= ADC_CTL0_IWDEN;
    }
    if(ADC_WATCHDOG_MODE_SINGLE_CHANNEL == p_watchdog->analog_watchdog_mode) {
        reg_temp |= ADC_CTL0_WDSC;
        reg_temp &= (uint32_t)(~ADC_CTL0_WDCHSEL);
        reg_temp |= (uint32_t)(p_watchdog->analog_watchdog_channel_select);
    }
    ADC_CTL0 = reg_temp;

    /* ADC analog watchdog low threshold */
    ADC_WDLT = (uint32_t)WDLT_WDLT(p_watchdog->analog_watchdog_low_threshold);
    /* ADC analog watchdog high threshold */
    ADC_WDHT = (uint32_t)WDHT_WDHT(p_watchdog->analog_watchdog_high_threshold);

    return HAL_ERR_NONE;
}

/*!
    \brief      enable ADC watchdog interrupt
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: watchdog interrupt callback function structure
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_watchdog_interrupt_enable(hal_adc_dev_struct *adc_dev, hal_adc_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [adc_dev] or [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* ADC analog watchdog interrupt handle config */
    adc_dev->adc_irq.adc_watchdog_handle = p_irq->adc_watchdog_handle;
    /* clear ADC watchdog interrupt flag */
    hals_adc_interrupt_flag_clear(ADC_INT_FLAG_WDE);
    /* enable ADC watchdog interrupt */
    hals_adc_interrupt_enable(ADC_INT_WDE);

    return HAL_ERR_NONE;
}

/*!
    \brief      disable ADC watchdog interrupt
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_watchdog_interrupt_disable(hal_adc_dev_struct *adc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* disable ADC watchdog interrupt */
    hals_adc_interrupt_disable(ADC_INT_WDE);
    /* ADC analog watchdog interrupt handle config */
    adc_dev->adc_irq.adc_watchdog_handle = NULL;

    return HAL_ERR_NONE;
}

/*!
    \brief      polling for watchdog event
    \param[in]  adc_dev: ADC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: time out duration in milliscond
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, details refer to gd32f3x0_hal.h
*/
int32_t hal_adc_watchdog_event_poll(hal_adc_dev_struct *adc_dev, uint32_t timeout_ms)
{
    uint32_t tick_start = 0;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* set timeout */
    tick_start = hal_sys_basetick_count_get();

    /* check watchdog event flag */
    while(RESET == hals_adc_flag_get(ADC_FLAG_WDE)) {
        /* check for the timeout */
        if(HAL_TIMEOUT_FOREVER != timeout_ms) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) {
                _adc_state_set(adc_dev, HAL_ADC_STATE_TIMEOUT);
                /* when timeout occurs, output timeout warning message */
                HAL_DEBUGW("adc_dev watchdog event poll timeout");
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    /* set ADC state */
    _adc_state_set(adc_dev, HAL_ADC_STATE_WATCHDOG);

    /* clear ADC analog watchdog event flag */
    hals_adc_flag_clear(ADC_FLAG_WDE);

    return HAL_ERR_NONE;
}

/*!
    \brief      ADC interrupt handler content function, which is merely used in ADC_CMP_IRQHandler
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_adc_irq(hal_adc_dev_struct *adc_dev)
{
    /* EOC interrupt handler */
    if(SET == (hals_adc_interrupt_flag_get(ADC_INT_FLAG_EOC))) {
        /* clear EOC flag */
        hals_adc_interrupt_flag_clear(ADC_INT_FLAG_EOC);
        if(NULL != (adc_dev->adc_irq.adc_eoc_handle)) {
            adc_dev->adc_irq.adc_eoc_handle(adc_dev);
        }
        /* if not in error state */
        if(SET == ((hal_adc_error_get(adc_dev)) & HAL_ADC_ERROR_SYSTEM)) {
            _adc_state_set(adc_dev, HAL_ADC_STATE_ROUTINE_EOC);
        }

        if((ADC_EXTTRIG_ROUTINE_NONE == __HAL_ADC_GET_ROUTINECH_EXTTRIGGER) &&
                (RESET == __HAL_ADC_GET_CONTINUOUS_MODE)) {
            hals_adc_interrupt_disable(ADC_INT_EOC);
            /* set ADC state */
            _adc_state_clear(adc_dev, HAL_ADC_STATE_ROUTINE_BUSY);
            if(RESET == ((hal_adc_state_get(adc_dev)) & HAL_ADC_STATE_INSERTED_BUSY)) {
                _adc_state_set(adc_dev, HAL_ADC_STATE_READY);
            }
        }
    }
    /* EOIC interrupt handler */
    if(SET == (hals_adc_interrupt_flag_get(ADC_INT_FLAG_EOIC))) {
        /* clear EOIC flag */
        hals_adc_interrupt_flag_clear(ADC_INT_FLAG_EOIC);
        if(NULL != (adc_dev->adc_irq.adc_eoic_handle)) {
            adc_dev->adc_irq.adc_eoic_handle(adc_dev);
        }
        /* if not in error state */
        if(SET == ((hal_adc_error_get(adc_dev)) & HAL_ADC_ERROR_SYSTEM)) {
            _adc_state_set(adc_dev, HAL_ADC_STATE_INSERTED_EOC);
        }

        if((ADC_EXTTRIG_INSERTED_NONE == __HAL_ADC_GET_INSERTEDCH_EXTTRIGGER) ||
                ((RESET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV) \
                 && ((ADC_EXTTRIG_ROUTINE_NONE == __HAL_ADC_GET_ROUTINECH_EXTTRIGGER) &&
                     (RESET == __HAL_ADC_GET_CONTINUOUS_MODE)))) {
            hals_adc_interrupt_disable(ADC_INT_EOIC);
            /* set ADC state */
            _adc_state_clear(adc_dev, HAL_ADC_STATE_INSERTED_BUSY);
            if(RESET == ((hal_adc_state_get(adc_dev)) & HAL_ADC_STATE_ROUTINE_BUSY)) {
                _adc_state_set(adc_dev, HAL_ADC_STATE_READY);
            }
        }
    }
    /* watchdog interrupt handler */
    if(SET == (hals_adc_interrupt_flag_get(ADC_INT_FLAG_WDE))) {
        /* clear watchdog flag */
        hals_adc_interrupt_flag_clear(ADC_INT_FLAG_WDE);
        if(NULL != (adc_dev->adc_irq.adc_watchdog_handle)) {
            adc_dev->adc_irq.adc_watchdog_handle(adc_dev);
        }
        /* set ADC state */
        _adc_state_set(adc_dev, HAL_ADC_STATE_WATCHDOG);
    }
}

/*!
    \brief      set user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to ADC interrupt callback functions structure
      \arg        hal_irq_handle_cb function pointer: the function is user-defined,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
      \arg        NULL: The corresponding callback mechanism is out of use, and
                    disable corresponding interrupt
      \arg        HAL_INTERRUPT_ENABLE_ONLY: The corresponding callback mechanism is out of use,
                    while enable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_adc_irq_handle_set(hal_adc_dev_struct *adc_dev, hal_adc_irq_struct *p_irq)
{
    /* EOC interrupt handler set */
    if(NULL != p_irq->adc_eoc_handle) {
        adc_dev->adc_irq.adc_eoc_handle = p_irq->adc_eoc_handle;
        hals_adc_interrupt_enable(ADC_INT_EOC);
    } else {
        adc_dev->adc_irq.adc_eoc_handle = NULL;
        hals_adc_interrupt_enable(ADC_INT_EOC);
    }
    /* EOIC interrupt handler set */
    if(NULL != p_irq->adc_eoic_handle) {
        adc_dev->adc_irq.adc_eoic_handle = p_irq->adc_eoic_handle;
        hals_adc_interrupt_enable(ADC_INT_EOIC);
    } else {
        adc_dev->adc_irq.adc_eoic_handle = NULL;
        hals_adc_interrupt_disable(ADC_INT_EOIC);
    }
    /* watchdog interrupt handler set */
    if(NULL != p_irq->adc_watchdog_handle) {
        adc_dev->adc_irq.adc_watchdog_handle = p_irq->adc_watchdog_handle;
        hals_adc_interrupt_enable(ADC_INT_WDE);
    } else {
        adc_dev->adc_irq.adc_watchdog_handle = NULL;
        hals_adc_interrupt_disable(ADC_INT_WDE);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_adc_irq_handle_all_reset(hal_adc_dev_struct *adc_dev)
{
    /* ADC interrupt handler reset */
    adc_dev->adc_irq.adc_watchdog_handle = NULL;
    adc_dev->adc_irq.adc_eoc_handle      = NULL;
    adc_dev->adc_irq.adc_eoic_handle     = NULL;
}

/*!
    \brief      get ADC routine sequence conversion result
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the routine sequence conversion result(0-0xFFFF)
*/
uint16_t hal_adc_routine_value_get(hal_adc_dev_struct *adc_dev)
{
    /* return ADC routine sequence converted value */
    return ((uint16_t)ADC_RDATA);
}

/*!
    \brief      get ADC inserted sequence conversion result
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  inschannel_sequence: ADC inserted channel sequence
    \param[out] none
    \retval     the inserted sequence conversion result(0-0xFFFF)
*/
uint16_t hal_adc_inserted_value_get(hal_adc_dev_struct *adc_dev, uint8_t inschannel_sequence)
{
    uint16_t idata_temp = 0;

    /* clear ADC interrupt flag */
    hals_adc_flag_clear(ADC_FLAG_EOIC);

    /* read ADC converted value */
    idata_temp = hals_adc_inserted_data_read(inschannel_sequence);

    /* return ADC converted value */
    return idata_temp;
}

/*!
    \brief      get ADC error
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the error state
*/
uint32_t hal_adc_error_get(hal_adc_dev_struct *adc_dev)
{
    /* return ADC error */
    return (adc_dev->error_state);
}

/*!
    \brief      get ADC state
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the state
*/
uint32_t hal_adc_state_get(hal_adc_dev_struct *adc_dev)
{
    /* return ADC state */
    return (adc_dev->state);
}

/*!
    \brief      enable the ADC
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, details refer to gd32f3x0_hal.h
*/
static int32_t _adc_enable(hal_adc_dev_struct *adc_dev)
{
    uint32_t tick_start = 0;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is disabled */
    if(RESET == _adc_enable_state_get()) {
        /* enable ADC */
        hals_adc_enable();
        /* wait for ADC enable */
        hal_sys_basetick_delay_ms(1U);

        tick_start = hal_sys_basetick_count_get();

        /* wait for ADC actually enabled */
        while(RESET == _adc_enable_state_get()) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, ADC_ENABLE_DELAYTIME)) {
                _adc_error_set(adc_dev, HAL_ADC_ERROR_SYSTEM);
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      disable the ADC
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, details refer to gd32f3x0_hal.h
*/
static int32_t _adc_disable(hal_adc_dev_struct *adc_dev)
{
    uint32_t tick_start = 0;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc_dev) {
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is not already disabled */
    if(RESET != _adc_enable_state_get()) {
        /* disable ADC */
        hals_adc_disable();

        /* wait for ADC disable */
        tick_start = hal_sys_basetick_count_get();

        /* wait for ADC actually disabled */
        while(RESET != _adc_enable_state_get()) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, ADC_ENABLE_DELAYTIME)) {
                _adc_error_set(adc_dev, HAL_ADC_ERROR_SYSTEM);
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      get ADC enable state
    \param[in]  adc_dev: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the enable state: SET or RESET
*/
static FlagStatus _adc_enable_state_get(void)
{
    if(0U != (ADC_CTL1 & ADC_CTL1_ADCON)) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      ADC DMA full transmission complete callback
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _adc_dma_full_transfer_complete(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_adc_dev_struct *p_adc;

    p_dma = (hal_dma_dev_struct *)dma;
    p_adc = (hal_adc_dev_struct *)(p_dma->p_periph);

    if(RESET == ((hal_adc_error_get(p_adc)) & (HAL_ADC_ERROR_DMA | HAL_ADC_ERROR_SYSTEM))) {
        /* set ADC state */
        _adc_state_set(p_adc, HAL_ADC_STATE_ROUTINE_EOC);

        if((ADC_EXTTRIG_ROUTINE_NONE == __HAL_ADC_GET_ROUTINECH_EXTTRIGGER) &&
                (DISABLE == __HAL_ADC_GET_CONTINUOUS_MODE)) {
            /* set ADC state */
            _adc_state_clear(p_adc, HAL_ADC_STATE_ROUTINE_BUSY);
            if(RESET == ((hal_adc_state_get(p_adc)) & HAL_ADC_STATE_INSERTED_BUSY)) {
                _adc_state_set(p_adc, HAL_ADC_STATE_READY);
            }
        }

        if(NULL != (p_adc->adc_dma.full_transcom_handle)) {
            p_adc->adc_dma.full_transcom_handle(p_adc);
        }
    }
}

/*!
    \brief      ADC DMA half transmission complete callback
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _adc_dma_half_transfer_complete(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_adc_dev_struct *p_adc;

    p_dma = (hal_dma_dev_struct *)dma;
    p_adc = (hal_adc_dev_struct *)(p_dma->p_periph);

    if(RESET == ((hal_adc_error_get(p_adc)) & (HAL_ADC_ERROR_DMA | HAL_ADC_ERROR_SYSTEM))) {
        /* set ADC state */
        _adc_state_set(p_adc, HAL_ADC_STATE_ROUTINE_EOC);

        if((ADC_EXTTRIG_ROUTINE_NONE == __HAL_ADC_GET_ROUTINECH_EXTTRIGGER) &&
                (DISABLE == __HAL_ADC_GET_CONTINUOUS_MODE)) {
            /* set ADC state */
            _adc_state_clear(p_adc, HAL_ADC_STATE_ROUTINE_BUSY);
            if(RESET == ((hal_adc_state_get(p_adc)) & HAL_ADC_STATE_INSERTED_BUSY)) {
                _adc_state_set(p_adc, HAL_ADC_STATE_READY);
            }
        }

        if(NULL != (p_adc->adc_dma.half_transcom_handle)) {
            p_adc->adc_dma.half_transcom_handle(p_adc);
        }
    }
}

/*!
    \brief      ADC DMA error callback
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _adc_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_adc_dev_struct *p_adc;

    p_dma = (hal_dma_dev_struct *)dma;
    p_adc = (hal_adc_dev_struct *)p_dma->p_periph;

    if(NULL != (p_adc->adc_dma.error_handle)) {
        p_adc->adc_dma.error_handle(p_adc);
    }

    /* set ADC state and error */
    _adc_error_set(p_adc, HAL_ADC_ERROR_DMA);
}

/*!
    \brief      set ADC state
    \param[in]  d_adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  adc_state: the state of ADC
                  the argument could be selected from enumeration <hal_adc_state_enum>
    \param[out] none
    \retval     the ADC state
*/
static void _adc_state_set(hal_adc_dev_struct *d_adc, hal_adc_state_enum adc_state)
{
    /* set ADC state */
    d_adc->state |= (adc_state);
}

/*!
    \brief      clear ADC state
    \param[in]  d_adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  adc_state: the state of ADC
                  the argument could be selected from enumeration <hal_adc_state_enum>
    \param[out] none
    \retval     the ADC state
*/
static void _adc_state_clear(hal_adc_dev_struct *d_adc, hal_adc_state_enum adc_state)
{
    /* clear ADC state */
    d_adc->state &= ~(adc_state);
}

/*!
    \brief      set ADC error
    \param[in]  d_adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  adc_state: the state of ADC
                  the argument could be selected from enumeration <hal_adc_error_enum>
    \param[out] none
    \retval     the ADC error
*/
static void _adc_error_set(hal_adc_dev_struct *d_adc, hal_adc_error_enum adc_error)
{
    /* set ADC error */
    d_adc->error_state |= (adc_error);
}

/*!
    \brief      clear ADC error
    \param[in]  d_adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  adc_state: the state of ADC
                  the argument could be selected from enumeration <hal_adc_error_enum>
    \param[out] none
    \retval     the ADC error
*/
static void _adc_error_clear(hal_adc_dev_struct *d_adc, hal_adc_error_enum adc_error)
{
    /* clear ADC error */
    d_adc->error_state &= ~(adc_error);
}

/*!
    \brief      reset ADC
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_adc_deinit(void)
{
    hal_rcu_periph_reset_enable(RCU_ADCRST);
    hal_rcu_periph_reset_disable(RCU_ADCRST);
}

/*!
    \brief      enable ADC interface
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_adc_enable(void)
{
    if(RESET == (ADC_CTL1 & ADC_CTL1_ADCON)) {
        ADC_CTL1 |= (uint32_t)ADC_CTL1_ADCON;
    }
}

/*!
    \brief      disable ADC interface
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_adc_disable(void)
{
    ADC_CTL1 &= ~((uint32_t)ADC_CTL1_ADCON);
}

/*!
    \brief      ADC calibration and reset calibration
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_adc_calibration_enable(void)
{
    /* reset the selected ADC calibration register */
    ADC_CTL1 |= (uint32_t) ADC_CTL1_RSTCLB;
    /* check the RSTCLB bit state */
    while((ADC_CTL1 & ADC_CTL1_RSTCLB)) {
    }

    /* enable ADC calibration process */
    ADC_CTL1 |= ADC_CTL1_CLB;
    /* check the CLB bit state */
    while((ADC_CTL1 & ADC_CTL1_CLB)) {
    }
}

/*!
    \brief      enable DMA request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_adc_dma_mode_enable(void)
{
    ADC_CTL1 |= (uint32_t)(ADC_CTL1_DMA);
}

/*!
    \brief      disable DMA request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_adc_dma_mode_disable(void)
{
    ADC_CTL1 &= ~((uint32_t)ADC_CTL1_DMA);
}

/*!
    \brief      enable or disable ADC special function
    \param[in]  function: the function to configure
                one or more parameters can be selected which is shown as below:
      \arg        ADC_SCAN_MODE: scan mode select
      \arg        ADC_INSERTED_CHANNEL_AUTO: inserted channel convert automatically
      \arg        ADC_CONTINUOUS_MODE: continuous mode select
    \param[in]  newvalue: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void hals_adc_special_function_config(uint32_t function, ControlStatus newvalue)
{
    if(newvalue) {
        /* enable ADC scan mode */
        if(RESET != (function & ADC_SCAN_MODE)) {
            ADC_CTL0 |= ADC_SCAN_MODE;
        }
        /* enable ADC inserted channel convert automatically */
        if(RESET != (function & ADC_INSERTED_CHANNEL_AUTO)) {
            ADC_CTL0 |= ADC_INSERTED_CHANNEL_AUTO;
        }
        /* enable ADC continuous mode */
        if(RESET != (function & ADC_CONTINUOUS_MODE)) {
            ADC_CTL1 |= ADC_CONTINUOUS_MODE;
        }
    } else {
        /* disable ADC scan mode */
        if(RESET != (function & ADC_SCAN_MODE)) {
            ADC_CTL0 &= ~ADC_SCAN_MODE;
        }
        /* disable ADC inserted channel convert automatically */
        if(RESET != (function & ADC_INSERTED_CHANNEL_AUTO)) {
            ADC_CTL0 &= ~ADC_INSERTED_CHANNEL_AUTO;
        }
        /* disable ADC continuous mode */
        if(RESET != (function & ADC_CONTINUOUS_MODE)) {
            ADC_CTL1 &= ~ADC_CONTINUOUS_MODE;
        }
    }
}

/*!
    \brief      configure ADC data alignment
    \param[in]  data_alignment: data alignment select
                only one parameter can be selected which is shown as below:
      \arg        ADC_LSB_ALIGNMENT: LSB alignment
      \arg        ADC_MSB_ALIGNMENT: MSB alignment
    \param[out] none
    \retval     none
*/
void hals_adc_data_alignment_config(uint32_t data_alignment)
{
    if(ADC_LSB_ALIGNMENT != data_alignment) {
        ADC_CTL1 |= ADC_CTL1_DAL;
    } else {
        ADC_CTL1 &= ~((uint32_t)ADC_CTL1_DAL);
    }
}

/*!
    \brief      configure the length of routine channel sequence or inserted channel sequence
    \param[in]  channel_sequence: select the channel sequence
                only one parameter can be selected which is shown as below:
      \arg        ADC_ROUTINE_CHANNEL: routine channel sequence
      \arg        ADC_INSERTED_CHANNEL: inserted channel sequence
    \param[in]  length: the length of the channel
                        routine channel 1-16
                        inserted channel 1-4
    \param[out] none
    \retval     none
*/
void hals_adc_channel_length_config(uint8_t channel_sequence, uint32_t length)
{
    switch(channel_sequence) {
    case ADC_ROUTINE_CHANNEL:
        /* configure the length of routine channel sequence */
        ADC_RSQ0 &= ~((uint32_t)ADC_RSQ0_RL);
        ADC_RSQ0 |= RSQ0_RL((uint32_t)(length - 1U));
        break;
    case ADC_INSERTED_CHANNEL:
        /* configure the length of inserted channel sequence */
        ADC_ISQ &= ~((uint32_t)ADC_ISQ_IL);
        ADC_ISQ |= ISQ_IL((uint32_t)(length - 1U));
        break;
    default:
        break;
    }
}

/*!
    \brief      configure ADC routine channel
    \param[in]  rank: the argument could be selected from enumeration <hal_adc_routine_sequence_enum>
    \param[in]  channel: the selected ADC channel
                only one parameter can be selected which is shown as below:
      \arg        ADC_CHANNEL_x(x=0..18): ADC Channelx
    \param[in]  sample_time: the sample time value
                only one parameter can be selected which is shown as below:
      \arg        ADC_SAMPLETIME_1POINT5: 1.5 cycles
      \arg        ADC_SAMPLETIME_7POINT5: 7.5 cycles
      \arg        ADC_SAMPLETIME_13POINT5: 13.5 cycles
      \arg        ADC_SAMPLETIME_28POINT5: 28.5 cycles
      \arg        ADC_SAMPLETIME_41POINT5: 41.5 cycles
      \arg        ADC_SAMPLETIME_55POINT5: 55.5 cycles
      \arg        ADC_SAMPLETIME_71POINT5: 71.5 cycles
      \arg        ADC_SAMPLETIME_239POINT5: 239.5 cycles
    \param[out] none
    \retval     none
*/
void hals_adc_routine_channel_config(uint8_t rank, uint8_t channel, uint32_t sample_time)
{
    uint32_t rsq, sampt;

    /* configure ADC routine sequence */
    if(rank < 6U) {
        rsq = ADC_RSQ2;
        rsq &=  ~((uint32_t)(ADC_RSQX_RSQN << (5U * rank)));
        rsq |= ((uint32_t)channel << (5U * rank));
        ADC_RSQ2 = rsq;
    } else if(rank < 12U) {
        rsq = ADC_RSQ1;
        rsq &= ~((uint32_t)(ADC_RSQX_RSQN << (5U * (rank - 6U))));
        rsq |= ((uint32_t)channel << (5U * (rank - 6U)));
        ADC_RSQ1 = rsq;
    } else if(rank < 16U) {
        rsq = ADC_RSQ0;
        rsq &= ~((uint32_t)(ADC_RSQX_RSQN << (5U * (rank - 12U))));
        rsq |= ((uint32_t)channel << (5U * (rank - 12U)));
        ADC_RSQ0 = rsq;
    } else {
    }

    /* configure ADC sampling time */
    if(channel < 10U) {
        sampt = ADC_SAMPT1;
        sampt &= ~((uint32_t)(ADC_SAMPTX_SPTN << (3U * channel)));
        sampt |= (uint32_t)(sample_time << (3U * channel));
        ADC_SAMPT1 = sampt;
    } else if(channel < 19U) {
        sampt = ADC_SAMPT0;
        sampt &= ~((uint32_t)(ADC_SAMPTX_SPTN << (3U * (channel - 10U))));
        sampt |= (uint32_t)(sample_time << (3U * (channel - 10U)));
        ADC_SAMPT0 = sampt;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      configure ADC inserted channel
    \param[in]  rank: the argument could be selected from enumeration <hal_adc_inserted_sequence_enum>
    \param[in]  channel: the selected ADC channel
                only one parameter can be selected which is shown as below:
      \arg        ADC_CHANNEL_x(x=0..18): ADC Channelx
    \param[in]  sample_time: The sample time value
                only one parameter can be selected which is shown as below:
      \arg        ADC_SAMPLETIME_1POINT5: 1.5 cycles
      \arg        ADC_SAMPLETIME_7POINT5: 7.5 cycles
      \arg        ADC_SAMPLETIME_13POINT5: 13.5 cycles
      \arg        ADC_SAMPLETIME_28POINT5: 28.5 cycles
      \arg        ADC_SAMPLETIME_41POINT5: 41.5 cycles
      \arg        ADC_SAMPLETIME_55POINT5: 55.5 cycles
      \arg        ADC_SAMPLETIME_71POINT5: 71.5 cycles
      \arg        ADC_SAMPLETIME_239POINT5: 239.5 cycles
    \param[out] none
    \retval     none
*/
void hals_adc_inserted_channel_config(uint8_t rank, uint8_t channel, uint32_t sample_time)
{
    uint8_t inserted_length;
    uint32_t isq, sampt;

    inserted_length = (uint8_t)GET_BITS(ADC_ISQ, 20U, 21U);

    isq = ADC_ISQ;
    isq &= ~((uint32_t)(ADC_ISQ_ISQN << (15U - (inserted_length - rank) * 5U)));
    isq |= ((uint32_t)channel << (15U - (inserted_length - rank) * 5U));
    ADC_ISQ = isq;

    /* configure ADC sampling time */
    if(channel < 10U) {
        sampt = ADC_SAMPT1;
        sampt &= ~((uint32_t)(ADC_SAMPTX_SPTN << (3U * channel)));
        sampt |= (uint32_t) sample_time << (3U * channel);
        ADC_SAMPT1 = sampt;
    } else if(channel < 19U) {
        sampt = ADC_SAMPT0;
        sampt &= ~((uint32_t)(ADC_SAMPTX_SPTN << (3U * (channel - 10U))));
        sampt |= ((uint32_t)sample_time << (3U * (channel - 10U)));
        ADC_SAMPT0 = sampt;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      configure ADC inserted channel offset
    \param[in]  inserted_channel: insert channel select
                only one parameter can be selected which is shown as below:
      \arg        ADC_INSERTED_CHANNEL_0: ADC inserted channel 0
      \arg        ADC_INSERTED_CHANNEL_1: ADC inserted channel 1
      \arg        ADC_INSERTED_CHANNEL_2: ADC inserted channel 2
      \arg        ADC_INSERTED_CHANNEL_3: ADC inserted channel 3
    \param[in]  offset: the offset data
    \param[out] none
    \retval     none
*/
void hals_adc_inserted_channel_offset_config(uint8_t inserted_channel, uint16_t offset)
{
    uint8_t inserted_length;
    uint32_t num = 0U;

    inserted_length = (uint8_t)GET_BITS(ADC_ISQ, 20U, 21U);
    num = 3U - (inserted_length - inserted_channel);

    if(num <= 3U) {
        /* calculate the offset of the register */
        num = num * 4U;
        /* configure the offset of the selected channels */
        REG32((ADC) + 0x14U + num) = IOFFX_IOFF((uint32_t)offset);
    }
}

/*!
    \brief      enable or disable ADC external trigger
    \param[in]  channel_sequence: select the channel sequence
                one or more parameters can be selected which is shown as below:
      \arg        ADC_ROUTINE_CHANNEL: routine channel sequence
      \arg        ADC_INSERTED_CHANNEL: inserted channel sequence
    \param[in]  newvalue: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void hals_adc_external_trigger_config(uint8_t channel_sequence, ControlStatus newvalue)
{
    if(newvalue) {
        /* external trigger enable for routine channel */
        if(RESET != (channel_sequence & ADC_ROUTINE_CHANNEL)) {
            ADC_CTL1 |= ADC_CTL1_ETERC;
        }
        /* external trigger enable for inserted channel */
        if(RESET != (channel_sequence & ADC_INSERTED_CHANNEL)) {
            ADC_CTL1 |= ADC_CTL1_ETEIC;
        }
    } else {
        /* external trigger disable for routine channel */
        if(RESET != (channel_sequence & ADC_ROUTINE_CHANNEL)) {
            ADC_CTL1 &= ~ADC_CTL1_ETERC;
        }
        /* external trigger disable for inserted channel */
        if(RESET != (channel_sequence & ADC_INSERTED_CHANNEL)) {
            ADC_CTL1 &= ~ADC_CTL1_ETEIC;
        }
    }
}

/*!
    \brief      configure ADC external trigger source
    \param[in]  channel_sequence: select the channel sequence
                only one parameter can be selected which is shown as below:
      \arg        ADC_ROUTINE_CHANNEL: routine channel sequence
      \arg        ADC_INSERTED_CHANNEL: inserted channel sequence
    \param[in]  external_trigger_source: routine or inserted sequence trigger source
                only one parameter can be selected which is shown as below:
                for routine channel:
      \arg        ADC_EXTTRIG_ROUTINE_T0_CH0: TIMER0 CH0 event select
      \arg        ADC_EXTTRIG_ROUTINE_T0_CH1: TIMER0 CH1 event select
      \arg        ADC_EXTTRIG_ROUTINE_T0_CH2: TIMER0 CH2 event select
      \arg        ADC_EXTTRIG_ROUTINE_T1_CH1: TIMER1 CH1 event select
      \arg        ADC_EXTTRIG_ROUTINE_T2_TRGO: TIMER2 TRGO event select
      \arg        ADC_EXTTRIG_ROUTINE_T14_CH0:  TIMER14 CH0 event select
      \arg        ADC_EXTTRIG_ROUTINE_EXTI_11: external interrupt line 11
      \arg        ADC_EXTTRIG_ROUTINE_NONE: software trigger
                for inserted channel:
      \arg        ADC_EXTTRIG_INSERTED_T0_TRGO: TIMER0 TRGO event select
      \arg        ADC_EXTTRIG_INSERTED_T0_CH3: TIMER0 CH3 event select
      \arg        ADC_EXTTRIG_INSERTED_T1_TRGO: TIMER1 TRGO event select
      \arg        ADC_EXTTRIG_INSERTED_T1_CH0: TIMER1 CH0 event select
      \arg        ADC_EXTTRIG_INSERTED_T2_CH3: TIMER2 CH3 event select
      \arg        ADC_EXTTRIG_INSERTED_T14_TRGO: TIMER14 TRGO event select
      \arg        ADC_EXTTRIG_INSERTED_EXTI_15: external interrupt line 15
      \arg        ADC_EXTTRIG_INSERTED_NONE: software trigger
    \param[out] none
    \retval     none
*/
void hals_adc_external_trigger_source_config(uint8_t channel_sequence, uint32_t external_trigger_source)
{
    switch(channel_sequence) {
    case ADC_ROUTINE_CHANNEL:
        /* external trigger select for routine channel */
        ADC_CTL1 &= ~((uint32_t)ADC_CTL1_ETSRC);
        ADC_CTL1 |= (uint32_t)external_trigger_source;
        break;
    case ADC_INSERTED_CHANNEL:
        /* external trigger select for inserted channel */
        ADC_CTL1 &= ~((uint32_t)ADC_CTL1_ETSIC);
        ADC_CTL1 |= (uint32_t)external_trigger_source;
        break;
    default:
        break;
    }
}

/*!
    \brief      enable ADC software trigger
    \param[in]  channel_sequence: select the channel sequence
                one or more parameters can be selected which is shown as below:
      \arg        ADC_ROUTINE_CHANNEL: routine channel sequence
      \arg        ADC_INSERTED_CHANNEL: inserted channel sequence
    \param[out] none
    \retval     none
*/
void hals_adc_software_trigger_enable(uint8_t channel_sequence)
{
    /* enable routine sequence software trigger */
    if(RESET != (channel_sequence & ADC_ROUTINE_CHANNEL)) {
        ADC_CTL1 |= ADC_CTL1_SWRCST;
    }
    /* enable inserted sequence software trigger */
    if(RESET != (channel_sequence & ADC_INSERTED_CHANNEL)) {
        ADC_CTL1 |= ADC_CTL1_SWICST;
    }
}

/*!
    \brief      read ADC inserted sequence data register
    \param[in]  inserted_channel: inserted channel select
                only one parameter can be selected which is shown as below:
      \arg        ADC_INSERTED_CHANNEL_0: ADC inserted channel 0
      \arg        ADC_INSERTED_CHANNEL_1: ADC inserted channel 1
      \arg        ADC_INSERTED_CHANNEL_2: ADC inserted channel 2
      \arg        ADC_INSERTED_CHANNEL_3: ADC inserted channel 3
    \param[out] none
    \retval     the conversion value
*/
uint16_t hals_adc_inserted_data_read(uint8_t inserted_channel)
{
    uint32_t idata;
    /* read the data of the selected channel */
    switch(inserted_channel) {
    case ADC_INSERTED_CHANNEL_0:
        idata = ADC_IDATA0;
        break;
    case ADC_INSERTED_CHANNEL_1:
        idata = ADC_IDATA1;
        break;
    case ADC_INSERTED_CHANNEL_2:
        idata = ADC_IDATA2;
        break;
    case ADC_INSERTED_CHANNEL_3:
        idata = ADC_IDATA3;
        break;
    default:
        idata = 0U;
        break;
    }
    return (uint16_t)idata;
}

/*!
    \brief      configure ADC resolution
    \param[in]  resolution: ADC resolution
                only one parameter can be selected which is shown as below:
      \arg        ADC_RESOLUTION_12B: 12-bit ADC resolution
      \arg        ADC_RESOLUTION_10B: 10-bit ADC resolution
      \arg        ADC_RESOLUTION_8B: 8-bit ADC resolution
      \arg        ADC_RESOLUTION_6B: 6-bit ADC resolution
    \param[out] none
    \retval     none
*/
void hals_adc_resolution_config(uint32_t resolution)
{
    ADC_CTL0 &= ~((uint32_t)ADC_CTL0_DRES);
    ADC_CTL0 |= (uint32_t)resolution;
}

/*!
    \brief      configure ADC oversample mode
    \param[in]  mode: ADC oversampling mode
                only one parameter can be selected which is shown as below:
      \arg        ADC_OVERSAMPLING_ALL_CONVERT: all oversampled conversions for a channel are done consecutively after a trigger
      \arg        ADC_OVERSAMPLING_ONE_CONVERT: each oversampled conversion for a channel needs a trigger
    \param[in]  shift: ADC oversampling shift
                only one parameter can be selected which is shown as below:
      \arg        ADC_OVERSAMPLING_SHIFT_NONE: no oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_1B: 1-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_2B: 2-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_3B: 3-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_4B: 3-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_5B: 5-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_6B: 6-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_7B: 7-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_8B: 8-bit oversampling shift
    \param[in]  ratio: ADC oversampling ratio
                only one parameter can be selected which is shown as below:
      \arg        ADC_OVERSAMPLING_RATIO_MUL2: oversampling ratio multiple 2
      \arg        ADC_OVERSAMPLING_RATIO_MUL4: oversampling ratio multiple 4
      \arg        ADC_OVERSAMPLING_RATIO_MUL8: oversampling ratio multiple 8
      \arg        ADC_OVERSAMPLING_RATIO_MUL16: oversampling ratio multiple 16
      \arg        ADC_OVERSAMPLING_RATIO_MUL32: oversampling ratio multiple 32
      \arg        ADC_OVERSAMPLING_RATIO_MUL64: oversampling ratio multiple 64
      \arg        ADC_OVERSAMPLING_RATIO_MUL128: oversampling ratio multiple 128
      \arg        ADC_OVERSAMPLING_RATIO_MUL256: oversampling ratio multiple 256
    \param[out] none
    \retval     none
*/
void hals_adc_oversample_mode_config(uint8_t mode, uint16_t shift, uint8_t ratio)
{
    /* configure ADC oversampling mode */
    if(ADC_OVERSAMPLING_ONE_CONVERT == mode) {
        ADC_OVSAMPCTL |= (uint32_t)ADC_OVSAMPCTL_TOVS;
    } else {
        ADC_OVSAMPCTL &= ~((uint32_t)ADC_OVSAMPCTL_TOVS);
    }

    /* configure the shift and ratio */
    ADC_OVSAMPCTL &= ~((uint32_t)(ADC_OVSAMPCTL_OVSR | ADC_OVSAMPCTL_OVSS));
    ADC_OVSAMPCTL |= ((uint32_t)shift | (uint32_t)ratio);
}

/*!
    \brief      enable ADC oversample mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_adc_oversample_mode_enable(void)
{
    ADC_OVSAMPCTL |= ADC_OVSAMPCTL_OVSEN;
}

/*!
    \brief      disable ADC oversample mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_adc_oversample_mode_disable(void)
{
    ADC_OVSAMPCTL &= ~((uint32_t)ADC_OVSAMPCTL_OVSEN);
}

/*!
    \brief      get the ADC flag bits
    \param[in]  flag: the adc flag bits
                only one parameter can be selected which is shown as below:
      \arg        ADC_FLAG_WDE: analog watchdog event flag
      \arg        ADC_FLAG_EOC: end of sequence conversion flag
      \arg        ADC_FLAG_EOIC: end of inserted sequence conversion flag
      \arg        ADC_FLAG_STIC: start flag of inserted channel sequence
      \arg        ADC_FLAG_STRC: start flag of routine channel sequence
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_adc_flag_get(uint32_t flag)
{
    FlagStatus reval = RESET;

    if(ADC_STAT & flag) {
        reval = SET;
    }
    return reval;
}

/*!
    \brief      clear the ADC flag
    \param[in]  flag: the adc flag
                one or more parameters can be selected which is shown as below:
      \arg        ADC_FLAG_WDE: analog watchdog event flag
      \arg        ADC_FLAG_EOC: end of sequence conversion flag
      \arg        ADC_FLAG_EOIC: end of inserted sequence conversion flag
      \arg        ADC_FLAG_STIC: start flag of inserted channel sequence
      \arg        ADC_FLAG_STRC: start flag of routine channel sequence
    \param[out] none
    \retval     none
*/
void hals_adc_flag_clear(uint32_t flag)
{
    ADC_STAT &= ~((uint32_t)flag);
}

/*!
    \brief      enable ADC interrupt
    \param[in]  interrupt: the adc interrupt
                one or more parameters can be selected which is shown as below:
      \arg        ADC_INT_WDE: analog watchdog interrupt
      \arg        ADC_INT_EOC: end of sequence conversion interrupt
      \arg        ADC_INT_EOIC: end of inserted sequence conversion interrupt
    \param[out] none
    \retval     none
*/
void hals_adc_interrupt_enable(uint32_t interrupt)
{
    /* enable analog watchdog interrupt */
    if(RESET != (interrupt & ADC_INT_WDE)) {
        ADC_CTL0 |= (uint32_t)ADC_CTL0_WDEIE;
    }

    /* enable end of sequence conversion interrupt */
    if(RESET != (interrupt & ADC_INT_EOC)) {
        ADC_CTL0 |= (uint32_t)ADC_CTL0_EOCIE;
    }

    /* enable end of inserted sequence conversion interrupt */
    if(RESET != (interrupt & ADC_INT_EOIC)) {
        ADC_CTL0 |= (uint32_t)ADC_CTL0_EOICIE;
    }
}

/*!
    \brief      disable ADC interrupt
    \param[in]  interrupt: the adc interrupt flag
                one or more parameters can be selected which is shown as below:
      \arg        ADC_INT_WDE: analog watchdog interrupt
      \arg        ADC_INT_EOC: end of sequence conversion interrupt
      \arg        ADC_INT_EOIC: end of inserted sequence conversion interrupt
    \param[out] none
    \retval     none
*/
void hals_adc_interrupt_disable(uint32_t interrupt)
{
    /* disable analog watchdog interrupt */
    if(RESET != (interrupt & ADC_INT_WDE)) {
        ADC_CTL0 &= ~(uint32_t)ADC_CTL0_WDEIE;
    }

    /* disable end of sequence conversion interrupt */
    if(RESET != (interrupt & ADC_INT_EOC)) {
        ADC_CTL0 &= ~(uint32_t)ADC_CTL0_EOCIE;
    }

    /* disable end of inserted sequence conversion interrupt */
    if(RESET != (interrupt & ADC_INT_EOIC)) {
        ADC_CTL0 &= ~(uint32_t)ADC_CTL0_EOICIE;
    }
}

/*!
    \brief      get the ADC interrupt flag
    \param[in]  flag: the adc interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        ADC_INT_FLAG_WDE: analog watchdog interrupt flag
      \arg        ADC_INT_FLAG_EOC: end of sequence conversion interrupt flag
      \arg        ADC_INT_FLAG_EOIC: end of inserted sequence conversion interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_adc_interrupt_flag_get(uint32_t flag)
{
    FlagStatus interrupt_flag = RESET;
    uint32_t state;

    /* check the interrupt bits */
    switch(flag) {
    case ADC_INT_FLAG_WDE:
        state = ADC_STAT & ADC_STAT_WDE;
        if((ADC_CTL0 & ADC_CTL0_WDEIE) && state) {
            interrupt_flag = SET;
        }
        break;
    case ADC_INT_FLAG_EOC:
        state = ADC_STAT & ADC_STAT_EOC;
        if((ADC_CTL0 & ADC_CTL0_EOCIE) && state) {
            interrupt_flag = SET;
        }
        break;
    case ADC_INT_FLAG_EOIC:
        state = ADC_STAT & ADC_STAT_EOIC;
        if((ADC_CTL0 & ADC_CTL0_EOICIE) && state) {
            interrupt_flag = SET;
        }
        break;
    default:
        break;
    }
    return interrupt_flag;
}

/*!
    \brief      clear ADC interrupt flag
    \param[in]  flag: the adc interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        ADC_INT_FLAG_WDE: analog watchdog interrupt flag
      \arg        ADC_INT_FLAG_EOC: end of sequence conversion interrupt flag
      \arg        ADC_INT_FLAG_EOIC: end of inserted sequence conversion interrupt flag
    \param[out] none
    \retval     none
*/
void hals_adc_interrupt_flag_clear(uint32_t flag)
{
    ADC_STAT &= ~((uint32_t)flag);
}
