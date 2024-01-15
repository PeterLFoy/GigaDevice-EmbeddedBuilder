/*!
    \file    gd32f3x0_hal_pmu.c
    \brief   PMU driver

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

/*!
    \brief      reset PMU peripheral
    \param[in]  pmu_dev: PMU device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_pmu_deinit(hal_pmu_dev_struct *pmu_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == pmu_dev) {
        HAL_DEBUGE("pointer [pmu_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    pmu_dev->state = HAL_PMU_STATE_BUSY;
    /* deinit PMU */
    hal_rcu_periph_reset_enable(RCU_PMURST);
    hal_rcu_periph_reset_disable(RCU_PMURST);

    /* change PMU error state and state */
    pmu_dev->error_state = HAL_PMU_ERROR_NONE;
    pmu_dev->state = HAL_PMU_STATE_RESET;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize the PMU structure with the default values
    \param[in]  hal_struct_type: the argument could be selected from enumeration <hal_pmu_struct_type_enum>
    \param[out] p_struct: pointer to PMU structure that contains the configuration information
    \retval     none
*/
void hal_pmu_struct_init(hal_pmu_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer [*p_struct] value is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_PMU_INIT_STRUCT:
        /* initialize PMU initialization structure with the default values */
        ((hal_pmu_init_struct *)p_struct)->int_event_mode      = PMU_LVD_EVENT_MODE;
        ((hal_pmu_init_struct *)p_struct)->lvd_threshold       = PMU_LVDT_0;
        ((hal_pmu_init_struct *)p_struct)->trig_type           = PMU_LVD_TRIG_BOTH;
        break;
    case HAL_PMU_IRQ_STRUCT:
        /* initialize PMU IRQ structure with the default values */
        ((hal_pmu_lvd_irq_struct *)p_struct)->pmu_lvd_handle          = NULL;
        break;
    case HAL_PMU_DEV_STRUCT:
        /* initialize PMU device structure with the default values */
        ((hal_pmu_dev_struct *)p_struct)->pmu_lvd_irq.pmu_lvd_handle  = NULL;
        ((hal_pmu_dev_struct *)p_struct)->error_state                 = HAL_PMU_ERROR_NONE;
        ((hal_pmu_dev_struct *)p_struct)->state                       = HAL_PMU_STATE_NONE;
        ((hal_pmu_dev_struct *)p_struct)->mutex                       = HAL_MUTEX_UNLOCKED;
        ((hal_pmu_dev_struct *)p_struct)->priv                        = NULL;
        break;
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      configure EXTI_16 and then configure low voltage detector threshold
    \param[in]  pmu_dev: PMU device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out]  pmu_lvd_init: the data needed to configure LVD
                  int_event_mode: PMU_INT_MODE, PMU_EVENT_MODE
                  trig_type: PMU_TRIG_RISING, PMU_TRIG_FALLING, PMU_TRIG_BOTH
                  lvd_threshold:
                  the argument could be selected from enumeration <hal_pmu_lvd_voltage_enum>
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_pmu_lvd_init(hal_pmu_dev_struct *pmu_dev, hal_pmu_init_struct *pmu_lvd_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == pmu_dev) || (NULL == pmu_lvd_init)) {
        HAL_DEBUGE("pointer [pmu_dev] or pointer [pmu_lvd_init] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    pmu_dev->state = HAL_PMU_STATE_BUSY;

    hal_exti_internal_deinit(EXTI_LINE_16_LVD);

    /* set the EXTI mode and enable events from EXTI line 16 */
    if(PMU_LVD_EVENT_MODE == pmu_lvd_init->int_event_mode) {
        EXTI_EVEN |= (uint32_t)EXTI_EVEN_EVEN16;
    }

    /* set the EXTI trigger type */
    switch(pmu_lvd_init->trig_type) {
    case PMU_LVD_TRIG_RISING:
        EXTI_RTEN |= (uint32_t)EXTI_RTEN_RTEN16;
        EXTI_FTEN &= ~(uint32_t)EXTI_FTEN_FTEN16;
        break;
    case PMU_LVD_TRIG_FALLING:
        EXTI_RTEN &= ~(uint32_t)EXTI_RTEN_RTEN16;
        EXTI_FTEN |= (uint32_t)EXTI_FTEN_FTEN16;
        break;
    case PMU_LVD_TRIG_BOTH:
        EXTI_RTEN |= (uint32_t)EXTI_RTEN_RTEN16;
        EXTI_FTEN |= (uint32_t)EXTI_FTEN_FTEN16;
        break;
    default:
        HAL_DEBUGE("parameter [pmu_lvd_init->trig_type] value is invalid");
        break;
    }

    hals_pmu_lvd_select(pmu_lvd_init->lvd_threshold);

    /* change PMU error state */
    pmu_dev->error_state = HAL_PMU_ERROR_NONE;
    /* change PMU state */
    pmu_dev->state = HAL_PMU_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      start LVD detector
    \param[in]  pmu_dev: PMU device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_pmu_lvd_start(hal_pmu_dev_struct *pmu_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == pmu_dev) {
        HAL_DEBUGE("pointer [pmu_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    pmu_dev->state = HAL_PMU_STATE_BUSY;

    /* enable LVD */
    PMU_CTL |= PMU_CTL_LVDEN;

    pmu_dev->state = HAL_PMU_STATE_READY;
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      stop LVD detector
    \param[in]  pmu_dev: PMU device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_pmu_lvd_stop(hal_pmu_dev_struct *pmu_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == pmu_dev) {
        HAL_DEBUGE("pointer [pmu_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    pmu_dev->state = HAL_PMU_STATE_BUSY;

    /* disable LVD */
    PMU_CTL &= ~PMU_CTL_LVDEN;
    EXTI_EVEN &= (uint32_t)~EXTI_EVEN_EVEN16;
    EXTI_INTEN &= (uint32_t)~EXTI_INTEN_INTEN16;
    hals_exti_interrupt_flag_clear(EXTI_LVD_16);

    pmu_dev->state = HAL_PMU_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      enable wakeup pin
    \param[in]  pmu_dev: PMU device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  wakeup_pin:
                one or more parameters can be selected which are shown as below:
      \arg        PMU_WAKEUP_PIN0: WKUP Pin 0 (PA0)
      \arg        PMU_WAKEUP_PIN1: WKUP Pin 1 (PC13)
      \arg        PMU_WAKEUP_PIN4: WKUP Pin 4 (PC5)
      \arg        PMU_WAKEUP_PIN5: WKUP Pin 5 (PB5)
      \arg        PMU_WAKEUP_PIN6: WKUP Pin 6 (PB15)
    \param[out] none
    \retval     none
*/
int32_t hal_pmu_wakeup_pin_enable(hal_pmu_dev_struct *pmu_dev, uint32_t wakeup_pin)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == pmu_dev)) {
        HAL_DEBUGE("pointer [pmu_dev] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    pmu_dev->state = HAL_PMU_STATE_BUSY;

    PMU_CS |= wakeup_pin;

    /* change PMU error state */
    pmu_dev->error_state = HAL_PMU_ERROR_NONE;
    /* change PMU state */
    pmu_dev->state = HAL_PMU_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      disable wakeup pin
    \param[in]  pmu_dev: PMU device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  wakeup_pin:
                one or more parameters can be selected which are shown as below:
      \arg        PMU_WAKEUP_PIN0: WKUP Pin 0 (PA0)
      \arg        PMU_WAKEUP_PIN1: WKUP Pin 1 (PC13)
      \arg        PMU_WAKEUP_PIN4: WKUP Pin 4 (PC5)
      \arg        PMU_WAKEUP_PIN5: WKUP Pin 5 (PB5)
      \arg        PMU_WAKEUP_PIN6: WKUP Pin 6 (PB15)
    \param[out] none
    \retval     none
*/
int32_t hal_pmu_wakeup_pin_disable(hal_pmu_dev_struct *pmu_dev, uint32_t wakeup_pin)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == pmu_dev)) {
        HAL_DEBUGE("pointer [pmu_dev] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    pmu_dev->state = HAL_PMU_STATE_BUSY;

    PMU_CS &= ~(wakeup_pin);

    /* change PMU error state */
    pmu_dev->error_state = HAL_PMU_ERROR_NONE;
    /* change PMU state */
    pmu_dev->state = HAL_PMU_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      PMU interrupt handler content function
    \param[in]  pmu_dev: PMU device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_pmu_lvd_irq(hal_pmu_dev_struct *pmu_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == pmu_dev)) {
        HAL_DEBUGE("pointer [pmu_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    pmu_dev->state = HAL_PMU_STATE_BUSY;

    /* LVD interrupt handle */
    if(RESET != hals_exti_interrupt_flag_get(EXTI_LVD_16)) {
        hals_exti_interrupt_flag_clear(EXTI_LVD_16);

        if(NULL != pmu_dev->pmu_lvd_irq.pmu_lvd_handle) {
            pmu_dev->pmu_lvd_irq.pmu_lvd_handle(pmu_dev);
        }
    }
    /* change PMU error state */
    pmu_dev->error_state = HAL_PMU_ERROR_NONE;
    /* change PMU state */
    pmu_dev->state = HAL_PMU_STATE_READY;
}

/*!
    \brief      set user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  pmu_dev: PMU device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  irq_handle: the callback handler of PMU interrupt
    \param[out] none
    \retval     none
*/
void hal_pmu_lvd_irq_handle_set(hal_pmu_dev_struct *pmu_dev, hal_pmu_lvd_irq_struct *irq_handle)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == pmu_dev) || (NULL == irq_handle)) {
        HAL_DEBUGE("pointer [pmu_dev] or [irq_handle] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    if(NULL != irq_handle->pmu_lvd_handle) {
        pmu_dev->pmu_lvd_irq.pmu_lvd_handle = irq_handle->pmu_lvd_handle;
    } else {
        pmu_dev->pmu_lvd_irq.pmu_lvd_handle = NULL;
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  pmu_dev: PMU device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_pmu_lvd_irq_handle_all_reset(hal_pmu_dev_struct *pmu_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == pmu_dev)) {
        HAL_DEBUGE("pointer [pmu_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    pmu_dev->pmu_lvd_irq.pmu_lvd_handle = NULL;
}

/*!
    \brief      start PMU lvd with interrupt mode
    \param[in]  pmu_dev: PMU device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  irq_handle: the callback handler of PMU interrupt
    \param[out] none
    \retval     none
*/
int32_t hal_pmu_lvd_start_interrupt(hal_pmu_dev_struct *pmu_dev, hal_pmu_lvd_irq_struct *irq_handle)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == pmu_dev) || (NULL == irq_handle)) {
        HAL_DEBUGE("pointer [pmu_dev] or pointer [irq_handle] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    HAL_LOCK(pmu_dev);
    /* enable LVD */
    PMU_CTL |= PMU_CTL_LVDEN;

    pmu_dev->pmu_lvd_irq.pmu_lvd_handle = irq_handle->pmu_lvd_handle;
    /* enable interrupt mode */
    if(NULL != pmu_dev->pmu_lvd_irq.pmu_lvd_handle) {
        EXTI_INTEN |= (uint32_t)EXTI_INTEN_INTEN16;
    } else {
        HAL_DEBUGE("pointer [irq_handle] address is invalid");
        return HAL_ERR_ADDRESS;
    }

    HAL_UNLOCK(pmu_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop PMU lvd with interrupt mode
    \param[in]  pmu_dev: PMU device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  none
    \param[out] none
    \retval     none
*/
int32_t hal_pmu_lvd_stop_interrupt(hal_pmu_dev_struct *pmu_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == pmu_dev)) {
        HAL_DEBUGE("pointer [pmu_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    HAL_LOCK(pmu_dev);
    EXTI_INTEN &= (uint32_t)~EXTI_INTEN_INTEN16;
    pmu_dev->pmu_lvd_irq.pmu_lvd_handle = NULL;
    hals_exti_interrupt_flag_clear(EXTI_LVD_16);
    /* disable LVD */
    PMU_CTL &= ~PMU_CTL_LVDEN;
    HAL_UNLOCK(pmu_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      select low voltage detector threshold
    \param[in]  lvdt_n:
                only one parameter can be selected which is shown as below:
      \arg        PMU_LVDT_0: voltage threshold is 2.1V
      \arg        PMU_LVDT_1: voltage threshold is 2.3V
      \arg        PMU_LVDT_2: voltage threshold is 2.4V
      \arg        PMU_LVDT_3: voltage threshold is 2.6V
      \arg        PMU_LVDT_4: voltage threshold is 2.7V
      \arg        PMU_LVDT_5: voltage threshold is 2.9V
      \arg        PMU_LVDT_6: voltage threshold is 3.0V
      \arg        PMU_LVDT_7: voltage threshold is 3.1V
    \param[out] none
    \retval     none
*/
void hals_pmu_lvd_select(uint32_t lvdt_n)
{
    /* disable LVD */
    PMU_CTL &= ~PMU_CTL_LVDEN;
    /* clear LVDT bits */
    PMU_CTL &= ~PMU_CTL_LVDT;
    /* set LVDT bits according to lvdt_n */
    PMU_CTL |= lvdt_n;
}

/*!
    \brief      configure LDO output voltage
                these bits set by software when the main PLL closed
    \param[in]  ldo_output:
                only one parameter can be selected which is shown as below:
      \arg        PMU_LDOVS_LOW: LDO output voltage low mode
      \arg        PMU_LDOVS_MID: LDO output voltage mid mode
      \arg        PMU_LDOVS_HIGH: LDO output voltage high mode
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_TIMEOUT, HAL_ERR_NONE,
*/
void hals_pmu_ldo_output_select(uint32_t ldo_output)
{
    PMU_CTL &= ~PMU_CTL_LDOVS;
    PMU_CTL |= ldo_output;
}

/*!
    \brief      enable low-driver mode in deep-sleep mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_pmu_lowdriver_mode_enable(void)
{
    PMU_CTL &= ~PMU_CTL_LDEN;
    PMU_CTL |= PMU_LOWDRIVER_ENABLE;
}

/*!
    \brief      disable low-driver mode in deep-sleep mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_pmu_lowdriver_mode_disable(void)
{
    PMU_CTL &= ~PMU_CTL_LDEN;
    PMU_CTL |= PMU_LOWDRIVER_DISABLE;
}

/*!
    \brief      enable high-driver mode
                this bit set by software only when IRC8M or HXTAL used as system clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_pmu_highdriver_mode_enable(void)
{
    PMU_CTL |= PMU_CTL_HDEN;
}

/*!
    \brief      disable high-driver mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_pmu_highdriver_mode_disable(void)
{
    PMU_CTL &= ~PMU_CTL_HDEN;
}

/*!
    \brief      switch high-driver mode
                this bit set by software only when IRC8M or HXTAL used as system clock
    \param[in]  highdr_switch:
                only one parameter can be selected which is shown as below:
      \arg        PMU_HIGHDR_SWITCH_NONE: disable high-driver mode switch
      \arg        PMU_HIGHDR_SWITCH_EN: enable high-driver mode switch
    \param[out] none
    \retval     none
*/
void hals_pmu_highdriver_switch_select(uint32_t highdr_switch)
{
    /* wait for HDRF flag to be set */
    while(SET != hals_pmu_flag_get(PMU_FLAG_HDR)) {
    }
    PMU_CTL &= ~PMU_CTL_HDS;
    PMU_CTL |= highdr_switch;
}

/*!
    \brief      low-driver mode when use low power LDO
    \param[in]  mode:
                only one parameter can be selected which is shown as below:
      \arg        PMU_NORMALDR_LOWPWR: normal-driver when use low power LDO
      \arg        PMU_LOWDR_LOWPWR: low-driver mode enabled when LDEN is 11 and use low power LDO
    \param[out] none
    \retval     none
*/
void hals_pmu_lowpower_driver_config(uint32_t mode)
{
    PMU_CTL &= ~PMU_CTL_LDLP;
    PMU_CTL |= mode;
}

/*!
    \brief      low-driver mode when use normal power LDO
    \param[in]  mode:
                only one parameter can be selected which is shown as below:
      \arg        PMU_NORMALDR_NORMALPWR: normal-driver when use low power LDO
      \arg        PMU_LOWDR_NORMALPWR: low-driver mode enabled when LDEN is 11 and use low power LDO
    \param[out] none
    \retval     none
*/
void hals_pmu_normalpower_driver_config(uint32_t mode)
{
    PMU_CTL &= ~PMU_CTL_LDNP;
    PMU_CTL |= mode;
}

/*!
    \brief      PMU work at sleep mode
    \param[in]  sleepmodecmd:
                only one parameter can be selected which is shown as below:
      \arg        WFI_CMD: use WFI command
      \arg        WFE_CMD: use WFE command
    \param[out] none
    \retval     none
*/
void hals_pmu_to_sleepmode(uint8_t sleepmodecmd)
{
#if (1 == HAL_PARAMETER_CHECK)
    if((WFI_CMD != sleepmodecmd) && (WFE_CMD != sleepmodecmd)) {
        HAL_DEBUGE("parameter [sleepmodecmd] value is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    /* clear sleepdeep bit of Cortex-M4 system control register */
    SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);

    /* select WFI or WFE command to enter sleep mode */
    if(WFI_CMD == sleepmodecmd) {
        __WFI();
    } else {
        __WFE();
    }
}

/*!
    \brief      PMU work at deepsleep mode
    \param[in]  ldo:
                only one parameter can be selected which is shown as below:
      \arg        PMU_LDO_NORMAL: LDO operates normally when pmu enter deepsleep mode
      \arg        PMU_LDO_LOWPOWER: LDO work at low power mode when pmu enter deepsleep mode
    \param[in]  lowdrive:
                only one parameter can be selected which is shown as below:
      \arg        PMU_LOWDRIVER_ENABLE: low-driver mode enable in deep-sleep mode
      \arg        PMU_LOWDRIVER_DISABLE: low-driver mode disable in deep-sleep mode
    \param[in]  deepsleepmodecmd:
                only one parameter can be selected which is shown as below:
      \arg        WFI_CMD: use WFI command
      \arg        WFE_CMD: use WFE command
    \param[out] none
    \retval     none
*/
void hals_pmu_to_deepsleepmode(uint32_t ldo, uint32_t lowdrive, uint8_t deepsleepmodecmd)
{
    static uint32_t reg_snap[ 4 ];
#if (1 == HAL_PARAMETER_CHECK)
    if((PMU_LDO_NORMAL != ldo) && (PMU_LDO_LOWPOWER != ldo)) {
        HAL_DEBUGE("parameter [ldo] value is invalid");
        return;
    }
    if((WFI_CMD != deepsleepmodecmd) && (WFE_CMD != deepsleepmodecmd)) {
        HAL_DEBUGE("parameter [deepsleepmodecmd] value is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */


    /* clear stbmod and ldolp bits */
    PMU_CTL &= ~((uint32_t)(PMU_CTL_STBMOD | PMU_CTL_LDOLP | PMU_CTL_LDEN | PMU_CTL_LDNP |
                            PMU_CTL_LDLP));

    /* set ldolp bit according to pmu_ldo */
    PMU_CTL |= ldo;

    /* low drive mode config in deep-sleep mode */
    if(PMU_LOWDRIVER_ENABLE == lowdrive) {
        if(PMU_LDO_NORMAL == ldo) {
            PMU_CTL |= (uint32_t)(PMU_CTL_LDEN | PMU_CTL_LDNP);
        } else {
            PMU_CTL |= (uint32_t)(PMU_CTL_LDEN | PMU_CTL_LDLP);
        }
    }

    /* set sleepdeep bit of Cortex-M4 system control register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    reg_snap[0] = REG32(0xE000E010U);
    reg_snap[1] = REG32(0xE000E100U);
    reg_snap[2] = REG32(0xE000E104U);
    reg_snap[3] = REG32(0xE000E108U);

    REG32(0xE000E010U) &= 0x00010004U;
    REG32(0xE000E180U)  = 0XB7FFEF19U;
    REG32(0xE000E184U)  = 0XFFFFFBFFU;
    REG32(0xE000E188U)  = 0xFFFFFFFFU;

    /* select WFI or WFE command to enter deepsleep mode */
    if(WFI_CMD == deepsleepmodecmd) {
        __WFI();
    } else {
        __SEV();
        __WFE();
        __WFE();
    }

    REG32(0xE000E010U) = reg_snap[0];
    REG32(0xE000E100U) = reg_snap[1];
    REG32(0xE000E104U) = reg_snap[2];
    REG32(0xE000E108U) = reg_snap[3];

    /* reset sleepdeep bit of Cortex-M4 system control register */
    SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
}

/*!
    \brief      pmu work at standby mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_pmu_to_standbymode(void)
{
    /* set stbmod bit */
    PMU_CTL |= PMU_CTL_STBMOD;

    /* reset wakeup flag */
    PMU_CTL |= PMU_CTL_WURST;

    /* set sleepdeep bit of Cortex-M4 system control register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    REG32(0xE000E010U) &= 0x00010004U;
    REG32(0xE000E180U) = 0XFFFFFFFBU;
    REG32(0xE000E184U) = 0XFFFFFFFFU;
    REG32(0xE000E188U) = 0xFFFFFFFFU;

    /* select WFI command to enter standby mode */
    __WFI();
}

/*!
    \brief      enable backup domain write
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_pmu_backup_write_enable(void)
{
    PMU_CTL |= PMU_CTL_BKPWEN;
}

/*!
    \brief      disable backup domain write
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_pmu_backup_write_disable(void)
{
    PMU_CTL &= ~PMU_CTL_BKPWEN;
}

/*!
    \brief      get flag state
    \param[in]  flag:
                only one parameter can be selected which is shown as below:
      \arg        PMU_FLAG_WAKEUP: wakeup flag
      \arg        PMU_FLAG_STANDBY: standby flag
      \arg        PMU_FLAG_LVD: lvd flag
      \arg        PMU_FLAG_LDOVSR: LDO voltage select ready flag
      \arg        PMU_FLAG_HDR: high-driver ready flag
      \arg        PMU_FLAG_HDSR: high-driver switch ready flag
      \arg        PMU_FLAG_LDR: low-driver mode ready flag
    \param[out] none
    \retval     FlagStatus SET or RESET
*/
FlagStatus hals_pmu_flag_get(uint32_t flag)
{
    FlagStatus ret_status = RESET;

    if(PMU_CS & flag) {
        ret_status = SET;
    }

    return ret_status;
}

/*!
    \brief      clear flag bit
    \param[in]  flag:
                one or more parameters can be selected which are shown as below:
      \arg        PMU_FLAG_RESET_WAKEUP: reset wakeup flag
      \arg        PMU_FLAG_RESET_STANDBY: reset standby flag
    \param[out] none
    \retval     none
*/
void hals_pmu_flag_clear(uint32_t flag)
{
    if(RESET != (flag & PMU_FLAG_RESET_WAKEUP)) {
        /* reset wakeup flag */
        PMU_CTL |= PMU_CTL_WURST;
    }
    if(RESET != (flag & PMU_FLAG_RESET_STANDBY)) {
        /* reset standby flag */
        PMU_CTL |= PMU_CTL_STBRST;
    }
}
