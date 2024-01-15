/*!
    \file    gd32f3x0_hal_wwdgt.c
    \brief   WWDGT driver

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
    \brief      initialize the parameters of WWDGT struct with the default values
    \param[in]  p_wwdgt_init: the initialization data needed to initialize WWDGT
    \param[out] none
    \retval     none
*/
void hal_wwdgt_struct_init(hal_wwdgt_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer [*p_struct] value is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    switch(hal_struct_type) {
    case HAL_WWDGT_INIT_STRUCT:
        /* initialize WWDGT initialization structure with the default values */
        ((hal_wwdgt_init_struct *)p_struct)->wwdgt_pre_select                        = WWDGT_PSC_DIV1;
        ((hal_wwdgt_init_struct *)p_struct)->wwdgt_cnt_value                         = 0x7F;
        ((hal_wwdgt_init_struct *)p_struct)->wwdgt_downcnt_value                     = 0x7F;
        break;
    case HAL_WWDGT_DEV_STRUCT:
        /* initialize WWDGT DEV structure with the default values */
        ((hal_wwdgt_dev_struct *)p_struct)->wwdgt_irq.early_wakeup_handle            = NULL;
        ((hal_wwdgt_dev_struct *)p_struct)->state                                    = HAL_WWDGT_STATE_NONE;
        ((hal_wwdgt_dev_struct *)p_struct)->mutex                                    = HAL_MUTEX_UNLOCKED;
        break;
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      initialize WWDGT
    \param[in]  wwdgt_dev: WWDGT device information structure
                the structure is not necessary to be reconfigured after structrue initialization,
                the structure parameters altering is automatically configured by core
    \param[in]  p_wwdgt_init: pointer to a hal_wwdgt_init_struct structure which contains
                parameters for initialization of the WWDGT peripheral
                members of the structure and the member values are shown as below:
                wwdgt_pre_select:
                only one parameter can be selected which is shown as below:
      \arg        WWDGT_PSC_DIV1 WWDGT prescaler set to 1
      \arg        WWDGT_PSC_DIV2 WWDGT prescaler set to 2
      \arg        WWDGT_PSC_DIV4 WWDGT prescaler set to 4
      \arg        WWDGT_PSC_DIV8 WWDGT prescaler set to 8
               wwdgt_cnt_value: 0x0 - 0xFFF
               wwdgt_Downcnt_value: 0x0 - 0xFFF
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_TIMEOUT, HAL_ERR_NONE,
                details refer to gd32f3x0_hal.h
*/
int32_t hal_wwdgt_init(hal_wwdgt_dev_struct *wwdgt_dev, hal_wwdgt_init_struct *p_wwdgt_init)
{
    uint32_t reg_cfg = 0x0U, reg_ctl = 0x0U;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == wwdgt_dev) || (NULL == p_wwdgt_init)) {
        HAL_DEBUGE("pointer [*cec_dev] or pointer [*cec] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_wwdgt_init) {
        HAL_DEBUGE("pointer [p_wwdgt_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }

    if((p_wwdgt_init->wwdgt_cnt_value > 0x7F) || (p_wwdgt_init->wwdgt_cnt_value < 0x40)) {
        HAL_DEBUGE("parameter [p_wwdgt_init->counter] value is invalid");
        return HAL_ERR_VAL;
    }

    if((p_wwdgt_init->wwdgt_downcnt_value > 0x7F) || (p_wwdgt_init->wwdgt_downcnt_value < 0x40)) {
        HAL_DEBUGE("parameter [p_wwdgt_init->window_value] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1== HAL_PARAMETER_CHECK */
    wwdgt_dev->state = HAL_WWDGT_STATE_BUSY;
    /* enable WWDGT clock */
    hal_rcu_periph_clk_enable(RCU_WWDGT);

    /* clear WIN and PSC bits, clear CNT bit */
    reg_cfg = WWDGT_CFG & (~((uint32_t)WWDGT_CFG_WIN | (uint32_t)WWDGT_CFG_PSC));
    reg_ctl = WWDGT_CTL & (~(uint32_t)WWDGT_CTL_CNT);

    /* configure WIN and PSC bits, configure CNT bit */
    reg_cfg |= (uint32_t)(WWDGT_CFG_WIN & p_wwdgt_init->wwdgt_downcnt_value);
    reg_cfg |= (uint32_t)(p_wwdgt_init->wwdgt_pre_select);
    reg_ctl |= (uint32_t)(WWDGT_CTL_CNT & p_wwdgt_init->wwdgt_cnt_value);

    WWDGT_CFG = (uint32_t)reg_cfg;
    WWDGT_CTL = (uint32_t)reg_ctl;

    return HAL_ERR_NONE;
}

/*!
    \brief      deinitialize WWDGT
    \param[in]  wwdgt_dev: WWDGT device information structure
                the structure is not necessary to be reconfigured after structrue initialization,
                the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_wwdgt_deinit(hal_wwdgt_dev_struct *wwdgt_dev)
{
    hal_rcu_periph_reset_enable(RCU_WWDGTRST);
    hal_rcu_periph_reset_disable(RCU_WWDGTRST);
    wwdgt_dev->state = HAL_WWDGT_STATE_RESET;
}

/*!
    \brief      WWDGT interrupt handler content function,which is merely used in wwdgt_handler
    \param[in]  dac_dev: WWDGT device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_wwdgt_irq(hal_wwdgt_dev_struct *wwdgt_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == wwdgt_dev) {
        HAL_DEBUGE("pointer [wwdgt_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* check wwdgt interrupt control bit */
    /* check wwdgt interrupt state bit */
    if(SET == hals_wwdgt_flag_get()) {
        /* change WWDGT state to error state */
        wwdgt_dev->state = HAL_WWDGT_STATE_ERROR;
        /* clear interrupt flag */
        hals_wwdgt_flag_clear();
        /* error callback */
        if(NULL != (wwdgt_dev->wwdgt_irq.early_wakeup_handle)) {
            wwdgt_dev->wwdgt_irq.early_wakeup_handle(wwdgt_dev);
        }
    }
}

/*!
    \brief      set user-defined interrupt callback function
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  wwdgt_dev: WWDGT device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to WWDGT interrupt callback functions structure
                  hal_irq_handle_cb: the function is user-defined,
    \param[out] none
    \retval     none
*/
void hal_wwdgt_irq_handle_set(hal_wwdgt_dev_struct *wwdgt_dev, hal_wwdgt_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == wwdgt_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [dac_dev] or pointer [p_irq] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* WWDGT interrupt handler set */
    if(NULL != p_irq->early_wakeup_handle) {
        wwdgt_dev->wwdgt_irq.early_wakeup_handle = p_irq->early_wakeup_handle;
        hals_wwdgt_interrupt_enable();
    } else {
        wwdgt_dev->wwdgt_irq.early_wakeup_handle = NULL;
        hals_wwdgt_interrupt_disable();
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  wwdgt_dev: WWDGT device information structure
                the structure is not necessary to be reconfigured after structrue initialization,
                the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_wwdgt_irq_handle_all_reset(hal_wwdgt_dev_struct *wwdgt_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == wwdgt_dev) {
        HAL_DEBUGE("pointer [wwdgt_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* WWDGT interrupt handler set */
    wwdgt_dev->wwdgt_irq.early_wakeup_handle = NULL;
}

/*!
    \brief      start WWDGT module function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_wwdgt_start(void)
{
    WWDGT_CTL |= WWDGT_CTL_WDGTEN;
}

/*!
    \brief      start WWDGT module function by interrupt method, the function is non-blocking
    \param[in]  wwdgt_dev: WWDGT device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to WWDGT interrupt callback functions structure
                  hal_irq_handle_cb: the function is user-defined,the corresponding callback mechanism is in use, 
                  and enable corresponding interrupt
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_wwdgt_start_interrupt(hal_wwdgt_dev_struct *wwdgt_dev, hal_wwdgt_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == wwdgt_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [dac_dev] or pointer [p_irq] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* WWDGT interrupt start */
    if(NULL != p_irq->early_wakeup_handle) {
        wwdgt_dev->wwdgt_irq.early_wakeup_handle = p_irq->early_wakeup_handle;
        hals_wwdgt_interrupt_enable();
    } else {
        return HAL_ERR_ADDRESS;
    }
    WWDGT_CTL |= WWDGT_CTL_WDGTEN;
    return HAL_ERR_NONE;
}

/*!
    \brief      reload the counter of WWDGT
    \param[in]  p_wwdgt_init: pointer to a hal_wwdgt_init_struct structure which contains
                parameters for initialization of the WWDGT peripheral
                members of the structure and the member values are shown as below:
                wwdgt_pre_select:
                only one parameter can be selected which is shown as below:
      \arg        WWDGT_PSC_DIV1 WWDGT prescaler set to 1
      \arg        WWDGT_PSC_DIV2 WWDGT prescaler set to 2
      \arg        WWDGT_PSC_DIV4 WWDGT prescaler set to 4
      \arg        WWDGT_PSC_DIV8 WWDGT prescaler set to 8
               wwdgt_cnt_value: 0x0 - 0xFFF
               wwdgt_Downcnt_value: 0x0 - 0xFFF
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_wwdgt_reload(hal_wwdgt_init_struct *p_wwdgt)
{
    uint32_t reg = 0x0U;

#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_wwdgt) {
        HAL_DEBUGE("pointer [p_wwdgt] address is invalid");
        return HAL_ERR_ADDRESS;
    }

    if((p_wwdgt->wwdgt_cnt_value > 0x7F) || (p_wwdgt->wwdgt_cnt_value < 0x40)) {
        HAL_DEBUGE("parameter [p_wwdgt->counter] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1== HAL_PARAMETER_CHECK */

    reg = WWDGT_CTL & (~(uint32_t)WWDGT_CTL_CNT);
    reg |= (uint32_t)(WWDGT_CTL_CNT & p_wwdgt->wwdgt_cnt_value);

    WWDGT_CTL = (uint32_t)reg;

    return HAL_ERR_NONE;
}

/*!
    \brief      check early wakeup interrupt state of WWDGT
    \param[in]  none
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_wwdgt_flag_get(void)
{
    if(RESET != (WWDGT_STAT & WWDGT_STAT_EWIF)) {
        return SET;
    }
    return RESET;
}

/*!
    \brief      clear early wakeup interrupt state of WWDGT
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_wwdgt_flag_clear(void)
{
    WWDGT_STAT &= (~(uint32_t)WWDGT_STAT_EWIF);
}

/*!
    \brief      enable early wakeup interrupt of WWDGT
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_wwdgt_interrupt_enable(void)
{
    WWDGT_CFG |= WWDGT_CFG_EWIE;
}

/*!
    \brief      disable early wakeup interrupt of WWDGT
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_wwdgt_interrupt_disable(void)
{
    WWDGT_CFG &= ~ WWDGT_CFG_EWIE;
}
