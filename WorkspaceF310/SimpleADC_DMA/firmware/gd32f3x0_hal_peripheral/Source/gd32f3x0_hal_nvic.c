/*!
    \file    gd32f3x0_hal_nvic.c
    \brief   NVIC driver

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
    \brief      set the priority group
    \param[in]  nvic_prigroup: the NVIC priority group
                only one parameter can be selected which is shown as below:
      \arg        NVIC_PRIGROUP_PRE0_SUB4:0 bits for pre-emption priority 4 bits for subpriority
      \arg        NVIC_PRIGROUP_PRE1_SUB3:1 bits for pre-emption priority 3 bits for subpriority
      \arg        NVIC_PRIGROUP_PRE2_SUB2:2 bits for pre-emption priority 2 bits for subpriority
      \arg        NVIC_PRIGROUP_PRE3_SUB1:3 bits for pre-emption priority 1 bits for subpriority
      \arg        NVIC_PRIGROUP_PRE4_SUB0:4 bits for pre-emption priority 0 bits for subpriority
    \param[out] none
    \retval     none
*/
void hal_nvic_irq_priority_group_set(uint32_t nvic_prigroup)
{
    /* set the priority group value */
    SCB->AIRCR = NVIC_AIRCR_VECTKEY_MASK | nvic_prigroup;
}

/*!
    \brief      enable NVIC request
    \param[in]  nvic_irq: the NVIC interrupt request, detailed in IRQn_Type
    \param[in]  nvic_irq_pre_priority: the pre-emption priority needed to set
    \param[in]  nvic_irq_sub_priority: the subpriority needed to set
    \param[out] none
    \retval     none
*/
void hal_nvic_irq_enable(IRQn_Type nvic_irq,
                         uint8_t nvic_irq_pre_priority,
                         uint8_t nvic_irq_sub_priority)
{
    uint32_t temp_priority = 0x00U, temp_pre = 0x00U, temp_sub = 0x00U;
    /* use the priority group value to get the temp_pre and the temp_sub */
    switch((SCB->AIRCR) & (uint32_t)0x700U) {
    case NVIC_PRIGROUP_PRE0_SUB4:
        temp_pre = 0U;
        temp_sub = 0x4U;
        break;
    case NVIC_PRIGROUP_PRE1_SUB3:
        temp_pre = 1U;
        temp_sub = 0x3U;
        break;
    case NVIC_PRIGROUP_PRE2_SUB2:
        temp_pre = 2U;
        temp_sub = 0x2U;
        break;
    case NVIC_PRIGROUP_PRE3_SUB1:
        temp_pre = 3U;
        temp_sub = 0x1U;
        break;
    case NVIC_PRIGROUP_PRE4_SUB0:
        temp_pre = 4U;
        temp_sub = 0x0U;
        break;
    default:
        hal_nvic_irq_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
        temp_pre = 2U;
        temp_sub = 0x2U;
        break;
    }
    /* get the temp_priority to fill the NVIC->IP register */
    temp_priority = (uint32_t)nvic_irq_pre_priority << (__NVIC_PRIO_BITS - temp_pre);
    temp_priority |= nvic_irq_sub_priority & (0x0FU >> (__NVIC_PRIO_BITS - temp_sub));
    temp_priority = temp_priority << __NVIC_PRIO_BITS;
    NVIC->IP[nvic_irq] = (uint8_t)temp_priority;
    /* enable the selected IRQ */
    NVIC->ISER[nvic_irq >> 0x05U] = (uint32_t)0x01U << (nvic_irq & (uint8_t)0x1FU);
}

/*!
    \brief      set system NVIC priority
    \param[in]  nvic_irq: the NVIC interrupt request, only support negative number in IRQn_Type
    \param[in]  nvic_irq_pre_priority: the pre-emption priority needed to set
    \param[in]  nvic_irq_sub_priority: the subpriority needed to set
    \param[out] none
    \retval     none
*/
void hal_nvic_set_priority(IRQn_Type nvic_irq,
                           uint8_t nvic_irq_pre_priority,
                           uint8_t nvic_irq_sub_priority)
{
    uint32_t prioritygroup = 0x00U;
    uint32_t temp_priority = 0x00U;

    prioritygroup = NVIC_GetPriorityGrouping();
    temp_priority = NVIC_EncodePriority(prioritygroup, nvic_irq_pre_priority, nvic_irq_sub_priority);
    NVIC_SetPriority(nvic_irq, temp_priority);
}

/*!
    \brief      disable NVIC request
    \param[in]  nvic_irq: the NVIC interrupt request, detailed in IRQn_Type
    \param[out] none
    \retval     none
*/
void hals_nvic_periph_irq_disable(IRQn_Type nvic_irq)
{
    /* disable the selected IRQ.*/
    NVIC->ICER[nvic_irq >> 0x05U] = (uint32_t)0x01U << (nvic_irq & (uint8_t)0x1FU);
}

/*!
    \brief      set the NVIC vector table base address
    \param[in]  nvic_vict_tab: the RAM or FLASH base address
                only one parameter can be selected which is shown as below:
      \arg        NVIC_VECTTAB_RAM: RAM base address
      \arg        NVIC_VECTTAB_FLASH: FLASH base address
    \param[in]  offset: vector table offset
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hals_nvic_vector_table_set(uint32_t nvic_vict_tab, uint32_t offset)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check nvic_vict_tab value */
    if((nvic_vict_tab == NVIC_VECTTAB_RAM) && offset >= ((uint32_t)0x00001BFF)) {
        HAL_DEBUGE("parameter [offset] value is out of range");
        return HAL_ERR_VAL;
    }

    /* check nvic_vict_tab value */
    if((nvic_vict_tab == NVIC_VECTTAB_FLASH) && offset >= ((uint32_t)0x0000FBFF)) {
        HAL_DEBUGE("parameter [offset] value is out of range");
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    SCB->VTOR = nvic_vict_tab | (offset & NVIC_VECTTAB_OFFSET_MASK);

    return HAL_ERR_NONE;
}

/*!
    \brief      set the state of the low power mode
    \param[in]  lowpower_mode: the low power mode state
                only one parameter can be selected which is shown as below:
      \arg        SCB_LPM_SLEEP_EXIT_ISR: if chose this para, the system always enter low power
                    mode by exiting from ISR
      \arg        SCB_LPM_DEEPSLEEP: if chose this para, the system will enter the DEEPSLEEP mode
      \arg        SCB_LPM_WAKE_BY_ALL_INT: if chose this para, the lowpower mode can be woke up
                    by all the enable and disable interrupts
    \param[out] none
    \retval     none
*/
void hals_system_lowpower_set(uint8_t lowpower_mode)
{
    SCB->SCR |= (uint32_t)lowpower_mode;
}

/*!
    \brief      reset the state of the low power mode
    \param[in]  lowpower_mode: the low power mode state
                only one parameter can be selected which is shown as below:
      \arg        SCB_LPM_SLEEP_EXIT_ISR: if chose this para, the system will exit low power
                    mode by exiting from ISR
      \arg        SCB_LPM_DEEPSLEEP: if chose this para, the system will enter the SLEEP mode
      \arg        SCB_LPM_WAKE_BY_ALL_INT: if chose this para, the lowpower mode only can be
                    woke up by the enable interrupts
    \param[out] none
    \retval     none
*/
void hals_system_lowpower_reset(uint8_t lowpower_mode)
{
    SCB->SCR &= (~(uint32_t)lowpower_mode);
}

/*!
    \brief      set the systick clock source
    \param[in]  systick_clksource: the systick clock source needed to choose
                only one parameter can be selected which is shown as below:
      \arg        SYSTICK_CLKSOURCE_HCLK: systick clock source is from HCLK
      \arg        SYSTICK_CLKSOURCE_HCLK_DIV8: systick clock source is from HCLK/8
    \param[out] none
    \retval     none
*/

void hals_systick_clksource_set(uint32_t systick_clksource)
{
    if(SYSTICK_CLKSOURCE_HCLK == systick_clksource) {
        /* set the systick clock source from HCLK */
        SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;
    } else {
        /* set the systick clock source from HCLK/8 */
        SysTick->CTRL &= SYSTICK_CLKSOURCE_HCLK_DIV8;
    }
}

