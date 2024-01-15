/*!
    \file    gd32f3x0_hal_sys.c
    \brief   SYS driver

    \version 2023-08-01, V1.0.0, HAL firmware for GD32F3x0
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc.

    All rights reserved.

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

#define DBG_RESET_VAL            ((uint32_t)0x00000000U)       /*!< DBG reset value */

/* global variable, basetick timer source */
hal_sys_timebase_source_enum g_basetick_source = SYS_TIMEBASE_SOURCE_SYSTICK;
/* global variable, debug selection */
hal_sys_debug_cfg_enum       g_debug_selection = SYS_DEBUG_DISABLE;

/* the callback of basetick interrupt definition */
static hal_sys_basetick_irq_handle_cb _basetick_irq_handle = NULL;

/* the internal basetick counter */
static __IO uint32_t g_basetick_count = 0U;


/* the clock source of the basetick timer */
static const hal_rcu_periph_reset_enum _BASETICK_SOURCE_RESET[] =  BASETICK_SOURCE_TIMERX_RST;

/* the clock source of the basetick timer */
static const hal_rcu_periph_enum _BASETICK_SOURCE_CLK[] = BASETICK_SOURCE_TIMERX_CLK;
/* the peripheral of the basetick timer */

static const uint32_t _BASETICK_SOURCE_PERIPH[] = BASETICK_SOURCE_TIMERX_PERIPH;

/* the interrupt number of the correspond timer */
static const IRQn_Type _BASETICK_SOURCE_IRQN[] = BASETICK_SOURCE_TIMERX_IRQN;

/* static function declaration */
static void _sys_systick_init(uint32_t count_freq, uint32_t prio);
static void _sys_basetick_timer_init(hal_sys_timebase_source_enum source, \
                                     uint32_t count_freq, \
                                     uint8_t prio);

/*!
    \brief      deinitialize SYS
    \param[in]  none
    \param[out] none
    \retval     none
*/
int32_t hal_sys_deinit(void)
{
    /* deinitialize the DBG */
    SYS_DBG_CTL0 = DBG_RESET_VAL;
    SYS_DBG_CTL1 = DBG_RESET_VAL;

    /* deinitialize time basetick */
    if(SYS_TIMEBASE_SOURCE_SYSTICK == g_basetick_source) {
        SysTick->LOAD &= ~SysTick_LOAD_RELOAD_Msk;
        SysTick->VAL  &= ~SysTick_VAL_CURRENT_Msk;
        SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk |   \
                           SysTick_CTRL_CLKSOURCE_Msk | \
                           SysTick_CTRL_COUNTFLAG_Msk);
    } else if((SYS_TIMEBASE_SOURCE_TIMER0 < g_basetick_source) || \
              (SYS_TIMEBASE_SOURCE_TIMER16 > g_basetick_source)) {

        /* disable timer's interrupt */
        hals_nvic_periph_irq_disable(_BASETICK_SOURCE_IRQN[g_basetick_source]);
        hal_rcu_periph_reset_enable(_BASETICK_SOURCE_RESET[g_basetick_source]);
        hal_rcu_periph_reset_disable(_BASETICK_SOURCE_RESET[g_basetick_source]);
    } else {
        return HAL_ERR_VAL;
    }
    return HAL_ERR_NONE;
}
/*!
    \brief      initialize debug
    \param[in]  debug_cfg: select debug
      \arg        SYS_DEBUG_ DISABLE: debug diable
      \arg        SYS_DEBUG_SERIAL_WIRE: serial wire
    \param[out] none
    \retval     none
*/
int32_t hal_sys_debug_init(hal_sys_debug_cfg_enum debug_cfg)
{
    g_debug_selection = debug_cfg;
    if(SYS_DEBUG_SERIAL_WIRE == debug_cfg) {
    }
    return HAL_ERR_NONE;
}
/*!
    \brief      initialize basetick timer
    \param[in]  source: select the source timer
      \arg        SYS_TIMEBASE_SOURCE_TIMERx: x=0,1,2,5,13,14,15,16, use the TIMERx as the source
      \arg        SYS_TIMEBASE_SOURCE_SYSTICK: use the systick as the source
    \param[out] none
    \retval     none
*/
int32_t hal_sys_timesource_init(hal_sys_timebase_source_enum source)
{
    g_basetick_source = source;
    if(SYS_TIMEBASE_SOURCE_SYSTICK == source) {
        _sys_systick_init(HAL_BASETICK_RATE_HZ, 0U);
    } else if((SYS_TIMEBASE_SOURCE_TIMER0 < g_basetick_source) || \
              (SYS_TIMEBASE_SOURCE_TIMER16 > g_basetick_source)) {
        _sys_basetick_timer_init(source, HAL_BASETICK_RATE_HZ, 0U);
    } else {
        return HAL_ERR_VAL;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      get the basetick count
    \param[in]  none
    \param[out] none
    \retval     uint32_t: 0-0xFFFFFFFF
*/
uint32_t hal_sys_basetick_count_get(void)
{
    return (g_basetick_count);
}

/*!
    \brief      check whether the delay is finished
    \param[in]  time_start: the starting time point of the delay
    \param[in]  delay: the delay interval
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hal_sys_basetick_timeout_check(uint32_t time_start, uint32_t delay)
{
    if(g_basetick_count - time_start > delay) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      set the basetick delay
    \param[in]  time_ms: the timeout interval
    \param[out] none
    \retval     none
*/
void hal_sys_basetick_delay_ms(uint32_t time_ms)
{
    uint32_t delay;
    uint32_t time_start = g_basetick_count;

    delay = (time_ms * (uint32_t)((float)HAL_BASETICK_RATE_HZ / (float)1000U));

    while(g_basetick_count - time_start < delay) {

    }
}

/*!
    \brief      suspend the basetick timer
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_sys_basetick_suspend(void)
{
    if(SYS_TIMEBASE_SOURCE_SYSTICK == g_basetick_source) {
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    } else {
        hals_timer_disable(_BASETICK_SOURCE_PERIPH[g_basetick_source]);
    }
}

/*!
    \brief      resume the basetick timer
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_sys_basetick_resume(void)
{
    if(SYS_TIMEBASE_SOURCE_SYSTICK == g_basetick_source) {
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    } else {
        hals_timer_enable(_BASETICK_SOURCE_PERIPH[g_basetick_source]);
    }
}

/*!
    \brief      basetick interrupt handler content function, which is merely \
                used in SysTick_Handler or TIMERx_UP_IRQHandler
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_sys_basetick_irq(void)
{
    g_basetick_count++;

    switch(g_basetick_source) {
    case SYS_TIMEBASE_SOURCE_TIMER0:
        hals_timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
        break;
#if (defined(GD32F350) || defined(GD32F330))
    case SYS_TIMEBASE_SOURCE_TIMER1:
        hals_timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
        break;
#endif /* GD32F350 and GD32F330 */
    case SYS_TIMEBASE_SOURCE_TIMER2:
        hals_timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
        break;
#ifdef GD32F350
    case SYS_TIMEBASE_SOURCE_TIMER5:
        hals_timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
        break;
#endif/* GD32F350 */
    case SYS_TIMEBASE_SOURCE_TIMER13:
        hals_timer_interrupt_flag_clear(TIMER13, TIMER_INT_FLAG_UP);
        break;

    case SYS_TIMEBASE_SOURCE_TIMER14:
        hals_timer_interrupt_flag_clear(TIMER14, TIMER_INT_FLAG_UP);
        break;

    case SYS_TIMEBASE_SOURCE_TIMER15:
        hals_timer_interrupt_flag_clear(TIMER15, TIMER_INT_FLAG_UP);
        break;

    case SYS_TIMEBASE_SOURCE_TIMER16:
        hals_timer_interrupt_flag_clear(TIMER16, TIMER_INT_FLAG_UP);
        break;

    default:
        break;
    }
    if(NULL != _basetick_irq_handle) {
        _basetick_irq_handle();
    }

}

/*!
    \brief      set user-defined interrupt callback function, which will be \
                registered and called when corresponding interrupt be triggered
    \param[in]  irq_handler:
    \param[out] none
    \retval     none
*/
void hal_sys_basetick_irq_handle_set(hal_sys_basetick_irq_handle_cb irq_handler)
{
    if(NULL == irq_handler) {
        HAL_DEBUGE("callback function is invalid");
    }

    _basetick_irq_handle = irq_handler;
}

/*!
    \brief      reset all user-defined interrupt callback function, which will \
                be registered and called when corresponding interrupt be triggered
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_sys_basetick_irq_handle_all_reset(void)
{
    _basetick_irq_handle = NULL;
}

/*!
    \brief      read DBG_ID code register
    \param[in]  none
    \param[out] none
    \retval     uint32_t: 0-0xFFFFFFFF
*/
uint32_t hals_sys_dbg_id_get(void)
{
    return SYS_DBG_ID;
}

/*!
    \brief      enable low power behavior when the MCU is in debug mode
    \param[in]  dbg_low_power:
                one or more parameters can be selected which are shown as below:
      \arg        SYS_DBG_LOW_POWER_SLEEP: keep debugger connection during sleep mode
      \arg        SYS_DBG_LOW_POWER_DEEPSLEEP: keep debugger connection during deepsleep mode
      \arg        SYS_DBG_LOW_POWER_STANDBY: keep debugger connection during standby mode
    \param[out] none
    \retval     none
*/
void hals_sys_dbg_low_power_enable(uint32_t dbg_low_power)
{
    SYS_DBG_CTL0 |= dbg_low_power;
}

/*!
    \brief      disable low power behavior when the MCU is in debug mode
    \param[in]  dbg_low_power:
                one or more parameters can be selected which are shown as below:
      \arg        SYS_DBG_LOW_POWER_SLEEP: donot keep debugger connection during sleep mode
      \arg        SYS_DBG_LOW_POWER_DEEPSLEEP: donot keep debugger connection during deepsleep mode
      \arg        SYS_DBG_LOW_POWER_STANDBY: donot keep debugger connection during standby mode
    \param[out] none
    \retval     none
*/
void hals_sys_dbg_low_power_disable(uint32_t dbg_low_power)
{
    SYS_DBG_CTL0 &= ~dbg_low_power;
}

/*!
    \brief      enable peripheral behavior when the MCU is in debug mode
    \param[in]  dbg_periph: refer to hal_sys_dbg_periph_enum
                only one parameter can be selected which are shown as below:
      \arg        SYS_DBG_FWDGT_HOLD: debug FWDGT kept when core is halted
      \arg        SYS_DBG_WWDGT_HOLD: debug WWDGT kept when core is halted
      \arg        SYS_DBG_TIMERx_HOLD (x=0,1,2,5,13,14,15,16,TIMER5 is only available in GD32F350,
                  TIMER1 is only available in GD32F350 and GD32F330): hold TIMERx counter when core is halted
      \arg        SYS_DBG_I2Cx_HOLD (x=0,1): hold I2Cx smbus when core is halted
      \arg        SYS_DBG_RTC_HOLD: hold RTC calendar and wakeup counter when core is halted
    \param[out] none
    \retval     none
*/
void hals_sys_dbg_periph_enable(hal_sys_dbg_periph_enum dbg_periph)
{
    SYS_DBG_REG_VAL(dbg_periph) |= BIT(SYS_DBG_BIT_POS(dbg_periph));
}

/*!
    \brief      disable peripheral behavior when the MCU is in debug mode
    \param[in]  dbg_periph: refer to hal_sys_dbg_periph_enum
                only one parameter can be selected which are shown as below:
      \arg        SYS_DBG_FWDGT_HOLD: debug FWDGT kept when core is halted
      \arg        SYS_DBG_WWDGT_HOLD: debug WWDGT kept when core is halted
      \arg        SYS_DBG_TIMERx_HOLD (x=0,1,2,5,13,14,15,16,TIMER5 is only available in GD32F350,
                  TIMER1 is only available in GD32F350 and GD32F330): hold TIMERx counter when core is halted
      \arg        SYS_DBG_I2Cx_HOLD (x=0,1): hold I2Cx smbus when core is halted
      \arg        SYS_DBG_RTC_HOLD: hold RTC calendar and wakeup counter when core is halted
    \param[out] none
    \retval     none
*/
void hals_sys_dbg_periph_disable(hal_sys_dbg_periph_enum dbg_periph)
{
    SYS_DBG_REG_VAL(dbg_periph) &= ~BIT(SYS_DBG_BIT_POS(dbg_periph));
}

/*!
    \brief      initlize systick when use it as the source
    \param[in]  count_freq: the frequence of basetick interrupt
    \param[in]  prio: the priority of basetick interrupt
    \param[out] none
    \retval     none
*/
static void _sys_systick_init(uint32_t count_freq, uint32_t prio)
{

#if (1 == HAL_PARAMETER_CHECK)
    uint32_t lowest_prio;
    lowest_prio = (0x01U << __NVIC_PRIO_BITS) - 1U;
    if(prio > lowest_prio) {
        HAL_DEBUGE("parameter [prio] value is greater than configurable priority");
        prio = lowest_prio;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(SysTick_Config(g_systemcoreclock / count_freq)) {
        /* capture error */
        while(1) {
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, prio);
}

/*!
    \brief      initlize timer when use it as the source
    \param[in]  source: select the source timer
      \arg        HAL_BASETICK_SOURCE_TIMERx: x=0,1,2,5,13,14,15,16
    \param[in]  count_freq: the frequence of basetick interrupt
    \param[in]  prio: the priority of basetick interrupt
    \param[out] none
    \retval     none
*/
static void _sys_basetick_timer_init(hal_sys_timebase_source_enum source, uint32_t count_freq, uint8_t prio)
{
    hal_timer_init_struct timer_initpara;
    hal_timer_dev_struct  timer_dev_temp;
    uint16_t timer_prescaler;
    uint8_t  timer_cfg_index;

#if (1 == HAL_PARAMETER_CHECK)
    uint8_t lowest_prio;
    /* check the parameter */
    lowest_prio = (0x01U << __NVIC_PRIO_BITS) - 1U;
    if(prio > lowest_prio) {
        HAL_DEBUGE("parameter [prio] value is greater than configurable priority");
        prio = lowest_prio;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* get the APBx clock */
    if((source == SYS_TIMEBASE_SOURCE_TIMER1) || (source == SYS_TIMEBASE_SOURCE_TIMER2) || \
            (source == SYS_TIMEBASE_SOURCE_TIMER5) || (source == SYS_TIMEBASE_SOURCE_TIMER13)) {
        timer_prescaler = (uint16_t)(hal_rcu_periph_clkfreq_get(RCU_PERIPH_CLKTYPE_APB1TIMER) / 1000000U) - 1U;
    } else if((source == SYS_TIMEBASE_SOURCE_TIMER0) || (source == SYS_TIMEBASE_SOURCE_TIMER14) || \
              (source == SYS_TIMEBASE_SOURCE_TIMER15) || (source == SYS_TIMEBASE_SOURCE_TIMER16)) {
        timer_prescaler = (uint16_t)(hal_rcu_periph_clkfreq_get(RCU_PERIPH_CLKTYPE_APB2TIMER) / 1000000U) - 1U;
    } else {
        HAL_DEBUGE("Basetick timer source is not available");
        return;
    }

    timer_cfg_index = source - 1U;
    /* enable the interrupt */
    hal_nvic_irq_enable(_BASETICK_SOURCE_IRQN[timer_cfg_index], 1, 0);

    /* enable the clock of timer */
    hal_rcu_periph_clk_enable(_BASETICK_SOURCE_CLK[timer_cfg_index]);

    timer_dev_temp.periph = _BASETICK_SOURCE_PERIPH[timer_cfg_index];
    hal_timer_deinit(&timer_dev_temp);
    hal_timer_struct_init(HAL_TIMER_INIT_STRUCT, &timer_initpara);
    /* initialize the using timer */
    timer_initpara.prescaler          = timer_prescaler;
    timer_initpara.alignedmode        = TIMER_COUNTER_EDGE;
    timer_initpara.counter_direction  = TIMER_COUNTER_UP;
    timer_initpara.period             = 1000000U / count_freq;
    timer_initpara.clock_division     = TIMER_CKDIV_DIV1;
    timer_initpara.repetition_counter = 0U;
    hal_timer_init(&timer_dev_temp, _BASETICK_SOURCE_PERIPH[timer_cfg_index], &timer_initpara);

    hals_timer_interrupt_enable(_BASETICK_SOURCE_PERIPH[timer_cfg_index], TIMER_INT_UP);
    hals_timer_enable(_BASETICK_SOURCE_PERIPH[timer_cfg_index]);
}
