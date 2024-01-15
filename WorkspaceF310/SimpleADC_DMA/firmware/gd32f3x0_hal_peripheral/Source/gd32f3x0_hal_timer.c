/*!
    \file    gd32f3x0_hal_timer.c
    \brief   TIMER driver

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

/* DMA callback function */
/* DMA transmission complete(TC) callback for TIMER update DMA request */
static void _update_dma_full_transfer_complete(void *dma);
/* DMA transmission complete(TC) callback for TIMER channel input capture DMA request */
static void _channelx_capture_dma_full_transfer_complete(void *dma);
/* DMA transmission complete(TC) callback for TIMER compare and pwm DMA request */
static void _channelx_compare_dma_full_transfer_complete(void *dma);
/* DMA transmission complete(TC) callback for TIMER commutation DMA request */
static void _commutation_dma_full_transfer_complete(void *dma);
/* DMA transmission complete(TC) callback for TIMER trigger DMA request */
static void _trigger_dma_full_transfer_complete(void *dma);
/* DMA error callback for TIMER all DMA request */
static void _timer_dma_error(void *dma);

/* enable TIMER */
static void _timer_enable(hal_timer_dev_struct *timer_dev);
/* disable TIMER */
static int32_t _timer_disable(hal_timer_dev_struct *timer_dev);
/* enable or disable TIMER primary output */
static void _timer_primary_output_config(hal_timer_dev_struct *timer_dev, ControlStatus state);


/*!
    \brief      initialize TIMER timebase
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which TIMER timebase is initialized
      \arg        TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[in]  timer: TIMER basic init configuration struct
                  prescaler: prescaler value of the counter clock,0~65535
                  alignedmode:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_COUNTER_EDGE: edge-aligned mode
      \arg          TIMER_COUNTER_CENTER_DOWN: center-aligned and counting down assert mode
      \arg          TIMER_COUNTER_CENTER_UP: center-aligned and counting up assert mode
      \arg          TIMER_COUNTER_CENTER_BOTH: center-aligned and counting up/down assert mode
                  counter_direction:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_COUNTER_UP: counting up direction
      \arg          TIMER_COUNTER_DOWN: counting down direction
                  period: counter auto reload value,0~65535 (for TIMER1 0x00000000~0xFFFFFFFF)
                  clock_division:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_CKDIV_DIV1: clock division value is 1, fDTS = fCK_TIMER
      \arg          TIMER_CKDIV_DIV2: clock division value is 2, fDTS = fCK_TIMER/2
      \arg          TIMER_CKDIV_DIV4: clock division value is 4, fDTS = fCK_TIMER/4
                  repetition_counter: counter repetition value,0~255
                  autoreload_shadow:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_CARL_SHADOW_ENABLE: channel output shadow state enable
      \arg          TIMER_CARL_SHADOW_DISABLE: channel output shadow state disable
                  trgo_selection: the argument could be selected from enumeration <hal_timer_trgo_selection_enum>
                  master_slave_mode:
                  only one parameter can be selected which is shown as below:
      \arg          ENABLE: master slave mode enable
      \arg          DISABLE: master slave mode disable
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_init(hal_timer_dev_struct *timer_dev, uint32_t periph, hal_timer_init_struct *timer)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == timer_dev) || (NULL == timer)) {
        HAL_DEBUGE("pointer [timer_dev] or [timer] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#if defined(GD32F350)
    /* check the parameters */
    if((TIMER0 != periph) && (TIMER1 != periph) && (TIMER2 != periph) && (TIMER5 != periph)
            && (TIMER13 != periph) && (TIMER14 != periph) && (TIMER15 != periph) && (TIMER16 != periph)) {
        HAL_DEBUGE("parameter [periph] value is invalid");
        return HAL_ERR_VAL;
    }
#elif defined(GD32F330)
    /* check the parameters */
    if((TIMER0 != periph) && (TIMER1 != periph) && (TIMER2 != periph) && (TIMER13 != periph)
            && (TIMER14 != periph) && (TIMER15 != periph) && (TIMER16 != periph)) {
        HAL_DEBUGE("parameter [periph] value is invalid");
        return HAL_ERR_VAL;
    }
#else
    /* check the parameters */
    if((TIMER0 != periph) && (TIMER2 != periph) && (TIMER13 != periph)
            && (TIMER14 != periph) && (TIMER15 != periph) && (TIMER16 != periph)) {
        HAL_DEBUGE("parameter [periph] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* GD32F350 and GD32F330 */
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_dev->state = HAL_TIMER_STATE_BUSY;

    /* TIMER periph settings */
    timer_dev->periph = periph;

    /* initialize TIMER basic settings */
    /* configure the counter prescaler value */
    TIMER_PSC(periph) = (uint16_t)timer->prescaler;

#if (defined(GD32F350) || defined(GD32F330))
    /* configure the counter direction and aligned mode */
    if((TIMER0 == periph) || (TIMER1 == periph) || (TIMER2 == periph)) {
        TIMER_CTL0(periph) &= ~(uint32_t)(TIMER_CTL0_DIR | TIMER_CTL0_CAM);
        TIMER_CTL0(periph) |= (uint32_t)timer->alignedmode;
        TIMER_CTL0(periph) |= (uint32_t)timer->counter_direction;
    }
    /* configure the autoreload value */
    TIMER_CAR(periph) = (uint32_t)timer->period;

    if((TIMER0 == periph) || (TIMER1 == periph) || (TIMER2 == periph) || (TIMER13 == periph)
            || (TIMER14 == periph) || (TIMER15 == periph) || (TIMER16 == periph)) {
        /* reset the CKDIV bit */
        TIMER_CTL0(periph) &= ~(uint32_t)TIMER_CTL0_CKDIV;
        TIMER_CTL0(periph) |= (uint32_t)timer->clock_division;
    }
#else
    /* configure the counter direction and aligned mode */
    if((TIMER0 == periph)  || (TIMER2 == periph)) {
        TIMER_CTL0(periph) &= ~(uint32_t)(TIMER_CTL0_DIR | TIMER_CTL0_CAM);
        TIMER_CTL0(periph) |= (uint32_t)timer->alignedmode;
        TIMER_CTL0(periph) |= (uint32_t)timer->counter_direction;
    }
    /* configure the autoreload value */
    TIMER_CAR(periph) = (uint32_t)timer->period;

    if((TIMER0 == periph) || (TIMER2 == periph) || (TIMER13 == periph)
            || (TIMER14 == periph) || (TIMER15 == periph) || (TIMER16 == periph)) {
        /* reset the CKDIV bit */
        TIMER_CTL0(periph) &= ~(uint32_t)TIMER_CTL0_CKDIV;
        TIMER_CTL0(periph) |= (uint32_t)timer->clock_division;
    }
#endif

    if((TIMER0 == periph) || (TIMER14 == periph) || (TIMER15 == periph) || (TIMER16 == periph)) {
        /* configure the repetition counter value */
        TIMER_CREP(periph) = (uint32_t)timer->repetition_counter;
    }

    /* generate an update event */
    TIMER_SWEVG(periph) |= (uint32_t)TIMER_SWEVG_UPG;

    /* setting the shadow register for TIMERx_CAR register */
    TIMER_CTL0(periph) &= ~TIMER_CTL0_ARSE;
    TIMER_CTL0(periph) |= timer->autoreload_shadow;

    if((TIMER13 != periph) && (TIMER15 != periph) && (TIMER16 != periph)) {
        /* configure TIMER master slave mode */
        hals_timer_master_slave_mode_config(periph, timer->master_slave_mode);
        /* select TIMER master mode output trigger source */
        hals_timer_master_output_trigger_source_select(periph, timer->trgo_selection);
    }

    /* change TIMER state */
    timer_dev->state = HAL_TIMER_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER input capture mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  timer_inputcapture: TIMER input capture configuration structure
                  ic_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_POLARITY_RISING : input capture rising edge
      \arg          TIMER_IC_POLARITY_FALLING : input capture falling edge
      \arg          TIMER_IC_POLARITY_BOTH_EDGE : input capture both edge
                  ic_selection:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_SELECTION_DIRECTTI : channel y is configured as input and icy is mapped on CIy
      \arg          TIMER_IC_SELECTION_INDIRECTTI : channel y is configured as input and icy is mapped on opposite input
      \arg          TIMER_IC_SELECTION_ITS : channel y is configured as input and icy is mapped on ITS
                  ic_prescaler:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_PRESCALER_OFF : no prescaler
      \arg          TIMER_IC_PRESCALER_DIV2 : divided by 2
      \arg          TIMER_IC_PRESCALER_DIV4 : divided by 4
      \arg          TIMER_IC_PRESCALER_DIV8 : divided by 8
                  ic_filter: 0~15
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_input_capture_config(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_input_capture_struct *timer_inputcapture)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == timer_dev) || (NULL == timer_inputcapture)) {
        HAL_DEBUGE("pointer [timer_dev] or [timer_inputcapture] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#if defined(GD32F350)
    /* check the parameters */
    if((((TIMER13 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)) && (channel != TIMER_CH_0)) ||
            ((TIMER14 == timer_dev->periph) && (channel != TIMER_CH_0) && (channel != TIMER_CH_1)) ||
            (TIMER5 == timer_dev->periph)) {
        HAL_DEBUGE("the [channel] value of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#else
    /* check the parameters */
    if((((TIMER13 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)) && (channel != TIMER_CH_0)) ||
            ((TIMER14 == timer_dev->periph) && (channel != TIMER_CH_0) && (channel != TIMER_CH_1))) {
        HAL_DEBUGE("the [channel] value of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* GD32F350 */
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_dev->state = HAL_TIMER_STATE_BUSY;

    /* configure TIMER input capture parameter */
    hals_timer_channel_input_capture_config(timer_dev->periph, channel, timer_inputcapture);

    /* change TIMER state */
    timer_dev->state = HAL_TIMER_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER output compare mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel 0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel 1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel 2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel 3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  timer_outputcompare: TIMER output compare configuration structure
                  compare_mode: the argument could be selected from enumeration <hal_timer_output_compare_enum>
                  oc_pulse_value:0~65535,(for TIMER1 0x00000000~0xFFFFFFFF)
                  oc_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_POLARITY_HIGH : channel output polarity is high
      \arg          TIMER_OC_POLARITY_LOW : channel output polarity is low
                  oc_idlestate:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_IDLE_STATE_LOW : idle state of channel output is high
      \arg          TIMER_OC_IDLE_STATE_HIGH : idle state of channel output is low
                  ocn_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OCN_POLARITY_HIGH : channel complementary output polarity is high
      \arg          TIMER_OCN_POLARITY_LOW : channel complementary output polarity is low
                  ocn_idlestate:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OCN_IDLE_STATE_LOW : idle state of channel complementary output is high
      \arg          TIMER_OCN_IDLE_STATE_HIGH :  idle state of channel complementary output is low
                  oc_shadow:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_SHADOW_ENABLE : channel output compare shadow enable
      \arg          TIMER_OC_SHADOW_DISABLE : channel output compare shadow disable
                  oc_fastmode:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_FAST_ENABLE : channel output fast function enable
      \arg          TIMER_OC_FAST_DISABLE : channel output fast function disable
                  oc_clearmode:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_CLEAR_ENABLE : channel output clear function enable
      \arg          TIMER_OC_CLEAR_DISABLE : channel output clear function disable
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_config(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_output_compare_struct *timer_outputcompare)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == timer_dev) || (NULL == timer_outputcompare)) {
        HAL_DEBUGE("pointer [timer_dev] or [timer_outputcompare] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#if defined(GD32F350)
    /* check the parameters */
    if((((TIMER13 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)) && (channel != TIMER_CH_0)) ||
            ((TIMER14 == timer_dev->periph) && (channel != TIMER_CH_0) && (channel != TIMER_CH_1)) ||
            (TIMER5 == timer_dev->periph)) {
        HAL_DEBUGE("the [channel] value of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#else
    /* check the parameters */
    if((((TIMER13 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)) && (channel != TIMER_CH_0)) ||
            ((TIMER14 == timer_dev->periph) && (channel != TIMER_CH_0) && (channel != TIMER_CH_1))) {
        HAL_DEBUGE("the [channel] value of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* GD32F350 */
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_dev->state = HAL_TIMER_STATE_BUSY;

    /* configure TIMER channel output compare mode */
    hals_timer_channel_output_config(timer_dev->periph, channel, timer_outputcompare);

    /* change TIMER state */
    timer_dev->state = HAL_TIMER_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER break function
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_break: TIMER break and complementary channel protection configuration structure
                  run_offstate:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_ROS_STATE_ENABLE : when POEN bit is set, the channel output signals (CHx_O/CHx_ON) are enabled, with relationship to CHxEN/CHxNEN bits
      \arg          TIMER_ROS_STATE_DISABLE : when POEN bit is set, the channel output signals (CHx_O/CHx_ON) are disabled
                  idel_offstate:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IOS_STATE_ENABLE : when POEN bit is reset, he channel output signals (CHx_O/CHx_ON) are enabled, with relationship to CHxEN/CHxNEN bits
      \arg          TIMER_IOS_STATE_DISABLE : when POEN bit is reset, the channel output signals (CHx_O/CHx_ON) are disabled
                  dead_time: 0~255
                  break_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_BREAK_POLARITY_LOW : break input polarity is low
      \arg          TIMER_BREAK_POLARITY_HIGH : break input polarity is high
                  output_autostate:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OUTAUTO_ENABLE : output automatic enable
      \arg          TIMER_OUTAUTO_DISABLE : output automatic disable
                  protect_mode:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_CCHP_PROT_OFF : protect disable
      \arg          TIMER_CCHP_PROT_0 : PROT mode 0
      \arg          TIMER_CCHP_PROT_1 : PROT mode 1
      \arg          TIMER_CCHP_PROT_2 : PROT mode 2
                  break_state:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_BREAK_ENABLE : break input enable
      \arg          TIMER_BREAK_DISABLE : break input disable
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_break_config(hal_timer_dev_struct *timer_dev, hal_timer_break_struct *timer_break)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == timer_dev) || (NULL == timer_break)) {
        HAL_DEBUGE("pointer [timer_dev] or [timer_break] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if((TIMER0 != timer_dev->periph) && (TIMER14 != timer_dev->periph) && (TIMER15 != timer_dev->periph) && (TIMER16 != timer_dev->periph)) {
        HAL_DEBUGE("the break function of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_dev->state = HAL_TIMER_STATE_BUSY;

    /* configure TIMER break function */
    TIMER_CCHP(timer_dev->periph) = (uint32_t)(((uint32_t)(timer_break->run_offstate)) |
                                    ((uint32_t)(timer_break->idel_offstate)) |
                                    ((uint32_t)(timer_break->dead_time)) |
                                    ((uint32_t)(timer_break->break_polarity)) |
                                    ((uint32_t)(timer_break->output_autostate)) |
                                    ((uint32_t)(timer_break->protect_mode)) |
                                    ((uint32_t)(timer_break->break_state)));
    /* change TIMER state */
    timer_dev->state = HAL_TIMER_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER OCPRE clear source
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_clearsource: TIMER output compare clear source configuration structure
                  clear_source:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OCPRE_CLEAR_SOURCE_CLR: OCPRE_CLR_INT is connected to the OCPRE_CLR input
      \arg          TIMER_OCPRE_CLEAR_SOURCE_ETIF: OCPRE_CLR_INT is connected to ETIF
                  exttrigger_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_EXT_TRI_POLARITY_RISING : active high or rising edge active
      \arg          TIMER_EXT_TRI_POLARITY_FALLING : active low or falling edge active
                  exttrigger_prescaler:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_EXT_TRI_PRESCALER_OFF : no divided
      \arg          TIMER_EXT_TRI_PRESCALER_DIV2 : divided by 2
      \arg          TIMER_EXT_TRI_PRESCALER_DIV4 : divided by 4
      \arg          TIMER_EXT_TRI_PRESCALER_DIV8 : divided by 8
                  exttrigger_filter: external trigger filter control,0~15
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_ocpre_clear_source_config(hal_timer_dev_struct *timer_dev, hal_timer_clear_source_struct *timer_clearsource)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == timer_dev) || (NULL == timer_clearsource)) {
        HAL_DEBUGE("pointer [timer_dev] or [timer_clearsource] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#if (defined(GD32F350) || defined(GD32F330))
    /* check the parameters */
    if((timer_clearsource-> clear_source == TIMER_OCPRE_CLEAR_SOURCE_ETIF) &&
            (TIMER0 != timer_dev->periph) && (TIMER1 != timer_dev->periph) && (TIMER2 != timer_dev->periph)) {
        HAL_DEBUGE("the TIMER_OCPRE_CLEAR_SOURCE_ETIF of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#else
    /* check the parameters */
    if((timer_clearsource-> clear_source == TIMER_OCPRE_CLEAR_SOURCE_ETIF) &&
            (TIMER0 != timer_dev->periph) && (TIMER2 != timer_dev->periph)) {
        HAL_DEBUGE("the TIMER_OCPRE_CLEAR_SOURCE_ETIF of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* GD32F350 and GD32F330 */
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_dev->state = HAL_TIMER_STATE_BUSY;

    /* configure TIMER channel output clear function */
    if(TIMER_OCPRE_CLEAR_SOURCE_ETIF == timer_clearsource->clear_source) {
        /* set OCRC bit */
        TIMER_SMCFG(timer_dev->periph) |= (uint32_t)TIMER_SMCFG_OCRC;
        /* configure TIMER external trigger input */
        hals_timer_external_trigger_config(timer_dev->periph, timer_clearsource->exttrigger_prescaler, timer_clearsource->exttrigger_polarity, timer_clearsource->exttrigger_filter);
    } else if(TIMER_OCPRE_CLEAR_SOURCE_CLR == timer_clearsource->clear_source) {
        /* reset OCRC bit */
        TIMER_SMCFG(timer_dev->periph) &= ~(uint32_t)TIMER_SMCFG_OCRC;
    } else {
        /* illegal parameters */
        HAL_DEBUGW("parameter [timer_clearsource->clear_source] value is undefine");
        return HAL_ERR_VAL;
    }

    /* change TIMER state */
    timer_dev->state = HAL_TIMER_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER ci0 trigger input
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  ci0_select:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CI0_CH0IN: the TIMERx_CH0 pin input is selected as channel 0 trigger input
      \arg        TIMER_CI0_XOR_CH012: the result of combinational XOR of TIMERx_CH0,CH1 and CH2 pins is selected as channel 0 trigger input
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_ci0_input_select(hal_timer_dev_struct *timer_dev, uint32_t ci0_select)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#if (defined(GD32F350) || defined(GD32F330))
    /* check the parameters */
    if((TIMER0 != timer_dev->periph) && (TIMER1 != timer_dev->periph) && (TIMER2 != timer_dev->periph)) {
        HAL_DEBUGE("the TIMER_CI0_XOR_CH012 of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#else
    /* check the parameters */
    if((TIMER0 != timer_dev->periph) && (TIMER2 != timer_dev->periph)) {
        HAL_DEBUGE("the TIMER_CI0_XOR_CH012 of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* GD32F350 and GD32F330 */
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_dev->state = HAL_TIMER_STATE_BUSY;

    if(TIMER_CI0_XOR_CH012 == ci0_select) {
        /* set TI0S bit */
        TIMER_CTL1(timer_dev->periph) |= (uint32_t)TIMER_CTL1_TI0S;
    } else if(TIMER_CI0_CH0IN == ci0_select) {
        /* reset TI0S bit */
        TIMER_CTL1(timer_dev->periph) &= ~(uint32_t)TIMER_CTL1_TI0S;
    } else {
        /* illegal parameters */
        HAL_DEBUGW("parameter [ci0_select] value is undefine");
        return HAL_ERR_VAL;
    }

    /* change TIMER state */
    timer_dev->state = HAL_TIMER_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER single pulse mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  single_pulse:
                only one parameter can be selected which is shown as below:
      \arg        ENABLE: single pulse mode enable
      \arg        DISABLE: single pulse mode disable
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_single_pulse_mode_config(hal_timer_dev_struct *timer_dev, uint32_t single_pulse)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if(TIMER13 == timer_dev->periph) {
        HAL_DEBUGE("the single pulse mode of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_dev->state = HAL_TIMER_STATE_BUSY;

    /* configure TIMER single pulse mode */
    if(ENABLE == single_pulse) {
        /* set SPM bit */
        TIMER_CTL0(timer_dev->periph) |= (uint32_t)TIMER_CTL0_SPM;
    } else if(DISABLE == single_pulse) {
        /* reset SPM bit */
        TIMER_CTL0(timer_dev->periph) &= ~((uint32_t)TIMER_CTL0_SPM);
    } else {
        /* illegal parameters */
        HAL_DEBUGW("parameter [single_pulse] value is undefine");
        return HAL_ERR_VAL;
    }

    /* change TIMER state */
    timer_dev->state = HAL_TIMER_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER clock source
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_clocksource: TIMER clock source configuration structure
                  clock_source: the argument could be selected from enumeration <hal_timer_clock_sourc_enum>
                  clock_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_CLOCK_TRIGGER_ETI_POLARITY_RISING: clock input source is ETI, active high or rising edge active
      \arg          TIMER_CLOCK_TRIGGER_ETI_POLARITY_FALLING: clock input source is ETI, active low or falling edge active
      \arg          TIMER_CLOCK_TRIGGER_POLARITY_RISING: clock input source is CIx(x=0,1), rising edge active
      \arg          TIMER_CLOCK_TRIGGER_POLARITY_FALLING: clock input source is CIx(x=0,1), falling edge active
      \arg          TIMER_CLOCK_TRIGGER_POLARITY_BOTH_EDGE: clock input source is CI0F_ED, both rising and falling edge active
                  clock_prescaler:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_EXT_TRI_PRESCALER_OFF: external trigger no divided
      \arg          TIMER_EXT_TRI_PRESCALER_DIV2: external trigger divided by 2
      \arg          TIMER_EXT_TRI_PRESCALER_DIV4: external trigger divided by 4
      \arg          TIMER_EXT_TRI_PRESCALER_DIV8: external trigger divided by 8
                  clock_filter: 0~15
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_clock_source_config(hal_timer_dev_struct *timer_dev, hal_timer_clock_source_struct *timer_clocksource)
{
    uint32_t smc_val, trgs_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == timer_dev) || (NULL == timer_clocksource)) {
        HAL_DEBUGE("pointer [timer_dev] or [timer_clocksource] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#if defined(GD32F350)
    /* check the parameters */
    if(((TIMER_CLOCK_SOURCE_CK_TIMER != timer_clocksource->clock_source) && ((TIMER5 == timer_dev->periph) || (TIMER13 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph))) ||
            (((TIMER_CLOCK_SOURCE_ETIMODE0 == timer_clocksource->clock_source) || (TIMER_CLOCK_SOURCE_ETIMODE1 == timer_clocksource->clock_source)) && (TIMER14 == timer_dev->periph))) {
        HAL_DEBUGE("the [timer_clocksource->clock_source] value of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#else
    /* check the parameters */
    if(((TIMER_CLOCK_SOURCE_CK_TIMER != timer_clocksource->clock_source) && ((TIMER13 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph))) ||
            (((TIMER_CLOCK_SOURCE_ETIMODE0 == timer_clocksource->clock_source) || (TIMER_CLOCK_SOURCE_ETIMODE1 == timer_clocksource->clock_source)) && (TIMER14 == timer_dev->periph))) {
        HAL_DEBUGE("the [timer_clocksource->clock_source] value of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* GD32F350 */
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_dev->state = HAL_TIMER_STATE_BUSY;

    /* read SMC, SMC1 and TRGS bits */
    smc_val = (uint32_t)(TIMER_SMCFG(timer_dev->periph) & TIMER_SMCFG_SMC);
    trgs_val = (uint32_t)(TIMER_SMCFG(timer_dev->periph) & TIMER_SMCFG_TRGS);

    /* reset the SMC1 bit */
    TIMER_SMCFG(timer_dev->periph) &= ~(uint32_t)TIMER_SMCFG_SMC1;

    switch(timer_clocksource->clock_source) {
    case TIMER_CLOCK_SOURCE_CK_TIMER:
        if((smc_val != TIMER_SLAVE_MODE_RESTART) && (smc_val != TIMER_SLAVE_MODE_PAUSE) && (smc_val != TIMER_SLAVE_MODE_EVENT)) {
            /* reset the SMC bit */
            TIMER_SMCFG(timer_dev->periph) &= ~(uint32_t)TIMER_SMCFG_SMC;
        }
        break;
    case TIMER_CLOCK_SOURCE_ITI0:
        /* configure TIMER the internal trigger as external clock input */
        hals_timer_input_trigger_source_select(timer_dev->periph, TIMER_SMCFG_TRGSEL_ITI0);
        /* reset the SMC bit */
        TIMER_SMCFG(timer_dev->periph) &= ~(uint32_t)TIMER_SMCFG_SMC;
        /* set the SMC bit */
        TIMER_SMCFG(timer_dev->periph) |= (uint32_t)TIMER_SLAVE_MODE_EXTERNAL0;
        break;
    case TIMER_CLOCK_SOURCE_ITI1:
        /* configure TIMER the internal trigger as external clock input */
        hals_timer_input_trigger_source_select(timer_dev->periph, TIMER_SMCFG_TRGSEL_ITI1);
        /* reset the SMC bit */
        TIMER_SMCFG(timer_dev->periph) &= ~(uint32_t)TIMER_SMCFG_SMC;
        /* set the SMC bit */
        TIMER_SMCFG(timer_dev->periph) |= (uint32_t)TIMER_SLAVE_MODE_EXTERNAL0;
        break;
    case TIMER_CLOCK_SOURCE_ITI2:
        /* configure TIMER the internal trigger as external clock input */
        hals_timer_input_trigger_source_select(timer_dev->periph, TIMER_SMCFG_TRGSEL_ITI2);
        /* reset the SMC bit */
        TIMER_SMCFG(timer_dev->periph) &= ~(uint32_t)TIMER_SMCFG_SMC;
        /* set the SMC bit */
        TIMER_SMCFG(timer_dev->periph) |= (uint32_t)TIMER_SLAVE_MODE_EXTERNAL0;
        break;
    case TIMER_CLOCK_SOURCE_ITI3:
        /* configure TIMER the internal trigger as external clock input */
        hals_timer_input_trigger_source_select(timer_dev->periph, TIMER_SMCFG_TRGSEL_ITI3);
        /* reset the SMC bit */
        TIMER_SMCFG(timer_dev->periph) &= ~(uint32_t)TIMER_SMCFG_SMC;
        /* set the SMC bit */
        TIMER_SMCFG(timer_dev->periph) |= (uint32_t)TIMER_SLAVE_MODE_EXTERNAL0;
        break;
    case TIMER_CLOCK_SOURCE_CI0FE0:
        /* configure TIMER the external trigger as external clock input */
        /* reset the CH0EN bit */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0EN | TIMER_CHCTL2_CH0NEN));
        /* reset the CH0P and CH0NP bits */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
        /* set the CH0P and CH0NP bits */
        TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)timer_clocksource->clock_polarity;
        /* reset the CH0CAPFLT bit */
        TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
        /* reset the CH0CAPFLT bit */
        TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)(timer_clocksource->clock_filter << 4U);
        /* select TIMER input trigger source */
        hals_timer_input_trigger_source_select(timer_dev->periph, TIMER_SMCFG_TRGSEL_CI0FE0);
        /* reset the SMC bit */
        TIMER_SMCFG(timer_dev->periph) &= ~(uint32_t)TIMER_SMCFG_SMC;
        /* set the SMC bit */
        TIMER_SMCFG(timer_dev->periph) |= (uint32_t)TIMER_SLAVE_MODE_EXTERNAL0;
        break;
    case TIMER_CLOCK_SOURCE_CI1FE1:
        /* configure TIMER the external trigger as external clock input */
        /* reset the CH1EN bit */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1EN | TIMER_CHCTL2_CH1NEN));
        /* reset the CH1NP bit */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
        /* set the CH1NP bit */
        TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)((uint32_t)timer_clocksource->clock_polarity << 4U);
        /* reset the CH1CAPFLT bit */
        TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
        /* set the CH1CAPFLT bit */
        TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)(timer_clocksource->clock_filter << 12U);
        /* select TIMER input trigger source */
        hals_timer_input_trigger_source_select(timer_dev->periph, TIMER_SMCFG_TRGSEL_CI1FE1);
        /* reset the SMC bit */
        TIMER_SMCFG(timer_dev->periph) &= (~(uint32_t)TIMER_SMCFG_SMC);
        /* set the SMC bit */
        TIMER_SMCFG(timer_dev->periph) |= (uint32_t)TIMER_SLAVE_MODE_EXTERNAL0;
        break;
    case TIMER_CLOCK_SOURCE_CI0FED:
        /* configure TIMER the external trigger as external clock input */
        /* reset the CH0EN bit */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0EN | TIMER_CHCTL2_CH0NEN));
        /* reset the CH0P and CH0NP bits */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
        /* set the CH0P and CH0NP bits */
        TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)timer_clocksource->clock_polarity;
        /* reset the CH0CAPFLT bit */
        TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
        /* reset the CH0CAPFLT bit */
        TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)(timer_clocksource->clock_filter << 4U);
        /* select TIMER input trigger source */
        hals_timer_input_trigger_source_select(timer_dev->periph, TIMER_SMCFG_TRGSEL_CI0F_ED);
        /* reset the SMC bit */
        TIMER_SMCFG(timer_dev->periph) &= (~(uint32_t)TIMER_SMCFG_SMC);
        /* set the SMC bit */
        TIMER_SMCFG(timer_dev->periph) |= (uint32_t)TIMER_SLAVE_MODE_EXTERNAL0;
        break;
    case TIMER_CLOCK_SOURCE_ETIMODE0:
        /* configure TIMER external trigger input */
        hals_timer_external_trigger_config(timer_dev->periph, timer_clocksource ->clock_prescaler,
                                           timer_clocksource->clock_polarity, timer_clocksource->clock_filter);
        /* select TIMER input trigger source */
        hals_timer_input_trigger_source_select(timer_dev->periph, TIMER_SMCFG_TRGSEL_ETIFP);
        /* reset the SMC bit */
        TIMER_SMCFG(timer_dev->periph) &= (~(uint32_t)TIMER_SMCFG_SMC);
        /* set the SMC bit */
        TIMER_SMCFG(timer_dev->periph) |= (uint32_t)TIMER_SLAVE_MODE_EXTERNAL0;
        break;
    case TIMER_CLOCK_SOURCE_ETIMODE1:
        if((smc_val != TIMER_SLAVE_MODE_RESTART) && (smc_val != TIMER_SLAVE_MODE_PAUSE) && (smc_val != TIMER_SLAVE_MODE_EVENT)) {
            /* reset the SMC bit */
            TIMER_SMCFG(timer_dev->periph) &= (~(uint32_t)TIMER_SMCFG_SMC);
        }
        if(trgs_val == TIMER_SMCFG_TRGSEL_ETIFP) {
            /* reset TRGS bits */
            TIMER_SMCFG(timer_dev->periph) &= (~(uint32_t)TIMER_SMCFG_TRGS);
        }
        /* configure TIMER external trigger input */
        hals_timer_external_trigger_config(timer_dev->periph, timer_clocksource ->clock_prescaler,
                                           timer_clocksource->clock_polarity, timer_clocksource->clock_filter);
        /* configure TIMER the external clock mode1 */
        TIMER_SMCFG(timer_dev->periph) |= (uint32_t)TIMER_SMCFG_SMC1;
        break;
    default:
        HAL_DEBUGW("parameter [timer_clocksource->clock_source] value is undefine");
        return HAL_ERR_VAL;
    }

    /* change TIMER state */
    timer_dev->state = HAL_TIMER_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER slave mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_slavemode: TIMER slave mode configuration structure
                  slavemode: the argument could be selected from enumeration <hal_timer_slave_mode_enum>
                  trigger_selection: the argument could be selected from enumeration <hal_timer_input_trigger_source_enum>
                  trigger_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_CLOCK_TRIGGER_ETI_POLARITY_RISING: trigger input source is ETI, active high or rising edge active
      \arg          TIMER_CLOCK_TRIGGER_ETI_POLARITY_FALLING: trigger input source is ETI, active low or falling edge active
      \arg          TIMER_CLOCK_TRIGGER_POLARITY_RISING: trigger input source is CIx(x=0,1), rising edge active
      \arg          TIMER_CLOCK_TRIGGER_POLARITY_FALLING: trigger input source is CIx(x=0,1), falling edge active
      \arg          TIMER_CLOCK_TRIGGER_POLARITY_BOTH_EDGE: trigger input source is CI0F_ED, both rising and falling edge active
                  trigger_prescaler:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_EXT_TRI_PRESCALER_OFF: external trigger no divided
      \arg          TIMER_EXT_TRI_PRESCALER_DIV2: external trigger divided by 2
      \arg          TIMER_EXT_TRI_PRESCALER_DIV4: external trigger divided by 4
      \arg          TIMER_EXT_TRI_PRESCALER_DIV8: external trigger divided by 8
                  trigger_filter: 0~15
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_slave_mode_config(hal_timer_dev_struct *timer_dev, hal_timer_slave_mode_struct *timer_slavemode)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == timer_dev) || (NULL == timer_slavemode)) {
        HAL_DEBUGE("pointer [timer_dev] or [timer_slavemode] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#if defined(GD32F350)
    /* check the parameters */
    if((TIMER5 == timer_dev->periph) || (TIMER13 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)) {
        HAL_DEBUGE("the [timer_slavemode->slavemode] of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#else
    /* check the parameters */
    if((TIMER13 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)) {
        HAL_DEBUGE("the [timer_slavemode->slavemode] of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* GD32F350 */
    /* check the parameters */
    if((TIMER_TRIGGER_SOURCE_ETIFP == timer_slavemode->trigger_selection) && (TIMER14 == timer_dev->periph)) {
        HAL_DEBUGE("the [timer_slavemode->trigger_selection] value of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
    if(TIMER_TRIGGER_SOURCE_DISABLE == timer_slavemode->trigger_selection) {
        HAL_DEBUGE("the [timer_slavemode->trigger_selection] value of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_dev->state = HAL_TIMER_STATE_BUSY;

    /* disable the TIMER interrupt */
    hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_TRG);
    /* disable the TIMER DMA trigger request */
    hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_TRGD);

    switch(timer_slavemode->trigger_selection) {
    case TIMER_TRIGGER_SOURCE_ITI0:
        /* no need to config polarity, prescaler, filter */
        break;
    case TIMER_TRIGGER_SOURCE_ITI1:
        /* no need to config polarity, prescaler, filter */
        break;
    case TIMER_TRIGGER_SOURCE_ITI2:
        /* no need to config polarity, prescaler, filter */
        break;
    case TIMER_TRIGGER_SOURCE_ITI3:
        /* no need to config polarity, prescaler, filter */
        break;
    case TIMER_TRIGGER_SOURCE_CI0FE0:
        /* reset the CH0EN bit */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0EN | TIMER_CHCTL2_CH0NEN));
        /* reset the CH0P and CH0NP bits */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
        /* config polarity */
        TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)(timer_slavemode->trigger_polarity);
        /* reset the CH0CAPFLT bit */
        TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
        /* config filter */
        TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_filter) << 4U);
        break;
    case TIMER_TRIGGER_SOURCE_CI1FE1:
        /* reset the CH1EN bit */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1EN | TIMER_CHCTL2_CH1NEN));
        /* reset the CH1P and CH1NP bits */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
        /* config polarity */
        TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_polarity) << 4U);
        /* reset the CH1CAPFLT bit */
        TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
        /* config filter */
        TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_filter) << 12U);
        break;
    case TIMER_TRIGGER_SOURCE_CI0FED:
        /* reset the CH0EN bit */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1EN | TIMER_CHCTL2_CH1NEN));
        /* reset the CH0CAPFLT bit */
        TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
        /* config filter */
        TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_filter) << 4U);
        break;
    case TIMER_TRIGGER_SOURCE_ETIFP:
        /* configure TIMER external trigger input */
        hals_timer_external_trigger_config(timer_dev->periph, timer_slavemode->trigger_prescaler,
                                           timer_slavemode->trigger_polarity, timer_slavemode->trigger_filter);
        break;
    default:
        HAL_DEBUGW("parameter [timer_slavemode->trigger_selection] value is undefine");
        return HAL_ERR_VAL;
    }

    /* select TIMER input trigger source  */
    hals_timer_input_trigger_source_select(timer_dev->periph, timer_slavemode->trigger_selection);
    /* select TIMER slave mode */
    hals_timer_slave_mode_select(timer_dev->periph, timer_slavemode->slavemode);

    /* change TIMER state */
    timer_dev->state = HAL_TIMER_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER decoder mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_decoder: TIMER decoder mode configuration structure
                  decoder_mode: the argument could be selected from enumeration <hal_timer_decoder_mode_enum>
                  ci0_polarity/ci1_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_POLARITY_RISING : input capture rising edge
      \arg          TIMER_IC_POLARITY_FALLING : input capture falling edge
                  ci0_selection/ci1_selection:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_SELECTION_DIRECTTI : channel y is configured as input and icy is mapped on CIy
      \arg          TIMER_IC_SELECTION_INDIRECTTI : channel y is configured as input and icy is mapped on opposite input
      \arg          TIMER_IC_SELECTION_ITS : channel y is configured as input and icy is mapped on ITS
                  ci0_prescaler/ci1_prescaler:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_PRESCALER_OFF : no prescaler
      \arg          TIMER_IC_PRESCALER_DIV2 : divided by 2
      \arg          TIMER_IC_PRESCALER_DIV4 : divided by 4
      \arg          TIMER_IC_PRESCALER_DIV8 : divided by 8
                  ci0_filter/ci1_filter: 0~15
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_decoder_config(hal_timer_dev_struct *timer_dev, hal_timer_decoder_struct *timer_decoder)
{
    static hal_timer_input_capture_struct ic0_capture;
    static hal_timer_input_capture_struct ic1_capture;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == timer_dev) || (NULL == timer_decoder)) {
        HAL_DEBUGE("pointer [timer_dev] or [timer_decoder] address is invalid");
        return HAL_ERR_ADDRESS;
    }

#if (defined(GD32F350) || defined(GD32F330))
    /* check the parameters */
    if((TIMER0 != timer_dev->periph) && (TIMER1 != timer_dev->periph) && (TIMER2 != timer_dev->periph)) {
        HAL_DEBUGE("the [timer_decoder->decoder_mode] of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#else
    /* check the parameters */
    if((TIMER0 != timer_dev->periph) && (TIMER2 != timer_dev->periph)) {
        HAL_DEBUGE("the [timer_decoder->decoder_mode] of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* GD32F350 */
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_dev->state = HAL_TIMER_STATE_BUSY;

    TIMER_SMCFG(timer_dev->periph) &= (~(uint32_t)TIMER_SMCFG_SMC);
    TIMER_SMCFG(timer_dev->periph) |= (uint32_t)timer_decoder->decoder_mode;

    /* channel 0 input parameter structure */
    ic0_capture.ic_polarity = timer_decoder->ci0_polarity;
    ic0_capture.ic_selection = timer_decoder->ci0_selection;
    ic0_capture.ic_prescaler = timer_decoder->ci0_prescaler;
    ic0_capture.ic_filter = timer_decoder->ci0_filter;
    /* channel 1 input parameter structure */
    ic1_capture.ic_polarity = timer_decoder->ci1_polarity;
    ic1_capture.ic_selection = timer_decoder->ci1_selection;
    ic1_capture.ic_prescaler = timer_decoder->ci1_prescaler;
    ic1_capture.ic_filter = timer_decoder->ci1_filter;
    /* configure TIMER input capture parameter */
    hals_timer_channel_input_capture_config(timer_dev->periph, TIMER_CH_0, &ic0_capture);
    /* configure TIMER input capture parameter */
    hals_timer_channel_input_capture_config(timer_dev->periph, TIMER_CH_1, &ic1_capture);

    /* change TIMER state */
    timer_dev->state = HAL_TIMER_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER hall sensor mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_hallsensor: TIMER hall sensor mode configuration structure
                  cmt_delay: commutation delay(channel 1 compare value)
                  ci0_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_POLARITY_RISING : input capture rising edge
                  ci0_selection:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_SELECTION_ITS : channel y is configured as input and icy is mapped on ITS
                  ci0_prescaler:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_PRESCALER_OFF : no prescaler
      \arg          TIMER_IC_PRESCALER_DIV2 : divided by 2
      \arg          TIMER_IC_PRESCALER_DIV4 : divided by 4
      \arg          TIMER_IC_PRESCALER_DIV8 : divided by 8
                  ci0_filter: 0~15
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_hall_sensor_config(hal_timer_dev_struct *timer_dev, hal_timer_hall_sensor_struct *timer_hallsensor)
{
    static hal_timer_input_capture_struct input_capture;
    static hal_timer_output_compare_struct output_compare;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == timer_dev) || (NULL == timer_hallsensor)) {
        HAL_DEBUGE("pointer [timer_dev] or [timer_hallsensor] address is invalid");
        return HAL_ERR_ADDRESS;
    }

#if (defined(GD32F350) || defined(GD32F330))
    /* check the parameters */
    if((TIMER0 != timer_dev->periph) && (TIMER1 != timer_dev->periph) && (TIMER2 != timer_dev->periph)) {
        HAL_DEBUGE("the hall sensor mode of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#else
    /* check the parameters */
    if((TIMER0 != timer_dev->periph) && (TIMER2 != timer_dev->periph)) {
        HAL_DEBUGE("the hall sensor mode of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* GD32F350 */
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_dev->state = HAL_TIMER_STATE_BUSY;

    /* structure parameter config */
    input_capture.ic_polarity  = timer_hallsensor->ci0_polarity;
    input_capture.ic_selection = timer_hallsensor->ci0_selection;
    input_capture.ic_prescaler = timer_hallsensor->ci0_prescaler;
    input_capture.ic_filter    = timer_hallsensor->ci0_filter;
    hals_timer_channel_input_capture_config(timer_dev->periph, TIMER_CH_0, &input_capture);

    /* configure TIMER hall sensor mode*/
    /* set TI0S bit */
    TIMER_CTL1(timer_dev->periph) |= (uint32_t)TIMER_CTL1_TI0S;
    /* select TIMER input trigger source */
    hals_timer_input_trigger_source_select(timer_dev->periph, TIMER_SMCFG_TRGSEL_CI0F_ED);
    /* select TIMER slave mode */
    hals_timer_slave_mode_select(timer_dev->periph, TIMER_SLAVE_MODE_RESTART);

    /* TIMER output compare mode structure config */
    output_compare.compare_mode = TIMER_OC_MODE_PWM0;
    output_compare.oc_pulse_value = timer_hallsensor->cmt_delay;
    output_compare.oc_polarity = TIMER_OC_POLARITY_HIGH;
    output_compare.ocn_polarity = TIMER_OCN_POLARITY_HIGH;
    output_compare.oc_idlestate = TIMER_OC_IDLE_STATE_LOW;
    output_compare.ocn_idlestate = TIMER_OCN_IDLE_STATE_LOW;
    output_compare.oc_shadow = TIMER_OC_SHADOW_ENABLE;
    output_compare.oc_fastmode = TIMER_OC_FAST_DISABLE;
    output_compare.oc_clearmode = TIMER_OC_CLEAR_DISABLE;
    /* initialize TIMER output compare mode */
    hals_timer_channel_output_config(timer_dev->periph, TIMER_CH_1, &output_compare);

    /* select TIMER master mode output trigger source */
    hals_timer_master_output_trigger_source_select(timer_dev->periph, TIMER_TRI_OUT_SRC_O1CPRE);
    /* change TIMER state */
    timer_dev->state = HAL_TIMER_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      initialize TIMER structure with the default values
    \param[in]  hal_struct_type: the argument could be selected from enumeration <hal_timer_struct_type_enum>
    \param[in]  p_struct: pointer to TIMER structure that contains the configuration information
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_struct_init(hal_timer_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer [*p_struct] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_TIMER_INIT_STRUCT:
        /* initialize TIMER basic init configuration structure with the default values */
        ((hal_timer_init_struct *)p_struct)->prescaler          = 0U;
        ((hal_timer_init_struct *)p_struct)->alignedmode        = TIMER_COUNTER_EDGE;
        ((hal_timer_init_struct *)p_struct)->counter_direction  = TIMER_COUNTER_UP;
        ((hal_timer_init_struct *)p_struct)->period             = 65535U;
        ((hal_timer_init_struct *)p_struct)->clock_division     = TIMER_CKDIV_DIV1;
        ((hal_timer_init_struct *)p_struct)->repetition_counter = 0U;
        ((hal_timer_init_struct *)p_struct)->autoreload_shadow  = TIMER_CARL_SHADOW_DISABLE;
        ((hal_timer_init_struct *)p_struct)->trgo_selection     = TIMER_TRI_OUT_SRC_RESET;
        ((hal_timer_init_struct *)p_struct)->master_slave_mode  = DISABLE;
        break;
    case HAL_TIMER_INPUT_CAPTURE_STRUCT:
        /* initialize TIMER input capture configuration structure with the default values */
        ((hal_timer_input_capture_struct *)p_struct)->ic_polarity     = TIMER_IC_POLARITY_RISING;
        ((hal_timer_input_capture_struct *)p_struct)->ic_selection    = TIMER_IC_SELECTION_DIRECTTI;
        ((hal_timer_input_capture_struct *)p_struct)->ic_prescaler    = TIMER_IC_PRESCALER_OFF;
        ((hal_timer_input_capture_struct *)p_struct)->ic_filter       = 0U;
        break;
    case HAL_TIMER_OUTPUT_COMPARE_STRUCT:
        /* initialize TIMER output compare configuration structure with the default values */
        ((hal_timer_output_compare_struct *)p_struct)->compare_mode   = TIMER_OC_MODE_TIMING;
        ((hal_timer_output_compare_struct *)p_struct)->oc_pulse_value = 0U;
        ((hal_timer_output_compare_struct *)p_struct)->oc_polarity    = TIMER_OC_POLARITY_HIGH;
        ((hal_timer_output_compare_struct *)p_struct)->oc_idlestate   = TIMER_OC_IDLE_STATE_LOW;
        ((hal_timer_output_compare_struct *)p_struct)->ocn_polarity   = TIMER_OCN_POLARITY_HIGH ;
        ((hal_timer_output_compare_struct *)p_struct)->ocn_idlestate  = TIMER_OCN_IDLE_STATE_LOW;
        ((hal_timer_output_compare_struct *)p_struct)->oc_shadow      = TIMER_OC_SHADOW_DISABLE;
        ((hal_timer_output_compare_struct *)p_struct)->oc_fastmode    = TIMER_OC_FAST_DISABLE;
        ((hal_timer_output_compare_struct *)p_struct)->oc_clearmode   = TIMER_OC_CLEAR_DISABLE;
        break;
    case HAL_TIMER_BREAK_STRUCT:
        /* initialize TIMER break and complementary channel protection configuration structure with the default values */
        ((hal_timer_break_struct *)p_struct)->run_offstate     = TIMER_ROS_STATE_DISABLE;
        ((hal_timer_break_struct *)p_struct)->idel_offstate    = TIMER_IOS_STATE_DISABLE;
        ((hal_timer_break_struct *)p_struct)->dead_time        = 0U;
        ((hal_timer_break_struct *)p_struct)->break_polarity   = TIMER_BREAK_POLARITY_HIGH;
        ((hal_timer_break_struct *)p_struct)->output_autostate = TIMER_OUTAUTO_DISABLE;
        ((hal_timer_break_struct *)p_struct)->protect_mode     = TIMER_CCHP_PROT_OFF;
        ((hal_timer_break_struct *)p_struct)->break_state      = TIMER_BREAK_DISABLE;
        break;
    case HAL_TIMER_CLEAR_SOURCE_STRUCT:
        /* initialize TIMER external trigger ETI configuration structure with the default values */
        ((hal_timer_clear_source_struct *)p_struct)->clear_source         = TIMER_OCPRE_CLEAR_SOURCE_CLR;
        ((hal_timer_clear_source_struct *)p_struct)->exttrigger_polarity  = TIMER_EXT_TRI_POLARITY_RISING;
        ((hal_timer_clear_source_struct *)p_struct)->exttrigger_prescaler = TIMER_EXT_TRI_PRESCALER_OFF;
        ((hal_timer_clear_source_struct *)p_struct)->exttrigger_filter    = 0U;
        break;
    case HAL_TIMER_CLOCK_SOURCE_STRUCT:
        /* initialize TIMER external trigger ETI configuration structure with the default values */
        ((hal_timer_clock_source_struct *)p_struct)->clock_source    = TIMER_CLOCK_SOURCE_CK_TIMER;
        ((hal_timer_clock_source_struct *)p_struct)->clock_polarity  = TIMER_CLOCK_TRIGGER_ETI_POLARITY_RISING;
        ((hal_timer_clock_source_struct *)p_struct)->clock_prescaler = TIMER_EXT_TRI_PRESCALER_OFF;
        ((hal_timer_clock_source_struct *)p_struct)->clock_filter    = 0U;
        break;
    case HAL_TIMER_SLAVE_MODE_STRUCT:
        /* initialize TIMER slave mode configuration structure with the default values */
        ((hal_timer_slave_mode_struct *)p_struct)->slavemode         = TIMER_SLAVE_DISABLE_MODE;
        ((hal_timer_slave_mode_struct *)p_struct)->trigger_selection = TIMER_TRIGGER_SOURCE_ITI0;
        ((hal_timer_slave_mode_struct *)p_struct)->trigger_polarity  = TIMER_CLOCK_TRIGGER_ETI_POLARITY_RISING;
        ((hal_timer_slave_mode_struct *)p_struct)->trigger_prescaler = TIMER_EXT_TRI_PRESCALER_OFF;
        ((hal_timer_slave_mode_struct *)p_struct)->trigger_filter    = 0U;
        break;
    case HAL_TIMER_DECODER_STRUCT:
        /* initialize TIMER decoder mode configuration structure with the default values */
        ((hal_timer_decoder_struct *)p_struct)->decoder_mode  = TIMER_QUADRATURE_DECODER_MODE0;
        ((hal_timer_decoder_struct *)p_struct)->ci0_polarity  = TIMER_IC_POLARITY_RISING;
        ((hal_timer_decoder_struct *)p_struct)->ci0_selection = TIMER_IC_SELECTION_DIRECTTI;
        ((hal_timer_decoder_struct *)p_struct)->ci0_prescaler = TIMER_IC_PRESCALER_OFF;
        ((hal_timer_decoder_struct *)p_struct)->ci0_filter    = 0U;
        ((hal_timer_decoder_struct *)p_struct)->ci1_polarity  = TIMER_IC_POLARITY_RISING;
        ((hal_timer_decoder_struct *)p_struct)->ci1_selection = TIMER_IC_SELECTION_DIRECTTI;
        ((hal_timer_decoder_struct *)p_struct)->ci1_prescaler = TIMER_IC_PRESCALER_OFF;
        ((hal_timer_decoder_struct *)p_struct)->ci1_filter    = 0U;
        break;
    case HAL_TIMER_DECODER_DMA_CONFIG_STRUCT:
        /* initialize TIMER decoder mode DMA transfer configuration structure with the default values */
        ((hal_timer_decoder_dma_config_struct *)p_struct)->mem_addr0 = NULL;
        ((hal_timer_decoder_dma_config_struct *)p_struct)->mem_addr1 = NULL;
        ((hal_timer_decoder_dma_config_struct *)p_struct)->length    = 0U;
        break;
    case HAL_TIMER_HALL_SENSOR_STRUCT:
        /* initialize TIMER HALL sensor mode configuration structure with the default values */
        ((hal_timer_hall_sensor_struct *)p_struct)->cmt_delay     = 0U;
        ((hal_timer_hall_sensor_struct *)p_struct)->ci0_polarity  = TIMER_IC_POLARITY_RISING;
        ((hal_timer_hall_sensor_struct *)p_struct)->ci0_selection = TIMER_IC_SELECTION_ITS;
        ((hal_timer_hall_sensor_struct *)p_struct)->ci0_prescaler = TIMER_IC_PRESCALER_OFF;
        ((hal_timer_hall_sensor_struct *)p_struct)->ci0_filter    = 0U;
        break;
    case HAL_TIMER_SINGLE_PULSE_STRUCT:
        /* initialize TIMER HALL sensor mode configuration structure with the default values */
        ((hal_timer_single_pulse_struct *)p_struct)->sp_compare_mode   = TIMER_OC_MODE_TIMING;
        ((hal_timer_single_pulse_struct *)p_struct)->sp_oc_pulse_value = 0U;
        ((hal_timer_single_pulse_struct *)p_struct)->sp_oc_polarity    = TIMER_OC_POLARITY_HIGH;
        ((hal_timer_single_pulse_struct *)p_struct)->sp_ocn_polarity   = TIMER_OCN_POLARITY_HIGH;
        ((hal_timer_single_pulse_struct *)p_struct)->sp_oc_idlestate   = TIMER_OC_IDLE_STATE_LOW;
        ((hal_timer_single_pulse_struct *)p_struct)->sp_ocn_idlestate  = TIMER_OCN_IDLE_STATE_LOW;
        ((hal_timer_single_pulse_struct *)p_struct)->sp_oc_fastmode    = TIMER_OC_FAST_DISABLE;
        ((hal_timer_single_pulse_struct *)p_struct)->sp_oc_clearmode   = TIMER_OC_CLEAR_DISABLE;
        ((hal_timer_single_pulse_struct *)p_struct)->sp_ic_polarity    = TIMER_IC_POLARITY_RISING;
        ((hal_timer_single_pulse_struct *)p_struct)->sp_ic_selection   = TIMER_IC_SELECTION_DIRECTTI;
        ((hal_timer_single_pulse_struct *)p_struct)->sp_ic_filter      = 0U;
        break;
    case HAL_TIMER_DMA_TRANSFER_CONFIG_STRUCT:
        /* initialize TIMER DMA transfer configuration structure with the default values */
        ((hal_timer_dma_transfer_config_struct *)p_struct)->start_addr = TIMER_DMA_START_ADDRESS_CTL0;
        ((hal_timer_dma_transfer_config_struct *)p_struct)->mem_addr   = NULL;
        ((hal_timer_dma_transfer_config_struct *)p_struct)->length     = TIMER_DMACFG_DMATC_1TRANSFER;
        break;
    case HAL_TIMER_IRQ_STRUCT:
        /* initialize TIMER interrupt user callback function pointer structure with the default values */
        ((hal_timer_irq_struct *)p_struct)->update_handle           = NULL;
        ((hal_timer_irq_struct *)p_struct)->channelx_capture_handle = NULL;
        ((hal_timer_irq_struct *)p_struct)->channelx_compare_handle = NULL;
        ((hal_timer_irq_struct *)p_struct)->commutation_handle      = NULL;
        ((hal_timer_irq_struct *)p_struct)->trigger_handle          = NULL;
        ((hal_timer_irq_struct *)p_struct)->break_handle            = NULL;
        break;
    case HAL_TIMER_DMA_HANDLE_CB_STRUCT:
        /* initialize TIMER DMA interrupt user callback function pointer structure with the default values */
        ((hal_timer_dma_handle_cb_struct *)p_struct)->update_dma_full_transcom_handle           = NULL;
        ((hal_timer_dma_handle_cb_struct *)p_struct)->channelx_capture_dma_full_transcom_handle = NULL;
        ((hal_timer_dma_handle_cb_struct *)p_struct)->channelx_compare_dma_full_transcom_handle = NULL;
        ((hal_timer_dma_handle_cb_struct *)p_struct)->commutation_dma_full_transcom_handle      = NULL;
        ((hal_timer_dma_handle_cb_struct *)p_struct)->trigger_dma_full_transcom_handle          = NULL;
        ((hal_timer_dma_handle_cb_struct *)p_struct)->error_handle                              = NULL;
        break;
    case HAL_TIMER_DEV_STRUCT:
        /* initialize TIMER device information structure with the default values */
        ((hal_timer_dev_struct *)p_struct)->periph = 0;
        ((hal_timer_dev_struct *)p_struct)->timer_irq.update_handle                             = NULL;
        ((hal_timer_dev_struct *)p_struct)->timer_irq.channelx_capture_handle                   = NULL;
        ((hal_timer_dev_struct *)p_struct)->timer_irq.channelx_compare_handle                   = NULL;
        ((hal_timer_dev_struct *)p_struct)->timer_irq.commutation_handle                        = NULL;
        ((hal_timer_dev_struct *)p_struct)->timer_irq.trigger_handle                            = NULL;
        ((hal_timer_dev_struct *)p_struct)->timer_irq.break_handle                              = NULL;
        ((hal_timer_dev_struct *)p_struct)->p_dma_timer[0]                                      = NULL;
        ((hal_timer_dev_struct *)p_struct)->p_dma_timer[1]                                      = NULL;
        ((hal_timer_dev_struct *)p_struct)->p_dma_timer[2]                                      = NULL;
        ((hal_timer_dev_struct *)p_struct)->p_dma_timer[3]                                      = NULL;
        ((hal_timer_dev_struct *)p_struct)->p_dma_timer[4]                                      = NULL;
        ((hal_timer_dev_struct *)p_struct)->p_dma_timer[5]                                      = NULL;
        ((hal_timer_dev_struct *)p_struct)->p_dma_timer[6]                                      = NULL;
        ((hal_timer_dev_struct *)p_struct)->service_channel                                     = HAL_TIMER_SERVICE_CHANNEL_NONE;
        ((hal_timer_dev_struct *)p_struct)->timer_dma.update_dma_full_transcom_handle           = NULL;
        ((hal_timer_dev_struct *)p_struct)->timer_dma.channelx_capture_dma_full_transcom_handle = NULL;
        ((hal_timer_dev_struct *)p_struct)->timer_dma.channelx_compare_dma_full_transcom_handle = NULL;
        ((hal_timer_dev_struct *)p_struct)->timer_dma.commutation_dma_full_transcom_handle      = NULL;
        ((hal_timer_dev_struct *)p_struct)->timer_dma.trigger_dma_full_transcom_handle          = NULL;
        ((hal_timer_dev_struct *)p_struct)->timer_dma.error_handle                              = NULL;
        ((hal_timer_dev_struct *)p_struct)->error_state                                         = HAL_TIMER_ERROR_NONE;
        ((hal_timer_dev_struct *)p_struct)->state                                               = HAL_TIMER_STATE_NONE;
        ((hal_timer_dev_struct *)p_struct)->mutex                                               = HAL_MUTEX_UNLOCKED;
        break;
    default:
        HAL_DEBUGW("parameter [struct_type] value is undefine");
        return HAL_ERR_VAL;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      deinitialize TIMER and device structure
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_deinit(hal_timer_dev_struct *timer_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    timer_dev->state = HAL_TIMER_STATE_BUSY;

    /* deinit a TIMER */
    switch(timer_dev->periph) {
    case TIMER0:
        /* reset TIMER0 */
        hal_rcu_periph_reset_enable(RCU_TIMER0RST);
        hal_rcu_periph_reset_disable(RCU_TIMER0RST);
        break;

#if (defined(GD32F350) || defined(GD32F330))
    case TIMER1:
        /* reset TIMER1 */
        hal_rcu_periph_reset_enable(RCU_TIMER1RST);
        hal_rcu_periph_reset_disable(RCU_TIMER1RST);
        break;
#endif
    case TIMER2:
        /* reset TIMER2 */
        hal_rcu_periph_reset_enable(RCU_TIMER2RST);
        hal_rcu_periph_reset_disable(RCU_TIMER2RST);
        break;
#ifdef GD32F350
    case TIMER5:
        /* reset TIMER5 */
        hal_rcu_periph_reset_enable(RCU_TIMER5RST);
        hal_rcu_periph_reset_disable(RCU_TIMER5RST);
        break;
#endif
    case TIMER13:
        /* reset TIMER13 */
        hal_rcu_periph_reset_enable(RCU_TIMER13RST);
        hal_rcu_periph_reset_disable(RCU_TIMER13RST);
        break;
    case TIMER14:
        /* reset TIMER14 */
        hal_rcu_periph_reset_enable(RCU_TIMER14RST);
        hal_rcu_periph_reset_disable(RCU_TIMER14RST);
        break;
    case TIMER15:
        /* reset TIMER15 */
        hal_rcu_periph_reset_enable(RCU_TIMER15RST);
        hal_rcu_periph_reset_disable(RCU_TIMER15RST);
        break;
    case TIMER16:
        /* reset TIMER16 */
        hal_rcu_periph_reset_enable(RCU_TIMER16RST);
        hal_rcu_periph_reset_disable(RCU_TIMER16RST);
        break;
    default:
        break;
    }

    /* initialize the TIMER structure with the default values */
    hal_timer_struct_init(HAL_TIMER_DEV_STRUCT, timer_dev);
    /* change TIMER error state, service channel and state */
    timer_dev->error_state = HAL_TIMER_ERROR_NONE;
    timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_NONE;
    timer_dev->state = HAL_TIMER_STATE_RESET;

    return HAL_ERR_NONE;
}

/*!
    \brief      start TIMER counter
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_counter_start(hal_timer_dev_struct *timer_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    _timer_enable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER counter
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_counter_stop(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER counter and update interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: TIMER interrupt user callback function pointer structure
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_counter_start_interrupt(hal_timer_dev_struct *timer_dev, hal_timer_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == timer_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [timer_dev] or pointer [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    /* TIMER update interrupt handler set */
    timer_dev->timer_irq.update_handle = p_irq->update_handle;
    /* clear the TIMER interrupt flag */
    hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_UP);
    /* enable the TIMER interrupt */
    hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_UP);
    /* enable a TIMER */
    _timer_enable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER counter and update interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_counter_stop_interrupt(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    timer_dev->timer_irq.update_handle = NULL;
    /* clear the TIMER interrupt flag */
    hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_UP);
    /* disable the TIMER interrupt */
    hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_UP);
    /* enable a TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER counter and update DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  dmacb: TIMER DMA callback function pointer structure
                  update_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER update DMA request
                  channelx_capture_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel input capture DMA request
                  channelx_compare_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel output compare DMA request
                  commutation_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER commutation DMA request
                  trigger_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER trigger DMA request
                  error_handle: TIMER DMA transfer error interrupt handler
    \param[in]  mem_addr: TIMER DMA transfer memory address
    \param[in]  dma_length: TIMER DMA transfer count, 0~65535
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_counter_start_dma(hal_timer_dev_struct *timer_dev, hal_timer_dma_handle_cb_struct *dmacb, uint32_t *mem_addr, uint16_t dma_length)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if(NULL == mem_addr) {
        HAL_DEBUGE("pointer [mem_addr] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    timer_dev->timer_dma.update_dma_full_transcom_handle = dmacb->update_dma_full_transcom_handle;
    timer_dev->timer_dma.error_handle = dmacb->error_handle;

    timer_dev->p_dma_timer[TIMER_DMA_ID_UP]->dma_irq.full_finish_handle = _update_dma_full_transfer_complete;
    timer_dev->p_dma_timer[TIMER_DMA_ID_UP]->dma_irq.half_finish_handle = NULL;
    timer_dev->p_dma_timer[TIMER_DMA_ID_UP]->dma_irq.error_handle = _timer_dma_error;

    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_UP], (uint32_t)mem_addr, TIMER_CAR_ADDRESS(timer_dev->periph), dma_length, NULL);
    /* enable the TIMER DMA update request */
    hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_UPD);
    /* enable a TIMER */
    _timer_enable(timer_dev);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER counter and update DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_counter_stop_dma(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* disable the TIMER DMA update request */
    hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_UPD);
    /* stop DMA transfer */
    hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_UP]);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER channel input capture mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_input_capture_start(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable a TIMER */
    _timer_enable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER channel input capture mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_input_capture_stop(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER channel input capture mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  p_irq: TIMER interrupt user callback function pointer structure
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_input_capture_start_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    /* TIMER input capture interrupt handler set */
    timer_dev->timer_irq.channelx_capture_handle = p_irq->channelx_capture_handle;
    switch(channel) {
    case TIMER_CH_0:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
        break;
    case TIMER_CH_1:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
        break;
    case TIMER_CH_2:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH2);
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH2);
        break;
    case TIMER_CH_3:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH3);
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH3);
        break;
    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }
    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable a TIMER */
    _timer_enable(timer_dev);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER channel input capture mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_input_capture_stop_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    /* TIMER input capture interrupt handler reset */
    timer_dev->timer_irq.channelx_capture_handle = NULL;
    switch(channel) {
    case TIMER_CH_0:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
        break;
    case TIMER_CH_1:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
        break;
    case TIMER_CH_2:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH2);
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH2);
        break;
    case TIMER_CH_3:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH3);
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH3);
        break;
    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }
    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER channel input capture and channel DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,14..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  dmacb: TIMER DMA interrupt user callback function pointer structure
                  update_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER update DMA request
                  channelx_capture_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel input capture DMA request
                  channelx_compare_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel output compare DMA request
                  commutation_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER commutation DMA request
                  trigger_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER trigger DMA request
                  error_handle: TIMER DMA transfer error interrupt handler
    \param[in]  mem_addr: TIMER DMA transfer memory address
    \param[in]  dma_length: TIMER DMA transfer count, 0~65535
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_input_capture_start_dma(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_dma_handle_cb_struct *dmacb, uint32_t *mem_addr, uint16_t dma_length)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    /* lock TIMER */
    HAL_LOCK(timer_dev);

    timer_dev->timer_dma.channelx_capture_dma_full_transcom_handle = dmacb->channelx_capture_dma_full_transcom_handle;
    timer_dev->timer_dma.error_handle = dmacb->error_handle;

    switch(channel) {
    case TIMER_CH_0:
        /* channel DMA config */
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.error_handle = _timer_dma_error;
        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0], TIMER_CH0CV_ADDRESS(timer_dev->periph), (uint32_t)mem_addr, dma_length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
        break;
    case TIMER_CH_1:
        /* channel DMA config */
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.error_handle = _timer_dma_error;
        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1], TIMER_CH1CV_ADDRESS(timer_dev->periph), (uint32_t)mem_addr, dma_length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
        break;
    case TIMER_CH_2:
        /* channel DMA config */
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.error_handle = _timer_dma_error;
        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH2], TIMER_CH2CV_ADDRESS(timer_dev->periph), (uint32_t)mem_addr, dma_length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH2D);
        break;
    case TIMER_CH_3:
        /* channel DMA config */
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]->dma_irq.error_handle = _timer_dma_error;
        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH3], TIMER_CH3CV_ADDRESS(timer_dev->periph), (uint32_t)mem_addr, dma_length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH3D);
        break;
    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }
    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable a TIMER */
    _timer_enable(timer_dev);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER channel input capture and channel DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,14..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_input_capture_stop_dma(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    switch(channel) {
    case TIMER_CH_0:
        /* disable the TIMER DMA */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]);
        break;
    case TIMER_CH_1:
        /* disable the TIMER DMA */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]);
        break;
    case TIMER_CH_2:
        /* disable the TIMER DMA */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH2D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]);
        break;
    case TIMER_CH_3:
        /* disable the TIMER DMA */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH3D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]);
        break;
    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }
    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER channel output compare mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_start(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    _timer_enable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER channel output compare mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_stop(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    hals_timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER channel output compare mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  p_irq: TIMER interrupt user callback function pointer structure
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_start_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* TIMER output compare interrupt handler set */
    timer_dev->timer_irq.channelx_compare_handle = p_irq->channelx_compare_handle;
    switch(channel) {
    case TIMER_CH_0:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
        /* enable the TIMER compare 0 interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
        break;

    case TIMER_CH_1:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
        /* enable the TIMER compare 1 interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
        break;

    case TIMER_CH_2:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH2);
        /* enable the TIMER compare 2 interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH2);
        break;

    case TIMER_CH_3:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH3);
        /* enable the TIMER compare 3 interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH3);
        break;
    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }

    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    _timer_enable(timer_dev);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER channel output compare mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_stop_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* TIMER output compare interrupt handler reset */
    timer_dev->timer_irq.channelx_compare_handle = NULL;
    switch(channel) {
    case TIMER_CH_0:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
        /* disable the TIMER compare 0 interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
        break;

    case TIMER_CH_1:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
        /* disable the TIMER compare 1 interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
        break;

    case TIMER_CH_2:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH2);
        /* disable the TIMER compare 2 interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH2);
        break;

    case TIMER_CH_3:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH3);
        /* disable the TIMER compare 3 interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH3);
        break;
    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }

    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER output compare mode and channel DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,14..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  dmacb: TIMER DMA callback function pointer structure
                  update_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER update DMA request
                  channelx_capture_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel input capture DMA request
                  channelx_compare_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel output compare DMA request
                  commutation_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER commutation DMA request
                  trigger_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER trigger DMA request
    \param[in]  mem_addr: TIMER DMA transfer memory address
    \param[in]  dma_length: TIMER DMA transfer count, 0~65535
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_start_dma(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_dma_handle_cb_struct *dmacb, uint32_t *mem_addr, uint16_t dma_length)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    timer_dev->timer_dma.channelx_compare_dma_full_transcom_handle = dmacb->channelx_compare_dma_full_transcom_handle;
    timer_dev->timer_dma.error_handle = dmacb->error_handle;

    switch(channel) {
    case TIMER_CH_0:
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.full_finish_handle = _channelx_compare_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.error_handle = _timer_dma_error;
        /* enable the DMA CH0 interrupt */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0], (uint32_t)mem_addr, TIMER_CH0CV_ADDRESS(timer_dev->periph), dma_length, NULL);
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
        break;

    case TIMER_CH_1:
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.full_finish_handle = _channelx_compare_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.error_handle = _timer_dma_error;
        /* enable the DMA CH1 interrupt */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1], (uint32_t)mem_addr, TIMER_CH1CV_ADDRESS(timer_dev->periph), dma_length, NULL);
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
        break;

    case TIMER_CH_2:
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.full_finish_handle = _channelx_compare_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.error_handle = _timer_dma_error;
        /* enable the DMA CH2 interrupt */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH2], (uint32_t)mem_addr, TIMER_CH2CV_ADDRESS(timer_dev->periph), dma_length, NULL);
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH2D);
        break;

    case TIMER_CH_3:
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]->dma_irq.full_finish_handle = _channelx_compare_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]->dma_irq.error_handle = _timer_dma_error;
        /* enable the DMA CH3 interrupt */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH3], (uint32_t)mem_addr, TIMER_CH3CV_ADDRESS(timer_dev->periph), dma_length, NULL);
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH3D);
        break;

    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }

    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    _timer_enable(timer_dev);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER channel output compare mode and channel DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,14..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_stop_dma(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    switch(channel) {
    case TIMER_CH_0:
        /* disbale the DMA CH0 interrupt */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]);
        break;

    case TIMER_CH_1:
        /* disbale the DMA CH0 interrupt */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]);
        break;

    case TIMER_CH_2:
        /* disbale the DMA CH0 interrupt */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH2D);
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]);
        break;

    case TIMER_CH_3:
        /* disbale the DMA CH0 interrupt */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH3D);
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]);
        break;
    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }

    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER complementary channel output compare mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_complementary_channel_start(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* configure TIMER channel complementary output enable state */
    hals_timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    _timer_enable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementary channel output compare mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_complementary_channel_stop(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* configure TIMER channel complementary output enable state */
    hals_timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER complementary channel output compare mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[in]  p_irq: TIMER interrupt user callback function pointer structure
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_complementary_channel_start_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* TIMER output compare interrupt handler set */
    timer_dev->timer_irq.channelx_compare_handle = p_irq->channelx_compare_handle;

    switch(channel) {
    case TIMER_CH_0:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
        /* enable the TIMER compare 0 interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
        break;

    case TIMER_CH_1:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
        /* enable the TIMER compare 1 interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
        break;

    case TIMER_CH_2:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH2);
        /* enable the TIMER compare 2 interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH2);
        break;

    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }

    /* configure TIMER channel complementary output enable state */
    hals_timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    _timer_enable(timer_dev);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementary channel output compare mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_complementary_channel_stop_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* TIMER output compare interrupt handler reset */
    timer_dev->timer_irq.channelx_compare_handle = NULL;
    switch(channel) {
    case TIMER_CH_0:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
        /* disable the TIMER compare 0 interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
        break;

    case TIMER_CH_1:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
        /* disable the TIMER compare 1 interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
        break;

    case TIMER_CH_2:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH2);
        /* disable the TIMER compare 2 interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH2);
        break;

    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }

    /* configure TIMER channel complementary output enable state */
    hals_timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_DISABLE);
    if(RESET == (TIMER_CHCTL2(timer_dev->periph) & (TIMER_CHCTL2_CH0NEN | TIMER_CHCTL2_CH1NEN | TIMER_CHCTL2_CH2NEN))) {
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_BRK);
    }
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);

    return ret_val;
}

/*!
    \brief      start TIMER complementary channel output compare mode and channel DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[in]  dmacb: TIMER DMA callback function pointer struct
                  update_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER update DMA request
                  channelx_capture_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel input capture DMA request
                  channelx_compare_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel output compare DMA request
                  commutation_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER commutation DMA request
                  trigger_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER trigger DMA request
                  error_handle: TIMER DMA transfer error interrupt handler
    \param[in]  mem_addr: TIMER DMA transfer memory address
    \param[in]  dma_length: TIMER DMA transfer count, 0~65535
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_complementary_channel_start_dma(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_dma_handle_cb_struct *dmacb, uint32_t *mem_addr, uint16_t dma_length)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    timer_dev->timer_dma.channelx_compare_dma_full_transcom_handle = dmacb->channelx_compare_dma_full_transcom_handle;
    timer_dev->timer_dma.error_handle = dmacb->error_handle;

    switch(channel) {
    case TIMER_CH_0:
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.full_finish_handle = _channelx_compare_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.error_handle = _timer_dma_error;
        /* enable the DMA CH0 interrupt */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0], (uint32_t)mem_addr, TIMER_CH0CV_ADDRESS(timer_dev->periph), dma_length, NULL);
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
        break;

    case TIMER_CH_1:
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.full_finish_handle = _channelx_compare_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.error_handle = _timer_dma_error;
        /* enable the DMA CH1 interrupt */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1], (uint32_t)mem_addr, TIMER_CH1CV_ADDRESS(timer_dev->periph), dma_length, NULL);
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
        break;

    case TIMER_CH_2:
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.full_finish_handle = _channelx_compare_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.error_handle = _timer_dma_error;
        /* enable the DMA CH2 interrupt */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH2], (uint32_t)mem_addr, TIMER_CH2CV_ADDRESS(timer_dev->periph), dma_length, NULL);
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH2D);
        break;

    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }

    /* configure TIMER channel enable state */
    hals_timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    _timer_enable(timer_dev);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementary channel output compare mode and channel DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_output_compare_complementary_channel_stop_dma(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    switch(channel) {
    case TIMER_CH_0:
        /* disbale the DMA CH0 interrupt */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]);
        break;

    case TIMER_CH_1:
        /* disbale the DMA CH0 interrupt */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]);
        break;

    case TIMER_CH_2:
        /* disbale the DMA CH0 interrupt */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH2D);
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]);
        break;

    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }

    /* configure TIMER channel complementary output enable state */
    hals_timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      TIMER single pulse mode configure
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_singlepulse: TIMER single pulse mode channel configuration struct
                  sp_compare_mode: the argument could be selected from enumeration <hal_timer_output_compare_enum>
                  sp_oc_pulse_value:0~65535,(for TIMER1 0x00000000~0xFFFFFFFF)
                  sp_oc_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_POLARITY_HIGH : channel output polarity is high
      \arg          TIMER_OC_POLARITY_LOW : channel output polarity is low
                  sp_oc_idlestate:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_IDLE_STATE_LOW : idle state of channel output is high
      \arg          TIMER_OC_IDLE_STATE_HIGH : idle state of channel output is low
                  sp_ocn_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OCN_POLARITY_HIGH : channel complementary output polarity is high
      \arg          TIMER_OCN_POLARITY_LOW : channel complementary output polarity is low
                  sp_ocn_idlestate:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OCN_IDLE_STATE_LOW : idle state of channel complementary output is high
      \arg          TIMER_OCN_IDLE_STATE_HIGH :  idle state of channel complementary output is low
                  sp_oc_fastmode:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_FAST_ENABLE : channel output fast function enable
      \arg          TIMER_OC_FAST_DISABLE : channel output fast function disable
                  sp_oc_clearmode:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_CLEAR_ENABLE : channel output clear function enable
      \arg          TIMER_OC_CLEAR_DISABLE : channel output clear function disable
                  sp_ic_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_POLARITY_RISING : input capture rising edge
      \arg          TIMER_IC_POLARITY_FALLING : input capture falling edge
      \arg          TIMER_IC_POLARITY_BOTH_EDGE : input capture both edge
                  sp_ic_selection:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_SELECTION_DIRECTTI : channel y is configured as input and icy is mapped on CIy
      \arg          TIMER_IC_SELECTION_INDIRECTTI : channel y is configured as input and icy is mapped on opposite input
      \arg          TIMER_IC_SELECTION_ITS : channel y is configured as input and icy is mapped on ITS
                  sp_ic_filter: 0~15
    \param[in]  channel_out: TIMER output channel.The channel will output single pulse
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  channel_in: TIMER input channel.If the channel input a active signal,TIMER will start count.
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_single_pulse_mode_channel_config(hal_timer_dev_struct *timer_dev, hal_timer_single_pulse_struct *timer_singlepulse, uint16_t channel_out, uint16_t channel_in)
{
    static hal_timer_input_capture_struct input_capture;
    static hal_timer_output_compare_struct output_compare;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(NULL == timer_singlepulse) {
        HAL_DEBUGE("pointer [timer_singlepulse] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(channel_out != channel_in) {
        /* TIMER output compare mode struct config */
        output_compare.compare_mode = timer_singlepulse->sp_compare_mode;
        output_compare.oc_pulse_value = timer_singlepulse->sp_oc_pulse_value;
        output_compare.oc_polarity = timer_singlepulse->sp_oc_polarity;
        output_compare.ocn_polarity = timer_singlepulse->sp_ocn_polarity;
        output_compare.oc_idlestate = timer_singlepulse->sp_oc_idlestate;
        output_compare.ocn_idlestate = timer_singlepulse->sp_ocn_idlestate;
        output_compare.oc_shadow = TIMER_OC_SHADOW_DISABLE;
        output_compare.oc_fastmode = timer_singlepulse->sp_oc_fastmode;
        output_compare.oc_clearmode = timer_singlepulse->sp_oc_clearmode;
        switch(channel_out) {
        case TIMER_CH_0:
            /* configure TIMER output compare mode */
            hals_timer_channel_output_config(timer_dev->periph, TIMER_CH_0, &output_compare);
            break;
        case TIMER_CH_1:
            /* configure TIMER output compare mode */
            hals_timer_channel_output_config(timer_dev->periph, TIMER_CH_1, &output_compare);
            break;
        default:
            HAL_DEBUGE("parameter [channel_out] value is invalid");
            return HAL_ERR_VAL;
        }

        /* TIMER input capture mode structure config */
        input_capture.ic_polarity  = timer_singlepulse->sp_ic_polarity;
        input_capture.ic_selection = timer_singlepulse->sp_ic_selection;
        input_capture.ic_prescaler = TIMER_IC_PRESCALER_OFF;
        input_capture.ic_filter    = timer_singlepulse->sp_ic_filter;
        switch(channel_in) {
        case TIMER_CH_0:
            /* configure TIMER input capture mode */
            hals_timer_channel_input_capture_config(timer_dev->periph, TIMER_CH_0, &input_capture);
            /* select TIMER input trigger source */
            hals_timer_input_trigger_source_select(timer_dev->periph, TIMER_SMCFG_TRGSEL_CI0FE0);
            /* select TIMER slave mode */
            hals_timer_slave_mode_select(timer_dev->periph, TIMER_SLAVE_MODE_EVENT);
            break;
        case TIMER_CH_1:
            /* configure TIMER input capture mode */
            hals_timer_channel_input_capture_config(timer_dev->periph, TIMER_CH_1, &input_capture);
            /* select TIMER input trigger source */
            hals_timer_input_trigger_source_select(timer_dev->periph, TIMER_SMCFG_TRGSEL_CI1FE1);
            /* select TIMER slave mode */
            hals_timer_slave_mode_select(timer_dev->periph, TIMER_SLAVE_MODE_EVENT);
            break;
        default:
            HAL_DEBUGE("parameter [channel_in] value is invalid");
            return HAL_ERR_VAL;
        }
        return HAL_ERR_NONE;
    } else {
        return HAL_ERR_VAL;
    }
}

/*!
    \brief      start TIMER single pulse mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_single_pulse_start(hal_timer_dev_struct *timer_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    /* lock TIMER */
    HAL_LOCK(timer_dev);

    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER single pulse mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_single_pulse_stop(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    /* lock TIMER */
    HAL_LOCK(timer_dev);

    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER single pulse mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: TIMER interrupt user callback function pointer structure
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_single_pulse_start_interrupt(hal_timer_dev_struct *timer_dev, hal_timer_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    /* TIMER input capture interrupt handler set */
    timer_dev->timer_irq.channelx_capture_handle = p_irq->channelx_capture_handle;
    /* TIMER output compare interrupt handler set */
    timer_dev->timer_irq.channelx_compare_handle = p_irq->channelx_compare_handle;
    /* clear the TIMER interrupt flag */
    hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
    hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
    /* enable the TIMER interrupt */
    hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
    hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER single pulse mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_single_pulse_stop_interrupt(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    /* TIMER input capture interrupt handler set */
    timer_dev->timer_irq.channelx_capture_handle = NULL;
    /* TIMER output compare interrupt handler set */
    timer_dev->timer_irq.channelx_compare_handle = NULL;
    /* clear the TIMER interrupt flag */
    hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
    hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
    /* disable the TIMER interrupt */
    hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
    hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER complementary channel single pulse mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel_out:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_single_pulse_complementary_channel_start(hal_timer_dev_struct *timer_dev, uint16_t channel_out)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    switch(channel_out) {
    case TIMER_CH_0:
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
        hals_timer_channel_complementary_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCXN_ENABLE);
        break;
    case TIMER_CH_1:
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
        hals_timer_channel_complementary_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCXN_ENABLE);
        break;
    default:
        HAL_DEBUGE("parameter [channel_out] value is invalid");
        return HAL_ERR_VAL;
    }
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementary channel single pulse mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel_out:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_single_pulse_complementary_channel_stop(hal_timer_dev_struct *timer_dev, uint16_t channel_out)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    switch(channel_out) {
    case TIMER_CH_0:
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
        hals_timer_channel_complementary_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCXN_DISABLE);
        break;
    case TIMER_CH_1:
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
        hals_timer_channel_complementary_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCXN_DISABLE);
        break;
    default:
        HAL_DEBUGE("parameter [channel_out] value is invalid");
        return HAL_ERR_VAL;
    }
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER complementary channel single pulse mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel_out:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
    \param[in]  p_irq: TIMER interrupt user callback function pointer struct
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_single_pulse_complementary_channel_start_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel_out, hal_timer_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    /* TIMER input capture interrupt handler set */
    timer_dev->timer_irq.channelx_capture_handle = p_irq->channelx_capture_handle;
    /* TIMER output compare interrupt handler set */
    timer_dev->timer_irq.channelx_compare_handle = p_irq->channelx_compare_handle;
    /* clear the TIMER interrupt flag */
    hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
    hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
    /* enable the TIMER interrupt */
    hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
    hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
    switch(channel_out) {
    case TIMER_CH_0:
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
        hals_timer_channel_complementary_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCXN_ENABLE);
        break;
    case TIMER_CH_1:
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
        hals_timer_channel_complementary_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCXN_ENABLE);
        break;
    default:
        HAL_DEBUGE("parameter [channel_out] value is invalid");
        return HAL_ERR_VAL;
    }
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementary channel single pulse mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel_out:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_single_pulse_complementary_channel_stop_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel_out)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    /* TIMER input capture interrupt handler set */
    timer_dev->timer_irq.channelx_capture_handle = NULL;
    /* TIMER output compare interrupt handler set */
    timer_dev->timer_irq.channelx_compare_handle = NULL;
    /* clear the TIMER interrupt flag */
    hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
    hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
    /* disable the TIMER interrupt */
    hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
    hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
    switch(channel_out) {
    case TIMER_CH_0:
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
        hals_timer_channel_complementary_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCXN_DISABLE);
        break;
    case TIMER_CH_1:
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
        hals_timer_channel_complementary_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCXN_DISABLE);
        break;
    default:
        HAL_DEBUGE("parameter [channel_out] value is invalid");
        return HAL_ERR_VAL;
    }
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      configure TIMER slave mode and interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_slavemode: TIMER slave mode configuration structure
                  slavemode: the argument could be selected from enumeration <hal_timer_slave_mode_enum>
                  trigger_selection: the argument could be selected from enumeration <hal_timer_input_trigger_source_enum>
                  trigger_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_CLOCK_TRIGGER_ETI_POLARITY_RISING: trigger input source is ETI, active high or rising edge active
      \arg          TIMER_CLOCK_TRIGGER_ETI_POLARITY_FALLING: trigger input source is ETI, active low or falling edge active
      \arg          TIMER_CLOCK_TRIGGER_POLARITY_RISING: trigger input source is CIx(x=0,1), rising edge active
      \arg          TIMER_CLOCK_TRIGGER_POLARITY_FALLING: trigger input source is CIx(x=0,1), falling edge active
      \arg          TIMER_CLOCK_TRIGGER_POLARITY_BOTH_EDGE: trigger input source is CI0F_ED, both rising and falling edge active
                  trigger_prescaler:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_EXT_TRI_PRESCALER_OFF: external trigger no divided
      \arg          TIMER_EXT_TRI_PRESCALER_DIV2: external trigger divided by 2
      \arg          TIMER_EXT_TRI_PRESCALER_DIV4: external trigger divided by 4
      \arg          TIMER_EXT_TRI_PRESCALER_DIV8: external trigger divided by 8
                  trigger_filter: 0~15
    \param[in]  p_irq: TIMER interrupt user callback function pointer structure
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_slave_mode_interrupt_config(hal_timer_dev_struct *timer_dev, hal_timer_slave_mode_struct *timer_slavemode, hal_timer_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == timer_dev) || (NULL == timer_slavemode)) {
        HAL_DEBUGE("pointer [timer_dev] or [timer_slavemode] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#if defined(GD32F350)
    /* check the parameters */
    if((TIMER5 == timer_dev->periph) || (TIMER13 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)) {
        HAL_DEBUGE("the [timer_slavemode->slavemode] of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#else
    /* check the parameters */
    if((TIMER13 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)) {
        HAL_DEBUGE("the [timer_slavemode->slavemode] of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* GD32F350 */
    /* check the parameters */
    if((TIMER_TRIGGER_SOURCE_ETIFP == timer_slavemode->trigger_selection) && (TIMER14 == timer_dev->periph)) {
        HAL_DEBUGE("the [timer_slavemode->trigger_selection] value of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
    if(TIMER_TRIGGER_SOURCE_DISABLE == timer_slavemode->trigger_selection) {
        HAL_DEBUGE("the [timer_slavemode->trigger_selection] value of [timer_dev] is invalid");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* TIMER trigger interrupt handler set */
    timer_dev->timer_irq.trigger_handle = p_irq->trigger_handle;

    switch(timer_slavemode->trigger_selection) {
    case TIMER_TRIGGER_SOURCE_ITI0:
        /* no need to config polarity, prescaler, filter */
        break;
    case TIMER_TRIGGER_SOURCE_ITI1:
        /* no need to config polarity, prescaler, filter */
        break;
    case TIMER_TRIGGER_SOURCE_ITI2:
        /* no need to config polarity, prescaler, filter */
        break;
    case TIMER_TRIGGER_SOURCE_ITI3:
        /* no need to config polarity, prescaler, filter */
        break;
    case TIMER_TRIGGER_SOURCE_CI0FE0:
        /* reset the CH0EN bit */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0EN | TIMER_CHCTL2_CH0NEN));
        /* reset the CH0P and CH0NP bits */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
        /* config polarity */
        TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)(timer_slavemode->trigger_polarity);
        /* reset the CH0CAPFLT bit */
        TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
        /* config filter */
        TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_filter) << 4U);
        break;
    case TIMER_TRIGGER_SOURCE_CI1FE1:
        /* reset the CH1EN bit */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1EN | TIMER_CHCTL2_CH1NEN));
        /* reset the CH1P and CH1NP bits */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
        /* config polarity */
        TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_polarity) << 4U);
        /* reset the CH1CAPFLT bit */
        TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
        /* config filter */
        TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_filter) << 12U);
        break;
    case TIMER_TRIGGER_SOURCE_CI0FED:
        /* reset the CH0EN bit */
        TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1EN | TIMER_CHCTL2_CH1NEN));
        /* reset the CH0CAPFLT bit */
        TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
        /* config filter */
        TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_filter) << 4U);
        break;
    case TIMER_TRIGGER_SOURCE_ETIFP:
        /* configure TIMER external trigger input */
        hals_timer_external_trigger_config(timer_dev->periph, timer_slavemode->trigger_prescaler,
                                           timer_slavemode->trigger_polarity, timer_slavemode->trigger_filter);
        break;
    default:
        HAL_DEBUGW("parameter [timer_slavemode->trigger_selection] value is undefine");
        return HAL_ERR_VAL;
    }

    /* select TIMER input trigger source  */
    hals_timer_input_trigger_source_select(timer_dev->periph, timer_slavemode->trigger_selection);
    /* select TIMER slave mode */
    hals_timer_slave_mode_select(timer_dev->periph, timer_slavemode->slavemode);

    /* enable the TIMER interrupt */
    hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_TRG);
    /* disable the TIMER DMA trigger request */
    hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_TRGD);

    return HAL_ERR_NONE;
}

/*!
    \brief      start TIMER decoder mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_0_1: TIMER channel0 and TIMER channel1
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_decoder_start(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    switch(channel) {
    case TIMER_CH_0:
        /* configure TIMER channel 0 enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
        break;
    case TIMER_CH_1:
        /* configure TIMER channel 1 enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
        break;
    case TIMER_CH_0_1:
        /* configure TIMER channel 0 and channel 1 enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
        break;
    default:
        HAL_DEBUGW("parameter [channel] value is undefine");
        return HAL_ERR_VAL;
    }
    /* enable a TIMER */
    hals_timer_enable(timer_dev->periph);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER decoder mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_0_1: TIMER channel0 and TIMER channel1
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_decoder_stop(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    switch(channel) {
    case TIMER_CH_0:
        /* configure TIMER channel 0 enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
        break;
    case TIMER_CH_1:
        /* configure TIMER channel 1 enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
        break;
    case TIMER_CH_0_1:
        /* configure TIMER channel 0 and channel 1 enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
        break;
    default:
        HAL_DEBUGW("parameter [channel] value is undefine");
        return HAL_ERR_VAL;
    }
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER decoder mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_0_1: TIMER channel0 and TIMER channel1
    \param[in]  p_irq: TIMER interrupt user callback function pointer structure
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_decoder_start_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);

    /* TIMER output compare interrupt handler set */
    timer_dev->timer_irq.channelx_capture_handle = p_irq->channelx_capture_handle;
    switch(channel) {
    case TIMER_CH_0:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
        /* configure TIMER channel 0 enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
        break;
    case TIMER_CH_1:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
        /* configure TIMER channel 1 enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
        break;
    case TIMER_CH_0_1:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
        break;
    default:
        HAL_DEBUGW("parameter [channel] value is undefine");
        return HAL_ERR_VAL;
    }
    /* enable a TIMER */
    hals_timer_enable(timer_dev->periph);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER decoder mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_0_1: TIMER channel0 and TIMER channel1
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_decoder_stop_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* TIMER output compare interrupt handler set */
    timer_dev->timer_irq.channelx_capture_handle = NULL;
    switch(channel) {
    case TIMER_CH_0:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
        break;
    case TIMER_CH_1:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
        break;
    case TIMER_CH_0_1:
        /* clear the TIMER interrupt flag */
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
        break;
    default:
        HAL_DEBUGW("parameter [channel] value is undefine");
        return HAL_ERR_VAL;
    }
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER decoder mode and channel DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_0_1: TIMER channel0 and TIMER channel1
    \param[in]  dmacb: TIMER DMA callback function pointer structure
                  update_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER update DMA request
                  channelx_capture_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel input capture DMA request
                  channelx_compare_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel output compare DMA request
                  commutation_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER commutation DMA request
                  trigger_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER trigger DMA request
                  error_handle: TIMER DMA transfer error interrupt handler
    \param[in]  decoder_dma: TIMER decoder mode DMA transfer configuration structure
                  mem_addr0: TIMER DMA transfer memory address for TIMER_CH_0 DMA request
                  mem_addr1: TIMER DMA transfer memory address for TIMER_CH_1 DMA request
                  length: TIMER DMA transfer count, 0~65535
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_decoder_start_dma(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_dma_handle_cb_struct *dmacb, hal_timer_decoder_dma_config_struct *decoder_dma)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    timer_dev->timer_dma.channelx_capture_dma_full_transcom_handle = dmacb->channelx_capture_dma_full_transcom_handle;
    timer_dev->timer_dma.error_handle = dmacb->error_handle;

    switch(channel) {
    case TIMER_CH_0:
        /* channel DMA config */
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.error_handle = _timer_dma_error;
        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0], TIMER_CH0CV_ADDRESS(timer_dev->periph), (uint32_t)decoder_dma->mem_addr0, decoder_dma->length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
        break;
    case TIMER_CH_1:
        /* channel DMA config */
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.error_handle = _timer_dma_error;
        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1], TIMER_CH1CV_ADDRESS(timer_dev->periph), (uint32_t)decoder_dma->mem_addr1, decoder_dma->length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
        break;
    case TIMER_CH_0_1:
        /* channel0 DMA config */
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.error_handle = _timer_dma_error;
        /* channel1 DMA config */
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.error_handle = _timer_dma_error;
        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0], TIMER_CH0CV_ADDRESS(timer_dev->periph), (uint32_t)decoder_dma->mem_addr0, decoder_dma->length, NULL);
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1], TIMER_CH1CV_ADDRESS(timer_dev->periph), (uint32_t)decoder_dma->mem_addr1, decoder_dma->length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
        break;
    default:
        HAL_DEBUGW("parameter [channel] value is undefine");
        return HAL_ERR_VAL;
    }
    /* enable a TIMER */
    hals_timer_enable(timer_dev->periph);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER decoder mode and channel DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_0_1: TIMER channel0 and TIMER channel1
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_decoder_stop_dma(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    switch(channel) {
    case TIMER_CH_0:
        /* disbale the DMA CH0 interrupt */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]);
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
        break;
    case TIMER_CH_1:
        /* disbale the DMA CH0 interrupt */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]);
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
        break;
    case TIMER_CH_0_1:
        /* disbale the DMA CH0 interrupt */
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]);
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]);
        /* configure TIMER channel enable state */
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
        hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
        break;
    default:
        HAL_DEBUGW("parameter [channel] value is undefine");
        return HAL_ERR_VAL;
    }
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER hall sensor mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_hall_sensor_start(hal_timer_dev_struct *timer_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
    /* enable a TIMER */
    hals_timer_enable(timer_dev->periph);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER hall sensor mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_hall_sensor_stop(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER hall sensor mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: TIMER interrupt user callback function pointer structure
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_hall_sensor_start_interrupt(hal_timer_dev_struct *timer_dev, hal_timer_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* TIMER output compare interrupt handler set */
    timer_dev->timer_irq.channelx_capture_handle = p_irq->channelx_capture_handle;
    /* clear the TIMER interrupt flag */
    hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
    /* enable the TIMER interrupt */
    hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
    /* enable a TIMER */
    hals_timer_enable(timer_dev->periph);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER hall sensor mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_hall_sensor_stop_interrupt(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* TIMER output compare interrupt handler set */
    timer_dev->timer_irq.channelx_compare_handle = NULL;
    /* clear the TIMER interrupt flag */
    hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
    /* disable the TIMER interrupt */
    hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER hall sensor mode and channel DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  dmacb: TIMER DMA callback function pointer struct
                  update_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER update DMA request
                  channelx_capture_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel input capture DMA request
                  channelx_compare_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel output compare DMA request
                  commutation_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER commutation DMA request
                  trigger_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER trigger DMA request
                  error_handle: TIMER DMA transfer error interrupt handler
    \param[in]  mem_addr: TIMER DMA transfer memory address
    \param[in]  dma_length: TIMER DMA transfer count, 0~65535
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_hall_sensor_start_dma(hal_timer_dev_struct *timer_dev, hal_timer_dma_handle_cb_struct *dmacb, uint32_t *mem_addr, uint16_t dma_length)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    timer_dev->timer_dma.channelx_capture_dma_full_transcom_handle = dmacb->channelx_capture_dma_full_transcom_handle;
    timer_dev->timer_dma.error_handle = dmacb->error_handle;

    timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;;
    timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.half_finish_handle = NULL;
    timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.error_handle = _timer_dma_error;
    /* enable the DMA CH0 interrupt */
    hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0], TIMER_CH0CV_ADDRESS(timer_dev->periph), (uint32_t)mem_addr, dma_length, NULL);
    hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);

    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
    /* enable a TIMER */
    hals_timer_enable(timer_dev->periph);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER hall sensor mode and channel DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_hall_sensor_stop_dma(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* disbale the DMA CH0 interrupt */
    hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
    /* stop DMA transfer */
    hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]);

    /* configure TIMER channel enable state */
    hals_timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);

    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER DMA transfer mode for writing data to TIMER
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  dmareq: specify which DMA request
                one or more parameters can be selected which are shown as below:
      \arg        TIMER_DMA_UPD: update DMA, TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_DMA_CH0D: channel 0 DMA request, TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH1D: channel 1 DMA request, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH2D: channel 2 DMA request, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH3D: channel 3 DMA request, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CMTD: commutation DMA request , TIMERx(x=0,14)
      \arg        TIMER_DMA_TRGD: trigger DMA request, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
    \param[in]  dmatcfg: TIMER DMA config parameter structure
                  start_addr: the argument could be selected from enumeration <hal_timer_dma_transfer_start_address_enum>
                  mem_addr:TIMER DMA transfer memory address
                  length: the argument could be selected from enumeration <hal_timer_dma_transfer_length_enum>
    \param[in]  dmacb: TIMER DMA callback function pointer structure
                  update_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER update DMA request
                  channelx_capture_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel input capture DMA request
                  channelx_compare_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel output compare DMA request
                  commutation_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER commutation DMA request
                  trigger_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER trigger DMA request
                  error_handle: TIMER DMA transfer error interrupt handler
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_dma_transfer_write_start(hal_timer_dev_struct *timer_dev, uint32_t dmareq, hal_timer_dma_transfer_config_struct *dmatcfg, hal_timer_dma_handle_cb_struct *dmacb)
{
    uint16_t length = 0;
    length = (uint16_t)((dmatcfg->length >> 8U) + 1);
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(NULL == dmatcfg) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    switch(dmareq) {
    case TIMER_DMA_UPD:
        timer_dev->timer_dma.update_dma_full_transcom_handle = dmacb->update_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_UP]->dma_irq.full_finish_handle = _update_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_UP]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_UP]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_UP], (uint32_t)dmatcfg->mem_addr, TIMER_DMATB_ADDRESS(timer_dev->periph), length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_UPD);
        break;
    case TIMER_DMA_CH0D:
        timer_dev->timer_dma.channelx_compare_dma_full_transcom_handle = dmacb->channelx_compare_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.full_finish_handle = _channelx_compare_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0], (uint32_t)dmatcfg->mem_addr, TIMER_DMATB_ADDRESS(timer_dev->periph), length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
        break;
    case TIMER_DMA_CH1D:
        timer_dev->timer_dma.channelx_compare_dma_full_transcom_handle = dmacb->channelx_compare_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.full_finish_handle = _channelx_compare_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1], (uint32_t)dmatcfg->mem_addr, TIMER_DMATB_ADDRESS(timer_dev->periph), length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
        break;
    case TIMER_DMA_CH2D:
        timer_dev->timer_dma.channelx_compare_dma_full_transcom_handle = dmacb->channelx_compare_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.full_finish_handle = _channelx_compare_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH2], (uint32_t)dmatcfg->mem_addr, TIMER_DMATB_ADDRESS(timer_dev->periph), length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH2D);
        break;
    case TIMER_DMA_CH3D:
        timer_dev->timer_dma.channelx_compare_dma_full_transcom_handle = dmacb->channelx_compare_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]->dma_irq.full_finish_handle = _channelx_compare_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH3], (uint32_t)dmatcfg->mem_addr, TIMER_DMATB_ADDRESS(timer_dev->periph), length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH3D);
        break;
    case TIMER_DMA_CMTD:
        timer_dev->timer_dma.commutation_dma_full_transcom_handle = dmacb->commutation_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CMT]->dma_irq.full_finish_handle = _commutation_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CMT]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CMT]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CMT], (uint32_t)dmatcfg->mem_addr, TIMER_DMATB_ADDRESS(timer_dev->periph), length, NULL);
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CMTD);
        break;
    case TIMER_DMA_TRGD:
        timer_dev->timer_dma.trigger_dma_full_transcom_handle = dmacb->trigger_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_TRG]->dma_irq.full_finish_handle = _trigger_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_TRG]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_TRG]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_TRG], (uint32_t)dmatcfg->mem_addr, TIMER_DMATB_ADDRESS(timer_dev->periph), length, NULL);
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_TRGD);
        break;
    default:
        HAL_DEBUGW("parameter [dmareq] value is undefine");
        return HAL_ERR_VAL;
    }
    /* configure the TIMER DMA transfer */
    hals_timer_dma_transfer_config(timer_dev->periph, dmatcfg->start_addr, dmatcfg->length);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER DMA transfer mode for writing data to TIMER
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  dmareq: specify which DMA request
                one or more parameters can be selected which are shown as below:
      \arg        TIMER_DMA_UPD: update DMA, TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_DMA_CH0D: channel 0 DMA request, TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH1D: channel 1 DMA request, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH2D: channel 2 DMA request, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH3D: channel 3 DMA request, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CMTD: commutation DMA request , TIMERx(x=0,14)
      \arg        TIMER_DMA_TRGD: trigger DMA request, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_dma_transfer_write_stop(hal_timer_dev_struct *timer_dev, uint32_t dmareq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    /* DMA config for request */
    switch(dmareq) {
    case TIMER_DMA_UPD:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_UP]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_UPD);
        break;
    case TIMER_DMA_CH0D:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
        break;
    case TIMER_DMA_CH1D:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
        break;
    case TIMER_DMA_CH2D:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH2D);
        break;
    case TIMER_DMA_CH3D:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH3D);
        break;
    case TIMER_DMA_CMTD:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CMT]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CMTD);
        break;
    case TIMER_DMA_TRGD:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_TRG]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_TRGD);
        break;
    default:
        HAL_DEBUGW("parameter [dmareq] value is undefine");
        return HAL_ERR_VAL;
    }
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      start TIMER DMA transfer mode for read data from TIMER
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  dmareq: specify which DMA request
                one or more parameters can be selected which are shown as below:
      \arg        TIMER_DMA_UPD: update DMA, TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_DMA_CH0D: channel 0 DMA request, TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH1D: channel 1 DMA request, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH2D: channel 2 DMA request, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH3D: channel 3 DMA request, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CMTD: commutation DMA request , TIMERx(x=0,14)
      \arg        TIMER_DMA_TRGD: trigger DMA request, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
    \param[in]  dmatcfg: TIMER DMA config parameter structure
                  start_addr: the argument could be selected from enumeration <hal_timer_dma_transfer_start_address_enum>
                  mem_addr:TIMER DMA transfer memory address
                  length: the argument could be selected from enumeration <hal_timer_dma_transfer_length_enum>
    \param[in]  dmacb: TIMER DMA callback function pointer structure
                  update_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER update DMA request
                  channelx_capture_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel input capture DMA request
                  channelx_compare_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel output compare DMA request
                  commutation_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER commutation DMA request
                  trigger_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER trigger DMA request
                  error_handle: TIMER DMA transfer error interrupt handler
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_dma_transfer_read_start(hal_timer_dev_struct *timer_dev, uint32_t dmareq, hal_timer_dma_transfer_config_struct *dmatcfg, hal_timer_dma_handle_cb_struct *dmacb)
{
    uint16_t length = 0;
    length = (uint16_t)((dmatcfg->length >> 8U) + 1);
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(NULL == dmatcfg) {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    switch(dmareq) {
    case TIMER_DMA_UPD:
        timer_dev->timer_dma.update_dma_full_transcom_handle = dmacb->update_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_UP]->dma_irq.full_finish_handle = _update_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_UP]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_UP]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_UP], TIMER_DMATB_ADDRESS(timer_dev->periph), (uint32_t)dmatcfg->mem_addr, length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_UPD);
        break;
    case TIMER_DMA_CH0D:
        timer_dev->timer_dma.channelx_capture_dma_full_transcom_handle = dmacb->channelx_capture_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0], TIMER_DMATB_ADDRESS(timer_dev->periph), (uint32_t)dmatcfg->mem_addr, length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
        break;
    case TIMER_DMA_CH1D:
        timer_dev->timer_dma.channelx_capture_dma_full_transcom_handle = dmacb->channelx_capture_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1], TIMER_DMATB_ADDRESS(timer_dev->periph), (uint32_t)dmatcfg->mem_addr, length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
        break;
    case TIMER_DMA_CH2D:
        timer_dev->timer_dma.channelx_capture_dma_full_transcom_handle = dmacb->channelx_capture_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH2], TIMER_DMATB_ADDRESS(timer_dev->periph), (uint32_t)dmatcfg->mem_addr, length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH2D);
        break;
    case TIMER_DMA_CH3D:
        timer_dev->timer_dma.channelx_capture_dma_full_transcom_handle = dmacb->channelx_capture_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]->dma_irq.full_finish_handle = _channelx_capture_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CH3], TIMER_DMATB_ADDRESS(timer_dev->periph), (uint32_t)dmatcfg->mem_addr, length, NULL);
        /* enable the TIMER DMA update request */
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CH3D);
        break;
    case TIMER_DMA_CMTD:
        timer_dev->timer_dma.commutation_dma_full_transcom_handle = dmacb->commutation_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CMT]->dma_irq.full_finish_handle = _commutation_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CMT]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_CMT]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_CMT], TIMER_DMATB_ADDRESS(timer_dev->periph), (uint32_t)dmatcfg->mem_addr, length, NULL);
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CMTD);
        break;
    case TIMER_DMA_TRGD:
        timer_dev->timer_dma.trigger_dma_full_transcom_handle = dmacb->trigger_dma_full_transcom_handle;
        timer_dev->timer_dma.error_handle = dmacb->error_handle;
        timer_dev->p_dma_timer[TIMER_DMA_ID_TRG]->dma_irq.full_finish_handle = _trigger_dma_full_transfer_complete;
        timer_dev->p_dma_timer[TIMER_DMA_ID_TRG]->dma_irq.half_finish_handle = NULL;
        timer_dev->p_dma_timer[TIMER_DMA_ID_TRG]->dma_irq.error_handle = _timer_dma_error;

        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->p_dma_timer[TIMER_DMA_ID_TRG], TIMER_DMATB_ADDRESS(timer_dev->periph), (uint32_t)dmatcfg->mem_addr, length, NULL);
        hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_TRGD);
        break;
    default:
        HAL_DEBUGW("parameter [dmareq] value is undefine");
        return HAL_ERR_VAL;
    }
    /* configure the TIMER DMA transfer */
    hals_timer_dma_transfer_config(timer_dev->periph, dmatcfg->start_addr, dmatcfg->length);
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER DMA transfer mode for read data from TIMER
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  dmareq: specify which DMA request
                one or more parameters can be selected which are shown as below:
      \arg        TIMER_DMA_UPD: update DMA, TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_DMA_CH0D: channel 0 DMA request, TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH1D: channel 1 DMA request, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH2D: channel 2 DMA request, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH3D: channel 3 DMA request, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CMTD: commutation DMA request , TIMERx(x=0,14)
      \arg        TIMER_DMA_TRGD: trigger DMA request, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_dma_transfer_read_stop(hal_timer_dev_struct *timer_dev, uint32_t dmareq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* lock TIMER */
    HAL_LOCK(timer_dev);
    switch(dmareq) {
    case TIMER_DMA_UPD:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_UP]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_UPD);
        break;
    case TIMER_DMA_CH0D:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
        break;
    case TIMER_DMA_CH1D:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
        break;
    case TIMER_DMA_CH2D:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH2D);
        break;
    case TIMER_DMA_CH3D:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CH3D);
        break;
    case TIMER_DMA_CMTD:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_CMT]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_CMTD);
        break;
    case TIMER_DMA_TRGD:
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->p_dma_timer[TIMER_DMA_ID_TRG]);
        hals_timer_dma_disable(timer_dev->periph, TIMER_DMA_TRGD);
        break;
    default:
        HAL_DEBUGW("parameter [dmareq] value is undefine");
        return HAL_ERR_VAL;
    }
    /* unlock TIMER */
    HAL_UNLOCK(timer_dev);
    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER commutation event
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  trigger_source:
       \arg        TIMER_TRIGGER_SOURCE_ITI0: trigger input source selection:ITI0
       \arg        TIMER_TRIGGER_SOURCE_ITI1: trigger input source selection:ITI1
       \arg        TIMER_TRIGGER_SOURCE_ITI2: trigger input source selection:ITI2
       \arg        TIMER_TRIGGER_SOURCE_ITI3: trigger input source selection:ITI3
       \arg        TIMER_TRIGGER_SOURCE_DISABLE: trigger input source selection:none
    \param[in]  com_source:
                only one parameter can be selected which is shown as below:
       \arg        TIMER_UPDATECTL_CCU: the shadow registers are updated when CMTG bit is set
       \arg        TIMER_UPDATECTL_CCUTRI: the shadow registers are updated when CMTG bit is set or an rising edge of TRGI occurs
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_commutation_event_config(hal_timer_dev_struct *timer_dev, uint32_t trigger_source, uint32_t com_source)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* trigger source config */
    if((TIMER_TRIGGER_SOURCE_ITI0 == trigger_source) || (TIMER_TRIGGER_SOURCE_ITI1 == trigger_source)
            || (TIMER_TRIGGER_SOURCE_ITI2 == trigger_source) || (TIMER_TRIGGER_SOURCE_ITI3 == trigger_source)) {
        TIMER_SMCFG(timer_dev->periph) &= ~TIMER_SMCFG_TRGS;
        TIMER_SMCFG(timer_dev->periph) |= trigger_source;
    }
    /* enable channel capture/compare control shadow register */
    hals_timer_channel_control_shadow_config(timer_dev->periph, ENABLE);
    /* configure TIMER channel control shadow register update control */
    hals_timer_channel_control_shadow_update_config(timer_dev->periph, com_source);
    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER commutation event and enable CMT interrupt
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  trigger_source:
       \arg        TIMER_TRIGGER_SOURCE_ITI0: trigger input source selection:ITI0
       \arg        TIMER_TRIGGER_SOURCE_ITI1: trigger input source selection:ITI1
       \arg        TIMER_TRIGGER_SOURCE_ITI2: trigger input source selection:ITI2
       \arg        TIMER_TRIGGER_SOURCE_ITI3: trigger input source selection:ITI3
       \arg        TIMER_TRIGGER_SOURCE_DISABLE: trigger input source selection:none
    \param[in]  com_source:
                only one parameter can be selected which is shown as below:
       \arg        TIMER_UPDATECTL_CCU: the shadow registers are updated when CMTG bit is set
       \arg        TIMER_UPDATECTL_CCUTRI: the shadow registers are updated when CMTG bit is set or an rising edge of TRGI occurs
    \param[in]  p_irq: TIMER interrupt user callback function pointer structure
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_commutation_event_interrupt_config(hal_timer_dev_struct *timer_dev, uint32_t trigger_source, uint32_t com_source, hal_timer_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* TIMER commutation interrupt handler set */
    timer_dev->timer_irq.commutation_handle = p_irq->commutation_handle;

    /* trigger source config */
    if((TIMER_TRIGGER_SOURCE_ITI0 == trigger_source) || (TIMER_TRIGGER_SOURCE_ITI1 == trigger_source)
            || (TIMER_TRIGGER_SOURCE_ITI2 == trigger_source) || (TIMER_TRIGGER_SOURCE_ITI3 == trigger_source)) {
        TIMER_SMCFG(timer_dev->periph) &= ~TIMER_SMCFG_TRGS;
        TIMER_SMCFG(timer_dev->periph) |= trigger_source;
    }
    /* enable channel capture/compare control shadow register */
    hals_timer_channel_control_shadow_config(timer_dev->periph, ENABLE);
    /* configure TIMER channel control shadow register update control */
    hals_timer_channel_control_shadow_update_config(timer_dev->periph, com_source);

    /* enable the TIMER interrupt */
    hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CMT);

    return HAL_ERR_NONE;
}

/*!
    \brief      configure TIMER commutation event and enable CMT DMA request
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  trigger_source:
       \arg        TIMER_TRIGGER_SOURCE_ITI0: trigger input source selection:ITI0
       \arg        TIMER_TRIGGER_SOURCE_ITI1: trigger input source selection:ITI1
       \arg        TIMER_TRIGGER_SOURCE_ITI2: trigger input source selection:ITI2
       \arg        TIMER_TRIGGER_SOURCE_ITI3: trigger input source selection:ITI3
       \arg        TIMER_TRIGGER_SOURCE_DISABLE: trigger input source selection:none
    \param[in]  com_source:
                only one parameter can be selected which is shown as below:
       \arg        TIMER_UPDATECTL_CCU: the shadow registers are updated when CMTG bit is set
       \arg        TIMER_UPDATECTL_CCUTRI: the shadow registers are updated when CMTG bit is set or an rising edge of TRGI occurs
    \param[in]  dmacb: TIMER DMA callback function pointer structure
                  update_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER update DMA request
                  channelx_capture_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel input capture DMA request
                  channelx_compare_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER channel output compare DMA request
                  commutation_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER commutation DMA request
                  trigger_dma_full_transcom_handle: TIMER DMA transfer complete interrupt handler for TIMER trigger DMA request
                  error_handle: TIMER DMA transfer error interrupt handler
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_commutation_event_dma_config(hal_timer_dev_struct *timer_dev, uint32_t trigger_source, uint32_t com_source, hal_timer_dma_handle_cb_struct *dmacb)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* trigger source config */
    if((TIMER_TRIGGER_SOURCE_ITI0 == trigger_source) || (TIMER_TRIGGER_SOURCE_ITI1 == trigger_source)
            || (TIMER_TRIGGER_SOURCE_ITI2 == trigger_source) || (TIMER_TRIGGER_SOURCE_ITI3 == trigger_source)) {
        TIMER_SMCFG(timer_dev->periph) &= ~TIMER_SMCFG_TRGS;
        TIMER_SMCFG(timer_dev->periph) |= trigger_source;
    }
    /* enable channel capture/compare control shadow register */
    hals_timer_channel_control_shadow_config(timer_dev->periph, ENABLE);
    /* configure TIMER channel control shadow register update control */
    hals_timer_channel_control_shadow_update_config(timer_dev->periph, com_source);

    /* DMA config for CMT DMA request */
    timer_dev->timer_dma.commutation_dma_full_transcom_handle = dmacb->commutation_dma_full_transcom_handle;
    timer_dev->timer_dma.error_handle = dmacb->error_handle;

    timer_dev->p_dma_timer[TIMER_DMA_ID_CMT]->dma_irq.full_finish_handle = _commutation_dma_full_transfer_complete;
    timer_dev->p_dma_timer[TIMER_DMA_ID_CMT]->dma_irq.half_finish_handle = NULL;
    timer_dev->p_dma_timer[TIMER_DMA_ID_CMT]->dma_irq.error_handle = _timer_dma_error;

    /* enable the TIMER DMA update request */
    hals_timer_dma_enable(timer_dev->periph, TIMER_DMA_CMTD);

    return HAL_ERR_NONE;
}

/*!
    \brief      set user-defined interrupt callback function
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to TIMER interrupt callback functions structure
                  please refer to hal_timer_irq_struct
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_irq_handle_set(hal_timer_dev_struct *timer_dev, hal_timer_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(NULL == p_irq) {
        HAL_DEBUGE("pointer [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* TIMER update interrupt call */
    if(NULL != p_irq->update_handle) {
        timer_dev->timer_irq.update_handle = p_irq->update_handle;
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_UP);
    } else {
        timer_dev->timer_irq.update_handle = NULL;
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_UP);
    }

    /* channel interrupt for input capture */
    if(NULL != p_irq->channelx_capture_handle) {
        timer_dev->timer_irq.channelx_capture_handle = p_irq->channelx_capture_handle;
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH2);
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH3);
    } else {
        timer_dev->timer_irq.channelx_capture_handle = NULL;
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH2);
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH3);
    }

    /* channel interrupt for compare output */
    if(NULL != p_irq->channelx_compare_handle) {
        timer_dev->timer_irq.channelx_compare_handle = p_irq->channelx_compare_handle;
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH2);
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH3);
    } else {
        timer_dev->timer_irq.channelx_compare_handle = NULL;
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH2);
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH3);
    }

    /* TIMER commutation interrupt call */
    if(NULL != p_irq->commutation_handle) {
        timer_dev->timer_irq.commutation_handle = p_irq->commutation_handle;
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_CMT);
    } else {
        timer_dev->timer_irq.commutation_handle = NULL;
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_CMT);
    }

    /* TIMER trigger interrupt call */
    if(NULL != p_irq->trigger_handle) {
        timer_dev->timer_irq.trigger_handle = p_irq->trigger_handle;
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_TRG);
    } else {
        timer_dev->timer_irq.trigger_handle = NULL;
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_TRG);
    }

    /* TIMER break interrupt handler set */
    if(NULL != p_irq->break_handle) {
        timer_dev->timer_irq.break_handle = p_irq->break_handle;
        /* enable the TIMER interrupt */
        hals_timer_interrupt_enable(timer_dev->periph, TIMER_INT_BRK);
    } else {
        timer_dev->timer_irq.break_handle = NULL;
        /* disable the TIMER interrupt */
        hals_timer_interrupt_disable(timer_dev->periph, TIMER_INT_BRK);
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      reset all user-defined interrupt callback function
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32f3x0_hal.h
*/
int32_t hal_timer_irq_handle_all_reset(hal_timer_dev_struct *timer_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* TIMER interrupt handler reset */
    timer_dev->timer_irq.update_handle = NULL;
    timer_dev->timer_irq.channelx_capture_handle = NULL;
    timer_dev->timer_irq.channelx_compare_handle = NULL;
    timer_dev->timer_irq.commutation_handle = NULL;
    timer_dev->timer_irq.trigger_handle = NULL;
    timer_dev->timer_irq.break_handle = NULL;

    return HAL_ERR_NONE;
}

/*!
    \brief      TIMER interrupt handler content function,which is merely used in timer_handler
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_timer_irq(hal_timer_dev_struct *timer_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev) {
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* check whether the update interrupt is set or not */
    if(SET == (hals_timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_UP))) {
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_UP);
        /* update interrupt handle */
        if(NULL != (timer_dev->timer_irq.update_handle)) {
            timer_dev->timer_irq.update_handle(timer_dev);
        }
    }

    /* check whether the channel 0 capture/compare interrupt is set or not */
    if(SET == (hals_timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_CH0))) {
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_0;
        /* channel 0 capture interrupt handle */
        if(0 != (TIMER_CHCTL0(timer_dev->periph) & TIMER_CHCTL0_CH0MS)) {
            if(NULL != (timer_dev->timer_irq.channelx_capture_handle)) {
                timer_dev->timer_irq.channelx_capture_handle(timer_dev);
            }
        } else {
            /* channel 0 compare interrupt handle */
            if(NULL != (timer_dev->timer_irq.channelx_compare_handle)) {
                timer_dev->timer_irq.channelx_compare_handle(timer_dev);
            }
        }
        /* clear service channel */
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_NONE;
    }
    /* check whether the channel 1 capture/compare interrupt is set or not */
    if(SET == (hals_timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_CH1))) {
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_1;
        /* channel 1 capture interrupt handle */
        if(0 != (TIMER_CHCTL0(timer_dev->periph) & TIMER_CHCTL0_CH1MS)) {
            if(NULL != (timer_dev->timer_irq.channelx_capture_handle)) {
                timer_dev->timer_irq.channelx_capture_handle(timer_dev);
            }
        } else {
            /* channel 1 compare interrupt handle */
            if(NULL != (timer_dev->timer_irq.channelx_compare_handle)) {
                timer_dev->timer_irq.channelx_compare_handle(timer_dev);
            }
        }
        /* clear service channel */
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_NONE;
    }

    /* check whether the channel 2 capture/compare interrupt is set or not */
    if(SET == (hals_timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_CH2))) {
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH2);
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_2;
        /* channel 2 capture interrupt handle */
        if(0 != (TIMER_CHCTL1(timer_dev->periph) & TIMER_CHCTL1_CH2MS)) {
            if(NULL != (timer_dev->timer_irq.channelx_capture_handle)) {
                timer_dev->timer_irq.channelx_capture_handle(timer_dev);
            }
        } else {
            /* channel 2 compare interrupt handle */
            if(NULL != (timer_dev->timer_irq.channelx_compare_handle)) {
                timer_dev->timer_irq.channelx_compare_handle(timer_dev);
            }
        }
        /* clear service channel */
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_NONE;
    }

    /* check whether the channel 3 capture/compare interrupt is set or not */
    if(SET == (hals_timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_CH3))) {
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH3);
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_3;
        /* channel 3 capture interrupt handle */
        if(0 != (TIMER_CHCTL1(timer_dev->periph) & TIMER_CHCTL1_CH3MS)) {
            if(NULL != (timer_dev->timer_irq.channelx_capture_handle)) {
                timer_dev->timer_irq.channelx_capture_handle(timer_dev);
            }
        } else {
            /* channel 3 compare interrupt handle */
            if(NULL != (timer_dev->timer_irq.channelx_compare_handle)) {
                timer_dev->timer_irq.channelx_compare_handle(timer_dev);
            }
        }
        /* clear service channel */
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_NONE;
    }
    /* check whether the commutation interrupt is set or not */
    if(SET == (hals_timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_CMT))) {
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CMT);
        /* commutation interrupt handle */
        if(NULL != (timer_dev->timer_irq.commutation_handle)) {
            timer_dev->timer_irq.commutation_handle(timer_dev);
        }
    }
    /* check whether the trigger interrupt is set or not */
    if(SET == (hals_timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_TRG))) {
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_TRG);
        /* trigger interrupt handle */
        if(NULL != (timer_dev->timer_irq.trigger_handle)) {
            timer_dev->timer_irq.trigger_handle(timer_dev);
        }
    }
    /* check whether the break interrupt is set or not */
    if(SET == (hals_timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_BRK))) {
        hals_timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_BRK);
        /* break interrupt handle */
        if(NULL != (timer_dev->timer_irq.break_handle)) {
            timer_dev->timer_irq.break_handle(timer_dev);
        }
    }
}

/*!
    \brief      enable a TIMER
    \param[in]  timer_periph: TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_enable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) |= (uint32_t)TIMER_CTL0_CEN;
}

/*!
    \brief      disable a TIMER
    \param[in]  timer_periph: TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_disable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_CEN;
}

/*!
    \brief      enable the update event
    \param[in]  timer_periph: TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_update_event_enable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_UPDIS;
}

/*!
    \brief      disable the update event
    \param[in]  timer_periph: TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_update_event_disable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) |= (uint32_t) TIMER_CTL0_UPDIS;
}

/*!
    \brief      configure TIMER update source
    \param[in]  timer_periph: TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[in]  update:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_UPDATE_SRC_GLOBAL: update generate by setting of UPG bit or the counter overflow/underflow,or the slave mode controller trigger
      \arg        TIMER_UPDATE_SRC_REGULAR: update generate only by counter overflow/underflow
    \param[out] none
    \retval     none
*/
void hals_timer_update_source_config(uint32_t timer_periph, uint8_t update)
{
    if(TIMER_UPDATE_SRC_REGULAR == update) {
        TIMER_CTL0(timer_periph) |= (uint32_t)TIMER_CTL0_UPS;
    } else if(TIMER_UPDATE_SRC_GLOBAL == update) {
        TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_UPS;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      set TIMER counter up direction
    \param[in]  timer_periph: TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_counter_up_direction(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_DIR;
}

/*!
    \brief      set TIMER counter down direction
    \param[in]  timer_periph: TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_counter_down_direction(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) |= (uint32_t)TIMER_CTL0_DIR;
}

/*!
    \brief      set TIMER counter alignment mode
    \param[in]  timer_periph: TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
    \param[in]  aligned:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_COUNTER_EDGE: edge-aligned mode
      \arg        TIMER_COUNTER_CENTER_DOWN: center-aligned and counting down assert mode
      \arg        TIMER_COUNTER_CENTER_UP: center-aligned and counting up assert mode
      \arg        TIMER_COUNTER_CENTER_BOTH: center-aligned and counting up/down assert mode
    \param[out] none
    \retval     none
*/
void hals_timer_counter_alignment(uint32_t timer_periph, uint16_t aligned)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_CAM;
    TIMER_CTL0(timer_periph) |= (uint32_t)aligned;
}

/*!
    \brief      enable the auto reload shadow function
    \param[in]  timer_periph: TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_auto_reload_shadow_enable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) |= (uint32_t)TIMER_CTL0_ARSE;
}

/*!
    \brief      disable the auto reload shadow function
    \param[in]  timer_periph: TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_auto_reload_shadow_disable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_ARSE;
}

/*!
    \brief      configure TIMER counter register value
    \param[in]  timer_periph: TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[in]  counter: the counter value
    \param[out] none
    \retval     none
*/
void hals_timer_counter_value_config(uint32_t timer_periph, uint32_t counter)
{
    TIMER_CNT(timer_periph) = (uint32_t)counter;
}

/*!
    \brief      read TIMER counter value
    \param[in]  timer_periph: TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[out] none
    \retval     counter value
*/
uint32_t hals_timer_counter_read(uint32_t timer_periph)
{
    uint32_t count_value = 0U;
    count_value = TIMER_CNT(timer_periph);
    return (count_value);
}

/*!
    \brief      configure TIMER prescaler
    \param[in]  timer_periph: TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[in]  prescaler: prescaler value
    \param[in]  pscreload: prescaler reload mode
                only one parameter can be selected which is shown as below:
      \arg        TIMER_PSC_RELOAD_NOW: the prescaler is loaded right now
      \arg        TIMER_PSC_RELOAD_UPDATE: the prescaler is loaded at the next update event
    \param[out] none
    \retval     none
*/
void hals_timer_prescaler_config(uint32_t timer_periph, uint16_t prescaler, uint8_t pscreload)
{
    TIMER_PSC(timer_periph) = (uint32_t)prescaler;

    if(TIMER_PSC_RELOAD_NOW == pscreload) {
        TIMER_SWEVG(timer_periph) |= (uint32_t)TIMER_SWEVG_UPG;
    }
}

/*!
    \brief      read TIMER prescaler value
    \param[in]  timer_periph: TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[out] none
    \retval     prescaler register value
*/
uint16_t hals_timer_prescaler_read(uint32_t timer_periph)
{
    uint16_t prescaler_value = 0U;
    prescaler_value = (uint16_t)(TIMER_PSC(timer_periph));
    return (prescaler_value);
}

/*!
    \brief      configure TIMER autoreload register value
    \param[in]  timer_periph: TIMERx(x=0,2,13..16),TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[in]  autoreload: the counter auto-reload value
    \param[out] none
    \retval     none
*/
void hals_timer_autoreload_value_config(uint32_t timer_periph, uint32_t autoreload)
{
    TIMER_CAR(timer_periph) = (uint32_t)autoreload;
}

/*!
    \brief      configure TIMER repetition register value
    \param[in]  timer_periph: TIMERx(x=0,14..16)
    \param[in]  repetition: the counter repetition value,0~255
    \param[out] none
    \retval     none
*/
void hals_timer_repetition_value_config(uint32_t timer_periph, uint16_t repetition)
{
    TIMER_CREP(timer_periph) = (uint32_t)repetition;
}

/*!
    \brief      enable or disable channel capture/compare control shadow register
    \param[in]  timer_periph: TIMERx(x=0,14..16)
    \param[in]  newvalue: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void hals_timer_channel_control_shadow_config(uint32_t timer_periph, ControlStatus newvalue)
{
    if(ENABLE == newvalue) {
        TIMER_CTL1(timer_periph) |= (uint32_t)TIMER_CTL1_CCSE;
    } else {
        TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_CCSE);
    }
}

/*!
    \brief      configure TIMER channel control shadow register update control
    \param[in]  timer_periph: TIMERx(x=0,14..16)
    \param[in]  ccuctl: channel control shadow register update control
                only one parameter can be selected which is shown as below:
      \arg        TIMER_UPDATECTL_CCU: the shadow registers update by when CMTG bit is set
      \arg        TIMER_UPDATECTL_CCUTRI: the shadow registers update by when CMTG bit is set or an rising edge of TRGI occurs
    \param[out] none
    \retval     none
*/
void hals_timer_channel_control_shadow_update_config(uint32_t timer_periph, uint8_t ccuctl)
{
    if(TIMER_UPDATECTL_CCU == ccuctl) {
        TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_CCUC);
    } else if(TIMER_UPDATECTL_CCUTRI == ccuctl) {
        TIMER_CTL1(timer_periph) |= (uint32_t)TIMER_CTL1_CCUC;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      select TIMER master mode output trigger source
    \param[in]  timer_periph: TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
    \param[in]  outrigger: the argument could be selected from enumeration <hal_timer_trgo_selection_enum>
                only one parameter can be selected which is shown as below:
      \arg        TIMER_TRI_OUT_SRC_RESET: the UPG bit as trigger output(TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350)
      \arg        TIMER_TRI_OUT_SRC_ENABLE: the counter enable signal TIMER_CTL0_CEN as trigger output(TIMERx(x=0,2,14),, TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350)
      \arg        TIMER_TRI_OUT_SRC_UPDATE: update event as trigger output(TIMERx(x=0,2,14),, TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350)
      \arg        TIMER_TRI_OUT_SRC_CH0: a capture or a compare match occurred in channal0 as trigger output TRGO(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_TRI_OUT_SRC_O0CPRE: O0CPRE as trigger output(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_TRI_OUT_SRC_O1CPRE: O1CPRE as trigger output(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_TRI_OUT_SRC_O2CPRE: O2CPRE as trigger output(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_TRI_OUT_SRC_O3CPRE: O3CPRE as trigger output(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_master_output_trigger_source_select(uint32_t timer_periph, hal_timer_trgo_selection_enum outrigger)
{
    TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_MMC);
    TIMER_CTL1(timer_periph) |= (uint32_t)outrigger;
}

/*!
    \brief      configure TIMER master slave mode
    \param[in]  timer_periph: TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
    \param[in]  master_slave:
                only one parameter can be selected which is shown as below:
      \arg        ENABLE: master slave mode enable
      \arg        DISABLE: master slave mode disable
    \param[out] none
    \retval     none
*/
void hals_timer_master_slave_mode_config(uint32_t timer_periph, uint32_t master_slave)
{
    if(ENABLE == master_slave) {
        TIMER_SMCFG(timer_periph) |= (uint32_t)TIMER_SMCFG_MSM;
    } else if(DISABLE == master_slave) {
        TIMER_SMCFG(timer_periph) &= ~(uint32_t)TIMER_SMCFG_MSM;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      select TIMER input trigger source
    \param[in]  timer_periph: TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
    \param[in]  intrigger:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_SMCFG_TRGSEL_ITI0: internal trigger 0(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_SMCFG_TRGSEL_ITI1: internal trigger 1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_SMCFG_TRGSEL_ITI2: internal trigger 2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_SMCFG_TRGSEL_CI0F_ED: TI0 edge detector(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_SMCFG_TRGSEL_CI0FE0: filtered TIMER input 0(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_SMCFG_TRGSEL_CI1FE1: filtered TIMER input 1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_SMCFG_TRGSEL_ETIFP: external trigger(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_input_trigger_source_select(uint32_t timer_periph, uint32_t intrigger)
{
    TIMER_SMCFG(timer_periph) &= (~(uint32_t)TIMER_SMCFG_TRGS);
    TIMER_SMCFG(timer_periph) |= (uint32_t)intrigger;
}

/*!
    \brief      select TIMER slave mode
    \param[in]  timer_periph: TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
    \param[in]  slavemode:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_SLAVE_MODE_DISABLE: slave mode disable(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_QUAD_DECODER_MODE0: encoder mode 0(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_QUAD_DECODER_MODE1: encoder mode 1(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_QUAD_DECODER_MODE2: encoder mode 2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_SLAVE_MODE_RESTART: restart mode(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_SLAVE_MODE_PAUSE: pause mode(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_SLAVE_MODE_EVENT: event mode(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_SLAVE_MODE_EXTERNAL0: external clock mode 0(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_slave_mode_select(uint32_t timer_periph, uint32_t slavemode)
{
    TIMER_SMCFG(timer_periph) &= (~(uint32_t)TIMER_SMCFG_SMC);
    TIMER_SMCFG(timer_periph) |= (uint32_t)slavemode;
}

/*!
    \brief      configure TIMER external trigger input
    \param[in]  timer_periph: TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
    \param[in]  extprescaler:
                only one parameter can be selected which is shown as below:
      \arg          TIMER_EXT_TRI_PRESCALER_OFF : no divided
      \arg          TIMER_EXT_TRI_PRESCALER_DIV2 : divided by 2
      \arg          TIMER_EXT_TRI_PRESCALER_DIV4 : divided by 4
      \arg          TIMER_EXT_TRI_PRESCALER_DIV8 : divided by 8
    \param[in]  extpolarity:
                only one parameter can be selected which is shown as below:
      \arg          TIMER_EXT_TRI_POLARITY_RISING : active high or rising edge active
      \arg          TIMER_EXT_TRI_POLARITY_FALLING : active low or falling edge active
    \param[in]  extfilter: a value between 0 and 15
    \param[out] none
    \retval     none
*/
void hals_timer_external_trigger_config(uint32_t timer_periph, uint32_t extprescaler, uint32_t extpolarity, uint32_t extfilter)
{
    TIMER_SMCFG(timer_periph) &= (~(uint32_t)(TIMER_SMCFG_ETP | TIMER_SMCFG_ETPSC | TIMER_SMCFG_ETFC | TIMER_SMCFG_SMC1));
    TIMER_SMCFG(timer_periph) |= (uint32_t)(extprescaler | extpolarity);
    TIMER_SMCFG(timer_periph) |= (uint32_t)(extfilter << 8U);
}

/*!
    \brief      get TIMER flags
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  flag: the timer interrupt flags
                only one parameter can be selected which is shown as below:
      \arg        TIMER_FLAG_UP: update flag, TIMERx(x=0,2,5,13..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_FLAG_CH0: channel 0 flag, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350,
      \arg        TIMER_FLAG_CH1: channel 1 flag, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350,
      \arg        TIMER_FLAG_CH2: channel 2 flag, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350,
      \arg        TIMER_FLAG_CH3: channel 3 flag, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350,
      \arg        TIMER_FLAG_CMT: channel commutation flag, TIMERx(x=0,14..16)
      \arg        TIMER_FLAG_TRG: trigger flag, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350,
      \arg        TIMER_FLAG_BRK: break flag,TIMERx(x=0,14..16)
      \arg        TIMER_FLAG_CH0O: channel 0 overcapture flag, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350,
      \arg        TIMER_FLAG_CH1O: channel 1 overcapture flag, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350,
      \arg        TIMER_FLAG_CH2O: channel 2 overcapture flag, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350,
      \arg        TIMER_FLAG_CH3O: channel 3 overcapture flag, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350,
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_timer_flag_get(uint32_t timer_periph, uint32_t flag)
{
    if(RESET != (TIMER_INTF(timer_periph) & flag)) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear TIMER flags
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  flag: the timer interrupt flags
                only one parameter can be selected which is shown as below:
      \arg        TIMER_FLAG_UP: update flag, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_FLAG_CH0: channel 0 flag, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_FLAG_CH1: channel 1 flag, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_FLAG_CH2: channel 2 flag, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_FLAG_CH3: channel 3 flag, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_FLAG_CMT: channel commutation flag, TIMERx(x=0,14..16)
      \arg        TIMER_FLAG_TRG: trigger flag, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_FLAG_BRK: break flag,TIMERx(x=0,14..16)
      \arg        TIMER_FLAG_CH0O: channel 0 overcapture flag, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_FLAG_CH1O: channel 1 overcapture flag, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_FLAG_CH2O: channel 2 overcapture flag, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_FLAG_CH3O: channel 3 overcapture flag, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_flag_clear(uint32_t timer_periph, uint32_t flag)
{
    TIMER_INTF(timer_periph) = (~(uint32_t)flag);
}

/*!
    \brief      enable the TIMER interrupt
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  interrupt: timer interrupt enable source
                only one parameter can be selected which is shown as below:
      \arg        TIMER_INT_UP: update interrupt disable, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_INT_CH0: channel 0 interrupt disable, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_CH1: channel 1 interrupt disable, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_CH2: channel 2 interrupt disable, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_CH3: channel 3 interrupt disable , TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_CMT: commutation interrupt disable, TIMERx(x=0,14..16)
      \arg        TIMER_INT_TRG: trigger interrupt disable, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_BRK: break interrupt disable, TIMERx(x=0,14..16)
    \param[out] none
    \retval     none
*/
void hals_timer_interrupt_enable(uint32_t timer_periph, uint32_t interrupt)
{
    TIMER_DMAINTEN(timer_periph) |= (uint32_t) interrupt;
}

/*!
    \brief      disable the TIMER interrupt
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  interrupt: timer interrupt source disable
                only one parameter can be selected which is shown as below:
      \arg        TIMER_INT_UP: update interrupt disable, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_INT_CH0: channel 0 interrupt disable, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_CH1: channel 1 interrupt disable, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_CH2: channel 2 interrupt disable, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_CH3: channel 3 interrupt disable , TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_CMT: commutation interrupt disable, TIMERx(x=0,14..16)
      \arg        TIMER_INT_TRG: trigger interrupt disable, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_BRK: break interrupt disable, TIMERx(x=0,14..16)
    \param[out] none
    \retval     none
*/
void hals_timer_interrupt_disable(uint32_t timer_periph, uint32_t interrupt)
{
    TIMER_DMAINTEN(timer_periph) &= (~(uint32_t)interrupt);
}

/*!
    \brief      get timer interrupt flag
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  interrupt: the timer interrupt bits
                only one parameter can be selected which is shown as below:
      \arg        TIMER_INT_FLAG_UP: update interrupt flag, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_INT_FLAG_CH0: channel 0 interrupt flag, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_FLAG_CH1: channel 1 interrupt flag, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_FLAG_CH2: channel 2 interrupt flag, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_FLAG_CH3: channel 3 interrupt flag, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_FLAG_CMT: channel commutation interrupt flag, TIMERx(x=0,14..16)
      \arg        TIMER_INT_FLAG_TRG: trigger interrupt flag, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_FLAG_BRK:  break interrupt flag, TIMERx(x=0,14..16)
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_timer_interrupt_flag_get(uint32_t timer_periph, uint32_t interrupt)
{
    uint32_t val;
    val = (TIMER_DMAINTEN(timer_periph) & interrupt);
    if((RESET != (TIMER_INTF(timer_periph) & interrupt)) && (RESET != val)) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear TIMER interrupt flag
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  interrupt: the timer interrupt bits
                only one parameter can be selected which is shown as below:
      \arg        TIMER_INT_FLAG_UP: update interrupt flag, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_INT_FLAG_CH0: channel 0 interrupt flag, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_FLAG_CH1: channel 1 interrupt flag, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_FLAG_CH2: channel 2 interrupt flag, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_FLAG_CH3: channel 3 interrupt flag, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_FLAG_CMT: channel commutation interrupt flag, TIMERx(x=0,14..16)
      \arg        TIMER_INT_FLAG_TRG: trigger interrupt flag, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_INT_FLAG_BRK:  break interrupt flag, TIMERx(x=0,14..16)
    \param[out] none
    \retval     none
*/
void hals_timer_interrupt_flag_clear(uint32_t timer_periph, uint32_t interrupt)
{
    TIMER_INTF(timer_periph) = (~(uint32_t)interrupt);
}

/*!
    \brief      enable the TIMER DMA
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  dma: specify which DMA to enable
                one or more parameters can be selected which is shown as below:
      \arg        TIMER_DMA_UPD: update DMA, TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_DMA_CH0D: channel 0 DMA request, TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH1D: channel 1 DMA request, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH2D: channel 2 DMA request, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH3D: channel 3 DMA request, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CMTD: commutation DMA request , TIMERx(x=0,14)
      \arg        TIMER_DMA_TRGD: trigger DMA request, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_dma_enable(uint32_t timer_periph, uint16_t dma)
{
    TIMER_DMAINTEN(timer_periph) |= (uint32_t) dma;
}

/*!
    \brief      disable the TIMER DMA
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  dma: specify which DMA to disable
                one or more parameters can be selected which are shown as below:
      \arg        TIMER_DMA_UPD: update DMA, TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_DMA_CH0D: channel 0 DMA request, TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH1D: channel 1 DMA request, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH2D: channel 2 DMA request, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CH3D: channel 3 DMA request, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_DMA_CMTD: commutation DMA request , TIMERx(x=0,14)
      \arg        TIMER_DMA_TRGD: trigger DMA request, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     none
*/
void hals_timer_dma_disable(uint32_t timer_periph, uint16_t dma)
{
    TIMER_DMAINTEN(timer_periph) &= (~(uint32_t)(dma));
}

/*!
    \brief      channel DMA request source selection
    \param[in]  timer_periph: TIMERx(x=0,2,14..16), TIMER1 just for GD32F330 and GD32F350
    \param[in]  dma_request: channel DMA request source selection
                only one parameter can be selected which is shown as below:
       \arg        TIMER_DMAREQUEST_CHANNELEVENT: DMA request of channel y is sent when channel y event occurs
       \arg        TIMER_DMAREQUEST_UPDATEEVENT: DMA request of channel y is sent when update event occurs
    \param[out] none
    \retval     none
*/
void hals_timer_channel_dma_request_source_select(uint32_t timer_periph, uint8_t dma_request)
{
    if(TIMER_DMAREQUEST_UPDATEEVENT == dma_request) {
        TIMER_CTL1(timer_periph) |= (uint32_t)TIMER_CTL1_DMAS;
    } else if(TIMER_DMAREQUEST_CHANNELEVENT == dma_request) {
        TIMER_CTL1(timer_periph) &= ~(uint32_t)TIMER_CTL1_DMAS;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      configure the TIMER DMA transfer
    \param[in]  timer_periph: TIMERx(x=0..2,14..16)
    \param[in]  dma_startaddr: the argument could be selected from enumeration <hal_timer_dma_transfer_start_address_enum>
    \param[in]  dma_lenth: the argument could be selected from enumeration <hal_timer_dma_transfer_length_enum>
    \param[out] none
    \retval     none
*/
void hals_timer_dma_transfer_config(uint32_t timer_periph, hal_timer_dma_transfer_start_address_enum dma_baseaddr, hal_timer_dma_transfer_length_enum dma_lenth)
{
    TIMER_DMACFG(timer_periph) &= (~(uint32_t)(TIMER_DMACFG_DMATA | TIMER_DMACFG_DMATC));
    TIMER_DMACFG(timer_periph) |= (uint32_t)(dma_baseaddr | dma_lenth);
}

/*!
    \brief      software generate events
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  event: the timer software event generation sources
                one or more parameters can be selected which are shown as below:
      \arg        TIMER_EVENT_SRC_UPG: update event,TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350, TIMER5 just for GD32F350
      \arg        TIMER_EVENT_SRC_CH0G: channel 0 capture or compare event generation, TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_EVENT_SRC_CH1G: channel 1 capture or compare event generation, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_EVENT_SRC_CH2G: channel 2 capture or compare event generation, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_EVENT_SRC_CH3G: channel 3 capture or compare event generation, TIMERx(x=0,2), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_EVENT_SRC_CMTG: channel commutation event generation, TIMERx(x=0,14..16)
      \arg        TIMER_EVENT_SRC_TRGG: trigger event generation, TIMERx(x=0,2,14), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_EVENT_SRC_BRKG: break event generation, TIMERx(x=0,14..16)
    \param[out] none
    \retval     none
*/
void hals_timer_event_software_generate(uint32_t timer_periph, uint16_t event)
{
    TIMER_SWEVG(timer_periph) |= (uint32_t)event;
}

/*!
    \brief      configure TIMER channel output compare
    \param[in]  timer_periph: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel 0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel 1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel 2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel 3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  timer_outputcompare: TIMER output compare configuration structure
                  compare_mode: the argument could be selected from enumeration <hal_timer_output_compare_enum>
                  oc_pulse_value:0~65535,(for TIMER1 0x00000000~0xFFFFFFFF)
                  oc_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_POLARITY_HIGH : channel output polarity is high
      \arg          TIMER_OC_POLARITY_LOW : channel output polarity is low
                  oc_idlestate:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_IDLE_STATE_LOW : idle state of channel output is high
      \arg          TIMER_OC_IDLE_STATE_HIGH : idle state of channel output is low
                  ocn_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OCN_POLARITY_HIGH : channel complementary output polarity is high
      \arg          TIMER_OCN_POLARITY_LOW : channel complementary output polarity is low
                  ocn_idlestate:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OCN_IDLE_STATE_LOW : idle state of channel complementary output is high
      \arg          TIMER_OCN_IDLE_STATE_HIGH :  idle state of channel complementary output is low
                  oc_shadow:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_SHADOW_ENABLE : channel output compare shadow enable
      \arg          TIMER_OC_SHADOW_DISABLE : channel output compare shadow disable
                  oc_fastmode:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_FAST_ENABLE : channel output fast function enable
      \arg          TIMER_OC_FAST_DISABLE : channel output fast function disable
                  oc_clearmode:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_OC_CLEAR_ENABLE : channel output clear function enable
      \arg          TIMER_OC_CLEAR_DISABLE : channel output clear function disable
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, details refer to gd32f3x0_hal.h
*/
void hals_timer_channel_output_config(uint32_t timer_periph, uint16_t channel, hal_timer_output_compare_struct *timer_outputcompare)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        /* reset the CH0EN CH0NEN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0EN | TIMER_CHCTL2_CH0NEN));
        /* reset the CH0MS bit */
        TIMER_CHCTL0(timer_periph) &= ~(uint32_t)TIMER_CHCTL0_CH0MS;
        /* configure TIMER_CH_0 complementary channel */
        /* reset the CH0NP bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0NP);
        /* set the CH0NP bit */
        TIMER_CHCTL2(timer_periph) |= (uint32_t)timer_outputcompare->ocn_polarity;
        /* configure TIMER_CH_0 channel idel state */
        if((TIMER0 == timer_periph) || (TIMER14 == timer_periph) || (TIMER15 == timer_periph) || (TIMER16 == timer_periph)) {
            /* reset the ISO0 bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO0);
            /* set the ISO0 bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)timer_outputcompare->oc_idlestate;
            /* reset the ISO0N bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO0N);
            /* set the ISO0N bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)timer_outputcompare->ocn_idlestate;
        }

        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        /* reset the CH1EN CH1NEN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1EN | TIMER_CHCTL2_CH1NEN));
        /* reset the CH1MS bit */
        TIMER_CHCTL0(timer_periph) &= ~(uint32_t)TIMER_CHCTL0_CH1MS;

#if (defined(GD32F350) || defined(GD32F330))
        /* configure TIMER_CH_1 complementary channel polarity */
        if((TIMER0 == timer_periph) || (TIMER1 == timer_periph) || (TIMER2 == timer_periph) || (TIMER14 == timer_periph)) {
#else
        /* configure TIMER_CH_1 complementary channel polarity */
        if((TIMER0 == timer_periph) || (TIMER2 == timer_periph) || (TIMER14 == timer_periph)) {
#endif
            /* reset the CH1NP bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1NP);
            /* set the CH1NP bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)(timer_outputcompare->ocn_polarity << 4U);
        }
        /* configure TIMER_CH_1 channel idel state */
        if((TIMER0 == timer_periph) || (TIMER14 == timer_periph)) {
            /* reset the ISO1 bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO1);
            /* set the ISO1 bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)(timer_outputcompare->oc_idlestate << 2U);
            /* reset the ISO1N bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO1N);
            /* set the ISO1N bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)(timer_outputcompare->ocn_idlestate << 2U);
        }
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        /* reset the CH2EN CH2NEN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH2EN | TIMER_CHCTL2_CH2NEN));
        /* reset the CH2MS bit */
        TIMER_CHCTL1(timer_periph) &= ~(uint32_t)TIMER_CHCTL1_CH2MS;

#if (defined(GD32F350) || defined(GD32F330))
        /* configure TIMER_CH_2 complementary channel */
        if((TIMER0 == timer_periph) || (TIMER1 == timer_periph) || (TIMER2 == timer_periph)) {
#else
        /* configure TIMER_CH_2 complementary channel */
        if((TIMER0 == timer_periph) || (TIMER2 == timer_periph)) {
#endif
            /* reset the CH2NP bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2NP);
            /* set the CH2NP bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)(timer_outputcompare->ocn_polarity << 8U);
        }
        /* configure TIMER_CH_2 channel idel state */
        if(TIMER0 == timer_periph) {
            /* reset the ISO2 bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO2);
            /* set the ISO2 bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)(timer_outputcompare->oc_idlestate << 4U);
            /* reset the ISO2N bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO2N);
            /* set the ISO2N bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)(timer_outputcompare->ocn_idlestate << 4U);
        }
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        /* reset the CH3EN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH3EN);
        /* reset the CH3MS bit */
        TIMER_CHCTL1(timer_periph) &= ~(uint32_t)TIMER_CHCTL1_CH3MS;

#if (defined(GD32F350) || defined(GD32F330))
        /* configure TIMER_CH_3 complementary channel */
        if((TIMER1 == timer_periph) || (TIMER2 == timer_periph)) {
#else
        /* configure TIMER_CH_3 complementary channel */
        if(TIMER2 == timer_periph) {
#endif
            /* reset the CH3NP bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH3NP);
            /* set the CH3NP bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)(timer_outputcompare->ocn_polarity << 12U);
        }
        /* configure TIMER_CH_3 channel idel state */
        if(TIMER0 == timer_periph) {
            /* reset the ISO3 bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO3);
            /* set the ISO3 bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)(timer_outputcompare->oc_idlestate << 6U);
        }
        break;
    default:
        break;
    }
    /* configure TIMER channel output compare mode */
    hals_timer_channel_output_mode_config(timer_periph, channel, timer_outputcompare->compare_mode);
    /* configure TIMER channel output pulse value */
    hals_timer_channel_output_pulse_value_config(timer_periph, channel, timer_outputcompare->oc_pulse_value);
    /* configure TIMER channel output polarity  */
    hals_timer_channel_output_polarity_config(timer_periph, channel, timer_outputcompare->oc_polarity);
    /* configure TIMER channel output shadow function */
    hals_timer_channel_output_shadow_config(timer_periph, channel, timer_outputcompare->oc_shadow);
    /* configure TIMER channel output fast mode */
    if((TIMER_OC_MODE_PWM0 == timer_outputcompare->compare_mode) || (TIMER_OC_MODE_PWM1 == timer_outputcompare->compare_mode)) {
        /* configure TIMER channel output fast function */
        hals_timer_channel_output_fast_config(timer_periph, channel, timer_outputcompare->oc_fastmode);
        /* configure TIMER channel output clear function */
        hals_timer_channel_output_clear_config(timer_periph, channel, timer_outputcompare->oc_clearmode);
    }
    /* configure TIMER channel output clear mode */
    if((TIMER_OC_MODE_ACTIVE == timer_outputcompare->compare_mode) || (TIMER_OC_MODE_INACTIVE == timer_outputcompare->compare_mode)
            || (TIMER_OC_MODE_TOGGLE == timer_outputcompare->compare_mode)) {
        /* configure TIMER channel output clear function */
        hals_timer_channel_output_clear_config(timer_periph, channel, timer_outputcompare->oc_clearmode);
    }
}

/*!
    \brief      configure TIMER channel output compare mode
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  ocmode: output compare mode, the argument could be selected from enumeration <hal_timer_output_compare_enum>
    \param[out] none
    \retval     none
*/
void hals_timer_channel_output_mode_config(uint32_t timer_periph, uint16_t channel, hal_timer_output_compare_enum ocmode)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH0COMCTL);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)ocmode;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH1COMCTL);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)((uint32_t)(ocmode) << 8U);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH2COMCTL);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)ocmode;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH3COMCTL);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)((uint32_t)(ocmode) << 8U);
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel output pulse value
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  pulse: channel output pulse value,0~65535, (for TIMER1 0x00000000~0xFFFFFFFF)
    \param[out] none
    \retval     none
*/
void hals_timer_channel_output_pulse_value_config(uint32_t timer_periph, uint16_t channel, uint32_t pulse)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CH0CV(timer_periph) = (uint32_t)pulse;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CH1CV(timer_periph) = (uint32_t)pulse;
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CH2CV(timer_periph) = (uint32_t)pulse;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        TIMER_CH3CV(timer_periph) = (uint32_t)pulse;
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel output shadow function
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  ocshadow: channel output shadow state
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_SHADOW_ENABLE: channel output shadow state enable
      \arg        TIMER_OC_SHADOW_DISABLE: channel output shadow state disable
    \param[out] none
    \retval     none
*/
void hals_timer_channel_output_shadow_config(uint32_t timer_periph, uint16_t channel, uint16_t ocshadow)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH0COMSEN);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)ocshadow;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH1COMSEN);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)((uint32_t)(ocshadow) << 8U);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH2COMSEN);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)ocshadow;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH3COMSEN);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)((uint32_t)(ocshadow) << 8U);
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel output fast function
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  ocfast: channel output fast function
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_FAST_ENABLE: channel output fast function enable
      \arg        TIMER_OC_FAST_DISABLE: channel output fast function disable
    \param[out] none
    \retval     none
*/
void hals_timer_channel_output_fast_config(uint32_t timer_periph, uint16_t channel, uint16_t ocfast)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH0COMFEN);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)ocfast;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH1COMFEN);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)((uint32_t)ocfast << 8U);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH2COMFEN);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)ocfast;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH3COMFEN);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)((uint32_t)ocfast << 8U);
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel output clear function
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  occlear: channel output clear function
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_CLEAR_ENABLE: channel output clear function enable
      \arg        TIMER_OC_CLEAR_DISABLE: channel output clear function disable
    \param[out] none
    \retval     none
*/
void hals_timer_channel_output_clear_config(uint32_t timer_periph, uint16_t channel, uint16_t occlear)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH0COMCEN);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)occlear;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH1COMCEN);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)((uint32_t)occlear << 8U);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH2COMCEN);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)occlear;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH3COMCEN);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)((uint32_t)occlear << 8U);
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel output polarity
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  ocpolarity: channel output polarity
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_POLARITY_HIGH: channel output polarity is high
      \arg        TIMER_OC_POLARITY_LOW: channel output polarity is low
    \param[out] none
    \retval     none
*/
void hals_timer_channel_output_polarity_config(uint32_t timer_periph, uint16_t channel, uint16_t ocpolarity)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0P);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)ocpolarity;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1P);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)ocpolarity << 4U);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2P);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)ocpolarity << 8U);
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH3P);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)ocpolarity << 12U);
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel complementary output polarity
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[in]  ocnpolarity: channel complementary output polarity
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OCN_POLARITY_HIGH: channel complementary output polarity is high
      \arg        TIMER_OCN_POLARITY_LOW: channel complementary output polarity is low
    \param[out] none
    \retval     none
*/
void hals_timer_channel_complementary_output_polarity_config(uint32_t timer_periph, uint16_t channel, uint16_t ocnpolarity)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0NP);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)ocnpolarity;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1NP);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)ocnpolarity << 4U);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2NP);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)ocnpolarity << 8U);
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH3NP);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)ocnpolarity << 12U);
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel enable state
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  state: TIMER channel enable state
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CCX_ENABLE: channel enable
      \arg        TIMER_CCX_DISABLE: channel disable
    \param[out] none
    \retval     none
*/
void hals_timer_channel_output_state_config(uint32_t timer_periph, uint16_t channel, uint32_t state)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)state;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)state << 4U);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2EN);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)state << 8U);
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH3EN);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)state << 12U);
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel complementary output enable state
    \param[in]  timer_periph: TIMERx(x=0,14..16)
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[in]  ocnstate: TIMER channel complementary output enable state
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CCXN_ENABLE: channel complementary enable
      \arg        TIMER_CCXN_DISABLE: channel complementary disable
    \param[out] none
    \retval     none
*/
void hals_timer_channel_complementary_output_state_config(uint32_t timer_periph, uint16_t channel, uint16_t ocnstate)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0NEN);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)ocnstate;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1NEN);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)ocnstate << 4U);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2NEN);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)ocnstate << 8U);
        break;
    default:
        break;
    }
}

/*!
    \brief      config TIMER input capture mode
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  timer_inputcapture: TIMER input capture configuration structure
                  ic_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_POLARITY_RISING : input capture rising edge
      \arg          TIMER_IC_POLARITY_FALLING : input capture falling edge
      \arg          TIMER_IC_POLARITY_BOTH_EDGE : input capture both edge
                  ic_selection:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_SELECTION_DIRECTTI : channel y is configured as input and icy is mapped on CIy
      \arg          TIMER_IC_SELECTION_INDIRECTTI : channel y is configured as input and icy is mapped on opposite input
      \arg          TIMER_IC_SELECTION_ITS : channel y is configured as input and icy is mapped on ITS
                  ic_prescaler:
                  only one parameter can be selected which is shown as below:
      \arg          TIMER_IC_PRESCALER_OFF : no prescaler
      \arg          TIMER_IC_PRESCALER_DIV2 : divided by 2
      \arg          TIMER_IC_PRESCALER_DIV4 : divided by 4
      \arg          TIMER_IC_PRESCALER_DIV8 : divided by 8
                  ic_filter: 0~15
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32f3x0_hal.h
*/
void hals_timer_channel_input_capture_config(uint32_t timer_periph, uint16_t channel, hal_timer_input_capture_struct *timer_inputcapture)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        /* reset the CH0EN CH0NEN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0EN | TIMER_CHCTL2_CH0NEN));
        /* reset the CH0P and CH0NP bits */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
        TIMER_CHCTL2(timer_periph) |= (uint32_t)(timer_inputcapture->ic_polarity);
        /* reset the CH0MS bit */
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH0MS);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)(timer_inputcapture->ic_selection);
        /* reset the CH0CAPPSC bit */
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPPSC);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)(timer_inputcapture->ic_prescaler);
        /* reset the CH0CAPFLT bit */
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)((uint32_t)(timer_inputcapture->ic_filter) << 4U);
        break;

    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        /* reset the CH1EN CH1NEN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1EN | TIMER_CHCTL2_CH1NEN));
        /* reset the CH1P and CH1NP bits */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(timer_inputcapture->ic_polarity) << 4U);
        /* reset the CH1MS bit */
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH1MS);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)((uint32_t)(timer_inputcapture->ic_selection) << 8U);
        /* reset the CH1CAPPSC bit */
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPPSC);
        TIMER_CHCTL0(timer_periph) |= ((uint32_t)(timer_inputcapture->ic_prescaler) << 8U);
        /* reset the CH1CAPFLT bit */
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)((uint32_t)(timer_inputcapture->ic_filter) << 12U);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        /* reset the CH2EN CH2NEN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH2EN | TIMER_CHCTL2_CH2NEN));
        /* reset the CH2P and CH2NP bits */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH2P | TIMER_CHCTL2_CH2NP));
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(timer_inputcapture->ic_polarity) << 8U);
        /* reset the CH2MS bit */
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH2MS);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)((uint32_t)(timer_inputcapture->ic_selection));
        /* reset the CH2CAPPSC bit */
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH2CAPPSC);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)(timer_inputcapture->ic_prescaler);
        /* reset the CH2CAPFLT bit */
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH2CAPFLT);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)((uint32_t)(timer_inputcapture->ic_filter) << 4U);
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        /* reset the CH3EN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH3EN);
        /* reset the CH3P and CH3NP bits */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)(TIMER_CHCTL2_CH3P | TIMER_CHCTL2_CH3NP));
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(timer_inputcapture->ic_polarity) << 12U);
        /* reset the CH3MS bit */
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH3MS);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)((uint32_t)(timer_inputcapture->ic_selection) << 8U);
        /* reset the CH3CAPPSC bit */
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH3CAPPSC);
        TIMER_CHCTL1(timer_periph) |= ((uint32_t)(timer_inputcapture->ic_prescaler) << 8U);
        /* reset the CH3CAPFLT bit */
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH3CAPFLT);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)((uint32_t)(timer_inputcapture->ic_filter) << 12U);
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel input capture prescaler value
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[in]  prescaler: channel input capture prescaler value
                only one parameter can be selected which is shown as below:
      \arg        TIMER_IC_PRESCALER_OFF: no prescaler
      \arg        TIMER_IC_PRESCALER_DIV2: divided by 2
      \arg        TIMER_IC_PRESCALER_DIV4: divided by 4
      \arg        TIMER_IC_PRESCALER_DIV8: divided by 8
    \param[out] none
    \retval     none
*/
void hals_timer_channel_input_capture_prescaler_config(uint32_t timer_periph, uint16_t channel, uint16_t prescaler)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPPSC);
        TIMER_CHCTL0(timer_periph) |= (uint32_t)prescaler;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CHCTL0(timer_periph) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPPSC);
        TIMER_CHCTL0(timer_periph) |= ((uint32_t)prescaler << 8U);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH2CAPPSC);
        TIMER_CHCTL1(timer_periph) |= (uint32_t)prescaler;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        TIMER_CHCTL1(timer_periph) &= (~(uint32_t)TIMER_CHCTL1_CH3CAPPSC);
        TIMER_CHCTL1(timer_periph) |= ((uint32_t)prescaler << 8U);
        break;
    default:
        break;
    }
}

/*!
    \brief      read TIMER channel capture/compare register value
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2)), TIMER1 just for GD32F330 and GD32F350
    \param[out] none
    \retval     channel capture compare register value
*/
uint32_t hals_timer_input_capture_register_read(uint32_t timer_periph, uint16_t channel)
{
    uint32_t ret_val;

    switch(channel) {
    /* read TIMER channel 0 capture compare register value */
    case TIMER_CH_0:
        ret_val = TIMER_CH0CV(timer_periph);
        break;
    /* read TIMER channel 1 capture compare register value */
    case TIMER_CH_1:
        ret_val = TIMER_CH1CV(timer_periph);
        break;
    /* read TIMER channel 2 capture compare register value */
    case TIMER_CH_2:
        ret_val = TIMER_CH2CV(timer_periph);
        break;
    /* read TIMER channel 3 capture compare register value */
    case TIMER_CH_3:
        ret_val = TIMER_CH3CV(timer_periph);
        break;
    default:
        break;
    }
    return (ret_val);
}

/*!
    \brief      enable TIMER break function
    \param[in]  timer_periph: TIMERx(x=0,14..16)
    \param[out] none
    \retval     none
*/
void hals_timer_break_enable(uint32_t timer_periph)
{
    TIMER_CCHP(timer_periph) |= (uint32_t)TIMER_CCHP_BRKEN;
}

/*!
    \brief      disable TIMER break function
    \param[in]  timer_periph: TIMERx(x=0,14..16)
    \param[out] none
    \retval     none
*/
void hals_timer_break_disable(uint32_t timer_periph)
{
    TIMER_CCHP(timer_periph) &= ~(uint32_t)TIMER_CCHP_BRKEN;
}

/*!
    \brief      enable TIMER output automatic function
    \param[in]  timer_periph: TIMERx(x=0,14..16)
    \param[out] none
    \retval     none
*/
void hals_timer_automatic_output_enable(uint32_t timer_periph)
{
    TIMER_CCHP(timer_periph) |= (uint32_t)TIMER_CCHP_OAEN;
}

/*!
    \brief      disable TIMER output automatic function
    \param[in]  timer_periph: TIMERx(x=0,14..16)
    \param[out] none
    \retval     none
*/
void hals_timer_automatic_output_disable(uint32_t timer_periph)
{
    TIMER_CCHP(timer_periph) &= ~(uint32_t)TIMER_CCHP_OAEN;
}

/*!
    \brief      configure TIMER channel remap function
    \param[in]  timer_periph: TIMERx(x=13)
    \param[in]  remap:
                only one parameter can be selected which is shown as below:
      \arg        TIMER13_CI0_RMP_GPIO: timer13 channel 0 input is connected to GPIO(TIMER13_CH0)
      \arg        TIMER13_CI0_RMP_RTCCLK: timer13 channel 0 input is connected to the RTCCLK
      \arg        TIMER13_CI0_RMP_HXTAL_DIV32: timer13 channel 0 input is connected to HXTAL/32 clock
      \arg        TIMER13_CI0_RMP_CKOUTSEL: timer13 channel 0 input is connected to CKOUTSEL
    \param[out] none
    \retval     none
*/
void hals_timer_channel_remap_config(uint32_t timer_periph, uint32_t remap)
{
    TIMER_IRMP(timer_periph) = (uint32_t)remap;
}

/*!
    \brief      configure TIMER write CHxVAL register selection
    \param[in]  timer_periph: TIMERx(x=0,2,13..16), TIMER1 just for GD32F330 and GD32F350
    \param[in]  ccsel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CHVSEL_DISABLE: no effect
      \arg        TIMER_CHVSEL_ENABLE:  when write the CHxVAL register, if the write value is same as the CHxVAL value, the write access is ignored
    \param[out] none
    \retval     none
*/
void hals_timer_write_chxval_register_config(uint32_t timer_periph, uint16_t chvsel)
{
    if(TIMER_CHVSEL_ENABLE == chvsel) {
        TIMER_CFG(timer_periph) |= (uint32_t)TIMER_CFG_CHVSEL;
    } else if(TIMER_CHVSEL_DISABLE == chvsel) {
        TIMER_CFG(timer_periph) &= ~(uint32_t)TIMER_CFG_CHVSEL;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      configure TIMER output value selection
    \param[in]  timer_periph: TIMERx(x=0,14..16)
    \param[in]  outsel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OUTSEL_DISABLE: no effect
      \arg        TIMER_OUTSEL_ENABLE: if POEN and IOS is 0, the output disabled
    \param[out] none
    \retval     none
*/
void hals_timer_output_value_selection_config(uint32_t timer_periph, uint16_t outsel)
{
    if(TIMER_OUTSEL_ENABLE == outsel) {
        TIMER_CFG(timer_periph) |= (uint32_t)TIMER_CFG_OUTSEL;
    } else if(TIMER_OUTSEL_DISABLE == outsel) {
        TIMER_CFG(timer_periph) &= ~(uint32_t)TIMER_CFG_OUTSEL;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      DMA transmission complete(TC) callback for TIMER channel input capture DMA request
    \param[in]  dma: DMA device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _update_dma_full_transfer_complete(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_timer_dev_struct *timer_dev;
    /* parameter assignment */
    p_dma = (hal_dma_dev_struct *)dma;
    timer_dev = (hal_timer_dev_struct *)(p_dma->p_periph);

    /* call DMA transmission complete(TC) handle for TIMER input capture DMA request */
    if(NULL != (timer_dev->timer_dma.update_dma_full_transcom_handle)) {
        timer_dev->timer_dma.update_dma_full_transcom_handle(timer_dev);
    }
}

/*!
    \brief      DMA transmission complete(TC) callback for TIMER channel input capture DMA request
    \param[in]  dma: DMA device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _channelx_capture_dma_full_transfer_complete(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_timer_dev_struct *timer_dev;
    /* parameter assignment */
    p_dma = (hal_dma_dev_struct *)dma;
    timer_dev = (hal_timer_dev_struct *)(p_dma->p_periph);
    /* service channel assignment */
    if(dma == timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]) {
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_0;
    } else if(dma == timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]) {
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_1;
    } else if(dma == timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]) {
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_2;
    } else if(dma == timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]) {
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_3;
    }
    /* call DMA transmission complete(TC) handle for TIMER input capture DMA request */
    if(NULL != (timer_dev->timer_dma.channelx_capture_dma_full_transcom_handle)) {
        timer_dev->timer_dma.channelx_capture_dma_full_transcom_handle(timer_dev);
    }
    timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_NONE;
}

/*!
    \brief      DMA transmission complete(TC) callback for TIMER channel input capture DMA request
    \param[in]  dma: DMA device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _channelx_compare_dma_full_transfer_complete(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_timer_dev_struct *timer_dev;
    /* parameter assignment */
    p_dma = (hal_dma_dev_struct *)dma;
    timer_dev = (hal_timer_dev_struct *)(p_dma->p_periph);
    /* service channel assignment */
    if(dma == timer_dev->p_dma_timer[TIMER_DMA_ID_CH0]) {
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_0;
    } else if(dma == timer_dev->p_dma_timer[TIMER_DMA_ID_CH1]) {
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_1;
    } else if(dma == timer_dev->p_dma_timer[TIMER_DMA_ID_CH2]) {
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_2;
    } else if(dma == timer_dev->p_dma_timer[TIMER_DMA_ID_CH3]) {
        timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_3;
    }
    /* call DMA transmission complete(TC) handle for TIMER input capture DMA request */
    if(NULL != (timer_dev->timer_dma.channelx_compare_dma_full_transcom_handle)) {
        timer_dev->timer_dma.channelx_compare_dma_full_transcom_handle(timer_dev);
    }
    timer_dev->service_channel = HAL_TIMER_SERVICE_CHANNEL_NONE;
}

/*!
    \brief      DMA transmission complete(TC) callback for TIMER channel input capture DMA request
    \param[in]  dma: DMA device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _commutation_dma_full_transfer_complete(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_timer_dev_struct *timer_dev;
    /* parameter assignment */
    p_dma = (hal_dma_dev_struct *)dma;
    timer_dev = (hal_timer_dev_struct *)(p_dma->p_periph);
    /* call DMA transmission complete(TC) handle for TIMER input capture DMA request */
    if(NULL != (timer_dev->timer_dma.commutation_dma_full_transcom_handle)) {
        timer_dev->timer_dma.commutation_dma_full_transcom_handle(timer_dev);
    }
}

/*!
    \brief      DMA transmission complete(TC) callback for TIMER commutation DMA request
    \param[in]  dma: DMA device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _trigger_dma_full_transfer_complete(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_timer_dev_struct *timer_dev;
    /* parameter assignment */
    p_dma = (hal_dma_dev_struct *)dma;
    timer_dev = (hal_timer_dev_struct *)(p_dma->p_periph);
    /* call DMA transmission complete(TC) handle for TIMER commutation DMA request */
    if(NULL != (timer_dev->timer_dma.trigger_dma_full_transcom_handle)) {
        timer_dev->timer_dma.trigger_dma_full_transcom_handle(timer_dev);
    }
}

/*!
    \brief      DMA transmission complete(TC) callback for TIMER trigger DMA request
    \param[in]  dma: DMA device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _timer_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_timer_dev_struct *timer_dev;
    /* parameter assignment */
    p_dma = (hal_dma_dev_struct *)dma;
    timer_dev = (hal_timer_dev_struct *)(p_dma->p_periph);
    /* call DMA transmission complete(TC) handle for TIMER trigger DMA request */
    if(NULL != (timer_dev->timer_dma.error_handle)) {
        timer_dev->timer_dma.error_handle(timer_dev);
    }
}

/*!
    \brief      enable TIMER
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _timer_enable(hal_timer_dev_struct *timer_dev)
{
#if (defined(GD32F350) || defined(GD32F330))
    /* check the parameters */
    if((TIMER0 == timer_dev->periph) || (TIMER1 == timer_dev->periph) || (TIMER2 == timer_dev->periph) || (TIMER14 == timer_dev->periph)) {
        if(TIMER_SLAVE_MODE_EVENT != (uint32_t)(TIMER_SMCFG(timer_dev->periph) & TIMER_SMCFG_SMC)) {
            /* enable a TIMER */
            TIMER_CTL0(timer_dev->periph) |= (uint32_t)TIMER_CTL0_CEN;
        }
    } else {
        /* enable a TIMER */
        TIMER_CTL0(timer_dev->periph) |= (uint32_t)TIMER_CTL0_CEN;
    }
#else
    /* check the parameters */
    if((TIMER0 == timer_dev->periph) || (TIMER2 == timer_dev->periph) || (TIMER14 == timer_dev->periph)) {
        if(TIMER_SLAVE_MODE_EVENT != (uint32_t)(TIMER_SMCFG(timer_dev->periph) & TIMER_SMCFG_SMC)) {
            /* enable a TIMER */
            TIMER_CTL0(timer_dev->periph) |= (uint32_t)TIMER_CTL0_CEN;
        }
    } else {
        /* enable a TIMER */
        TIMER_CTL0(timer_dev->periph) |= (uint32_t)TIMER_CTL0_CEN;
    }
#endif /* GD32F350 */
}

/*!
    \brief      disable TIMER
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE HAL_ERR_NO_SUPPORT, details refer to gd32f3x0_hal.h
*/
static int32_t _timer_disable(hal_timer_dev_struct *timer_dev)
{
    uint32_t chctl2;
    chctl2 = TIMER_CHCTL2(timer_dev->periph);
    /* determine whether channel is disabled */
    if(0 == (chctl2 & TIMER_CHX_EN_MASK)) {
        if(0 == (chctl2 & TIMER_CHNX_EN_MASK)) {
            /* complementary channel is disabled */
            hals_timer_disable(timer_dev->periph);
            return HAL_ERR_NONE;
        } else {
            /* complementary channel is not disabled */
            return HAL_ERR_NO_SUPPORT;
        }
    } else {
        /* channel is not disabled */
        return HAL_ERR_NO_SUPPORT;
    }
}

/*!
    \brief      enable or disable TIMER primary output
    \param[in]  timer_dev: TIMER device information structure
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  state: EBANLE, DISABLE
    \param[out] none
    \retval     none
*/
static void _timer_primary_output_config(hal_timer_dev_struct *timer_dev, ControlStatus state)
{
    uint32_t chctl2;
    chctl2 = TIMER_CHCTL2(timer_dev->periph);
    /* TIMER which has primary output enable(POE) bit */
    if((TIMER0 == timer_dev->periph) || (TIMER14 == timer_dev->periph)
            || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)) {
        if(ENABLE == state) {
            /* enable TIMER primary output */
            TIMER_CCHP(timer_dev->periph) |= (uint32_t)TIMER_CCHP_POEN;
        } else {
            /* disable TIMER primary output */
            if(0 == (chctl2 & TIMER_CHX_EN_MASK)) {
                if(0 == (chctl2 & TIMER_CHNX_EN_MASK)) {
                    TIMER_CCHP(timer_dev->periph) &= (~(uint32_t)TIMER_CCHP_POEN);
                }
            }
        }
    }
}
