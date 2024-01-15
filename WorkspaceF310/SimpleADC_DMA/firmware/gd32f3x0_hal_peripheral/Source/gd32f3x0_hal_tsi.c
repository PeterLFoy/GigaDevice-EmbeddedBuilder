/*!
    \file    gd32f3x0_hal_tsi.c
    \brief   TSI driver

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

#if defined(GD32F350)

#include "gd32f3x0_hal.h"

static uint32_t _hals_used_groups(uint32_t channel_pins);

/*!
    \brief      initialize TSI
    \param[in]  tsi_dev: TSI device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  tsi_init: the pointer of TSI init structure
                  charge_time:
                  only one parameter can be selected which is shown as below:
      \arg          TSI_CHARGE_xCTCLK (x = 1...16): charge state duration time
                  transfer_time:
                  only one parameter can be selected which is shown as below:
      \arg          TSI_TRANSFER_xCTCLK (x = 1...16): charge transfer state duration time
                ctclk_div:
                only one parameter can be selected which is shown as below:
      \arg          TSI_CTCDIV_DIV1: fCTCLK = fHCLK
      \arg          TSI_CTCDIV_DIV2: fCTCLK = fHCLK/2
      \arg          TSI_CTCDIV_DIV4: fCTCLK = fHCLK/4
      \arg          TSI_CTCDIV_DIV8: fCTCLK = fHCLK/8
      \arg          TSI_CTCDIV_DIV16: fCTCLK = fHCLK/16
      \arg          TSI_CTCDIV_DIV32: fCTCLK = fHCLK/32
      \arg          TSI_CTCDIV_DIV64: fCTCLK = fHCLK/64
      \arg          TSI_CTCDIV_DIV128: fCTCLK = fHCLK/128
      \arg          TSI_CTCDIV_DIV256: fCTCLK = fHCLK/256
      \arg          TSI_CTCDIV_DIV512: fCTCLK = fHCLK/512
      \arg          TSI_CTCDIV_DIV1024: fCTCLK = fHCLK/1024
      \arg          TSI_CTCDIV_DIV2048: fCTCLK = fHCLK/2048
      \arg          TSI_CTCDIV_DIV4096: fCTCLK = fHCLK/4096
      \arg          TSI_CTCDIV_DIV8192: fCTCLK = fHCLK/8192
      \arg          TSI_CTCDIV_DIV16384: fCTCLK = fHCLK/16384
      \arg          TSI_CTCDIV_DIV32768: fCTCLK = fHCLK/32768
                seq_max:
                only one parameter can be selected which is shown as below:
      \arg          TSI_MAXNUM255: the max cycle number of a sequence is 255
      \arg          TSI_MAXNUM511: the max cycle number of a sequence is 511
      \arg          TSI_MAXNUM1023: the max cycle number of a sequence is 1023
      \arg          TSI_MAXNUM2047: the max cycle number of a sequence is 2047
      \arg          TSI_MAXNUM4095: the max cycle number of a sequence is 4095
      \arg          TSI_MAXNUM8191: the max cycle number of a sequence is 8191
      \arg          TSI_MAXNUM16383: the max cycle number of a sequence is 16383
                extend_charge_state:
                only one parameter can be selected which is shown as below:
      \arg          TSI_EXTEND_CHARGE_DISABLE: extend charge state disable
      \arg          TSI_EXTEND_CHARGE_ENABLE: extend charge state enable
                ecclk_div: extend charge clock(ecclk) division factor
      \arg          TSI_EXTEND_DIV1: fECCLK = fHCLK
      \arg          TSI_EXTEND_DIV2: fECCLK = fHCLK/2
      \arg          TSI_EXTEND_DIV3: fECCLK = fHCLK/3
      \arg          TSI_EXTEND_DIV4: fECCLK = fHCLK/4
      \arg          TSI_EXTEND_DIV5: fECCLK = fHCLK/5
      \arg          TSI_EXTEND_DIV6: fECCLK = fHCLK/6
      \arg          TSI_EXTEND_DIV7: fECCLK = fHCLK/7
      \arg          TSI_EXTEND_DIV8: fECCLK = fHCLK/8
                extend_charge_time: 1~128
                pin_mode:
      \arg          TSI_OUTPUT_LOW: TSI pin will output low when IDLE
      \arg          TSI_INPUT_FLOATING:  TSI pin will keep input_floating when IDLE
                edge_sel:
      \arg          TSI_FALLING_TRIGGER: falling edge trigger TSI charge transfer sequence
      \arg          TSI_RISING_TRIGGER:  rising edge trigger TSI charge transfer sequence
                trig_mode:
      \arg          TSI_HW_TRIGGER_DISABLE: hardware trigger disable
      \arg          TSI_HW_TRIGGER_ENABLE: hardware trigger enable
                 sample_pins:
                 one or more parameters can be selected which are shown as below:
      \arg          TSI_GxPy(x=0..5,y=0..3), pin y of group x
                 channel_pins:
                 one or more parameters can be selected which are shown as below:
      \arg          TSI_GxPy(x=0..5,y=0..3), pin y of group x
                 shield_pins:
                 one or more parameters can be selected which are shown as below:
      \arg          TSI_GxPy(x=0..5,y=0..3), pin y of group x
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_tsi_init(hal_tsi_dev_struct *tsi_dev, hal_tsi_init_struct *tsi_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == tsi_dev) || (NULL == tsi_init)) {
        HAL_DEBUGE("pointer [*tsi_dev] or pointer [*tsi_init] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    tsi_dev->state = HAL_TSI_STATE_BUSY;

    /* tsi param config */
    /* configure charge pulse and transfer pulse */
    hals_tsi_pulse_config(tsi_init->ctclk_div, tsi_init->charge_time, tsi_init->transfer_time);
    /* configure the max cycle number of a charge-transfer sequence */
    hals_tsi_max_number_config(tsi_init->seq_max);
    /* configure extend charge state */
    hals_tsi_extend_charge_config((ControlStatus) tsi_init->extend_charge_state, tsi_init->ecclk_div,
                                  tsi_init->extend_charge_time);
    /* TSI IO pin will output low or keep input_floating when IDLE  */
    hals_tsi_pin_mode_config(tsi_init->pin_mode);
    /* set hardware trigger mode */
    hals_tsi_hardware_mode_config(tsi_init->edge_sel);
    /* TSI trigger mode select */
    TSI_CTL0 &= ~TSI_CTL0_TRGMOD;
    TSI_CTL0 |= tsi_init->trig_mode;
    /* switch off hysteresis function of channel */
    TSI_PHM = ~(tsi_init->sample_pins | tsi_init->channel_pins | tsi_init->shield_pins);
    /* sample pin config */
    TSI_SAMPCFG = (tsi_init->sample_pins);
    /* channel and shield pin config */
    TSI_CHCFG = (tsi_init->channel_pins | tsi_init->shield_pins);
    /* Group config */
    TSI_GCTL = _hals_used_groups(tsi_init->channel_pins);

    /* disable interrupts and clear flags */
    TSI_INTEN &= (~(TSI_INTEN_CTCFIE | TSI_INTEN_MNERRIE));
    TSI_INTC |= (TSI_INTF_CTCF | TSI_INTF_MNERR);

    /* enable TSI module */
    TSI_CTL0 |= TSI_CTL0_TSIEN;

    /* change TSI error state */
    tsi_dev->error_state = HAL_TSI_ERROR_NONE;
    /* change TSI state */
    tsi_dev->state = HAL_TSI_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize the TSI structure with the default values
    \param[in]  hal_struct_type: the argument could be selected from enumeration <hal_tsi_struct_type_enum>
    \param[out]  p_struct: pointer to TSI structure that contains the configuration information
    \retval     none
*/
void hal_tsi_struct_init(hal_tsi_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer [*p_struct] value is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_TSI_INIT_STRUCT:
        /* initialize TSI initialization structure with the default values */
        ((hal_tsi_init_struct *)p_struct)->charge_time                 = TSI_CHARGE_1CTCLK;
        ((hal_tsi_init_struct *)p_struct)->transfer_time               = TSI_TRANSFER_1CTCLK;
        ((hal_tsi_init_struct *)p_struct)->ctclk_div                   = TSI_CTCDIV_DIV1;
        ((hal_tsi_init_struct *)p_struct)->seq_max                     = TSI_MAXNUM255;
        ((hal_tsi_init_struct *)p_struct)->extend_charge_state         = TSI_EXTEND_CHARGE_DISABLE;
        ((hal_tsi_init_struct *)p_struct)->ecclk_div                   = TSI_EXTEND_DIV1;
        ((hal_tsi_init_struct *)p_struct)->extend_charge_time          = TSI_EXTENDMAX(0);
        ((hal_tsi_init_struct *)p_struct)->pin_mode                    = TSI_OUTPUT_LOW;
        ((hal_tsi_init_struct *)p_struct)->edge_sel                    = TSI_FALLING_TRIGGER;
        ((hal_tsi_init_struct *)p_struct)->trig_mode                   = TSI_HW_TRIGGER_DISABLE;
        ((hal_tsi_init_struct *)p_struct)->sample_pins                 = 0U;
        ((hal_tsi_init_struct *)p_struct)->channel_pins                = 0U;
        ((hal_tsi_init_struct *)p_struct)->shield_pins                 = 0U;
        break;
    case HAL_TSI_IRQ_STRUCT:
        /* initialize TSI initialization structure with the default values */
        ((hal_tsi_irq_struct *)p_struct)->tsi_cctcf_handle             = NULL;
        ((hal_tsi_irq_struct *)p_struct)->tsi_mnerr_handle             = NULL;
        break;
    case HAL_TSI_DEV_STRUCT:
        /* initialize TSI initialization structure with the default values */
        ((hal_tsi_dev_struct *)p_struct)->tsi_irq.tsi_cctcf_handle     = NULL;
        ((hal_tsi_dev_struct *)p_struct)->tsi_irq.tsi_mnerr_handle     = NULL;
        ((hal_tsi_dev_struct *)p_struct)->error_state                  = HAL_TSI_ERROR_NONE;
        ((hal_tsi_dev_struct *)p_struct)->state                        = HAL_TSI_STATE_NONE;
        ((hal_tsi_dev_struct *)p_struct)->mutex                        = HAL_MUTEX_UNLOCKED;
        ((hal_tsi_dev_struct *)p_struct)->priv                         = NULL;
        break;
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize TSI device structure and init structure
    \param[out]  tsi_dev: TSI device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_tsi_deinit(hal_tsi_dev_struct *tsi_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == tsi_dev) {
        HAL_DEBUGE("pointer [tsi_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    tsi_dev->state = HAL_TSI_STATE_BUSY;
    /* deinit TSI */
    hal_rcu_periph_reset_enable(RCU_TSIRST);
    hal_rcu_periph_reset_disable(RCU_TSIRST);

    /* change TSI error state and state */
    tsi_dev->error_state = HAL_TSI_ERROR_NONE;
    tsi_dev->state = HAL_TSI_STATE_RESET;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      start TSI module function
    \param[in]  tsi_dev: TSI device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_tsi_start(hal_tsi_dev_struct *tsi_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == tsi_dev) {
        HAL_DEBUGE("pointer [tsi_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    tsi_dev->state = HAL_TSI_STATE_BUSY;

    /* disable interrupts and clear flags */
    TSI_INTEN &= ~(TSI_INTEN_CTCFIE | TSI_INTEN_MNERRIE);
    TSI_INTC |= (TSI_INTF_CTCF | TSI_INTF_MNERR);

    /* set gpio pin mode to output low when idle */
    hals_tsi_pin_mode_config(TSI_OUTPUT_LOW);
    /* start a charge-transfer sequence */
    TSI_CTL0 |= TSI_CTL0_TSIS;

    tsi_dev->state = HAL_TSI_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      stop tsi module function
    \param[in]  tsi_dev: TSI device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_tsi_stop(hal_tsi_dev_struct *tsi_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == tsi_dev) {
        HAL_DEBUGE("pointer [tsi_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* stop a charge-transfer sequence */
    TSI_CTL0 &= ~TSI_CTL0_TSIS;
    /* set gpio pin mode to output low when idle */
    hals_tsi_pin_mode_config(TSI_OUTPUT_LOW);
    /* clear flags */
    TSI_INTC |= (TSI_INTF_CTCF | TSI_INTF_MNERR);

    tsi_dev->state = HAL_TSI_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      start TSI interrupt
    \param[in]  tsi_dev: TSI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: TSI interrupt callback function structure
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_tsi_start_interrupt(hal_tsi_dev_struct *tsi_dev, hal_tsi_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == tsi_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [tsi_dev] or pointer [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* lock TSI */
    HAL_LOCK(tsi_dev);
    tsi_dev->state = HAL_TSI_STATE_BUSY;
    tsi_dev->tsi_irq.tsi_cctcf_handle = p_irq->tsi_cctcf_handle;
    tsi_dev->tsi_irq.tsi_mnerr_handle = p_irq->tsi_mnerr_handle;

    /* clear the specified TSI interrupt flag and enable TSI interrupt */
    if(NULL != tsi_dev->tsi_irq.tsi_cctcf_handle) {
        TSI_INTC |= (TSI_INTF_CTCF);
        TSI_INTEN |= (TSI_INTEN_CTCFIE);
    } else {
        TSI_INTEN &= ~(TSI_INTEN_CTCFIE);
    }

    if(NULL != tsi_dev->tsi_irq.tsi_mnerr_handle) {
        TSI_INTC |= (TSI_INTF_MNERR);
        TSI_INTEN |= (TSI_INTEN_MNERRIE);
    } else {
        TSI_INTEN &= ~(TSI_INTEN_MNERRIE);
    }

    /* set gpio pin mode to output low when idle */
    hals_tsi_pin_mode_config(TSI_OUTPUT_LOW);
    /* start a charge-transfer sequence */
    TSI_CTL0 |= TSI_CTL0_TSIS;

    /* unlock TSI */
    HAL_UNLOCK(tsi_dev);

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TSI interrupt
    \param[in]  tsi_dev: TSI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_tsi_stop_interrupt(hal_tsi_dev_struct *tsi_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == tsi_dev) {
        HAL_DEBUGE("pointer [tsi_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* lock TSI */
    HAL_LOCK(tsi_dev);
    tsi_dev->state = HAL_TSI_STATE_BUSY;
    tsi_dev->tsi_irq.tsi_cctcf_handle = NULL;
    tsi_dev->tsi_irq.tsi_mnerr_handle = NULL;

    /* stop a charge-transfer sequence */
    TSI_CTL0 &= ~TSI_CTL0_TSIS;
    /* clear the TSI interrupt flag */
    TSI_INTC |= (TSI_INTF_CTCF | TSI_INTF_MNERR);
    /* disable TSI interrupt */
    TSI_INTEN &= ~(TSI_INTEN_CTCFIE | TSI_INTEN_MNERRIE);
    /* unlock TSI */
    HAL_UNLOCK(tsi_dev);
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      TSI interrupt handler content function,which is merely used in tsi_handler
    \param[in]  tsi_dev: TSI device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_tsi_irq(hal_tsi_dev_struct *tsi_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == tsi_dev) {
        HAL_DEBUGE("pointer [tsi_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* check tsi interrupt state bit */
    if(SET == hals_tsi_flag_get(TSI_FLAG_MNERR)) {
        /* change TSI state to error state */
        tsi_dev->state = HAL_TSI_STATE_ERROR;
        /* set TSI error state to max cycle number error */
        tsi_dev->error_state = HAL_TSI_ERROR_MNERR;
        /* clear max cycle number error flag */
        TSI_INTC |= (TSI_INTF_MNERR);

        /* TSI max cycle number error handler callback */
        if(NULL != (tsi_dev->tsi_irq.tsi_mnerr_handle)) {
            tsi_dev->tsi_irq.tsi_mnerr_handle(tsi_dev);
        }
    }
    /* check tsi interrupt state bit */
    if(SET == hals_tsi_flag_get(TSI_FLAG_CTCF)) {
        /* clear charge transfer complete flag */
        TSI_INTC |= (TSI_INTF_CTCF);

        /* TSI charge-transfer complete handler callback */
        if(NULL != (tsi_dev->tsi_irq.tsi_cctcf_handle)) {
            tsi_dev->tsi_irq.tsi_cctcf_handle(tsi_dev);
        }
    }
}

/*!
    \brief      set user-defined interrupt callback function
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  tsi_dev: TSI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to TSI interrupt callback functions structure
                  hal_irq_handle_cb: the function is user-defined,
    \param[out] none
    \retval     none
*/
void hal_tsi_irq_handle_set(hal_tsi_dev_struct *tsi_dev, hal_tsi_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == tsi_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [tsi_dev] or pointer [p_irq] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* TSI interrupt handler set */
    if(NULL != p_irq->tsi_cctcf_handle) {
        tsi_dev->tsi_irq.tsi_cctcf_handle = p_irq->tsi_cctcf_handle;
        TSI_INTEN |= (TSI_INTEN_CTCFIE);
    } else {
        tsi_dev->tsi_irq.tsi_cctcf_handle = NULL;
        TSI_INTEN &= ~(TSI_INTEN_CTCFIE);
    }
    if(NULL != p_irq->tsi_mnerr_handle) {
        tsi_dev->tsi_irq.tsi_mnerr_handle = p_irq->tsi_mnerr_handle;
        TSI_INTEN |= (TSI_INTEN_MNERRIE);
    } else {
        tsi_dev->tsi_irq.tsi_mnerr_handle = NULL;
        TSI_INTEN &= ~(TSI_INTEN_MNERRIE);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  tsi_dev: TSI device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_tsi_irq_handle_all_reset(hal_tsi_dev_struct *tsi_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == tsi_dev) {
        HAL_DEBUGE("pointer [tsi_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* TSI interrupt handler set */
    tsi_dev->tsi_irq.tsi_cctcf_handle = NULL;
    tsi_dev->tsi_irq.tsi_mnerr_handle = NULL;
}

/*!
    \brief      get the cycle number of specific group as soon as a charge-transfer sequence completes
    \param[in]  tsi_dev: TSI device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  group_id: TSI group ID, TSI_GROUP_IDX0~TSI_GROUP_IDX5
    \param[out] none
    \retval     tsi group cycle value: 0x0 ¨C 0x3FFF
*/
uint16_t hal_tsi_group_cycle_get(hal_tsi_dev_struct *tsi_dev, uint8_t group_id)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == tsi_dev) {
        HAL_DEBUGE("pointer [tsi_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    return (uint16_t)(REG32(TSI + 0x00000034U + (group_id) * 4));
}

/*!
    \brief      configure TSI pins
    \param[in]  tsi_dev: TSI device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  pins_config: pointer to hal_tsi_init_struct
    \param[out] none
    \retval     none
*/
void hal_tsi_pins_config(hal_tsi_dev_struct *tsi_dev, hal_tsi_init_struct *pins_config)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == tsi_dev) {
        HAL_DEBUGE("pointer [tsi_dev] or pointer [pins_config] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    hal_tsi_stop(tsi_dev);
    /* switch off hysteresis function of channel */
    TSI_PHM = ~(pins_config->sample_pins | pins_config->channel_pins | pins_config->shield_pins);
    /* sample pin config */
    TSI_SAMPCFG = (pins_config->sample_pins);
    /* channel and shield pin config */
    TSI_CHCFG = (pins_config->channel_pins | pins_config->shield_pins);
    /* group config */
    TSI_GCTL = _hals_used_groups(pins_config->channel_pins);
}

/*!
    \brief      TSI poll for charge-transfer sequence complete
    \param[in]  tsi_dev: TSI device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_tsi_poll_transfer(hal_tsi_dev_struct *tsi_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == tsi_dev) {
        HAL_DEBUGE("pointer [tsi_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* wait for charge-transfer sequence complete */
    while(SET == hals_tsi_flag_get(TSI_FLAG_CTCF));
}

/*!
    \brief      enable TSI module
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_tsi_enable(void)
{
    TSI_CTL0 |= TSI_CTL0_TSIEN;
}

/*!
    \brief      disable TSI module
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_tsi_disable(void)
{
    TSI_CTL0 &= ~TSI_CTL0_TSIEN;
}

/*!
    \brief      enable sample pin
    \param[in]  sample: sample pin
                one or more parameters can be selected which are shown as below:
      \arg        TSI_SAMPCFG_GxPy(x=0..5,y=0..3):pin y of group x is sample pin
    \param[out] none
    \retval     none
*/
void hals_tsi_sample_pin_enable(uint32_t sample)
{
    if(RESET == (TSI_CTL0 & TSI_CTL0_TSIS)) {
        TSI_SAMPCFG |= sample;
    }
}

/*!
    \brief      disable sample pin
    \param[in]  sample: sample pin
                one or more parameters can be selected which are shown as below:
      \arg        TSI_SAMPCFG_GxPy(x=0..5,y =0..3): pin y of group x is sample pin
    \param[out] none
    \retval     none
*/
void hals_tsi_sample_pin_disable(uint32_t sample)
{
    if(RESET == (TSI_CTL0 & TSI_CTL0_TSIS)) {
        TSI_SAMPCFG &= ~sample;
    }
}

/*!
    \brief      enable channel pin
    \param[in]  channel: channel pin
                one or more parameters can be selected which are shown as below:
      \arg        TSI_CHCFG_GxPy(x=0..5,y=0..3): pin y of group x
    \param[out] none
    \retval     none
*/
void hals_tsi_channel_pin_enable(uint32_t channel)
{
    TSI_CHCFG |= channel;
}

/*!
    \brief      disable channel pin
    \param[in]  channel: channel pin
                one or more parameters can be selected which are shown as below:
      \arg        TSI_CHCFG_GxPy(x=0..5,y=0..3): pin y of group x
    \param[out] none
    \retval     none
*/
void hals_tsi_channel_pin_disable(uint32_t channel)
{
    TSI_CHCFG &= ~channel;
}

/*!
    \brief      start a charge-transfer sequence when TSI is in software trigger mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_tsi_software_start(void)
{
    TSI_CTL0 |= TSI_CTL0_TSIS;
}

/*!
    \brief      stop a charge-transfer sequence when TSI is in software trigger mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_tsi_software_stop(void)
{
    TSI_CTL0 &= ~TSI_CTL0_TSIS;
}

/*!
    \brief      configure TSI triggering by hardware
    \param[in]  trigger_edge: the edge type in hardware trigger mode
                only one parameter can be selected which is shown as below:
      \arg        TSI_FALLING_TRIGGER: falling edge trigger TSI charge transfer sequence
      \arg        TSI_RISING_TRIGGER:  rising edge trigger TSI charge transfer sequence
    \param[out] none
    \retval     none
*/
void hals_tsi_hardware_mode_config(uint8_t trigger_edge)
{
    if(RESET == (TSI_CTL0 & TSI_CTL0_TSIS)) {
        /*configure the edge type in hardware trigger mode*/
        if(TSI_FALLING_TRIGGER == trigger_edge) {
            TSI_CTL0 &= ~TSI_CTL0_EGSEL;
        } else {
            TSI_CTL0 |= TSI_CTL0_EGSEL;
        }
    }
}

/*!
    \brief      configure TSI pin mode when charge-transfer sequence is IDLE
    \param[in]  pin_mode: pin mode when charge-transfer sequence is IDLE
                only one parameter can be selected which is shown as below:
      \arg        TSI_OUTPUT_LOW: TSI pin will output low when IDLE
      \arg        TSI_INPUT_FLOATING: TSI pin will keep input_floating when IDLE
    \param[out] none
    \retval     none
*/
void hals_tsi_pin_mode_config(uint8_t pin_mode)
{
    if(RESET == (TSI_CTL0 & TSI_CTL0_TSIS)) {
        if(TSI_OUTPUT_LOW == pin_mode) {
            TSI_CTL0 &= ~TSI_CTL0_PINMOD;
        } else {
            TSI_CTL0 |= TSI_CTL0_PINMOD;
        }
    }
}

/*!
    \brief      configure extend charge state
    \param[in]  extend: enable or disable extend charge state
                only one parameter can be selected which is shown as below:
      \arg        ENABLE:  enable extend charge state
      \arg        DISABLE: disable extend charge state
    \param[in]  prescaler: ECCLK clock division factor
                only one parameter can be selected which is shown as below:
      \arg        TSI_EXTEND_DIV1: fECCLK = fHCLK
      \arg        TSI_EXTEND_DIV2: fECCLK = fHCLK/2
      \arg        TSI_EXTEND_DIV3: fECCLK = fHCLK/3
      \arg        TSI_EXTEND_DIV4: fECCLK = fHCLK/4
      \arg        TSI_EXTEND_DIV5: fECCLK = fHCLK/5
      \arg        TSI_EXTEND_DIV6: fECCLK = fHCLK/6
      \arg        TSI_EXTEND_DIV7: fECCLK = fHCLK/7
      \arg        TSI_EXTEND_DIV8: fECCLK = fHCLK/8
    \param[in]  max_duration: value range 1...128,extend charge state maximum duration time is 1*tECCLK~128*tECCLK
    \param[out] none
    \retval     none
*/
void hals_tsi_extend_charge_config(ControlStatus extend, uint8_t prescaler, uint32_t max_duration)
{
    uint32_t ctl0, ctl1;

    if(RESET == (TSI_CTL0 & TSI_CTL0_TSIS)) {
        if(DISABLE == extend) {
            /*disable extend charge state*/
            TSI_CTL0 &= ~TSI_CTL0_ECEN;
        } else {
            if(TSI_EXTEND_DIV3 > prescaler) {
                /*configure extend charge state maximum duration time*/
                ctl0 = TSI_CTL0;
                ctl0 &= ~TSI_CTL0_ECDT;
                ctl0 |= TSI_EXTENDMAX((max_duration - 1U));
                TSI_CTL0 = ctl0;
                /*configure ECCLK clock division factor*/
                ctl0 = TSI_CTL0;
                ctl0 &= ~TSI_CTL0_ECDIV;
                ctl0 |= (uint32_t)prescaler << 15U;
                TSI_CTL0 = ctl0;
                ctl1 = TSI_CTL1;
                ctl1 &= ~TSI_CTL1_ECDIV;
                TSI_CTL1 = ctl1;
                /*enable extend charge state*/
                TSI_CTL0 |= TSI_CTL0_ECEN;
            } else {
                /*configure extend charge state maximum duration time*/
                ctl0 = TSI_CTL0;
                ctl0 &= ~TSI_CTL0_ECDT;
                ctl0 |= TSI_EXTENDMAX((max_duration - 1U));
                TSI_CTL0 = ctl0;
                /*configure ECCLK clock division factor*/
                ctl0 = TSI_CTL0;
                ctl0 &= ~TSI_CTL0_ECDIV;
                ctl0 |= (prescaler & 0x01U) << 15U;
                TSI_CTL0 = ctl0;
                ctl1 = TSI_CTL1;
                ctl1 &= ~TSI_CTL1_ECDIV;
                ctl1 |= ((prescaler) & 0x06U) << 27U;
                TSI_CTL1 = ctl1;
                /*enable extend charge state*/
                TSI_CTL0 |= TSI_CTL0_ECEN;
            }
        }
    }
}

/*!
    \brief      configure charge pulse and transfer pulse
    \param[in]  prescaler: CTCLK clock division factor
                only one parameter can be selected which is shown as below:
      \arg        TSI_CTCDIV_DIV1: fCTCLK = fHCLK
      \arg        TSI_CTCDIV_DIV2: fCTCLK = fHCLK/2
      \arg        TSI_CTCDIV_DIV4: fCTCLK = fHCLK/4
      \arg        TSI_CTCDIV_DIV8: fCTCLK = fHCLK/8
      \arg        TSI_CTCDIV_DIV16: fCTCLK = fHCLK/16
      \arg        TSI_CTCDIV_DIV32: fCTCLK = fHCLK/32
      \arg        TSI_CTCDIV_DIV64: fCTCLK = fHCLK/64
      \arg        TSI_CTCDIV_DIV128: fCTCLK = fHCLK/128
      \arg        TSI_CTCDIV_DIV256: fCTCLK = fHCLK/256
      \arg        TSI_CTCDIV_DIV512: fCTCLK = fHCLK/512
      \arg        TSI_CTCDIV_DIV1024: fCTCLK = fHCLK/1024
      \arg        TSI_CTCDIV_DIV2048: fCTCLK = fHCLK/2048
      \arg        TSI_CTCDIV_DIV4096: fCTCLK = fHCLK/4096
      \arg        TSI_CTCDIV_DIV8192: fCTCLK = fHCLK/8192
      \arg        TSI_CTCDIV_DIV16384: fCTCLK = fHCLK/16384
      \arg        TSI_CTCDIV_DIV32768: fCTCLK = fHCLK/32768
    \param[in]  charge_duration: charge state duration time
                only one parameter can be selected which is shown as below:
      \arg        TSI_CHARGE_xCTCLK(x=1..16): the duration time of charge state is x CTCLK
    \param[in]  transfer_duration: charge transfer state duration time
                only one parameter can be selected which is shown as below:
      \arg        TSI_TRANSFER_xCTCLK(x=1..16): the duration time of transfer state is x CTCLK
    \param[out] none
    \retval     none
*/
void hals_tsi_pulse_config(uint32_t prescaler, uint32_t charge_duration, uint32_t transfer_duration)
{
    uint32_t ctl0, ctl1;

    if(RESET == (TSI_CTL0 & TSI_CTL0_TSIS)) {
        if(TSI_CTCDIV_DIV256 > prescaler) {
            /* configure TSI_CTL0 */
            ctl0 = TSI_CTL0;
            /*configure TSI clock division factor,charge state duration time,charge transfer state duration time */
            ctl0 &= ~(TSI_CTL0_CTCDIV | TSI_CTL0_CTDT | TSI_CTL0_CDT);
            ctl0 |= ((prescaler << 12U) | charge_duration | transfer_duration);
            TSI_CTL0 = ctl0;

            /* configure TSI_CTL1 */
            ctl1 = TSI_CTL1;
            ctl1 &= ~TSI_CTL1_CTCDIV;
            TSI_CTL1 = ctl1;
        } else {
            /* configure TSI_CTL */
            ctl0 = TSI_CTL0;
            prescaler &= ~0x08U;
            /*configure TSI clock division factor,charge state duration time,charge transfer state duration time */
            ctl0 &= ~(TSI_CTL0_CTCDIV | TSI_CTL0_CTDT | TSI_CTL0_CDT);
            ctl0 |= ((prescaler << 12U) | charge_duration | transfer_duration);
            TSI_CTL0 = ctl0;

            /* configure TSI_CTL1 */
            ctl1 = TSI_CTL1;
            ctl1 |= TSI_CTL1_CTCDIV;
            TSI_CTL1 = ctl1;
        }
    }
}

/*!
    \brief      configure the max cycle number of a charge-transfer sequence
    \param[in]  max_number: max cycle number
                only one parameter can be selected which is shown as below:
      \arg        TSI_MAXNUM255: the max cycle number of a sequence is 255
      \arg        TSI_MAXNUM511: the max cycle number of a sequence is 511
      \arg        TSI_MAXNUM1023: the max cycle number of a sequence is 1023
      \arg        TSI_MAXNUM2047: the max cycle number of a sequence is 2047
      \arg        TSI_MAXNUM4095: the max cycle number of a sequence is 4095
      \arg        TSI_MAXNUM8191: the max cycle number of a sequence is 8191
      \arg        TSI_MAXNUM16383: the max cycle number of a sequence is 16383
    \param[out] none
    \retval     none
*/
void hals_tsi_max_number_config(uint32_t max_number)
{
    if(RESET == (TSI_CTL0 & TSI_CTL0_TSIS)) {
        uint32_t maxnum;
        maxnum = TSI_CTL0;
        /*configure the max cycle number of a charge-transfer sequence*/
        maxnum &= ~TSI_CTL0_MCN;
        maxnum |= max_number;
        TSI_CTL0 = maxnum;
    }
}

/*!
    \brief      switch on hysteresis pin
    \param[in]  group_pin: select pin which will be switched on hysteresis
                one or more parameters can be selected which are shown as below:
      \arg        TSI_PHM_GxPy(x=0..5,y=0..3): pin y of group x switch on hysteresis
    \param[out] none
    \retval     none
*/
void hals_tsi_hysteresis_on(uint32_t group_pin)
{
    TSI_PHM |= group_pin;
}

/*!
    \brief      switch off hysteresis pin
    \param[in]  group_pin: select pin which will be switched off hysteresis
                one or more parameters can be selected which are shown as below:
      \arg        TSI_PHM_GxPy(x=0..5,y=0..3): pin y of group x switch off hysteresis
    \param[out] none
    \retval     none
*/
void hals_tsi_hysteresis_off(uint32_t group_pin)
{
    TSI_PHM &= ~group_pin;
}

/*!
    \brief      switch on analog pin
    \param[in]  group_pin: select pin which will be switched on analog
                one or more parameters can be selected which are shown as below:
      \arg        TSI_ASW_GxPy(x=0..5,y=0..3):pin y of group x switch on analog
    \param[out] none
    \retval     none
*/
void hals_tsi_analog_on(uint32_t group_pin)
{
    TSI_ASW |= group_pin;
}

/*!
    \brief      switch off analog pin
    \param[in]  group_pin: select pin which will be switched off analog
                one or more parameters can be selected which are shown as below:
      \arg        TSI_ASW_GxPy(x=0..5,y=0..3):pin y of group x switch off analog
    \param[out] none
    \retval     none
*/
void hals_tsi_analog_off(uint32_t group_pin)
{
    TSI_ASW &= ~group_pin;
}

/*!
    \brief      enable group
    \param[in]  group: select group to be enabled
                one or more parameters can be selected which are shown as below:
      \arg        TSI_GCTL_GEx(x=0..5): the x group will be enabled
    \param[out] none
    \retval     none
*/
void hals_tsi_group_enable(uint32_t group)
{
    TSI_GCTL |= group;
}

/*!
    \brief      disable group
    \param[in]  group: select group to be disabled
                one or more parameters can be selected which are shown as below:
      \arg        TSI_GCTL_GEx(x=0..5):the x group will be disabled
    \param[out] none
    \retval     none
*/
void hals_tsi_group_disable(uint32_t group)
{
    TSI_GCTL &= ~group;
}

/*!
    \brief      get group complete status
    \param[in]  group: select group
                only one parameter can be selected which is shown as below:
      \arg        TSI_GCTL_GCx(x=0..5): get the complete status of group x
    \param[out] none
    \retval     FlagStatus: group complete status,SET or RESET
*/
FlagStatus hals_tsi_group_status_get(uint32_t group)
{
    FlagStatus flag_status;

    if(TSI_GCTL & group) {
        flag_status = SET;
    } else {
        flag_status = RESET;
    }
    return flag_status;
}

/*!
    \brief      get TSI flag
    \param[in]  flag:
                only one parameter can be selected which is shown as below:
      \arg        TSI_FLAG_CTCF: charge-transfer complete flag
      \arg        TSI_FLAG_MNERR: max cycle number error
    \param[out] none
    \retval     FlagStatus:SET or RESET
*/
FlagStatus hals_tsi_flag_get(uint32_t flag)
{
    FlagStatus flag_status;
    if(TSI_INTF & flag) {
        flag_status = SET;
    } else {
        flag_status = RESET;
    }
    return flag_status;
}

/*!
    \brief      clear TSI flag
    \param[in]  flag: select flag which will be cleared
                only one parameter can be selected which is shown as below:
      \arg        TSI_FLAG_CTCF: clear charge-transfer complete flag
      \arg        TSI_FLAG_MNERR: clear max cycle number error
    \param[out] none
    \retval     none
*/
void hals_tsi_flag_clear(uint32_t flag)
{
    TSI_INTC |= flag;
}

/*!
    \brief      enable TSI interrupt
    \param[in]  source: select interrupt which will be enabled
                only one parameter can be selected which is shown as below:
      \arg        TSI_INT_CCTCF: charge-transfer complete flag interrupt enable
      \arg        TSI_INT_MNERR:  max cycle number error interrupt enable
    \param[out] none
    \retval     none
*/
void hals_tsi_interrupt_enable(uint32_t source)
{
    TSI_INTEN |= source;
}

/*!
    \brief      disable TSI interrupt
    \param[in]  source: select interrupt which will be disabled
                only one parameter can be selected which is shown as below:
      \arg        TSI_INT_CCTCF: charge-transfer complete flag interrupt disable
      \arg        TSI_INT_MNERR: max cycle number error interrupt disable
    \param[out] none
    \retval     none
*/
void hals_tsi_interrupt_disable(uint32_t source)
{
    TSI_INTEN &= ~source;
}

/*!
    \brief      get used groups
    \param[in]  channel_pins: used channel pins
                 one or more parameters can be selected which are shown as below:
      \arg          TSI_GxPy(x=0..5,y=0..3), pin y of group x
    \param[out] none
    \retval     none
*/
static uint32_t _hals_used_groups(uint32_t channel_pins)
{
    uint32_t groups = 0x0U;
    uint32_t index = 0;

    for(index = 0U; index < (uint32_t)TSI_GROUP_NUMS; index++) {
        if(0U != (channel_pins & (0x0F << (index * 4U)))) {
            groups |= (1U << index);
        }
    }

    return groups;
}

#endif /* GD32F350 */
