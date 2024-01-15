/*!
    \file    gd32f3x0_hal_fwdgt.c
    \brief   FWDGT driver

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
    \brief      initialize the FWDGT structure with the default values
    \param[in]  hal_struct_type: the argument could be selected from enumeration <hal_fwdgt_struct_type_enum>
    \param[in]  p_struct: pointer to FWDGT structure that contains the configuration information
    \param[out] none
    \retval     none
*/
void hal_fwdgt_struct_init(hal_fwdgt_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer [*p_struct] value is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    switch(hal_struct_type) {
    case HAL_FWDGT_INIT_STRUCT:
        /* initialize WWDGT initialization structure with the default values */
        ((hal_fwdgt_init_struct *)p_struct)->fwdgt_pre_select                        = FWDGT_PSC_DIV256;
        ((hal_fwdgt_init_struct *)p_struct)->wdgt_cnt_value                          = 0x7F;
        ((hal_fwdgt_init_struct *)p_struct)->fwdgt_cnt_reload_value                  = 0x7F;
        break;
    case HAL_FWDGT_DEV_STRUCT:
        /* initialize WWDGT DEV structure with the default values */
        ((hal_fwdgt_dev_struct *)p_struct)->state                                    = HAL_FWDGT_STATE_NONE;
        ((hal_fwdgt_dev_struct *)p_struct)->mutex                                    = HAL_MUTEX_UNLOCKED;
        break;
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      initialize FWDGT
    \param[in]  fwdgt_dev: FWDGT device information structure
                the structure is not necessary to be reconfigured after structrue initialization,
                the structure parameters altering is automatically configured by core
    \param[in]  p_fwdgt_init: pointer to a hal_fwdgt_init_struct structure which contains
                parameters for initialization of the FWDGT peripheral
                members of the structure and the member values are shown as below:
                fwdgt_pre_select:
                only one parameter can be selected which is shown as below:
      \arg        FWDGT_PSC_DIV4   FWDGT prescaler set to 4
      \arg        FWDGT_PSC_DIV8   FWDGT prescaler set to 8
      \arg        FWDGT_PSC_DIV16  FWDGT prescaler set to 16
      \arg        FWDGT_PSC_DIV32  FWDGT prescaler set to 32
      \arg        FWDGT_PSC_DIV64  FWDGT prescaler set to 64
      \arg        FWDGT_PSC_DIV128 FWDGT prescaler set to 128
      \arg        FWDGT_PSC_DIV256 FWDGT prescaler set to 256
               fwdgt_cnt_reload_value: 0x0 - 0xFFF
               wdgt_cnt_value: 0x0 - 0xFFF, window function is disabled when the value is 0xFFF
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_TIMEOUT, HAL_ERR_NONE,
                details refer to gd32f3x0_hal.h
*/
int32_t hal_fwdgt_init(hal_fwdgt_dev_struct *fwdgt_dev, hal_fwdgt_init_struct *p_fwdgt_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_fwdgt_init) {
        HAL_DEBUGE("pointer [p_fwdgt_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }

    if(p_fwdgt_init->wdgt_cnt_value < p_fwdgt_init->fwdgt_cnt_reload_value) {
        HAL_DEBUGE("fwdgt window value is smaller than reload value, it will lead to system reset!");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    fwdgt_dev->state = HAL_FWDGT_STATE_BUSY;
    /* enable IRC40K */
    hals_rcu_osci_on(RCU_IRC40K);
    /* wait till IRC40K is ready */
    hals_rcu_osci_stab_wait(RCU_IRC40K);

    hals_fwdgt_write_enable();

    if(ERROR == hals_fwdgt_config(p_fwdgt_init->fwdgt_cnt_reload_value,
                                  (uint8_t)p_fwdgt_init->fwdgt_pre_select)) {
        HAL_DEBUGE("fwdgt_config() timeout");
        return HAL_ERR_TIMEOUT;
    }

    if(ERROR == hals_fwdgt_window_value_config(p_fwdgt_init->wdgt_cnt_value)) {
        HAL_DEBUGE("fwdgt_window_value_config() timeout");
        return HAL_ERR_TIMEOUT;
    }

    hals_fwdgt_write_disable();

    return HAL_ERR_NONE;
}

/*!
    \brief      deinitialize FWDGT
    \param[in]  fwdgt_dev: FWDGT device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_fwdgt_deinit(hal_fwdgt_dev_struct *fwdgt_dev)
{
    /* enable IRC40K */
    hals_rcu_osci_on(RCU_IRC40K);
    /* disable IRC40K */
    hals_rcu_osci_off(RCU_IRC40K);
    fwdgt_dev->state = HAL_FWDGT_STATE_RESET;
}

/*!
    \brief      start FWDGT module function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_fwdgt_start(void)
{
    FWDGT_CTL = FWDGT_KEY_ENABLE;
}

/*!
    \brief      reload the counter of FWDGT
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_fwdgt_reload(void)
{
    FWDGT_CTL = FWDGT_KEY_RELOAD;
}

/*!
    \brief      enable write access to FWDGT_PSC and FWDGT_RLD and FWDGT_WND
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_fwdgt_write_enable(void)
{
    FWDGT_CTL = FWDGT_WRITEACCESS_ENABLE;
}

/*!
    \brief      disable write access to FWDGT_PSC,FWDGT_RLD and FWDGT_WND
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_fwdgt_write_disable(void)
{
    FWDGT_CTL = FWDGT_WRITEACCESS_DISABLE;
}

/*!
    \brief      configure the free watchdog timer counter window value
    \param[in]  wdgt_cnt_value: specify window value(0x0000 - 0x0FFF)
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus hals_fwdgt_window_value_config(uint32_t wdgt_cnt_value)
{
    uint32_t time_index = FWDGT_WND_TIMEOUT;
    uint32_t flag_status = RESET;

    /* enable write access to FWDGT_WND */
    FWDGT_CTL = FWDGT_WRITEACCESS_ENABLE;

    /* wait until the WUD flag to be reset */
    do {
        flag_status = FWDGT_STAT & FWDGT_STAT_WUD;
    } while((--time_index > 0U) && ((uint32_t)RESET != flag_status));

    if((uint32_t)RESET != flag_status) {
        return ERROR;
    }

    FWDGT_WND = WND_WND(wdgt_cnt_value);

    return SUCCESS;
}

/*!
    \brief      configure counter reload value, and prescaler divider value
    \param[in]  fwdgt_cnt_reload_value: specify reload value(0x0000 - 0x0FFF)
    \param[in]  fwdgt_pre_select: FWDGT prescaler value
                only one parameter can be selected which is shown as below:
      \arg        FWDGT_PSC_DIV4   FWDGT prescaler set to 4
      \arg        FWDGT_PSC_DIV8   FWDGT prescaler set to 8
      \arg        FWDGT_PSC_DIV16  FWDGT prescaler set to 16
      \arg        FWDGT_PSC_DIV32  FWDGT prescaler set to 32
      \arg        FWDGT_PSC_DIV64  FWDGT prescaler set to 64
      \arg        FWDGT_PSC_DIV128 FWDGT prescaler set to 128
      \arg        FWDGT_PSC_DIV256 FWDGT prescaler set to 256
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus hals_fwdgt_config(uint32_t fwdgt_cnt_reload_value, uint32_t fwdgt_pre_select)
{
    uint32_t timeout = FWDGT_PSC_TIMEOUT;
    uint32_t flag_status = RESET;

    /* enable write access to FWDGT_PSC,and FWDGT_RLD */
    FWDGT_CTL = FWDGT_WRITEACCESS_ENABLE;

    /* wait until the PUD flag to be reset */
    do {
        flag_status = FWDGT_STAT & FWDGT_STAT_PUD;
    } while((--timeout > 0U) && ((uint32_t)RESET != flag_status));

    if((uint32_t)RESET != flag_status) {
        return ERROR;
    }

    /* configure FWDGT */
    FWDGT_PSC = (uint32_t)fwdgt_pre_select;

    timeout = FWDGT_RLD_TIMEOUT;
    /* wait until the RUD flag to be reset */
    do {
        flag_status = FWDGT_STAT & FWDGT_STAT_RUD;
    } while((--timeout > 0U) && ((uint32_t)RESET != flag_status));

    if((uint32_t)RESET != flag_status) {
        return ERROR;
    }

    FWDGT_RLD = RLD_RLD(fwdgt_cnt_reload_value);

    /* reload the counter */
    FWDGT_CTL = FWDGT_KEY_RELOAD;

    return SUCCESS;
}

/*!
    \brief      get flag state of FWDGT
    \param[in]  flag: flag to get
                only one parameter can be selected which is shown as below:
      \arg        FWDGT_FLAG_PUD: a write operation to FWDGT_PSC register is on going
      \arg        FWDGT_FLAG_RUD: a write operation to FWDGT_RLD register is on going
      \arg        FWDGT_FLAG_WUD: a write operation to FWDGT_WND register is on going
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_fwdgt_flag_get(uint16_t flag)
{
    if(RESET != (FWDGT_STAT & flag)) {
        return SET;
    }
    return RESET;
}
