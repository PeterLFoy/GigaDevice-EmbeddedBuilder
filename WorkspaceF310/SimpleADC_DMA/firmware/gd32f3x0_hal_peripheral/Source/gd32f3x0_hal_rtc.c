/*!
    \file    gd32f3x0_hal_rtc.c
    \brief   RTC driver

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
    \brief      init RTC
    \param[in]  rtc_dev: RTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  rtc_init: the pointer of RTC calendar configure structure
                  rtc_year: 0~99
                  rtc_month:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_JAN: Janurary
      \arg          RTC_FEB: February
      \arg          RTC_MAR: March
      \arg          RTC_APR: April
      \arg          RTC_MAY: May
      \arg          RTC_JUN: June
      \arg          RTC_JUL: July
      \arg          RTC_AUG: August
      \arg          RTC_SEP: September
      \arg          RTC_OCT: October
      \arg          RTC_NOV: November
      \arg          RTC_DEC: December
                  rtc_date: 0~31
                  rtc_day_of_week:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_MONDAY: Monday
      \arg          RTC_TUESDAY: Tuesday
      \arg          RTC_WEDSDAY: Wednesday
      \arg          RTC_THURSDAY: Thursday
      \arg          RTC_FRIDAY: Friday
      \arg          RTC_SATURDAY: Saturday
      \arg          RTC_SUNDAY: Sunday
                  rtc_hour: 0 - 24 or 1 - 12
                  rtc_minute: 0 - 59
                  rtc_second: 0 - 59
                  rtc_am_pm:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_AM: AM format
      \arg          RTC_PM: PM format
                  rtc_daylight_saving:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_DAYLIGHT_SAVING_NONE: no daylight saving
      \arg          RTC_DAYLIGHT_SAVING_ADD_1H: add 1 hour(summer time change)
      \arg          RTC_DAYLIGHT_SAVING_SUB_1H: subtract 1 hour(winter time change)
                  rtc_clock_format:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_24H_FORMAT: 24-hour format
      \arg          RTC_12H_FORMAT: 12-hour format
                  rtc_psc_factor_s: 0x0 - 0x7FFF
                  rtc_psc_factor_a: 0x0 - 0x7F
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_init(hal_rtc_dev_struct *rtc_dev, hal_rtc_init_struct *rtc_init)
{
    ErrStatus error_status = ERROR;
    uint32_t reg_time = 0x00U, reg_date = 0x00U;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == rtc_dev) || (NULL == rtc_init)) {
        HAL_DEBUGE("pointer [*rtc_dev] or pointer [*rtc_init] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }

    /* check parameter rtc_init->rtc_year */
    if(rtc_init->rtc_year > 99) {
        HAL_DEBUGE("parameter [rtc_init->rtc_year] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check parameter rtc_init->rtc_date */
    if((rtc_init->rtc_date < 1U) || (rtc_init->rtc_date > 31U)) {
        HAL_DEBUGE("parameter [rtc_init->rtc_date] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check parameter rtc_init->rtc_clock_format */
    if((RTC_24H_FORMAT != rtc_init->rtc_clock_format) && (RTC_12H_FORMAT != rtc_init->rtc_clock_format)) {
        HAL_DEBUGE("parameter [rtc_init->rtc_clock_format] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check parameter rtc_initpara_struct->rtc_hour, and rtc_initpara_struct->rtc_am_pm */
    if(RTC_12H_FORMAT == rtc_init->rtc_clock_format) {

        if((rtc_init->rtc_hour < 1U) || (rtc_init->rtc_hour > 12U)) {
            HAL_DEBUGE("parameter [rtc_init->rtc_hour] value is invalid");
            return HAL_ERR_VAL;
        }
        if((RTC_AM != rtc_init->rtc_am_pm) && (RTC_PM != rtc_init->rtc_am_pm)) {
            HAL_DEBUGE("parameter [rtc_init->rtc_am_pm] value is invalid");
            return HAL_ERR_VAL;
        }
    } else {
        if(rtc_init->rtc_hour > 23U) {
            HAL_DEBUGE("parameter [rtc_init->rtc_hour] value is invalid");
            return HAL_ERR_VAL;
        }
        rtc_init->rtc_am_pm = RTC_AM;
    }

    /* check parameter rtc_initpara_struct->rtc_minute */
    if(rtc_init->rtc_minute > 59) {
        HAL_DEBUGE("parameter [rtc_init->rtc_minute] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check parameter rtc_initpara_struct->rtc_second */
    if(rtc_init->rtc_second > 59) {
        HAL_DEBUGE("parameter [rtc_init->rtc_second] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check parameter rtc_initpara_struct->rtc_factor_syn */
    if(rtc_init->rtc_psc_factor_s > 0x7FFF) {
        HAL_DEBUGE("parameter [rtc_init->rtc_psc_factor_s] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check parameter rtc_initpara_struct->rtc_factor_asyn */
    if(rtc_init->rtc_psc_factor_a > 0x7F) {
        HAL_DEBUGE("parameter [rtc_init->rtc_psc_factor_a] value is invalid");
        return HAL_ERR_VAL;
    }

#endif /* 1 = HAL_PARAMETER_CHECK */
    rtc_dev->state = HAL_RTC_STATE_BUSY;

    /* calendar config */
    /* 1st: disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;
    /* 2nd: enter init mode */
    error_status = hals_rtc_init_mode_enter();
    if(ERROR != error_status) {
        RTC_PSC = (uint32_t)(PSC_FACTOR_A(rtc_init->rtc_psc_factor_a) | \
                             PSC_FACTOR_S(rtc_init->rtc_psc_factor_s));
        RTC_CTL &= (uint32_t)(~RTC_CTL_CS);
        RTC_CTL |=  rtc_init->rtc_clock_format;
        reg_date = (DATE_YR(hals_rtc_normal_2_bcd(rtc_init->rtc_year)) | \
                    DATE_DOW(hals_rtc_normal_2_bcd(rtc_init->rtc_day_of_week)) | \
                    DATE_MON(hals_rtc_normal_2_bcd(rtc_init->rtc_month)) | \
                    DATE_DAY(hals_rtc_normal_2_bcd(rtc_init->rtc_date)));

        reg_time = (rtc_init->rtc_am_pm | \
                    TIME_HR(hals_rtc_normal_2_bcd(rtc_init->rtc_hour)) | \
                    TIME_MN(hals_rtc_normal_2_bcd(rtc_init->rtc_minute)) | \
                    TIME_SC(hals_rtc_normal_2_bcd(rtc_init->rtc_second)));
        RTC_TIME = (uint32_t)reg_time;
        RTC_DATE = (uint32_t)reg_date;
        RTC_TIME &= (uint32_t)(~RTC_PM);
        RTC_TIME |=  rtc_init->rtc_am_pm;
        RTC_CTL &= (uint32_t)(~(RTC_DAYLIGHT_SAVING_ADD_1H | RTC_DAYLIGHT_SAVING_SUB_1H));
        RTC_CTL |= rtc_init->rtc_daylight_saving;

        /* 3rd: exit init mode */
        hals_rtc_init_mode_exit();
        /* 4th: wait the RSYNF flag to set */
        error_status = hals_rtc_register_sync_wait();
    }
    /* 5th: enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
    if(ERROR == error_status) {
        return HAL_ERR_BUSY;
    }

    /* change RTC error state */
    rtc_dev->error_state = HAL_RTC_ERROR_NONE;
    /* change RTC state */
    rtc_dev->state = HAL_RTC_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      configure RTC alarm output
    \param[in]  rtc_dev: RTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  hal_rtc_alarm_output_config_struct: the pointer of RTC alarm output configure structure
                  rtc_alarm_output_polarity:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_HIGH: enable alarm flag output with high level
      \arg          RTC_ALARM_LOW: enable alarm flag output with low level
                  rtc_alarm_output_type:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_OUTPUT_OD: RTC alarm output open-drain mode
      \arg          RTC_ALARM_OUTPUT_PP: RTC alarm output push-pull mode
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_alarm_output_config(hal_rtc_dev_struct *rtc_dev,
                                    hal_rtc_alarm_output_config_struct *rtc_alarm_output_config)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == rtc_dev) || (NULL == rtc_alarm_output_config)) {
        HAL_DEBUGE("pointer [*rtc_dev] or pointer [*rtc_alarm_output_config] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    rtc_dev->state = HAL_RTC_STATE_BUSY;

    /* alram output config */
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;
    RTC_CTL &= (uint32_t)~(RTC_CTL_OS | RTC_CTL_OPOL);

    RTC_CTL |= (uint32_t)(rtc_alarm_output_config->rtc_alarm_output_polarity);

    /* alarm output */
    RTC_TAMP &= (uint32_t)~(RTC_TAMP_PC13VAL);
    RTC_TAMP |= (uint32_t)(rtc_alarm_output_config->rtc_alarm_output_type);

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
    /* change RTC error state */
    rtc_dev->error_state = HAL_RTC_ERROR_NONE;
    /* change RTC state */
    rtc_dev->state = HAL_RTC_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      configure RTC alarm
    \param[in]  rtc_dev: RTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  hal_rtc_alarm_config_struct: the pointer of RTC alarm configure structure
                  rtc_alarm_am_pm:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_AM: AM format
      \arg          RTC_ALARM_PM: PM format
                  rtc_alarm_weekday_mask:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_WEEKDAY_MASK_DISABLE: RTC alarm weekday no mask
      \arg          RTC_ALARM_WEEKDAY_MASK_ENABLE: RTC alarm weekday mask
                  rtc_alarm_hour_mask:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_HOUR_MASK_DISABLE: RTC alarm hour no mask
      \arg          RTC_ALARM_HOUR_MASK_ENABLE: RTC alarm hour mask
                  rtc_alarm_minute_mask:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_MINUTE_MASK_DISABLE: RTC alarm minute no mask
      \arg          RTC_ALARM_MINUTE_MASK_ENABLE: RTC alarm minute mask
                  rtc_alarm_second_mask:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_SECOND_MASK_DISABLE: RTC alarm second no mask
      \arg          RTC_ALARM_SECOND_MASK_ENABLE: RTC alarm second mask
                  rtc_alarm_subsecond_mask:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_MASKSSC_0_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_1_14: RTC alarm second mask
      \arg          RTC_MASKSSC_2_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_3_14: RTC alarm second mask
      \arg          RTC_MASKSSC_4_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_5_14: RTC alarm second mask
      \arg          RTC_MASKSSC_6_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_7_14: RTC alarm second mask
      \arg          RTC_MASKSSC_8_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_9_14: RTC alarm second mask
      \arg          RTC_MASKSSC_10_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_11_14: RTC alarm second mask
      \arg          RTC_MASKSSC_12_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_13_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_14: RTC alarm second mask
      \arg          RTC_MASKSSC_NONE: RTC alarm second mask
                  rtc_weekday_or_date:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_SELECT_WEEKDAY: RTC alarm select weekday
      \arg          RTC_ALARM_SELECT_DATE: RTC alarm select date
                  rtc_alarm_day: 1 - 31 or below
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_MONDAY: RTC alarm Monday
      \arg          RTC_ALARM_TUESDAY: RTC alarm Tuesday
      \arg          RTC_ALARM_WEDSDAY: RTC alarm Wednesday
      \arg          RTC_ALARM_THURSDAY: RTC alarm Thursday
      \arg          RTC_ALARM_FRIDAY: RTC alarm Friday
      \arg          RTC_ALARM_SATURDAY: RTC alarm Saturday
      \arg          RTC_ALARM_SUNDAY: RTC alarm Sunday
                  rtc_hour: 0 - 24 or 1 - 12
                  rtc_minute: 0 - 59
                  rtc_second: 0 - 59
                  rtc_alarm_subsecond: 0x00~0x7FFF
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_alarm_config(hal_rtc_dev_struct *rtc_dev, hal_rtc_alarm_config_struct *rtc_alarm_config)
{
    uint32_t reg_alrm0td = 0x00U;
    uint32_t reg_alrm0ss = 0x00U;
#if (1 == HAL_PARAMETER_CHECK)
    uint32_t rtc_display_format = 0U;
    /* check the parameters */
    if((NULL == rtc_dev) || (NULL == rtc_alarm_config)) {
        HAL_DEBUGE("pointer [*rtc_dev] or pointer [*rtc_alarm_config] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }

    /* check parameter rtc_alarm_time->rtc_alarm_mask */
    if(0x80808080U != ((rtc_alarm_config->rtc_alarm_hour_mask | rtc_alarm_config->rtc_alarm_minute_mask |
                        rtc_alarm_config->rtc_alarm_second_mask) | 0x80808080U)) {
        HAL_DEBUGE("parameter [rtc_alarm_time->rtc_alarm_mask] value is invalid");
        return HAL_ERR_VAL;
    }

    rtc_display_format = RTC_CTL & RTC_CTL_CS;
    /* check parameter rtc_alarm_config->rtc_alarm_hour, and rtc_alarm_config->rtc_alarm_am_pm */
    if(0U != rtc_display_format) {

        if((rtc_alarm_config->rtc_alarm_hour < 1U) || (rtc_alarm_config->rtc_alarm_hour > 12U)) {
            HAL_DEBUGE("parameter [rtc_alarm_config->rtc_alarm_hour] value is invalid");
            return HAL_ERR_VAL;
        }
        if((RTC_ALARM_AM != rtc_alarm_config->rtc_alarm_am_pm) && (RTC_ALARM_PM != rtc_alarm_config->rtc_alarm_am_pm)) {
            HAL_DEBUGE("parameter [rtc_alarm_config->rtc_alarm_am_pm] value is invalid");
            return HAL_ERR_VAL;
        }
    } else {
        if(rtc_alarm_config->rtc_alarm_hour > 23U) {
            HAL_DEBUGE("parameter [rtc_alarm_config->rtc_alarm_hour] value is invalid");
            return HAL_ERR_VAL;
        }
        rtc_alarm_config->rtc_alarm_am_pm = RTC_ALARM_AM;
    }

    /* check parameter rtc_alarm_time->rtc_alarm_minute */
    if(rtc_alarm_config->rtc_alarm_minute > 59) {
        HAL_DEBUGE("parameter [rtc_alarm_config->rtc_alarm_minute] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check parameter rtc_alarm_time->rtc_alarm_second */
    if(rtc_alarm_config->rtc_alarm_second > 59) {
        HAL_DEBUGE("parameter [rtc_alarm_config->rtc_alarm_second] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check parameter rtc_alarm_time->rtc_weekday_or_date, and rtc_alarm_time->rtc_alarm_day */
    if(RTC_ALARM_SELECT_DATE == rtc_alarm_config->rtc_weekday_or_date) {
        if((rtc_alarm_config->rtc_alarm_day < 1U) || (rtc_alarm_config->rtc_alarm_day > 31U)) {
            HAL_DEBUGE("parameter [rtc_alarm_config->rtc_weekday_or_date] value is invalid");
            return HAL_ERR_VAL;
        }
    } else if(RTC_ALARM_SELECT_WEEKDAY == rtc_alarm_config->rtc_weekday_or_date) {
        if((rtc_alarm_config->rtc_alarm_day < 1U) || (rtc_alarm_config->rtc_alarm_day > 7U)) {
            HAL_DEBUGE("parameter [rtc_alarm_config->rtc_weekday_or_date] value is invalid");
            return HAL_ERR_VAL;
        }
    } else {
        HAL_DEBUGE("parameter [rtc_alarm_config->rtc_weekday_or_date] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check parameter rtc_alarm_time->rtc_alarm_subsecond_mask */
    if(RTC_ALRM0SS_MASKSSC != (rtc_alarm_config->rtc_alarm_subsecond_mask | RTC_ALRM0SS_MASKSSC)) {
        HAL_DEBUGE("parameter [rtc_alarm_config->rtc_alarm_subsecond_mask] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    rtc_dev->state = HAL_RTC_STATE_BUSY;

    /* alarm config */
    reg_alrm0td = (rtc_alarm_config->rtc_alarm_am_pm | \
                   rtc_alarm_config->rtc_alarm_weekday_mask | \
                   rtc_alarm_config->rtc_alarm_hour_mask | \
                   rtc_alarm_config->rtc_alarm_minute_mask | \
                   rtc_alarm_config->rtc_alarm_second_mask | \
                   rtc_alarm_config->rtc_weekday_or_date | \
                   ALRM0TD_DAY(hals_rtc_normal_2_bcd(rtc_alarm_config->rtc_alarm_day)) | \
                   ALRM0TD_HR(hals_rtc_normal_2_bcd(rtc_alarm_config->rtc_alarm_hour)) | \
                   ALRM0TD_MN(hals_rtc_normal_2_bcd(rtc_alarm_config->rtc_alarm_minute)) | \
                   ALRM0TD_SC(hals_rtc_normal_2_bcd(rtc_alarm_config->rtc_alarm_second)));
    reg_alrm0ss = rtc_alarm_config->rtc_alarm_subsecond_mask | \
                  SS_SSC(rtc_alarm_config->rtc_alarm_subsecond);

    /* disable RTC alarm */
    if(ERROR == hals_rtc_alarm_disable()) {
        HAL_DEBUGE("disable RTC alarm error");
        return HAL_ERR_TIMEOUT;
    }

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_ALRM0TD = (uint32_t)reg_alrm0td;
    RTC_ALRM0SS = reg_alrm0ss;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
    /* change RTC error state */
    rtc_dev->error_state = HAL_RTC_ERROR_NONE;
    /* change RTC state */
    rtc_dev->state = HAL_RTC_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      configure RTC tamper
    \param[in]  rtc_dev: RTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  hal_rtc_alarm_config_struct: the pointer of RTC alarm configure structure
                  rtc_tamper_filter:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_FLT_EDGE: detecting tamper event using edge mode. precharge duration is disabled automatically
      \arg          RTC_FLT_2S: detecting tamper event using level mode.2 consecutive valid level samples will make a effective tamper event
      \arg          RTC_FLT_4S: detecting tamper event using level mode.4 consecutive valid level samples will make an effective tamper event
      \arg          RTC_FLT_8S: detecting tamper event using level mode.8 consecutive valid level samples will make a effective tamper event
                  rtc_tamper_sample_frequency:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_FREQ_DIV32768: sample once every 32768 RTCCLK(1Hz if RTCCLK=32.768KHz)
      \arg          RTC_FREQ_DIV16384: sample once every 16384 RTCCLK(2Hz if RTCCLK=32.768KHz)
      \arg          RTC_FREQ_DIV8192: sample once every 8192 RTCCLK(4Hz if RTCCLK=32.768KHz)
      \arg          RTC_FREQ_DIV4096: sample once every 4096 RTCCLK(8Hz if RTCCLK=32.768KHz)
      \arg          RTC_FREQ_DIV2048: sample once every 2048 RTCCLK(16Hz if RTCCLK=32.768KHz)
      \arg          RTC_FREQ_DIV1024: sample once every 1024 RTCCLK(32Hz if RTCCLK=32.768KHz)
      \arg          RTC_FREQ_DIV512: sample once every 512 RTCCLK(64Hz if RTCCLK=32.768KHz)
      \arg          RTC_FREQ_DIV256: sample once every 256 RTCCLK(128Hz if RTCCLK=32.768KHz)
                  rtc_tamper_precharge_enable:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_PRCH_DISALE:  RTC tamper precharge feature disable during a voltage level detection
      \arg          RTC_PRCH_ENALE: RTC tamper precharge feature enable during a voltage level detection
                  rtc_tamper_precharge_time:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_PRCH_1C: 1 RTC clock prechagre time before each sampling
      \arg          RTC_PRCH_2C: 2 RTC clock prechagre time before each sampling
      \arg          RTC_PRCH_4C: 4 RTC clock prechagre time before each sampling
      \arg          RTC_PRCH_8C: 8 RTC clock prechagre time before each sampling
                  rtc_tamper_with_timestamp:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_TAMPER_TIMESTAMP_DISALE: RTC tamper time-stamp feature disable
      \arg          RTC_TAMPER_TIMESTAMP_ENALE: RTC tamper time-stamp feature enable
                  rtc_tamper0_trigger:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_TAMPER_TRIGGER_EDGE_RISING: tamper detection is in rising edge mode
      \arg          RTC_TAMPER_TRIGGER_EDGE_FALLING: tamper detection is in falling edge mode
      \arg          RTC_TAMPER_TRIGGER_LEVEL_LOW: tamper detection is in low level mode
      \arg          RTC_TAMPER_TRIGGER_LEVEL_HIGH: tamper detection is in high level mode
                  rtc_tamper1_trigger:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_TAMPER_TRIGGER_EDGE_RISING: tamper detection is in rising edge mode
      \arg          RTC_TAMPER_TRIGGER_EDGE_FALLING: tamper detection is in falling edge mode
      \arg          RTC_TAMPER_TRIGGER_LEVEL_LOW: tamper detection is in low level mode
      \arg          RTC_TAMPER_TRIGGER_LEVEL_HIGH: tamper detection is in high level mode
                  rtc_tamper0_source:
      \arg          ENABLE: enable TAMPER0
      \arg          DISABLE: disable TAMPER0
                  rtc_tamper1_source:
      \arg          ENABLE: enable TAMPER1
      \arg          DISABLE: disable TAMPER1
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_tamp_config(hal_rtc_dev_struct *rtc_dev, hal_rtc_tamper_config_struct *rtc_tamper_config)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == rtc_dev) || (NULL == rtc_tamper_config)) {
        HAL_DEBUGE("pointer [*rtc_dev] or pointer [*rtc_tamper_config] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }

#endif /* 1 = HAL_PARAMETER_CHECK */
    rtc_dev->state = HAL_RTC_STATE_BUSY;

    /* tamper config */
    /* tamper filter must be used when the tamper source is voltage level detection */
    RTC_TAMP &= (uint32_t)~(RTC_TAMP_DISPU | RTC_TAMP_PRCH | RTC_TAMP_FREQ | RTC_TAMP_FLT | RTC_TAMP_TPTS | RTC_TAMP_TP0EG |
                            RTC_TAMP_TP1EG | RTC_TAMPER0 | RTC_TAMPER1);
    RTC_TAMP |= (rtc_tamper_config->rtc_tamper_filter | \
                 rtc_tamper_config->rtc_tamper_sample_frequency | \
                 rtc_tamper_config->rtc_tamper_precharge_enable | \
                 rtc_tamper_config->rtc_tamper_precharge_time | \
                 rtc_tamper_config->rtc_tamper_with_timestamp | \
                 (rtc_tamper_config->rtc_tamper0_trigger << 1) | \
                 (rtc_tamper_config->rtc_tamper1_trigger << 4)
                );

    /* change RTC error state */
    rtc_dev->error_state = HAL_RTC_ERROR_NONE;
    /* change RTC state */
    rtc_dev->state = HAL_RTC_STATE_READY;

    if(rtc_tamper_config->rtc_tamper0_source == ENABLE) {
        /* enable tamper0 */
        RTC_TAMP |= RTC_TAMPER0;
    }

    if(rtc_tamper_config->rtc_tamper1_source == ENABLE) {
        /* enable tamper1 */
        RTC_TAMP |= RTC_TAMPER1;
    }
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      configure RTC time stamp
    \param[in]  rtc_dev: RTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  rtc_timestamp_config: rtc timestamp configuration
                  only one parameter can be selected which is shown as below:
      \arg          RTC_TIMESTAMP_RISING_EDGE: rising edge is valid event edge for time-stamp event
      \arg          RTC_TIMESTAMP_FALLING_EDGE: falling edge is valid event edge for time-stamp event
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_timestamp_config(hal_rtc_dev_struct *rtc_dev, uint32_t rtc_timestamp_config)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == rtc_dev) {
        HAL_DEBUGE("pointer [*rtc_dev] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    rtc_dev->state = HAL_RTC_STATE_BUSY;

    /* tamper config */
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;
    /* clear the bits to be configured in RTC_CTL */
    RTC_CTL &= (uint32_t)(~(RTC_CTL_TSEG));
    /* new configuration */
    RTC_CTL |= (uint32_t)(rtc_timestamp_config | RTC_CTL_TSEN);

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
    /* change RTC error state */
    rtc_dev->error_state = HAL_RTC_ERROR_NONE;
    /* change RTC state */
    rtc_dev->state = HAL_RTC_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      configure RTC calibration
    \param[in]  rtc_dev: RTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  rtc_calib_config: rtc calibration configuration
                  only one parameter can be selected which is shown as below:
      \arg          RTC_CALIBRATION_NONE: calibration output none
      \arg          RTC_CALIBRATION_OUTPUT_512HZ: calibration output of 512Hz is enable
      \arg          RTC_CALIBRATION_OUTPUT_1HZ: calibration output of 1Hz is enable
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_calib_config(hal_rtc_dev_struct *rtc_dev, uint8_t rtc_calib_config)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == rtc_dev) {
        HAL_DEBUGE("pointer [*rtc_dev] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    rtc_dev->state = HAL_RTC_STATE_BUSY;

    /* tamper config */
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    switch(rtc_calib_config) {
    case RTC_CALIBRATION_NONE:
        /* clear the bits to be configured in RTC_CTL */
        RTC_CTL &= (uint32_t)(~(RTC_CTL_COEN));
        break;
    case RTC_CALIBRATION_OUTPUT_512HZ:
        /* clear the bits to be configured in RTC_CTL */
        RTC_CTL &= (uint32_t)(~(RTC_CTL_COS));
        RTC_CTL |= (uint32_t)((RTC_CTL_COEN));
        break;
    case RTC_CALIBRATION_OUTPUT_1HZ:
        /* clear the bits to be configured in RTC_CTL */
        RTC_CTL |= (uint32_t)(RTC_CTL_COEN | RTC_CTL_COS);
        break;
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
    /* change RTC error state */
    rtc_dev->error_state = HAL_RTC_ERROR_NONE;
    /* change RTC state */
    rtc_dev->state = HAL_RTC_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      configure RTC reference clock detection function
    \param[in]  rtc_dev: RTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  refclock_detection: rtc reference clock detection
                  only one parameter can be selected which is shown as below:
      \arg          DISABLE: disable RTC reference clock detection function
      \arg          ENABLE: enable RTC reference clock detection function
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_refclock_detection_config(hal_rtc_dev_struct *rtc_dev, ControlStatus refclock_detection)
{
    ErrStatus error_status = ERROR;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == rtc_dev) {
        HAL_DEBUGE("pointer [*rtc_dev] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    rtc_dev->state = HAL_RTC_STATE_BUSY;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* enter init mode */
    error_status = hals_rtc_init_mode_enter();

    if(ERROR != error_status) {
        if(ENABLE == refclock_detection) {
            RTC_CTL |= (uint32_t)RTC_CTL_REFEN;
        } else {
            RTC_CTL &= (uint32_t)~RTC_CTL_REFEN;
        }
        /* exit init mode */
        hals_rtc_init_mode_exit();
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    if(ERROR != error_status) {
        /* change RTC error state */
        rtc_dev->error_state = HAL_RTC_ERROR_NONE;
        /* change RTC state */
        rtc_dev->state = HAL_RTC_STATE_READY;
        /* return function state */
        return HAL_ERR_NONE;
    } else {
        /* change RTC error state */
        rtc_dev->error_state = HAL_RTC_ERROR_SYSTEM;
        /* change RTC state */
        rtc_dev->state = HAL_RTC_STATE_ERROR;
        /* return function state */
        return HAL_ERR_HARDWARE;
    }
}

/*!
    \brief      initialize the RTC structure with the default values
    \param[in]  hal_struct_type: the argument could be selected from enumeration <hal_rtc_struct_type_enum>
    \param[out]  p_struct: pointer to RTC structure that contains the configuration information
    \retval     none
*/
void hal_rtc_struct_init(hal_rtc_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer [*p_struct] value is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_RTC_INIT_STRUCT:
        /* RTC init struct */
        ((hal_rtc_init_struct *)p_struct)->rtc_year                             = 0x00;
        ((hal_rtc_init_struct *)p_struct)->rtc_month                            = RTC_JAN;
        ((hal_rtc_init_struct *)p_struct)->rtc_date                             = 0x1;
        ((hal_rtc_init_struct *)p_struct)->rtc_day_of_week                      = RTC_MONDAY;
        ((hal_rtc_init_struct *)p_struct)->rtc_hour                             = 0x0;
        ((hal_rtc_init_struct *)p_struct)->rtc_minute                           = 0x0;
        ((hal_rtc_init_struct *)p_struct)->rtc_second                           = 0x0;
        ((hal_rtc_init_struct *)p_struct)->rtc_am_pm                            = RTC_AM;
        ((hal_rtc_init_struct *)p_struct)->rtc_daylight_saving                  = RTC_DAYLIGHT_SAVING_NONE;
        ((hal_rtc_init_struct *)p_struct)->rtc_clock_format                     = RTC_24H_FORMAT;
        ((hal_rtc_init_struct *)p_struct)->rtc_psc_factor_s                     = 0x18F;
        ((hal_rtc_init_struct *)p_struct)->rtc_psc_factor_a                     = 0x63;
        break;
    case HAL_RTC_ALARM_OUTPUT_CONFIG_STRUCT:
        /* RTC alarm out config struct */
        ((hal_rtc_alarm_output_config_struct *)p_struct)->rtc_alarm_output_polarity        = RTC_ALARM_HIGH;
        ((hal_rtc_alarm_output_config_struct *)p_struct)->rtc_alarm_output_polarity        = RTC_ALARM_OUTPUT_OD;
        break;
    case HAL_RTC_ALARM_CONFIG_STRUCT:
        /* RTC alarm config struct */
        ((hal_rtc_alarm_config_struct *)p_struct)->rtc_alarm_am_pm                         = RTC_ALARM_AM;
        ((hal_rtc_alarm_config_struct *)p_struct)->rtc_alarm_weekday_mask                  = RTC_ALARM_WEEKDAY_MASK_DISABLE;
        ((hal_rtc_alarm_config_struct *)p_struct)->rtc_alarm_hour_mask                     = RTC_ALARM_HOUR_MASK_DISABLE;
        ((hal_rtc_alarm_config_struct *)p_struct)->rtc_alarm_minute_mask                   = RTC_ALARM_MINUTE_MASK_DISABLE;
        ((hal_rtc_alarm_config_struct *)p_struct)->rtc_alarm_second_mask                   = RTC_ALARM_SECOND_MASK_DISABLE;
        ((hal_rtc_alarm_config_struct *)p_struct)->rtc_alarm_subsecond_mask                = RTC_MASKSSC_0_14;
        ((hal_rtc_alarm_config_struct *)p_struct)->rtc_weekday_or_date                     = RTC_ALARM_SELECT_WEEKDAY;
        ((hal_rtc_alarm_config_struct *)p_struct)->rtc_alarm_day                           = RTC_ALARM_MONDAY;
        ((hal_rtc_alarm_config_struct *)p_struct)->rtc_alarm_hour                          = 0x0;
        ((hal_rtc_alarm_config_struct *)p_struct)->rtc_alarm_minute                        = 0x0;
        ((hal_rtc_alarm_config_struct *)p_struct)->rtc_alarm_second                        = 0x0;
        ((hal_rtc_alarm_config_struct *)p_struct)->rtc_alarm_subsecond                     = 0x0;
        break;
    case HAL_RTC_TAMPER_CONFIG_STRUCT:
        /* RTC alarm config struct */
        ((hal_rtc_tamper_config_struct *)p_struct)->rtc_tamper_filter                      = RTC_FLT_EDGE;
        ((hal_rtc_tamper_config_struct *)p_struct)->rtc_tamper_sample_frequency            = RTC_FREQ_DIV32768;
        ((hal_rtc_tamper_config_struct *)p_struct)->rtc_tamper_precharge_time              = RTC_PRCH_1C;
        ((hal_rtc_tamper_config_struct *)p_struct)->rtc_tamper_precharge_enable            = RTC_PRCH_DISALE;
        ((hal_rtc_tamper_config_struct *)p_struct)->rtc_tamper_with_timestamp              = RTC_TAMPER_TIMESTAMP_DISALE;
        ((hal_rtc_tamper_config_struct *)p_struct)->rtc_tamper0_trigger                    = RTC_TAMPER_TRIGGER_EDGE_RISING;
        ((hal_rtc_tamper_config_struct *)p_struct)->rtc_tamper1_trigger                    = RTC_TAMPER_TRIGGER_EDGE_RISING;
        ((hal_rtc_tamper_config_struct *)p_struct)->rtc_tamper0_source                     = DISABLE;
        ((hal_rtc_tamper_config_struct *)p_struct)->rtc_tamper1_source                     = ENABLE;
        break;
    case HAL_RTC_TIMESTAMP_STRUCT:
        /* RTC timestamp config struct */
        ((hal_rtc_timestamp_struct *)p_struct)->rtc_timestamp_month = RTC_JAN;
        ((hal_rtc_timestamp_struct *)p_struct)->rtc_timestamp_date = 0x1;
        ((hal_rtc_timestamp_struct *)p_struct)->rtc_timestamp_weekday = RTC_MONDAY;
        ((hal_rtc_timestamp_struct *)p_struct)->rtc_timestamp_hour = 0x0;
        ((hal_rtc_timestamp_struct *)p_struct)->rtc_timestamp_minute = 0x0;
        ((hal_rtc_timestamp_struct *)p_struct)->rtc_timestamp_second = 0x0;
        ((hal_rtc_timestamp_struct *)p_struct)->rtc_timestamp_subsecond = 0x0;
        ((hal_rtc_timestamp_struct *)p_struct)->rtc_am_pm = RTC_AM;
        break;
    case HAL_RTC_IRQ_STRUCT:
        /* initialize RTC initialization structure with the default values */
        ((hal_rtc_irq_struct *)p_struct)->rtc_timestamp_handle                             = NULL;
        ((hal_rtc_irq_struct *)p_struct)->rtc_alarm_handle                                 = NULL;
        ((hal_rtc_irq_struct *)p_struct)->rtc_tamper0_handle                                 = NULL;
        ((hal_rtc_irq_struct *)p_struct)->rtc_tamper1_handle                                 = NULL;
        break;
    case HAL_RTC_DEV_STRUCT:
        /* initialize RTC initialization structure with the default values */
        ((hal_rtc_dev_struct *)p_struct)->rtc_irq.rtc_timestamp_handle                     = NULL;
        ((hal_rtc_dev_struct *)p_struct)->rtc_irq.rtc_alarm_handle                         = NULL;
        ((hal_rtc_dev_struct *)p_struct)->rtc_irq.rtc_tamper0_handle                         = NULL;
        ((hal_rtc_dev_struct *)p_struct)->rtc_irq.rtc_tamper1_handle                         = NULL;
        ((hal_rtc_dev_struct *)p_struct)->error_state                                      = HAL_RTC_ERROR_NONE;
        ((hal_rtc_dev_struct *)p_struct)->state                                            = HAL_RTC_STATE_NONE;
        ((hal_rtc_dev_struct *)p_struct)->mutex                                            = HAL_MUTEX_UNLOCKED;
        ((hal_rtc_dev_struct *)p_struct)->priv                                             = NULL;
        break;
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize RTC device structure and init structure
    \param[out]  rtc_dev: RTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_deinit(hal_rtc_dev_struct *rtc_dev)
{
    ErrStatus error_status = ERROR;
    int32_t ret_status = HAL_ERR_TIMEOUT;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == rtc_dev) {
        HAL_DEBUGE("pointer [rtc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    rtc_dev->state = HAL_RTC_STATE_BUSY;
    /* deinit RTC */

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;
    /* enter init mode */
    error_status = hals_rtc_init_mode_enter();
    if(ERROR != error_status) {
        /* before reset RTC_TIME and RTC_DATE, BPSHAD bit in RTC_CTL should be reset as the condition.
           in order to read calendar from shadow register, not the real registers being reset */
        RTC_TIME = RTC_REGISTER_RESET;
        RTC_DATE = RTC_DATE_RESET;

        RTC_PSC = RTC_PSC_RESET;

        /* reset RTC_STAT register, also exit init mode.
           at the same time, RTC_STAT_SOPF bit is reset, as the condition to reset RTC_SHIFTCTL register later */
        RTC_STAT = RTC_STAT_RESET;

        /* to write RTC_ALRM0SS register, ALRM0EN bit in RTC_CTL register should be reset as the condition */
        RTC_ALRM0TD = RTC_REGISTER_RESET;
        RTC_ALRM0SS = RTC_REGISTER_RESET;

        /* reset RTC_SHIFTCTL and RTC_HRFC register, this can be done without the init mode */
        RTC_SHIFTCTL = RTC_REGISTER_RESET;
        RTC_HRFC = RTC_REGISTER_RESET;

        ret_status = hals_rtc_register_sync_wait();
        if(HAL_ERR_TIMEOUT == ret_status) {
            HAL_DEBUGE("rtc register synchronized with APB clock timout");
            return HAL_ERR_TIMEOUT;
        }
    } else {
        /* enable the write protection */
        RTC_WPK = RTC_LOCK_KEY;

        HAL_DEBUGE("rtc enter init mode timout");
        return HAL_ERR_TIMEOUT;
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    /* change RTC error state and state */
    rtc_dev->error_state = HAL_RTC_ERROR_NONE;
    rtc_dev->state = HAL_RTC_STATE_NONE;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      enbale RTC interrupt
    \param[in]  rtc_dev: RTC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: RTC interrupt callback function structure
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_interrupt_enable(hal_rtc_dev_struct *rtc_dev, hal_rtc_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == rtc_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [rtc_dev] or pointer [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* lock RTC */
    HAL_LOCK(rtc_dev);
    rtc_dev->state = HAL_RTC_STATE_BUSY;
    rtc_dev->rtc_irq.rtc_timestamp_handle = p_irq->rtc_timestamp_handle;
    rtc_dev->rtc_irq.rtc_alarm_handle = p_irq->rtc_alarm_handle;
    rtc_dev->rtc_irq.rtc_tamper0_handle = p_irq->rtc_tamper0_handle;
    rtc_dev->rtc_irq.rtc_tamper1_handle = p_irq->rtc_tamper1_handle;

    /* clear the specified RTC interrupt flag and enable RTC interrupt */
    if(NULL != rtc_dev->rtc_irq.rtc_timestamp_handle) {
        hals_rtc_flag_clear(RTC_FLAG_TIMESTAMP);
        hals_rtc_interrupt_enable(RTC_INT_TIMESTAMP);
    } else{
        hals_rtc_interrupt_disable(RTC_INT_TIMESTAMP);
    }

    if(NULL != rtc_dev->rtc_irq.rtc_alarm_handle) {
        hals_rtc_flag_clear(RTC_FLAG_ALARM0);
        hals_rtc_interrupt_enable(RTC_INT_ALARM);
    } else{
        hals_rtc_interrupt_disable(RTC_INT_ALARM);
    }

    if(NULL != rtc_dev->rtc_irq.rtc_tamper0_handle) {
        hals_rtc_flag_clear(RTC_FLAG_TAMP0);
        hals_rtc_interrupt_enable(RTC_INT_TAMP);
    } else{
        hals_rtc_interrupt_disable(RTC_INT_TAMP);
    }

    if(NULL != rtc_dev->rtc_irq.rtc_tamper1_handle) {
        hals_rtc_flag_clear(RTC_FLAG_TAMP1);
        hals_rtc_interrupt_enable(RTC_INT_TAMP);
    } else{
        hals_rtc_interrupt_disable(RTC_INT_TAMP);
    }

    /* unlock rtc */
    HAL_UNLOCK(rtc_dev);
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      disable RTC interrupt
    \param[in]  rtc_dev: RTC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_interrupt_disable(hal_rtc_dev_struct *rtc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == rtc_dev) {
        HAL_DEBUGE("pointer [rtc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* lock RTC */
    HAL_LOCK(rtc_dev);
    rtc_dev->state = HAL_RTC_STATE_BUSY;
    rtc_dev->rtc_irq.rtc_timestamp_handle = NULL;
    rtc_dev->rtc_irq.rtc_alarm_handle = NULL;
    rtc_dev->rtc_irq.rtc_tamper0_handle = NULL;
    rtc_dev->rtc_irq.rtc_tamper1_handle = NULL;
    /* clear the RTC interrupt flag */
    hals_rtc_flag_clear(RTC_FLAG_TIMESTAMP | RTC_FLAG_ALARM0 | RTC_FLAG_TAMP0 | RTC_FLAG_TAMP1);
    /*  disable RTC interrupt */
    hals_rtc_interrupt_disable(RTC_INT_TIMESTAMP);
    hals_rtc_interrupt_disable(RTC_INT_ALARM);
    hals_rtc_interrupt_disable(RTC_INT_TAMP);
    /* unlock RTC */
    HAL_UNLOCK(rtc_dev);
    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      RTC interrupt handler content function,which is merely used in rtc_handler
    \param[in]  rtc_dev: RTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_rtc_irq(hal_rtc_dev_struct *rtc_dev)
{
    uint32_t int_en = 0U;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == rtc_dev) {
        HAL_DEBUGE("pointer [rtc_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* check rtc underflow interrupt state bit */
    int_en = RTC_CTL & RTC_CTL_TSIE;
    if((SET == hals_rtc_flag_get(RTC_FLAG_TIMESTAMP)) && (RESET != int_en)) {
        /* error callback */
        if(NULL != (rtc_dev->rtc_irq.rtc_timestamp_handle)) {
            rtc_dev->rtc_irq.rtc_timestamp_handle(rtc_dev);
        }
        /* clear interrupt flag */
        hals_rtc_flag_clear(RTC_FLAG_TIMESTAMP);
        hals_rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);
        EXTI_PD = (uint32_t)EXTI_RTC_TAMPER_TIMESTAMP_19;

    }

    int_en = RTC_CTL & RTC_CTL_ALRM0IE;
    if((SET == hals_rtc_flag_get(RTC_FLAG_ALARM0)) && (RESET != int_en)) {
        /* clear interrupt flag */
        hals_rtc_flag_clear(RTC_FLAG_ALARM0);
        EXTI_PD = (uint32_t)EXTI_RTC_ALARM_17;

        /* error callback */
        if(NULL != (rtc_dev->rtc_irq.rtc_alarm_handle)) {
            rtc_dev->rtc_irq.rtc_alarm_handle(rtc_dev);
        }
    }
    int_en = RTC_TAMP & RTC_TAMP_TPIE;
    if((SET == (hals_rtc_flag_get(RTC_FLAG_TAMP0))) && (RESET != int_en)) {
        /* clear interrupt flag */
        hals_rtc_flag_clear(RTC_FLAG_TAMP0);

        /* error callback */
        if(NULL != (rtc_dev->rtc_irq.rtc_tamper0_handle)) {
            rtc_dev->rtc_irq.rtc_tamper0_handle(rtc_dev);
        }
        hals_rtc_flag_clear(RTC_FLAG_TIMESTAMP);
        hals_rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);
        EXTI_PD = (uint32_t)EXTI_RTC_TAMPER_TIMESTAMP_19;
    }
    int_en = RTC_TAMP & RTC_TAMP_TPIE;
    if((SET == (hals_rtc_flag_get(RTC_FLAG_TAMP1))) && (RESET != int_en)) {
        /* clear interrupt flag */
        hals_rtc_flag_clear(RTC_FLAG_TAMP1);

        /* error callback */
        if(NULL != (rtc_dev->rtc_irq.rtc_tamper0_handle)) {
            rtc_dev->rtc_irq.rtc_tamper0_handle(rtc_dev);
        }
        hals_rtc_flag_clear(RTC_FLAG_TIMESTAMP);
        hals_rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);
        EXTI_PD = (uint32_t)EXTI_RTC_TAMPER_TIMESTAMP_19;
    }
}

/*!
    \brief      set user-defined interrupt callback function
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  rtc_dev: RTC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to RTC interrupt callback functions structure
                  hal_irq_handle_cb: the function is user-defined,
    \param[out] none
    \retval     none
*/
void hal_rtc_irq_handle_set(hal_rtc_dev_struct *rtc_dev, hal_rtc_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == rtc_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [rtc_dev] or pointer [p_irq] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* RTC interrupt handler set */
    if(NULL != p_irq->rtc_timestamp_handle) {
        rtc_dev->rtc_irq.rtc_timestamp_handle = p_irq->rtc_timestamp_handle;
        hals_rtc_flag_clear(RTC_FLAG_TIMESTAMP);
        hals_rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);
        hals_rtc_interrupt_enable(RTC_INT_TIMESTAMP);
        hal_exti_internal_init(EXTI_LINE_19_RTC_TAMPER_TIMESTAMP, EXTI_INTERRUPT_TRIG_RISING);
    } else {
        rtc_dev->rtc_irq.rtc_timestamp_handle = NULL;
        hals_rtc_interrupt_disable(RTC_INT_TIMESTAMP);
        EXTI_PD = (uint32_t)EXTI_RTC_TAMPER_TIMESTAMP_19;
    }
    if(NULL != p_irq->rtc_alarm_handle) {
        rtc_dev->rtc_irq.rtc_alarm_handle = p_irq->rtc_alarm_handle;
        hals_rtc_flag_clear(RTC_FLAG_ALARM0);
        hals_rtc_interrupt_enable(RTC_INT_ALARM);
        /* enable RTC alarm */
        hal_exti_internal_init(EXTI_LINE_17_RTC_ALARM, EXTI_INTERRUPT_TRIG_RISING);
        hals_rtc_alarm_enable();
    } else {
        rtc_dev->rtc_irq.rtc_alarm_handle = NULL;
        hals_rtc_interrupt_disable(RTC_INT_ALARM);
        /* disable RTC alarm */
        hals_rtc_alarm_disable();
        EXTI_PD = (uint32_t)EXTI_RTC_ALARM_17;
    }
    if(NULL != p_irq->rtc_tamper0_handle) {
        rtc_dev->rtc_irq.rtc_tamper0_handle = p_irq->rtc_tamper0_handle;
    } else {
        rtc_dev->rtc_irq.rtc_tamper0_handle = NULL;
    }
    if(NULL != p_irq->rtc_tamper1_handle) {
        rtc_dev->rtc_irq.rtc_tamper1_handle = p_irq->rtc_tamper1_handle;
    } else {
        rtc_dev->rtc_irq.rtc_tamper1_handle = NULL;
    }
    if((NULL != p_irq->rtc_tamper0_handle) || (NULL != p_irq->rtc_tamper1_handle)) {
        hals_rtc_flag_clear(RTC_FLAG_TAMP0);
        hals_rtc_flag_clear(RTC_FLAG_TAMP1);
        hals_rtc_interrupt_enable(RTC_INT_TAMP);
        hal_exti_internal_init(EXTI_LINE_19_RTC_TAMPER_TIMESTAMP, EXTI_INTERRUPT_TRIG_RISING);
    } else {
        hals_rtc_interrupt_disable(RTC_INT_TAMP);
        EXTI_PD = (uint32_t)EXTI_RTC_TAMPER_TIMESTAMP_19;
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  rtc_dev: RTC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_rtc_irq_handle_all_reset(hal_rtc_dev_struct *rtc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == rtc_dev) {
        HAL_DEBUGE("pointer [rtc_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* RTC interrupt handler set */
    rtc_dev->rtc_irq.rtc_timestamp_handle = NULL;
    rtc_dev->rtc_irq.rtc_alarm_handle = NULL;
    rtc_dev->rtc_irq.rtc_tamper0_handle = NULL;
    rtc_dev->rtc_irq.rtc_tamper1_handle = NULL;
}

/*!
    \brief      get current time and date
    \param[in]  none
    \param[out] rtc_initpara_struct: pointer to a hal_rtc_init_struct structure which contains
                parameters for initialization of the rtc peripheral
                members of the structure and the member values are shown as below:
                  rtc_year: 0~99
                  rtc_month:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_JAN: Janurary
      \arg          RTC_FEB: February
      \arg          RTC_MAR: March
      \arg          RTC_APR: April
      \arg          RTC_MAY: May
      \arg          RTC_JUN: June
      \arg          RTC_JUL: July
      \arg          RTC_AUG: August
      \arg          RTC_SEP: September
      \arg          RTC_OCT: October
      \arg          RTC_NOV: November
      \arg          RTC_DEC: December
                  rtc_date: 0~31
                  rtc_day_of_week:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_MONDAY: Monday
      \arg          RTC_TUESDAY: Tuesday
      \arg          RTC_WEDSDAY: Wednesday
      \arg          RTC_THURSDAY: Thursday
      \arg          RTC_FRIDAY: Friday
      \arg          RTC_SATURDAY: Saturday
      \arg          RTC_SUNDAY: Sunday
                  rtc_hour: 0 - 24 or 1 - 12
                  rtc_minute: 0 - 59
                  rtc_second: 0 - 59
                  rtc_am_pm:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_AM: AM format
      \arg          RTC_PM: PM format
                  rtc_daylight_saving:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_DAYLIGHT_SAVING_NONE: no daylight saving
      \arg          RTC_DAYLIGHT_SAVING_ADD_1H: add 1 hour(summer time change)
      \arg          RTC_DAYLIGHT_SAVING_SUB_1H: subtract 1 hour(winter time change)
                  rtc_clock_format:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_24H_FORMAT: 24-hour format
      \arg          RTC_12H_FORMAT: 12-hour format
                  rtc_psc_factor_s: 0x0 - 0x7FFF
                  rtc_psc_factor_a: 0x0 - 0x7F

    \retval     none
*/
void hal_rtc_current_time_get(hal_rtc_init_struct *rtc_initpara_struct)
{
    uint32_t temp_tr = 0x00U, temp_dr = 0x00U, temp_pscr = 0x00U, temp_ctlr = 0x00U;
    uint16_t temp_ss = 0x00U;

    /* if BPSHAD bit is reset, reading RTC_SS will lock RTC_TIME and RTC_DATE automatically */
    temp_ss = (uint16_t)RTC_SS;
    temp_tr = (uint32_t)RTC_TIME;
    temp_dr = (uint32_t)RTC_DATE;
    temp_pscr = (uint32_t)RTC_PSC;
    temp_ctlr = (uint32_t)RTC_CTL;
    /* read RTC_DATE to unlock the 3 shadow registers */
    (void)(RTC_DATE);

    /* get current time and construct rtc_parameter_struct structure */
    rtc_initpara_struct->rtc_year = hals_rtc_bcd_2_normal((uint8_t)GET_DATE_YR(temp_dr));
    rtc_initpara_struct->rtc_month = hals_rtc_bcd_2_normal(GET_DATE_MON(temp_dr));
    rtc_initpara_struct->rtc_date = hals_rtc_bcd_2_normal((uint8_t)GET_DATE_DAY(temp_dr));
    rtc_initpara_struct->rtc_day_of_week = hals_rtc_bcd_2_normal(GET_DATE_DOW(temp_dr));
    rtc_initpara_struct->rtc_hour = hals_rtc_bcd_2_normal((uint8_t)GET_TIME_HR(temp_tr));
    rtc_initpara_struct->rtc_minute = hals_rtc_bcd_2_normal((uint8_t)GET_TIME_MN(temp_tr));
    rtc_initpara_struct->rtc_second = hals_rtc_bcd_2_normal((uint8_t)GET_TIME_SC(temp_tr));
    rtc_initpara_struct->rtc_subsecond = temp_ss;
    rtc_initpara_struct->rtc_daylight_saving = (uint32_t)(temp_ctlr & (RTC_CTL_A1H | RTC_CTL_S1H));
    rtc_initpara_struct->rtc_psc_factor_a = (uint16_t)GET_PSC_FACTOR_A(temp_pscr);
    rtc_initpara_struct->rtc_psc_factor_s = (uint16_t)GET_PSC_FACTOR_S(temp_pscr);
    rtc_initpara_struct->rtc_am_pm = (uint32_t)(temp_pscr & RTC_TIME_PM);
    rtc_initpara_struct->rtc_clock_format = (uint32_t)(temp_ctlr & RTC_CTL_CS);

}

/*!
    \brief      get RTC alarm
    \param[in]  none
    \param[out] rtc_alarm_time: pointer to a hal_rtc_alarm_struct structure which contains
                parameters for RTC alarm configuration
                members of the structure and the member values are shown as below:
                  rtc_alarm_am_pm:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_AM: AM format
      \arg          RTC_ALARM_PM: PM format
                  rtc_alarm_weekday_mask:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_WEEKDAY_MASK_DISABLE: RTC alarm weekday no mask
      \arg          RTC_ALARM_WEEKDAY_MASK_ENABLE: RTC alarm weekday mask
                  rtc_alarm_hour_mask:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_HOUR_MASK_DISABLE: RTC alarm hour no mask
      \arg          RTC_ALARM_HOUR_MASK_ENABLE: RTC alarm hour mask
                  rtc_alarm_minute_mask:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_MINUTE_MASK_DISABLE: RTC alarm minute no mask
      \arg          RTC_ALARM_MINUTE_MASK_ENABLE: RTC alarm minute mask
                  rtc_alarm_second_mask:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_SECOND_MASK_DISABLE: RTC alarm second no mask
      \arg          RTC_ALARM_SECOND_MASK_ENABLE: RTC alarm second mask
                  rtc_alarm_subsecond_mask:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_MASKSSC_0_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_1_14: RTC alarm second mask
      \arg          RTC_MASKSSC_2_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_3_14: RTC alarm second mask
      \arg          RTC_MASKSSC_4_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_5_14: RTC alarm second mask
      \arg          RTC_MASKSSC_6_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_7_14: RTC alarm second mask
      \arg          RTC_MASKSSC_8_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_9_14: RTC alarm second mask
      \arg          RTC_MASKSSC_10_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_11_14: RTC alarm second mask
      \arg          RTC_MASKSSC_12_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_13_14: RTC alarm second no mask
      \arg          RTC_MASKSSC_14: RTC alarm second mask
      \arg          RTC_MASKSSC_NONE: RTC alarm second mask
                  rtc_weekday_or_date:
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_SELECT_WEEKDAY: RTC alarm select weekday
      \arg          RTC_ALARM_SELECT_DATE: RTC alarm select date
                  rtc_alarm_day: 1 - 31 or below
                  only one parameter can be selected which is shown as below:
      \arg          RTC_ALARM_MONDAY: RTC alarm Monday
      \arg          RTC_ALARM_TUESDAY: RTC alarm Tuesday
      \arg          RTC_ALARM_WEDSDAY: RTC alarm Wednesday
      \arg          RTC_ALARM_THURSDAY: RTC alarm Thursday
      \arg          RTC_ALARM_FRIDAY: RTC alarm Friday
      \arg          RTC_ALARM_SATURDAY: RTC alarm Saturday
      \arg          RTC_ALARM_SUNDAY: RTC alarm Sunday
                  rtc_hour: 0 - 24 or 1 - 12
                  rtc_minute: 0 - 59
                  rtc_second: 0 - 59
                  rtc_alarm_subsecond: 0x00~0x7FFF
    \retval     none
*/
void hal_rtc_alarm_get(hal_rtc_alarm_config_struct *rtc_alarm_time)
{
    uint32_t reg_alrm0td = 0x00U, reg_alrm0ss = 0x00U;

    /* get the value of RTC_ALRM0TD and RTC_ALRM0SS register */
    reg_alrm0td = RTC_ALRM0TD;
    reg_alrm0ss = RTC_ALRM0SS;

    /* get alarm parameters and construct the rtc_alarm_struct structure */
    rtc_alarm_time->rtc_alarm_am_pm = (uint32_t)(reg_alrm0td & RTC_ALRM0TD_PM);
    rtc_alarm_time->rtc_alarm_weekday_mask = (uint32_t)(reg_alrm0td & RTC_ALARM_WEEKDAY_MASK_ENABLE);
    rtc_alarm_time->rtc_alarm_hour_mask = reg_alrm0td & RTC_ALARM_HOUR_MASK_ENABLE;
    rtc_alarm_time->rtc_alarm_minute_mask = reg_alrm0td & RTC_ALARM_MINUTE_MASK_ENABLE;
    rtc_alarm_time->rtc_alarm_second_mask = reg_alrm0td & RTC_ALARM_SECOND_MASK_ENABLE;
    rtc_alarm_time->rtc_alarm_subsecond_mask = reg_alrm0ss & RTC_ALRM0SS_MASKSSC;
    rtc_alarm_time->rtc_weekday_or_date = (uint32_t)(reg_alrm0td & RTC_ALRM0TD_DOWS);
    rtc_alarm_time->rtc_alarm_day = hals_rtc_bcd_2_normal((uint8_t)GET_ALRM0TD_DAY(reg_alrm0td));
    rtc_alarm_time->rtc_alarm_hour = hals_rtc_bcd_2_normal((uint8_t)GET_ALRM0TD_HR(reg_alrm0td));
    rtc_alarm_time->rtc_alarm_minute = hals_rtc_bcd_2_normal((uint8_t)GET_ALRM0TD_MN(reg_alrm0td));
    rtc_alarm_time->rtc_alarm_second = hals_rtc_bcd_2_normal((uint8_t)GET_ALRM0TD_SC(reg_alrm0td));
    rtc_alarm_time->rtc_alarm_subsecond = reg_alrm0ss & RTC_ALRM0SS_SSC;
}

/*!
    \brief      poll for RTC alarm event
    \param[in]  timeout_ms: the time cost for event polling
    \param[out] none
    \retval     error code: HAL_ERR_TIMEOUT, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_alarm_event_poll(uint32_t timeout_ms)
{
    uint32_t flag_status = RESET;
    uint32_t tick_start = 0;

    tick_start = hal_sys_basetick_count_get();
    /* wait until ALRM0F flag to be set after the alarm is enabled */
    do {
        flag_status = RTC_STAT & RTC_STAT_ALRM0F;
    } while((SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) && ((uint32_t)RESET == flag_status));

    if((uint32_t)RESET == flag_status) {
        return HAL_ERR_TIMEOUT;
    }

    hals_rtc_flag_clear(RTC_FLAG_ALARM0);
    return HAL_ERR_NONE;
}

/*!
    \brief      get RTC timestamp time and date
    \param[in]  none
    \param[out] rtc_timestamp: pointer to a rtc_timestamp_struct structure which contains
                parameters for RTC time-stamp configuration
                members of the structure and the member values are shown as below:
                  rtc_timestamp_month: RTC_JAN, RTC_FEB, RTC_MAR, RTC_APR, RTC_MAY, RTC_JUN,
                                       RTC_JUL, RTC_AUG, RTC_SEP, RTC_OCT, RTC_NOV, RTC_DEC
                  rtc_timestamp_date: 0x1 - 0x31(BCD format)
                  rtc_timestamp_weekday: RTC_MONDAY, RTC_TUESDAY, RTC_WEDSDAY, RTC_THURSDAY, RTC_FRIDAY,
                                     RTC_SATURDAY, RTC_SUNDAY if RTC_ALARM_WEEKDAY_SELECTED is set
                  rtc_timestamp_hour: 0x0 - 0x12(BCD format) or 0x0 - 0x23(BCD format) depending on the rtc_display_format
                  rtc_timestamp_minute: 0x0 - 0x59(BCD format)
                  rtc_timestamp_second: 0x0 - 0x59(BCD format)
                  rtc_am_pm: RTC_AM, RTC_PM
    \retval     none
*/
void hal_rtc_timestamp_get(hal_rtc_timestamp_struct *rtc_timestamp)
{
    uint32_t temp_tts = 0x00U, temp_dts = 0x00U, temp_ssts = 0x00U;

    /* get the value of time_stamp registers */
    temp_tts = (uint32_t)RTC_TTS;
    temp_dts = (uint32_t)RTC_DTS;
    temp_ssts = (uint32_t)RTC_SSTS;

    /* get timestamp time and construct the rtc_timestamp_struct structure */
    rtc_timestamp->rtc_am_pm = (uint32_t)(temp_tts & RTC_TTS_PM);
    rtc_timestamp->rtc_timestamp_month = hals_rtc_bcd_2_normal((uint8_t)GET_DTS_MON(temp_dts));
    rtc_timestamp->rtc_timestamp_date = hals_rtc_bcd_2_normal((uint8_t)GET_DTS_DAY(temp_dts));
    rtc_timestamp->rtc_timestamp_weekday = (uint8_t)GET_DTS_DOW(temp_dts);
    rtc_timestamp->rtc_timestamp_hour = hals_rtc_bcd_2_normal((uint8_t)GET_TTS_HR(temp_tts));
    rtc_timestamp->rtc_timestamp_minute = hals_rtc_bcd_2_normal((uint8_t)GET_TTS_MN(temp_tts));
    rtc_timestamp->rtc_timestamp_second = hals_rtc_bcd_2_normal((uint8_t)GET_TTS_SC(temp_tts));
    rtc_timestamp->rtc_timestamp_subsecond = temp_ssts;

}

/*!
    \brief      poll for RTC timestamp event
    \param[in]  timeout_ms: the time cost for event polling
    \param[out] none
    \retval     error code: HAL_ERR_TIMEOUT, HAL_ERR_HARDWARE, HAL_ERR_NONE,
                details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_timestamp_event_poll(uint32_t timeout_ms)
{
    uint32_t flag_status = RESET;
    uint32_t tick_start = 0;

    /* wait until TSF flag to be set */
    flag_status = RTC_STAT & RTC_STAT_TSF;
    tick_start = hal_sys_basetick_count_get();
    while((SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) && ((uint32_t)RESET == flag_status)) {
        flag_status = RTC_STAT & RTC_STAT_TSF;
        if((uint32_t)RESET != (RTC_STAT & RTC_STAT_TSOVRF)) {
            return HAL_ERR_HARDWARE;
        }
    }

    if((uint32_t)RESET == flag_status) {
        return HAL_ERR_TIMEOUT;
    }

    hals_rtc_flag_clear(RTC_FLAG_TIMESTAMP);
    hals_rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);

    return HAL_ERR_NONE;
}

/*!
    \brief      poll for RTC tamper0 event
    \param[in]  timeout_ms: the time cost for event polling
    \param[out] none
    \retval     error code: HAL_ERR_TIMEOUT, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_tamper0_event_poll(uint32_t timeout_ms)
{
    uint32_t flag_status = RESET;
    uint32_t tick_start = 0;

    tick_start = hal_sys_basetick_count_get();
    /* wait until TP0F flag to be set */
    do {
        flag_status = RTC_STAT & RTC_STAT_TP0F;
    } while((SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) && ((uint32_t)RESET == flag_status));

    if((uint32_t)RESET == flag_status) {
        return HAL_ERR_TIMEOUT;
    }

    hals_rtc_flag_clear(RTC_FLAG_TAMP0);

    return HAL_ERR_NONE;
}

/*!
    \brief      poll for RTC tamper1 event
    \param[in]  timeout_ms: the time cost for event polling
    \param[out] none
    \retval     error code: HAL_ERR_TIMEOUT, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_rtc_tamper1_event_poll(uint32_t timeout_ms)
{
    uint32_t flag_status = RESET;
    uint32_t tick_start = 0;

    tick_start = hal_sys_basetick_count_get();
    /* wait until TP1F flag to be set */
    do {
        flag_status = RTC_STAT & RTC_STAT_TP1F;
    } while((SET == hal_sys_basetick_timeout_check(tick_start, timeout_ms)) && ((uint32_t)RESET == flag_status));

    if((uint32_t)RESET == flag_status) {
        return HAL_ERR_TIMEOUT;
    }

    hals_rtc_flag_clear(RTC_FLAG_TAMP1);

    return HAL_ERR_NONE;
}

/*!
    \brief      ajust the daylight saving time by adding or substracting one hour from the current time
    \param[in]  operation: hour ajustment operation
                only one parameter can be selected which is shown as below:
      \arg        RTC_DAYLIGHT_SAVING_ADD_1H: add one hour
      \arg        RTC_DAYLIGHT_SAVING_SUB_1H: substract one hour
      \arg        RTC_DAYLIGHT_SAVING_NONE: no add or subtract one hour
    \param[in]  record: daylight saving mark operation
                only one parameter can be selected which is shown as below:
      \arg        RTC_RECORD_DAYLIGHTSAVING_SET: set daylight saving mark
      \arg        RTC_RECORD_DAYLIGHTSAVING_RESET: reset daylight saving mark
    \param[out] none
    \retval     none
*/
void hal_rtc_daylight_saving_time_adjust(uint32_t operation, uint32_t record)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL &= (uint32_t)(~(RTC_CTL_A1H | RTC_CTL_S1H | RTC_CTL_DSM));
    RTC_CTL |= (uint32_t)(operation | record);

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      reset most of the RTC registers
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus hals_rtc_deinit(void)
{
    ErrStatus error_status = ERROR;

    /* RTC_TAMP register is not under write protection */
    RTC_TAMP = RTC_REGISTER_RESET;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* reset RTC_CTL register, this can be done without the init mode */
    RTC_CTL &= RTC_REGISTER_RESET;

    /* enter init mode */
    error_status = hals_rtc_init_mode_enter();

    if(ERROR != error_status) {
        /* before reset RTC_TIME and RTC_DATE, BPSHAD bit in RTC_CTL should be reset as the condition.
           in order to read calendar from shadow register, not the real registers being reset */
        RTC_TIME = RTC_REGISTER_RESET;
        RTC_DATE = RTC_DATE_RESET;

        RTC_PSC = RTC_PSC_RESET;

        /* reset RTC_STAT register, also exit init mode.
           at the same time, RTC_STAT_SOPF bit is reset, as the condition to reset RTC_SHIFTCTL register later */
        RTC_STAT = RTC_STAT_RESET;

        /* to write RTC_ALRM0SS register, ALRM0EN bit in RTC_CTL register should be reset as the condition */
        RTC_ALRM0TD = RTC_REGISTER_RESET;
        RTC_ALRM0SS = RTC_REGISTER_RESET;

        /* reset RTC_SHIFTCTL and RTC_HRFC register, this can be done without the init mode */
        RTC_SHIFTCTL = RTC_REGISTER_RESET;
        RTC_HRFC = RTC_REGISTER_RESET;

        error_status = hals_rtc_register_sync_wait();
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return error_status;
}

/*!
    \brief      enter RTC init mode
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus hals_rtc_init_mode_enter(void)
{
    uint32_t time_index = RTC_INITM_TIMEOUT;
    uint32_t flag_status = RESET;
    ErrStatus error_status = ERROR;

    /* check whether it has been in init mode */
    if(RESET == (RTC_STAT & RTC_STAT_INITF)) {
        RTC_STAT |= RTC_STAT_INITM;

        /* wait until the INITF flag to be set */
        do {
            flag_status = RTC_STAT & RTC_STAT_INITF;
        } while((--time_index > 0x00U) && (RESET == flag_status));

        if(RESET != flag_status) {
            error_status = SUCCESS;
        }
    } else {
        error_status = SUCCESS;
    }
    return error_status;
}

/*!
    \brief      exit RTC init mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_rtc_init_mode_exit(void)
{
    RTC_STAT &= (uint32_t)(~RTC_STAT_INITM);
}

/*!
    \brief      wait until RTC_TIME and RTC_DATE registers are synchronized with APB clock, and the shadow
                registers are updated
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus hals_rtc_register_sync_wait(void)
{
    volatile uint32_t time_index = RTC_RSYNF_TIMEOUT;
    uint32_t flag_status = RESET;
    ErrStatus error_status = ERROR;

    if(RESET == (RTC_CTL & RTC_CTL_BPSHAD)) {
        /* disable the write protection */
        RTC_WPK = RTC_UNLOCK_KEY1;
        RTC_WPK = RTC_UNLOCK_KEY2;

        /* firstly clear RSYNF flag */
        RTC_STAT &= (uint32_t)(~RTC_STAT_RSYNF);

        /* wait until RSYNF flag to be set */
        do {
            flag_status = RTC_STAT & RTC_STAT_RSYNF;
        } while((--time_index > 0x00U) && (RESET == flag_status));

        if(RESET != flag_status) {
            error_status = SUCCESS;
        }

        /* enable the write protection */
        RTC_WPK = RTC_LOCK_KEY;
    } else {
        error_status = SUCCESS;
    }

    return error_status;
}

/*!
    \brief      get current subsecond value
    \param[in]  none
    \param[out] none
    \retval     current subsecond value
*/
uint32_t hals_rtc_subsecond_get(void)
{
    uint32_t reg = 0x00U;
    /* if BPSHAD bit is reset, reading RTC_SS will lock RTC_TIME and RTC_DATE automatically */
    reg = (uint32_t)RTC_SS;
    /* read RTC_DATE to unlock the 3 shadow registers */
    (void)(RTC_DATE);

    return reg;
}

/*!
    \brief      configure subsecond of RTC alarm
    \param[in]  mask_subsecond: alarm subsecond mask
                only one parameter can be selected which is shown as below:
      \arg        RTC_MASKSSC_0_14: mask alarm subsecond configuration
      \arg        RTC_MASKSSC_1_14: mask RTC_ALRM0SS_SSC[14:1], and RTC_ALRM0SS_SSC[0] is to be compared
      \arg        RTC_MASKSSC_2_14: mask RTC_ALRM0SS_SSC[14:2], and RTC_ALRM0SS_SSC[1:0] is to be compared
      \arg        RTC_MASKSSC_3_14: mask RTC_ALRM0SS_SSC[14:3], and RTC_ALRM0SS_SSC[2:0] is to be compared
      \arg        RTC_MASKSSC_4_14: mask RTC_ALRM0SS_SSC[14:4], and RTC_ALRM0SS_SSC[3:0] is to be compared
      \arg        RTC_MASKSSC_5_14: mask RTC_ALRM0SS_SSC[14:5], and RTC_ALRM0SS_SSC[4:0] is to be compared
      \arg        RTC_MASKSSC_6_14: mask RTC_ALRM0SS_SSC[14:6], and RTC_ALRM0SS_SSC[5:0] is to be compared
      \arg        RTC_MASKSSC_7_14: mask RTC_ALRM0SS_SSC[14:7], and RTC_ALRM0SS_SSC[6:0] is to be compared
      \arg        RTC_MASKSSC_8_14: mask RTC_ALRM0SS_SSC[14:8], and RTC_ALRM0SS_SSC[7:0] is to be compared
      \arg        RTC_MASKSSC_9_14: mask RTC_ALRM0SS_SSC[14:9], and RTC_ALRM0SS_SSC[8:0] is to be compared
      \arg        RTC_MASKSSC_10_14: mask RTC_ALRM0SS_SSC[14:10], and RTC_ALRM0SS_SSC[9:0] is to be compared
      \arg        RTC_MASKSSC_11_14: mask RTC_ALRM0SS_SSC[14:11], and RTC_ALRM0SS_SSC[10:0] is to be compared
      \arg        RTC_MASKSSC_12_14: mask RTC_ALRM0SS_SSC[14:12], and RTC_ALRM0SS_SSC[11:0] is to be compared
      \arg        RTC_MASKSSC_13_14: mask RTC_ALRM0SS_SSC[14:13], and RTC_ALRM0SS_SSC[12:0] is to be compared
      \arg        RTC_MASKSSC_14: mask RTC_ALRM0SS_SSC[14], and RTC_ALRM0SS_SSC[13:0] is to be compared
      \arg        RTC_MASKSSC_NONE: mask none, and RTC_ALRM0SS_SSC[14:0] is to be compared
    \param[in]  subsecond: alarm subsecond value(0x000 - 0x7FFF)
    \param[out] none
    \retval     none
*/
void hals_rtc_alarm_subsecond_config(uint32_t mask_subsecond, uint32_t subsecond)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_ALRM0SS = mask_subsecond | subsecond;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      get RTC alarm subsecond
    \param[in]  none
    \param[out] none
    \retval     RTC alarm subsecond value
*/
uint32_t hals_rtc_alarm_subsecond_get(void)
{
    return ((uint32_t)(RTC_ALRM0SS & RTC_ALRM0SS_SSC));
}

/*!
    \brief      enable RTC alarm
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_rtc_alarm_enable(void)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL |= RTC_CTL_ALRM0EN;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      disable RTC alarm
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus hals_rtc_alarm_disable(void)
{
    volatile uint32_t time_index = RTC_ALRM0WF_TIMEOUT;
    ErrStatus error_status = ERROR;
    uint32_t flag_status = RESET;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* clear the state of alarm */
    RTC_CTL &= (uint32_t)(~RTC_CTL_ALRM0EN);

    /* wait until ALRM0WF flag to be set after the alarm is disabled */
    do {
        flag_status = RTC_STAT & RTC_STAT_ALRM0WF;
    } while((--time_index > 0x00U) && (RESET == flag_status));

    if(RESET != flag_status) {
        error_status = SUCCESS;
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return error_status;
}

/*!
    \brief      enable RTC time-stamp
    \param[in]  edge: specify which edge to detect of time-stamp
                only one parameter can be selected which is shown as below:
      \arg        RTC_TIMESTAMP_RISING_EDGE: rising edge is valid event edge for timestamp event
      \arg        RTC_TIMESTAMP_FALLING_EDGE: falling edge is valid event edge for timestamp event
    \param[out] none
    \retval     none
*/
void hals_rtc_timestamp_enable(uint32_t edge)
{
    uint32_t reg_ctl = 0x00U;

    /* clear the bits to be configured in RTC_CTL */
    reg_ctl = (uint32_t)(RTC_CTL & (uint32_t)(~(RTC_CTL_TSEG | RTC_CTL_TSEN)));

    /* new configuration */
    reg_ctl |= (uint32_t)(edge | RTC_CTL_TSEN);

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL = (uint32_t)reg_ctl;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      disable RTC time-stamp
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_rtc_timestamp_disable(void)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* clear the TSEN bit */
    RTC_CTL &= (uint32_t)(~ RTC_CTL_TSEN);

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      get RTC time-stamp subsecond
    \param[in]  none
    \param[out] none
    \retval     RTC time-stamp subsecond value
*/
uint32_t hals_rtc_timestamp_subsecond_get(void)
{
    return ((uint32_t)RTC_SSTS);
}

/*!
    \brief      enable RTC tamper
    \param[in]  source: specify which tamper source to be disabled
                only one parameter can be selected which is shown as below:
      \arg        RTC_TAMPER0
      \arg        RTC_TAMPER1
    \param[out] none
    \retval     none
*/
void hals_rtc_tamper_enable(uint32_t source)
{
    /* enable tamper */
    RTC_TAMP |= (uint32_t)source;
}

/*!
    \brief      disable RTC tamper
    \param[in]  source: specify which tamper source to be disabled
                only one parameter can be selected which is shown as below:
      \arg        RTC_TAMPER0
      \arg        RTC_TAMPER1
    \param[out] none
    \retval     none
*/
void hals_rtc_tamper_disable(uint32_t source)
{
    /* disable tamper */
    RTC_TAMP &= (uint32_t)~source;
}

/*!
    \brief      enable specified RTC interrupt
    \param[in]  interrupt: specify which interrupt source to be enabled
                only one parameter can be selected which is shown as below:
      \arg        RTC_INT_TIMESTAMP: timestamp interrupt
      \arg        RTC_INT_ALARM: alarm interrupt
      \arg        RTC_INT_TAMP: tamp interrupt
    \param[out] none
    \retval     none
*/
void hals_rtc_interrupt_enable(uint32_t interrupt)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* enable the interrupts in RTC_CTL register */
    RTC_CTL |= (uint32_t)(interrupt & (uint32_t)~RTC_TAMP_TPIE);
    /* enable the interrupts in RTC_TAMP register */
    RTC_TAMP |= (uint32_t)(interrupt & RTC_TAMP_TPIE);

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      disble specified RTC interrupt
    \param[in]  interrupt: specify which interrupt source to be disabled
                only one parameter can be selected which is shown as below:
      \arg        RTC_INT_TIMESTAMP: timestamp interrupt
      \arg        RTC_INT_ALARM: alarm interrupt
      \arg        RTC_INT_TAMP: tamp interrupt
    \param[out] none
    \retval     none
*/
void hals_rtc_interrupt_disable(uint32_t interrupt)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* disable the interrupts in RTC_CTL register */
    RTC_CTL &= (uint32_t)~(interrupt & (uint32_t)~RTC_TAMP_TPIE);
    /* disable the interrupts in RTC_TAMP register */
    RTC_TAMP &= (uint32_t)~(interrupt & RTC_TAMP_TPIE);

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      check specified flag
    \param[in]  flag: specify which flag to check
                only one parameter can be selected which is shown as below:
      \arg        RTC_FLAG_RECALIBRATION: recalibration pending flag
      \arg        RTC_FLAG_TAMP1: tamper 1 event flag
      \arg        RTC_FLAG_TAMP0: tamper 0 event flag
      \arg        RTC_FLAG_TIMESTAMP_OVERFLOW: time-stamp overflow event flag
      \arg        RTC_FLAG_TIMESTAMP: time-stamp event flag
      \arg        RTC_FLAG_ALARM0: alarm event flag
      \arg        RTC_FLAG_INIT: init mode event flag
      \arg        RTC_FLAG_RSYN: time and date registers synchronized event flag
      \arg        RTC_FLAG_YCM: year parameter configured event flag
      \arg        RTC_FLAG_SHIFT: shift operation pending flag
      \arg        RTC_FLAG_ALARM0_WRITTEN: alarm writen available flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_rtc_flag_get(uint32_t flag)
{
    FlagStatus flag_state = RESET;

    if(RESET != (RTC_STAT & flag)) {
        flag_state = SET;
    }
    return flag_state;
}

/*!
    \brief      clear specified flag
    \param[in]  flag: specify which flag to clear
      \arg        RTC_FLAG_TAMP1: tamper 1 event flag
      \arg        RTC_FLAG_TAMP0: tamper 0 event flag
      \arg        RTC_FLAG_TIMESTAMP_OVERFLOW: time-stamp overflow event flag
      \arg        RTC_FLAG_TIMESTAMP: time-stamp event flag
      \arg        RTC_FLAG_ALARM0: alarm event flag
      \arg        RTC_FLAG_RSYN: time and date registers synchronized event flag
    \param[out] none
    \retval     none
*/
void hals_rtc_flag_clear(uint32_t flag)
{
    RTC_STAT &= (uint32_t)(~flag);
}

/*!
    \brief      configure rtc alternate output source
    \param[in]  source: specify signal to output
                only one parameter can be selected which is shown as below:
      \arg        RTC_CALIBRATION_512HZ: when the LSE freqency is 32768Hz and the RTC_PSC
                                         is the default value, output 512Hz signal
      \arg        RTC_CALIBRATION_1HZ: when the LSE freqency is 32768Hz and the RTC_PSC
                                       is the default value, output 1Hz signal
      \arg        RTC_ALARM_HIGH: when the  alarm flag is set, the output pin is high
      \arg        RTC_ALARM_LOW: when the  Alarm flag is set, the output pin is low
    \param[in]  mode: specify the output pin (PC13) mode when output alarm signal
                only one parameter can be selected which is shown as below:
      \arg        RTC_ALARM_OUTPUT_OD: open drain mode
      \arg        RTC_ALARM_OUTPUT_PP: push pull mode
    \param[out] none
    \retval     none
*/
void hals_rtc_alter_output_config(uint32_t source, uint32_t mode)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL &= (uint32_t)~(RTC_CTL_COEN | RTC_CTL_OS | RTC_CTL_OPOL | RTC_CTL_COS);

    RTC_CTL |= (uint32_t)(source);

    /* alarm output */
    if(RESET != (source & RTC_OS_ENABLE)) {
        RTC_TAMP &= (uint32_t)~(RTC_TAMP_PC13VAL);
        RTC_TAMP |= (uint32_t)(mode);
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      configure RTC calibration register
    \param[in]  window: select calibration window
                only one parameter can be selected which is shown as below:
      \arg        RTC_CALIBRATION_WINDOW_32S: 2exp20 RTCCLK cycles, 32s if RTCCLK = 32768 Hz
      \arg        RTC_CALIBRATION_WINDOW_16S: 2exp19 RTCCLK cycles, 16s if RTCCLK = 32768 Hz
      \arg        RTC_CALIBRATION_WINDOW_8S: 2exp18 RTCCLK cycles, 8s if RTCCLK = 32768 Hz
    \param[in]  plus: add RTC clock or not
                only one parameter can be selected which is shown as below:
      \arg        RTC_CALIBRATION_PLUS_SET: add one RTC clock every 2048 rtc clock
      \arg        RTC_CALIBRATION_PLUS_RESET: no effect
    \param[in]  minus: the RTC clock to minus during the calibration window(0x0 - 0x1FF)
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus hals_rtc_calibration_config(uint32_t window, uint32_t plus, uint32_t minus)
{
    uint32_t time_index = RTC_HRFC_TIMEOUT;
    ErrStatus error_status = ERROR;
    uint32_t flag_status = RESET;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* check if a calibration operation is ongoing */
    do {
        flag_status = RTC_STAT & RTC_STAT_SCPF;
    } while((--time_index > 0x00U) && (RESET != flag_status));

    if(RESET == flag_status) {
        RTC_HRFC = (uint32_t)(window | plus | HRFC_CMSK(minus));
        error_status = SUCCESS;
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return error_status;
}

/*!
    \brief      ajust RTC second or subsecond value of current time
    \param[in]  add: add 1s to current time or not
                only one parameter can be selected which is shown as below:
      \arg        RTC_SHIFT_ADD1S_RESET: no effect
      \arg        RTC_SHIFT_ADD1S_SET: add 1s to current time
    \param[in]  minus: number of subsecond to minus from current time(0x0 - 0x7FFF)
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus hals_rtc_second_adjust(uint32_t add, uint32_t minus)
{
    uint32_t time_index = RTC_SHIFTCTL_TIMEOUT;
    ErrStatus error_status = ERROR;
    uint32_t flag_status = RESET;
    uint32_t temp = 0U;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* check if a shift operation is ongoing */
    do {
        flag_status = RTC_STAT & RTC_STAT_SOPF;
    } while((--time_index > 0x00U) && (RESET != flag_status));

    temp = RTC_CTL & RTC_CTL_REFEN;
    /* check if the function of reference clock detection is disabled */
    if((RESET == flag_status) && (RESET == temp)) {
        RTC_SHIFTCTL = (uint32_t)(add | SHIFTCTL_SFS(minus));
        error_status = hals_rtc_register_sync_wait();
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return error_status;
}

/*!
    \brief      enable RTC bypass shadow registers function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_rtc_bypass_shadow_enable(void)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL |= (uint8_t)RTC_CTL_BPSHAD;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      disable RTC bypass shadow registers function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_rtc_bypass_shadow_disable(void)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL &= (uint8_t)~RTC_CTL_BPSHAD;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      write data to backup register
    \param[in]  backup_index: the index of backup register
      \arg        0 - 4
    \param[in]  data: the data to be written to backup register
      \arg        0x0 - 0xFFFFFFFF
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hals_rtc_backup_data_write(uint32_t backup_index, uint32_t data)
{
    uint32_t reg = 0x00U;

#if (1 == HAL_PARAMETER_CHECK)
    /* check parameter backup_index */
    if(backup_index > 4U) {
        HAL_DEBUGE("parameter [backup_index] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    reg = RTC_BKP0 + (backup_index << 2);
    *(__IO uint32_t *)reg = data;

    return HAL_ERR_NONE;
}

/*!
    \brief      read data from backup register
    \param[in]  backup_index: the index of backup register
      \arg        0 - 4
    \param[out] data: the data read from backup register
      \arg        0x0 - 0xFFFFFFFF
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hals_rtc_backup_data_read(uint32_t backup_index, uint32_t *data)
{
    uint32_t reg = 0x00U;

#if (1 == HAL_PARAMETER_CHECK)
    /* check parameter backup_index */
    if(backup_index > 4U) {
        HAL_DEBUGE("parameter [backup_index] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    reg = RTC_BKP0 + (backup_index << 2);
    *data = (*(__IO uint32_t *)reg);

    return HAL_ERR_NONE;
}

/*!
    \brief      convert from  BCD format to binary format
    \param[in]  data: data to be converted
    \param[out] none
    \retval     converted data
*/
uint8_t hals_rtc_bcd_2_normal(uint8_t data)
{
    uint8_t temp = 0U;
    temp = ((uint8_t)(data & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
    return (temp + (data & (uint8_t)0x0F));
}

/*!
    \brief      convert from  BCD format to binary format
    \param[in]  data: data to be converted
    \param[out] none
    \retval     converted data
*/
uint8_t hals_rtc_normal_2_bcd(uint8_t data)
{
    uint8_t bcd_high = 0;

    while(data >= 10) {
        bcd_high++;
        data -= 10;
    }

    return ((uint8_t)(bcd_high << 4) | data);
}
