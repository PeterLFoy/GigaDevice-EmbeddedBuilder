/*!
    \file    gd32f3x0_hal_rcu.c
    \brief   RCU driver

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

/* define clock source */
#define SEL_IRC8M       0x00U
#define SEL_HXTAL       0x01U
#define SEL_PLL         0x02U
#define RCU_MODIFY(__delay)     do{                                     \
                                    volatile uint32_t i;                \
                                    if(0 != __delay){                   \
                                        RCU_CFG0 |= RCU_AHB_CKSYS_DIV2; \
                                        for(i=0; i<__delay; i++){       \
                                        }                               \
                                        RCU_CFG0 |= RCU_AHB_CKSYS_DIV4; \
                                        for(i=0; i<__delay; i++){       \
                                        }                               \
                                    }                                   \
                                }while(0)

/* define startup timeout count */
#define OSC_STARTUP_TIMEOUT            ((uint32_t)0x000FFFFFU)
#define LXTAL_STARTUP_TIMEOUT          ((uint32_t)0x03FFFFFFU)

/* CKOUT GPIOA */
#define _CKOUT_GPIO_CLK_ENABLE()       hal_rcu_periph_clk_enable(RCU_GPIOA)
#define _CKOUT_GPIO_PORT               GPIOA
#define _CKOUT_GPIO_PIN                GPIO_PIN_8
#define _RCU_CKSYSSRC_INDEX(x)         (x)

hal_rcu_irq_struct rcu_irq = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

int32_t g_systemcoreclock = 8000000;
extern hal_sys_timebase_source_enum g_basetick_source;

/* RCU system clock oscillator stable flag */
static const hal_rcu_flag_enum s_rcu_stab_flag[] = {
    RCU_FLAG_IRC8MSTB,
    RCU_FLAG_HXTALSTB,
    RCU_FLAG_PLLSTB
};

/* RCU system clock source timeout */
static const uint32_t s_rcu_timeout[] = {
    HAL_IRC8M_TIMEOUT,
    HAL_HXTAL_TIMEOUT,
    HAL_PLL_TIMEOUT
};

/* RCU system clock source */
static const uint32_t s_rcu_scss[] = {
    RCU_SCSS_IRC8M,
    RCU_SCSS_HXTAL,
    RCU_SCSS_PLL
};

/*!
    \brief      configure the RCU oscillators
    \param[in]  rcu_osci: the pointer of the RCU oscillators structure
                  hxtal: HXTAL status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  lxtal: LXTAL status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc48m: IRC48M status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc28m: IRC28M status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    calibration_value: calibration value
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc8m: IRC8M status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    calibration_value: calibration value
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc40k: IRC40K status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  pll: PLL status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                    pll_source: RCU_PLL_SRC_HXTAL_IRC48M, RCU_PLL_SRC_IRC8M_DIV2
                    pll_presel: the argument could be selected from enumeration <hal_rcu_pll_presel_enum>
                    pre_div: the argument could be selected from enumeration <hal_rcu_pll_prediv_enum>
                    pll_mul: the argument could be selected from enumeration <hal_rcu_pll_mul_enum>
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_TIMEOUT, details refer to gd32f3x0_hal.h
*/
int32_t hal_rcu_osci_config(hal_rcu_osci_struct *rcu_osci)
{
    FlagStatus pwr_state = RESET;

#if (1 == HAL_PARAMETER_CHECK)

    if(NULL == rcu_osci) {
        HAL_DEBUGE("parameter [rcu_osci] is a NULL pointer.");

        return HAL_ERR_ADDRESS;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    /* clear all rcu configuration */
    hal_rcu_deinit();

    /* configure HXTAL */
    if(ENABLE == rcu_osci->hxtal.need_configure) {
        /* it is not allowed to be disabled when the HXTAL is used as system clock or clock source of PLL */
        if((RCU_SCSS_HXTAL == hals_rcu_system_clock_source_get()) || \
                ((RCU_SCSS_PLL == hals_rcu_system_clock_source_get()) && \
                 (RCU_PLLSRC_HXTAL_IRC48M == (RCU_CFG0 & RCU_CFG0_PLLSEL)))) {
            if((RESET != (RCU_CTL0 & RCU_CTL0_HXTALSTB)) && (RCU_OSC_OFF == rcu_osci->hxtal.state)) {
                return HAL_ERR_VAL;
            }
        } else {
            /* configure the new HXTAL state */
            switch(rcu_osci->hxtal.state) {
            case RCU_OSC_OFF:
                hals_rcu_osci_off(RCU_HXTAL);
                hals_rcu_osci_bypass_mode_disable(RCU_HXTAL);
                break;
            case RCU_OSC_ON:
                hals_rcu_osci_on(RCU_HXTAL);
                break;
            case RCU_OSC_BYPASS:
                hals_rcu_osci_off(RCU_HXTAL);
                hals_rcu_osci_bypass_mode_enable(RCU_HXTAL);
                hals_rcu_osci_on(RCU_HXTAL);
                break;
            default:
                break;
            }

            if(RCU_OSC_OFF != rcu_osci->hxtal.state) {
                if(ERROR == hals_rcu_osci_stab_wait(RCU_HXTAL)) {
                    return HAL_ERR_TIMEOUT;
                }
            } else {
                uint32_t tick_start = hal_sys_basetick_count_get();

                /* wait till HXTAL is disable */
                while(RESET != (RCU_CTL0 & RCU_CTL0_HXTALSTB)) {
                    if(SET == hal_sys_basetick_timeout_check(tick_start, HAL_HXTAL_TIMEOUT)) {
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }
        }
    }

    /* configure LXTAL */
    if(ENABLE == rcu_osci->lxtal.need_configure) {
        if(RESET == (RCU_APB1EN & RCU_APB1EN_PMUEN)) {
            hal_rcu_periph_clk_enable(RCU_PMU);
            pwr_state = SET;
        }

        /* update LXTAL configuration in backup domain control register */
        if(RESET == (PMU_CTL & PMU_CTL_BKPWEN)) {
            uint32_t tick_start = 0U;

            /* enable write access to backup domain */
            hals_pmu_backup_write_enable();

            tick_start = hal_sys_basetick_count_get();

            while(RESET == (PMU_CTL & PMU_CTL_BKPWEN)) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, HAL_BP_TIMEOUT)) {
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* configure the new LXTAL state */
        switch(rcu_osci->lxtal.state) {
        case RCU_OSC_OFF:
            hals_rcu_osci_off(RCU_LXTAL);
            hals_rcu_osci_bypass_mode_disable(RCU_LXTAL);
            break;
        case RCU_OSC_ON:
            hals_rcu_osci_on(RCU_LXTAL);
            break;
        case RCU_OSC_BYPASS:
            hals_rcu_osci_off(RCU_LXTAL);
            hals_rcu_osci_bypass_mode_enable(RCU_LXTAL);
            hals_rcu_osci_on(RCU_LXTAL);
            break;
        default:
            break;
        }

        if(RCU_OSC_OFF != rcu_osci->lxtal.state) {
            if(ERROR == hals_rcu_osci_stab_wait(RCU_LXTAL)) {
                return HAL_ERR_TIMEOUT;
            }
        } else {
            /* wait till LXTAL is disabled */
            uint32_t tick_start = hal_sys_basetick_count_get();

            while(RESET != (RCU_BDCTL & RCU_BDCTL_LXTALSTB)) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, HAL_LXTAL_TIMEOUT)) {
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        if(SET == pwr_state) {
            hal_rcu_periph_clk_disable(RCU_PMU);

            pwr_state = RESET;
        }
    }

    /* configure IRC8M */
    if(ENABLE == rcu_osci->irc8m.need_configure) {
        /* it is not allowed to be disabled when IRC8M is used as system clock or as PLL source */
        if((RCU_SCSS_IRC8M == hals_rcu_system_clock_source_get()) || \
                ((RCU_SCSS_PLL == hals_rcu_system_clock_source_get()) && \
                 (RCU_PLLSRC_IRC8M_DIV2 == (RCU_CFG0 & RCU_CFG0_PLLSEL)))) {
            if((RESET != (RCU_CTL0 & RCU_CTL0_IRC8MSTB)) && (RCU_OSC_OFF == rcu_osci->irc8m.state)) {
                return HAL_ERR_VAL;
            } else {
                if(rcu_osci->irc8m.adjust_value) {
                    /* adjusts the IRC8M calibration value */
                    hals_rcu_irc8m_adjust_value_set(rcu_osci->irc8m.adjust_value);
                }
            }
        } else {
            if(RCU_OSC_OFF != rcu_osci->irc8m.state) {
                hals_rcu_osci_on(RCU_IRC8M);

                /* wait till IRC8M is stable */
                if(ERROR == hals_rcu_osci_stab_wait(RCU_IRC8M)) {
                    return HAL_ERR_TIMEOUT;
                }

                if(rcu_osci->irc8m.adjust_value) {
                    /* adjusts the IRC8M calibration value */
                    hals_rcu_irc8m_adjust_value_set(rcu_osci->irc8m.adjust_value);
                }
            } else {
                uint32_t tick_start = 0U;

                hals_rcu_osci_off(RCU_IRC8M);

                tick_start = hal_sys_basetick_count_get();

                /* wait till IRC8M is disabled */
                while(RESET != (RCU_CTL0 & RCU_CTL0_IRC8MSTB)) {
                    if(SET == hal_sys_basetick_timeout_check(tick_start, HAL_IRC8M_TIMEOUT)) {
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }
        }
    }

    /* configure IRC28M */
    if(ENABLE == rcu_osci->irc28m.need_configure) {
        /* set the IRC28M new state */
        if(RCU_OSC_ON == rcu_osci->irc28m.state) {
            RCU_CFG2 &= ~RCU_CFG2_ADCSEL;

            hals_rcu_osci_on(RCU_IRC28M);

            /* wait till IRC28M is stable */
            if(ERROR == hals_rcu_osci_stab_wait(RCU_IRC28M)) {
                return HAL_ERR_TIMEOUT;
            }

            if(rcu_osci->irc28m.adjust_value) {
                /* adjusts the IRC28M calibration value */
                hals_rcu_irc28m_adjust_value_set(rcu_osci->irc28m.adjust_value);
            }
        } else if(RCU_OSC_ADCCTL == rcu_osci->irc28m.state) {
            RCU_CFG2 &= ~RCU_CFG2_ADCSEL;

            hals_rcu_osci_on(RCU_IRC28M);

            /* wait till IRC28M is stable */
            if(ERROR == hals_rcu_osci_stab_wait(RCU_IRC28M)) {
                return HAL_ERR_TIMEOUT;
            }

            if(rcu_osci->irc28m.adjust_value) {
                /* adjusts the IRC28M calibration value */
                hals_rcu_irc28m_adjust_value_set(rcu_osci->irc28m.adjust_value);
            }
        } else {
            uint32_t tick_start = 0U;

            RCU_CFG2 |= RCU_CFG2_ADCSEL;

            hals_rcu_osci_off(RCU_IRC28M);

            tick_start = hal_sys_basetick_count_get();

            /* wait till IRC28M is disabled */
            while(RESET != (RCU_CTL1 & RCU_CTL1_IRC28MSTB)) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, HAL_IRC28M_TIMEOUT)) {
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
    }

    /* configure IRC48M */
    if(ENABLE == rcu_osci->irc48m.need_configure) {
        /* set the IRC40K new state */
        if(RCU_OSC_ON == rcu_osci->irc48m.state) {
            hals_rcu_osci_on(RCU_IRC48M);

            /* wait till IRC40K is stable */
            if(ERROR == hals_rcu_osci_stab_wait(RCU_IRC48M)) {
                return HAL_ERR_TIMEOUT;
            }
        } else {
            uint32_t tick_start = 0U;

            hals_rcu_osci_off(RCU_IRC48M);

            tick_start = hal_sys_basetick_count_get();

            /* wait till IRC48M is disabled */
            while(RESET != (RCU_ADDCTL & RCU_ADDCTL_IRC48MSTB)) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, HAL_IRC48M_TIMEOUT)) {
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
    }

    /* configure IRC40K */
    if(ENABLE == rcu_osci->irc40k.need_configure) {
        /* set the IRC40K new state */
        if(RCU_OSC_OFF != rcu_osci->irc40k.state) {
            hals_rcu_osci_on(RCU_IRC40K);

            /* wait till IRC40K is stable */
            if(ERROR == hals_rcu_osci_stab_wait(RCU_IRC40K)) {
                return HAL_ERR_TIMEOUT;
            }
        } else {
            uint32_t tick_start = 0U;

            hals_rcu_osci_off(RCU_IRC40K);

            tick_start = hal_sys_basetick_count_get();

            /* wait till IRC40K is disabled */
            while(RESET != (RCU_RSTSCK & RCU_RSTSCK_IRC40KSTB)) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, HAL_IRC40K_TIMEOUT)) {
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
    }

    /* configure PLL */
    if(ENABLE == rcu_osci->pll.need_configure) {
        /* check if the PLL is used as system clock or not */
        if(RCU_OSC_NONE != rcu_osci->pll.state) {
            if(RCU_SCSS_PLL != hals_rcu_system_clock_source_get()) {
                uint32_t tick_start = 0U;

                if(RCU_OSC_ON == rcu_osci->pll.state) {
                    hals_rcu_osci_off(RCU_PLL_CK);

                    tick_start = hal_sys_basetick_count_get();

                    /* wait till PLL is disabled */
                    while(RESET != (RCU_CTL0 & RCU_CTL0_PLLSTB)) {
                        if(SET == hal_sys_basetick_timeout_check(tick_start, HAL_PLL_TIMEOUT)) {
                            return HAL_ERR_TIMEOUT;
                        }
                    }

                    /* configure the PLL */
                    hals_rcu_pll_preselection_config(rcu_osci->pll.pll_presel);
                    hals_rcu_hxtal_prediv_config(rcu_osci->pll.pre_div);
                    hals_rcu_pll_config(rcu_osci->pll.pll_source, rcu_osci->pll.pll_mul);

                    hals_rcu_osci_on(RCU_PLL_CK);

                    /* wait till PLL is stable */
                    if(ERROR == hals_rcu_osci_stab_wait(RCU_PLL_CK)) {
                        return HAL_ERR_TIMEOUT;
                    }
                } else {
                    hals_rcu_osci_off(RCU_PLL_CK);

                    tick_start = hal_sys_basetick_count_get();

                    /* wait till PLL is disabled */
                    while(RESET != (RCU_CTL0 & RCU_CTL0_PLLSTB)) {
                        if(SET == hal_sys_basetick_timeout_check(tick_start, HAL_PLL_TIMEOUT)) {
                            return HAL_ERR_TIMEOUT;
                        }
                    }
                }
            } else {
                return HAL_ERR_VAL;
            }
        }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      configure the clock out to output on CKOUT pin
    \param[in]  ckout_src: clock out source
                  the argument could be selected from enumeraion <hal_rcu_ckout_src_enum>
    \param[in]  ckout_div: clock out divider
                  the argument could be selected from enumeraion <hal_rcu_ckout_div_enum>
    \param[out] none
    \retval
*/
void hal_rcu_clock_out_config(hal_rcu_ckout_src_enum ckout_src, \
                              hal_rcu_ckout_div_enum ckout_div)
{
    uint32_t clkout_src = 0U;

    hal_gpio_init_struct gpio_init_struct;

    _CKOUT_GPIO_CLK_ENABLE();

    /* configure clock out GPIO pin */
    gpio_init_struct.mode = GPIO_MODE_AF_PP;
    gpio_init_struct.ospeed = GPIO_OSPEED_50MHZ;
    gpio_init_struct.pull = GPIO_PULL_NONE;
    gpio_init_struct.af = GPIO_AF_0;

    hal_gpio_init(_CKOUT_GPIO_PORT, _CKOUT_GPIO_PIN, &gpio_init_struct);

    if(RCU_CKOUT_SRC_CKPLL_DIV1 == ckout_src) {
        clkout_src = RCU_CKOUTSRC_CKPLL_DIV1;
    } else {
        clkout_src = CFG0_CKOUTSEL(ckout_src);
    }

    /* clock out configuration */
    hals_rcu_ckout_config(clkout_src, ckout_div);
}

/*!
    \brief      deinitialize the RCU
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rcu_deinit(void)
{
    /* enable IRC8M */
    RCU_CTL0 |= RCU_CTL0_IRC8MEN;
    while(0U == (RCU_CTL0 & RCU_CTL0_IRC8MSTB)) {
    }
    
    RCU_MODIFY(0x50);

    RCU_CFG0 &= ~RCU_CFG0_SCS;

    /* reset CTL register */
    RCU_CTL0 &= ~(RCU_CTL0_HXTALEN | RCU_CTL0_CKMEN | RCU_CTL0_PLLEN | RCU_CTL0_HXTALBPS);
    RCU_CTL1 &= ~RCU_CTL1_IRC28MEN;

    /* reset RCU */
    RCU_CFG0 &= ~(RCU_CFG0_SCS | RCU_CFG0_AHBPSC | RCU_CFG0_APB1PSC | RCU_CFG0_APB2PSC | \
                  RCU_CFG0_ADCPSC | RCU_CFG0_CKOUTSEL | RCU_CFG0_CKOUTDIV | RCU_CFG0_PLLDV);
    RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PLLMF | RCU_CFG0_PLLMF4 | RCU_CFG0_PLLDV);
#if (defined(GD32F350))
    RCU_CFG0 &= ~(RCU_CFG0_USBFSPSC);
    RCU_CFG2 &= ~(RCU_CFG2_CECSEL | RCU_CFG2_USBFSPSC2);
#endif /* GD32F350 */

    RCU_CFG1 &= ~(RCU_CFG1_PREDV | RCU_CFG1_PLLMF5 | RCU_CFG1_PLLPRESEL);
    RCU_CFG2 &= ~(RCU_CFG2_USART0SEL | RCU_CFG2_ADCSEL);
    RCU_CFG2 &= ~RCU_CFG2_IRC28MDIV;
    RCU_CFG2 &= ~RCU_CFG2_ADCPSC2;
    RCU_ADDCTL &= ~RCU_ADDCTL_IRC48MEN;
    RCU_INT = 0x00000000U;
    RCU_ADDINT = 0x00000000U;

    g_systemcoreclock = IRC8M_VALUE;
}

/*!
    \brief      initialize the RCU structure with the default values
    \param[in]  hal_struct_type: the argument could be selected from enumeration <hal_rcu_struct_type_enum>
    \param[in]  p_struct: pointer of the RCU structure
    \param[out] none
    \retval     none
*/
void hal_rcu_struct_init(hal_rcu_struct_type_enum rcu_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer [*p_struct] value is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    switch(rcu_struct_type) {
    case HAL_RCU_CLK_STRUCT:
        /* initialize rcu clock structure with the default values */
        ((hal_rcu_clk_struct *)p_struct)->clock_type = RCU_CLKTYPE_NONE;
        ((hal_rcu_clk_struct *)p_struct)->ahbclk_divider = RCU_SYSCLK_AHBDIV1;
        ((hal_rcu_clk_struct *)p_struct)->apb1clk_divider = RCU_AHBCLK_APB1DIV1;
        ((hal_rcu_clk_struct *)p_struct)->apb2clk_divider = RCU_AHBCLK_APB2DIV1;
        ((hal_rcu_clk_struct *)p_struct)->sysclk_source = RCU_SYSCLK_SRC_IRC8M;
        ((hal_rcu_clk_struct *)p_struct)->ck48mclk_source = RCU_USB_CK48MSRC_IRC48M;
        break;
    case HAL_RCU_OSCI_STRUCT:
        /* initialize rcu oscillator structure with the default values */
        ((hal_rcu_osci_struct *)p_struct)->hxtal.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->hxtal.state = RCU_OSC_NONE;
        ((hal_rcu_osci_struct *)p_struct)->lxtal.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->lxtal.state = RCU_OSC_NONE;
        ((hal_rcu_osci_struct *)p_struct)->irc48m.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->irc48m.state = RCU_OSC_NONE;
        ((hal_rcu_osci_struct *)p_struct)->irc28m.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->irc28m.adjust_value = 0U;
        ((hal_rcu_osci_struct *)p_struct)->irc28m.state = RCU_OSC_NONE;
        ((hal_rcu_osci_struct *)p_struct)->irc8m.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->irc8m.adjust_value = 0U;
        ((hal_rcu_osci_struct *)p_struct)->irc8m.state = RCU_OSC_NONE;
        ((hal_rcu_osci_struct *)p_struct)->irc40k.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->irc40k.state = RCU_OSC_NONE;
        ((hal_rcu_osci_struct *)p_struct)->pll.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->pll.state = RCU_OSC_NONE;
        break;
    case HAL_RCU_PERIPHCLK_STRUCT:
        /* initialize rcu peripheral clock structure with the default values */
        ((hal_rcu_periphclk_struct *)p_struct)->periph_clock_type = RCU_PERIPH_CLKTYPE_NONE;
        ((hal_rcu_periphclk_struct *)p_struct)->adc_clock_source = RCU_ADCCK_IRC28M_DIV2;
        ((hal_rcu_periphclk_struct *)p_struct)->rtc_clock_source = RCU_RTC_CLKSRC_NONE;
        ((hal_rcu_periphclk_struct *)p_struct)->usart0_clock_source = RCU_USART0_CLKSRC_IRC8M;
#if defined(GD32F350)
        ((hal_rcu_periphclk_struct *)p_struct)->cec_clock_source = RCU_CEC_CLKSRC_DIV244;
        ((hal_rcu_periphclk_struct *)p_struct)->usbfs_clock_source = RCU_PLLCLK_USBFS_DIV1;
#endif /* GD32F350 */
        break;
    default:
        break;
    }
}

/*!
    \brief      enable the peripherals clock
    \param[in]  periph: RCU peripherals
                  the argument could be selected from enumeration <rcu_periph_enum>
    \param[out] none
    \retval     none
*/
void hal_rcu_periph_clk_enable(hal_rcu_periph_enum periph)
{
    RCU_REG_VAL(periph) |= BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      disable the peripherals clock
    \param[in]  periph: RCU peripherals
                  the argument could be selected from enumeration <rcu_periph_enum>
    \param[out] none
    \retval     none
*/
void hal_rcu_periph_clk_disable(hal_rcu_periph_enum periph)
{
    RCU_REG_VAL(periph) &= ~BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      enable the HXTAL clock monitor
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rcu_hxtal_clock_monitor_enable(void)
{
    RCU_CTL0 |= RCU_CTL0_CKMEN;
}

/*!
    \brief      disable the HXTAL clock monitor
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rcu_hxtal_clock_monitor_disable(void)
{
    RCU_CTL0 &= ~RCU_CTL0_CKMEN;
}

/*!
    \brief      reset the peripherals
    \param[in]  periph_reset: RCU peripherals reset, refer to rcu_periph_reset_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_GPIOxRST (x=A,B,C,D,F): reset GPIO ports
      \arg        RCU_TSIRST: reset TSI
      \arg        RCU_CFGCMPRST: reset CFGCMP
      \arg        RCU_ADCRST: reset ADC
      \arg        RCU_TIMERxRST (x=0,1,2,5,13,14,15,16): reset TIMER (RCU_TIMER5 only for GD32F350)
      \arg        RCU_SPIxRST (x=0,1): reset SPI
      \arg        RCU_USARTxRST (x=0,1): reset USART
      \arg        RCU_WWDGTRST: reset WWDGT
      \arg        RCU_I2CxRST (x=0,1): reset I2C
      \arg        RCU_USBFSRST: reset USBFS (only for GD32F350)
      \arg        RCU_PMURST: reset PMU
      \arg        RCU_DACRST: reset DAC (only for GD32F350)
      \arg        RCU_CECRST: reset CEC (only for GD32F350)
      \arg        RCU_CTCRST: reset CTC
    \param[out] none
    \retval     none
*/
void hal_rcu_periph_reset_enable(hal_rcu_periph_reset_enum periph_reset)
{
    RCU_REG_VAL(periph_reset) |= BIT(RCU_BIT_POS(periph_reset));
}

/*!
    \brief      disable reset the peripheral
    \param[in]  periph_reset: RCU peripherals reset, refer to rcu_periph_reset_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_GPIOxRST (x=A,B,C,D,F): reset GPIO ports
      \arg        RCU_TSIRST: reset TSI
      \arg        RCU_CFGCMPRST: reset CFGCMP
      \arg        RCU_ADCRST: reset ADC
      \arg        RCU_TIMERxRST (x=0,1,2,5,13,14,15,16): reset TIMER (RCU_TIMER5 only for GD32F350)
      \arg        RCU_SPIxRST (x=0,1,2): reset SPI
      \arg        RCU_USARTxRST (x=0,1): reset USART
      \arg        RCU_WWDGTRST: reset WWDGT
      \arg        RCU_I2CxRST (x=0,1,2): reset I2C
      \arg        RCU_USBFSRST: reset USBFS (only for GD32F350)
      \arg        RCU_PMURST: reset PMU
      \arg        RCU_DACRST: reset DAC (only for GD32F350)
      \arg        RCU_CECRST: reset CEC (only for GD32F350)
      \arg        RCU_CTCRST: reset CTC
    \param[out] none
    \retval     none
*/
void hal_rcu_periph_reset_disable(hal_rcu_periph_reset_enum periph_reset)
{
    RCU_REG_VAL(periph_reset) &= ~BIT(RCU_BIT_POS(periph_reset));
}

/*!
    \brief      initialize the RCU extended peripherals(RTC, Usart0, ADC, USBFS, CEC) clocks
    \param[in]  periph_clk: the pointer of peripheral clock structure
                  periph_clock_type(member):
                  only one parameter can be selected which is shown as below:
      \arg          RCU_PERIPH_CLKTYPE_NONE: no clock type
      \arg          RCU_PERIPH_CLKTYPE_RTC: RTC clock type
      \arg          RCU_PERIPH_CLKTYPE_USART0: usart0 clock type
      \arg          RCU_PERIPH_CLKTYPE_ADC: ADC clock type
      \arg          RCU_PERIPH_CLKTYPE_USBFS: USBFS clock type
      \arg          RCU_PERIPH_CLKTYPE_CEC: USBFS clock type
                  rtc_clock_source(member):
                  only one parameter can be selected which is shown as below:
      \arg          RCU_RTC_CLKSRC_NONE: RTC has no clock source
      \arg          RCU_RTC_CLKSRC_LXTAL: RTC select LXTAL as clock source
      \arg          RCU_RTC_CLKSRC_IRC40K: RTC select IRC40K as clock source
      \arg          RCU_RTC_CLKSRC_HXTAL_DIV32: RTC select HXTAL/32 as clock source
                  usart0_clock_source(member):
                  only one parameter can be selected which is shown as below:
      \arg          RCU_USART0_CLKSRC_APB2: USART0 select APB2 as clock source
      \arg          RCU_USART0_CLKSRC_SYS: USART0 select SYSCLK as clock source
      \arg          RCU_USART0_CLKSRC_LXTAL: USART0 select LXTAL as clock source
      \arg          RCU_USART0_CLKSRC_IRC8M: USART0 select IRC8M as clock source
                  adc_clock_source(member):
                  only one parameter can be selected which is shown as below:
      \arg          RCU_ADCCK_IRC28M_DIV2: ADC clock source select IRC28M/2
      \arg          RCU_ADCCK_IRC28M: ADC clock source select IRC28M
      \arg          RCU_ADCCK_APB2_DIV2: ADC clock source select APB2/2
      \arg          RCU_ADCCK_AHB_DIV3: ADC clock source select AHB/3
      \arg          RCU_ADCCK_APB2_DIV4: ADC clock source select APB2/4
      \arg          RCU_ADCCK_AHB_DIV5: ADC clock source select AHB/5
      \arg          RCU_ADCCK_APB2_DIV6: ADC clock source select APB2/6
      \arg          RCU_ADCCK_AHB_DIV7: ADC clock source select AHB/7
      \arg          RCU_ADCCK_APB2_DIV8: ADC clock source select APB2/8
      \arg          RCU_ADCCK_AHB_DIV9: ADC clock source select AHB/9
                  usbfs_clock_source(member):
                  only one parameter can be selected which is shown as below:
      \arg          RCU_PLLCLK_USBFS_DIV1_5: USBFS clock prescaler select CK_PLL/1.5
      \arg          RCU_PLLCLK_USBFS_DIV1: USBFS clock prescaler select CK_PLL
      \arg          RCU_PLLCLK_USBFS_DIV2_5: USBFS clock prescaler select CK_PLL/2.5
      \arg          RCU_PLLCLK_USBFS_DIV2: USBFS clock prescaler select CK_PLL/2
      \arg          RCU_PLLCLK_USBFS_DIV3_5: USBFS clock prescaler select CK_PLL/3.5
      \arg          RCU_PLLCLK_USBFS_DIV3: USBFS clock prescaler select CK_PLL/3
                  cec_clock_source(member):
                  only one parameter can be selected which is shown as below:
      \arg          RCU_CEC_CLKSRC_DIV244: CK_CEC clock source select IRC8M/244
      \arg          RCU_CEC_CLKSRC_LXTAL: CK_CEC clock source select LXTAL
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_TIMEOUT, details refer to gd32f3x0_hal.h
*/
int32_t hal_rcu_periph_clock_config(hal_rcu_periphclk_struct *periph_clk)
{
    uint32_t backup_reg = 0U;
    FlagStatus pwr_state = RESET;

#if (1 == HAL_PARAMETER_CHECK)

    if(NULL == periph_clk) {
        HAL_DEBUGE("parameter [periph_clk] is a NULL pointer.");

        return HAL_ERR_ADDRESS;
    }

    if(periph_clk->periph_clock_type > RCU_PERIPH_CLKTYPE_MAX) {
        HAL_DEBUGE("parameter member [periph_clk->periph_clock_type] is invalid.");

        return HAL_ERR_VAL;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure RTC clock */
    if(RCU_PERIPH_CLKTYPE_RTC == (periph_clk->periph_clock_type & RCU_PERIPH_CLKTYPE_RTC)) {
        /* need to enable pmu clock */
        if(RESET == (RCU_APB1EN & RCU_APB1EN_PMUEN)) {
            hal_rcu_periph_clk_enable(RCU_PMU);

            pwr_state = SET;
        }

        /* RTC clock need to activate backup domain */
        if(RESET == (PMU_CTL & PMU_CTL_BKPWEN)) {
            uint32_t tick_start = 0U;

            hals_pmu_backup_write_enable();

            tick_start = hal_sys_basetick_count_get();

            while(RESET == (PMU_CTL & PMU_CTL_BKPWEN)) {
                if(SET == hal_sys_basetick_timeout_check(tick_start, HAL_BP_TIMEOUT)) {
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* check if the RTC clock source is modified */
        backup_reg = RCU_BDCTL & RCU_BDCTL_RTCSRC;

        if((0x00000000U != backup_reg) && \
                (backup_reg != (periph_clk->rtc_clock_source & RCU_BDCTL_RTCSRC))) {
            /* store the BDCTL register value before resetting the backup domain */
            backup_reg = RCU_BDCTL & ~RCU_BDCTL_RTCSRC;

            /* reset the backup domain */
            hals_rcu_bkp_reset_enable();
            hals_rcu_bkp_reset_disable();

            RCU_BDCTL = backup_reg;

            if(RESET != (backup_reg & RCU_BDCTL_LXTALEN)) {
                if(ERROR == hals_rcu_osci_stab_wait(RCU_LXTAL)) {
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* configure the RTC clock source */
        hals_rcu_rtc_clock_config(periph_clk->rtc_clock_source);

        if(SET == pwr_state) {
            hal_rcu_periph_clk_disable(RCU_PMU);

            pwr_state = RESET;
        }
    }

    /* configure the USART0 clock source */
    if(RCU_PERIPH_CLKTYPE_USART0 == (periph_clk->periph_clock_type & RCU_PERIPH_CLKTYPE_USART0)) {
        hals_rcu_usart_clock_config(periph_clk->usart0_clock_source);
    }

    /* configure the ADC clock source */
    if(RCU_PERIPH_CLKTYPE_ADC == (periph_clk->periph_clock_type & RCU_PERIPH_CLKTYPE_ADC)) {
        hals_rcu_adc_clock_config(periph_clk->adc_clock_source);
    }

#if defined(GD32F350)
    /* configure the CEC clock source */
    if(RCU_PERIPH_CLKTYPE_CEC == (periph_clk->periph_clock_type & RCU_PERIPH_CLKTYPE_CEC)) {
        hals_rcu_cec_clock_config(periph_clk->cec_clock_source);
    }

    /* configure the USBFS clock source */
    if(RCU_PERIPH_CLKTYPE_USBFS == (periph_clk->periph_clock_type & RCU_PERIPH_CLKTYPE_USBFS)) {
        hals_rcu_usbfs_clock_config(periph_clk->usbfs_clock_source);
    }
#endif /* GD32F350 */

    return HAL_ERR_NONE;
}

/*!
    \brief      get the peripherals clock frequency
    \param[in]  periph_clk:
                only one parameter can be selected which is shown as below:
      \arg        RCU_PERIPH_CLKTYPE_NONE: no clock type
      \arg        RCU_PERIPH_CLKTYPE_RTC: RTC clock type
      \arg        RCU_PERIPH_CLKTYPE_USART0: usart0 clock type
      \arg        RCU_PERIPH_CLKTYPE_ADC: ADC clock type
      \arg        RCU_PERIPH_CLKTYPE_APB1TIMER: APB1 timer clock type
      \arg        RCU_PERIPH_CLKTYPE_APB2TIMER: APB2 timer clock type
      \arg        RCU_PERIPH_CLKTYPE_CEC: CEC clock type
    \param[out] none
    \retval     peripheral clock frequency
*/
uint32_t hal_rcu_periph_clkfreq_get(uint32_t periph_clk)
{
    uint32_t periph_freq = 0U;
    uint32_t src_clk = 0U;

#if (1 == HAL_PARAMETER_CHECK)

    if(periph_clk >= RCU_PERIPH_CLKTYPE_MAX) {
        HAL_DEBUGE("parameter [periph_clk] is invalid.");

        return 0U;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(periph_clk) {
    case RCU_PERIPH_CLKTYPE_RTC:
        /* get the current RTC clock source */
        src_clk = RCU_BDCTL & RCU_BDCTL_RTCSRC;

        if((RCU_RTCSRC_LXTAL == src_clk) && (SET == hals_rcu_flag_get(RCU_FLAG_LXTALSTB))) {
            periph_freq = LXTAL_VALUE;
        } else if((RCU_RTCSRC_IRC40K == src_clk) && (SET == hals_rcu_flag_get(RCU_FLAG_IRC40KSTB))) {
            periph_freq = IRC40K_VALUE;
        } else if((RCU_RTCSRC_HXTAL_DIV32 == src_clk) && (SET == hals_rcu_flag_get(RCU_FLAG_HXTALSTB))) {
            periph_freq = HXTAL_VALUE / 32U;
        } else {
            periph_freq = 0U;
        }
        break;
    case RCU_PERIPH_CLKTYPE_USART0:
        /* get the current USART0 clock source */
        periph_freq = hals_rcu_clock_freq_get(CK_USART);
        break;
    case RCU_PERIPH_CLKTYPE_ADC:
        /* get the current ADC clock source */
        periph_freq = hals_rcu_clock_freq_get(CK_ADC);
        break;
#if defined(GD32F350)
    case RCU_PERIPH_CLKTYPE_CEC:
        /* get the current CEC clock source */
        periph_freq = hals_rcu_clock_freq_get(CK_CEC);
        break;
#endif /* GD32F350 */
    case RCU_PERIPH_CLKTYPE_APB1TIMER:
        /* get the current APB1 TIMER clock source */
        if(RCU_APB1_CKAHB_DIV1 == (RCU_CFG0 & RCU_CFG0_APB1PSC)) {
            periph_freq = hals_rcu_clock_freq_get(CK_APB1);
        } else {
            periph_freq = hals_rcu_clock_freq_get(CK_APB1) * 2;
        }
        break;
    case RCU_PERIPH_CLKTYPE_APB2TIMER:
        /* get the current APB2 TIMER clock source */
        if(RCU_APB2_CKAHB_DIV1 == (RCU_CFG0 & RCU_CFG0_APB2PSC)) {
            periph_freq = hals_rcu_clock_freq_get(CK_APB2);
        } else {
            periph_freq = hals_rcu_clock_freq_get(CK_APB2) * 2;
        }
        break;
    default:
        break;
    }

    return periph_freq;
}

/*!
    \brief      get the RCU oscillators configuration
    \param[in]  rcu_osci: the pointer of the RCU oscillators structure
                  hxtal: HXTAL status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  lxtal: LXTAL status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc28m: IRC28M status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    calibration_value: calibration value
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc8m: IRC8M status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    calibration_value: calibration value
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc40k: IRC40K status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  pll: PLL status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                    pll_source: RCU_PLL_SRC_HXTAL, RCU_PLL_SRC_IRC8M_DIV2
                    pre_div: the argument could be selected from enumeration <hal_rcu_pll_prediv_enum>
                    pll_mul: the argument could be selected from enumeration <hal_rcu_pll_mul_enum>
    \param[out] none
    \retval     none
*/
void hal_rcu_osci_config_get(hal_rcu_osci_struct *rcu_osci)
{
#if (1 == HAL_PARAMETER_CHECK)

    if(NULL == rcu_osci) {
        HAL_DEBUGE("parameter [rcu_osci] is a NULL pointer.");

        return;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure all the oscillator type */
    rcu_osci->hxtal.need_configure = ENABLE;
    rcu_osci->lxtal.need_configure = ENABLE;
    rcu_osci->irc8m.need_configure = ENABLE;
    rcu_osci->irc28m.need_configure = ENABLE;
    rcu_osci->irc48m.need_configure = ENABLE;
    rcu_osci->irc40k.need_configure = ENABLE;
    rcu_osci->pll.need_configure = ENABLE;

    /* get the current HXTAL state */
    if(RESET != (RCU_CTL0 & RCU_CTL0_HXTALBPS)) {
        rcu_osci->hxtal.state = RCU_OSC_BYPASS;
    } else if(RESET != (RCU_CTL0 & RCU_CTL0_HXTALEN)) {
        rcu_osci->hxtal.state = RCU_OSC_ON;
    } else {
        rcu_osci->hxtal.state = RCU_OSC_OFF;
    }

    /* get the current LXTAL state */
    if(RESET != (RCU_BDCTL & RCU_BDCTL_LXTALBPS)) {
        rcu_osci->lxtal.state = RCU_OSC_BYPASS;
    } else if(RESET != (RCU_BDCTL & RCU_BDCTL_LXTALEN)) {
        rcu_osci->lxtal.state = RCU_OSC_ON;
    } else {
        rcu_osci->lxtal.state = RCU_OSC_OFF;
    }

    /* get the current IRC8M state and adjust value */
    if(RESET != (RCU_CTL0 & RCU_CTL0_IRC8MEN)) {
        rcu_osci->irc8m.state = RCU_OSC_ON;
    } else {
        rcu_osci->irc8m.state = RCU_OSC_OFF;
    }

    rcu_osci->irc8m.adjust_value = (uint8_t)((RCU_CTL0 & RCU_CTL0_IRC8MCALIB) >> 8);

    /* get the current IRC28M state and adjust value */
    if(RESET == (RCU_CFG2 & RCU_CFG2_ADCSEL)) {
        rcu_osci->irc28m.state = RCU_OSC_ADCCTL;
    } else {
        if(RESET != (RCU_CTL1 & RCU_CTL1_IRC28MEN)) {
            rcu_osci->irc28m.state = RCU_OSC_ON;
        } else {
            rcu_osci->irc28m.state = RCU_OSC_OFF;
        }
    }

    rcu_osci->irc28m.adjust_value = (uint8_t)((RCU_CTL1 & RCU_CTL1_IRC28MCALIB) >> 8);

    /* get the current IRC48M state */

    if(RESET != (RCU_ADDCTL & RCU_ADDCTL_IRC48MEN)) {
        rcu_osci->irc48m.state = RCU_OSC_ON;
    } else {
        rcu_osci->irc48m.state = RCU_OSC_OFF;
    }


    /* get the current IRC40K state */
    if(RESET != (RCU_RSTSCK & RCU_RSTSCK_IRC40KEN)) {
        rcu_osci->irc40k.state = RCU_OSC_ON;
    } else {
        rcu_osci->irc40k.state = RCU_OSC_OFF;
    }

    /* get the current PLL state */
    if(RESET != (RCU_CTL0 & RCU_CTL0_PLLEN)) {
        rcu_osci->pll.state = RCU_OSC_ON;
    } else {
        rcu_osci->pll.state = RCU_OSC_OFF;
    }

    /* get the PLL parameters */
    rcu_osci->pll.pll_mul = (hal_rcu_pll_mul_enum)(RCU_CFG0 & RCU_CFG0_PLLMF);
    rcu_osci->pll.pll_source = (hal_rcu_pll_src_enum)(RCU_CFG0 & RCU_CFG0_PLLSEL);
    rcu_osci->pll.pre_div = (hal_rcu_pll_prediv_enum)(RCU_CFG0 & RCU_CFG0_PLLPREDV);
    rcu_osci->pll.pll_presel = (hal_rcu_pll_presel_enum)(RCU_CFG1 & RCU_CFG1_PLLPRESEL);
}

/*!
    \brief      configure the RCU clock
    \param[in]  rcu_clk: the pointer of the RCU clock structure
                  clock_type(member):
                  only one parameter can be selected which is shown as below:
      \arg          RCU_CLKTYPE_NONE: no clock type
      \arg          RCU_CLKTYPE_SYSCLK: system clock type
      \arg          RCU_CLKTYPE_CK48MCLK: ck48m clock to configure
      \arg          RCU_CLKTYPE_AHBCLK: AHB bus clock type
      \arg          RCU_CLKTYPE_APB1CLK: APB1 bus clock type
      \arg          RCU_CLKTYPE_APB2CLK: APB2 bus clock type
                  sysclk_source(member):
                  only one parameter can be selected which is shown as below:
      \arg          RCU_SYSCLK_SRC_IRC8M: IRC8M as system clock source
      \arg          RCU_SYSCLK_SRC_HXTAL: HXTAL as system clock source
      \arg          RCU_SYSCLK_SRC_PLL: PLL as system clock source
                  ck48mclk_source(member):
                  only one parameter can be selected which is shown as below:
      \arg          RCU_USB_CK48MSRC_PLL48M: CK48M source clock select PLL48M
      \arg          RCU_USB_CK48MSRC_IRC48M: CK48M source clock select IRC48M
                  ahbclk_divider(member): AHB clock divider
                    the argument could be selected from enumeration <hal_rcu_sysclk_ahbdiv_enum>
                  apb1clk_divider(member): APB1 clock divider
                    the argument could be selected from enumeration <hal_rcu_sysclk_apb1div_enum>
                  apb2clk_divider(member): APB2 clock divider
                    the argument could be selected from enumeration <hal_rcu_sysclk_apb2div_enum>
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_TIMEOUT, details refer to gd32f3x0_hal.h
    \note       user need compute the AHB clock freuuency before, then decide the fmc_wscnt value
*/

int32_t hal_rcu_clock_config(hal_rcu_clk_struct *rcu_clk)
{
#if (1 == HAL_PARAMETER_CHECK)

    if(NULL == rcu_clk) {
        HAL_DEBUGE("parameter [rcu_clk] is a NULL pointer.");

        return HAL_ERR_ADDRESS;
    }

    if(rcu_clk->clock_type > RCU_CLKTYPE_MAX) {
        HAL_DEBUGE("parameter member [rcu_clk->clock_type] is invalid.");

        return HAL_ERR_VAL;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure system clock */
    if(RCU_CLKTYPE_SYSCLK == (rcu_clk->clock_type & RCU_CLKTYPE_SYSCLK)) {
        uint32_t tick_start = 0U;
        uint32_t time_out = s_rcu_timeout[_RCU_CKSYSSRC_INDEX(rcu_clk->sysclk_source)];

        if(RESET == hals_rcu_flag_get(s_rcu_stab_flag[_RCU_CKSYSSRC_INDEX(rcu_clk->sysclk_source)])) {
            return HAL_ERR_VAL;
        }

        /* configure the system clock source */
        hals_rcu_system_clock_source_config(rcu_clk->sysclk_source);

        tick_start = hal_sys_basetick_count_get();

        /* wait till system clock source is stable */
        while(s_rcu_scss[_RCU_CKSYSSRC_INDEX(rcu_clk->sysclk_source)] != hals_rcu_system_clock_source_get()) {
            if(SET == hal_sys_basetick_timeout_check(tick_start, time_out)) {
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    /* configure ck48m clock */
    if(RCU_CLKTYPE_CK48MCLK == (rcu_clk->clock_type & RCU_CLKTYPE_CK48MCLK)) {
        /* configure ck48m clock */
        hals_rcu_ck48m_clock_config(rcu_clk->ck48mclk_source);
    }

    /* configure AHB bus clock */
    if(RCU_CLKTYPE_AHBCLK == (rcu_clk->clock_type & RCU_CLKTYPE_AHBCLK)) {
        /* configure AHB clock */
        hals_rcu_ahb_clock_config(rcu_clk->ahbclk_divider);
    }

    /* configure APB1 bus clock */
    if(RCU_CLKTYPE_APB1CLK == (rcu_clk->clock_type & RCU_CLKTYPE_APB1CLK)) {
        hals_rcu_apb1_clock_config(rcu_clk->apb1clk_divider);
    }

    /* configure APB2 bus clock */
    if(RCU_CLKTYPE_APB2CLK == (rcu_clk->clock_type & RCU_CLKTYPE_APB2CLK)) {
        hals_rcu_apb2_clock_config(rcu_clk->apb2clk_divider);
    }

    /* update the SystemCoreClock global variable */
    g_systemcoreclock = hals_rcu_clock_freq_get(CK_SYS);

    /* configure the source of time base considering new system clocks settings */
    hal_sys_timesource_init(g_basetick_source);

    return HAL_ERR_NONE;
}

/*!
    \brief      get the RCU clock configuration
    \param[in]  rcu_clk: the pointer of the RCU clock structure
                  clock_type(member):
                  only one parameter can be selected which is shown as below:
      \arg          RCU_CLKTYPE_NONE: no clock type
      \arg          RCU_CLKTYPE_SYSCLK: system clock type
      \arg          RCU_CLKTYPE_CK48MCLK: ck48m clock to configure
      \arg          RCU_CLKTYPE_AHBCLK: AHB bus clock type
      \arg          RCU_CLKTYPE_APB1CLK: APB1 bus clock type
      \arg          RCU_CLKTYPE_APB2CLK: APB2 bus clock type
                  sysclk_source(member):
                  only one parameter can be selected which is shown as below:
      \arg          RCU_SYSCLK_SRC_IRC8M: IRC8M as system clock source
      \arg          RCU_SYSCLK_SRC_HXTAL: HXTAL as system clock source
      \arg          RCU_SYSCLK_SRC_PLL: PLL as system clock source
                  ahbclk_divider(member): AHB clock divider
                    the argument could be selected from enumeration <hal_rcu_sysclk_ahbdiv_enum>
                  apb1clk_divider(member): APB1 clock divider
                    the argument could be selected from enumeration <hal_rcu_sysclk_apb1div_enum>
                  apb2clk_divider(member): APB2 clock divider
                    the argument could be selected from enumeration <hal_rcu_sysclk_apb2div_enum>
    \param[out] none
    \retval     none
*/
void hal_rcu_clock_config_get(hal_rcu_clk_struct *rcu_clk)
{
#if (1 == HAL_PARAMETER_CHECK)

    if(NULL == rcu_clk) {
        HAL_DEBUGE("parameter [rcu_clk] is a NULL pointer.");

        return;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    /* get all clock type configuration */
    rcu_clk->clock_type = RCU_CLKTYPE_SYSCLK | RCU_CLKTYPE_CK48MCLK | RCU_CLKTYPE_AHBCLK | RCU_CLKTYPE_APB1CLK | RCU_CLKTYPE_APB2CLK;
    rcu_clk->sysclk_source = (hal_rcu_sysclk_src_enum)(RCU_CFG0 & RCU_CFG0_SCS);
    rcu_clk->ck48mclk_source = (hal_rcu_ck48mclk_src_enum)(RCU_ADDCTL & RCU_ADDCTL_CK48MSEL);
    rcu_clk->ahbclk_divider = (hal_rcu_sysclk_ahbdiv_enum)(RCU_CFG0 & RCU_CFG0_AHBPSC);
    rcu_clk->apb1clk_divider = (hal_rcu_ahbclk_apbldiv_enum)(RCU_CFG0 & RCU_CFG0_APB1PSC);
    rcu_clk->apb2clk_divider = (hal_rcu_ahbclk_apb2div_enum)(RCU_CFG0 & RCU_CFG0_APB2PSC);
}

/*!
    \brief      update the SystemCoreClock with current core clock retrieved from cpu registers
    \param[in]  none
    \param[out] none
    \retval     none
*/
int32_t hal_SystemCoreClockUpdate(void)
{
    int32_t sws = 0U;
    int32_t pllmf = 0U, pllmf4 = 0U, pllmf5 = 0U, pllsel = 0U, pllpresel = 0U, prediv = 0U, idx = 0U, clk_exp = 0U;
    /* exponent of AHB clock divider */
    const uint8_t ahb_exp[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

    sws = GET_BITS(RCU_CFG0, 2, 3);
    switch(sws) {
    /* IRC8M is selected as CK_SYS */
    case SEL_IRC8M:
        g_systemcoreclock = IRC8M_VALUE;
        break;
    /* HXTAL is selected as CK_SYS */
    case SEL_HXTAL:
        g_systemcoreclock = HXTAL_VALUE;
        break;
    /* PLL is selected as CK_SYS */
    case SEL_PLL:
        /* get the value of PLLMF[3:0] */
        pllmf = GET_BITS(RCU_CFG0, 18, 21);
        pllmf4 = GET_BITS(RCU_CFG0, 27, 27);
        pllmf5 = GET_BITS(RCU_CFG1, 31, 31);
        /* high 16 bits */
        if((0U == pllmf4) && (0U == pllmf5)) {
            pllmf += 2U;
        }
        if((1U == pllmf4) && (0U == pllmf5)) {
            pllmf += 17U;
        }
        if((0U == pllmf4) && (1U == pllmf5)) {
            pllmf += 33U;
        }
        if((1U == pllmf4) && (1U == pllmf5)) {
            pllmf += 49U;
        }
        /* PLL clock source selection, HXTAL or IRC8M/2 */
        pllsel = GET_BITS(RCU_CFG0, 16, 16);
        if(0U != pllsel) {
            prediv = (GET_BITS(RCU_CFG1, 0, 3) + 1U);
            if(0U == pllpresel) {
                g_systemcoreclock = (HXTAL_VALUE / prediv) * pllmf;
            } else {
                g_systemcoreclock = (IRC48M_VALUE / prediv) * pllmf;
            }
        } else {
            g_systemcoreclock = (IRC8M_VALUE >> 1) * pllmf;
        }
        break;
    /* IRC8M is selected as CK_SYS */
    default:
        g_systemcoreclock = IRC8M_VALUE;
        break;
    }
    /* calculate AHB clock frequency */
    idx = GET_BITS(RCU_CFG0, 4, 7);
    clk_exp = ahb_exp[idx];
    g_systemcoreclock >>= clk_exp;

    return g_systemcoreclock;
}

/*!
    \brief      RCU interrupt handler content function,which is merely used in RCU_handler
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rcu_irq(void)
{
    /* IRC40K stable interrupt handler */
    if(SET == hals_rcu_interrupt_flag_get(RCU_INT_FLAG_IRC40KSTB)) {
        hals_rcu_interrupt_flag_clear(RCU_INT_FLAG_IRC40KSTB_CLR);
        if(NULL != rcu_irq.irc40k_stable_handle) {
            rcu_irq.irc40k_stable_handle(NULL);
        }
    }

    /* IRC8M stable interrupt handler */
    if(SET == hals_rcu_interrupt_flag_get(RCU_INT_FLAG_IRC8MSTB)) {
        hals_rcu_interrupt_flag_clear(RCU_INT_FLAG_IRC8MSTB_CLR);
        if(NULL != rcu_irq.irc8m_stable_handle) {
            rcu_irq.irc8m_stable_handle(NULL);
        }
    }

    /* IRC28M stable interrupt handler */
    if(SET == hals_rcu_interrupt_flag_get(RCU_INT_FLAG_IRC28MSTB)) {
        hals_rcu_interrupt_flag_clear(RCU_INT_FLAG_IRC28MSTB_CLR);
        if(NULL != rcu_irq.irc28m_stable_handle) {
            rcu_irq.irc28m_stable_handle(NULL);
        }
    }

    /* IRC48M stable interrupt handler */
    if(SET == hals_rcu_interrupt_flag_get(RCU_INT_FLAG_IRC48MSTB)) {
        hals_rcu_interrupt_flag_clear(RCU_INT_FLAG_IRC48MSTB_CLR);
        if(NULL != rcu_irq.irc48m_stable_handle) {
            rcu_irq.irc48m_stable_handle(NULL);
        }
    }

    /* LXTAL stable interrupt handler */
    if(SET == hals_rcu_interrupt_flag_get(RCU_INT_FLAG_LXTALSTB)) {
        hals_rcu_interrupt_flag_clear(RCU_INT_FLAG_LXTALSTB_CLR);
        if(NULL != rcu_irq.lxtal_stable_handle) {
            rcu_irq.lxtal_stable_handle(NULL);
        }
    }

    /* HXTAL stable interrupt handler */
    if(SET == hals_rcu_interrupt_flag_get(RCU_INT_FLAG_HXTALSTB)) {
        hals_rcu_interrupt_flag_clear(RCU_INT_FLAG_HXTALSTB_CLR);
        if(NULL != rcu_irq.hxtal_stable_handle) {
            rcu_irq.hxtal_stable_handle(NULL);
        }
    }

    /* PLL stable interrupt handler */
    if(SET == hals_rcu_interrupt_flag_get(RCU_INT_FLAG_PLLSTB)) {
        hals_rcu_interrupt_flag_clear(RCU_INT_FLAG_PLLSTB_CLR);
        if(NULL != rcu_irq.pll_stable_handle) {
            rcu_irq.pll_stable_handle(NULL);
        }
    }

    /* HXTAL stuck interrupt handler */
    if(SET == hals_rcu_interrupt_flag_get(RCU_INT_FLAG_CKM)) {
        hals_rcu_interrupt_flag_clear(RCU_INT_FLAG_CKM_CLR);
        if(NULL != rcu_irq.hxtal_stuck_handle) {
            rcu_irq.hxtal_stuck_handle(NULL);
        }
    }
}

/*!
    \brief      set user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  prcu_irq: pointer of RCU interrupt handler structure
    \param[out] none
    \retval     none
*/
void hal_rcu_irq_handle_set(hal_rcu_irq_struct *prcu_irq)
{
#if (1 == HAL_PARAMETER_CHECK)

    if(NULL == prcu_irq) {
        HAL_DEBUGE("parameter [prcu_irq] is a NULL pointer.");

        return;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    /* HXTAL stable interrupt handler set */
    if(NULL != prcu_irq->hxtal_stable_handle) {
        rcu_irq.hxtal_stable_handle = prcu_irq->hxtal_stable_handle;

        hals_rcu_interrupt_enable(RCU_INT_HXTALSTB);
    } else {
        rcu_irq.hxtal_stable_handle = NULL;

        hals_rcu_interrupt_disable(RCU_INT_HXTALSTB);
    }

    /* IRC28M stable interrupt handler set */
    if(NULL != prcu_irq->irc28m_stable_handle) {
        rcu_irq.irc28m_stable_handle = prcu_irq->irc28m_stable_handle;

        hals_rcu_interrupt_enable(RCU_INT_IRC28MSTB);
    } else {
        rcu_irq.irc28m_stable_handle = NULL;

        hals_rcu_interrupt_disable(RCU_INT_IRC28MSTB);
    }

    /* IRC48M stable interrupt handler set */
    if(NULL != prcu_irq->irc48m_stable_handle) {
        rcu_irq.irc48m_stable_handle = prcu_irq->irc48m_stable_handle;

        hals_rcu_interrupt_enable(RCU_INT_IRC48MSTB);
    } else {
        rcu_irq.irc48m_stable_handle = NULL;

        hals_rcu_interrupt_disable(RCU_INT_IRC48MSTB);
    }

    /* IRC40K stable interrupt handler set */
    if(NULL != prcu_irq->irc40k_stable_handle) {
        rcu_irq.irc40k_stable_handle = prcu_irq->irc40k_stable_handle;

        hals_rcu_interrupt_enable(RCU_INT_IRC40KSTB);
    } else {
        rcu_irq.irc40k_stable_handle = NULL;

        hals_rcu_interrupt_disable(RCU_INT_IRC40KSTB);
    }

    /* IRC8M stable interrupt handler set */
    if(NULL != prcu_irq->irc8m_stable_handle) {
        rcu_irq.irc8m_stable_handle = prcu_irq->irc8m_stable_handle;

        hals_rcu_interrupt_enable(RCU_INT_IRC8MSTB);
    } else {
        rcu_irq.irc8m_stable_handle = NULL;

        hals_rcu_interrupt_disable(RCU_INT_IRC8MSTB);
    }

    /* LXTAL stable interrupt handler set */
    if(NULL != prcu_irq->lxtal_stable_handle) {
        rcu_irq.lxtal_stable_handle = prcu_irq->lxtal_stable_handle;

        hals_rcu_interrupt_enable(RCU_INT_LXTALSTB);
    } else {
        rcu_irq.lxtal_stable_handle = NULL;

        hals_rcu_interrupt_disable(RCU_INT_LXTALSTB);
    }

    /* PLL stable interrupt handler set */
    if(NULL != prcu_irq->pll_stable_handle) {
        rcu_irq.pll_stable_handle = prcu_irq->pll_stable_handle;

        hals_rcu_interrupt_enable(RCU_INT_PLLSTB);
    } else {
        rcu_irq.pll_stable_handle = NULL;

        hals_rcu_interrupt_disable(RCU_INT_PLLSTB);
    }

    /* HXTAL stuck interrupt handler set */
    if(NULL != prcu_irq->hxtal_stuck_handle) {
        rcu_irq.hxtal_stuck_handle = prcu_irq->hxtal_stuck_handle;
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rcu_irq_handle_all_reset(void)
{
    /* reset all interrupt handler */
    rcu_irq.hxtal_stable_handle = NULL;
    rcu_irq.hxtal_stuck_handle = NULL;
    rcu_irq.irc48m_stable_handle = NULL;
    rcu_irq.irc28m_stable_handle = NULL;
    rcu_irq.irc40k_stable_handle = NULL;
    rcu_irq.irc8m_stable_handle = NULL;
    rcu_irq.lxtal_stable_handle = NULL;
    rcu_irq.pll_stable_handle = NULL;
}

/*!
    \brief      reset the BKP
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_rcu_bkp_reset_enable(void)
{
    RCU_BDCTL |= RCU_BDCTL_BKPRST;
}

/*!
    \brief      disable the BKP reset
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_rcu_bkp_reset_disable(void)
{
    RCU_BDCTL &= ~RCU_BDCTL_BKPRST;
}

/*!
    \brief      get the clock stabilization and periphral reset flags
    \param[in]  flag: the clock stabilization and periphral reset flags, refer to rcu_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_FLAG_IRC40KSTB: IRC40K stabilization flag
      \arg        RCU_FLAG_LXTALSTB: LXTAL stabilization flag
      \arg        RCU_FLAG_IRC8MSTB: IRC8M stabilization flag
      \arg        RCU_FLAG_HXTALSTB: HXTAL stabilization flag
      \arg        RCU_FLAG_PLLSTB: PLL stabilization flag
      \arg        RCU_FLAG_IRC28MSTB: IRC28M stabilization flag
      \arg        RCU_FLAG_IRC48MSTB: IRC48M stabilization flag
      \arg        RCU_FLAG_V12RST: V12 domain power reset flag
      \arg        RCU_FLAG_OBLRST: option byte loader reset flag
      \arg        RCU_FLAG_EPRST: external pin reset flag
      \arg        RCU_FLAG_PORRST: power reset flag
      \arg        RCU_FLAG_SWRST: software reset flag
      \arg        RCU_FLAG_FWDGTRST: free watchdog timer reset flag
      \arg        RCU_FLAG_WWDGTRST: window watchdog timer reset flag
      \arg        RCU_FLAG_LPRST: low-power reset flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_rcu_flag_get(hal_rcu_flag_enum flag)
{
    if(RESET != (RCU_REG_VAL(flag) & BIT(RCU_BIT_POS(flag)))) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear the reset flag
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_rcu_all_reset_flag_clear(void)
{
    RCU_RSTSCK |= RCU_RSTSCK_RSTFC;
}

/*!
    \brief      wait until oscillator stabilization flags is SET
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
      \arg        RCU_IRC8M: IRC8M
      \arg        RCU_IRC28M: IRC28M
      \arg        RCU_IRC48M: IRC48M
      \arg        RCU_IRC40K: IRC40K
      \arg        RCU_PLL_CK: PLL
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus hals_rcu_osci_stab_wait(hal_rcu_osci_type_enum osci)
{
    uint32_t stb_cnt = 0U;
    ErrStatus reval = ERROR;
    FlagStatus osci_stat = RESET;
    switch(osci) {
    case RCU_HXTAL:
        /* wait until HXTAL is stabilization and osci_stat is not more than timeout */
        while((RESET == osci_stat) && (HXTAL_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = hals_rcu_flag_get(RCU_FLAG_HXTALSTB);
            stb_cnt++;
        }
        if(RESET != hals_rcu_flag_get(RCU_FLAG_HXTALSTB)) {
            reval = SUCCESS;
        }
        break;
    /* wait LXTAL stable */
    case RCU_LXTAL:
        while((RESET == osci_stat) && (LXTAL_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = hals_rcu_flag_get(RCU_FLAG_LXTALSTB);
            stb_cnt++;
        }

        /* check whether flag is set or not */
        if(RESET != hals_rcu_flag_get(RCU_FLAG_LXTALSTB)) {
            reval = SUCCESS;
        }
        break;

    /* wait IRC8M stable */
    case RCU_IRC8M:
        while((RESET == osci_stat) && (IRC8M_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = hals_rcu_flag_get(RCU_FLAG_IRC8MSTB);
            stb_cnt++;
        }

        /* check whether flag is set or not */
        if(RESET != hals_rcu_flag_get(RCU_FLAG_IRC8MSTB)) {
            reval = SUCCESS;
        }
        break;

    /* wait IRC28M stable */
    case RCU_IRC28M:
        while((RESET == osci_stat) && (OSC_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = hals_rcu_flag_get(RCU_FLAG_IRC28MSTB);
            stb_cnt++;
        }

        /* check whether flag is set or not */
        if(RESET != hals_rcu_flag_get(RCU_FLAG_IRC28MSTB)) {
            reval = SUCCESS;
        }
        break;
    /* wait IRC48M stable */
    case RCU_IRC48M:
        while((RESET == osci_stat) && (OSC_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = hals_rcu_flag_get(RCU_FLAG_IRC48MSTB);
            stb_cnt++;
        }

        /* check whether flag is set or not */
        if(RESET != hals_rcu_flag_get(RCU_FLAG_IRC48MSTB)) {
            reval = SUCCESS;
        }
        break;

    /* wait IRC40K stable */
    case RCU_IRC40K:
        while((RESET == osci_stat) && (OSC_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = hals_rcu_flag_get(RCU_FLAG_IRC40KSTB);
            stb_cnt++;
        }

        /* check whether flag is set or not */
        if(RESET != hals_rcu_flag_get(RCU_FLAG_IRC40KSTB)) {
            reval = SUCCESS;
        }
        break;

    /* wait PLL stable */
    case RCU_PLL_CK:
        while((RESET == osci_stat) && (OSC_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = hals_rcu_flag_get(RCU_FLAG_PLLSTB);
            stb_cnt++;
        }

        /* check whether flag is set or not */
        if(RESET != hals_rcu_flag_get(RCU_FLAG_PLLSTB)) {
            reval = SUCCESS;
        }
        break;

    default:
        break;
    }
    /* return value */
    return reval;
}

/*!
    \brief      configure the RTC clock source selection
    \param[in]  rtc_clock_source: RTC clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_RTCSRC_NONE: no clock selected
      \arg        RCU_RTCSRC_LXTAL: CK_LXTAL selected as RTC source clock
      \arg        RCU_RTCSRC_IRC40K: CK_IRC40K selected as RTC source clock
      \arg        RCU_RTCSRC_HXTAL_DIV32: CK_HXTAL/32 selected as RTC source clock
    \param[out] none
    \retval     none
*/
void hals_rcu_rtc_clock_config(uint32_t rtc_clock_source)
{
    /* reset the RTCSRC bits and set according to rtc_clock_source */
    RCU_BDCTL &= ~RCU_BDCTL_RTCSRC;
    RCU_BDCTL |= rtc_clock_source;
}

/*!
    \brief      configure the USART clock source selection
    \param[in]  ck_usart: USART clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_USART0SRC_CKAPB2: CK_USART0 select CK_APB2
      \arg        RCU_USART0SRC_CKSYS: CK_USART0 select CK_SYS
      \arg        RCU_USART0SRC_LXTAL: CK_USART0 select CK_LXTAL
      \arg        RCU_USART0SRC_IRC8M: CK_USART0 select CK_IRC8M
    \param[out] none
    \retval     none
*/
void hals_rcu_usart_clock_config(uint32_t ck_usart)
{
    /* reset the USART0SEL bits and set according to ck_usart */
    RCU_CFG2 &= ~RCU_CFG2_USART0SEL;
    RCU_CFG2 |= ck_usart;
}

/*!
    \brief      configure the ADC clock prescaler selection
    \param[in]  ck_adc: ADC clock prescaler selection, refer to rcu_adc_clock_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_ADCCK_IRC28M_DIV2: select CK_IRC28M/2 as CK_ADC
      \arg        RCU_ADCCK_IRC28M: select CK_IRC28M as CK_ADC
      \arg        RCU_ADCCK_APB2_DIV2: select CK_APB2/2 as CK_ADC
      \arg        RCU_ADCCK_AHB_DIV3: select CK_AHB/3 as CK_ADC
      \arg        RCU_ADCCK_APB2_DIV4: select CK_APB2/4 as CK_ADC
      \arg        RCU_ADCCK_AHB_DIV5: select CK_AHB/5 as CK_ADC
      \arg        RCU_ADCCK_APB2_DIV6: select CK_APB2/6 as CK_ADC
      \arg        RCU_ADCCK_AHB_DIV7: select CK_AHB/7 as CK_ADC
      \arg        RCU_ADCCK_APB2_DIV8: select CK_APB2/8 as CK_ADC
      \arg        RCU_ADCCK_AHB_DIV9: select CK_AHB/9 as CK_ADC
    \param[out] none
    \retval     none
*/
void hals_rcu_adc_clock_config(hal_rcu_adc_clksrc_enum ck_adc)
{
    /* reset the ADCPSC, ADCSEL, IRC28MDIV bits */
    RCU_CFG0 &= ~RCU_CFG0_ADCPSC;
    RCU_CFG2 &= ~(RCU_CFG2_ADCSEL | RCU_CFG2_IRC28MDIV | RCU_CFG2_ADCPSC2);

    /* set the ADC clock according to ck_adc */
    switch(ck_adc) {
    case RCU_ADCCK_IRC28M_DIV2:
        RCU_CFG2 &= ~RCU_CFG2_IRC28MDIV;
        RCU_CFG2 &= ~RCU_CFG2_ADCSEL;
        break;
    case RCU_ADCCK_IRC28M:
        RCU_CFG2 |= RCU_CFG2_IRC28MDIV;
        RCU_CFG2 &= ~RCU_CFG2_ADCSEL;
        break;
    case RCU_ADCCK_APB2_DIV2:
        RCU_CFG0 |= RCU_ADC_CKAPB2_DIV2;
        RCU_CFG2 |= RCU_CFG2_ADCSEL;
        break;
    case RCU_ADCCK_AHB_DIV3:
        RCU_CFG0 |= RCU_ADC_CKAPB2_DIV2;
        RCU_CFG2 |= RCU_CFG2_ADCPSC2;
        RCU_CFG2 |= RCU_CFG2_ADCSEL;
        break;
    case RCU_ADCCK_APB2_DIV4:
        RCU_CFG0 |= RCU_ADC_CKAPB2_DIV4;
        RCU_CFG2 |= RCU_CFG2_ADCSEL;
        break;
    case RCU_ADCCK_AHB_DIV5:
        RCU_CFG0 |= RCU_ADC_CKAPB2_DIV4;
        RCU_CFG2 |= RCU_CFG2_ADCPSC2;
        RCU_CFG2 |= RCU_CFG2_ADCSEL;
        break;
    case RCU_ADCCK_APB2_DIV6:
        RCU_CFG0 |= RCU_ADC_CKAPB2_DIV6;
        RCU_CFG2 |= RCU_CFG2_ADCSEL;
        break;
    case RCU_ADCCK_AHB_DIV7:
        RCU_CFG0 |= RCU_ADC_CKAPB2_DIV6;
        RCU_CFG2 |= RCU_CFG2_ADCPSC2;
        RCU_CFG2 |= RCU_CFG2_ADCSEL;
        break;
    case RCU_ADCCK_APB2_DIV8:
        RCU_CFG0 |= RCU_ADC_CKAPB2_DIV8;
        RCU_CFG2 |= RCU_CFG2_ADCSEL;
        break;
    case RCU_ADCCK_AHB_DIV9:
        RCU_CFG0 |= RCU_ADC_CKAPB2_DIV8;
        RCU_CFG2 |= RCU_CFG2_ADCPSC2;
        RCU_CFG2 |= RCU_CFG2_ADCSEL;
        break;
    default:
        break;
    }
}

/*!
    \brief      configure the CEC clock source selection
    \param[in]  ck_cec: CEC clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_CECSRC_IRC8M_DIV244: CK_CEC select CK_IRC8M/244
      \arg        RCU_CECSRC_LXTAL: CK_CEC select CK_LXTAL
    \param[out] none
    \retval     none
*/
void hals_rcu_cec_clock_config(uint32_t ck_cec)
{
    /* reset the CECSEL bit and set according to ck_cec */
    RCU_CFG2 &= ~RCU_CFG2_CECSEL;
    RCU_CFG2 |= ck_cec;
}

/*!
    \brief      configure the USBFS clock prescaler selection
    \param[in]  ck_usbfs: USBFS clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_USBFS_CKPLL_DIV1_5: select CK_PLL/1.5 as CK_USBFS
      \arg        RCU_USBFS_CKPLL_DIV1: select CK_PLL as CK_USBFS
      \arg        RCU_USBFS_CKPLL_DIV2_5: select CK_PLL/2.5 as CK_USBFS
      \arg        RCU_USBFS_CKPLL_DIV2: select CK_PLL/2 as CK_USBFS
      \arg        RCU_USBFS_CKPLL_DIV3: select CK_PLL/3 as CK_USBFS
      \arg        RCU_USBFS_CKPLL_DIV3_5: select CK_PLL/3.5 as CK_USBFS
    \param[out] none
    \retval     none
*/
void hals_rcu_usbfs_clock_config(uint32_t ck_usbfs)
{
    /* reset the USBFSPSC bits and set according to ck_usbfs */
    RCU_CFG0 &= ~RCU_CFG0_USBFSPSC;
    RCU_CFG2 &= ~RCU_CFG2_USBFSPSC2;

    RCU_CFG0 |= (ck_usbfs & (~RCU_CFG2_USBFSPSC2));
    RCU_CFG2 |= (ck_usbfs & RCU_CFG2_USBFSPSC2);
}

/*!
    \brief      get the system clock, bus and peripheral clock frequency
    \param[in]  clock: the clock frequency which to get
                only one parameter can be selected which is shown as below:
      \arg        CK_SYS: system clock frequency
      \arg        CK_AHB: AHB clock frequency
      \arg        CK_APB1: APB1 clock frequency
      \arg        CK_APB2: APB2 clock frequency
      \arg        CK_ADC: ADC clock frequency
      \arg        CK_CEC: CEC clock frequency
      \arg        CK_USART: USART clock frequency
    \param[out] none
    \retval     clock frequency of system, AHB, APB1, APB2, ADC, CEC or USRAT
*/
uint32_t hals_rcu_clock_freq_get(hal_rcu_clock_freq_enum clock)
{
    uint32_t sws = 0U, adcps = 0U, adcps2 = 0U, ck_freq = 0U;
    uint32_t cksys_freq = 0U, ahb_freq = 0U, apb1_freq = 0U, apb2_freq = 0U;
    uint32_t adc_freq = 0U, cec_freq = 0U, usart_freq = 0U;
    uint32_t pllmf = 0U, pllmf4 = 0U, pllmf5 = 0U, pllsel = 0U, pllpresel = 0U, prediv = 0U, idx = 0U, clk_exp = 0U;
    /* exponent of AHB, APB1 and APB2 clock divider */
    const uint8_t ahb_exp[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    const uint8_t apb1_exp[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    const uint8_t apb2_exp[8] = {0, 0, 0, 0, 1, 2, 3, 4};

    sws = GET_BITS(RCU_CFG0, 2, 3);
    switch(sws) {
    /* IRC8M is selected as CK_SYS */
    case SEL_IRC8M:
        cksys_freq = IRC8M_VALUE;
        break;
    /* HXTAL is selected as CK_SYS */
    case SEL_HXTAL:
        cksys_freq = HXTAL_VALUE;
        break;
    /* PLL is selected as CK_SYS */
    case SEL_PLL:
        /* get the value of PLLMF[3:0] */
        pllmf = GET_BITS(RCU_CFG0, 18, 21);
        pllmf4 = GET_BITS(RCU_CFG0, 27, 27);
        pllmf5 = GET_BITS(RCU_CFG1, 31, 31);
        /* high 16 bits */
        /* high 16 bits */
        if((0U == pllmf4) && (0U == pllmf5)) {
            pllmf += 2U;
        }
        if((1U == pllmf4) && (0U == pllmf5)) {
            pllmf += 17U;
        }
        if((0U == pllmf4) && (1U == pllmf5)) {
            pllmf += 33U;
        }
        if((1U == pllmf4) && (1U == pllmf5)) {
            pllmf += 49U;
        }

        /* PLL clock source selection, HXTAL or IRC48M or IRC8M/2 */
        pllsel = GET_BITS(RCU_CFG0, 16, 16);
        pllpresel = GET_BITS(RCU_CFG1, 30, 30);
        if(0U != pllsel) {
            prediv = (GET_BITS(RCU_CFG1, 0, 3) + 1U);
            if(0U == pllpresel) {
                cksys_freq = (HXTAL_VALUE / prediv) * pllmf;
            } else {
                cksys_freq = (IRC48M_VALUE / prediv) * pllmf;
            }
        } else {
            cksys_freq = (IRC8M_VALUE >> 1) * pllmf;
        }
        break;
    /* IRC8M is selected as CK_SYS */
    default:
        cksys_freq = IRC8M_VALUE;
        break;
    }
    /* calculate AHB clock frequency */
    idx = GET_BITS(RCU_CFG0, 4, 7);
    clk_exp = ahb_exp[idx];
    ahb_freq = cksys_freq >> clk_exp;

    /* calculate APB1 clock frequency */
    idx = GET_BITS(RCU_CFG0, 8, 10);
    clk_exp = apb1_exp[idx];
    apb1_freq = ahb_freq >> clk_exp;

    /* calculate APB2 clock frequency */
    idx = GET_BITS(RCU_CFG0, 11, 13);
    clk_exp = apb2_exp[idx];
    apb2_freq = ahb_freq >> clk_exp;

    /* return the clocks frequency */
    switch(clock) {
    case CK_SYS:
        ck_freq = cksys_freq;
        break;
    case CK_AHB:
        ck_freq = ahb_freq;
        break;
    case CK_APB1:
        ck_freq = apb1_freq;
        break;
    case CK_APB2:
        ck_freq = apb2_freq;
        break;
    case CK_ADC:
        /* calculate ADC clock frequency */
        if(RCU_ADCSRC_AHB_APB2DIV != (RCU_CFG2 & RCU_CFG2_ADCSEL)) {
            if(RCU_ADC_IRC28M_DIV1 != (RCU_CFG2 & RCU_CFG2_IRC28MDIV)) {
                adc_freq = IRC28M_VALUE >> 1;
            } else {
                adc_freq = IRC28M_VALUE;
            }
        } else {
            /* ADC clock select CK_APB2 divided by 2/4/6/8 or CK_AHB divided by 3/5/7/9 */
            adcps = GET_BITS(RCU_CFG0, 14, 15);
            adcps2 = GET_BITS(RCU_CFG2, 31, 31);
            switch(adcps) {
            case 0:
                if(0U == adcps2) {
                    adc_freq = apb2_freq / 2U;
                } else {
                    adc_freq = ahb_freq / 3U;
                }
                break;
            case 1:
                if(0U == adcps2) {
                    adc_freq = apb2_freq / 4U;
                } else {
                    adc_freq = ahb_freq / 5U;
                }
                break;
            case 2:
                if(0U == adcps2) {
                    adc_freq = apb2_freq / 6U;
                } else {
                    adc_freq = ahb_freq / 7U;
                }
                break;
            case 3:
                if(0U == adcps2) {
                    adc_freq = apb2_freq / 8U;
                } else {
                    adc_freq = ahb_freq / 9U;
                }
                break;
            default:
                break;
            }
        }
        ck_freq = adc_freq;
        break;
    case CK_CEC:
        /* calculate CEC clock frequency */
        if(RCU_CECSRC_LXTAL != (RCU_CFG2 & RCU_CFG2_CECSEL)) {
            cec_freq = IRC8M_VALUE / 244U;
        } else {
            cec_freq = LXTAL_VALUE;
        }
        ck_freq = cec_freq;
        break;
    case CK_USART:
        /* calculate USART clock frequency */
        if(RCU_USART0SRC_CKAPB2 == (RCU_CFG2 & RCU_CFG2_USART0SEL)) {
            usart_freq = apb2_freq;
        } else if(RCU_USART0SRC_CKSYS == (RCU_CFG2 & RCU_CFG2_USART0SEL)) {
            usart_freq = cksys_freq;
        } else if(RCU_USART0SRC_LXTAL == (RCU_CFG2 & RCU_CFG2_USART0SEL)) {
            usart_freq = LXTAL_VALUE;
        } else if(RCU_USART0SRC_IRC8M == (RCU_CFG2 & RCU_CFG2_USART0SEL)) {
            usart_freq = IRC8M_VALUE;
        } else {
        }
        ck_freq = usart_freq;
        break;
    default:
        break;
    }
    return ck_freq;
}

/*!
    \brief      get the system clock source
    \param[in]  none
    \param[out] none
    \retval     which clock is selected as CK_SYS source
                only one parameter can be selected which is shown as below:
      \arg        RCU_SCSS_IRC8M: select CK_IRC8M as the CK_SYS source
      \arg        RCU_SCSS_HXTAL: select CK_HXTAL as the CK_SYS source
      \arg        RCU_SCSS_PLL: select CK_PLL as the CK_SYS source
*/
uint32_t hals_rcu_system_clock_source_get(void)
{
    return (RCU_CFG0 & RCU_CFG0_SCSS);
}

/*!
    \brief      turn on the oscillator
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
      \arg        RCU_IRC8M: IRC8M
      \arg        RCU_IRC28M: IRC28M
      \arg        RCU_IRC48M: IRC48M
      \arg        RCU_IRC40K: IRC40K
      \arg        RCU_PLL_CK: PLL
    \param[out] none
    \retval     none
*/
void hals_rcu_osci_on(hal_rcu_osci_type_enum osci)
{
    RCU_REG_VAL(osci) |= BIT(RCU_BIT_POS(osci));
}

/*!
    \brief      turn off the oscillator
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
      \arg        RCU_IRC8M: IRC8M
      \arg        RCU_IRC28M: IRC28M
      \arg        RCU_IRC48M: IRC48M
      \arg        RCU_IRC40K: IRC40K
      \arg        RCU_PLL_CK: PLL
    \param[out] none
    \retval     none
*/
void hals_rcu_osci_off(hal_rcu_osci_type_enum osci)
{
    RCU_REG_VAL(osci) &= ~BIT(RCU_BIT_POS(osci));
}

/*!
    \brief      enable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
    \param[out] none
    \retval     none
*/
void hals_rcu_osci_bypass_mode_enable(hal_rcu_osci_type_enum osci)
{
    uint32_t reg;
    switch(osci) {
    case RCU_HXTAL:
        /* HXTALEN must be reset before enable the oscillator bypass mode */
        reg = RCU_CTL0;
        RCU_CTL0 &= ~RCU_CTL0_HXTALEN;
        RCU_CTL0 = (reg | RCU_CTL0_HXTALBPS);
        break;
    case RCU_LXTAL:
        /* LXTALEN must be reset before enable the oscillator bypass mode */
        reg = RCU_BDCTL;
        RCU_BDCTL &= ~RCU_BDCTL_LXTALEN;
        RCU_BDCTL = (reg | RCU_BDCTL_LXTALBPS);
        break;
    case RCU_IRC8M:
    case RCU_IRC28M:
    case RCU_IRC48M:
    case RCU_IRC40K:
    case RCU_PLL_CK:
        break;
    default:
        break;
    }
}

/*!
    \brief      disable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
    \param[out] none
    \retval     none
*/
void hals_rcu_osci_bypass_mode_disable(hal_rcu_osci_type_enum osci)
{
    uint32_t reg;
    switch(osci) {
    case RCU_HXTAL:
        /* HXTALEN must be reset before disable the oscillator bypass mode */
        reg = RCU_CTL0;
        RCU_CTL0 &= ~RCU_CTL0_HXTALEN;
        RCU_CTL0 = (reg & (~RCU_CTL0_HXTALBPS));
        break;
    case RCU_LXTAL:
        /* LXTALEN must be reset before disable the oscillator bypass mode */
        reg = RCU_BDCTL;
        RCU_BDCTL &= ~RCU_BDCTL_LXTALEN;
        RCU_BDCTL = (reg & (~RCU_BDCTL_LXTALBPS));
        break;
    case RCU_IRC8M:
    case RCU_IRC28M:
    case RCU_IRC48M:
    case RCU_IRC40K:
    case RCU_PLL_CK:
        break;
    default:
        break;
    }
}

/*!
    \brief      set the IRC8M adjust value
    \param[in]  irc8m_adjval: IRC8M adjust value, must be between 0 and 0x1F
    \param[out] none
    \retval     none
*/
void hals_rcu_irc8m_adjust_value_set(uint8_t irc8m_adjval)
{
    uint32_t adjust = 0U;
    adjust = RCU_CTL0;
    /* reset the IRC8MADJ bits and set according to irc8m_adjval */
    adjust &= ~RCU_CTL0_IRC8MADJ;
    RCU_CTL0 = (adjust | (((uint32_t)irc8m_adjval) << 3));
}

/*!
    \brief      set the IRC28M adjust value
    \param[in]  irc28m_adjval: IRC28M adjust value, must be between 0 and 0x1F
    \param[out] none
    \retval     none
*/
void hals_rcu_irc28m_adjust_value_set(uint8_t irc28m_adjval)
{
    uint32_t adjust = 0U;
    adjust = RCU_CTL1;
    /* reset the IRC28MADJ bits and set according to irc28m_adjval */
    adjust &= ~RCU_CTL1_IRC28MADJ;
    RCU_CTL1 = (adjust | (((uint32_t)irc28m_adjval) << 3));
}

/*!
    \brief      configure the PLL clock source preselection
    \param[in]  pll_presel: PLL clock source preselection
                only one parameter can be selected which is shown as below:
      \arg        RCU_PLLPRESEL_IRC48M: select IRC48M as PLL preselection clock
      \arg        RCU_PLLPRESEL_HXTAL: select HXTAL as PLL preselection clock
    \param[out] none
    \retval     none
*/
void hals_rcu_pll_preselection_config(uint32_t pll_presel)
{
    RCU_CFG1 &= ~(RCU_CFG1_PLLPRESEL);
    RCU_CFG1 |= pll_presel;
}

/*!
    \brief      configure the HXTAL divider used as input of PLL
    \param[in]  hxtal_prediv: HXTAL divider used as input of PLL
                only one parameter can be selected which is shown as below:
      \arg        RCU_PLL_PREDVx(x=1..16): HXTAL or IRC48M divided x used as input of PLL
    \param[out] none
    \retval     none
*/
void hals_rcu_hxtal_prediv_config(uint32_t hxtal_prediv)
{
    uint32_t prediv = 0U;
    prediv = RCU_CFG1;
    /* reset the HXTALPREDV bits and set according to hxtal_prediv */
    prediv &= ~RCU_CFG1_PREDV;
    RCU_CFG1 = (prediv | hxtal_prediv);
}

/*!
    \brief      configure the PLL clock source selection and PLL multiply factor
    \param[in]  pll_src: PLL clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_PLLSRC_IRC8M_DIV2: select CK_IRC8M/2 as PLL source clock
      \arg        RCU_PLLSRC_HXTAL_IRC48M: select HXTAL or IRC48M as PLL source clock
    \param[in]  pll_mul: PLL multiply factor
                only one parameter can be selected which is shown as below:
      \arg        RCU_PLL_MULx(x=2..64): PLL source clock * x
    \param[out] none
    \retval     none
*/
void hals_rcu_pll_config(uint32_t pll_src, uint32_t pll_mul)
{
    RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PLLMF | RCU_CFG0_PLLMF4);
    RCU_CFG1 &= ~(RCU_CFG1_PLLMF5);
    RCU_CFG0 |= (pll_src | (pll_mul & (~RCU_CFG1_PLLMF5)));
    RCU_CFG1 |= (pll_mul & RCU_CFG1_PLLMF5);
}

/*!
    \brief      configure the system clock source
    \param[in]  ck_sys: system clock source select
                only one parameter can be selected which is shown as below:
      \arg        RCU_CKSYSSRC_IRC8M: select CK_IRC8M as the CK_SYS source
      \arg        RCU_CKSYSSRC_HXTAL: select CK_HXTAL as the CK_SYS source
      \arg        RCU_CKSYSSRC_PLL: select CK_PLL as the CK_SYS source
    \param[out] none
    \retval     none
*/
void hals_rcu_system_clock_source_config(uint32_t ck_sys)
{
    uint32_t cksys_source = 0U;
    cksys_source = RCU_CFG0;
    /* reset the SCS bits and set according to ck_sys */
    cksys_source &= ~RCU_CFG0_SCS;
    RCU_CFG0 = (ck_sys | cksys_source);
}

/*!
    \brief      configure the AHB clock prescaler selection
    \param[in]  ck_ahb: AHB clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_AHB_CKSYS_DIVx, x=1, 2, 4, 8, 16, 64, 128, 256, 512
    \param[out] none
    \retval     none
*/
void hals_rcu_ahb_clock_config(uint32_t ck_ahb)
{
    uint32_t ahbpsc = 0U;
    ahbpsc = RCU_CFG0;
    /* reset the AHBPSC bits and set according to ck_ahb */
    ahbpsc &= ~RCU_CFG0_AHBPSC;
    RCU_CFG0 = (ck_ahb | ahbpsc);
}

/*!
    \brief      configure the APB1 clock prescaler selection
    \param[in]  ck_apb1: APB1 clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_APB1_CKAHB_DIV1: select CK_AHB as CK_APB1
      \arg        RCU_APB1_CKAHB_DIV2: select CK_AHB/2 as CK_APB1
      \arg        RCU_APB1_CKAHB_DIV4: select CK_AHB/4 as CK_APB1
      \arg        RCU_APB1_CKAHB_DIV8: select CK_AHB/8 as CK_APB1
      \arg        RCU_APB1_CKAHB_DIV16: select CK_AHB/16 as CK_APB1
    \param[out] none
    \retval     none
*/
void hals_rcu_apb1_clock_config(uint32_t ck_apb1)
{
    uint32_t apb1psc = 0U;
    apb1psc = RCU_CFG0;
    /* reset the APB1PSC and set according to ck_apb1 */
    apb1psc &= ~RCU_CFG0_APB1PSC;
    RCU_CFG0 = (ck_apb1 | apb1psc);
}

/*!
    \brief      configure the APB2 clock prescaler selection
    \param[in]  ck_apb2: APB2 clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_APB2_CKAHB_DIV1: select CK_AHB as CK_APB2
      \arg        RCU_APB2_CKAHB_DIV2: select CK_AHB/2 as CK_APB2
      \arg        RCU_APB2_CKAHB_DIV4: select CK_AHB/4 as CK_APB2
      \arg        RCU_APB2_CKAHB_DIV8: select CK_AHB/8 as CK_APB2
      \arg        RCU_APB2_CKAHB_DIV16: select CK_AHB/16 as CK_APB2
    \param[out] none
    \retval     none
*/
void hals_rcu_apb2_clock_config(uint32_t ck_apb2)
{
    uint32_t apb2psc = 0U;
    apb2psc = RCU_CFG0;
    /* reset the APB2PSC and set according to ck_apb2 */
    apb2psc &= ~RCU_CFG0_APB2PSC;
    RCU_CFG0 = (ck_apb2 | apb2psc);
}

/*!
    \brief      configure the CK48M clock source selection
    \param[in]  ck48m_clock_source: CK48M clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_CK48MSRC_PLL48M: CK_PLL48M selected as CK48M source clock
      \arg        RCU_CK48MSRC_IRC48M: CK_IRC48M selected as CK48M source clock
    \param[out] none
    \retval     none
*/
void hals_rcu_ck48m_clock_config(uint32_t ck48m_clock_source)
{
    uint32_t reg;

    reg = RCU_ADDCTL;
    /* reset the CK48MSEL bit and set according to ck48m_clock_source */
    reg &= ~RCU_ADDCTL_CK48MSEL;
    RCU_ADDCTL = (reg | ck48m_clock_source);
}

/*!
    \brief      configure the CK_OUT clock source and divider
    \param[in]  ckout_src: CK_OUT clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_CKOUTSRC_NONE: no clock selected
      \arg        RCU_CKOUTSRC_IRC28M: IRC28M selected
      \arg        RCU_CKOUTSRC_IRC40K: IRC40K selected
      \arg        RCU_CKOUTSRC_LXTAL: LXTAL selected
      \arg        RCU_CKOUTSRC_CKSYS: CKSYS selected
      \arg        RCU_CKOUTSRC_IRC8M: IRC8M selected
      \arg        RCU_CKOUTSRC_HXTAL: HXTAL selected
      \arg        RCU_CKOUTSRC_CKPLL_DIV1: CK_PLL selected
      \arg        RCU_CKOUTSRC_CKPLL_DIV2: CK_PLL/2 selected
    \param[in]  ckout_div: CK_OUT divider
      \arg        RCU_CKOUT_DIVx(x=1,2,4,8,16,32,64,128): CK_OUT is divided by x
    \param[out] none
    \retval     none
*/
void hals_rcu_ckout_config(uint32_t ckout_src, uint32_t ckout_div)
{
    uint32_t ckout = 0U;
    ckout = RCU_CFG0;
    /* reset the CKOUTSEL, CKOUTDIV and PLLDV bits and set according to ckout_src and ckout_div */
    ckout &= ~(RCU_CFG0_CKOUTSEL | RCU_CFG0_CKOUTDIV | RCU_CFG0_PLLDV);
    RCU_CFG0 = (ckout | ckout_src | ckout_div);
}

/*!
    \brief      get the clock stabilization interrupt and ckm flags
    \param[in]  int_flag: interrupt and ckm flags, refer to rcu_int_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_FLAG_IRC40KSTB: IRC40K stabilization interrupt flag
      \arg        RCU_INT_FLAG_LXTALSTB: LXTAL stabilization interrupt flag
      \arg        RCU_INT_FLAG_IRC8MSTB: IRC8M stabilization interrupt flag
      \arg        RCU_INT_FLAG_HXTALSTB: HXTAL stabilization interrupt flag
      \arg        RCU_INT_FLAG_PLLSTB: PLL stabilization interrupt flag
      \arg        RCU_INT_FLAG_IRC28MSTB: IRC28M stabilization interrupt flag
      \arg        RCU_INT_FLAG_IRC48MSTB: IRC48M stabilization interrupt flag
      \arg        RCU_INT_FLAG_CKM: HXTAL clock stuck interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hals_rcu_interrupt_flag_get(hal_rcu_int_flag_enum int_flag)
{
    if(RESET != (RCU_REG_VAL(int_flag) & BIT(RCU_BIT_POS(int_flag)))) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear the interrupt flags
    \param[in]  int_flag_clear: clock stabilization and stuck interrupt flags clear, refer to rcu_int_flag_clear_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_FLAG_IRC40KSTB_CLR: IRC40K stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_LXTALSTB_CLR: LXTAL stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_IRC8MSTB_CLR: IRC8M stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_HXTALSTB_CLR: HXTAL stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_PLLSTB_CLR: PLL stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_IRC28MSTB_CLR: IRC28M stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_IRC48MSTB_CLR: IRC48M stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_CKM_CLR: clock stuck interrupt flag clear
    \param[out] none
    \retval     none
*/
void hals_rcu_interrupt_flag_clear(hal_rcu_int_flag_clear_enum int_flag_clear)
{
    RCU_REG_VAL(int_flag_clear) |= BIT(RCU_BIT_POS(int_flag_clear));
}

/*!
    \brief      enable the stabilization interrupt
    \param[in]  stab_int: clock stabilization interrupt, refer to rcu_int_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_IRC40KSTB: IRC40K stabilization interrupt enable
      \arg        RCU_INT_LXTALSTB: LXTAL stabilization interrupt enable
      \arg        RCU_INT_IRC8MSTB: IRC8M stabilization interrupt enable
      \arg        RCU_INT_HXTALSTB: HXTAL stabilization interrupt enable
      \arg        RCU_INT_PLLSTB: PLL stabilization interrupt enable
      \arg        RCU_INT_IRC28MSTB: IRC28M stabilization interrupt enable
      \arg        RCU_INT_IRC48MSTB: IRC48M stabilization interrupt enable
    \param[out] none
    \retval     none
*/
void hals_rcu_interrupt_enable(hal_rcu_int_enum stab_int)
{
    RCU_REG_VAL(stab_int) |= BIT(RCU_BIT_POS(stab_int));
}

/*!
    \brief      disable the stabilization interrupt
    \param[in]  stab_int: clock stabilization interrupt, refer to rcu_int_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_IRC40KSTB: IRC40K stabilization interrupt disable
      \arg        RCU_INT_LXTALSTB: LXTAL stabilization interrupt disable
      \arg        RCU_INT_IRC8MSTB: IRC8M stabilization interrupt disable
      \arg        RCU_INT_HXTALSTB: HXTAL stabilization interrupt disable
      \arg        RCU_INT_PLLSTB: PLL stabilization interrupt disable
      \arg        RCU_INT_IRC28MSTB: IRC28M stabilization interrupt disable
      \arg        RCU_INT_IRC48MSTB: IRC48M stabilization interrupt disable
    \param[out] none
    \retval     none
*/
void hals_rcu_interrupt_disable(hal_rcu_int_enum stab_int)
{
    RCU_REG_VAL(stab_int) &= ~BIT(RCU_BIT_POS(stab_int));
}
