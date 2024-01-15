/*!
    \file    gd32f3x0_hal_syscfg.c
    \brief   SYSCFG driver

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

#define EXTI_PORT_OFFSET                             ((uint32_t)10U)                             /*!< EXTI Port offset */
#define EXTI_PIN_OFFSET                              ((uint32_t)1U)                              /*!< EXTI PIN offset */

#define _GPIO_PIN_VALUE_MASK                         ((uint32_t)0xFFFF0000U)                     /*!< GPIO A,B,C PIN value mask */
#define _GPIOD_PIN_VALUE_MASK                        ((uint32_t)0xFFFFFFFBU)                     /*!< GPIO D PIN value mask */
#define _GPIOF_PIN_VALUE_MASK                        ((uint32_t)0xFFFFFF0CU)                     /*!< GPIO F PIN value mask */

#define _SYSCFG_SRAM_PCEF_FLAG                       ((uint32_t)0x00000100U)

/* EXTI source select definition */
#define EXTISS0                                      ((uint8_t)0x00U)                            /*!< EXTI source select register 0 */
#define EXTISS1                                      ((uint8_t)0x01U)                            /*!< EXTI source select register 1 */
#define EXTISS2                                      ((uint8_t)0x02U)                            /*!< EXTI source select register 2 */
#define EXTISS3                                      ((uint8_t)0x03U)                            /*!< EXTI source select register 3 */

/* EXTI source select mask bits definition */
#define EXTI_SS_MASK                                 BITS(0,3)                                   /*!< EXTI source select mask */

/* EXTI source select jumping step definition */
#define EXTI_SS_JSTEP                                ((uint8_t)0x04U)                            /*!< EXTI source select jumping step */

/* EXTI source select moving step definition */
#define EXTI_SS_MSTEP(pin)                           (EXTI_SS_JSTEP * ((pin) % EXTI_SS_JSTEP))   /*!< EXTI source select moving step */

/* static function declaration */
static void _syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin);

/*!
    \brief      deinitialize syscfg
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_syscfg_deinit(void)
{
    hal_rcu_periph_reset_enable(RCU_CFGCMPRST);
    hal_rcu_periph_reset_disable(RCU_CFGCMPRST);
}

/*!
    \brief      enable the DMA channels remapping
    \param[in]  syscfg_dma_remap: specify the DMA channels to remap
                one or more parameters can be selected which are shown as below:
      \arg        SYSCFG_DMA_REMAP_TIMER16: remap TIMER16 channel0 and UP DMA requests to channel1(defaut channel0)
      \arg        SYSCFG_DMA_REMAP_TIMER15: remap TIMER15 channel2 and UP DMA requests to channel3(defaut channel2)
      \arg        SYSCFG_DMA_REMAP_USART0RX: remap USART0 Rx DMA request to channel4(default channel2)
      \arg        SYSCFG_DMA_REMAP_USART0TX: remap USART0 Tx DMA request to channel3(default channel1)
      \arg        SYSCFG_DMA_REMAP_ADC: remap ADC DMA requests from channel0 to channel1
    \param[out] none
    \retval     int32_t: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hals_syscfg_dma_remap_enable(uint32_t syscfg_dma_remap)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check syscfg_dma_remap parameter */
    if((SYSCFG_DMA_REMAP_TIMER16 != syscfg_dma_remap)  && \
            (SYSCFG_DMA_REMAP_TIMER15 != syscfg_dma_remap)  && \
            (SYSCFG_DMA_REMAP_USART0RX != syscfg_dma_remap) && \
            (SYSCFG_DMA_REMAP_USART0TX != syscfg_dma_remap) && \
            (SYSCFG_DMA_REMAP_ADC != syscfg_dma_remap)) {
        HAL_DEBUGE("parameter [syscfg_dma_remap] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    SYSCFG_CFG0 |= syscfg_dma_remap;

    return HAL_ERR_NONE;
}

/*!
    \brief      disable the DMA channels remapping
    \param[in]  syscfg_dma_remap: specify the DMA channels to remap
                one or more parameters can be selected which are shown as below:
      \arg        SYSCFG_DMA_REMAP_TIMER16: remap TIMER16 channel0 and UP DMA requests to channel1(defaut channel0)
      \arg        SYSCFG_DMA_REMAP_TIMER15: remap TIMER15 channel2 and UP DMA requests to channel3(defaut channel2)
      \arg        SYSCFG_DMA_REMAP_USART0RX: remap USART0 Rx DMA request to channel4(default channel2)
      \arg        SYSCFG_DMA_REMAP_USART0TX: remap USART0 Tx DMA request to channel3(default channel1)
      \arg        SYSCFG_DMA_REMAP_ADC: remap ADC DMA requests from channel0 to channel1
    \param[out] none
    \retval     int32_t: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hals_syscfg_dma_remap_disable(uint32_t syscfg_dma_remap)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check syscfg_dma_remap parameter */
    if((SYSCFG_DMA_REMAP_TIMER16 != syscfg_dma_remap)  &&  \
            (SYSCFG_DMA_REMAP_TIMER15 != syscfg_dma_remap)  &&  \
            (SYSCFG_DMA_REMAP_USART0RX != syscfg_dma_remap) &&  \
            (SYSCFG_DMA_REMAP_USART0TX != syscfg_dma_remap) &&  \
            (SYSCFG_DMA_REMAP_ADC != syscfg_dma_remap)) {
        HAL_DEBUGE("parameter [syscfg_dma_remap] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    SYSCFG_CFG0 &= ~syscfg_dma_remap;

    return HAL_ERR_NONE;
}

/*!
    \brief      enable PB9 high current capability
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_syscfg_high_current_enable(void)
{
    SYSCFG_CFG0 |= SYSCFG_HIGH_CURRENT_ENABLE;
}

/*!
    \brief      disable PB9 high current capability
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_syscfg_high_current_disable(void)
{
    SYSCFG_CFG0 &= SYSCFG_HIGH_CURRENT_DISABLE;
}

/*!
    \brief      configure the GPIO pin as EXTI Line
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
    \param[in]  pin: GPIO pin
                only one parameter can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15)
    \param[out] none
    \retval     int32_t: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hals_syscfg_exti_config(uint32_t gpio_periph, uint32_t pin)
{
    uint32_t exti_port;
    uint8_t exti_pin;
    uint32_t i;

#if (1 == HAL_PARAMETER_CHECK)
    /* check gpio_periph value */
    if((GPIOA != gpio_periph) && \
            (GPIOB != gpio_periph) && \
            (GPIOC != gpio_periph) && \
            (GPIOF != gpio_periph)) {
        HAL_DEBUGE("parameter [gpio_periph] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check if pin is PA0 ~ PA15/PB0 ~ PB15 /PC0 ~ PC15 or not */
    if((GPIOA == gpio_periph) || \
            (GPIOB == gpio_periph) || \
            (GPIOC == gpio_periph)) {
#if (1 == HAL_PARAMETER_CHECK)
        if((0U != (pin & _GPIO_PIN_VALUE_MASK))) {
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;
        }
#endif /* 1 == HAL_PARAMETER_CHECK */

        /* check if only one GPIO pin is set or not and get EXTI source pin value */
        for(exti_pin = 0U; exti_pin < 16U; exti_pin++) {
            if((EXTI_PIN_OFFSET << exti_pin) & pin) {
                for(i = (exti_pin + 1); i < 16U; i++) {
                    if(0U != ((EXTI_PIN_OFFSET << i) & pin)) {
                        HAL_DEBUGE("parameter [pin] value is invalid");
                        return HAL_ERR_VAL;
                    }
                }
                break;
            }
        }
    }

    /* check if pin is PD2 or not */
    if((GPIOD == gpio_periph)) {
#if (1 == HAL_PARAMETER_CHECK)
        if((0U != (pin & _GPIOD_PIN_VALUE_MASK))) {
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;
        }
#endif /* 1 == HAL_PARAMETER_CHECK */

        /* check if only one GPIO pin is set or not and get EXTI source pin value */
        for(exti_pin = 2U; exti_pin < 3U; exti_pin++) {
            if((EXTI_PIN_OFFSET << exti_pin) & pin) {
                for(i = (exti_pin + 1); i < 16U; i++) {
                    if(0U != ((EXTI_PIN_OFFSET << i) & pin)) {
                        HAL_DEBUGE("parameter [pin] value is invalid");
                        return HAL_ERR_VAL;
                    }
                }
                break;
            }
        }
    }

    /* check if pin is PF0 ~ PF1/PF4 ~ PF7 or not */
    if((GPIOF == gpio_periph)) {
#if (1 == HAL_PARAMETER_CHECK)
        if((0U != (pin & _GPIOF_PIN_VALUE_MASK))) {
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;
        }
#endif /* 1 == HAL_PARAMETER_CHECK */

        /* check if only one GPIO pin is set or not and get EXTI source pin value */
        for(exti_pin = 0U; exti_pin < 2U; exti_pin++) {
            if((EXTI_PIN_OFFSET << exti_pin) & pin) {
                for(i = (exti_pin + 1); i < 16U; i++) {
                    if(0U != ((EXTI_PIN_OFFSET << i) & pin)) {
                        HAL_DEBUGE("parameter [pin] value is invalid");
                        return HAL_ERR_VAL;
                    }
                }
                break;
            }
        }

        for(exti_pin = 4U; exti_pin < 8U; exti_pin++) {
            if((EXTI_PIN_OFFSET << exti_pin) & pin) {
                for(i = (exti_pin + 1); i < 16U; i++) {
                    if(0U != ((EXTI_PIN_OFFSET << i) & pin)) {
                        HAL_DEBUGE("parameter [pin] value is invalid");
                        return HAL_ERR_VAL;
                    }
                }
                break;
            }
        }
    }

    /* get EXTI GPIO port */
    exti_port = gpio_periph - GPIO_BASE;
    exti_port = (exti_port >> EXTI_PORT_OFFSET);

    /* configure the GPIO pin as EXTI Line */
    _syscfg_exti_line_config((uint8_t)exti_port, exti_pin);

    return HAL_ERR_NONE;
}

/*!
    \brief      connect TIMER0/14/15/16 break input to the selected parameter
    \param[in]  syscfg_lock: Specify the parameter to be connected
                only one parameter can be selected which are shown as below:
      \arg        SYSCFG_LOCK_LOCKUP: Cortex-M23 lockup output connected to the break input
      \arg        SYSCFG_LOCK_SRAM_PARITY_ERROR: SRAM_PARITY check error connected to the break input
      \arg        SYSCFG_LOCK_LVD: LVD interrupt connected to the break input
    \param[out] none
    \retval     int32_t: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hals_syscfg_lock_config(uint32_t syscfg_lock)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check syscfg_lock parameter */
    if((SYSCFG_LOCK_LOCKUP != syscfg_lock) && \
            (SYSCFG_LOCK_SRAM_PARITY_ERROR != syscfg_lock) && \
            (SYSCFG_LOCK_LVD != syscfg_lock)) {
        HAL_DEBUGE("parameter [syscfg_lock] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    SYSCFG_CFG2 |= syscfg_lock;

    return HAL_ERR_NONE;
}

/*!
    \brief      check if the SRAM parity check error flag in SYSCFG_CFG2 is set or not
    \param[in]  none
    \param[out] none
    \retval     FlagStatus: SET or RESET
  */
FlagStatus hals_syscfg_sram_pcef_flag_get(void)
{
    if((SYSCFG_CFG2 & _SYSCFG_SRAM_PCEF_FLAG) != (uint32_t)RESET) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear the SRAM parity check error flag in SYSCFG_CFG2 by writing 1
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_syscfg_sram_pcef_flag_clear(void)
{
    SYSCFG_CFG2 |= (uint32_t)_SYSCFG_SRAM_PCEF_FLAG;
}

/*!
    \brief      configure the I/O compensation cell
    \param[in]  syscfg_compensation: specifies the I/O compensation cell mode
                only one parameter can be selected which is shown as below:
      \arg        SYSCFG_COMPENSATION_ENABLE: I/O compensation cell is enabled
      \arg        SYSCFG_COMPENSATION_DISABLE: I/O compensation cell is disabled
    \param[out] none
    \retval     none
*/
void hals_syscfg_compensation_config(uint32_t syscfg_compensation)
{
    uint32_t reg;

    reg = SYSCFG_CPSCTL;
    /* reset the SYSCFG_CPSCTL_CPS_EN bit and set according to syscfg_compensation */
    reg &= ~SYSCFG_CPSCTL_CPS_EN;
    SYSCFG_CPSCTL = (reg | syscfg_compensation);
}

/*!
    \brief      check if the I/O compensation cell ready flag is set or not
    \param[in]  none
    \param[out] none
    \retval     FlagStatus: SET or RESET
  */
FlagStatus hals_syscfg_cps_rdy_flag_get(void)
{
    if(((uint32_t)RESET) != (SYSCFG_CPSCTL & SYSCFG_CPSCTL_CPS_RDY)) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      configure the GPIO pin as EXTI Line
    \param[in]  exti_port: specify the GPIO port used in EXTI
                only one parameter can be selected which is shown as below:
      \arg        EXTI_SOURCE_GPIOx(x = A,B,C,D,F): EXTI GPIO port
    \param[in]  exti_pin: specify the EXTI line
                only one parameter can be selected which is shown as below:
      \arg        EXTI_SOURCE_PINx(x = 0..15): EXTI GPIO pin
    \param[out] none
    \retval     none
*/
static void _syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin)
{
    uint32_t clear_exti_mask = ~((uint32_t)EXTI_SS_MASK << (EXTI_SS_MSTEP(exti_pin)));
    uint32_t config_exti_mask = ((uint32_t)exti_port) << (EXTI_SS_MSTEP(exti_pin));

    switch(exti_pin / EXTI_SS_JSTEP) {
    case EXTISS0:
        /* clear EXTI source line(0..3) */
        SYSCFG_EXTISS0 &= clear_exti_mask;
        /* configure EXTI soure line(0..3) */
        SYSCFG_EXTISS0 |= config_exti_mask;
        break;
    case EXTISS1:
        /* clear EXTI soure line(4..7) */
        SYSCFG_EXTISS1 &= clear_exti_mask;
        /* configure EXTI soure line(4..7) */
        SYSCFG_EXTISS1 |= config_exti_mask;
        break;
    case EXTISS2:
        /* clear EXTI soure line(8..11) */
        SYSCFG_EXTISS2 &= clear_exti_mask;
        /* configure EXTI soure line(8..11) */
        SYSCFG_EXTISS2 |= config_exti_mask;
        break;
    case EXTISS3:
        /* clear EXTI soure line(12..15) */
        SYSCFG_EXTISS3 &= clear_exti_mask;
        /* configure EXTI soure line(12..15) */
        SYSCFG_EXTISS3 |= config_exti_mask;
        break;
    default:
        break;
    }
}
