/*!
    \file    gd32f3x0_hal_cmp.h
    \brief   definitions for the CMP

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

#ifndef GD32F3X0_HAL_CMP_H
#define GD32F3X0_HAL_CMP_H

#include "gd32f3x0_hal.h"


/* CMP definitions */
#define CMP                                      CMP_BASE                       /*!< CMP base address */

/* registers definitions */
#define CMP_CS                                   REG32((CMP) + 0x00000000U)     /*!< CMP control and status register */

/* bits definitions */
/* CMP_CS */
#define CMP_CS_CMP0EN                            BIT(0)                         /*!< CMP0 enable */
#define CMP_CS_CMP0SW                            BIT(1)                         /*!< CMP switch mode enable */
#define CMP_CS_CMP0M                             BITS(2,3)                      /*!< CMP0 mode */
#define CMP_CS_CMP0MSEL                          BITS(4,6)                      /*!< CMP_IM input selection */
#define CMP_CS_CMP0OSEL                          BITS(8,10)                     /*!< CMP0 output selection */
#define CMP_CS_CMP0PL                            BIT(11)                        /*!< CMP0 output polarity */
#define CMP_CS_CMP0HST                           BITS(12,13)                    /*!< CMP0 hysteresis */
#define CMP_CS_CMP0O                             BIT(14)                        /*!< CMP0 output state bit */
#define CMP_CS_CMP0LK                            BIT(15)                        /*!< CMP0 lock */
#define CMP_CS_CMP1EN                            BIT(16)                        /*!< CMP1 enable */
#define CMP_CS_CMP1M                             BITS(18,19)                    /*!< CMP1 mode */
#define CMP_CS_CMP1MSEL                          BITS(20,22)                    /*!< CMP_IM input selection */
#define CMP_CS_WNDEN                             BIT(23)                        /*!< CMP window mode enable */
#define CMP_CS_CMP1OSEL                          BITS(24,26)                    /*!< CMP1 output selection */
#define CMP_CS_CMP1PL                            BIT(27)                        /*!< CMP1 output polarity */
#define CMP_CS_CMP1HST                           BITS(28,29)                    /*!< CMP1 hysteresis */
#define CMP_CS_CMP1O                             BIT(30)                        /*!< CMP1 output state bit */
#define CMP_CS_CMP1LK                            BIT(31)                        /*!< CMP1 lock */

#define CS_CMPXM(regval)                         (BITS(2,3) & ((uint32_t)(regval) << 2U))
#define CS_CMPXMSEL(regval)                      (BITS(4,6) & ((uint32_t)(regval) << 4U))
#define CS_CMPXOSEL(regval)                      (BITS(8,10) & ((uint32_t)(regval) << 8U))
#define CS_CMPXPL(regval)                        (BIT(11) & ((uint32_t)(regval) << 11U))
#define CS_CMPXHST(regval)                       (BITS(12,13) & ((uint32_t)(regval) << 12U))

/* constants definitions */
/* CMP state enum */
typedef enum{
    HAL_CMP_STATE_NONE = 0,                                                     /*!< NONE(default value) */
    HAL_CMP_STATE_RESET,                                                        /*!< RESET */
    HAL_CMP_STATE_BUSY,                                                         /*!< BUSY */
    HAL_CMP_STATE_TIMEOUT,                                                      /*!< TIMEOUT */
    HAL_CMP_STATE_ERROR,                                                        /*!< ERROR */
    HAL_CMP_STATE_READY                                                         /*!< READY */
}hal_cmp_state_enum;

/* CMP structure type enum */
typedef enum {
    HAL_CMP_INIT_STRUCT,                                                        /*!< CMP initialization structure */
    HAL_CMP_DEV_STRUCT                                                          /*!< CMP device structure */
} hal_cmp_struct_type_enum;

/* CMP device interrupt callback function pointer structure */
typedef struct {
    __IO hal_irq_handle_cb  output_changed_handle;                              /*!< CMP output level changed handler function */
} hal_cmp_irq_struct;

/* CMP device structure */
typedef struct{
    uint32_t                 periph;                                            /*!< the CMP0 or CMP1 */
    hal_cmp_irq_struct       cmp_irq;                                           /*!< CMP interrupt callback */
    hal_cmp_state_enum       state;                                             /*!< CMP device state */
    hal_mutex_enum           mutex;
    uint32_t                 output_level;                                      /*!< CMP output level */
    void                     *priv;                                             /* priv data */    
}hal_cmp_dev_struct;

/* CMP peripheral parameter check */
#define IS_CMP_PERIPHERAL(CMP)                  (((CMP) == CMP0) || ((CMP) == CMP1))

/* CMP output level */
#define CMP_OUTPUTLEVEL_HIGH                    ((uint32_t)0x00000001U)        	/*!< CMP output high */
#define CMP_OUTPUTLEVEL_LOW                     ((uint32_t)0x00000000U)        	/*!< CMP output low */

/* CMP initialize */
/* @PARA: periph */
/* @DEFINE: CMP peripherals */
#define CMP0                                    ((uint32_t)0x00000000)          /*!< CMP0 */
#define CMP1                                    ((uint32_t)0x00000001)          /*!< CMP1 */

/* @STRUCT_MEMBER: mode */
/* @DEFINE: power and speed mode */
/* CMP operating mode */
#define CMP_MODE_HIGHSPEED                       CS_CMPXM(0)                    /*!< CMP mode high speed */
#define CMP_MODE_MIDDLESPEED                     CS_CMPXM(1)                    /*!< CMP mode middle speed */
#define CMP_MODE_LOWSPEED                        CS_CMPXM(2)                    /*!< CMP mode low speed */
#define CMP_MODE_VERYLOWSPEED                    CS_CMPXM(3)                    /*!< CMP mode very low speed */

/* @STRUCT_MEMBER: polarity */
/* @DEFINE: output polarity */
#define CMP_OUTPUT_POLARITY_NONINVERTED          CS_CMPXPL(0)                   /*!< CMP output not inverted */
#define CMP_OUTPUT_POLARITY_INVERTED             CS_CMPXPL(1)                   /*!< CMP output inverted */

/* @STRUCT_MEMBER: hysteresis */
/* @DEFINE: input hysteresis */
#define CMP_HYSTERESIS_NO                        CS_CMPXHST(0)                  /*!< CMP output no hysteresis */
#define CMP_HYSTERESIS_LOW                       CS_CMPXHST(1)                  /*!< CMP output low hysteresis */
#define CMP_HYSTERESIS_MIDDLE                    CS_CMPXHST(2)                  /*!< CMP output middle hysteresis */
#define CMP_HYSTERESIS_HIGH                      CS_CMPXHST(3)                  /*!< CMP output high hysteresis */

/* @STRUCT_MEMBER: outputsel */
/* @DEFINE: output selection */
#define CMP_OUTPUT_NONE                          CS_CMPXOSEL(0)                 /*!< CMP output none */
#define CMP_OUTPUT_TIMER0_BKIN                   CS_CMPXOSEL(1)                 /*!< CMP output TIMER0 break input */
#define CMP_OUTPUT_TIMER0_IC0                    CS_CMPXOSEL(2)                 /*!< CMP output TIMER0_CH0 input capture */
#define CMP_OUTPUT_TIMER0_OCPRECLR               CS_CMPXOSEL(3)                 /*!< CMP output TIMER0 OCPRE_CLR input */
#define CMP_OUTPUT_TIMER1_IC3                    CS_CMPXOSEL(4)                 /*!< CMP output TIMER1_CH3 input capture */
#define CMP_OUTPUT_TIMER1_OCPRECLR               CS_CMPXOSEL(5)                 /*!< CMP output TIMER1 OCPRE_CLR input */
#define CMP_OUTPUT_TIMER2_IC0                    CS_CMPXOSEL(6)                 /*!< CMP output TIMER2_CH0 input capture */
#define CMP_OUTPUT_TIMER2_OCPRECLR               CS_CMPXOSEL(7)                 /*!< CMP output TIMER2 OCPRE_CLR input */

/* @STRUCT_MEMBER: inverting_input */
/* @DEFINE: inverting input */
#define CMP_INVERTING_INPUT_1_4VREFINT           CS_CMPXMSEL(0)                 /*!< CMP inverting input 1/4 Vrefint */
#define CMP_INVERTING_INPUT_1_2VREFINT           CS_CMPXMSEL(1)                 /*!< CMP inverting input 1/2 Vrefint */
#define CMP_INVERTING_INPUT_3_4VREFINT           CS_CMPXMSEL(2)                 /*!< CMP inverting input 3/4 Vrefint */
#define CMP_INVERTING_INPUT_VREFINT              CS_CMPXMSEL(3)                 /*!< CMP inverting input Vrefint */
#define CMP_INVERTING_INPUT_PA4                  CS_CMPXMSEL(4)                 /*!< CMP inverting input PA4(DAC0_OUT0) */
#define CMP_INVERTING_INPUT_PA5                  CS_CMPXMSEL(5)                 /*!< CMP inverting input PA5 */
#define CMP_INVERTING_INPUT_PA0_PA2              CS_CMPXMSEL(6)                 /*!< CMP inverting input PA0 for CMP0 or PA2 for CMP1 */

/* @STRUCT_MEMBER: noninverting_input */
/* @ENUM: noninverting input */
typedef enum{
    CMP0_IP_PA1 = 0,                                                            /*!< PA1 input, only for CMP0 */
    CMP0_IP_PA4_SWCLOSE,                                                        /*!< PA4 input, switch between PA1 and PA4 closed, only for CMP0 */
    CMP1_IP_PA3,                                                                /*!< PA3 input, only for CMP1 */
    CMP1_IP_OF_CMP0                                                             /*!< window mode, select input of CMP0, only for CMP1 */
}hal_cmp_noninverting_input_enum;

/* @STRUCT_MEMBER: exti_type */
/* @ENUM: EXTI trigger mode */
typedef enum{
    CMP_EXTI_NONE = 0,                                                          /*!< no exti interrupt or envnt trigger */
    CMP_EXTI_INT_RISING = EXTI_INTERRUPT_TRIG_RISING,                           /*!< exti interrupt with rising edge */
    CMP_EXTI_INT_FALLING = EXTI_INTERRUPT_TRIG_FALLING,                         /*!< exti interrupt with falling edge */
    CMP_EXTI_INT_BOTH = EXTI_INTERRUPT_TRIG_BOTH,                               /*!< exti interrupt with both rising and falling edge */
    CMP_EXTI_EVENT_RISING = EXTI_EVENT_TRIG_RISING,                             /*!< exti event with rising edge */
    CMP_EXTI_EVENT_FALLING = EXTI_EVENT_TRIG_FALLING,                           /*!< exti event with falling edge */
    CMP_EXTI_EVENT_BOTH = EXTI_EVENT_TRIG_BOTH                                  /*!< exti event with both rising and falling edge */
}hal_cmp_exti_type_enum;

/* @PARA: p_init */
/* @STRUCT: CMP init struct */
typedef struct{
    uint32_t mode;                                                              /*!< operating mode for power and speed */
    uint32_t polarity;                                                          /*!< output polarity */
    uint32_t hysteresis;                                                        /*!< hysteresis level */
    uint32_t outputsel;                                                         /*!< internal output destination selection */
    uint32_t inverting_input;                                                   /*!< inverting input selection */
    hal_cmp_noninverting_input_enum noninverting_input;                         /*!< noninverting input selection */
    hal_cmp_exti_type_enum exti_type;                                           /*!< CMP output EXTI trigger type */
}hal_cmp_init_struct;

/* @FUNCTION: initialize CMP */
int32_t hal_cmp_init(hal_cmp_dev_struct *cmp_dev, uint32_t periph, hal_cmp_init_struct *p_init);
/* @END */

/* function declarations */
/* initialize the CMP structure with the default values */
int32_t hal_cmp_struct_init(hal_cmp_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize CMP */
int32_t hal_cmp_deinit(hal_cmp_dev_struct *cmp_dev);

/* start CMP module function */
int32_t hal_cmp_start(hal_cmp_dev_struct *cmp_dev);
/* stop  CMP module function */
int32_t hal_cmp_stop(hal_cmp_dev_struct *cmp_dev);
/* start CMP module in interrupt mode */
int32_t hal_cmp_start_interrupt(hal_cmp_dev_struct *cmp_dev, hal_cmp_irq_struct *p_irq);
/* stop CMP module in interrupt mode */
int32_t hal_cmp_stop_interrupt(hal_cmp_dev_struct *cmp_dev);

/* set user-defined interrupt callback function,
which will be registered and called when corresponding interrupt be triggered */
void hal_cmp_irq_handle_set(hal_cmp_dev_struct *cmp_dev, hal_cmp_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_cmp_irq_handle_all_reset(hal_cmp_dev_struct *cmp_dev);
/* cmp external trigger interrupt handler content function,which is merely used in ADC_CMP_IRQHandler */
void hal_cmp_irq(hal_cmp_dev_struct *cmp_dev);

/* lock the CMP */
int32_t hal_cmp_lock(hal_cmp_dev_struct *cmp_dev);
/* get output level of CMP */
uint32_t hal_cmp_output_level_get(hal_cmp_dev_struct *cmp_dev);
/* get the state of CMP */
hal_cmp_state_enum hal_cmp_state_get(hal_cmp_dev_struct *cmp_dev);

#endif /* GD32F3X0_HAL_CMP_H */
