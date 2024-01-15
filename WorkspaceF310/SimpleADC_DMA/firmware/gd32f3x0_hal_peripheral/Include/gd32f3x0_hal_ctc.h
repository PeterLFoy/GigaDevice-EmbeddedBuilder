/*!
    \file    gd32f3x0_hal_ctc.h
    \brief   definitions for the CTC

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

#ifndef GD32F3X0_HAL_CTC_H
#define GD32F3X0_HAL_CTC_H

#include "gd32f3x0_hal.h"

/* CTC definitions */
#define CTC                          CTC_BASE                    /*!< base address */

/* registers definitions */
#define CTC_CTL0                     REG32((CTC) + 0x00000000U)  /*!< CTC control register 0 */
#define CTC_CTL1                     REG32((CTC) + 0x00000004U)  /*!< CTC control register 1 */
#define CTC_STAT                     REG32((CTC) + 0x00000008U)  /*!< CTC status register */
#define CTC_INTC                     REG32((CTC) + 0x0000000CU)  /*!< CTC interrupt clear register */

/* bits definitions */
/* CTC_CTL0 */
#define CTC_CTL0_CKOKIE              BIT(0)                    /*!< clock trim OK(CKOKIF) interrupt enable */
#define CTC_CTL0_CKWARNIE            BIT(1)                    /*!< clock trim warning(CKWARNIF) interrupt enable */
#define CTC_CTL0_ERRIE               BIT(2)                    /*!< error(ERRIF) interrupt enable */
#define CTC_CTL0_EREFIE              BIT(3)                    /*!< EREFIF interrupt enable */
#define CTC_CTL0_CNTEN               BIT(5)                    /*!< CTC counter enable */
#define CTC_CTL0_AUTOTRIM            BIT(6)                    /*!< hardware automatically trim mode */
#define CTC_CTL0_SWREFPUL            BIT(7)                    /*!< software reference source sync pulse */
#define CTC_CTL0_TRIMVALUE           BITS(8,13)                /*!< IRC48M trim value */

/* CTC_CTL1 */
#define CTC_CTL1_RLVALUE             BITS(0,15)                /*!< CTC counter reload value */
#define CTC_CTL1_CKLIM               BITS(16,23)               /*!< clock trim base limit value */
#define CTC_CTL1_REFPSC              BITS(24,26)               /*!< reference signal source prescaler */
#define CTC_CTL1_REFSEL              BITS(28,29)               /*!< reference signal source selection */
#define CTC_CTL1_REFPOL              BIT(31)                   /*!< reference signal source polarity */

/* CTC_STAT */
#define CTC_STAT_CKOKIF              BIT(0)                    /*!< clock trim OK interrupt flag */
#define CTC_STAT_CKWARNIF            BIT(1)                    /*!< clock trim warning interrupt flag */
#define CTC_STAT_ERRIF               BIT(2)                    /*!< error interrupt flag */
#define CTC_STAT_EREFIF              BIT(3)                    /*!< expect reference interrupt flag */
#define CTC_STAT_CKERR               BIT(8)                    /*!< clock trim error bit */
#define CTC_STAT_REFMISS             BIT(9)                    /*!< reference sync pulse miss */
#define CTC_STAT_TRIMERR             BIT(10)                   /*!< trim value error bit */
#define CTC_STAT_REFDIR              BIT(15)                   /*!< CTC trim counter direction when reference sync pulse occurred */
#define CTC_STAT_REFCAP              BITS(16,31)               /*!< CTC counter capture when reference sync pulse occurred */

/* CTC_INTC */
#define CTC_INTC_CKOKIC              BIT(0)                    /*!< CKOKIF interrupt clear bit */
#define CTC_INTC_CKWARNIC            BIT(1)                    /*!< CKWARNIF interrupt clear bit */
#define CTC_INTC_ERRIC               BIT(2)                    /*!< ERRIF interrupt clear bit */
#define CTC_INTC_EREFIC              BIT(3)                    /*!< EREFIF interrupt clear bit */

/* constants definitions */
#define CTL0_TRIMVALUE(regval)                           (BITS(8,13) & ((uint32_t)(regval) << 8))
#define CTL1_CKLIM(regval)                               (BITS(16,23) & ((uint32_t)(regval) << 16))
#define GET_STAT_REFCAP(regval)                          GET_BITS((regval),16,31)
#define GET_CTL0_TRIMVALUE(regval)                       GET_BITS((regval),8,13)

/* hardware automatically trim mode definitions */
#define CTC_HARDWARE_TRIM_MODE_ENABLE                    CTC_CTL0_AUTOTRIM            /*!< hardware automatically trim mode enable*/
#define CTC_HARDWARE_TRIM_MODE_DISABLE                   ((uint32_t)0x00000000U)      /*!< hardware automatically trim mode disable*/

/* reference signal source polarity definitions */
/* @STRUCT_MEMBER: polarity */
/* @DEFINE: reference signal source */
#define CTC_REFSOURCE_POLARITY_FALLING                   CTC_CTL1_REFPOL              /*!< reference signal source polarity is falling edge*/
#define CTC_REFSOURCE_POLARITY_RISING                    ((uint32_t)0x00000000U)      /*!< reference signal source polarity is rising edge*/

/* reference signal source selection definitions */
#define CTL1_REFSEL(regval)                              (BITS(28,29) & ((uint32_t)(regval) << 28))
/* @STRUCT_MEMBER: source */
/* @DEFINE: reference signal source */
#define CTC_REFSOURCE_GPIO                               CTL1_REFSEL(0)               /*!< GPIO is selected */
#define CTC_REFSOURCE_LXTAL                              CTL1_REFSEL(1)               /*!< LXTAL is clock selected */

/* reference signal source prescaler definitions */
#define CTL1_REFPSC(regval)                              (BITS(24,26) & ((uint32_t)(regval) << 24))
/* @STRUCT_MEMBER: prescaler */
/* @DEFINE: reference signal source prescaler*/
#define CTC_REFSOURCE_PSC_OFF                            CTL1_REFPSC(0)               /*!< reference signal not divided */
#define CTC_REFSOURCE_PSC_DIV2                           CTL1_REFPSC(1)               /*!< reference signal divided by 2 */
#define CTC_REFSOURCE_PSC_DIV4                           CTL1_REFPSC(2)               /*!< reference signal divided by 4 */
#define CTC_REFSOURCE_PSC_DIV8                           CTL1_REFPSC(3)               /*!< reference signal divided by 8 */
#define CTC_REFSOURCE_PSC_DIV16                          CTL1_REFPSC(4)               /*!< reference signal divided by 16 */
#define CTC_REFSOURCE_PSC_DIV32                          CTL1_REFPSC(5)               /*!< reference signal divided by 32 */
#define CTC_REFSOURCE_PSC_DIV64                          CTL1_REFPSC(6)               /*!< reference signal divided by 64 */
#define CTC_REFSOURCE_PSC_DIV128                         CTL1_REFPSC(7)               /*!< reference signal divided by 128 */

/* CTC interrupt enable definitions */
#define CTC_INT_CKOK                                     CTC_CTL0_CKOKIE              /*!< clock trim OK interrupt enable */
#define CTC_INT_CKWARN                                   CTC_CTL0_CKWARNIE            /*!< clock trim warning interrupt enable */
#define CTC_INT_ERR                                      CTC_CTL0_ERRIE               /*!< error interrupt enable */
#define CTC_INT_EREF                                     CTC_CTL0_EREFIE              /*!< expect reference interrupt enable */

/* CTC interrupt source definitions */
#define CTC_INT_FLAG_CKOK                                CTC_STAT_CKOKIF              /*!< clock trim OK interrupt flag */
#define CTC_INT_FLAG_CKWARN                              CTC_STAT_CKWARNIF            /*!< clock trim warning interrupt flag */
#define CTC_INT_FLAG_ERR                                 CTC_STAT_ERRIF               /*!< error interrupt flag */
#define CTC_INT_FLAG_EREF                                CTC_STAT_EREFIF              /*!< expect reference interrupt flag */
#define CTC_INT_FLAG_CKERR                               CTC_STAT_CKERR               /*!< clock trim error bit */
#define CTC_INT_FLAG_REFMISS                             CTC_STAT_REFMISS             /*!< reference sync pulse miss */
#define CTC_INT_FLAG_TRIMERR                             CTC_STAT_TRIMERR             /*!< trim value error */

/* CTC flag definitions */
#define CTC_FLAG_CKOK                                    CTC_STAT_CKOKIF              /*!< clock trim OK flag */
#define CTC_FLAG_CKWARN                                  CTC_STAT_CKWARNIF            /*!< clock trim warning flag */
#define CTC_FLAG_ERR                                     CTC_STAT_ERRIF               /*!< error flag */
#define CTC_FLAG_EREF                                    CTC_STAT_EREFIF              /*!< expect reference flag */
#define CTC_FLAG_CKERR                                   CTC_STAT_CKERR               /*!< clock trim error bit */
#define CTC_FLAG_REFMISS                                 CTC_STAT_REFMISS             /*!< reference sync pulse miss */
#define CTC_FLAG_TRIMERR                                 CTC_STAT_TRIMERR             /*!< trim value error bit */

#define CRS_CAL_RELOADVALUE(__Fclock__, __Fref__)  ((int)(((__Fclock__) / (__Fref__)) - 1U) + 0.5)
#define CRS_CAL_CKLIM(__Fclock__, __Fref__)    ((int)(__Fclock__ / __Fref__ * 0.0012 / 2 + 0.5))

/* @STRUCT_MEMBER: cal_style */
/* @DEFINE: calculation mode */
#define CTC_VALUE_USER_DEF                    ((uint8_t)0x00U)     /*!< user-defined is selected */
#define CTC_VALUE_AUTO_CAL                    ((uint8_t)0x01U)     /*!< Automatic calculation is selected */

/* CTC device interrupt callback function pointer structure */
typedef struct {
    __IO hal_irq_handle_cb                ctc_ckok_handle;         /*!< CTC clock trim OK handler function */
    __IO hal_irq_handle_cb                ctc_ckwarn_handle;       /*!< CTC clock trim warning handler function */
    __IO hal_irq_handle_cb                ctc_err_handle;          /*!< CTC error interrupt handler function */
    __IO hal_irq_handle_cb                ctc_eref_handle;         /*!< CTC expect reference handler function */
} hal_ctc_irq_struct;

/* CTC settings*/
/* @PARA: ctc_init */
/* @STRUCT: CTC basic config struct */
typedef struct{
    uint8_t cal_style;                                          /*!< calculation mode */
    uint32_t source;                                            /*!< reference source */
    uint32_t polarity;                                          /*!< reference signal source polarity */
    uint32_t prescaler;                                         /*!< reference signal source prescaler */
    uint8_t limit_value;                                        /*!< clock trim base limit value */
    uint16_t reload_value;                                      /*!< CTC counter reload value */
    uint32_t frequency;                                         /*!< reference signal source frequency */
}hal_ctc_init_struct;

/* CTC structure type enum */
typedef enum {
    HAL_CTC_INIT_STRUCT = 0,                                       /*!< CTC initialization structure */
    HAL_CTC_IRQ_STRUCT,
    HAL_CTC_DEV_STRUCT
} hal_ctc_struct_type_enum;

/* CTC error type enum */
typedef enum {
    HAL_CTC_ERROR_NONE             = (uint32_t)0x00000000U,               /*!< no error */
    HAL_CTC_ERROR_SYSTEM           = (uint32_t)0x00000001U,               /*!< CTC internal error: if problem of clocking, enable/disable, wrong state */
    HAL_CTC_ERROR_CONFIG           = (uint32_t)0x00000002U,               /*!< configuration error occurs */
} hal_ctc_error_enum;

/* CTC state enum */
typedef enum {
    HAL_CTC_STATE_NONE = 0,                                         /*!< NONE(default value) */
    HAL_CTC_STATE_RESET,                                            /*!< RESET */
    HAL_CTC_STATE_BUSY,                                             /*!< BUSY */
    HAL_CTC_STATE_TIMEOUT,                                          /*!< TIMEOUT */
    HAL_CTC_STATE_ERROR,                                            /*!< ERROR */
    HAL_CTC_STATE_READY                                             /*!< READY */
} hal_ctc_state_enum;

/* CTC device information structrue */
typedef struct {
    hal_ctc_irq_struct               ctc_irq;                   /*!< CTC device interrupt callback function pointer structure */
    hal_ctc_error_enum               error_state;               /*!< CTC error state */
    hal_ctc_state_enum               state;                     /*!< CTC state */
    hal_mutex_enum                   mutex;
    void                             *priv;                     /* priv data */
} hal_ctc_dev_struct;

/* @STRUCT_MEMBER: limit_value */
/* @=NULL */

/* @STRUCT_MEMBER: reload_value */
/* @=NULL */

/* @STRUCT_MEMBER: frequency */
/* @=NULL */

/* @PARA: trim_value */
/* @=NULL */

/* reset CTC peripheral */
int32_t hal_ctc_deinit(hal_ctc_dev_struct *ctc_dev);
/* initialize CTC init structure */
/* @FUNCTION: initialize CTC init structure*/
int32_t hal_ctc_init(hal_ctc_dev_struct *ctc_dev, hal_ctc_init_struct *ctc_init);
/* configure the IRC48M trim value */
/* @FUNCTION: configure the IRC48M trim value*/
void hal_ctc_irc48m_trim_value_config(hal_ctc_dev_struct *ctc_dev, uint8_t trim_value);
/* @END */

/* initialize the CTC device structure with the default values */
void hal_ctc_struct_init(hal_ctc_struct_type_enum hal_struct_type, void *p_struct);

/* start CTC peripheral */
int32_t hal_ctc_start(hal_ctc_dev_struct *ctc_dev);
/* stop CTC peripheral */
int32_t hal_ctc_stop(hal_ctc_dev_struct *ctc_dev);

/* CTC interrupt handler content function,which is merely used in ctc_handler */
void hal_ctc_irq(hal_ctc_dev_struct *ctc_dev);
/* set user-defined interrupt callback function, 
    which will be registered and called when corresponding interrupt be triggered */
void hal_ctc_irq_handle_set(hal_ctc_dev_struct *ctc_dev, hal_ctc_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
    which will be registered and called when corresponding interrupt be triggered */
void hal_ctc_irq_handle_all_reset(hal_ctc_dev_struct *ctc_dev);
/* start CTC with interrupt */
int32_t hal_ctc_start_interrupt(hal_ctc_dev_struct *ctc_dev, hal_ctc_irq_struct *p_irq);
/* stop CTC with interrupt */
int32_t hal_ctc_stop_interrupt(hal_ctc_dev_struct *ctc_dev);

/* enable CTC trim counter */
void hals_ctc_counter_enable(void);
/* disable CTC trim counter */
void hals_ctc_counter_disable(void);
/* configure reference signal source prescaler */
void hals_ctc_refsource_prescaler_config(uint32_t prescaler);
/* select reference signal source */
void hals_ctc_refsource_signal_select(uint32_t refs);
/* configure reference signal source polarity */
void hals_ctc_refsource_polarity_config(uint32_t polarity);
/* configure clock trim base limit value */
void hals_ctc_clock_limit_value_config(uint8_t limit_value);
/* configure CTC counter reload value */
void hals_ctc_counter_reload_value_config(uint16_t reload_value);
/* configure hardware automatically trim mode */
void hals_ctc_hardware_trim_mode_config(uint32_t hardmode);

/* generate software reference source sync pulse */
void hals_ctc_software_refsource_pulse_generate(void);
/* configure the IRC48M trim value */
void hals_ctc_irc48m_trim_value_config(uint8_t trim_value);

/* read CTC counter capture value when reference sync pulse occurred */
uint16_t hals_ctc_counter_capture_value_read(void);
/* read CTC trim counter direction when reference sync pulse occurred */
FlagStatus hals_ctc_counter_direction_read(void);
/* read CTC counter reload value */
uint16_t hals_ctc_counter_reload_value_read(void);
/* read the IRC48M trim value */
uint8_t hals_ctc_irc48m_trim_value_read(void);

/* interrupt & flag functions */
/* get CTC flag */
FlagStatus hals_ctc_flag_get(uint32_t flag);
/* clear CTC flag */
void hals_ctc_flag_clear(uint32_t flag);
/* enable the CTC interrupt */
void hals_ctc_interrupt_enable(uint32_t interrupt);
/* disable the CTC interrupt */
void hals_ctc_interrupt_disable(uint32_t interrupt);
/* get CTC interrupt flag */
FlagStatus hals_ctc_interrupt_flag_get(uint32_t int_flag);
/* clear CTC interrupt flag */
void hals_ctc_interrupt_flag_clear(uint32_t int_flag);
#endif /* GD32F3X0_HAL_CTC_H */
