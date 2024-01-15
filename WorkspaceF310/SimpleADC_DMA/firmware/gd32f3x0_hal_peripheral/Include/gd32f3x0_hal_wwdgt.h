/*!
    \file    gd32f3x0_hal_wwdgt.h
    \brief   definitions for the WWDGT

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

#ifndef GD32F3X0_HAL_WWDGT_H
#define GD32F3X0_HAL_WWDGT_H

#include "gd32f3x0_hal.h"

/* WWDGT definitions */
#define WWDGT                       WWDGT_BASE

/* registers definitions */
#define WWDGT_CTL                   REG32((WWDGT) + 0x00000000U)                     /*!< WWDGT control register */
#define WWDGT_CFG                   REG32((WWDGT) + 0x00000004U)                     /*!< WWDGT configuration register */
#define WWDGT_STAT                  REG32((WWDGT) + 0x00000008U)                     /*!< WWDGT status register */

/* bits definitions */
/* WWDGT_CTL */
#define WWDGT_CTL_CNT               BITS(0,6)                                        /*!< WWDGT counter value */
#define WWDGT_CTL_WDGTEN            BIT(7)                                           /*!< WWDGT counter enable */

/* WWDGT_CFG */
#define WWDGT_CFG_WIN               BITS(0,6)                                        /*!< WWDGT counter window value */
#define WWDGT_CFG_PSC               BITS(7,8)                                        /*!< WWDGT prescaler divider value */
#define WWDGT_CFG_EWIE              BIT(9)                                           /*!< WWDGT early wakeup interrupt enable */

/* WWDGT_STAT */
#define WWDGT_STAT_EWIF             BIT(0)                                           /*!< WWDGT early wakeup interrupt flag */

/* WWDGT state enum */
typedef enum {
    HAL_WWDGT_STATE_NONE = 0,                                                        /*!< NONE(default value) */
    HAL_WWDGT_STATE_RESET,                                                           /*!< RESET */
    HAL_WWDGT_STATE_READY,                                                           /*!< READY */
    HAL_WWDGT_STATE_BUSY,                                                            /*!< BUSY */
    HAL_WWDGT_STATE_ERROR,                                                       /*!< ERROR */
} hal_wwdgt_state_enum;

/* WWDGT structure type enum */
typedef enum {
    HAL_WWDGT_INIT_STRUCT,                                                           /*!< WWDGT initialization structure */
    HAL_WWDGT_IRQ_STRUCT,                                                            /*!< WWDGT interrupt structure */
    HAL_WWDGT_DEV_STRUCT                                                             /*!< WWDGT device information structure */
} hal_wwdgt_struct_type_enum;

/* WWDGT device interrupt callback function pointer structure */
typedef struct {
    __IO hal_irq_handle_cb                   early_wakeup_handle;                    /*!< WWDGT handler function */
} hal_wwdgt_irq_struct;

/* WWDGT device information structrue */
typedef struct {
    hal_wwdgt_irq_struct               wwdgt_irq;                                    /*!< WWDGT device interrupt callback function pointer structure */
    hal_wwdgt_state_enum               state;                                        /*!< WWDGT state */
    hal_mutex_enum                     mutex;
} hal_wwdgt_dev_struct;

/* WWDGT settings */

/* @PARA: p_wwdgt_init */
/* @STRUCT: WWDGT basic config struct */
typedef struct{
    uint32_t wwdgt_pre_select;                                                       /*!< Window Watchdog Timer Prescaler Selection */
    uint32_t wwdgt_cnt_value;                                                        /*!< Window Watchdog Counter Window Value */
    uint32_t wwdgt_downcnt_value;                                                    /*!< Window Watchdog Timer Downcounter Value */
}hal_wwdgt_init_struct;

/* @STRUCT_MEMBER: wwdgt_cnt_value */
/* @=NULL */

/* @STRUCT_MEMBER: wwdgt_Downcnt_value */
/* @=NULL */

/* @STRUCT_MEMBER: wwdgt_pre_select */
/* @DEFINE: Window Watchdog Timer Prescaler Selection */
#define CFG_PSC(regval)             (BITS(7,8) & ((uint32_t)(regval) << 7U))
#define WWDGT_PSC_DIV1              ((uint32_t)CFG_PSC(0))                            /*!< WWDGT prescaler set to 1 */
#define WWDGT_PSC_DIV2              ((uint32_t)CFG_PSC(1))                            /*!< WWDGT prescaler set to 2 */
#define WWDGT_PSC_DIV4              ((uint32_t)CFG_PSC(2))                            /*!< WWDGT prescaler set to 4 */
#define WWDGT_PSC_DIV8              ((uint32_t)CFG_PSC(3))                            /*!< WWDGT prescaler set to 8 */

/* @FUNCTION: initialize WWDGT base  */
/* initialize the parameters of WWDGT struct with the default values */
void hal_wwdgt_struct_init(hal_wwdgt_struct_type_enum hal_struct_type,void *p_struct);
/* initialize WWDGT */
int32_t hal_wwdgt_init(hal_wwdgt_dev_struct *wwdgt_dev,hal_wwdgt_init_struct *p_wwdgt_init);
/* deinitialize WWDGT */
void hal_wwdgt_deinit(hal_wwdgt_dev_struct *wwdgt_dev);
/* WWDGT interrupt handler content function */
void hal_wwdgt_irq(hal_wwdgt_dev_struct *wwdgt_dev);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_wwdgt_irq_handle_set(hal_wwdgt_dev_struct *wwdgt_dev, hal_wwdgt_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_wwdgt_irq_handle_all_reset(hal_wwdgt_dev_struct *wwdgt_dev);
/* start WWDGT module function */
void hal_wwdgt_start(void);
/* start WWDGT module function by interrupt method, the function is non-blocking */
int32_t hal_wwdgt_start_interrupt(hal_wwdgt_dev_struct *wwdgt_dev, hal_wwdgt_irq_struct *p_irq);
/* reload the counter of WWDGT */
int32_t hal_wwdgt_reload(hal_wwdgt_init_struct *p_wwdgt);
/* check early wakeup interrupt state of WWDGT */
FlagStatus hals_wwdgt_flag_get(void);
/* clear early wakeup interrupt state of WWDGT */
void hals_wwdgt_flag_clear(void);
/* enable early wakeup interrupt of WWDGT */
void hals_wwdgt_interrupt_enable(void);
/* disable early wakeup interrupt of WWDGT */
void hals_wwdgt_interrupt_disable(void);
/* @END */

#endif /* GD32F3X0_HAL_WWDGT_H */
