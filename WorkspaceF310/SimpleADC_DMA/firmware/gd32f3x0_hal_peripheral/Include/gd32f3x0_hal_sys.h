/*!
    \file    gd32f3x0_hal_sys.h
    \brief   definitions for the SYS

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
#ifndef GD32F3X0_HAL_SYS_H
#define GD32F3X0_HAL_SYS_H
#include "gd32f3x0_hal.h"

/* DBG definitions */
#define SYS_DBG                      DBG_BASE

/* registers definitions */
#define SYS_DBG_ID                   REG32(SYS_DBG + 0x00000000U)                      /*!< DBG_ID code register */
#define SYS_DBG_CTL0                 REG32(SYS_DBG + 0x00000004U)                      /*!< DBG control register 0 */
#define SYS_DBG_CTL1                 REG32(SYS_DBG + 0x00000008U)                      /*!< DBG control register 1 */

/* bits definitions */
/* SYS DBG_ID */
#define SYS_DBG_ID_ID_CODE           BITS(0,31)                                        /*!< DBG ID code values */

/* SYS DBG_CTL0 */
#define SYS_DBG_CTL0_SLP_HOLD        BIT(0)                                            /*!< keep debugger connection during sleep mode */
#define SYS_DBG_CTL0_DSLP_HOLD       BIT(1)                                            /*!< keep debugger connection during deepsleep mode */
#define SYS_DBG_CTL0_STB_HOLD        BIT(2)                                            /*!< keep debugger connection during standby mode */
#define SYS_DBG_CTL0_FWDGT_HOLD      BIT(8)                                            /*!< debug FWDGT kept when core is halted */
#define SYS_DBG_CTL0_WWDGT_HOLD      BIT(9)                                            /*!< debug WWDGT kept when core is halted */
#define SYS_DBG_CTL0_TIMER0_HOLD     BIT(10)                                           /*!< hold TIMER0 counter when core is halted */
#if (defined(GD32F350) || defined(GD32F330))
#define SYS_DBG_CTL0_TIMER1_HOLD     BIT(11)                                           /*!< hold TIMER1 counter when core is halted */
#endif /* GD32F350 and GD32F330 */
#define SYS_DBG_CTL0_TIMER2_HOLD     BIT(12)                                           /*!< hold TIMER2 counter when core is halted */
#define SYS_DBG_CTL0_I2C0_HOLD       BIT(15)                                           /*!< hold I2C0 smbus when core is halted */
#define SYS_DBG_CTL0_I2C1_HOLD       BIT(16)                                           /*!< hold I2C1 smbus when core is halted */
#ifdef GD32F350
#define SYS_DBG_CTL0_TIMER5_HOLD     BIT(19)                                           /*!< hold TIMER5 counter when core is halted */
#endif /* GD32F350 */
#define SYS_DBG_CTL0_TIMER13_HOLD    BIT(27)                                           /*!< hold TIMER13 counter when core is halted */
  
/* SYS DBG_CTL1 */                                                                     
#define SYS_DBG_CTL1_RTC_HOLD        BIT(10)                                           /*!< hold RTC calendar and wakeup counter when core is halted */
#define SYS_DBG_CTL1_TIMER14_HOLD    BIT(16)                                           /*!< hold TIMER14 counter when core is halted */
#define SYS_DBG_CTL1_TIMER15_HOLD    BIT(17)                                           /*!< hold TIMER15 counter when core is halted */
#define SYS_DBG_CTL1_TIMER16_HOLD    BIT(18)                                           /*!< hold TIMER16 counter when core is halted */
  
/* constants definitions */                                                            
#define SYS_DBG_LOW_POWER_SLEEP      SYS_DBG_CTL0_SLP_HOLD                             /*!< keep debugger connection during sleep mode */
#define SYS_DBG_LOW_POWER_DEEPSLEEP  SYS_DBG_CTL0_DSLP_HOLD                            /*!< keep debugger connection during deepsleep mode */
#define SYS_DBG_LOW_POWER_STANDBY    SYS_DBG_CTL0_STB_HOLD                             /*!< keep debugger connection during standby mode */

/* define the peripheral debug hold bit position and its register index offset */
#define SYS_DBG_REGIDX_BIT(regidx, bitpos)      (((regidx) << 6) | (bitpos))
#define SYS_DBG_REG_VAL(periph)                 (REG32(SYS_DBG + ((uint32_t)(periph) >> 6)))
#define SYS_DBG_BIT_POS(val)                    ((uint32_t)(val) & 0x0000001FU)

#ifdef GD32F310
/* GD32F310 */
#define BASETICK_SOURCE_TIMERX_RST  \
{   RCU_TIMER0RST,  RCU_TIMER2RST,  \
    RCU_TIMER13RST, RCU_TIMER14RST, \
    RCU_TIMER15RST, RCU_TIMER16RST }

#define BASETICK_SOURCE_TIMERX_CLK  \
{   RCU_TIMER0,  RCU_TIMER2,        \
    RCU_TIMER13, RCU_TIMER14,       \
    RCU_TIMER15, RCU_TIMER16 }

#define BASETICK_SOURCE_TIMERX_PERIPH   \
{   TIMER0,  TIMER2, TIMER13,           \
    TIMER14, TIMER15,TIMER16 }

#define BASETICK_SOURCE_TIMERX_IRQN           \
{   TIMER0_BRK_UP_TRG_COM_IRQn,TIMER2_IRQn,   \
    TIMER13_IRQn, TIMER14_IRQn,               \
    TIMER15_IRQn, TIMER16_IRQn}
#endif /* GD32F310 */

#ifdef GD32F330
/* GD32F330 */
#define BASETICK_SOURCE_TIMERX_RST  \
{   RCU_TIMER0RST,  RCU_TIMER1RST,  \
    RCU_TIMER2RST,                  \
    RCU_TIMER13RST, RCU_TIMER14RST, \
    RCU_TIMER15RST, RCU_TIMER16RST }

#define BASETICK_SOURCE_TIMERX_CLK  \
{   RCU_TIMER0,  RCU_TIMER1,        \
    RCU_TIMER2,                     \
    RCU_TIMER13, RCU_TIMER14,       \
    RCU_TIMER15, RCU_TIMER16 };

#define BASETICK_SOURCE_TIMERX_PERIPH   \
{   TIMER0,  TIMER1 , TIMER2,           \
    TIMER13, TIMER14, TIMER15, TIMER16 };

#define BASETICK_SOURCE_TIMERX_IRQN           \
{   TIMER0_BRK_UP_TRG_COM_IRQn, TIMER1_IRQn,  \
    TIMER2_IRQn,                              \
    TIMER13_IRQn, TIMER14_IRQn,               \
    TIMER15_IRQn, TIMER16_IRQn}
#endif /* GD32F330 */
#ifdef GD32F350
/* GD32F350 */
#define BASETICK_SOURCE_TIMERX_RST  \
{   RCU_TIMER0RST,  RCU_TIMER1RST,  \
    RCU_TIMER2RST,  RCU_TIMER5RST,  \
    RCU_TIMER13RST, RCU_TIMER14RST, \
    RCU_TIMER15RST, RCU_TIMER16RST }

#define BASETICK_SOURCE_TIMERX_CLK  \
{   RCU_TIMER0,  RCU_TIMER1,        \
    RCU_TIMER2,  RCU_TIMER5,        \
    RCU_TIMER13, RCU_TIMER14,       \
    RCU_TIMER15, RCU_TIMER16 }

#define BASETICK_SOURCE_TIMERX_PERIPH   \
{   TIMER0,  TIMER1 , TIMER2, TIMER5,   \
    TIMER13, TIMER14, TIMER15,TIMER16 }

#define BASETICK_SOURCE_TIMERX_IRQN           \
{   TIMER0_BRK_UP_TRG_COM_IRQn, TIMER1_IRQn,  \
    TIMER2_IRQn, TIMER5_DAC_IRQn,             \
    TIMER13_IRQn, TIMER14_IRQn,               \
    TIMER15_IRQn, TIMER16_IRQn}
#endif /* GD32F350 */




/* register index */
typedef enum {
    SYS_DBG_IDX_CTL0            = 0x04U,                                               /*!< DBG control register 0 offset */
    SYS_DBG_IDX_CTL1            = 0x08U,                                               /*!< DBG control register 1 offset */
} hal_sys_dbg_reg_idx;

/* peripherals hold bit */
typedef enum {
    SYS_DBG_FWDGT_HOLD          = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL0, 8U),                /*!< debug FWDGT kept when core is halted */
    SYS_DBG_WWDGT_HOLD          = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL0, 9U),                /*!< debug WWDGT kept when core is halted */
    SYS_DBG_TIMER0_HOLD         = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL0, 10U),               /*!< hold TIMER0 counter when core is halted */
#if (defined(GD32F350) || defined(GD32F330))
    SYS_DBG_TIMER1_HOLD         = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL0, 11U),               /*!< hold TIMER1 counter when core is halted */
#endif /* GD32F350 and GD32F330 */
    SYS_DBG_TIMER2_HOLD         = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL0, 12U),               /*!< hold TIMER2 counter when core is halted */
#ifdef GD32F350
    SYS_DBG_TIMER5_HOLD         = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL0, 19U),               /*!< hold TIMER5 counter when core is halted */
#endif /* GD32F350 */
    SYS_DBG_TIMER13_HOLD        = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL0, 27U),               /*!< hold TIMER13 counter when core is halted */
    SYS_DBG_TIMER14_HOLD        = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL1, 16U),               /*!< hold TIMER14 counter when core is halted */
    SYS_DBG_TIMER15_HOLD        = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL1, 17U),               /*!< hold TIMER15 counter when core is halted */
    SYS_DBG_TIMER16_HOLD        = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL1, 18U),               /*!< hold TIMER16 counter when core is halted  */
    SYS_DBG_I2C0_HOLD           = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL0, 15U),               /*!< hold I2C0 smbus when core is halted */
    SYS_DBG_I2C1_HOLD           = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL0, 16U),               /*!< hold I2C1 smbus when core is halted */
    SYS_DBG_RTC_HOLD            = SYS_DBG_REGIDX_BIT(SYS_DBG_IDX_CTL1, 10U),               /*!< hold RTC calendar and wakeup counter when core is halted */
} hal_sys_dbg_periph_enum;

/* @PARA: debug_cfg */
/* @ENUM: debug selecttion */
typedef enum{
    SYS_DEBUG_DISABLE = 0,                                                             /*!< Debug disable */
    SYS_DEBUG_SERIAL_WIRE,                                                             /*!< Serial wire debug*/
}hal_sys_debug_cfg_enum;

/* @PARA: timebase_source */
/* @ENUM: SYS timebase source  */
typedef enum{
    SYS_TIMEBASE_SOURCE_SYSTICK = 0,                                                   /*!< Select SysTick as timebase source  */
    SYS_TIMEBASE_SOURCE_TIMER0,                                                        /*!< Select TIMER0  as timebase source  */
    SYS_TIMEBASE_SOURCE_TIMER1,                                                        /*!< Select TIMER1  as timebase source  */
    SYS_TIMEBASE_SOURCE_TIMER2,                                                        /*!< Select TIMER2  as timebase source  */
    SYS_TIMEBASE_SOURCE_TIMER5,                                                        /*!< Select TIMER5  as timebase source  */
    SYS_TIMEBASE_SOURCE_TIMER13,                                                       /*!< Select TIMER13 as timebase source  */
    SYS_TIMEBASE_SOURCE_TIMER14,                                                       /*!< Select TIMER14 as timebase source  */
    SYS_TIMEBASE_SOURCE_TIMER15,                                                       /*!< Select TIMER15 as timebase source  */
    SYS_TIMEBASE_SOURCE_TIMER16                                                        /*!< Select TIMER16 as timebase source  */
}hal_sys_timebase_source_enum;

/* the callback of basetick interrupt declaration */
typedef void (*hal_sys_basetick_irq_handle_cb)(void);

/* deinitialize SYS */
int32_t hal_sys_deinit(void);
/* @FUNCTION: initialize SYS debug configuration */
int32_t hal_sys_debug_init( hal_sys_debug_cfg_enum debug_cfg);
/* @FUNCTION: initialize SYS timebase source */
int32_t hal_sys_timesource_init( hal_sys_timebase_source_enum timebase_source);
/* @END */

/* timeout and delay fucntions */
/* get the basetick count */
uint32_t hal_sys_basetick_count_get(void);
/* check whether the delay is finished */
FlagStatus hal_sys_basetick_timeout_check(uint32_t time_start, uint32_t delay);
/* set the basetick delay */
void hal_sys_basetick_delay_ms(uint32_t time_ms);

/* control fucntions */
/* suspend the basetick timer */
void hal_sys_basetick_suspend(void);
/* resume the basetick timer */
void hal_sys_basetick_resume(void);

/* interrupt functions */
/* basetick interrupt handler content function, 
which is merely used in SysTick_Handler or TIMERx_UP_IRQHandler */
void hal_sys_basetick_irq(void);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_sys_basetick_irq_handle_set(hal_sys_basetick_irq_handle_cb irq_handler);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_sys_basetick_irq_handle_all_reset(void);

/* debug related fucntions */
/* read DBG_ID code register */
uint32_t hals_sys_dbg_id_get(void);

/* enable low power behavior when the MCU is in debug mode */
void hals_sys_dbg_low_power_enable(uint32_t dbg_low_power);
/* disable low power behavior when the MCU is in debug mode */
void hals_sys_dbg_low_power_disable(uint32_t dbg_low_power);

/* enable peripheral behavior when the MCU is in debug mode */
void hals_sys_dbg_periph_enable(hal_sys_dbg_periph_enum dbg_periph);
/* disable peripheral behavior when the MCU is in debug mode */
void hals_sys_dbg_periph_disable(hal_sys_dbg_periph_enum dbg_periph);

#endif /* GD32F3X0_HAL_SYS_H */
