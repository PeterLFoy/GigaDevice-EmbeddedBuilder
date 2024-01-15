/*!
    \file    gd32f3x0_libopt.h
    \brief   HAL library optional for gd32f3x0
    
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

#ifndef GD32F3X0_LIBOPT_H
#define GD32F3X0_LIBOPT_H

/* if set, flash operation (write and eraser) will reserve original data located
   in out of targeted scope */
#define FLASH_OPER_RESERVE_ORIGINAL_DATA        0
/* if set, the parameters check will be implemented in function */
#define HAL_PARAMETER_CHECK                     0
/* if set, print debug message according to level of marco 'HAL_DEBUG_PRINTF_LEVEL'
   and halt code according to level of marco 'HAL_DEBUG_HALT_LEVEL' */
#define HAL_DEBUG                               0

#if (1 == HAL_DEBUG)
#define HAL_DEBUG_PRINTF                        printf
#define HAL_DEBUG_PRINTF_LEVEL                  HAL_DEBUG_LVL_ALL
#define HAL_DEBUG_HALT_LEVEL                    HAL_DEBUG_LVL_NONE

#define HAL_DEBUG_UART                          USART0
#define HAL_DEBUG_EXTRA_DO
#endif /* 1 == HAL_DEBUG */

/* define value of high speed crystal oscillator (HXTAL) in Hz */
#define HAL_HXTAL_VALUE                         ((uint32_t)8000000)
/*!< HXTAL state change timeout (in ms) */
#define HAL_HXTAL_TIMEOUT                       ((uint32_t)100)
/* define value of low speed crystal oscillator (LXTAL)in Hz */
#define HAL_LXTAL_VALUE                         ((uint32_t)32768)
/*!< LXTAL state change timeout (in ms) */
#define HAL_LXTAL_TIMEOUT                       ((uint32_t)5000)

#include "gd32f3x0_hal_dma.h"
#include "gd32f3x0_hal_fmc.h"
#include "gd32f3x0_hal_pmu.h"
#include "gd32f3x0_hal_dac.h"
#include "gd32f3x0_hal_gpio.h"
#include "gd32f3x0_hal_rcu.h"
#include "gd32f3x0_hal_exti.h"
#include "gd32f3x0_hal_sys.h"
#include "gd32f3x0_hal_syscfg.h"
#include "gd32f3x0_hal_nvic.h"
#include "gd32f3x0_hal_cmp.h"
#include "gd32f3x0_hal_cec.h"
#include "gd32f3x0_hal_crc.h"
#include "gd32f3x0_hal_adc.h"
#include "gd32f3x0_hal_ctc.h"
#include "gd32f3x0_hal_fwdgt.h"
#include "gd32f3x0_hal_tsi.h"
#include "gd32f3x0_hal_wwdgt.h"
#include "gd32f3x0_hal_spi_com.h"
#include "gd32f3x0_hal_spi.h"
#include "gd32f3x0_hal_i2s.h"
#include "gd32f3x0_hal_usart_com.h"
#include "gd32f3x0_hal_uart.h"
#include "gd32f3x0_hal_usrt.h"
#include "gd32f3x0_hal_irda.h"
#include "gd32f3x0_hal_smartcard.h" 
#include "gd32f3x0_hal_rtc.h" 
#include "gd32f3x0_hal_i2c_com.h"
#include "gd32f3x0_hal_i2c.h"
#include "gd32f3x0_hal_smbus.h"
#include "gd32f3x0_hal_timer.h"
#endif /* GD32F3X0_LIBOPT_H */
