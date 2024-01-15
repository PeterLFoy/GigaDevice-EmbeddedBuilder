/*!
    \file    gd32f3x0_hal.h
    \brief   general definitions for GD32F3x0
    
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

#ifndef GD32F3X0_HAL_H
#define GD32F3X0_HAL_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdarg.h>
#include <stdio.h>
   

typedef enum
{
  HAL_MUTEX_UNLOCKED = 0x00U,
  HAL_MUTEX_LOCKED   = 0x01U
} hal_mutex_enum;

#define HAL_BASETICK_RATE_HZ         1000U

#if HAL_BASETICK_RATE_HZ > 10000
#error "basetick rate should not be set more than 10000"
#endif

typedef void (*hal_irq_handle_cb)(void *ptr);

#define HAL_TIMEOUT_FOREVER          0xFFFFFFFFU

/* definitions for error constants */
#define HAL_ERR_NONE                 0    /* No error, everything OK. */
#define HAL_ERR_ALREADY_DONE        -1    /* already done */
#define HAL_ERR_LOCK                -2    /* already locked */
#define HAL_ERR_BUSY                -3    /* busy */
#define HAL_ERR_NO_SUPPORT          -4    /* no support */
#define HAL_ERR_TIMEOUT             -5    /* Timeout.                 */
#define HAL_ERR_VAL                 -6    /* Illegal value.           */
#define HAL_ERR_ADDRESS             -7    /* Out of address range error.     */
#define HAL_ERR_HARDWARE            -8    /* peripheral error flag set  */

#define HAL_LOCK(PERIPHERAL)                                                      \
                                do{                                               \
                                    if((PERIPHERAL)->mutex == HAL_MUTEX_LOCKED)   \
                                    {                                             \
                                       return HAL_ERR_LOCK;                       \
                                    }                                             \
                                    else                                          \
                                    {                                             \
                                       (PERIPHERAL)->mutex = HAL_MUTEX_LOCKED;    \
                                    }                                             \
                                  }while (0U)

#define HAL_UNLOCK(PERIPHERAL)                                                    \
                                  do{                                             \
                                      (PERIPHERAL)->mutex = HAL_MUTEX_UNLOCKED;   \
                                    }while (0U)

#include "gd32f3x0.h"

#if (1 == HAL_DEBUG)
#define HAL_DEBUG_LVL_NONE          0U
#define HAL_DEBUG_LVL_ALL           0XFFU
#define HAL_DEBUG_LVL_FATAL         BIT(0)
#define HAL_DEBUG_LVL_ERROR         BIT(1)
#define HAL_DEBUG_LVL_WARN          BIT(2)
#define HAL_DEBUG_LVL_INFO          BIT(3)

#ifndef HAL_DEBUG_PRINTF_LEVEL
#define HAL_DEBUG_PRINTF_LEVEL      HAL_DEBUG_LVL_ALL
#endif /* HAL_DEBUG_PRINTF_LEVEL */

#ifndef HAL_DEBUG_HALT_LEVEL
#define HAL_DEBUG_HALT_LEVEL        HAL_DEBUG_LVL_NONE
#endif /* HAL_DEBUG_HALT_LEVEL */

#define _LOG_IMPL(print_sw, halt_sw, level, message, ...) \
    do{ \
        if(print_sw){ \
            HAL_DEBUG_PRINTF("<" level "> "message"\r\n", ##__VA_ARGS__); \
            HAL_DEBUG_EXTRA_DO; \
        } \
        if(halt_sw){ \
            while(1); \
        } \
    }while (0)
    

#define LOGF_HANDLE(message, ...) \
            _LOG_IMPL((HAL_DEBUG_PRINTF_LEVEL) & HAL_DEBUG_LVL_FATAL, \
                       (HAL_DEBUG_HALT_LEVEL) & HAL_DEBUG_LVL_FATAL, "F", message, ##__VA_ARGS__)
#define LOGE_HANDLE(message, ...) \
            _LOG_IMPL((HAL_DEBUG_PRINTF_LEVEL) & HAL_DEBUG_LVL_ERROR, \
                       (HAL_DEBUG_HALT_LEVEL) & HAL_DEBUG_LVL_ERROR, "E", message, ##__VA_ARGS__)
#define LOGW_HANDLE(message, ...) \
            _LOG_IMPL((HAL_DEBUG_PRINTF_LEVEL) & HAL_DEBUG_LVL_WARN, \
                       (HAL_DEBUG_HALT_LEVEL) & HAL_DEBUG_LVL_WARN, "W", message, ##__VA_ARGS__)
#define LOGI_HANDLE(message, ...) \
            _LOG_IMPL((HAL_DEBUG_PRINTF_LEVEL) & HAL_DEBUG_LVL_INFO, \
                       (HAL_DEBUG_HALT_LEVEL) & HAL_DEBUG_LVL_INFO, "I", message, ##__VA_ARGS__)

#define HAL_DEBUGF(message, ...)      LOGF_HANDLE(message, ##__VA_ARGS__)   //fatal error
#define HAL_DEBUGE(message, ...)      LOGE_HANDLE(message, ##__VA_ARGS__)   //error
#define HAL_DEBUGW(message, ...)      LOGW_HANDLE(message, ##__VA_ARGS__)   //warning
#define HAL_DEBUGI(message, ...)      LOGI_HANDLE(message, ##__VA_ARGS__)   //information
#else
#define HAL_DEBUGF(message, ...)      ((void)0U)
#define HAL_DEBUGE(message, ...)      ((void)0U)
#define HAL_DEBUGW(message, ...)      ((void)0U)
#define HAL_DEBUGI(message, ...)      ((void)0U)
#endif /* 1 == HAL_DEBUG */

#ifdef __cplusplus
}
#endif

#endif /* GD32F3X0_HAL_H */
