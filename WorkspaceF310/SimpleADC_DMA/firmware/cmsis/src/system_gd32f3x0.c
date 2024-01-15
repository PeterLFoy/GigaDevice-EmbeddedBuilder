/*!
    \file  system_gd32f3x0.c
    \brief CMSIS Cortex-M4 Device Peripheral Access Layer Source File for
           GD32F3x0 Device Series
*/

/* Copyright (c) 2012 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

/* This file refers the CMSIS standard, some adjustments are made according to GigaDevice chips */

#include "gd32f3x0.h"

#define VECT_TAB_OFFSET  (uint32_t)0x00            /* vector table base offset */

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

/*!
    \brief      setup the microcontroller system, initialize the system
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SystemInit(void)
{
#if (defined(GD32F350))
    RCU_APB2EN |= BIT(0);
    CMP_CS |= (CMP_CS_CMP1MSEL | CMP_CS_CMP0MSEL);
#endif /* GD32F350 */
    if(((FMC_OBSTAT & OB_OBSTAT_PLEVEL_HIGH) != OB_OBSTAT_PLEVEL_HIGH) &&
            (((FMC_OBSTAT >> 13) & 0x1) == SET)) {
        FMC_KEY = UNLOCK_KEY0;
        FMC_KEY = UNLOCK_KEY1 ;
        FMC_OBKEY = UNLOCK_KEY0;
        FMC_OBKEY = UNLOCK_KEY1 ;
        FMC_CTL |= FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_START;
        while((uint32_t)0x00U != (FMC_STAT & FMC_STAT_BUSY));
        FMC_CTL &= ~FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_OBPG;
        if((FMC_OBSTAT & OB_OBSTAT_PLEVEL_HIGH) == OB_OBSTAT_PLEVEL_NO) {
            OB_SPC = FMC_NSPC;
        } else if((FMC_OBSTAT & OB_OBSTAT_PLEVEL_HIGH) == OB_OBSTAT_PLEVEL_LOW) {
            OB_SPC = FMC_LSPC;
        }
        OB_USER = OB_USER_DEFAULT & ((uint8_t)(FMC_OBSTAT >> 8));
        OB_DATA0 = ((uint8_t)(FMC_OBSTAT >> 16));
        OB_DATA1 = ((uint8_t)(FMC_OBSTAT >> 24));
        OB_WP0 = ((uint8_t)(FMC_WP));
        OB_WP1 = ((uint8_t)(FMC_WP >> 8));
        while((uint32_t)0x00U != (FMC_STAT & FMC_STAT_BUSY));
        FMC_CTL &= ~FMC_CTL_OBPG;
        FMC_CTL &= ~FMC_CTL_OBWEN;
        FMC_CTL |= FMC_CTL_LK;
    }
    /* FPU settings */
#if (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
#endif

#ifdef VECT_TAB_SRAM
    hals_nvic_vector_table_set(NVIC_VECTTAB_RAM, VECT_TAB_OFFSET);
#else
    hals_nvic_vector_table_set(NVIC_VECTTAB_FLASH, VECT_TAB_OFFSET);
#endif
}
