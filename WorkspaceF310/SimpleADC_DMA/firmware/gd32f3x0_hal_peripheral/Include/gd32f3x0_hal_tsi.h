/*!
    \file    gd32f3x0_hal_tsi.h
    \brief   definitions for the TSI

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

#ifndef GD32F3X0_HAL_TSI_H
#define GD32F3X0_HAL_TSI_H

#if defined(GD32F350)

#include "gd32f3x0_hal.h"

/* TSI definitions */
#define TSI                     TSI_BASE                          /*!< TSI base address */

/* registers definitions */
#define TSI_CTL0                REG32(TSI + 0x00000000U)          /*!< TSI control register 0 */
#define TSI_INTEN               REG32(TSI + 0x00000004U)          /*!< TSI interrupt enable register */
#define TSI_INTC                REG32(TSI + 0x00000008U)          /*!< TSI interrupt flag clear register */
#define TSI_INTF                REG32(TSI + 0x0000000CU)          /*!< TSI interrupt flag register */
#define TSI_PHM                 REG32(TSI + 0x00000010U)          /*!< TSI pin hysteresis mode register */
#define TSI_ASW                 REG32(TSI + 0x00000018U)          /*!< TSI analog switch register */
#define TSI_SAMPCFG             REG32(TSI + 0x00000020U)          /*!< TSI sample configuration register */
#define TSI_CHCFG               REG32(TSI + 0x00000028U)          /*!< TSI channel configuration register */
#define TSI_GCTL                REG32(TSI + 0x00000030U)          /*!< TSI group control register */
#define TSI_G0CYCN              REG32(TSI + 0x00000034U)          /*!< TSI group 0 cycle number register */
#define TSI_G1CYCN              REG32(TSI + 0x00000038U)          /*!< TSI group 1 cycle number register */
#define TSI_G2CYCN              REG32(TSI + 0x0000003CU)          /*!< TSI group 2 cycle number register */
#define TSI_G3CYCN              REG32(TSI + 0x00000040U)          /*!< TSI group 3 cycle number register */
#define TSI_G4CYCN              REG32(TSI + 0x00000044U)          /*!< TSI group 4 cycle number register */
#define TSI_G5CYCN              REG32(TSI + 0x00000048U)          /*!< TSI group 5 cycle number register */
#define TSI_CTL1                REG32(TSI + 0x00000300U)          /*!< TSI control registers1 */

/* bits definitions */
/* TSI_CTL0 */
#define TSI_CTL0_TSIEN          BIT(0)                            /*!< TSI enable */
#define TSI_CTL0_TSIS           BIT(1)                            /*!< TSI start */
#define TSI_CTL0_TRGMOD         BIT(2)                            /*!< trigger mode selection */
#define TSI_CTL0_EGSEL          BIT(3)                            /*!< edge selection */
#define TSI_CTL0_PINMOD         BIT(4)                            /*!< pin mode */
#define TSI_CTL0_MCN            BITS(5,7)                         /*!< max cycle number of a sequence */
#define TSI_CTL0_CTCDIV         BITS(12,14)                       /*!< CTCLK clock division factor */
#define TSI_CTL0_ECDIV          BIT(15)                           /*!< ECCLK clock division factor */
#define TSI_CTL0_ECEN           BIT(16)                           /*!< extend charge state enable */
#define TSI_CTL0_ECDT           BITS(17,23)                       /*!< extend charge state maximum duration time */
#define TSI_CTL0_CTDT           BITS(24,27)                       /*!< charge transfer state duration time */
#define TSI_CTL0_CDT            BITS(28,31)                       /*!< charge state duration time */

/* TSI_INTEN */
#define TSI_INTEN_CTCFIE        BIT(0)                            /*!< charge transfer complete flag interrupt enable */
#define TSI_INTEN_MNERRIE       BIT(1)                            /*!< max cycle number error interrupt enable */

/* TSI_INTC */
#define TSI_INTC_CCTCF          BIT(0)                            /*!< clear charge transfer complete flag */
#define TSI_INTC_CMNERR         BIT(1)                            /*!< clear max cycle number error */

/* TSI_INTF */
#define TSI_INTF_CTCF           BIT(0)                            /*!< charge transfer complete flag */
#define TSI_INTF_MNERR          BIT(1)                            /*!< max cycle number error */

/* TSI_PHM */
#define TSI_PHM_G0P0            BIT(0)                            /*!< pin G0P0 Schmitt trigger hysteresis state */
#define TSI_PHM_G0P1            BIT(1)                            /*!< pin G0P1 Schmitt trigger hysteresis state */
#define TSI_PHM_G0P2            BIT(2)                            /*!< pin G0P2 Schmitt trigger hysteresis state */
#define TSI_PHM_G0P3            BIT(3)                            /*!< pin G0P3 Schmitt trigger hysteresis state */
#define TSI_PHM_G1P0            BIT(4)                            /*!< pin G1P0 Schmitt trigger hysteresis state */
#define TSI_PHM_G1P1            BIT(5)                            /*!< pin G1P1 Schmitt trigger hysteresis state */
#define TSI_PHM_G1P2            BIT(6)                            /*!< pin G1P2 Schmitt trigger hysteresis state */
#define TSI_PHM_G1P3            BIT(7)                            /*!< pin G1P3 Schmitt trigger hysteresis state */
#define TSI_PHM_G2P0            BIT(8)                            /*!< pin G2P0 Schmitt trigger hysteresis state */
#define TSI_PHM_G2P1            BIT(9)                            /*!< pin G2P1 Schmitt trigger hysteresis state */
#define TSI_PHM_G2P2            BIT(10)                           /*!< pin G2P2 Schmitt trigger hysteresis state */
#define TSI_PHM_G2P3            BIT(11)                           /*!< pin G2P3 Schmitt trigger hysteresis state */
#define TSI_PHM_G3P0            BIT(12)                           /*!< pin G3P0 Schmitt trigger hysteresis state */
#define TSI_PHM_G3P1            BIT(13)                           /*!< pin G3P1 Schmitt trigger hysteresis state */
#define TSI_PHM_G3P2            BIT(14)                           /*!< pin G3P2 Schmitt trigger hysteresis state */
#define TSI_PHM_G3P3            BIT(15)                           /*!< pin G3P3 Schmitt trigger hysteresis state */
#define TSI_PHM_G4P0            BIT(16)                           /*!< pin G4P0 Schmitt trigger hysteresis state */
#define TSI_PHM_G4P1            BIT(17)                           /*!< pin G4P1 Schmitt trigger hysteresis state */
#define TSI_PHM_G4P2            BIT(18)                           /*!< pin G4P2 Schmitt trigger hysteresis state */
#define TSI_PHM_G4P3            BIT(19)                           /*!< pin G4P3 Schmitt trigger hysteresis state */
#define TSI_PHM_G5P0            BIT(20)                           /*!< pin G5P0 Schmitt trigger hysteresis state */
#define TSI_PHM_G5P1            BIT(21)                           /*!< pin G5P1 Schmitt trigger hysteresis state */
#define TSI_PHM_G5P2            BIT(22)                           /*!< pin G5P2 Schmitt trigger hysteresis state */
#define TSI_PHM_G5P3            BIT(23)                           /*!< pin G5P3 Schmitt trigger hysteresis state */

/* TSI_ASW */
#define TSI_ASW_G0P0            BIT(0)                            /*!< pin G0P0 analog switch state */
#define TSI_ASW_G0P1            BIT(1)                            /*!< pin G0P1 analog switch state */
#define TSI_ASW_G0P2            BIT(2)                            /*!< pin G0P2 analog switch state */
#define TSI_ASW_G0P3            BIT(3)                            /*!< pin G0P3 analog switch state */
#define TSI_ASW_G1P0            BIT(4)                            /*!< pin G1P0 analog switch state */
#define TSI_ASW_G1P1            BIT(5)                            /*!< pin G1P1 analog switch state */
#define TSI_ASW_G1P2            BIT(6)                            /*!< pin G1P2 analog switch state */
#define TSI_ASW_G1P3            BIT(7)                            /*!< pin G1P3 analog switch state */
#define TSI_ASW_G2P0            BIT(8)                            /*!< pin G2P0 analog switch state */
#define TSI_ASW_G2P1            BIT(9)                            /*!< pin G2P1 analog switch state */
#define TSI_ASW_G2P2            BIT(10)                           /*!< pin G2P2 analog switch state */
#define TSI_ASW_G2P3            BIT(11)                           /*!< pin G2P3 analog switch state */
#define TSI_ASW_G3P0            BIT(12)                           /*!< pin G3P0 analog switch state */
#define TSI_ASW_G3P1            BIT(13)                           /*!< pin G3P1 analog switch state */
#define TSI_ASW_G3P2            BIT(14)                           /*!< pin G3P2 analog switch state */
#define TSI_ASW_G3P3            BIT(15)                           /*!< pin G3P3 analog switch state */
#define TSI_ASW_G4P0            BIT(16)                           /*!< pin G4P0 analog switch state */
#define TSI_ASW_G4P1            BIT(17)                           /*!< pin G4P1 analog switch state */
#define TSI_ASW_G4P2            BIT(18)                           /*!< pin G4P2 analog switch state */
#define TSI_ASW_G4P3            BIT(19)                           /*!< pin G4P3 analog switch state */
#define TSI_ASW_G5P0            BIT(20)                           /*!< pin G5P0 analog switch state */
#define TSI_ASW_G5P1            BIT(21)                           /*!< pin G5P1 analog switch state */
#define TSI_ASW_G5P2            BIT(22)                           /*!< pin G5P2 analog switch state */
#define TSI_ASW_G5P3            BIT(23)                           /*!< pin G5P3 analog switch state */

/* TSI_SAMPCFG */
#define TSI_SAMPCFG_G0P0        BIT(0)                            /*!< pin G0P0 sample pin mode */
#define TSI_SAMPCFG_G0P1        BIT(1)                            /*!< pin G0P1 sample pin mode */
#define TSI_SAMPCFG_G0P2        BIT(2)                            /*!< pin G0P2 sample pin mode */
#define TSI_SAMPCFG_G0P3        BIT(3)                            /*!< pin G0P3 sample pin mode */
#define TSI_SAMPCFG_G1P0        BIT(4)                            /*!< pin G1P0 sample pin mode */
#define TSI_SAMPCFG_G1P1        BIT(5)                            /*!< pin G1P1 sample pin mode */
#define TSI_SAMPCFG_G1P2        BIT(6)                            /*!< pin G1P2 sample pin mode */
#define TSI_SAMPCFG_G1P3        BIT(7)                            /*!< pin G1P3 sample pin mode */
#define TSI_SAMPCFG_G2P0        BIT(8)                            /*!< pin G2P0 sample pin mode */
#define TSI_SAMPCFG_G2P1        BIT(9)                            /*!< pin G2P1 sample pin mode */
#define TSI_SAMPCFG_G2P2        BIT(10)                           /*!< pin G2P2 sample pin mode */
#define TSI_SAMPCFG_G2P3        BIT(11)                           /*!< pin G2P3 sample pin mode */
#define TSI_SAMPCFG_G3P0        BIT(12)                           /*!< pin G3P0 sample pin mode */
#define TSI_SAMPCFG_G3P1        BIT(13)                           /*!< pin G3P1 sample pin mode */
#define TSI_SAMPCFG_G3P2        BIT(14)                           /*!< pin G3P2 sample pin mode */
#define TSI_SAMPCFG_G3P3        BIT(15)                           /*!< pin G3P3 sample pin mode */
#define TSI_SAMPCFG_G4P0        BIT(16)                           /*!< pin G4P0 sample pin mode */
#define TSI_SAMPCFG_G4P1        BIT(17)                           /*!< pin G4P1 sample pin mode */
#define TSI_SAMPCFG_G4P2        BIT(18)                           /*!< pin G4P2 sample pin mode */
#define TSI_SAMPCFG_G4P3        BIT(19)                           /*!< pin G4P3 sample pin mode */
#define TSI_SAMPCFG_G5P0        BIT(20)                           /*!< pin G5P0 sample pin mode */
#define TSI_SAMPCFG_G5P1        BIT(21)                           /*!< pin G5P1 sample pin mode */
#define TSI_SAMPCFG_G5P2        BIT(22)                           /*!< pin G5P2 sample pin mode */
#define TSI_SAMPCFG_G5P3        BIT(23)                           /*!< pin G5P3 sample pin mode */

/* TSI_CHCFG */
#define TSI_CHCFG_G0P0          BIT(0)                            /*!< pin G0P0 channel pin mode */
#define TSI_CHCFG_G0P1          BIT(1)                            /*!< pin G0P1 channel pin mode */
#define TSI_CHCFG_G0P2          BIT(2)                            /*!< pin G0P2 channel pin mode */
#define TSI_CHCFG_G0P3          BIT(3)                            /*!< pin G0P3 channel pin mode */
#define TSI_CHCFG_G1P0          BIT(4)                            /*!< pin G1P0 channel pin mode */
#define TSI_CHCFG_G1P1          BIT(5)                            /*!< pin G1P1 channel pin mode */
#define TSI_CHCFG_G1P2          BIT(6)                            /*!< pin G1P2 channel pin mode */
#define TSI_CHCFG_G1P3          BIT(7)                            /*!< pin G1P3 channel pin mode */
#define TSI_CHCFG_G2P0          BIT(8)                            /*!< pin G2P0 channel pin mode */
#define TSI_CHCFG_G2P1          BIT(9)                            /*!< pin G2P1 channel pin mode */
#define TSI_CHCFG_G2P2          BIT(10)                           /*!< pin G2P2 channel pin mode */
#define TSI_CHCFG_G2P3          BIT(11)                           /*!< pin G2P3 channel pin mode */
#define TSI_CHCFG_G3P0          BIT(12)                           /*!< pin G3P0 channel pin mode */
#define TSI_CHCFG_G3P1          BIT(13)                           /*!< pin G3P1 channel pin mode */
#define TSI_CHCFG_G3P2          BIT(14)                           /*!< pin G3P2 channel pin mode */
#define TSI_CHCFG_G3P3          BIT(15)                           /*!< pin G3P3 channel pin mode */
#define TSI_CHCFG_G4P0          BIT(16)                           /*!< pin G4P0 channel pin mode */
#define TSI_CHCFG_G4P1          BIT(17)                           /*!< pin G4P1 channel pin mode */
#define TSI_CHCFG_G4P2          BIT(18)                           /*!< pin G4P2 channel pin mode */
#define TSI_CHCFG_G4P3          BIT(19)                           /*!< pin G4P3 channel pin mode */
#define TSI_CHCFG_G5P0          BIT(20)                           /*!< pin G5P0 channel pin mode */
#define TSI_CHCFG_G5P1          BIT(21)                           /*!< pin G5P1 channel pin mode */
#define TSI_CHCFG_G5P2          BIT(22)                           /*!< pin G5P2 channel pin mode */
#define TSI_CHCFG_G5P3          BIT(23)                           /*!< pin G5P3 channel pin mode */

/* TSI_GCTL */
#define TSI_GCTL_GE0            BIT(0)                            /*!< group0 enable */
#define TSI_GCTL_GE1            BIT(1)                            /*!< group1 enable */
#define TSI_GCTL_GE2            BIT(2)                            /*!< group2 enable */
#define TSI_GCTL_GE3            BIT(3)                            /*!< group3 enable */
#define TSI_GCTL_GE4            BIT(4)                            /*!< group4 enable */
#define TSI_GCTL_GE5            BIT(5)                            /*!< group5 enable */
#define TSI_GCTL_GC0            BIT(16)                           /*!< group0 complete */
#define TSI_GCTL_GC1            BIT(17)                           /*!< group1 complete */
#define TSI_GCTL_GC2            BIT(18)                           /*!< group2 complete */
#define TSI_GCTL_GC3            BIT(19)                           /*!< group3 complete */
#define TSI_GCTL_GC4            BIT(20)                           /*!< group4 complete */
#define TSI_GCTL_GC5            BIT(21)                           /*!< group5 complete */

/* TSI_CTL1 */
#define TSI_CTL1_CTCDIV         BIT(24)                           /*!< CTCLK clock division factor */
#define TSI_CTL1_ECDIV          BITS(28,29)                       /*!< ECCLK clock division factor */

#define CTL_CDT(regval)         (BITS(28,31) & ((uint32_t)(regval) << 28U))
#define CTL_CTDT(regval)        (BITS(24,27) & ((uint32_t)(regval) << 24U))
#define CTL_MCN(regval)         (BITS(5,7) & ((uint32_t)(regval) << 5U))
#define TSI_EXTENDMAX(regval)   (BITS(17,23) & ((uint32_t)(regval) << 17U)) /* value range 1..128, extend charge state maximum duration time */

#define TSI_INT_CCTCF           TSI_INTEN_CTCFIE                  /*!< charge transfer complete flag interrupt enable */
#define TSI_INT_MNERR           TSI_INTEN_MNERRIE                 /*!< max cycle number error interrupt enable */
#define TSI_FLAG_CTCF           TSI_INTF_CTCF                     /*!< charge transfer complete flag */
#define TSI_FLAG_MNERR          TSI_INTF_MNERR                    /*!< max cycle number error */

/* TSI GROUP IDX */
enum {
    TSI_GROUP_IDX0 = 0x00U,
    TSI_GROUP_IDX1,
    TSI_GROUP_IDX2,
    TSI_GROUP_IDX3,
    TSI_GROUP_IDX4,
    TSI_GROUP_IDX5,
    TSI_GROUP_NUMS
};

/* TSI state enum */
typedef enum {
    HAL_TSI_STATE_NONE = 0,                                       /*!< NONE(default value) */
    HAL_TSI_STATE_RESET,                                          /*!< RESET */
    HAL_TSI_STATE_BUSY,                                           /*!< BUSY */
    HAL_TSI_STATE_TIMEOUT,                                        /*!< TIMEOUT */
    HAL_TSI_STATE_ERROR,                                          /*!< ERROR */
    HAL_TSI_STATE_READY                                           /*!< READY */
} hal_tsi_state_enum;

/* TSI error type enum */
typedef enum {
    HAL_TSI_ERROR_NONE             = (uint32_t)0x00U,             /*!< no error */
    HAL_TSI_ERROR_SYSTEM           = (uint32_t)0x01U,             /*!< TSI internal error: if problem of clocking, enable/disable, wrong state */
    HAL_TSI_ERROR_MNERR            = (uint32_t)0x02U,             /*!< TSI max cycle number error */
    HAL_TSI_ERROR_CONFIG           = (uint32_t)0x04U              /*!< configuration error occurs */
} hal_tsi_error_enum;

/* TSI structure type enum */
typedef enum {
    HAL_TSI_INIT_STRUCT = 0,                                      /*!< TSI initialization structure */
    HAL_TSI_IRQ_STRUCT,                                           /*!< TSI irq structure */
    HAL_TSI_DEV_STRUCT                                            /*!< TSI device structure */
} hal_tsi_struct_type_enum;

/* TSI device interrupt callback function pointer structure */
typedef struct {
    __IO hal_irq_handle_cb           tsi_cctcf_handle;            /*!< TSI charge-transfer complete handler function */
    __IO hal_irq_handle_cb           tsi_mnerr_handle;            /*!< TSI max cycle number error handler function */
} hal_tsi_irq_struct;

/* TSI device information structrue */
typedef struct {
    hal_tsi_irq_struct               tsi_irq;                     /*!< TSI device interrupt callback function pointer structure */
    hal_tsi_error_enum               error_state;                 /*!< TSI error state */
    hal_tsi_state_enum               state;                       /*!< TSI state */
    hal_mutex_enum                   mutex;                       /*!< mutex */
    void                             *priv;                       /*!< priv data */
} hal_tsi_dev_struct;

/* @PARA: tsi_init */
/* @STRUCT: TSI init struct */
typedef struct{
    uint32_t charge_time;                                         /*!< charge state duration time */
    uint32_t transfer_time;                                       /*!< charge transfer state duration time */
    uint32_t ctclk_div;                                           /*!< charge transfer clock(ctclk) division factor */
    uint32_t seq_max;                                             /*!< max cycle number */
    uint32_t extend_charge_state;                                 /*!< extend charge state */
    uint32_t ecclk_div;                                           /*!< extend charge clock(ecclk) division factor */
    uint32_t extend_charge_time;                                  /*!< extend charge state maximum duration time */
    uint32_t pin_mode;                                            /*!< pin mode */
    uint32_t edge_sel;                                            /*!< edge selection */
    uint32_t trig_mode;                                           /*!< trigger mode selection */
    uint32_t sample_pins;                                         /*!< sample pins */
    uint32_t channel_pins;                                        /*!< channel pins */
    uint32_t shield_pins;                                         /*!< shield pins */
}hal_tsi_init_struct;

/* @STRUCT_MEMBER: charge_time */
/* @DEFINE: charge state duration time */
#define TSI_CHARGE_1CTCLK           CTL_CDT(0)                    /*!< the duration time of charge state is 1 CTCLK */
#define TSI_CHARGE_2CTCLK           CTL_CDT(1)                    /*!< the duration time of charge state is 2 CTCLK */
#define TSI_CHARGE_3CTCLK           CTL_CDT(2)                    /*!< the duration time of charge state is 3 CTCLK */
#define TSI_CHARGE_4CTCLK           CTL_CDT(3)                    /*!< the duration time of charge state is 4 CTCLK */
#define TSI_CHARGE_5CTCLK           CTL_CDT(4)                    /*!< the duration time of charge state is 5 CTCLK */
#define TSI_CHARGE_6CTCLK           CTL_CDT(5)                    /*!< the duration time of charge state is 6 CTCLK */
#define TSI_CHARGE_7CTCLK           CTL_CDT(6)                    /*!< the duration time of charge state is 7 CTCLK */
#define TSI_CHARGE_8CTCLK           CTL_CDT(7)                    /*!< the duration time of charge state is 8 CTCLK */
#define TSI_CHARGE_9CTCLK           CTL_CDT(8)                    /*!< the duration time of charge state is 9 CTCLK */
#define TSI_CHARGE_10CTCLK          CTL_CDT(9)                    /*!< the duration time of charge state is 10 CTCLK */
#define TSI_CHARGE_11CTCLK          CTL_CDT(10)                   /*!< the duration time of charge state is 11 CTCLK */
#define TSI_CHARGE_12CTCLK          CTL_CDT(11)                   /*!< the duration time of charge state is 12 CTCLK */
#define TSI_CHARGE_13CTCLK          CTL_CDT(12)                   /*!< the duration time of charge state is 13 CTCLK */
#define TSI_CHARGE_14CTCLK          CTL_CDT(13)                   /*!< the duration time of charge state is 14 CTCLK */
#define TSI_CHARGE_15CTCLK          CTL_CDT(14)                   /*!< the duration time of charge state is 15 CTCLK */
#define TSI_CHARGE_16CTCLK          CTL_CDT(15)                   /*!< the duration time of charge state is 16 CTCLK */

/* @STRUCT_MEMBER: transfer_time */
/* @DEFINE: charge transfer state duration time */
#define TSI_TRANSFER_1CTCLK         CTL_CTDT(0)                   /*!< the duration time of transfer state is 1 CTCLK */
#define TSI_TRANSFER_2CTCLK         CTL_CTDT(1)                   /*!< the duration time of transfer state is 2 CTCLK */
#define TSI_TRANSFER_3CTCLK         CTL_CTDT(2)                   /*!< the duration time of transfer state is 3 CTCLK */
#define TSI_TRANSFER_4CTCLK         CTL_CTDT(3)                   /*!< the duration time of transfer state is 4 CTCLK */
#define TSI_TRANSFER_5CTCLK         CTL_CTDT(4)                   /*!< the duration time of transfer state is 5 CTCLK */
#define TSI_TRANSFER_6CTCLK         CTL_CTDT(5)                   /*!< the duration time of transfer state is 6 CTCLK */
#define TSI_TRANSFER_7CTCLK         CTL_CTDT(6)                   /*!< the duration time of transfer state is 7 CTCLK */
#define TSI_TRANSFER_8CTCLK         CTL_CTDT(7)                   /*!< the duration time of transfer state is 8 CTCLK */
#define TSI_TRANSFER_9CTCLK         CTL_CTDT(8)                   /*!< the duration time of transfer state is 9 CTCLK */
#define TSI_TRANSFER_10CTCLK        CTL_CTDT(9)                   /*!< the duration time of transfer state is 10 CTCLK */
#define TSI_TRANSFER_11CTCLK        CTL_CTDT(10)                  /*!< the duration time of transfer state is 11 CTCLK */
#define TSI_TRANSFER_12CTCLK        CTL_CTDT(11)                  /*!< the duration time of transfer state is 12 CTCLK */
#define TSI_TRANSFER_13CTCLK        CTL_CTDT(12)                  /*!< the duration time of transfer state is 13 CTCLK */
#define TSI_TRANSFER_14CTCLK        CTL_CTDT(13)                  /*!< the duration time of transfer state is 14 CTCLK */
#define TSI_TRANSFER_15CTCLK        CTL_CTDT(14)                  /*!< the duration time of transfer state is 15 CTCLK */
#define TSI_TRANSFER_16CTCLK        CTL_CTDT(15)                  /*!< the duration time of transfer state is 16 CTCLK */

/* @STRUCT_MEMBER: ctclk_div */
/* @DEFINE: charge transfer clock(ctclk) division factor */
#define TSI_CTCDIV_DIV1            ((uint32_t)0x00000000U)         /*!< fCTCLK = fHCLK */
#define TSI_CTCDIV_DIV2            ((uint32_t)0x00000001U)         /*!< fCTCLK = fHCLK/2 */
#define TSI_CTCDIV_DIV4            ((uint32_t)0x00000002U)         /*!< fCTCLK = fHCLK/4 */
#define TSI_CTCDIV_DIV8            ((uint32_t)0x00000003U)         /*!< fCTCLK = fHCLK/8 */
#define TSI_CTCDIV_DIV16           ((uint32_t)0x00000004U)         /*!< fCTCLK = fHCLK/16 */
#define TSI_CTCDIV_DIV32           ((uint32_t)0x00000005U)         /*!< fCTCLK = fHCLK/32 */
#define TSI_CTCDIV_DIV64           ((uint32_t)0x00000006U)         /*!< fCTCLK = fHCLK/64 */
#define TSI_CTCDIV_DIV128          ((uint32_t)0x00000007U)         /*!< fCTCLK = fHCLK/128 */
#define TSI_CTCDIV_DIV256          ((uint32_t)0x00000008U)         /*!< fCTCLK = fHCLK/256 */
#define TSI_CTCDIV_DIV512          ((uint32_t)0x00000009U)         /*!< fCTCLK = fHCLK/512 */
#define TSI_CTCDIV_DIV1024         ((uint32_t)0x0000000AU)         /*!< fCTCLK = fHCLK/1024 */
#define TSI_CTCDIV_DIV2048         ((uint32_t)0x0000000BU)         /*!< fCTCLK = fHCLK/2048 */
#define TSI_CTCDIV_DIV4096         ((uint32_t)0x0000000CU)         /*!< fCTCLK = fHCLK/4096 */
#define TSI_CTCDIV_DIV8192         ((uint32_t)0x0000000DU)         /*!< fCTCLK = fHCLK/8192 */
#define TSI_CTCDIV_DIV16384        ((uint32_t)0x0000000EU)         /*!< fCTCLK = fHCLK/16384 */
#define TSI_CTCDIV_DIV32768        ((uint32_t)0x0000000FU)         /*!< fCTCLK = fHCLK/32768 */

/* @STRUCT_MEMBER: seq_max */
/* @DEFINE: max cycle number */
#define TSI_MAXNUM255              CTL_MCN(0)                      /*!< the max cycle number of a sequence is 255 */
#define TSI_MAXNUM511              CTL_MCN(1)                      /*!< the max cycle number of a sequence is 511 */
#define TSI_MAXNUM1023             CTL_MCN(2)                      /*!< the max cycle number of a sequence is 1023 */
#define TSI_MAXNUM2047             CTL_MCN(3)                      /*!< the max cycle number of a sequence is 2047 */
#define TSI_MAXNUM4095             CTL_MCN(4)                      /*!< the max cycle number of a sequence is 4095 */
#define TSI_MAXNUM8191             CTL_MCN(5)                      /*!< the max cycle number of a sequence is 8191 */
#define TSI_MAXNUM16383            CTL_MCN(6)                      /*!< the max cycle number of a sequence is 16383 */

/* @STRUCT_MEMBER: extend_charge_state */
/* @DEFINE: extend charge state */
#define TSI_EXTEND_CHARGE_DISABLE  ((uint32_t)0x00000000U)         /*!< extend charge state disable */
#define TSI_EXTEND_CHARGE_ENABLE   ((uint32_t)0x00000001U)         /*!< extend charge state enable */

/* @STRUCT_MEMBER: ecclk_div */
/* @DEFINE: extend charge clock(ecclk) division factor */
#define TSI_EXTEND_DIV1            ((uint32_t)0x00000000U)         /*!< fECCLK = fHCLK */
#define TSI_EXTEND_DIV2            ((uint32_t)0x00000001U)         /*!< fECCLK = fHCLK/2 */
#define TSI_EXTEND_DIV3            ((uint32_t)0x00000002U)         /*!< fECCLK = fHCLK/3 */
#define TSI_EXTEND_DIV4            ((uint32_t)0x00000003U)         /*!< fECCLK = fHCLK/4 */
#define TSI_EXTEND_DIV5            ((uint32_t)0x00000004U)         /*!< fECCLK = fHCLK/5 */
#define TSI_EXTEND_DIV6            ((uint32_t)0x00000005U)         /*!< fECCLK = fHCLK/6 */
#define TSI_EXTEND_DIV7            ((uint32_t)0x00000006U)         /*!< fECCLK = fHCLK/7 */
#define TSI_EXTEND_DIV8            ((uint32_t)0x00000007U)         /*!< fECCLK = fHCLK/8 */

/* @STRUCT_MEMBER: extend_charge_time */
/* @=NULL */

/* @STRUCT_MEMBER: pin_mode */
/* @DEFINE: pin mode */
#define TSI_OUTPUT_LOW             ((uint32_t)0x00000000U)         /*!< TSI pin will output low when IDLE */
#define TSI_INPUT_FLOATING         TSI_CTL0_PINMOD                 /*!< TSI pin will keep input_floating when IDLE */

/* @STRUCT_MEMBER: edge_sel */
/* @DEFINE: edge selection */
#define TSI_FALLING_TRIGGER        ((uint32_t)0x00000000U)         /*!< falling edge trigger TSI charge transfer sequence */
#define TSI_RISING_TRIGGER         TSI_CTL0_EGSEL                  /*!< rising edge trigger TSI charge transfer sequence */

/* @PARA: trig_mode */
/* @DEFINE:tsi hardware trigger */
#define TSI_HW_TRIGGER_DISABLE      ((uint32_t)0x00000000U)         /*!< hardware trigger disable */
#define TSI_HW_TRIGGER_ENABLE       TSI_CTL0_TRGMOD                 /*!< hardware trigger enable */

/* @STRUCT_MEMBER: sample_pins */
/* @DEFINE: sample_pins selection */
#define TSI_G0P0             BIT(0)                                 /*!< TSI G0P0 */
#define TSI_G0P1             BIT(1)                                 /*!< TSI G0P1 */
#define TSI_G0P2             BIT(2)                                 /*!< TSI G0P2 */
#define TSI_G0P3             BIT(3)                                 /*!< TSI G0P3 */
#define TSI_G1P0             BIT(4)                                 /*!< TSI G1P0 */
#define TSI_G1P1             BIT(5)                                 /*!< TSI G1P1 */
#define TSI_G1P2             BIT(6)                                 /*!< TSI G1P2 */
#define TSI_G1P3             BIT(7)                                 /*!< TSI G1P3 */
#define TSI_G2P0             BIT(8)                                 /*!< TSI G2P0 */
#define TSI_G2P1             BIT(9)                                 /*!< TSI G2P1 */
#define TSI_G2P2             BIT(10)                                /*!< TSI G2P2 */
#define TSI_G2P3             BIT(11)                                /*!< TSI G2P3 */
#define TSI_G3P0             BIT(12)                                /*!< TSI G3P0 */
#define TSI_G3P1             BIT(13)                                /*!< TSI G3P1 */
#define TSI_G3P2             BIT(14)                                /*!< TSI G3P2 */
#define TSI_G3P3             BIT(15)                                /*!< TSI G3P3 */
#define TSI_G4P0             BIT(16)                                /*!< TSI G4P0 */
#define TSI_G4P1             BIT(17)                                /*!< TSI G4P1 */
#define TSI_G4P2             BIT(18)                                /*!< TSI G4P2 */
#define TSI_G4P3             BIT(19)                                /*!< TSI G4P3 */
#define TSI_G5P0             BIT(20)                                /*!< TSI G5P0 */
#define TSI_G5P1             BIT(21)                                /*!< TSI G5P1 */
#define TSI_G5P2             BIT(22)                                /*!< TSI G5P2 */
#define TSI_G5P3             BIT(23)                                /*!< TSI G5P3 */

/* @STRUCT_MEMBER: channel_pins */
/* @DEFINE: channel_pins selection */
/* @REFER: TSI_G0P0 */

/* @STRUCT_MEMBER: shield_pins */
/* @DEFINE: shield_pins selection */
/* @REFER: TSI_G0P0 */
/* @FUNCTION: initialize TSI */
int32_t hal_tsi_init(hal_tsi_dev_struct *tsi_dev, hal_tsi_init_struct *tsi_init);

/* @END */

/* hal interface */
/* initialize the TSI structure with the default values */
void hal_tsi_struct_init(hal_tsi_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize TSI device structure and init structure */
int32_t hal_tsi_deinit(hal_tsi_dev_struct *tsi_dev);
/* start TSI module function */
int32_t hal_tsi_start(hal_tsi_dev_struct *tsi_dev);
/* stop tsi module function */
int32_t hal_tsi_stop(hal_tsi_dev_struct *tsi_dev);
/* start TSI interrupt */
int32_t hal_tsi_start_interrupt(hal_tsi_dev_struct *tsi_dev, hal_tsi_irq_struct *p_irq);
/* stop TSI interrupt */
int32_t hal_tsi_stop_interrupt(hal_tsi_dev_struct *tsi_dev);
/* TSI interrupt handler content function, which is merely used in tsi_handler */
void hal_tsi_irq(hal_tsi_dev_struct *tsi_dev);
/* set user-defined interrupt callback function
    which will be registered and called when corresponding interrupt be triggered */
void hal_tsi_irq_handle_set(hal_tsi_dev_struct *tsi_dev, hal_tsi_irq_struct *p_irq);
/* reset all user-defined interrupt callback function
    which will be registered and called when corresponding interrupt be triggered */
void hal_tsi_irq_handle_all_reset(hal_tsi_dev_struct *tsi_dev);
/* get the cycle number of specific group as soon as a charge-transfer sequence completes */
uint16_t hal_tsi_group_cycle_get(hal_tsi_dev_struct *tsi_dev, uint8_t group_id);
/* configure TSI pins */
void hal_tsi_pins_config(hal_tsi_dev_struct *tsi_dev, hal_tsi_init_struct *pins_config);
/* TSI poll for charge-transfer sequence complete */
void hal_tsi_poll_transfer(hal_tsi_dev_struct *tsi_dev);

/* simple interface */
/* enable TSI module */
void hals_tsi_enable(void);
/* disable TSI module */
void hals_tsi_disable(void);
/* enable sample pin */
void hals_tsi_sample_pin_enable(uint32_t sample);
/* disable sample pin */
void hals_tsi_sample_pin_disable(uint32_t sample);
/* enable channel pin */
void hals_tsi_channel_pin_enable(uint32_t channel);
/* disable channel pin */
void hals_tsi_channel_pin_disable(uint32_t channel);

/* start a charge-transfer sequence when TSI is in software trigger mode */
void hals_tsi_software_start(void);
/* stop a charge-transfer sequence when TSI is in software trigger mode */
void hals_tsi_software_stop(void);
/* configure TSI triggering by hardware */
void hals_tsi_hardware_mode_config(uint8_t trigger_edge);
/* configure TSI pin mode when charge-transfer sequence is IDLE */
void hals_tsi_pin_mode_config(uint8_t pin_mode);
/* configure extend charge state */
void hals_tsi_extend_charge_config(ControlStatus extend, uint8_t prescaler, uint32_t max_duration);

/* configure charge pulse and transfer pulse */
void hals_tsi_pulse_config(uint32_t prescaler, uint32_t charge_duration, uint32_t transfer_duration);
/* configure the max cycle number of a charge-transfer sequence */
void hals_tsi_max_number_config(uint32_t max_number);
/* switch on hysteresis pin */
void hals_tsi_hysteresis_on(uint32_t group_pin);
/* switch off hysteresis pin */
void hals_tsi_hysteresis_off(uint32_t group_pin);
/* switch on analog pin */
void hals_tsi_analog_on(uint32_t group_pin);
/* switch off analog pin */
void hals_tsi_analog_off(uint32_t group_pin);

/* enable group */
void hals_tsi_group_enable(uint32_t group);
/* disable group */
void hals_tsi_group_disable(uint32_t group);
/* get group complete status */
FlagStatus hals_tsi_group_status_get(uint32_t group);

/* get TSI flag */
FlagStatus hals_tsi_flag_get(uint32_t flag);
/* clear TSI flag */
void hals_tsi_flag_clear(uint32_t flag);
/* enable TSI interrupt */
void hals_tsi_interrupt_enable(uint32_t source);
/* disable TSI interrupt */
void hals_tsi_interrupt_disable(uint32_t source);

#endif /* GD32F350 */

#endif /* GD32F3X0_HAL_TSI_H */
