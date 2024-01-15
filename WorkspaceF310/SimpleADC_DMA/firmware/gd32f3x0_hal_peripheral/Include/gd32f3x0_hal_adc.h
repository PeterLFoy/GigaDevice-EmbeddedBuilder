/*!
    \file    gd32f3x0_hal_adc.h
    \brief   definitions for the ADC

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

#ifndef GD32F3X0_HAL_ADC_H
#define GD32F3X0_HAL_ADC_H

#include "gd32f3x0_hal.h"

/* ADC definitions */
#define ADC                              ADC_BASE                                    /*!< ADC base address */

/* registers definitions */
#define ADC_STAT                         REG32(ADC + 0x00000000U)                    /*!< ADC status register */
#define ADC_CTL0                         REG32(ADC + 0x00000004U)                    /*!< ADC control register 0 */
#define ADC_CTL1                         REG32(ADC + 0x00000008U)                    /*!< ADC control register 1 */
#define ADC_SAMPT0                       REG32(ADC + 0x0000000CU)                    /*!< ADC sample time register 0 */
#define ADC_SAMPT1                       REG32(ADC + 0x00000010U)                    /*!< ADC sample time register 1 */
#define ADC_IOFF0                        REG32(ADC + 0x00000014U)                    /*!< ADC inserted channel data offset register 0 */
#define ADC_IOFF1                        REG32(ADC + 0x00000018U)                    /*!< ADC inserted channel data offset register 1 */
#define ADC_IOFF2                        REG32(ADC + 0x0000001CU)                    /*!< ADC inserted channel data offset register 2 */
#define ADC_IOFF3                        REG32(ADC + 0x00000020U)                    /*!< ADC inserted channel data offset register 3 */
#define ADC_WDHT                         REG32(ADC + 0x00000024U)                    /*!< ADC watchdog high threshold register */
#define ADC_WDLT                         REG32(ADC + 0x00000028U)                    /*!< ADC watchdog low threshold register */
#define ADC_RSQ0                         REG32(ADC + 0x0000002CU)                    /*!< ADC routine sequence register 0 */
#define ADC_RSQ1                         REG32(ADC + 0x00000030U)                    /*!< ADC routine sequence register 1 */
#define ADC_RSQ2                         REG32(ADC + 0x00000034U)                    /*!< ADC routine sequence register 2 */
#define ADC_ISQ                          REG32(ADC + 0x00000038U)                    /*!< ADC inserted sequence register */
#define ADC_IDATA0                       REG32(ADC + 0x0000003CU)                    /*!< ADC inserted data register 0 */
#define ADC_IDATA1                       REG32(ADC + 0x00000040U)                    /*!< ADC inserted data register 1 */
#define ADC_IDATA2                       REG32(ADC + 0x00000044U)                    /*!< ADC inserted data register 2 */
#define ADC_IDATA3                       REG32(ADC + 0x00000048U)                    /*!< ADC inserted data register 3 */
#define ADC_RDATA                        REG32(ADC + 0x0000004CU)                    /*!< ADC routine data register */
#define ADC_OVSAMPCTL                    REG32(ADC + 0x00000080U)                    /*!< ADC oversampling control register */

/* bits definitions */
/* ADC_STAT */
#define ADC_STAT_WDE                     BIT(0)                                      /*!< analog watchdog event flag */
#define ADC_STAT_EOC                     BIT(1)                                      /*!< end of conversion flag */
#define ADC_STAT_EOIC                    BIT(2)                                      /*!< inserted channel end of conversion flag */
#define ADC_STAT_STIC                    BIT(3)                                      /*!< inserted channel start flag */
#define ADC_STAT_STRC                    BIT(4)                                      /*!< routine channel start flag */

/* ADC_CTL0 */
#define ADC_CTL0_WDCHSEL                 BITS(0,4)                                   /*!< analog watchdog channel select bits */
#define ADC_CTL0_EOCIE                   BIT(5)                                      /*!< interrupt enable for EOC */
#define ADC_CTL0_WDEIE                   BIT(6)                                      /*!< analog watchdog interrupt enable */
#define ADC_CTL0_EOICIE                  BIT(7)                                      /*!< interrupt enable for inserted channels */
#define ADC_CTL0_SM                      BIT(8)                                      /*!< scan mode */
#define ADC_CTL0_WDSC                    BIT(9)                                      /*!< when in scan mode, analog watchdog is effective on a single channel */
#define ADC_CTL0_ICA                     BIT(10)                                     /*!< automatic inserted sequence conversion */
#define ADC_CTL0_DISRC                   BIT(11)                                     /*!< discontinuous mode on routine channels */
#define ADC_CTL0_DISIC                   BIT(12)                                     /*!< discontinuous mode on inserted channels */
#define ADC_CTL0_DISNUM                  BITS(13,15)                                 /*!< discontinuous mode channel count */
#define ADC_CTL0_IWDEN                   BIT(22)                                     /*!< analog watchdog enable on inserted channels */
#define ADC_CTL0_RWDEN                   BIT(23)                                     /*!< analog watchdog enable on routine channels */
#define ADC_CTL0_DRES                    BITS(24,25)                                 /*!< ADC data resolution */

/* ADC_CTL1 */
#define ADC_CTL1_ADCON                   BIT(0)                                      /*!< ADC converter on */
#define ADC_CTL1_CTN                     BIT(1)                                      /*!< continuous conversion */
#define ADC_CTL1_CLB                     BIT(2)                                      /*!< ADC calibration */
#define ADC_CTL1_RSTCLB                  BIT(3)                                      /*!< reset calibration */
#define ADC_CTL1_DMA                     BIT(8)                                      /*!< direct memory access mode */
#define ADC_CTL1_DAL                     BIT(11)                                     /*!< data alignment */
#define ADC_CTL1_ETSIC                   BITS(12,14)                                 /*!< external trigger select for inserted channel */
#define ADC_CTL1_ETEIC                   BIT(15)                                     /*!< external trigger enable for inserted channel */
#define ADC_CTL1_ETSRC                   BITS(17,19)                                 /*!< external trigger select for routine channel */
#define ADC_CTL1_ETERC                   BIT(20)                                     /*!< external trigger enable for routine channel */
#define ADC_CTL1_SWICST                  BIT(21)                                     /*!< start on inserted channel */
#define ADC_CTL1_SWRCST                  BIT(22)                                     /*!< start on routine channel */
#define ADC_CTL1_TSVREN                  BIT(23)                                     /*!< enable channel 16 and 17 */
#define ADC_CTL1_VBETEN                  BIT(24)                                     /*!< VBAT enable */

/* ADC_SAMPTx x=0,1 */
#define ADC_SAMPTX_SPTN                  BITS(0,2)                                   /*!< channel n(n=0..18) sample time selection */

/* ADC_IOFFx x=0..3 */
#define ADC_IOFFX_IOFF                   BITS(0,11)                                  /*!< data offset for inserted channel x */

/* ADC_WDHT */
#define ADC_WDHT_WDHT                    BITS(0,11)                                  /*!< analog watchdog high threshold */

/* ADC_WDLT */
#define ADC_WDLT_WDLT                    BITS(0,11)                                  /*!< analog watchdog low threshold */

/* ADC_RSQx x=0..2 */
#define ADC_RSQX_RSQN                    BITS(0,4)                                   /*!< n conversion in routine sequence */
#define ADC_RSQ0_RL                      BITS(20,23)                                 /*!< routine channel sequence length */

/* ADC_ISQ */
#define ADC_ISQ_ISQN                     BITS(0,4)                                   /*!< n conversion in routine sequence */
#define ADC_ISQ_IL                       BITS(20,21)                                 /*!< inserted sequence length */

/* ADC_IDATAx x=0..3 */
#define ADC_IDATAX_IDATAN                BITS(0,15)                                  /*!< inserted channel x conversion data  */

/* ADC_RDATA */
#define ADC_RDATA_RDATA                  BITS(0,15)                                  /*!< routine channel data */

/* ADC_OVSAMPCTL */
#define ADC_OVSAMPCTL_OVSEN              BIT(0)                                      /*!< oversampling enable */
#define ADC_OVSAMPCTL_OVSR               BITS(2,4)                                   /*!< oversampling ratio */
#define ADC_OVSAMPCTL_OVSS               BITS(5,8)                                   /*!< oversampling shift */
#define ADC_OVSAMPCTL_TOVS               BIT(9)                                      /*!< triggered oversampling */

/* constants definitions */
/* ADC flag definitions */
#define ADC_FLAG_WDE                     ADC_STAT_WDE                                /*!< analog watchdog event flag */
#define ADC_FLAG_EOC                     ADC_STAT_EOC                                /*!< end of sequence conversion flag */
#define ADC_FLAG_EOIC                    ADC_STAT_EOIC                               /*!< end of inserted channel sequence conversion flag */
#define ADC_FLAG_STIC                    ADC_STAT_STIC                               /*!< start flag of inserted channel sequence */
#define ADC_FLAG_STRC                    ADC_STAT_STRC                               /*!< start flag of routine channel sequence */

/* ADC_CTL0 register value */
#define CTL0_DISNUM(regval)              (BITS(13,15) & ((uint32_t)(regval) << 13))  /*!< number of conversions in discontinuous mode */

/* ADC special function */
#define ADC_SCAN_MODE                    ADC_CTL0_SM                                 /*!< scan mode */
#define ADC_INSERTED_CHANNEL_AUTO        ADC_CTL0_ICA                                /*!< inserted channel sequence convert automatically */
#define ADC_CONTINUOUS_MODE              ADC_CTL1_CTN                                /*!< continuous mode */

/* external trigger select for routine channel */
#define CTL1_ETSRC(regval)               (BITS(17,19) & ((uint32_t)(regval) << 17))

/* external trigger select for inserted channel */
#define CTL1_ETSIC(regval)               (BITS(12,14) & ((uint32_t)(regval) << 12))

/* adc_samptx register value */
#define SAMPTX_SPT(regval)               (BITS(0,2) & ((uint32_t)(regval) << 0))

/* ADC data offset for inserted channel x */
#define IOFFX_IOFF(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))

/* ADC analog watchdog high threshold */
#define WDHT_WDHT(regval)                (BITS(0,11) & ((uint32_t)(regval) << 0))

/* ADC analog watchdog low  threshold */
#define WDLT_WDLT(regval)                (BITS(0,11) & ((uint32_t)(regval) << 0))

/* ADC routine channel sequence length */
#define RSQ0_RL(regval)                  (BITS(20,23) & ((uint32_t)(regval) << 20))

/* ADC inserted channel sequence length */
#define ISQ_IL(regval)                   (BITS(20,21) & ((uint32_t)(regval) << 20))

/* ADC resolution definitions */
#define CTL0_DRES(regval)                (BITS(24,25) & ((uint32_t)(regval) << 24))

/* ADC oversampling shift */
#define OVSAMPCTL_OVSS(regval)           (BITS(5,8) & ((uint32_t)(regval) << 5))
#define ADC_OVERSAMPLING_SHIFT_NONE      OVSAMPCTL_OVSS(0)                           /*!< no oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_1B        OVSAMPCTL_OVSS(1)                           /*!< 1-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_2B        OVSAMPCTL_OVSS(2)                           /*!< 2-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_3B        OVSAMPCTL_OVSS(3)                           /*!< 3-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_4B        OVSAMPCTL_OVSS(4)                           /*!< 4-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_5B        OVSAMPCTL_OVSS(5)                           /*!< 5-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_6B        OVSAMPCTL_OVSS(6)                           /*!< 6-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_7B        OVSAMPCTL_OVSS(7)                           /*!< 7-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_8B        OVSAMPCTL_OVSS(8)                           /*!< 8-bit oversampling shift */

/* ADC oversampling ratio */
#define OVSAMPCTL_OVSR(regval)           (BITS(2,4) & ((uint32_t)(regval) << 2))
#define ADC_OVERSAMPLING_RATIO_MUL2      OVSAMPCTL_OVSR(0)                           /*!< oversampling ratio multiple 2 */
#define ADC_OVERSAMPLING_RATIO_MUL4      OVSAMPCTL_OVSR(1)                           /*!< oversampling ratio multiple 4 */
#define ADC_OVERSAMPLING_RATIO_MUL8      OVSAMPCTL_OVSR(2)                           /*!< oversampling ratio multiple 8 */
#define ADC_OVERSAMPLING_RATIO_MUL16     OVSAMPCTL_OVSR(3)                           /*!< oversampling ratio multiple 16 */
#define ADC_OVERSAMPLING_RATIO_MUL32     OVSAMPCTL_OVSR(4)                           /*!< oversampling ratio multiple 32 */
#define ADC_OVERSAMPLING_RATIO_MUL64     OVSAMPCTL_OVSR(5)                           /*!< oversampling ratio multiple 64 */
#define ADC_OVERSAMPLING_RATIO_MUL128    OVSAMPCTL_OVSR(6)                           /*!< oversampling ratio multiple 128 */
#define ADC_OVERSAMPLING_RATIO_MUL256    OVSAMPCTL_OVSR(7)                           /*!< oversampling ratio multiple 256 */

/* ADC channel sequence definitions */
#define ADC_ROUTINE_CHANNEL              ((uint8_t)0x01U)                            /*!< ADC routine channel sequence */
#define ADC_INSERTED_CHANNEL             ((uint8_t)0x02U)                            /*!< ADC inserted channel sequence */
#define ADC_ROUTINE_INSERTED_CHANNEL     ((uint8_t)0x03U)                            /*!< both routine and inserted channel sequence */
#define ADC_CHANNEL_DISCON_DISABLE       ((uint8_t)0x04U)                            /*!< disable discontinuous mode of routine & inserted channel */

/* ADC inserted channel definitions */
#define ADC_INSERTED_CHANNEL_0           ((uint8_t)0x00U)                            /*!< ADC inserted channel 0 */
#define ADC_INSERTED_CHANNEL_1           ((uint8_t)0x01U)                            /*!< ADC inserted channel 1 */
#define ADC_INSERTED_CHANNEL_2           ((uint8_t)0x02U)                            /*!< ADC inserted channel 2 */
#define ADC_INSERTED_CHANNEL_3           ((uint8_t)0x03U)                            /*!< ADC inserted channel 3 */

/* ADC interrupt definitions */
#define ADC_INT_WDE                      ADC_STAT_WDE                                /*!< analog watchdog event interrupt */
#define ADC_INT_EOC                      ADC_STAT_EOC                                /*!< end of sequence conversion interrupt */
#define ADC_INT_EOIC                     ADC_STAT_EOIC                               /*!< end of inserted sequence conversion interrupt */

/* ADC interrupt flag */
#define ADC_INT_FLAG_WDE                 ADC_STAT_WDE                                /*!< analog watchdog event interrupt flag */
#define ADC_INT_FLAG_EOC                 ADC_STAT_EOC                                /*!< end of sequence conversion interrupt flag */
#define ADC_INT_FLAG_EOIC                ADC_STAT_EOIC                               /*!< end of inserted sequence conversion interrupt flag */

/* the callback of ADC interrupt declaration */
typedef void (*hal_adc_dma_handle_cb)(void *ptr);

/* ADC structure type enum */
typedef enum {
    HAL_ADC_INIT_STRUCT,                                                             /*!< ADC initialization structure */
    HAL_ADC_ROUTINE_RANK_CONFIG_STRUCT,                                              /*!< ADC routine rank configuration structure */
    HAL_ADC_ROUTINE_CONFIG_STRUCT,                                                   /*!< ADC routine channel configuration structure */
    HAL_ADC_INSERTED_RANK_CONFIG_STRUCT,                                             /*!< ADC inserted rank configuration structure */
    HAL_ADC_INSERTED_CONFIG_STRUCT,                                                  /*!< ADC inserted channel configuration structure */
    HAL_ADC_IRQ_STRUCT,                                                              /*!< ADC device interrupt callback function pointer structure */
    HAL_ADC_DMA_HANDLE_CB_STRUCT,                                                    /*!< ADC DMA callback function pointer structure */
    HAL_ADC_WATCHDOG_CONFIG_STRUCT,                                                  /*!< ADC watchdog configuration structure */
    HAL_ADC_DEV_STRUCT                                                               /*!< ADC device information structrue */
} hal_adc_struct_type_enum;

/* ADC state type enum */
typedef enum {
    HAL_ADC_STATE_RESET         = (uint32_t)0x00000000U,                             /*!< ADC is not initialized or disabled */
    HAL_ADC_STATE_READY         = (uint32_t)0x00000001U,                             /*!< ADC is ready */
    HAL_ADC_STATE_BUSY_SYSTEM   = (uint32_t)0x00000002U,                             /*!< ADC is busy to internal system (initialization, calibration) */
    HAL_ADC_STATE_TIMEOUT       = (uint32_t)0x00000004U,                             /*!< ADC timeout occurs */
    HAL_ADC_STATE_ROUTINE_BUSY  = (uint32_t)0x00000010U,                             /*!< a conversion is ongoing on routine sequence */
    HAL_ADC_STATE_ROUTINE_EOC   = (uint32_t)0x00000020U,                             /*!< conversion data available on routine sequence */
    HAL_ADC_STATE_INSERTED_BUSY = (uint32_t)0x00000100U,                             /*!< a conversion is ongoing on inserted sequence */
    HAL_ADC_STATE_INSERTED_EOC  = (uint32_t)0x00000200U,                             /*!< conversion data available on inserted sequence */
    HAL_ADC_STATE_WATCHDOG      = (uint32_t)0x00001000U,                             /*!< analog watchdog */
} hal_adc_state_enum;

/* ADC error type enum */
typedef enum {
    HAL_ADC_ERROR_NONE   = (uint32_t)0x00U,                                          /*!< no error */
    HAL_ADC_ERROR_SYSTEM = (uint32_t)0x01U,                                          /*!< ADC internal error: if problem of clocking, enable/disable, wrong state */
    HAL_ADC_ERROR_DMA    = (uint32_t)0x02U,                                          /*!< DMA transfer error */
    HAL_ADC_ERROR_CONFIG = (uint32_t)0x04U,                                          /*!< configuration error occurs */
} hal_adc_error_enum;

/* ADC device interrupt callback function pointer structure */
typedef struct {
    hal_irq_handle_cb                adc_eoc_handle;                                 /*!< EOC interrupt handler function */
    hal_irq_handle_cb                adc_eoic_handle;                                /*!< EOIC interrupt handler function */
    hal_irq_handle_cb                adc_watchdog_handle;                            /*!< watchdog event interrupt handler function */
} hal_adc_irq_struct;

/* ADC DMA callback function pointer structure */
typedef struct {
    __IO hal_adc_dma_handle_cb       full_transcom_handle;                           /*!< DAC DMA transfer complete interrupt handler function */
    __IO hal_adc_dma_handle_cb       half_transcom_handle;                           /*!< DAC DMA transfer complete interrupt handler function */
    __IO hal_adc_dma_handle_cb       error_handle;                                   /*!< DAC DMA error underflow handler function */
} hal_adc_dma_handle_cb_struct;

/* ADC device information structrue */
typedef struct {
    hal_adc_irq_struct               adc_irq;                                        /*!< ADC device interrupt callback function pointer structure */
    hal_dma_dev_struct               *p_dma_adc;                                     /*!< DMA device information structrue */
    hal_adc_dma_handle_cb_struct     adc_dma;                                        /*!< DMA callback function pointer structure */
    hal_adc_error_enum               error_state;                                    /*!< ADC error state */
    hal_mutex_enum                   mutex;                                          /*!< ADC mutex set */
    hal_adc_state_enum               state;                                          /*!< ADC state */
} hal_adc_dev_struct;

/* @STRUCT_MEMBER: oversampling_shift */
/* @ENUM:  oversampling shift */
typedef enum {
    ADC_OVERSAMPLE_SHIFT_NONE = (uint16_t)(ADC_OVERSAMPLING_SHIFT_NONE),             /*!< no oversampling shift */
    ADC_OVERSAMPLE_SHIFT_1B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_1B),               /*!< 1-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_2B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_2B),               /*!< 2-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_3B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_3B),               /*!< 3-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_4B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_4B),               /*!< 4-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_5B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_5B),               /*!< 5-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_6B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_6B),               /*!< 6-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_7B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_7B),               /*!< 7-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_8B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_8B),               /*!< 8-bit oversampling shift */
} hal_adc_oversample_shift_enum;

/* @STRUCT_MEMBER: oversampling_ratio */
/* @ENUM:  oversampling ratio */
typedef enum {
    ADC_OVERSAMPLE_RATIO_MUL2   = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL2),            /*!< oversampling ratio multiple 2 */
    ADC_OVERSAMPLE_RATIO_MUL4   = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL4),            /*!< oversampling ratio multiple 4 */
    ADC_OVERSAMPLE_RATIO_MUL8   = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL8),            /*!< oversampling ratio multiple 8 */
    ADC_OVERSAMPLE_RATIO_MUL16  = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL16),           /*!< oversampling ratio multiple 16 */
    ADC_OVERSAMPLE_RATIO_MUL32  = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL32),           /*!< oversampling ratio multiple 32 */
    ADC_OVERSAMPLE_RATIO_MUL64  = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL64),           /*!< oversampling ratio multiple 64 */
    ADC_OVERSAMPLE_RATIO_MUL128 = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL128),          /*!< oversampling ratio multiple 128 */
    ADC_OVERSAMPLE_RATIO_MUL256 = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL256),          /*!< oversampling ratio multiple 256 */
} hal_adc_oversample_ratio_enum;

/* @STRUCT_MEMBER: routine_sequence */
/* @ENUM:  routine rank sequence */
typedef enum {
    ADC_ROUTINE_SEQUENCE_0  = (uint8_t)0x00U,                                        /*!< ADC routine channel sequence 0 */
    ADC_ROUTINE_SEQUENCE_1  = (uint8_t)0x01U,                                        /*!< ADC routine channel sequence 1 */
    ADC_ROUTINE_SEQUENCE_2  = (uint8_t)0x02U,                                        /*!< ADC routine channel sequence 2 */
    ADC_ROUTINE_SEQUENCE_3  = (uint8_t)0x03U,                                        /*!< ADC routine channel sequence 3 */
    ADC_ROUTINE_SEQUENCE_4  = (uint8_t)0x04U,                                        /*!< ADC routine channel sequence 4 */
    ADC_ROUTINE_SEQUENCE_5  = (uint8_t)0x05U,                                        /*!< ADC routine channel sequence 5 */
    ADC_ROUTINE_SEQUENCE_6  = (uint8_t)0x06U,                                        /*!< ADC routine channel sequence 6 */
    ADC_ROUTINE_SEQUENCE_7  = (uint8_t)0x07U,                                        /*!< ADC routine channel sequence 7 */
    ADC_ROUTINE_SEQUENCE_8  = (uint8_t)0x08U,                                        /*!< ADC routine channel sequence 8 */
    ADC_ROUTINE_SEQUENCE_9  = (uint8_t)0x09U,                                        /*!< ADC routine channel sequence 9 */
    ADC_ROUTINE_SEQUENCE_10 = (uint8_t)0x0AU,                                        /*!< ADC routine channel sequence 10 */
    ADC_ROUTINE_SEQUENCE_11 = (uint8_t)0x0BU,                                        /*!< ADC routine channel sequence 11 */
    ADC_ROUTINE_SEQUENCE_12 = (uint8_t)0x0CU,                                        /*!< ADC routine channel sequence 12 */
    ADC_ROUTINE_SEQUENCE_13 = (uint8_t)0x0DU,                                        /*!< ADC routine channel sequence 13 */
    ADC_ROUTINE_SEQUENCE_14 = (uint8_t)0x0EU,                                        /*!< ADC routine channel sequence 14 */
    ADC_ROUTINE_SEQUENCE_15 = (uint8_t)0x0FU,                                        /*!< ADC routine channel sequence 15 */
} hal_adc_routine_sequence_enum;

/* @STRUCT_MEMBER: inserted_sequence */
/* @ENUM:  inserted rank sequence */
typedef enum {
    ADC_INSERTED_SEQUENCE_0 = (uint8_t)0x00U,                                        /*!< ADC inserted channel sequence 0 */
    ADC_INSERTED_SEQUENCE_1 = (uint8_t)0x01U,                                        /*!< ADC inserted channel sequence 1 */
    ADC_INSERTED_SEQUENCE_2 = (uint8_t)0x02U,                                        /*!< ADC inserted channel sequence 2 */
    ADC_INSERTED_SEQUENCE_3 = (uint8_t)0x03U,                                        /*!< ADC inserted channel sequence 3 */
} hal_adc_inserted_sequence_enum;

/* ADC Common Settings */
/* @PARA: adc */
/* @STRUCT: ADC common settings config struct */
typedef struct {
    uint32_t                         data_alignment;                                 /*!< ADC data alignment */
    uint32_t                         resolution;                                     /*!< ADC data resolution select */
    ControlStatus                    scan_mode;                                      /*!< scan mode */
    ControlStatus                    hardware_oversampling;                          /*!< oversampling enable */
    uint32_t                         oversample_trigger_mode;                        /*!< triggered oversampling */
    hal_adc_oversample_shift_enum    oversampling_shift;                             /*!< oversampling shift */
    hal_adc_oversample_ratio_enum    oversampling_ratio;                             /*!< oversampling ratio */
} hal_adc_init_struct;

/* @STRUCT_MEMBER: data_alignment */
/* @DEFINE: data alignment */
#define ADC_LSB_ALIGNMENT           ((uint32_t)0x00000000U)                          /*!< LSB alignment */
#define ADC_MSB_ALIGNMENT           ADC_CTL1_DAL                                     /*!< MSB alignment */

/* @STRUCT_MEMBER: resolution */
/* @DEFINE: adc resolution */
#define ADC_RESOLUTION_12B          CTL0_DRES(0)                                     /*!< 12-bit ADC resolution */
#define ADC_RESOLUTION_10B          CTL0_DRES(1)                                     /*!< 10-bit ADC resolution */
#define ADC_RESOLUTION_8B           CTL0_DRES(2)                                     /*!< 8-bit ADC resolution */
#define ADC_RESOLUTION_6B           CTL0_DRES(3)                                     /*!< 6-bit ADC resolution */

/* @STRUCT_MEMBER: scan_mode */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: hardware_oversampling */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: oversample_trigger_mode */
/* @DEFINE: triggered oversampling */
#define ADC_OVERSAMPLING_ALL_CONVERT     0U                                          /*!< all oversampled conversions for a channel are done consecutively after a trigger */
#define ADC_OVERSAMPLING_ONE_CONVERT     1U                                          /*!< each oversampled conversion for a channel needs a trigger */

/* @FUNCTION: initialize ADC */
int32_t hal_adc_init(hal_adc_dev_struct *adc_dev, hal_adc_init_struct *adc);

/* Routine sequence Settings */
/* @PARA: p_rrank */
/* @STRUCT: routine rank settings config struct */
typedef struct {
    uint32_t                         channel;                                        /*!< ADC channel select */
    uint32_t                         sampling_time;                                  /*!< ADC sample time */
    hal_adc_routine_sequence_enum    routine_sequence;                               /*!< ADC routine channel sequence */
} hal_adc_routine_rank_config_struct;

/* @STRUCT_MEMBER: channel */
/* @DEFINE: ADC channel select */
#define ADC_CHANNEL_0                    ((uint8_t)0x00U)                            /*!< ADC channel 0 */
#define ADC_CHANNEL_1                    ((uint8_t)0x01U)                            /*!< ADC channel 1 */
#define ADC_CHANNEL_2                    ((uint8_t)0x02U)                            /*!< ADC channel 2 */
#define ADC_CHANNEL_3                    ((uint8_t)0x03U)                            /*!< ADC channel 3 */
#define ADC_CHANNEL_4                    ((uint8_t)0x04U)                            /*!< ADC channel 4 */
#define ADC_CHANNEL_5                    ((uint8_t)0x05U)                            /*!< ADC channel 5 */
#define ADC_CHANNEL_6                    ((uint8_t)0x06U)                            /*!< ADC channel 6 */
#define ADC_CHANNEL_7                    ((uint8_t)0x07U)                            /*!< ADC channel 7 */
#define ADC_CHANNEL_8                    ((uint8_t)0x08U)                            /*!< ADC channel 8 */
#define ADC_CHANNEL_9                    ((uint8_t)0x09U)                            /*!< ADC channel 9 */
#define ADC_CHANNEL_10                   ((uint8_t)0x0AU)                            /*!< ADC channel 10 */
#define ADC_CHANNEL_11                   ((uint8_t)0x0BU)                            /*!< ADC channel 11 */
#define ADC_CHANNEL_12                   ((uint8_t)0x0CU)                            /*!< ADC channel 12 */
#define ADC_CHANNEL_13                   ((uint8_t)0x0DU)                            /*!< ADC channel 13 */
#define ADC_CHANNEL_14                   ((uint8_t)0x0EU)                            /*!< ADC channel 14 */
#define ADC_CHANNEL_15                   ((uint8_t)0x0FU)                            /*!< ADC channel 15 */
#define ADC_CHANNEL_16                   ((uint8_t)0x10U)                            /*!< ADC channel 16 */
#define ADC_CHANNEL_17                   ((uint8_t)0x11U)                            /*!< ADC channel 17 */
#define ADC_CHANNEL_18                   ((uint8_t)0x12U)                            /*!< ADC channel 18 */

/* @STRUCT_MEMBER: sampling_time */
/* @DEFINE: ADC sample time */
#define ADC_SAMPLETIME_1POINT5           SAMPTX_SPT(0)                               /*!< 1.5 sampling cycles */
#define ADC_SAMPLETIME_7POINT5           SAMPTX_SPT(1)                               /*!< 7.5 sampling cycles */
#define ADC_SAMPLETIME_13POINT5          SAMPTX_SPT(2)                               /*!< 13.5 sampling cycles */
#define ADC_SAMPLETIME_28POINT5          SAMPTX_SPT(3)                               /*!< 28.5 sampling cycles */
#define ADC_SAMPLETIME_41POINT5          SAMPTX_SPT(4)                               /*!< 41.5 sampling cycles */
#define ADC_SAMPLETIME_55POINT5          SAMPTX_SPT(5)                               /*!< 55.5 sampling cycles */
#define ADC_SAMPLETIME_71POINT5          SAMPTX_SPT(6)                               /*!< 71.5 sampling cycles */
#define ADC_SAMPLETIME_239POINT5         SAMPTX_SPT(7)                               /*!< 239.5 sampling cycles */

/* @FUNCTION: configure ADC routine rank */
int32_t hal_adc_routine_rank_config(hal_adc_dev_struct *adc_dev, hal_adc_routine_rank_config_struct *p_rrank);

/* @PARA: p_rchannel */
/* @STRUCT: routine sequence settings config struct */
typedef struct {
    ControlStatus              routine_sequence_conversions;                         /*!< routine sequence enable */
    uint32_t                   routine_sequence_length;                              /*!< routine sequence length */
    uint32_t                   routine_sequence_external_trigger_select;             /*!< external trigger select for routine sequence */
    ControlStatus              continuous_mode;                                      /*!< continuous mode */
    ControlStatus              discontinuous_mode;                                   /*!< discontinuous mode */
    uint32_t                   number_of_conversions_in_discontinuous_mode;          /*!< discontinuous mode length */
} hal_adc_routine_config_struct;

/* @STRUCT_MEMBER: routine_sequence_conversions */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: routine_sequence_length */
/* @=NULL */

/* @STRUCT_MEMBER: routine_sequence_external_trigger_select */
/* @DEFINE: external trigger select for routine sequence */
#define ADC_EXTTRIG_ROUTINE_T0_CH0       CTL1_ETSRC(0)                              /*!< TIMER0 CH0 event select */
#define ADC_EXTTRIG_ROUTINE_T0_CH1       CTL1_ETSRC(1)                              /*!< TIMER0 CH1 event select */
#define ADC_EXTTRIG_ROUTINE_T0_CH2       CTL1_ETSRC(2)                              /*!< TIMER0 CH2 event select */
#define ADC_EXTTRIG_ROUTINE_T1_CH1       CTL1_ETSRC(3)                              /*!< TIMER1 CH1 event select */
#define ADC_EXTTRIG_ROUTINE_T2_TRGO      CTL1_ETSRC(4)                              /*!< TIMER2 TRGO event select */
#define ADC_EXTTRIG_ROUTINE_T14_CH0      CTL1_ETSRC(5)                              /*!< TIMER14 CH0 event select */
#define ADC_EXTTRIG_ROUTINE_EXTI_11      CTL1_ETSRC(6)                              /*!< external interrupt line 11 */
#define ADC_EXTTRIG_ROUTINE_NONE         CTL1_ETSRC(7)                              /*!< software trigger */

/* @STRUCT_MEMBER: continuous_mode */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: discontinuous_mode */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: number_of_conversions_in_discontinuous_mode */
/* @=NULL */

/* @FUNCTION: configure adc routine channel */
int32_t hal_adc_routine_channel_config(hal_adc_dev_struct *adc_dev, hal_adc_routine_config_struct *p_rchannel);

/* @PARA: p_irank */
/* ADC inserted channel configuration structure */
/* @STRUCT: inserted rank settings config struct */
typedef struct {
    uint32_t                            channel;                                   /*!< ADC channel select */
    uint32_t                            sampling_time;                             /*!< ADC sample time */
    uint32_t                            data_offset;                               /*!< ADC data offset */
    hal_adc_inserted_sequence_enum      inserted_sequence;                         /*!< ADC inserted channel sequence */
} hal_adc_inserted_rank_config_struct;

/* @STRUCT_MEMBER: channel */
/* @REFER: ADC_CHANNEL_0 */

/* @STRUCT_MEMBER: sampling_time */
/* @DEFINE: ADC sample time */
#define ADC_SAMPLETIME_1POINT5           SAMPTX_SPT(0)                             /*!< 1.5 sampling cycles */
#define ADC_SAMPLETIME_7POINT5           SAMPTX_SPT(1)                             /*!< 7.5 sampling cycles */
#define ADC_SAMPLETIME_13POINT5          SAMPTX_SPT(2)                             /*!< 13.5 sampling cycles */
#define ADC_SAMPLETIME_28POINT5          SAMPTX_SPT(3)                             /*!< 28.5 sampling cycles */
#define ADC_SAMPLETIME_41POINT5          SAMPTX_SPT(4)                             /*!< 41.5 sampling cycles */
#define ADC_SAMPLETIME_55POINT5          SAMPTX_SPT(5)                             /*!< 55.5 sampling cycles */
#define ADC_SAMPLETIME_71POINT5          SAMPTX_SPT(6)                             /*!< 71.5 sampling cycles */
#define ADC_SAMPLETIME_239POINT5         SAMPTX_SPT(7)                             /*!< 239.5 sampling cycles */

/* @STRUCT_MEMBER: data_offset */
/* @=NULL */

/* @FUNCTION: configure ADC inserted rank */
int32_t hal_adc_inserted_rank_config(hal_adc_dev_struct *adc_dev, hal_adc_inserted_rank_config_struct *p_irank);

/* @PARA: p_ichannel */
/* ADC inserted channel initialization structure */
/* @STRUCT: inserted sequence settings config struct */
typedef struct {
    ControlStatus           inserted_sequence_conversions;                         /*!< inserted sequence enable */
    uint32_t                inserted_sequence_length;                              /*!< ADC inserted channel sequence,0~3 */
    uint32_t                inserted_sequence_external_trigger_select;             /*!< ADC inserted channel sequence external trigger select */
    ControlStatus           auto_convert;                                          /*!< ADC inserted channel convert automatically */
    ControlStatus           discontinuous_mode;                                    /*!< ADC inserted channel discontinuous mode */
} hal_adc_inserted_config_struct;

/* @STRUCT_MEMBER: inserted_sequence_conversions */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: inserted_sequence_length */
/* @=NULL */

/* @STRUCT_MEMBER: inserted_sequence_external_trigger_select */
/* @DEFINE: external trigger select for inserted sequence */
#define ADC_EXTTRIG_INSERTED_T0_TRGO        CTL1_ETSIC(0)                          /*!< TIMER0 TRGO event select */
#define ADC_EXTTRIG_INSERTED_T0_CH3         CTL1_ETSIC(1)                          /*!< TIMER0 CH3 event select */
#define ADC_EXTTRIG_INSERTED_T1_TRGO        CTL1_ETSIC(2)                          /*!< TIMER1 TRGO event select */
#define ADC_EXTTRIG_INSERTED_T1_CH0         CTL1_ETSIC(3)                          /*!< TIMER1 CH0 event select */
#define ADC_EXTTRIG_INSERTED_T2_CH3         CTL1_ETSIC(4)                          /*!< TIMER2 CH3 event select */
#define ADC_EXTTRIG_INSERTED_T14_TRGO       CTL1_ETSIC(5)                          /*!< TIMER14 TRGO event select */
#define ADC_EXTTRIG_INSERTED_EXTI_15        CTL1_ETSIC(6)                          /*!< external interrupt line 15 */
#define ADC_EXTTRIG_INSERTED_NONE           CTL1_ETSIC(7)                          /*!< software trigger */

/* @STRUCT_MEMBER: auto_convert */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: discontinuous_mode */
/* @SP: ENABLE/DISABLE */

/* @FUNCTION: configure ADC inserted channel */
int32_t hal_adc_inserted_channel_config(hal_adc_dev_struct *adc_dev, hal_adc_inserted_config_struct *p_ichannel);

/* Analog Watchdog Settings */
/* @PARA: p_watchdog */
/* @STRUCT: analog watchdog settings config struct */
typedef struct {
    ControlStatus               routine_sequence_analog_watchdog;                  /*!< routine sequence analog watchdog enable */
    ControlStatus               inserted_sequence_analog_watchdog;                 /*!< inserted sequence analog watchdog enable */
    uint32_t                    analog_watchdog_mode;                              /*!< analog watchdog mode */
    uint32_t                    analog_watchdog_channel_select;                    /*!< analog watchdog channel select */
    uint16_t                    analog_watchdog_high_threshold;                    /*!< ADC analog watchdog high threshold */
    uint16_t                    analog_watchdog_low_threshold;                     /*!< ADC analog watchdog low threshold */
} hal_adc_watchdog_config_struct;

/* @STRUCT_MEMBER: routine_sequence_analog_watchdog */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: inserted_sequence_analog_watchdog */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: analog_watchdog_mode */
/* @DEFINE: analog watchdog mode */
#define ADC_WATCHDOG_MODE_ALL_CHANNELS                    0                       /*!< Analog watchdog is effective on all channels */
#define ADC_WATCHDOG_MODE_SINGLE_CHANNEL                  1                       /*!< Analog watchdog is effective on a single channel */

/* @STRUCT_MEMBER: analog_watchdog_channel_select */
/* @DEFINE: ADC analog watchdog channel select */
#define ADC_CHANNEL_0                    ((uint8_t)0x00U)                            /*!< ADC channel 0 */
#define ADC_CHANNEL_1                    ((uint8_t)0x01U)                            /*!< ADC channel 1 */
#define ADC_CHANNEL_2                    ((uint8_t)0x02U)                            /*!< ADC channel 2 */
#define ADC_CHANNEL_3                    ((uint8_t)0x03U)                            /*!< ADC channel 3 */
#define ADC_CHANNEL_4                    ((uint8_t)0x04U)                            /*!< ADC channel 4 */
#define ADC_CHANNEL_5                    ((uint8_t)0x05U)                            /*!< ADC channel 5 */
#define ADC_CHANNEL_6                    ((uint8_t)0x06U)                            /*!< ADC channel 6 */
#define ADC_CHANNEL_7                    ((uint8_t)0x07U)                            /*!< ADC channel 7 */
#define ADC_CHANNEL_8                    ((uint8_t)0x08U)                            /*!< ADC channel 8 */
#define ADC_CHANNEL_9                    ((uint8_t)0x09U)                            /*!< ADC channel 9 */
#define ADC_CHANNEL_10                   ((uint8_t)0x0AU)                            /*!< ADC channel 10 */
#define ADC_CHANNEL_11                   ((uint8_t)0x0BU)                            /*!< ADC channel 11 */
#define ADC_CHANNEL_12                   ((uint8_t)0x0CU)                            /*!< ADC channel 12 */
#define ADC_CHANNEL_13                   ((uint8_t)0x0DU)                            /*!< ADC channel 13 */
#define ADC_CHANNEL_14                   ((uint8_t)0x0EU)                            /*!< ADC channel 14 */
#define ADC_CHANNEL_15                   ((uint8_t)0x0FU)                            /*!< ADC channel 15 */
#define ADC_CHANNEL_16                   ((uint8_t)0x10U)                            /*!< ADC channel 16 */
#define ADC_CHANNEL_17                   ((uint8_t)0x11U)                            /*!< ADC channel 17 */
#define ADC_CHANNEL_18                   ((uint8_t)0x12U)                            /*!< ADC channel 18 */

/* @STRUCT_MEMBER: analog_watchdog_high_threshold */
/* @=NULL */

/* @STRUCT_MEMBER: analog_watchdog_low_threshold */
/* @=NULL */

/* @FUNCTION: configure watchdog */
int32_t hal_adc_watchdog_config(hal_adc_dev_struct *adc_dev, hal_adc_watchdog_config_struct *p_watchdog);
/* @END */


/* ADC internal channel */
#define ADC_CHANNEL_TEMPSENSOR                  ADC_CHANNEL_16                    /*!< ADC temperature sensor channel */
#define ADC_CHANNEL_VREFINT                     ADC_CHANNEL_17                    /*!< ADC VREFINT channel */
#define ADC_CHANNEL_BATTERY                     ADC_CHANNEL_18                    /*!< ADC backup battery voltage channel */

/* get ADC init value */
#define __HAL_ADC_GET_SCAN_MODE                 (uint32_t)((ADC_CTL0) & (ADC_CTL0_SM))               /*!< get ADC scan mode init value */
#define __HAL_ADC_GET_CONTINUOUS_MODE           (uint32_t)((ADC_CTL1) & (ADC_CTL1_CTN))              /*!< get ADC continuous mode init value */
#define __HAL_ADC_GET_TEMPVREF_ENABLE           (uint32_t)((ADC_CTL1) & (ADC_CTL1_TSVREN))           /*!< get ADC internal channel enable init value */
#define __HAL_ADC_GET_BATTERY_ENABLE            (uint32_t)((ADC_CTL1) & (ADC_CTL1_VBETEN))           /*!< get ADC internal channel enable init value */
#define __HAL_ADC_GET_OVERSAMPLE_ENABLE         (uint32_t)((ADC_OVSAMPCTL) & (ADC_OVSAMPCTL_OVSEN))  /*!< get ADC oversampling enable init value */
#define __HAL_ADC_GET_DMA_MODE                  (uint32_t)((ADC_CTL1) & (ADC_CTL1_DMA))              /*!< get ADC DMA mode init value */

#define __HAL_ADC_GET_ROUTINECH_LENGTH          (uint32_t)((ADC_RSQ0) & (ADC_RSQ0_RL))               /*!< get ADC routine sequence length value */
#define __HAL_ADC_GET_ROUTINECH_EXTTRIGGER      (uint32_t)((ADC_CTL1) & (ADC_CTL1_ETSRC))            /*!< get ADC routine external trigger select */

#define __HAL_ADC_GET_INSERTEDCH_LENGTH         (uint32_t)((ADC_RSQ0) & (ADC_RSQ0_RL))               /*!< get ADC inserted sequence length value */
#define __HAL_ADC_GET_INSERTEDCH_EXTTRIGGER     (uint32_t)((ADC_CTL1) & (ADC_CTL1_ETSIC))            /*!< get ADC inserted external trigger select */
#define __HAL_ADC_GET_INSERTEDCH_AUTOCONV       (uint32_t)((ADC_CTL0) & (ADC_CTL0_ICA))              /*!< get ADC inserted sequence automatic conversion */

/* function declarations */
/* initialize the ADC structure with the default values */
void hal_adc_struct_init(hal_adc_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize ADC */
int32_t hal_adc_deinit(hal_adc_dev_struct *adc_dev);
/* ADC calibration */
int32_t hal_adc_calibration_start(hal_adc_dev_struct *adc_dev);

/* ADC routine channel */
/* enable ADC and start the conversion of routine sequence */
int32_t hal_adc_start(hal_adc_dev_struct *adc_dev);
/* stop the conversion of routine sequence and disable ADC */
int32_t hal_adc_stop(hal_adc_dev_struct *adc_dev);
/* polling for ADC routine sequence conversion */
int32_t hal_adc_routine_conversion_poll(hal_adc_dev_struct *adc_dev, uint32_t timeout_ms);
/* enable ADC routine sequence software trigger */
void hal_adc_routine_software_trigger_enable(hal_adc_dev_struct *adc_dev);
/* start ADC EOC interrupt */
int32_t hal_adc_start_interrupt(hal_adc_dev_struct *adc_dev, hal_adc_irq_struct *p_irq);
/* stop ADC EOC interrupt */
int32_t hal_adc_stop_interrupt(hal_adc_dev_struct *adc_dev);

/* ADC DMA */
/* enable ADC and start the conversion of routine sequence with DMA */
int32_t hal_adc_start_dma(hal_adc_dev_struct *adc_dev, uint32_t *pdata, uint32_t length, hal_adc_dma_handle_cb_struct *dmacb);
/* stop the conversion of routine sequence, disable ADC DMA mode and disable ADC */
int32_t hal_adc_stop_dma(hal_adc_dev_struct *adc_dev);

/* ADC inserted channel */
/* enable ADC and start the conversion of inserted sequence */
int32_t hal_adc_inserted_start(hal_adc_dev_struct *adc_dev);
/* stop the conversion of inserted sequence and disable ADC */
int32_t hal_adc_inserted_stop(hal_adc_dev_struct *adc_dev);
/* polling for ADC inserted sequence conversion */
int32_t hal_adc_inserted_conversion_poll(hal_adc_dev_struct *adc_dev, uint32_t timeout_ms);
/* enable ADC inserted sequence software trigger */
void hal_adc_inserted_software_trigger_enable(hal_adc_dev_struct *adc_dev);
/* start ADC EOIC interrupt */
int32_t hal_adc_inserted_start_interrupt(hal_adc_dev_struct *adc_dev, hal_adc_irq_struct *p_irq);
/* stop ADC EOIC interrupt */
int32_t hal_adc_inserted_stop_interrupt(hal_adc_dev_struct *adc_dev);

/* ADC watchdog */
/* enable ADC watchdog interrupt */
int32_t hal_adc_watchdog_interrupt_enable(hal_adc_dev_struct *adc_dev, hal_adc_irq_struct *p_irq);
/* disable ADC watchdog interrupt */
int32_t hal_adc_watchdog_interrupt_disable(hal_adc_dev_struct *adc_dev);
/* polling for ADC watchdog event conversion */
int32_t hal_adc_watchdog_event_poll(hal_adc_dev_struct *adc_dev, uint32_t timeout_ms);

/* interrupt handle */
/* ADC interrupt handler content function, which is merely used in ADC_CMP_IRQHandler */
void hal_adc_irq(hal_adc_dev_struct *adc_dev);
/* set user-defined interrupt callback function,
which will be registered and called when corresponding interrupt be triggered */
void hal_adc_irq_handle_set(hal_adc_dev_struct *adc_dev, hal_adc_irq_struct *p_irq);
/* reset all user-defined interrupt callback function,
which will be registered and called when corresponding interrupt be triggered */
void hal_adc_irq_handle_all_reset(hal_adc_dev_struct *adc_dev);

/* get ADC value and state */
/* get routine sequence conversion result */
uint16_t hal_adc_routine_value_get(hal_adc_dev_struct *adc_dev);
/* get inserted sequence conversion result */
uint16_t hal_adc_inserted_value_get(hal_adc_dev_struct *adc_dev, uint8_t inschannel_sequence);
/* get ADC error */
uint32_t hal_adc_error_get(hal_adc_dev_struct *adc_dev);
/* get ADC state */
uint32_t hal_adc_state_get(hal_adc_dev_struct *adc_dev);

/* function declarations */
/* reset ADC */
void hals_adc_deinit(void);
/* enable ADC interface */
void hals_adc_enable(void);
/* disable ADC interface */
void hals_adc_disable(void);

/* ADC calibration and reset calibration */
void hals_adc_calibration_enable(void);
/* enable DMA request */
void hals_adc_dma_mode_enable(void);
/* disable DMA request */
void hals_adc_dma_mode_disable(void);

/* enable or disable ADC special function */
void hals_adc_special_function_config(uint32_t function, ControlStatus newvalue);

/* configure ADC data alignment */
void hals_adc_data_alignment_config(uint32_t data_alignment);
/* configure the length of routine channel sequence or inserted channel sequence */
void hals_adc_channel_length_config(uint8_t channel_sequence, uint32_t length);
/* configure ADC routine channel */
void hals_adc_routine_channel_config(uint8_t rank, uint8_t channel, uint32_t sample_time);
/* configure ADC inserted channel */
void hals_adc_inserted_channel_config(uint8_t rank, uint8_t channel, uint32_t sample_time);
/* configure ADC inserted channel offset */
void hals_adc_inserted_channel_offset_config(uint8_t inserted_channel, uint16_t offset);
/* enable ADC external trigger */
void hals_adc_external_trigger_config(uint8_t channel_sequence, ControlStatus newvalue);
/* configure ADC external trigger source */
void hals_adc_external_trigger_source_config(uint8_t channel_sequence, uint32_t external_trigger_source);
/* enable ADC software trigger */
void hals_adc_software_trigger_enable(uint8_t channel_sequence);

/* read ADC inserted sequence data register */
uint16_t hals_adc_inserted_data_read(uint8_t inserted_channel);

/* configure ADC resolution */
void hals_adc_resolution_config(uint32_t resolution);
/* configure ADC oversample mode */
void hals_adc_oversample_mode_config(uint8_t mode, uint16_t shift, uint8_t ratio);
/* enable ADC oversample mode */
void hals_adc_oversample_mode_enable(void);
/* disable ADC oversample mode */
void hals_adc_oversample_mode_disable(void);

/* get the ADC flag bits */
FlagStatus hals_adc_flag_get(uint32_t flag);
/* clear the ADC flag bits */
void hals_adc_flag_clear(uint32_t flag);
/* enable ADC interrupt */
void hals_adc_interrupt_enable(uint32_t interrupt);
/* disable ADC interrupt */
void hals_adc_interrupt_disable(uint32_t interrupt);
/* get the ADC interrupt bits */
FlagStatus hals_adc_interrupt_flag_get(uint32_t flag);
/* clear the ADC interrupt flag */
void hals_adc_interrupt_flag_clear(uint32_t flag);

#endif /* GD32F3X0_HAL_ADC_H */
