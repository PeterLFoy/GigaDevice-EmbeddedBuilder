/*!
    \file    gd32f3x0_hal_dac.h
    \brief   definitions for the DAC

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


#ifdef GD32F350
#ifndef GD32F3X0_HAL_DAC_H
#define GD32F3X0_HAL_DAC_H
#include "gd32f3x0_hal.h"


/* DAC definitions */
#define DAC                         DAC_BASE                    /*!< DAC base address */

/* registers definitions */
#define DAC_CTL                     REG32(DAC + (0x00000000U))  /*!< DAC control register */
#define DAC_SWT                     REG32(DAC + (0x00000004U))  /*!< DAC software trigger register */
#define DAC_R12DH                   REG32(DAC + (0x00000008U))  /*!< DAC 12-bit right-aligned data holding register */
#define DAC_L12DH                   REG32(DAC + (0x0000000CU))  /*!< DAC 12-bit left-aligned data holding register */
#define DAC_R8DH                    REG32(DAC + (0x00000010U))  /*!< DAC 8-bit right-aligned data holding register */
#define DAC_DO                      REG32(DAC + (0x0000002CU))  /*!< DAC output data register */
#define DAC_STAT                    REG32(DAC + (0x00000034U))  /*!< DAC status register */

#define CTL_DWM(regval)             (BITS(6,7) & ((uint32_t)(regval) << 6))
#define DWBW(regval)                (BITS(8,11) & ((uint32_t)(regval) << 8))
#define CTL_DTSEL(regval)           (BITS(3,5) & ((uint32_t)(regval) << 3))

/* DAC state enum */
typedef enum {
    DAC_STATE_NONE = 0,                                         /*!< NONE(default value) */
    DAC_STATE_RESET,                                            /*!< DAC is not initialized or disabled */
    DAC_STATE_BUSY,                                             /*!< DAC is busy */
    DAC_STATE_TIMEOUT,                                          /*!< DAC timeout occurs */
    DAC_STATE_ERROR,                                            /*!< DAC is error */
    DAC_STATE_READY,                                            /*!< DAC is ready */
} hal_dac_state_enum;

/* DAC structure type enum */
typedef enum {
    HAL_DAC_INIT_STRUCT,                                        /*!< DAC initialization structure */
    HAL_DAC_IRQ_STRUCT,                                         /*!< DAC device interrupt callback function pointer structure */
    HAL_DAC_DMA_HANDLE_CB_STRUCT,                               /*!< DAC DMA callback function pointer structure */
    HAL_DAC_DEV_STRUCT,                                         /*!< DAC device information structrue */
} hal_dac_struct_type_enum;

/* DAC device interrupt callback function pointer structure */
typedef struct {
    __IO hal_irq_handle_cb                dac_underflow_handle; /*!< DAC underflow handler function */
} hal_dac_irq_struct;

/* DAC DMA callback function pointer structure */
typedef void (*hal_dac_dma_handle_cb)(void *ptr);
typedef struct {
    __IO hal_dac_dma_handle_cb            full_transcom_handle; /*!< DAC DMA transfer complete interrupt handler function */
    __IO  hal_dac_dma_handle_cb           half_transcom_handle; /*!< DAC DMA transfer complete interrupt handler function */
    __IO hal_dac_dma_handle_cb            error_handle;         /*!< DAC DMA underflow error handler function */
} hal_dac_dma_handle_cb_struct;

/* DAC error type enum */
typedef enum {
    DAC_ERROR_NONE             = (uint32_t)0x00U,               /*!< no error */
    DAC_ERROR_SYSTEM           = (uint32_t)0x01U,               /*!< DAC internal error: if problem of clocking, enable/disable, wrong state */
    DAC_ERROR_DMA_UNDERFLOW    = (uint32_t)0x02U,               /*!< DMA transfer error */
    DAC_ERROR_CONFIG           = (uint32_t)0x04U,               /*!< DAC DMA underflow error */
} hal_dac_error_enum;

/* DAC device information structrue */
typedef struct {
    hal_dac_irq_struct               dac_irq;                   /*!< DAC device interrupt callback function pointer structure */
    hal_dma_dev_struct               *p_dma_dac;                /*!< DMA device information structrue */
    hal_dac_dma_handle_cb_struct     dac_dma;                   /*!< DMA callback function pointer structure */
    hal_dac_error_enum               error_state;               /*!< DAC error state */
    hal_dac_state_enum               state;                     /*!< DAC state */
    hal_mutex_enum                   mutex;
    void                             *priv;                     /* priv data */
} hal_dac_dev_struct;

/* @PARA: dac */
/* @STRUCT: DAC basic config struct */
typedef struct {
    uint32_t output_waveform_selection;                         /*!< output waveform selection */
    uint32_t noise_wave_bit_width;                              /*!< noise wave bit width */
    uint32_t output_buffer_enable;                              /*!< output buffer enable */
    uint32_t trigger_enable;                                    /*!< trigger enable */
    uint32_t trigger_selection;                                 /*!< trigger selection */
    uint32_t aligned_mode;                                      /*!< aligned mode */
    uint32_t output_value;                                      /*!< output value */
} hal_dac_init_struct;

/* @STRUCT_MEMBER: output_waveform_selection */
/* @DEFINE: output waveform selection */
#define DAC_WAVE_DISABLE            CTL_DWM(0)                  /*!< wave disable */
#define DAC_WAVE_MODE_LFSR          CTL_DWM(1)                  /*!< LFSR noise mode */
#define DAC_WAVE_MODE_TRIANGLE      CTL_DWM(2)                  /*!< triangle noise mode */

/* @STRUCT_MEMBER: noise_wave_bit_width */
/* @DEFINE: noise wave bit width */
#define DAC_WAVE_BIT_WIDTH_1        DWBW(0)                     /*!< bit width of the wave signal is 1 */
#define DAC_WAVE_BIT_WIDTH_2        DWBW(1)                     /*!< bit width of the wave signal is 2 */
#define DAC_WAVE_BIT_WIDTH_3        DWBW(2)                     /*!< bit width of the wave signal is 3 */
#define DAC_WAVE_BIT_WIDTH_4        DWBW(3)                     /*!< bit width of the wave signal is 4 */
#define DAC_WAVE_BIT_WIDTH_5        DWBW(4)                     /*!< bit width of the wave signal is 5 */
#define DAC_WAVE_BIT_WIDTH_6        DWBW(5)                     /*!< bit width of the wave signal is 6 */
#define DAC_WAVE_BIT_WIDTH_7        DWBW(6)                     /*!< bit width of the wave signal is 7 */
#define DAC_WAVE_BIT_WIDTH_8        DWBW(7)                     /*!< bit width of the wave signal is 8 */
#define DAC_WAVE_BIT_WIDTH_9        DWBW(8)                     /*!< bit width of the wave signal is 9 */
#define DAC_WAVE_BIT_WIDTH_10       DWBW(9)                     /*!< bit width of the wave signal is 10 */
#define DAC_WAVE_BIT_WIDTH_11       DWBW(10)                    /*!< bit width of the wave signal is 11 */
#define DAC_WAVE_BIT_WIDTH_12       DWBW(11)                    /*!< bit width of the wave signal is 12 */

/* @STRUCT_MEMBER: output_buffer_enable */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: trigger_enable */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: trigger_selection */
/* @DEFINE: trigger selection */
#define DAC_TRIGGER_T5_TRGO         CTL_DTSEL(0)                /*!< TIMER5 TRGO */
#define DAC_TRIGGER_T2_TRGO         CTL_DTSEL(1)                /*!< TIMER2 TRGO */
#define DAC_TRIGGER_T14_TRGO        CTL_DTSEL(3)                /*!< TIMER14 TRGO */
#define DAC_TRIGGER_T1_TRGO         CTL_DTSEL(4)                /*!< TIMER1 TRGO */
#define DAC_TRIGGER_EXTI_9          CTL_DTSEL(6)                /*!< EXTI interrupt line9 event */
#define DAC_TRIGGER_SOFTWARE        CTL_DTSEL(7)                /*!< software trigger */

/* @STRUCT_MEMBER: aligned_mode */
/* @DEFINE: aligned mode */
#define DAC_R12_ALIGNED_MODE        0                           /*!< 12-bit right-aligned */
#define DAC_L12_ALIGNED_MODE        1                           /*!< 12-bit left-aligned */
#define DAC_R8_ALIGNED_MODE         2                           /*!< 8-bit right-aligned */

/* @STOP_ANALYSIS: */
/* bits definitions */
/* DAC_CTL */
#define DAC_CTL_DEN                 BIT(0)                      /*!< DAC enable/disable bit */
#define DAC_CTL_DBOFF               BIT(1)                      /*!< DAC output buffer turn on/turn off bit */
#define DAC_CTL_DTEN                BIT(2)                      /*!< DAC trigger enable/disable bit */
#define DAC_CTL_DTSEL               BITS(3,5)                   /*!< DAC trigger source selection enable/disable bits */
#define DAC_CTL_DWM                 BITS(6,7)                   /*!< DAC noise wave mode */
#define DAC_CTL_DWBW                BITS(8,11)                  /*!< DAC noise wave bit width */
#define DAC_CTL_DDMAEN              BIT(12)                     /*!< DAC DMA enable/disable bit */
#define DAC_CTL_DDUDRIE             BIT(13)                     /*!< DAC DMA underrun interrupt enable/disable bit */

/* DAC_SWT */
#define DAC_SWT_SWTR                BIT(0)                      /*!< DAC software trigger bit,cleared by hardware */
/* DAC_R12DH */
#define DAC_R12DH_DAC_DH            BITS(0,11)                  /*!< DAC 12-bit right-aligned data bits */
/* DAC_L12DH */
#define DAC_L12DH_DAC_DH            BITS(4,15)                  /*!< DAC 12-bit left-aligned data bits */
/* DAC_R8DH */
#define DAC_R8DH_DAC_DH             BITS(0,7)                   /*!< DAC 8-bit right-aligned data bits */
/* DAC_DO */
#define DAC_DO_DAC_DO               BITS(0,11)                  /*!< DAC 12-bit output data bits */
/* DAC_STAT */
#define DAC_STAT_DDUDR              BIT(13)                     /*!< DAC DMA underrun flag */
/* @STOP_ANALYSIS_END: */

/* @STRUCT_MEMBER: output_value */
/* @=NULL */

/* DAC settings*/
/* @FUNCTION: initialize DAC */
int32_t hal_dac_init(hal_dac_dev_struct *dac_dev, hal_dac_init_struct *dac);

/* @END */

/* deinitialize DAC device structure and init structure */
int32_t hal_dac_deinit(hal_dac_dev_struct *dac_dev);
/* initialize DAC structure */
void hal_dac_struct_init(hal_dac_struct_type_enum hal_struct_type, void *p_struct);
/* start DAC module function */
int32_t hal_dac_start(hal_dac_dev_struct *dac_dev);
/* stop DAC module function */
int32_t hal_dac_stop(hal_dac_dev_struct *dac_dev);
/* start DAC under error interrupt */
int32_t hal_dac_start_interrupt(hal_dac_dev_struct *dac_dev, hal_dac_irq_struct *p_irq);
/* stop DAC under error interrupt */
int32_t hal_dac_stop_interrupt(hal_dac_dev_struct *dac_dev);
/* start DAC with DMA */
int32_t hal_dac_start_dma(hal_dac_dev_struct *dac_dev, uint32_t* pdata,uint32_t length,
                          uint32_t aligned_mode, hal_dac_dma_handle_cb_struct *dmacb);
/* stop DAC with DMA */
int32_t hal_dac_stop_dma(hal_dac_dev_struct *dac_dev);
/* DAC interrupt handler content function,which is merely used in dac_handler */
int32_t hal_dac_irq(hal_dac_dev_struct *dac);
/* set user-defined interrupt callback function, 
    which will be registered and called when corresponding interrupt be triggered */
int32_t hal_dac_irq_handle_set(hal_dac_dev_struct *dac_dev, hal_dac_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
    which will be registered and called when corresponding interrupt be triggered */
int32_t hal_dac_irq_handle_all_reset(hal_dac_dev_struct *dac_dev);
/* configure DAC wave mode */
void hals_dac_wave_mode_config(uint32_t wave_mode);
/* configure DAC wave bit width */
void hals_dac_wave_bit_width_config(uint32_t bit_width);
/* enable or disable DAC output buffer */
void hals_dac_output_buffer_config(uint32_t output_buffer);
/* enable or disable DAC trigger */
void hals_dac_trigger_config(uint32_t trigger_enable);
/* configure DAC trigger source */
void hals_dac_trigger_source_config(uint32_t triggersource);
/* enable DAC software trigger */
void hals_dac_software_trigger_enable(void);
/* disable DAC software trigger */
void hals_dac_software_trigger_disable(void);
/* set DAC data holding register value */
void hals_dac_output_value_set(uint32_t dac_align, uint16_t data);
/* enable DAC */
void hals_dac_enable(void);
/* disable DAC */
void hals_dac_disable(void);
/* enable DAC DMA */
void hals_dac_dma_enable(void);
/* disable DAC DMA */
void hals_dac_dma_disable(void);
/* enable DAC interrupt(DAC DMA underrun interrupt) */
void hals_dac_interrupt_enable(void);
/* disable DAC interrupt(DAC DMA underrun interrupt) */
void hals_dac_interrupt_disable(void);
/* get the specified DAC interrupt flag(DAC DMA underrun interrupt flag) */
FlagStatus hals_dac_interrupt_flag_get(void);
/* clear the specified DAC interrupt flag(DAC DMA underrun interrupt flag) */
void hals_dac_interrupt_flag_clear(void);
#endif /* GD32F3X0_HAL_DAC_H */
#endif /* GD32F350 */
