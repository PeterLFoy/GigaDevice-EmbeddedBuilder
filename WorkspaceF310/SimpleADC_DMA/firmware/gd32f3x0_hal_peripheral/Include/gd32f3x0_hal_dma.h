/*!
    \file    gd32f3x0_hal_dma.h
    \brief   definitions for the DMA

    \version 2023-06-01, V1.0.0, firmware for GD32F3x0
*/

/*
    Copyright (c) 2023 GigaDevice Semiconductor Inc.

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

#ifndef GD32F3X0_HAL_DMA_H
#define GD32F3X0_HAL_DMA_H

#include "gd32f3x0_hal.h"

/* DMA definitions */
#define DMA                               DMA_BASE                    /*!< DMA base address */

/* registers definitions */
#define DMA_INTF                          REG32(DMA + 0x00000000U)    /*!< DMA interrupt flag register */
#define DMA_INTC                          REG32(DMA + 0x00000004U)    /*!< DMA interrupt flag clear register */
#define DMA_CH0CTL                        REG32(DMA + 0x00000008U)    /*!< DMA channel 0 control register */
#define DMA_CH0CNT                        REG32(DMA + 0x0000000CU)    /*!< DMA channel 0 counter register */
#define DMA_CH0PADDR                      REG32(DMA + 0x00000010U)    /*!< DMA channel 0 peripheral base address register */
#define DMA_CH0MADDR                      REG32(DMA + 0x00000014U)    /*!< DMA channel 0 memory base address register */
#define DMA_CH1CTL                        REG32(DMA + 0x0000001CU)    /*!< DMA channel 1 control register */
#define DMA_CH1CNT                        REG32(DMA + 0x00000020U)    /*!< DMA channel 1 counter register */
#define DMA_CH1PADDR                      REG32(DMA + 0x00000024U)    /*!< DMA channel 1 peripheral base address register */
#define DMA_CH1MADDR                      REG32(DMA + 0x00000028U)    /*!< DMA channel 1 memory base address register */
#define DMA_CH2CTL                        REG32(DMA + 0x00000030U)    /*!< DMA channel 2 control register */
#define DMA_CH2CNT                        REG32(DMA + 0x00000034U)    /*!< DMA channel 2 counter register */
#define DMA_CH2PADDR                      REG32(DMA + 0x00000038U)    /*!< DMA channel 2 peripheral base address register */
#define DMA_CH2MADDR                      REG32(DMA + 0x0000003CU)    /*!< DMA channel 2 memory base address register */
#define DMA_CH3CTL                        REG32(DMA + 0x00000044U)    /*!< DMA channel 3 control register */
#define DMA_CH3CNT                        REG32(DMA + 0x00000048U)    /*!< DMA channel 3 counter register */
#define DMA_CH3PADDR                      REG32(DMA + 0x0000004CU)    /*!< DMA channel 3 peripheral base address register */
#define DMA_CH3MADDR                      REG32(DMA + 0x00000050U)    /*!< DMA channel 3 memory base address register */
#define DMA_CH4CTL                        REG32(DMA + 0x00000058U)    /*!< DMA channel 4 control register */
#define DMA_CH4CNT                        REG32(DMA + 0x0000005CU)    /*!< DMA channel 4 counter register */
#define DMA_CH4PADDR                      REG32(DMA + 0x00000060U)    /*!< DMA channel 4 peripheral base address register */
#define DMA_CH4MADDR                      REG32(DMA + 0x00000064U)    /*!< DMA channel 4 memory base address register */
#define DMA_CH5CTL                        REG32(DMA + 0x0000006CU)    /*!< DMA channel 5 control register */
#define DMA_CH5CNT                        REG32(DMA + 0x00000070U)    /*!< DMA channel 5 counter register */
#define DMA_CH5PADDR                      REG32(DMA + 0x00000074U)    /*!< DMA channel 5 peripheral base address register */
#define DMA_CH5MADDR                      REG32(DMA + 0x00000078U)    /*!< DMA channel 5 memory base address register */
#define DMA_CH6CTL                        REG32(DMA + 0x00000080U)    /*!< DMA channel 6 control register */
#define DMA_CH6CNT                        REG32(DMA + 0x00000084U)    /*!< DMA channel 6 counter register */
#define DMA_CH6PADDR                      REG32(DMA + 0x00000088U)    /*!< DMA channel 6 peripheral base address register */
#define DMA_CH6MADDR                      REG32(DMA + 0x0000008CU)    /*!< DMA channel 6 memory base address register */

/* bits definitions */
/* DMA_INTF */
#define DMA_INTF_GIF                      BIT(0)                /*!< global interrupt flag of channel */
#define DMA_INTF_FTFIF                    BIT(1)                /*!< full transfer finish flag of channel */
#define DMA_INTF_HTFIF                    BIT(2)                /*!< half transfer finish flag of channel */
#define DMA_INTF_ERRIF                    BIT(3)                /*!< error flag of channel */

/* DMA_INTC */
#define DMA_INTC_GIFC                     BIT(0)                /*!< clear global interrupt flag of channel */
#define DMA_INTC_FTFIFC                   BIT(1)                /*!< clear transfer finish flag of channel */
#define DMA_INTC_HTFIFC                   BIT(2)                /*!< clear half transfer finish flag of channel */
#define DMA_INTC_ERRIFC                   BIT(3)                /*!< clear error flag of channel */

/* DMA_CHxCTL,x=0..6 */
#define DMA_CHXCTL_CHEN                   BIT(0)                /*!< channel x enable */
#define DMA_CHXCTL_FTFIE                  BIT(1)                /*!< enable bit for channel x transfer complete interrupt */
#define DMA_CHXCTL_HTFIE                  BIT(2)                /*!< enable bit for channel x transfer half complete interrupt */
#define DMA_CHXCTL_ERRIE                  BIT(3)                /*!< enable bit for channel x error interrupt */
#define DMA_CHXCTL_DIR                    BIT(4)                /*!< direction of the data transfer on the channel */
#define DMA_CHXCTL_CMEN                   BIT(5)                /*!< circulation mode */
#define DMA_CHXCTL_PNAGA                  BIT(6)                /*!< next address generation algorithm of peripheral */
#define DMA_CHXCTL_MNAGA                  BIT(7)                /*!< next address generation algorithm of memory */
#define DMA_CHXCTL_PWIDTH                 BITS(8,9)             /*!< transfer data size of peripheral */
#define DMA_CHXCTL_MWIDTH                 BITS(10,11)           /*!< transfer data size of memory */
#define DMA_CHXCTL_PRIO                   BITS(12,13)           /*!< priority level of channelx */
#define DMA_CHXCTL_M2M                    BIT(14)               /*!< memory to memory mode */

/* DMA_CHxCNT,x=0..6 */
#define DMA_CHXCNT_CNT                    BITS(0,15)            /*!< transfer counter */

/* DMA_CHxPADDR,x=0..6 */
#define DMA_CHXPADDR_PADDR                BITS(0,31)            /*!< peripheral base address */

/* DMA_CHxMADDR,x=0..6 */
#define DMA_CHXMADDR_MADDR                BITS(0,31)            /*!< memory base address */

/* DMA reset value */
#define DMA_CHCTL_RESET_VALUE             ((uint32_t)0x00000000U)                         /*!< the reset value of DMA channel CHXCTL register */
#define DMA_CHCNT_RESET_VALUE             ((uint32_t)0x00000000U)                         /*!< the reset value of DMA channel CHXCNT register */
#define DMA_CHPADDR_RESET_VALUE           ((uint32_t)0x00000000U)                         /*!< the reset value of DMA channel CHXPADDR register */
#define DMA_CHMADDR_RESET_VALUE           ((uint32_t)0x00000000U)                         /*!< the reset value of DMA channel CHXMADDR register */
#define DMA_CHINTF_RESET_VALUE            (DMA_INTF_GIF | DMA_INTF_FTFIF | \
                                           DMA_INTF_HTFIF | DMA_INTF_ERRIF)
#define DMA_FLAG_ADD(flag,shift)          ((flag) << ((uint32_t)(shift) * 4U))            /*!< DMA channel flag shift */

/* DMA_CHCTL base address */
#define DMA_CHXCTL_BASE                   (DMA + (uint32_t)0x00000008U)                   /*!< the base address of DMA channel CHXCTL register */
#define DMA_CHXCNT_BASE                   (DMA + (uint32_t)0x0000000CU)                   /*!< the base address of DMA channel CHXCNT register */
#define DMA_CHXPADDR_BASE                 (DMA + (uint32_t)0x00000010U)                   /*!< the base address of DMA channel CHXPADDR register */
#define DMA_CHXMADDR_BASE                 (DMA + (uint32_t)0x00000014U)                   /*!< the base address of DMA channel CHXMADDR register */

/* DMA channel shift bit */
#define DMA_CHCTL(channel)                REG32(DMA_CHXCTL_BASE + (uint32_t)0x0000014U * (uint32_t)(channel))         /*!< the address of DMA channel CHXCTL register */
#define DMA_CHCNT(channel)                REG32(DMA_CHXCNT_BASE + (uint32_t)0x0000014U * (uint32_t)(channel))         /*!< the address of DMA channel CHXCNT register */
#define DMA_CHPADDR(channel)              REG32(DMA_CHXPADDR_BASE + (uint32_t)0x0000014U * (uint32_t)(channel))       /*!< the address of DMA channel CHXPADDR register */
#define DMA_CHMADDR(channel)              REG32(DMA_CHXMADDR_BASE + (uint32_t)0x0000014U * (uint32_t)(channel))       /*!< the address of DMA channel CHXMADDR register */

/* DMA_CHxCTL register */
/* interrupt enable bits */
#define DMA_INT_FTF                       DMA_CHXCTL_FTFIE                                /*!< enable bit for channel full transfer finish interrupt */
#define DMA_INT_HTF                       DMA_CHXCTL_HTFIE                                /*!< enable bit for channel half transfer finish interrupt */
#define DMA_INT_ERR                       DMA_CHXCTL_ERRIE                                /*!< enable bit for channel error interrupt */

/* DMA_CHxCNT register */
#define DMA_CHANNEL_CNT_MASK              DMA_CHXCNT_CNT                                 /* transfer counter */   

/* DMA channel remap */
#define DMA_REMAP_ADC                     SYSCFG_DMA_REMAP_ADC                          /* remap ADC DMA requests from channel0 to channel1 */      
#define DMA_REMAP_USART0_RX               SYSCFG_DMA_REMAP_USART0RX                     /* remap USART0 Rx DMA request to channel4(default channel2) */
#define DMA_REMAP_USART0_TX               SYSCFG_DMA_REMAP_USART0TX                     /* remap USART0 Tx DMA request to channel3(default channel1) */
#define DMA_REMAP_TIMER15_CH0_UP          SYSCFG_DMA_REMAP_TIMER15                      /* remap TIMER15 channel2 and UP DMA requests to channel3(defaut channel2) */
#define DMA_REMAP_TIMER16_CH0_UP          SYSCFG_DMA_REMAP_TIMER16                      /* remap TIMER16 channel0 and UP DMA requests to channel1(defaut channel0) */    

/* periph bind with dma */
#define hal_periph_dma_info_bind(_PERIPH_INFO, _P_DMA_TYPE, _DMA_INFO)      \
                          do{                                               \
                              (_PERIPH_INFO)._P_DMA_TYPE = &(_DMA_INFO);    \
                              (_DMA_INFO).p_periph = &(_PERIPH_INFO);       \
                          }while(0)

/* DMA settings*/
/* @PARA: channelx */
/* @ENUM: specify which DMA channel is initialized */
typedef enum {
    DMA_CH0 = 0,                          /*!< DMA Channel0 */
    DMA_CH1,                              /*!< DMA Channel1 */
    DMA_CH2,                              /*!< DMA Channel2 */
    DMA_CH3,                              /*!< DMA Channel3 */
    DMA_CH4,                              /*!< DMA Channel4 */
    DMA_CH5,                              /*!< DMA Channel5 */
    DMA_CH6                               /*!< DMA Channel6 */
} dma_channel_enum;

/* DMA */
/* DMA structure type enum */
typedef enum {
    HAL_DMA_INIT_STRUCT,                  /*!< DMA initialize structure */
    HAL_DMA_DEV_STRUCT,                   /*!< DMA device information structure */
    HAL_DMA_IRQ_STRUCT                    /*!< DMA device interrupt callback function pointer structure */
} hal_dma_struct_type_enum;

/* DMA transfer state enum */
typedef enum {
    HAL_DMA_TARNSFER_HALF,                /*!< half transfer */
    HAL_DMA_TARNSFER_FULL                 /*!< full transfer */
} hal_dma_transfer_state_enum;

/* DMA error type enum */
typedef enum {
    HAL_DMA_ERROR_NONE             = (uint32_t)0x00U,               /*!< no error */
    HAL_DMA_ERROR_TRANSFER         = (uint32_t)0x01U,               /*!< DMA transfer error */
    HAL_DMA_ERROR_NOTRANSFER       = (uint32_t)0x02U,               /*!< DMA not in transfer error */
    HAL_DMA_ERROR_TIMEOUT          = (uint32_t)0x03U,               /*!< DMA transfer timeout error */
    HAL_DMA_ERROR_UNSUPPORT        = (uint32_t)0x04U,               /*!< DMA transfer mode configure error */
} hal_dma_error_enum;

/* DMA state enum */
typedef enum {
    HAL_DMA_STATE_NONE = 0,                                         /*!< NONE(default value) */
    HAL_DMA_STATE_RESET,                                            /*!< RESET */
    HAL_DMA_STATE_BUSY,                                             /*!< BUSY */
    HAL_DMA_STATE_TIMEOUT,                                          /*!< TIMEOUT */
    HAL_DMA_STATE_ERROR,                                            /*!< ERROR */
    HAL_DMA_STATE_READY,                                            /*!< READY */
} hal_dma_state_enum;

/* DMA device interrupt callback function pointer structure */
typedef struct {
    __IO hal_irq_handle_cb error_handle;            /*!< channel error interrupt handler pointer */
    __IO hal_irq_handle_cb half_finish_handle;      /*!< channel half transfer finish interrupt handler pointer */
    __IO hal_irq_handle_cb full_finish_handle;      /*!< channel full transfer finish interrupt handler pointer */
} hal_dma_irq_struct;
/* DMA device information structure */
typedef struct {
    dma_channel_enum       channel;                 /*!< DMA channel */
    hal_dma_irq_struct     dma_irq;                 /*!< DMA device interrupt callback function pointer structure */
    hal_dma_transfer_state_enum transfer_state;     /*!< DMA device transfer state */
    hal_dma_error_enum     error_state;             /*!< DMA device error state */
    hal_dma_state_enum     state;                   /*!< DMA device state */
    void                   *p_periph;               /*!< user private periph */
    hal_mutex_enum         mutex;                   /*!< DMA device mutex */
} hal_dma_dev_struct;

/* @PARA: dma */
/* @STRUCT: DMA init structure */
typedef struct {
    uint32_t direction;                             /*!< transfer direction */
    uint32_t priority;                              /*!< channel priority  */
    uint32_t mode;                                  /*!< transfer mode */
    uint32_t periph_inc;                            /*!< peripheral increasing mode */
    uint32_t memory_inc;                            /*!< memory increasing mode */
    uint32_t periph_width;                          /*!< transfer data size of peripheral */
    uint32_t memory_width;                          /*!< transfer data size of memory */
} hal_dma_init_struct;

/* @STRUCT_MEMBER: direction */
/* @DEFINE: transfer direction */
#define DMA_DIR_PERIPH_TO_MEMORY            ((uint32_t)0x00000000U)                    /*!< read from peripheral and write to memory */
#define DMA_DIR_MEMORY_TO_PERIPH            DMA_CHXCTL_DIR                             /*!< read from memory and write to peripheral */
#define DMA_DIR_MEMORY_TO_MEMORY            DMA_CHXCTL_M2M                             /*!< read from memory and write to memory */
/* @STRUCT_MEMBER: priority */
/* @DEFINE: channel priority */
#define DMA_PRIORITY_LEVEL_LOW              (BITS(12,13) & ((uint32_t)(0) << 12))     /*!< low priority */
#define DMA_PRIORITY_LEVEL_MEDIUM           (BITS(12,13) & ((uint32_t)(1) << 12))     /*!< medium priority */
#define DMA_PRIORITY_LEVEL_HIGH             (BITS(12,13) & ((uint32_t)(2) << 12))     /*!< high priority */
#define DMA_PRIORITY_LEVEL_ULTRA_HIGH       (BITS(12,13) & ((uint32_t)(3) << 12))     /*!< ultra high priority */
/* @STRUCT_MEMBER: mode */
/* @DEFINE: transfer mode */
#define DMA_NORMAL_MODE                     (0x00000000U)                             /*!< disable circulation mode */
#define DMA_CIRCULAR_MODE                   DMA_CHXCTL_CMEN                           /*!< enable circulation mode */

/* @STRUCT_MEMBER: peripheral_increasing_mode */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: memory_increasing_mode */
/* @SP: ENABLE/DISABLE */

/* @STRUCT_MEMBER: peripheral_width */
/* @DEFINE: peripheral width */
#define DMA_PERIPH_SIZE_8BITS               (BITS(8,9) & ((0) << 8))                  /*!< transfer data size of peripheral is 8-bit */
#define DMA_PERIPH_SIZE_16BITS              (BITS(8,9) & ((1) << 8))                  /*!< transfer data size of peripheral is 16-bit */
#define DMA_PERIPH_SIZE_32BITS              (BITS(8,9) & ((2) << 8))                  /*!< transfer data size of peripheral is 32-bit */

/* @STRUCT_MEMBER: memory_width */
/* @DEFINE: memory width */
#define DMA_MEMORY_SIZE_8BITS               (BITS(10,11) & ((0) << 10))               /*!< transfer data size of memory is 8-bit */
#define DMA_MEMORY_SIZE_16BITS              (BITS(10,11) & ((1) << 10))               /*!< transfer data size of memory is 16-bit */
#define DMA_MEMORY_SIZE_32BITS              (BITS(10,11) & ((2) << 10))               /*!< transfer data size of memory is 32-bit */

/* @FUNCTION: initialize DMA channel */
int32_t hal_dma_init(hal_dma_dev_struct *dma_dev, dma_channel_enum channelx, hal_dma_init_struct *dma);
/* @END */

/* initialize the DMA structure with the default values */
int32_t hal_dma_struct_init(hal_dma_struct_type_enum struct_type, void *p_struct);
/* deinitialize DMA device structure and init structure */
int32_t hal_dma_deinit(hal_dma_dev_struct *dma_dev);
/* start DMA transfering */
int32_t hal_dma_start(hal_dma_dev_struct *dma_dev, uint32_t src_addr, \
                      uint32_t dst_addr, uint16_t length);
/* stop DMA transfering */
int32_t hal_dma_stop(hal_dma_dev_struct *dma_dev);
/* DMA interrupt handler content function,which is merely used in dma_handler*/
int32_t hal_dma_irq(hal_dma_dev_struct *dma_dev);
/* set user-defined interrupt callback function,
    which will be registered and called when corresponding interrupt be triggered */
int32_t hal_dma_irq_handle_set(hal_dma_dev_struct *dma_dev, \
                               hal_dma_irq_struct *p_irq);
/* reset all user-defined interrupt callback function,
    which will be registered and called when corresponding interrupt be triggered */
int32_t hal_dma_irq_handle_all_reset(hal_dma_dev_struct *dma_dev);

/* start transfering amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_dma_start_poll(hal_dma_dev_struct *dma_dev, hal_dma_transfer_state_enum \
                           transfer_state, uint32_t timeout_ms);
/* start transfering amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_dma_start_interrupt(hal_dma_dev_struct *dma_dev, uint32_t src_addr, \
                                uint32_t dst_addr, uint16_t length, \
                                hal_dma_irq_struct *p_irq);
/* stop transfering amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_dma_stop_interrupt(hal_dma_dev_struct *dma_dev);
/* enable the DMA channels remapping */
int32_t hal_dma_channel_remap_enable(uint32_t dma_remap);
/* disable the DMA channels remapping */
int32_t hal_dma_channel_remap_disable(uint32_t dma_remap);

/* enable DMA circulation mode */
void hals_dma_circulation_enable(dma_channel_enum channelx);
/* disable DMA circulation mode */
void hals_dma_circulation_disable(dma_channel_enum channelx);
/* enable memory to memory mode */
void hals_dma_memory_to_memory_enable(dma_channel_enum channelx);
/* disable memory to memory mode */
void hals_dma_memory_to_memory_disable(dma_channel_enum channelx);
/* enable DMA channel */
void hals_dma_channel_enable(dma_channel_enum channelx);
/* disable DMA channel */
void hals_dma_channel_disable(dma_channel_enum channelx);
/* set DMA peripheral base address */
void hals_dma_periph_address_config(dma_channel_enum channelx, uint32_t address);
/* set DMA memory base address */
void hals_dma_memory_address_config(dma_channel_enum channelx, uint32_t address);
/* set the number of remaining data to be transferred by the DMA */
void hals_dma_transfer_number_config(dma_channel_enum channelx, uint16_t number);

/* get the number of remaining data to be transferred by the DMA */
uint32_t hals_dma_transfer_number_get(dma_channel_enum channelx);
/* configure priority level of DMA channel */
void hals_dma_priority_config(dma_channel_enum channelx, uint32_t priority);
/* configure transfer data size of memory */
void hals_dma_memory_width_config(dma_channel_enum channelx, uint32_t mwidth);
/* configure transfer data size of peripheral */
void hals_dma_periph_width_config(dma_channel_enum channelx, uint32_t pwidth);
/* enable next address increasement algorithm of memory */
void hals_dma_memory_increase_enable(dma_channel_enum channelx);
/* disable next address increasement algorithm of memory */
void hals_dma_memory_increase_disable(dma_channel_enum channelx);
/* enable next address increasement algorithm of peripheral */
void hals_dma_periph_increase_enable(dma_channel_enum channelx);
/* disable next address increasement algorithm of peripheral */
void hals_dma_periph_increase_disable(dma_channel_enum channelx);
/* configure the direction of data transfer on the channel */
void hals_dma_transfer_direction_config(dma_channel_enum channelx, uint32_t direction);
/* check DMA flag is set or not */
FlagStatus hals_dma_flag_get(dma_channel_enum channelx, uint32_t flag);
/* clear DMA a channel flag */
void hals_dma_flag_clear(dma_channel_enum channelx, uint32_t flag);
/* enable DMA interrupt */
void hals_dma_interrupt_enable(dma_channel_enum channelx, uint32_t source);
/* disable DMA interrupt */
void hals_dma_interrupt_disable(dma_channel_enum channelx, uint32_t source);
/* check DMA flag and interrupt enable bit is set or not */
FlagStatus hals_dma_interrupt_flag_get(dma_channel_enum channelx, uint32_t flag);
/* clear DMA a channel flag */
void hals_dma_interrupt_flag_clear(dma_channel_enum channelx, uint32_t flag);

#endif /* GD32F3X0_HAL_DMA_H */
