/*!
    \file    gd32f3x0_hal_cec.h
    \brief   definitions for the CEC

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
#ifndef GD32F3X0_HAL_CEC_H
#define GD32F3X0_HAL_CEC_H

#include "gd32f3x0_hal.h"

/* CEC definitions */
#define CEC                                CEC_BASE                   /*!< CEC base address */

/* CEC settings*/
#define CEC_HEADER_ADDR_POS                (4U)
#define CEC_MAX_BUFFER_LEN                 (16U)

/* registers definitions */
#define CEC_CTL                            REG32(CEC + 0x00000000U)   /*!< CEC control register */
#define CEC_CFG                            REG32(CEC + 0x00000004U)   /*!< CEC configuration register */
#define CEC_TDATA                          REG32(CEC + 0x00000008U)   /*!< CEC transmit data register */
#define CEC_RDATA                          REG32(CEC + 0x0000000CU)   /*!< CEC receive data register */
#define CEC_INTF                           REG32(CEC + 0x00000010U)   /*!< CEC interrupt flag Register */
#define CEC_INTEN                          REG32(CEC + 0x00000014U)   /*!< CEC interrupt enable register */

/* bits definitions */
/* CEC_CTL */
#define CEC_CTL_CECEN                      BIT(0)                     /*!< enable or disable HDMI-CEC controller bit */
#define CEC_CTL_STAOM                      BIT(1)                     /*!< start of sending a message. */
#define CEC_CTL_ENDOM                      BIT(2)                     /*!< ENDOM bit value in the next frame in Tx mode */

/* CEC_INTF */
#define CEC_INTF_BR                        BIT(0)                     /*!< Rx-byte data received */
#define CEC_INTF_REND                      BIT(1)                     /*!< end of reception */
#define CEC_INTF_RO                        BIT(2)                     /*!< Rx overrun */
#define CEC_INTF_BRE                       BIT(3)                     /*!< bit rising error */
#define CEC_INTF_BPSE                      BIT(4)                     /*!< short bit period error */
#define CEC_INTF_BPLE                      BIT(5)                     /*!< long bit period error */
#define CEC_INTF_RAE                       BIT(6)                     /*!< Rx ACK error */
#define CEC_INTF_ARBF                      BIT(7)                     /*!< arbitration fail */
#define CEC_INTF_TBR                       BIT(8)                     /*!< Tx-byte data request */
#define CEC_INTF_TEND                      BIT(9)                     /*!< transmission successfully end */
#define CEC_INTF_TU                        BIT(10)                    /*!< Tx data buffer underrun */
#define CEC_INTF_TERR                      BIT(11)                    /*!< Tx-error */
#define CEC_INTF_TAERR                     BIT(12)                    /*!< Tx ACK error flag */

/* interrupt flag bits */
#define CEC_INT_FLAG_BR                    CEC_INTF_BR                /*!< RX-byte data received */
#define CEC_INT_FLAG_REND                  CEC_INTF_REND              /*!< end of reception */
#define CEC_INT_FLAG_RO                    CEC_INTF_RO                /*!< RX overrun */
#define CEC_INT_FLAG_BRE                   CEC_INTF_BRE               /*!< bit rising error */
#define CEC_INT_FLAG_BPSE                  CEC_INTF_BPSE              /*!< short bit period error */
#define CEC_INT_FLAG_BPLE                  CEC_INTF_BPLE              /*!< long bit period error */
#define CEC_INT_FLAG_RAE                   CEC_INTF_RAE               /*!< RX ACK error */
#define CEC_INT_FLAG_ARBF                  CEC_INTF_ARBF              /*!< arbitration lost */
#define CEC_INT_FLAG_TBR                   CEC_INTF_TBR               /*!< TX-byte data request */
#define CEC_INT_FLAG_TEND                  CEC_INTF_TEND              /*!< transmission successfully end */
#define CEC_INT_FLAG_TU                    CEC_INTF_TU                /*!< TX data buffer underrun */
#define CEC_INT_FLAG_TERR                  CEC_INTF_TERR              /*!< TX-error */
#define CEC_INT_FLAG_TAERR                 CEC_INTF_TAERR             /*!< TX ACK error flag */

/* CEC_INTEN */
#define CEC_INTEN_BRIE                     BIT(0)                     /*!< BR interrupt enable */
#define CEC_INTEN_RENDIE                   BIT(1)                     /*!< REND interrupt enable */
#define CEC_INTEN_ROIE                     BIT(2)                     /*!< RO interrupt enable */
#define CEC_INTEN_BREIE                    BIT(3)                     /*!< BRE interrupt enable. */
#define CEC_INTEN_BPSEIE                   BIT(4)                     /*!< BPSE interrupt enable */
#define CEC_INTEN_BPLEIE                   BIT(5)                     /*!< BPLE interrupt enable. */
#define CEC_INTEN_RAEIE                    BIT(6)                     /*!< RAE interrupt enable */
#define CEC_INTEN_ARBFIE                   BIT(7)                     /*!< ARBF interrupt enable */
#define CEC_INTEN_TBRIE                    BIT(8)                     /*!< TBR interrupt enable */
#define CEC_INTEN_TENDIE                   BIT(9)                     /*!< TEND interrupt enable */
#define CEC_INTEN_TUIE                     BIT(10)                    /*!< TU interrupt enable */
#define CEC_INTEN_TERRIE                   BIT(11)                    /*!< TE interrupt enable */
#define CEC_INTEN_TAERRIE                  BIT(12)                    /*!< TAE interrupt enable */

#define CFG_SFT(regval)                    (BITS(0, 2) & ((regval) << 0U))

/* CEC state enum */
typedef enum {
    HAL_CEC_STATE_NONE = 0,                                           /*!< NONE(default value) */
    HAL_CEC_STATE_RESET,                                              /*!< RESET */
    HAL_CEC_STATE_READY,                                              /*!< READY */
    HAL_CEC_STATE_BUSY,                                               /*!< BUSY */
    HAL_CEC_STATE_BUSY_RX,                                            /*!< BUSY_RX */
    HAL_CEC_STATE_BUSY_TX                                             /*!< BUSY_TX */
} hal_cec_state_enum;

/* CEC structure type enum */
typedef enum {
    HAL_CEC_INIT_STRUCT,                                              /*!< CEC initialization structure */
    HAL_CEC_IRQ_STRUCT,                                               /*!< CEC device interrupt callback function pointer structure */
    HAL_CEC_DEV_STRUCT                                                /*!< CEC device information structrue */
} hal_cec_struct_type_enum;

/* CEC error type enum */
typedef enum {
    HAL_CEC_ERROR_NONE            = (uint32_t)0x0000U,                /*!< no error */
    HAL_CEC_ERROR_RO              = (uint32_t)0x0004U,                /*!< RX overrun */
    HAL_CEC_ERROR_BRE             = (uint32_t)0x0008U,                /*!< bit rising error */
    HAL_CEC_ERROR_BPSE            = (uint32_t)0x0010U,                /*!< short bit period error */
    HAL_CEC_ERROR_BPLE            = (uint32_t)0x0020U,                /*!< long bit period error */
    HAL_CEC_ERROR_RAE             = (uint32_t)0x0040U,                /*!< RX ACK error */
    HAL_CEC_ERROR_ARBF            = (uint32_t)0x0080U,                /*!< arbitration lost */
    HAL_CEC_ERROR_TU              = (uint32_t)0x0400U,                /*!< long bit period error */
    HAL_CEC_ERROR_TERR            = (uint32_t)0x0800U,                /*!< RX ACK error */
    HAL_CEC_ERROR_TAERR           = (uint32_t)0x1000U                 /*!< arbitration lost */
} hal_cec_error_enum;

/* CEC device interrupt callback function pointer structure */
typedef struct {
    __IO hal_irq_handle_cb                cec_tx_handle;              /*!< CEC tx handler function */
    __IO hal_irq_handle_cb                cec_rx_handle;              /*!< CEC rx handler function */
} hal_cec_irq_struct;

/* CEC device information structrue */
typedef struct {
    hal_cec_irq_struct               cec_irq;                         /*!< CEC device interrupt callback function pointer structure */
    hal_cec_state_enum               state;                           /*!< CEC state */
    hal_cec_error_enum               error_state;                     /*!< CEC error state */
    __IO uint8_t                     *tx_buffer;                      /*!< tx buffer address */
    __IO uint32_t                    tx_count;                        /*!< tx buffer count */
    __IO uint8_t                     *rx_buffer;                      /*!< rx buffer address */
    __IO uint32_t                    rx_count;                        /*!< rx buffer count */
    void                             *err_callback;                   /*!< CEC error callback fuction */
    void                             *tx_callback;                    /*!< CEC transmit callback fuction */
    void                             *rx_callback;                    /*!< CEC receive callback fuction */
    hal_mutex_enum                   mutex;                           /*!< mutex locked and unlocked state */
    void                             *priv;                           /*!< priv data */
} hal_cec_dev_struct;

/* CEC callback fuction */
typedef void (*hal_cec_user_cb)(hal_cec_dev_struct *cec);

/* @PARA: cec */
/* @STRUCT: CEC initialization config struct */
typedef struct {
    uint32_t signal_free_time;                                        /*!< signal free time */
    uint32_t reception_bit_timing_tolerance;                          /*!< reception bit timing tolerance */
    uint32_t sft_start_option_bit;                                    /*!< the SFT start option bit */
    uint32_t listen_mode;                                             /*!< listen mode enable bit */
    uint32_t own_address;                                             /*!< own address */
    uint32_t bre_stop_receive;                                        /*!< whether stop receive message when detected BRE */
    uint32_t bre_generate_error;                                      /*!< generate an error-bit when detected BRE in singlecast */
    uint32_t blpe_generate_error;                                     /*!< generate an error-bit when detected BPLE in singlecast */
    uint32_t not_generate_error_broadcast;                            /*!< do not generate an error-bit in broadcast message */
} hal_cec_init_struct;

/* @STRUCT_MEMBER: signal_free_time */
/* @DEFINE: signal free time */
#define CEC_SFT_PROTOCOL_PERIOD      CFG_SFT(0)                       /*!< the signal free time will perform as HDMI-CEC protocol description */
#define CEC_SFT_1POINT5_PERIOD       CFG_SFT(1)                       /*!< 1.5 nominal data bit periods */
#define CEC_SFT_2POINT5_PERIOD       CFG_SFT(2)                       /*!< 2.5 nominal data bit periods */
#define CEC_SFT_3POINT5_PERIOD       CFG_SFT(3)                       /*!< 3.5 nominal data bit periods */
#define CEC_SFT_4POINT5_PERIOD       CFG_SFT(4)                       /*!< 4.5 nominal data bit periods */
#define CEC_SFT_5POINT5_PERIOD       CFG_SFT(5)                       /*!< 5.5 nominal data bit periods */
#define CEC_SFT_6POINT5_PERIOD       CFG_SFT(6)                       /*!< 6.5 nominal data bit periods */
#define CEC_SFT_7POINT5_PERIOD       CFG_SFT(7)                       /*!< 7.5 nominal data bit periods */

/* @STRUCT_MEMBER: reception_bit_timing_tolerance */
/* @DEFINE: reception bit timing tolerance */
#define CEC_STANTARD_RTOL            ((uint32_t)0x00000000U)          /*!< Extended bit timing tolerance */
#define CEC_EXTENDED_RTOL            BIT(3)                           /*!< Standard bit timing tolerance */

/* @STRUCT_MEMBER: sft_start_option_bit */
/* @DEFINE: the SFT start option bit */
#define CEC_SFT_START_STAOM          ((uint32_t)0x00000000U)          /*!< signal free time counter starts counting when STAOM is asserted */
#define CEC_SFT_START_LAST           BIT(8)                           /*!< signal free time counter starts automatically after transmission/reception end */

/* @STRUCT_MEMBER: listen_mode */
/* @DEFINE: listen mode enable bit */
#define CEC_PARTIAL_LISTENING_MODE   ((uint32_t)(0x00000000U))        /*!< Only receive broadcast and singlecast in OAD address with appropriate ACK */
#define CEC_WHOLE_LISTENING_MODE     BIT(31)                          /*!< Receive broadcast and singlecast in OAD address with appropriate ACK and receive message whose destination address is not in OAD without feedback ACK */

/* @STRUCT_MEMBER: own_address */
/* @DEFINE: own address */
#define CEC_OWN_ADDRESS0                   BIT(16)                    /*!< own address is 0 */
#define CEC_OWN_ADDRESS1                   BIT(17)                    /*!< own address is 1 */
#define CEC_OWN_ADDRESS2                   BIT(18)                    /*!< own address is 2 */
#define CEC_OWN_ADDRESS3                   BIT(19)                    /*!< own address is 3 */
#define CEC_OWN_ADDRESS4                   BIT(20)                    /*!< own address is 4 */
#define CEC_OWN_ADDRESS5                   BIT(21)                    /*!< own address is 5 */
#define CEC_OWN_ADDRESS6                   BIT(22)                    /*!< own address is 6 */
#define CEC_OWN_ADDRESS7                   BIT(23)                    /*!< own address is 7 */
#define CEC_OWN_ADDRESS8                   BIT(24)                    /*!< own address is 8 */
#define CEC_OWN_ADDRESS9                   BIT(25)                    /*!< own address is 9 */
#define CEC_OWN_ADDRESS10                  BIT(26)                    /*!< own address is 10 */
#define CEC_OWN_ADDRESS11                  BIT(27)                    /*!< own address is 11 */
#define CEC_OWN_ADDRESS12                  BIT(28)                    /*!< own address is 12 */
#define CEC_OWN_ADDRESS13                  BIT(29)                    /*!< own address is 13 */
#define CEC_OWN_ADDRESS14                  BIT(30)                    /*!< own address is 14 */

/* @STRUCT_MEMBER: bre_stop_receive */
/* @DEFINE: whether stop receive message when detected BRE */
#define CEC_NOT_STOP_RECEPTION             ((uint32_t)0x00000000U)    /*!< do not stop reception when detected bit rising error */
#define CEC_STOP_RECEPTION                 BIT(4)                     /*!< stop reception when detected bit rising error */

/* @STRUCT_MEMBER: bre_generate_error */
/* @DEFINE: generate an error-bit when detected BRE in singlecast */
#define CEC_NO_RISING_PERIOD_ERROR         ((uint32_t)0x00000000U)    /*!< do not generate Error-bit on bit rising error */
#define CEC_GEN_RISING_PERIOD_ERROR        BIT(5)                     /*!< generate Error-bit on bit rising error */

/* @STRUCT_MEMBER: blpe_generate_error */
/* @DEFINE: generate an error-bit when detected BPLE in singlecast */
#define CEC_NO_LONG_PERIOD_ERROR           ((uint32_t)0x00000000U)    /*!< do not generate Error-bit on long bit period error */
#define CEC_GEN_LONG_PERIOD_ERROR          BIT(6)                     /*!< generate Error-bit on long bit period error */

/* @STRUCT_MEMBER: not_generate_error_broadcast */
/* @DEFINE: do not generate an error-bit in broadcast message */
#define CEC_GEN_BROADCAST_ERROR            ((uint32_t)0x00000000U)    /*!< generate Error-bit in broadcast */
#define CEC_NO_BROADCAST_ERROR             BIT(7)                     /*!< do not generate Error-bit in broadcast */


/* function declarations */
/* @FUNCTION: initialize the CEC structure with the default values */
void hal_cec_struct_init(hal_cec_struct_type_enum hal_struct_type, void *p_struct);

/* @FUNCTION: initialize CEC */
int32_t hal_cec_init(hal_cec_dev_struct *cec_dev, hal_cec_init_struct *cec);
/* @END */

/* deinitialize CEC device structure */
int32_t hal_cec_deinit(hal_cec_dev_struct *cec_dev);

/* start CEC module function */
int32_t hal_cec_start(hal_cec_dev_struct *cec_dev);

/* stop CEC module function */
int32_t hal_cec_stop(hal_cec_dev_struct *cec_dev);

/* transmit amounts of data by interrupt method */
int32_t hal_cec_transmit_interrupt(hal_cec_dev_struct *cec_dev, uint32_t tx_length, uint8_t *p_buffer, \
                                   uint8_t header_addr, uint8_t destination_addr, hal_cec_user_cb p_func);

/* receive amounts of data by interrupt method */
int32_t hal_cec_receive_interrupt(hal_cec_dev_struct *cec_dev, uint8_t *p_buffer, hal_cec_user_cb p_func);

/* set user-defined interrupt callback function */
void hal_cec_irq_handle_set(hal_cec_dev_struct *cec_dev, hal_cec_irq_struct *p_irq);

/* reset all user-defined interrupt callback function */
void hal_cec_irq_handle_all_reset(hal_cec_dev_struct *cec_dev);

/* CEC interrupt handler content function */
void hal_cec_irq(hal_cec_dev_struct *cec_dev);

#endif /* GD32F3X0_HAL_CEC_H */
#endif /* GD32F350 */
