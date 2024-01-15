/*!
    \file    gd32f3x0_hal_i2c.h
    \brief   definitions for the I2C

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

#ifndef GD32F3X0_HAL_I2C_H
#define GD32F3X0_HAL_I2C_H

#include "gd32f3x0_hal.h"

/* constants definitions */
/* define the I2C bit position and its register index offset */
#define I2C_REGIDX_BIT(regidx, bitpos)  (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos))
#define I2C_REG_VAL(i2cx, offset)       (REG32((i2cx) + (((uint32_t)(offset) & 0x0000FFFFU) >> 6)))
#define I2C_BIT_POS(val)                ((uint32_t)(val) & 0x0000001FU)
#define I2C_REGIDX_BIT2(regidx, bitpos, regidx2, bitpos2)   (((uint32_t)(regidx2) << 22) | (uint32_t)((bitpos2) << 16)\
                                                              | (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos)))
#define I2C_REG_VAL2(i2cx, offset)      (REG32((i2cx) + ((uint32_t)(offset) >> 22)))
#define I2C_BIT_POS2(val)               (((uint32_t)(val) & 0x001F0000U) >> 16)

/* register offset */
#define I2C_CTL1_REG_OFFSET           (0x00000004U)                              /*!< CTL1 register offset */
#define I2C_STAT0_REG_OFFSET          (0x00000014U)                              /*!< STAT0 register offset */
#define I2C_STAT1_REG_OFFSET          (0x00000018U)                              /*!< STAT1 register offset */

/* I2C flags */
typedef enum {
    /* flags in STAT0 register */
    I2C_FLAG_SBSEND = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 0U),                 /*!< start condition sent out in master mode */
    I2C_FLAG_ADDSEND = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 1U),                /*!< address is sent in master mode or received and matches in slave mode */
    I2C_FLAG_BTC = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 2U),                    /*!< byte transmission finishes */
    I2C_FLAG_ADD10SEND = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 3U),              /*!< header of 10-bit address is sent in master mode */
    I2C_FLAG_STPDET = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 4U),                 /*!< stop condition detected in slave mode */
    I2C_FLAG_RBNE = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 6U),                   /*!< I2C_DATA is not empty during receiving */
    I2C_FLAG_TBE = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 7U),                    /*!< I2C_DATA is empty during transmitting */
    I2C_FLAG_BERR = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 8U),                   /*!< a bus error occurs indication a unexpected start or stop condition on I2C bus */
    I2C_FLAG_LOSTARB = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 9U),                /*!< arbitration lost in master mode */
    I2C_FLAG_AERR = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 10U),                  /*!< acknowledge error */
    I2C_FLAG_OUERR = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 11U),                 /*!< over-run or under-run situation occurs in slave mode */
    I2C_FLAG_PECERR = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 12U),                /*!< PEC error when receiving data */
    I2C_FLAG_SMBTO = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 14U),                 /*!< timeout signal in SMBus mode */
    I2C_FLAG_SMBALT = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 15U),                /*!< SMBus alert status */
    /* flags in STAT1 register */
    I2C_FLAG_MASTER = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 0U),                 /*!< a flag indicating whether I2C block is in master or slave mode */
    I2C_FLAG_I2CBSY = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 1U),                 /*!< busy flag */
    I2C_FLAG_TR = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 2U),                     /*!< whether the I2C is a transmitter or a receiver */
    I2C_FLAG_RXGC = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 4U),                   /*!< general call address (00h) received */
    I2C_FLAG_DEFSMB = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 5U),                 /*!< default address of SMBus device */
    I2C_FLAG_HSTSMB = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 6U),                 /*!< SMBus host header detected in slave mode */
    I2C_FLAG_DUMOD = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 7U)                   /*!< dual flag in slave mode indicating which address is matched in dual-address mode */
} i2c_flag_enum;

/* I2C interrupt flags */
typedef enum {
    /* interrupt flags in CTL1 register */
    I2C_INT_FLAG_SBSEND = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 0U),        /*!< start condition sent out in master mode interrupt flag */
    I2C_INT_FLAG_ADDSEND = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 1U),       /*!< address is sent in master mode or received and matches in slave mode interrupt flag */
    I2C_INT_FLAG_BTC =  I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 2U),          /*!< byte transmission finishes interrupt flag */
    I2C_INT_FLAG_ADD10SEND =  I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 3U),    /*!< header of 10-bit address is sent in master mode interrupt flag */
    I2C_INT_FLAG_STPDET = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 4U),        /*!< stop condition detected in slave mode interrupt flag */
    I2C_INT_FLAG_RBNE = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 6U),          /*!< I2C_DATA is not empty during receiving interrupt flag */
    I2C_INT_FLAG_TBE = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 7U),           /*!< I2C_DATA is empty during transmitting interrupt flag */
    I2C_INT_FLAG_BERR = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 8U),          /*!< a bus error occurs indication a unexpected start or stop condition on I2C bus interrupt flag */
    I2C_INT_FLAG_LOSTARB = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 9U),       /*!< arbitration lost in master mode interrupt flag */
    I2C_INT_FLAG_AERR = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 10U),         /*!< acknowledge error interrupt flag */
    I2C_INT_FLAG_OUERR = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 11U),        /*!< over-run or under-run situation occurs in slave mode interrupt flag */
    I2C_INT_FLAG_PECERR = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 12U),       /*!< PEC error when receiving data interrupt flag */
    I2C_INT_FLAG_SMBTO = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 14U),        /*!< timeout signal in SMBus mode interrupt flag */
    I2C_INT_FLAG_SMBALT = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 15U),       /*!< SMBus alert status interrupt flag */
} i2c_interrupt_flag_enum;

/* I2C interrupt */
typedef enum {
    /* interrupt in CTL1 register */
    I2C_INT_ERR = I2C_REGIDX_BIT(I2C_CTL1_REG_OFFSET, 8U),                      /*!< error interrupt enable */
    I2C_INT_EV = I2C_REGIDX_BIT(I2C_CTL1_REG_OFFSET, 9U),                       /*!< event interrupt enable */
    I2C_INT_BUF = I2C_REGIDX_BIT(I2C_CTL1_REG_OFFSET, 10U),                     /*!< buffer interrupt enable */
} i2c_interrupt_enum;

#define HAL_I2C_ENABLE( i2c_periph )                (I2C_CTL0(i2c_periph) |= I2C_CTL0_I2CEN)    /*!< enable I2C */
#define HAL_I2C_DISABLE( i2c_periph )               (I2C_CTL0(i2c_periph) &= ~(I2C_CTL0_I2CEN))  /*!< disable I2C */
#define HAL_I2C_START_ON_BUS( i2c_periph )          (I2C_CTL0(i2c_periph) |= I2C_CTL0_START)     /*!< generate a START condition on I2C bus */
#define HAL_I2C_STOP_ON_BUS( i2c_periph )           (I2C_CTL0(i2c_periph) |= I2C_CTL0_STOP)      /*!< generate a STOP condition on I2C bus */

typedef enum {
    HAL_I2C_INIT_STRUCT,                                                       /*!< I2C initialize structure */
    HAL_I2C_DEV_STRUCT,                                                        /*!< I2C device information structure */
    HAL_I2C_IRQ_STRUCT,                                                        /*!< I2C device interrupt callback function pointer structure */
} hal_i2c_struct_type_enum;

#define I2C_BUSY_TIMEOUT                    ((uint32_t)10000)                  /* 10s timeout */
#define I2C_TIMEOUT                         ((uint32_t)100000)                 /* 100s timeout */
#define I2C_ADDR_TIMEOUT                    ((uint32_t)10000)                  /* 10s timeout */

/* I2C error state */
#define HAL_I2C_ERROR_NONE                  0U                                 /*!< no error */
#define HAL_I2C_ERROR_BERR                  BIT(0)                             /*!< bus error */
#define HAL_I2C_ERROR_LOSTARB               BIT(1)                             /*!< arbitration lost in master mode */
#define HAL_I2C_ERROR_AERR                  BIT(2)                             /*!< acknowledge error */
#define HAL_I2C_ERROR_OUERR                 BIT(3)                             /*!< over-run error */
#define HAL_I2C_ERROR_PECERR                BIT(4)                             /*!< PEC error */
#define HAL_I2C_ERROR_DMATX                 BIT(7)                             /*!< DMA TX error */
#define HAL_I2C_ERROR_DMARX                 BIT(8)                             /*!< DMA RX error */

/* I2C previous state */
#define HAL_I2C_PREVIOUS_STATE_NONE         0U                                 /*!< default value */
#define HAL_I2C_PREVIOUS_STATE_TX           BIT(0)                             /*!< the last communication is TX */
#define HAL_I2C_PREVIOUS_STATE_RX           BIT(1)                             /*!< the last communication is RX */

/* I2C frame flag in serial transfer */
#define I2C_NO_OPTION_TRANSFER              ((uint16_t)0x0001U)                /*!< there is only one frame in serial transfer */
#define I2C_FIRST_TRANSFER                  ((uint16_t)0x0002U)                /*!< first frame in serial transfer */
#define I2C_LAST_TRANSFER                   ((uint16_t)0x0003U)                /*!< last frame in serial transfer */
#define I2C_NEXT_TRANSFER                   ((uint16_t)0x0004U)                /*!< next frame in serial transfer, there are at least there frames in serial transfer */

#define I2C_MEMORY_ADDRESS_8BIT             ((uint16_t)0x0000U)                /*!< memory address is 8 bits */
#define I2C_MEMORY_ADDRESS_16BIT            ((uint32_t)0x0001U)                /*!< memory address is 16 bits */

typedef struct {
    __IO uint8_t *buffer;                                                      /*!< pointer to transfer buffer */
    __IO uint32_t length;                                                      /*!< transfer length */
    __IO uint32_t pos;                                                         /*!< transfer position */
} i2c_buffer_struct;

typedef struct {
    hal_irq_handle_cb event_handle;                                            /*!< event callback function */
    hal_irq_handle_cb error_handle;                                            /*!< error callback function */
} hal_i2c_irq_struct;

typedef enum {
    HAL_I2C_STATE_READY,                                                       /*!< I2C is ready for use */
    HAL_I2C_STATE_BUSY,                                                        /*!< I2C transfer process is ongoing */
    HAL_I2C_STATE_MEMORY_BUSY_TX,                                              /*!< I2C write memory process is ongoing */
    HAL_I2C_STATE_MEMORY_BUSY_RX,                                              /*!< I2C read memory process is ongoing */
    HAL_I2C_STATE_LISTEN,                                                      /*!< I2C addressing listen is ongoing in slave mode */
    HAL_I2C_STATE_BUSY_LISTEN                                                  /*!< I2C addressing listen and data transfer is ongoing */
} hal_i2c_run_state_enum;

typedef struct{
    uint32_t                        device_address;                            /*!< device address */
    uint16_t                        memory_address;                            /*!< memory address */
    uint16_t                        address_size;                              /*!< memory address size */
    FlagStatus                      address_complete;                          /*!< addressing complete flag, initialize to RESET */
    uint8_t                         address_count;                             /*!< address count, initialize to 0 */
    FlagStatus                      second_addressing;                         /*!< initialize to RESET */
} hal_i2c_slave_address_struct;

typedef struct {
    uint32_t                        periph;                                    /*!< I2C port */
    hal_i2c_irq_struct              i2c_irq;                                   /*!< device interrupt callback function pointer */
    hal_dma_dev_struct              *p_dma_rx;                                 /*!< DMA receive pointer */
    hal_dma_dev_struct              *p_dma_tx;                                 /*!< DMA transmit pointer */
    i2c_buffer_struct               txbuffer;                                  /*!< transmit buffer */
    i2c_buffer_struct               rxbuffer;                                  /*!< receive buffer */
    hal_i2c_slave_address_struct    slave_address;                             /*!< slave address */
    __IO uint16_t                   transfer_option;                           /*!< transfer option */
    __IO uint16_t                   last_error;                                /*!< the last error code */
    __IO uint32_t                   error_state;                               /*!< error state */
    __IO hal_i2c_run_state_enum     tx_state;                                  /*!< transmit state */
    __IO hal_i2c_run_state_enum     rx_state;                                  /*!< receive state */
    __IO uint32_t                   previous_state;                            /*!< previous state */
    void                            *rx_callback;                              /*!< receive callback function pointer */
    void                            *tx_callback;                              /*!< transmit callback function pointer */
    hal_mutex_enum                  mutex;                                     /*!< mutex locked and unlocked state */
} hal_i2c_dev_struct;

typedef void (*hal_i2c_user_cb)(hal_i2c_dev_struct *i2c_dev);

/* I2C register bit mask */
#define I2CCLK_MAX                    ((uint32_t)0x0000007FU)             /*!< i2cclk maximum value */
#define I2CCLK_MIN                    ((uint32_t)0x00000002U)             /*!< i2cclk minimum value */
#define I2C_FLAG_MASK                 ((uint32_t)0x0000FFFFU)             /*!< i2c flag mask */
#define I2C_ADDRESS_MASK              ((uint32_t)0x000003FFU)             /*!< i2c address mask */
#define I2C_ADDRESS2_MASK             ((uint32_t)0x000000FEU)             /*!< the second i2c address mask */

/* I2C register bit offset */
#define STAT1_PECV_OFFSET             ((uint32_t)8U)     /* bit offset of PECV in I2C_STAT1 */

/* I2C DMA mode configure */
/* DMA mode switch */
#define I2C_DMA_ON                    I2C_CTL1_DMAON                           /*!< DMA mode enabled */
#define I2C_DMA_OFF                   ((uint32_t)0x00000000U)                  /*!< DMA mode disabled */

/* flag indicating DMA last transfer */
#define I2C_DMALST_ON                 I2C_CTL1_DMALST                          /*!< next DMA EOT is the last transfer */
#define I2C_DMALST_OFF                ((uint32_t)0x00000000U)                  /*!< next DMA EOT is not the last transfer */

/* SMBus/I2C mode switch and SMBus type selection */
#define I2C_I2CMODE_ENABLE            ((uint32_t)0x00000000U)                  /*!< I2C mode */
#define I2C_SMBUSMODE_ENABLE          I2C_CTL0_SMBEN                           /*!< SMBus mode */

/* I2C transfer direction */
#define I2C_RECEIVER                  ((uint32_t)0x00000001U)                  /*!< receiver */
#define I2C_TRANSMITTER               ((uint32_t)0xFFFFFFFEU)                  /*!< transmitter */

/* whether or not to send an ACK */
#define I2C_ACK_DISABLE               ((uint32_t)0x00000000U)                  /*!< ACK will be not sent */
#define I2C_ACK_ENABLE                I2C_CTL0_ACKEN                           /*!< ACK will be sent */

/* I2C POAP position*/
#define I2C_ACKPOS_CURRENT            ((uint32_t)0x00000000U)                  /*!< ACKEN bit decides whether or not to send ACK or not for the current byte */
#define I2C_ACKPOS_NEXT               I2C_CTL0_POAP                            /*!< ACKEN bit decides whether or not to send ACK for the next byte */

/* software reset I2C */
#define I2C_SRESET_RESET              ((uint32_t)0x00000000U)                  /*!< I2C is not under reset */
#define I2C_SRESET_SET                I2C_CTL0_SRESET                          /*!< I2C is under reset */

/* issue or not alert through SMBA pin */
#define I2C_SALTSEND_DISABLE          ((uint32_t)0x00000000U)                  /*!< not issue alert through SMBA */
#define I2C_SALTSEND_ENABLE           I2C_CTL0_SALT                            /*!< issue alert through SMBA pin */

/* transmit I2C data */
#define DATA_TRANS(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))

/* receive I2C data */
#define DATA_RECV(regval)             GET_BITS((uint32_t)(regval), 0, 7)

/* I2C settings*/
/* @PARA: i2c_init */
/* @STRUCT: I2C init struct */
typedef struct {
    uint32_t clock_speed;       /*!< I2C clock speed */
    uint32_t duty_cycle;        /*!< duty cycle in fast mode or fast mode plus */
    uint32_t address_format;    /*!< I2C addformat, 7bits or 10bits */
    uint32_t own_address1;      /*!< I2C own address */
    uint32_t dual_address;      /*!< dual-address mode switch */
    uint32_t own_address2;      /*!< I2C own address2 in dual-address mode */
    uint32_t general_call;      /*!< whether or not to response to a general call */
    uint32_t no_stretch;        /*!< whether to stretch SCL low when data is not ready in slave mode */
}hal_i2c_init_struct;

/* @STRUCT_MEMBER: duty_cycle */
/* @DEFINE: duty cycle in fast mode or fast mode plus */
#define I2C_DTCY_2                    ((uint32_t)0x00000000U)                  /*!< Tlow/Thigh = 2 in I2C fast mode or fast mode plus */
#define I2C_DTCY_16_9                 I2C_CKCFG_DTCY                           /*!< Tlow/Thigh = 16/9 in I2C fast mode or fast mode plus */

/* @STRUCT_MEMBER: clock_speed */
/* @=NULL */

/* @STRUCT_MEMBER: address_format*/
/* @DEFINE: I2C clock speed */
#define I2C_ADDFORMAT_7BITS           ((uint32_t)0x00000000U)                  /*!< address format is 7 bits */
#define I2C_ADDFORMAT_10BITS          I2C_SADDR0_ADDFORMAT                     /*!< address format is 10 bits */

/* @STRUCT_MEMBER: own_address1 */
/* @=NULL */

/* @STRUCT_MEMBER: dual_address */
/* @DEFINE: dual-address mode switch  */
#define I2C_DUADEN_DISABLE                  ((uint32_t)0x00000000U)            /*!< dual-address mode disabled */
#define I2C_DUADEN_ENABLE                   I2C_SADDR1_DUADEN                  /*!< dual-address mode enabled */

/* @STRUCT_MEMBER: own_address2 */
/* @=NULL */

/* @STRUCT_MEMBER: general_call */
/* @DEFINE: whether or not to response to a general call */
#define I2C_GCEN_ENABLE               I2C_CTL0_GCEN                            /*!< slave will response to a general call */
#define I2C_GCEN_DISABLE              ((uint32_t)0x00000000U)                  /*!< slave will not response to a general call */

/* @STRUCT_MEMBER: no_stretch */
/* @DEFINE: whether to stretch SCL low when data is not ready in slave mode */
#define I2C_SCLSTRETCH_ENABLE         ((uint32_t)0x00000000U)                  /*!< SCL stretching is enabled */
#define I2C_SCLSTRETCH_DISABLE        I2C_CTL0_SS                              /*!< SCL stretching is disabled */

/* function declarations */
/* @FUNCTION: initialize the I2C structure with the default values  */
void hal_i2c_struct_init(hal_i2c_struct_type_enum struct_type, void *p_struct);

/* @FUNCTION: deinitialize I2C  */
void hal_i2c_deinit(hal_i2c_dev_struct *i2c_dev);

/* @FUNCTION: initialize I2C */
int32_t hal_i2c_init(hal_i2c_dev_struct *i2c_dev, uint32_t periph, hal_i2c_init_struct *i2c_init); 
/* @END */

/* I2C error interrupt handler */
void hal_i2c_error_irq(hal_i2c_dev_struct *i2c_dev);
/* I2C event interrupt handler */
void hal_i2c_event_irq(hal_i2c_dev_struct *i2c_dev);
/* start I2C module function */
void hal_i2c_start(hal_i2c_dev_struct *i2c_dev);
/* stop I2C module function */
void hal_i2c_stop(hal_i2c_dev_struct *i2c_dev);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_i2c_irq_handle_set(hal_i2c_dev_struct *i2c_dev, hal_i2c_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_i2c_irq_handle_all_reset(hal_i2c_dev_struct *i2c_dev);

/* transmit amounts of data in master mode, poll transmit process and completed status,
 the function is blocking */
int32_t hal_i2c_master_transmit_poll(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                                uint32_t length, uint32_t timeout_ms);
/* receive amounts of data in master mode, poll receive process and completed status,
the function is blocking */
int32_t hal_i2c_master_receive_poll(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                                    uint32_t length, uint32_t timeout_ms);
/* transmit amounts of data in slave mode, poll transmit process and completed status,
the function is blocking */
int32_t hal_i2c_slave_transmit_poll(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                                    uint32_t length, uint32_t timeout_ms);
/* receive amounts of data in slave mode, poll receive process and completed status,
the function is blocking */
int32_t hal_i2c_slave_receive_poll(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                               uint32_t length, uint32_t timeout_ms);
/* write amounts of data to memory, poll transmit process and completed status,
the function is blocking */
int32_t hal_i2c_memory_write_poll(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                              uint32_t length, uint32_t timeout_ms);
/* read amounts of data from memory, poll transmit process and completed status,
the function is blocking */
int32_t hal_i2c_memory_read_poll(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                              uint32_t length, uint32_t timeout_ms);
/* transmit amounts of data in master mode by interrupt method,the function is non-blocking */
int32_t hal_i2c_master_transmit_interrupt(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* receive amounts of data in master mode by interrupt method,the function is non-blocking */
int32_t hal_i2c_master_receive_interrupt(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer,\
                                        uint32_t length, hal_i2c_user_cb p_user_func);
/* transmit amounts of data in slave mode by interrupt method,the function is non-blocking */
int32_t hal_i2c_slave_transmit_interrupt(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer,\
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* receive amounts of data in slave mode by interrupt method,the function is non-blocking */
int32_t hal_i2c_slave_receive_interrupt(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer,\
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* write amounts of data to memory by interrupt method,the function is non-blocking */
int32_t hal_i2c_memory_write_interrupt(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                              uint32_t length, hal_i2c_user_cb p_user_func);
/* read amounts of data from memory by interrupt method,the function is non-blocking */
int32_t hal_i2c_memory_read_interrupt(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                              uint32_t length, hal_i2c_user_cb p_user_func);
/* transmit amounts of data in master mode by dma method,the function is non-blocking */
int32_t hal_i2c_master_transmit_dma(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                               uint32_t length, hal_i2c_user_cb p_user_func);
/* receive amounts of data in master mode by dma method,the function is non-blocking */
int32_t hal_i2c_master_receive_dma(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                               uint32_t length, hal_i2c_user_cb p_user_func);
/* transmit amounts of data in slave mode by dma method,the function is non-blocking */
int32_t hal_i2c_slave_transmit_dma(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                               uint32_t length, hal_i2c_user_cb p_user_func);
/* receive amounts of data in slave mode by dma method,the function is non-blocking */
int32_t hal_i2c_slave_receive_dma(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                               uint32_t length, hal_i2c_user_cb p_user_func);
/* write amounts of data to memory by dma method,the function is non-blocking */
int32_t hal_i2c_memory_write_dma(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                              uint32_t length, hal_i2c_user_cb p_user_func);
/* read amounts of data from memory by dma method,the function is non-blocking */
int32_t hal_i2c_memory_read_dma(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                              uint32_t length, hal_i2c_user_cb p_user_func);
/* check whether the device is ready for access */
int32_t hal_i2c_device_ready_check(hal_i2c_dev_struct *i2c_dev, uint32_t timeout_ms);
/* serial transmit amounts of data in master mode by interrupt method,the function is non-blocking */
int32_t hal_i2c_master_serial_transmit_interrupt(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* serial receive amounts of data in master mode by interrupt method,the function is non-blocking */
int32_t hal_i2c_master_serial_receive_interrupt(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer, \
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* serial transmit amounts of data in slave mode by interrupt method,the function is non-blocking */
int32_t hal_i2c_slave_serial_transmit_interrupt(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer,\
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* serial receive amounts of data in slave mode by interrupt method,the function is non-blocking */
int32_t hal_i2c_slave_serial_receive_interrupt(hal_i2c_dev_struct *i2c_dev, uint8_t *p_buffer,\
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* enable address listen in slave mode by interrupt method */
int32_t hal_i2c_address_listen_interrupt_enable(hal_i2c_dev_struct *i2c_dev, hal_i2c_user_cb p_user_func);
/* disable address listen in slave mode by interrupt method */
int32_t hal_i2c_address_listen_interrupt_disable(hal_i2c_dev_struct *i2c_dev);

/* configure I2C clock */
void hals_i2c_clock_config(uint32_t i2c_periph, uint32_t clkspeed, uint32_t dutycyc);
/* configure I2C address */
void hals_i2c_mode_addr_config(uint32_t i2c_periph, uint32_t mode, uint32_t addformat, uint32_t addr);
/* whether or not to send an ACK */
void hals_i2c_ack_config(uint32_t i2c_periph, uint32_t ack);
/* configure I2C POAP position */
void hals_i2c_ackpos_config(uint32_t i2c_periph, uint32_t pos);
/* master sends slave address */
void hals_i2c_master_addressing(uint32_t i2c_periph, uint32_t addr, uint32_t trandirection);
/* enable dual-address mode */
void hals_i2c_dualaddr_enable(uint32_t i2c_periph, uint32_t addr);
/* disable dual-address mode */
void hals_i2c_dualaddr_disable(uint32_t i2c_periph);
/* whether to stretch SCL low when data is not ready in slave mode */
void hals_i2c_stretch_scl_low_config(uint32_t i2c_periph, uint32_t stretchpara);
/* whether or not to response to a general call */
void hals_i2c_slave_response_to_gcall_config(uint32_t i2c_periph, uint32_t gcallpara);

/* I2C transmit data function */
void hals_i2c_data_transmit(uint32_t i2c_periph, uint8_t data);
/* I2C receive data function */
uint8_t hals_i2c_data_receive(uint32_t i2c_periph);
/* configure I2C DMA mode */
void hals_i2c_dma_config(uint32_t i2c_periph, uint32_t dmastate);
/* configure whether next DMA EOT is DMA last transfer or not */
void hals_i2c_dma_last_transfer_config(uint32_t i2c_periph, uint32_t dmalast);
/* check I2C flag is set or not */
FlagStatus hals_i2c_flag_get(uint32_t i2c_periph, i2c_flag_enum flag);
/* clear I2C flag status */
void hals_i2c_flag_clear(uint32_t i2c_periph, i2c_flag_enum flag);
/* enable I2C interrupt */
void hals_i2c_interrupt_enable(uint32_t i2c_periph, i2c_interrupt_enum interrupt);
/* disable I2C interrupt */
void hals_i2c_interrupt_disable(uint32_t i2c_periph, i2c_interrupt_enum interrupt);
/* get I2C interrupt flag status */
FlagStatus hals_i2c_interrupt_flag_get(uint32_t i2c_periph, i2c_interrupt_flag_enum int_flag);
/* clear I2C interrupt flag status */
void hals_i2c_interrupt_flag_clear(uint32_t i2c_periph, i2c_interrupt_flag_enum int_flag);

#endif /* GD32F3X0_HAL_I2C_H */
