/*!
    \file    gd32f3x0_hal_i2c_com.h
    \brief   common definitions for the I2C

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

#ifndef GD32F3X0_HAL_I2C_COM_H
#define GD32F3X0_HAL_I2C_COM_H

#include "gd32f3x0_hal.h"

/* I2Cx(x=0,1) definitions */
#define I2C0                          I2C_BASE                                   /*!< I2C0 base address */
#define I2C1                          (I2C_BASE+0x00000400U)                     /*!< I2C1 base address */

/* registers definitions */
#define I2C_CTL0(i2cx)                REG32((i2cx) + 0x00000000U)                /*!< I2C control register 0 */
#define I2C_CTL1(i2cx)                REG32((i2cx) + 0x00000004U)                /*!< I2C control register 1 */
#define I2C_SADDR0(i2cx)              REG32((i2cx) + 0x00000008U)                /*!< I2C slave address register 0*/
#define I2C_SADDR1(i2cx)              REG32((i2cx) + 0x0000000CU)                /*!< I2C slave address register 1 */
#define I2C_DATA(i2cx)                REG32((i2cx) + 0x00000010U)                /*!< I2C transfer buffer register */
#define I2C_STAT0(i2cx)               REG32((i2cx) + 0x00000014U)                /*!< I2C transfer status register 0 */
#define I2C_STAT1(i2cx)               REG32((i2cx) + 0x00000018U)                /*!< I2C transfer status register */
#define I2C_CKCFG(i2cx)               REG32((i2cx) + 0x0000001CU)                /*!< I2C clock configure register */
#define I2C_RT(i2cx)                  REG32((i2cx) + 0x00000020U)                /*!< I2C rise time register */
#define I2C_FMPCFG(i2cx)              REG32((i2cx) + 0x00000090U)                /*!< I2C fast-mode-plus configure register */

/* bits definitions */
/* I2Cx_CTL0 */
#define I2C_CTL0_I2CEN                BIT(0)                                     /*!< peripheral enable */
#define I2C_CTL0_SMBEN                BIT(1)                                     /*!< SMBus mode */
#define I2C_CTL0_SMBSEL               BIT(3)                                     /*!< SMBus type */
#define I2C_CTL0_ARPEN                BIT(4)                                     /*!< ARP enable */
#define I2C_CTL0_PECEN                BIT(5)                                     /*!< PEC enable */
#define I2C_CTL0_GCEN                 BIT(6)                                     /*!< general call enable */
#define I2C_CTL0_SS                   BIT(7)                                     /*!< clock stretching disable (slave mode) */
#define I2C_CTL0_START                BIT(8)                                     /*!< start generation */
#define I2C_CTL0_STOP                 BIT(9)                                     /*!< stop generation */
#define I2C_CTL0_ACKEN                BIT(10)                                    /*!< acknowledge enable */
#define I2C_CTL0_POAP                 BIT(11)                                    /*!< acknowledge/PEC position (for data reception) */
#define I2C_CTL0_PECTRANS             BIT(12)                                    /*!< packet error checking */
#define I2C_CTL0_SALT                 BIT(13)                                    /*!< SMBus alert */
#define I2C_CTL0_SRESET               BIT(15)                                    /*!< software reset */

/* I2Cx_CTL1 */
#define I2C_CTL1_I2CCLK               BITS(0,5)                                  /*!< I2CCLK[5:0] bits (peripheral clock frequency) */
#define I2C_CTL1_ERRIE                BIT(8)                                     /*!< error interrupt enable */
#define I2C_CTL1_EVIE                 BIT(9)                                     /*!< event interrupt enable */
#define I2C_CTL1_BUFIE                BIT(10)                                    /*!< buffer interrupt enable */
#define I2C_CTL1_DMAON                BIT(11)                                    /*!< DMA requests enable */
#define I2C_CTL1_DMALST               BIT(12)                                    /*!< DMA last transfer */

/* I2Cx_SADDR0 */
#define I2C_SADDR0_ADDRESS0           BIT(0)                                     /*!< bit 0 of a 10-bit address */
#define I2C_SADDR0_ADDRESS            BITS(1,7)                                  /*!< 7-bit address or bits 7:1 of a 10-bit address */
#define I2C_SADDR0_ADDRESS_H          BITS(8,9)                                  /*!< highest two bits of a 10-bit address */
#define I2C_SADDR0_ADDFORMAT          BIT(15)                                    /*!< address mode for the I2C slave */

/* I2Cx_SADDR1 */
#define I2C_SADDR1_DUADEN             BIT(0)                                     /*!< aual-address mode switch */
#define I2C_SADDR1_ADDRESS2           BITS(1,7)                                  /*!< second I2C address for the slave in dual-address mode */

/* I2Cx_STAT0 */
#define I2C_STAT0_SBSEND              BIT(0)                                     /*!< start bit (master mode) */
#define I2C_STAT0_ADDSEND             BIT(1)                                     /*!< address sent (master mode)/matched (slave mode) */
#define I2C_STAT0_BTC                 BIT(2)                                     /*!< byte transfer finished */
#define I2C_STAT0_ADD10SEND           BIT(3)                                     /*!< 10-bit header sent (master mode) */
#define I2C_STAT0_STPDET              BIT(4)                                     /*!< stop detection (slave mode) */
#define I2C_STAT0_RBNE                BIT(6)                                     /*!< data register not empty (receivers) */
#define I2C_STAT0_TBE                 BIT(7)                                     /*!< data register empty (transmitters) */
#define I2C_STAT0_BERR                BIT(8)                                     /*!< bus error */
#define I2C_STAT0_LOSTARB             BIT(9)                                     /*!< arbitration lost (master mode) */
#define I2C_STAT0_AERR                BIT(10)                                    /*!< acknowledge failure */
#define I2C_STAT0_OUERR               BIT(11)                                    /*!< overrun/underrun */
#define I2C_STAT0_PECERR              BIT(12)                                    /*!< PEC error in reception */
#define I2C_STAT0_SMBTO               BIT(14)                                    /*!< timeout signal in SMBus mode */
#define I2C_STAT0_SMBALT              BIT(15)                                    /*!< SMBus alert status */

/* I2Cx_STAT1 */
#define I2C_STAT1_MASTER              BIT(0)                                     /*!< master/slave */
#define I2C_STAT1_I2CBSY              BIT(1)                                     /*!< bus busy */
#define I2C_STAT1_TR                  BIT(2)                                     /*!< transmitter/receiver */
#define I2C_STAT1_RXGC                BIT(4)                                     /*!< general call address (slave mode) */
#define I2C_STAT1_DEFSMB              BIT(5)                                     /*!< SMBus device default address (slave mode) */
#define I2C_STAT1_HSTSMB              BIT(6)                                     /*!< SMBus host header (slave mode) */
#define I2C_STAT1_DUMODF              BIT(7)                                     /*!< dual flag (slave mode) */
#define I2C_STAT1_PECV                BITS(8,15)                                 /*!< packet error checking value */

/* I2Cx_CKCFG */
#define I2C_CKCFG_CLKC                BITS(0,11)                                 /*!< clock control register in fast/standard mode or fast mode plus(master mode) */
#define I2C_CKCFG_DTCY                BIT(14)                                    /*!< duty cycle of fast mode or fast mode plus */
#define I2C_CKCFG_FAST                BIT(15)                                    /*!< I2C speed selection in master mode */

/* I2Cx_FMPCFG */
#define I2C_FMPCFG_FMPEN              BIT(0)        /*!< fast mode plus enable bit */

#endif /* GD32F3X0_HAL_I2C_COM_H */
