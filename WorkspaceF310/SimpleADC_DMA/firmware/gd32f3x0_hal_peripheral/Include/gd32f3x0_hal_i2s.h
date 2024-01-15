/*!
    \file    gd32f3x0_hal_i2s.h
    \brief   definitions for the I2S

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

#ifndef GD32F3X0_HAL_I2S_H
#define GD32F3X0_HAL_I2S_H

#include "gd32f3x0_hal.h"

#if (defined(GD32F350) || defined(GD32F310))
/* constants definitions */
/* @PARA: hal_struct_type */
/* @ENUM: I2S structure type enum */
typedef enum {
    HAL_I2S_INIT_STRUCT = 0U,                                                   /*!< I2S initialization structure */
    HAL_I2S_DEV_STRUCT,                                                         /*!< I2S device information structure */
} hal_i2s_struct_type_enum;

/* I2S run state */
typedef enum {
    HAL_I2S_STATE_RESET               = 0x00U,                                   /*!< I2S not yet initialized or disabled */
    HAL_I2S_STATE_READY               = 0x01U,                                   /*!< I2S initialized and ready for use */
    HAL_I2S_STATE_BUSY                = 0x02U,                                   /*!< I2S internal process is ongoing  */
    HAL_I2S_STATE_BUSY_TX             = 0x03U,                                   /*!< data transmission process is ongoing */
    HAL_I2S_STATE_BUSY_RX             = 0x04U,                                   /*!< data reception process is ongoing */
    HAL_I2S_STATE_TIMEOUT             = 0x05U,                                   /*!< I2S timeout state */
    HAL_I2S_STATE_ERROR               = 0x06U                                    /*!< I2S error state */
} hal_i2s_run_state_enum;

/* @STRUCT_MEMBER: standard */
/* @ENUM: I2S standard enumeration */
#define I2SCTL_I2SSTD(regval)        (BITS(4,5) & ((uint32_t)(regval) << 4))
typedef enum {
    I2S_STD_PHILLIPS                  = I2SCTL_I2SSTD(0),                        /*!< I2S phillips standard */
    I2S_STD_MSB                       = I2SCTL_I2SSTD(1),                        /*!< I2S MSB standard */
    I2S_STD_LSB                       = I2SCTL_I2SSTD(2),                        /*!< I2S LSB standard */
    I2S_STD_PCMSHORT                  = I2SCTL_I2SSTD(3),                        /*!< I2S PCM short standard */
    I2S_STD_PCMLONG                   = (I2SCTL_I2SSTD(3) | SPI_I2SCTL_PCMSMOD)  /*!< I2S PCM long standard */
} hal_i2s_standard_enum;

/* @STRUCT_MEMBER: audiosample */
/* @ENUM: I2S audio sample rate enumeration */
typedef enum {
    I2S_AUDIOSAMPLE_8K                = ((uint32_t)8000U),                       /*!< I2S audio sample rate is 8KHz */
    I2S_AUDIOSAMPLE_11K               = ((uint32_t)11025U),                      /*!< I2S audio sample rate is 11KHz */
    I2S_AUDIOSAMPLE_16K               = ((uint32_t)16000U),                      /*!< I2S audio sample rate is 16KHz */
    I2S_AUDIOSAMPLE_22K               = ((uint32_t)22050U),                      /*!< I2S audio sample rate is 22KHz */
    I2S_AUDIOSAMPLE_32K               = ((uint32_t)32000U),                      /*!< I2S audio sample rate is 32KHz */
    I2S_AUDIOSAMPLE_44K               = ((uint32_t)44100U),                      /*!< I2S audio sample rate is 44KHz */
    I2S_AUDIOSAMPLE_48K               = ((uint32_t)48000U),                      /*!< I2S audio sample rate is 48KHz */
    I2S_AUDIOSAMPLE_96K               = ((uint32_t)96000U),                      /*!< I2S audio sample rate is 96KHz */
    I2S_AUDIOSAMPLE_192K              = ((uint32_t)192000U)                      /*!< I2S audio sample rate is 192KHz */
} hal_i2s_audiosample_enum;

/* I2S receive or transmit buffer structure definitions */
typedef struct {
    __IO uint16_t *buffer;                                                      /*!< pointer to I2S transfer buffer*/
    __IO uint16_t length;                                                       /*!< I2S transfer length */
    __IO uint16_t pos;                                                          /*!< I2S transfer position */
} hal_i2s_buffer_struct;

/* I2S device interrupt callback function pointer structure */
typedef struct {
    hal_irq_handle_cb              receive_handler;                             /*!< I2S receive complete callback function */
    hal_irq_handle_cb              transmit_handler;                            /*!< I2S transmit complete callback function */
    hal_irq_handle_cb              error_handle;                                /*!< I2S error complete callback function */
} hal_i2s_irq_struct;

/* @PARA: i2s */
/* @STRUCT: I2S device information structure */
typedef struct {
    uint32_t                       periph;                                      /*!< I2S peripheral */
    hal_i2s_irq_struct             i2s_irq;                                     /*!< I2S device interrupt callback function pointer */
    hal_dma_dev_struct             *p_dma_rx;                                   /*!< DMA receive pointer */
    hal_dma_dev_struct             *p_dma_tx;                                   /*!< DMA transmit pointer */
    hal_i2s_buffer_struct          txbuffer;                                    /*!< transmit buffer */
    hal_i2s_buffer_struct          rxbuffer;                                    /*!< receive buffer */
    void                           *rx_callback;                                /*!< receive callback function pointer */
    void                           *tx_callback;                                /*!< transmit callback function pointer */
    void                           *error_callback;                             /*!< error callback function pointer */
    __IO hal_i2s_run_state_enum    state;                                       /*!< I2S communication state */
    __IO uint32_t                  error_code;                                  /*!< I2S error code*/
    hal_mutex_enum                 mutex;
} hal_i2s_dev_struct;

/* I2S device user callback function pointer */
typedef void (*hal_i2s_user_cb)(hal_i2s_dev_struct *i2s);

/* I2S callback structure */
typedef struct {
    hal_i2s_user_cb complete_func;                                              /*!< I2S user complete callback function */
    hal_i2s_user_cb error_func;                                                 /*!< I2S user error callback function */
} hal_i2s_user_callback_struct;

/* @PARA: p_init */
/* @STRUCT: I2S init structure */
typedef struct {
    uint32_t                 mode;                                               /*!< I2S operation mode */
    hal_i2s_standard_enum    standard;                                           /*!< I2S standard */
    uint32_t                 frameformat;                                        /*!< I2S frame format */
    uint32_t                 mckout;                                             /*!< I2S master clock output */
    hal_i2s_audiosample_enum audiosample;                                        /*!< I2S audio sample rate */
    uint32_t                 ckpl;                                               /*!< I2S idle state clock polarity */
} hal_i2s_init_struct;

/* @STRUCT_MEMBER: mode */
/* @DEFINE: I2S operation mode */
#define I2SCTL_I2SOPMOD(regval)         (BITS(8,9) & ((uint32_t)(regval) << 8))
#define I2S_MODE_SLAVETX                I2SCTL_I2SOPMOD(0)                      /*!< I2S slave transmit mode */
#define I2S_MODE_SLAVERX                I2SCTL_I2SOPMOD(1)                      /*!< I2S slave receive mode */
#define I2S_MODE_MASTERTX               I2SCTL_I2SOPMOD(2)                      /*!< I2S master transmit mode */
#define I2S_MODE_MASTERRX               I2SCTL_I2SOPMOD(3)                      /*!< I2S master receive mode */

/* @STRUCT_MEMBER: frameformat */
/* @DEFINE: I2S frame format */
#define I2SCTL_DTLEN(regval)            (BITS(1,2) & ((uint32_t)(regval) << 1))
#define I2S_FRAMEFORMAT_DT16B_CH16B     I2SCTL_DTLEN(0)                         /*!< I2S data length is 16 bit and channel length is 16 bit */
#define I2S_FRAMEFORMAT_DT16B_CH32B     (I2SCTL_DTLEN(0) | SPI_I2SCTL_CHLEN)    /*!< I2S data length is 16 bit and channel length is 32 bit */
#define I2S_FRAMEFORMAT_DT24B_CH32B     (I2SCTL_DTLEN(1) | SPI_I2SCTL_CHLEN)    /*!< I2S data length is 24 bit and channel length is 32 bit */
#define I2S_FRAMEFORMAT_DT32B_CH32B     (I2SCTL_DTLEN(2) | SPI_I2SCTL_CHLEN)    /*!< I2S data length is 32 bit and channel length is 32 bit */

/* @STRUCT_MEMBER: mckout */
/* @DEFINE: I2S master clock output */
#define I2S_MCKOUT_DISABLE              ((uint32_t)0x00000000U)                 /*!< I2S master clock output disable */
#define I2S_MCKOUT_ENABLE               SPI_I2SPSC_MCKOEN                       /*!< I2S master clock output enable */

/* @STRUCT_MEMBER: ckpl */
/* @DEFINE: I2S clock polarity */
#define I2S_CKPL_LOW                    ((uint32_t)0x00000000U)                 /*!< I2S clock polarity low level */
#define I2S_CKPL_HIGH                   SPI_I2SCTL_CKPL                         /*!< I2S clock polarity high level */

/* I2S error code */
#define HAL_I2S_ERROR_NONE              0x00000000U                             /*!< no error */
#define HAL_I2S_ERROR_TIMEOUT           0x00000001U                             /*!< timeout error */
#define HAL_I2S_ERROR_OVERRUN           0x00000002U                             /*!< overrun error */
#define HAL_I2S_ERROR_UNDERRUN          0x00000004U                             /*!< underrun error */

/* I2S flag definitions */
#define I2S_FLAG_RBNE                   SPI_STAT_RBNE                           /*!< receive buffer not empty flag */
#define I2S_FLAG_TBE                    SPI_STAT_TBE                            /*!< transmit buffer empty flag */
#define I2S_FLAG_CH                     SPI_STAT_I2SCH                          /*!< channel side flag */
#define I2S_FLAG_TXURERR                SPI_STAT_TXURERR                        /*!< underrun error flag */
#define I2S_FLAG_RXORERR                SPI_STAT_RXORERR                        /*!< overrun error flag */
#define I2S_FLAG_TRANS                  SPI_STAT_TRANS                          /*!< transmit on-going flag */
#define I2S_FLAG_FERR                   SPI_STAT_FERR                           /*!< format error interrupt flag */

/* I2S interrupt enable/disable constants definitions */
#define I2S_INT_TBE                     SPI_CTL1_TBEIE                          /*!< transmit buffer empty interrupt */
#define I2S_INT_RBNE                    SPI_CTL1_RBNEIE                         /*!< receive buffer not empty interrupt */
#define I2S_INT_ERR                     SPI_CTL1_ERRIE                          /*!< error interrupt */

/* I2S interrupt flag constants definitions */
#define I2S_INT_FLAG_TBE                ((uint8_t)0x00U)                        /*!< transmit buffer empty interrupt flag */
#define I2S_INT_FLAG_RBNE               ((uint8_t)0x01U)                        /*!< receive buffer not empty interrupt flag */
#define I2S_INT_FLAG_RXORERR            ((uint8_t)0x02U)                        /*!< overrun interrupt flag */
#define I2S_INT_FLAG_TXURERR            ((uint8_t)0x05U)                        /*!< underrun error interrupt flag */
#define I2S_INT_FLAG_FERR               ((uint8_t)0x06U)                        /*!< format error interrupt flag */

/* disable\enable I2S */
#define __HAL_I2S_DISABLE(periph)                        SPI_I2SCTL(periph) = (SPI_I2SCTL(periph) & (~SPI_I2SCTL_I2SEN))
#define __HAL_I2S_ENABLE(periph)                         SPI_I2SCTL(periph) = (SPI_I2SCTL(periph) | (SPI_I2SCTL_I2SEN))

/* disable\enable I2S interrupt function */
#define __HAL_I2S_INT_DISABLE(periph, interrupt)         SPI_CTL1(periph) = (SPI_CTL1(periph) & ~(interrupt))
#define __HAL_I2S_INT_ENABLE(periph, interrupt)          SPI_CTL1(periph) = (SPI_CTL1(periph) | (interrupt))

/* disable\enable I2S DMA function */
#define __HAL_I2S_DMA_DISABLE(periph, dma)               SPI_CTL1(periph) = (SPI_CTL1(periph) & ~(dma))
#define __HAL_I2S_DMA_ENABLE(periph, dma)                SPI_CTL1(periph) = (SPI_CTL1(periph) | (dma))

/* function declarations */
/* I2S deinitialization and initialization functions */
/* deinitialize I2S */
void hal_i2s_deinit(hal_i2s_dev_struct *i2s);
/* @FUNCTION: initialize I2S structure */
void hal_i2s_struct_init(hal_i2s_struct_type_enum hal_struct_type, void *p_struct);
/* @FUNCTION: initialize I2S */
int32_t hal_i2s_init(hal_i2s_dev_struct *i2s, uint32_t periph, hal_i2s_init_struct *p_init);
/* @END */

/* transmit amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_i2s_transmit_poll(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, uint32_t timeout_ms);
/* receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_i2s_receive_poll(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, uint32_t timeout_ms);

/* transmit amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_i2s_transmit_interrupt(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length,
                                   hal_i2s_user_callback_struct *p_user_func);
/* receive amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_i2s_receive_interrupt(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length,
                                  hal_i2s_user_callback_struct *p_user_func);

/* transmit amounts of data by DMA method */
/* the function is non-blocking */
int32_t hal_i2s_transmit_dma(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length,
                             hal_i2s_user_callback_struct *p_user_func);
/* receive amounts of data by DMA method */
/* the function is non-blocking */
int32_t hal_i2s_receive_dma(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length,
                            hal_i2s_user_callback_struct *p_user_func);

/* I2S interrupt handler content function,which is merely used in i2s_handler */
void hal_i2s_irq(hal_i2s_dev_struct *i2s);
/* set user-defined interrupt callback function,
which will be registered and called when corresponding interrupt be triggered */
void hal_i2s_irq_handle_set(hal_i2s_dev_struct *i2s, hal_i2s_irq_struct *p_irq);
/* reset all user-defined interrupt callback function,
which will be registered and called when corresponding interrupt be triggered */
void hal_i2s_irq_handle_all_reset(hal_i2s_dev_struct *i2s);

/* start I2S module function */
void hal_i2s_start(hal_i2s_dev_struct *i2s);
/* stop I2S module function */
void hal_i2s_stop(hal_i2s_dev_struct *i2s);

/* I2S DMA pause function */
void hal_i2s_dma_pause(hal_i2s_dev_struct *i2s);
/* I2S DMA resume function */
void hal_i2s_dma_resume(hal_i2s_dev_struct *i2s);
/* I2S DMA stop function */
void hal_i2s_dma_stop(hal_i2s_dev_struct *i2s);
#endif /* GD32F350 and GD32F310 */

#endif /* GD32F3X0_HAL_I2S_H */
