/*!
    \file    gd32f3x0_hal_spi.h
    \brief   definitions for the SPI

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

#ifndef GD32F3X0_HAL_SPI_H
#define GD32F3X0_HAL_SPI_H

#include "gd32f3x0_hal.h"

/* constants definitions */
/* @PARA: hal_struct_type */
/* @ENUM: SPI structure type enumeration */
typedef enum {
    HAL_SPI_INIT_STRUCT = 0U,                                                     /*!< SPI initialization structure */
    HAL_SPI_DEV_STRUCT,                                                           /*!< SPI device information structure */
} hal_spi_struct_type_enum;

/* SPI run state enumeration */
typedef enum {
    HAL_SPI_STATE_READY             = 0x01U,                                      /*!< peripheral Initialized and ready for use */
    HAL_SPI_STATE_BUSY              = 0x02U,                                      /*!< an internal process is ongoing */
    HAL_SPI_STATE_BUSY_TX           = 0x03U,                                      /*!< data transmission process is ongoing */
    HAL_SPI_STATE_BUSY_RX           = 0x04U,                                      /*!< data reception process is ongoing */
    HAL_SPI_STATE_BUSY_TX_RX        = 0x05U,                                      /*!< data Transmission and Reception process is ongoing */
    HAL_SPI_STATE_ERROR             = 0x06U,                                      /*!< SPI error state */
    HAL_SPI_STATE_ABORT             = 0x07U                                       /*!< SPI abort is ongoing */
} hal_spi_run_state_enum;

/* @STRUCT_MEMBER: trans_mode */
/* @ENUM: SPI transmit type enumeration */
typedef enum {
    SPI_TRANSMODE_FULLDUPLEX      = ((uint32_t)0x00000000U),                                  /*!< SPI receive and send data at fullduplex communication */
    SPI_TRANSMODE_RECEIVEONLY     = SPI_CTL0_RO,                                              /*!< SPI only receive data */
    SPI_TRANSMODE_BDRECEIVE       = SPI_CTL0_BDEN,                                            /*!< bidirectional receive data */
    SPI_TRANSMODE_BDTRANSMIT      = (SPI_CTL0_BDEN | SPI_CTL0_BDOEN),                         /*!< bidirectional transmit data*/
    SPI_TRANSMODE_QUADRECEIVE     = (SPI_QCTL_QMOD | SPI_QCTL_IO23_DRV | SPI_QCTL_QRD),       /*!< quad-wire receive data */
    SPI_TRANSMODE_QUADTRANSMIT    = (SPI_QCTL_QMOD | SPI_QCTL_IO23_DRV)                       /*!< quad-wire transmit data*/
} hal_spi_trans_mode_enum;

/* @STRUCT_MEMBER: prescaler */
/* @ENUM: SPI clock prescaler factor enumeration */
#define CTL0_PSC(regval)          (BITS(3,5) & ((uint32_t)(regval) << 3))
typedef enum {
    SPI_PSC_2                     = CTL0_PSC(0),                             /*!< SPI clock prescaler factor is 2 */
    SPI_PSC_4                     = CTL0_PSC(1),                             /*!< SPI clock prescaler factor is 4 */
    SPI_PSC_8                     = CTL0_PSC(2),                             /*!< SPI clock prescaler factor is 8 */
    SPI_PSC_16                    = CTL0_PSC(3),                             /*!< SPI clock prescaler factor is 16 */
    SPI_PSC_32                    = CTL0_PSC(4),                             /*!< SPI clock prescaler factor is 32 */
    SPI_PSC_64                    = CTL0_PSC(5),                             /*!< SPI clock prescaler factor is 64 */
    SPI_PSC_128                   = CTL0_PSC(6),                             /*!< SPI clock prescaler factor is 128 */
    SPI_PSC_256                   = CTL0_PSC(7)                              /*!< SPI clock prescaler factor is 256 */
} hal_spi_prescaler_enum;

/* SPI device interrupt callback function pointer structure */
typedef struct {
    hal_irq_handle_cb receive_handler;                  /*!< SPI receive complete callback function */
    hal_irq_handle_cb transmit_handler;                 /*!< SPI transmit complete callback function */
    hal_irq_handle_cb transmit_receive_handler;         /*!< SPI transmit and receive complete callback function */
    hal_irq_handle_cb error_handler;                    /*!< SPI error complete callback function */
} hal_spi_irq_struct;

/* SPI receive or transmit buffer struct definitions */
typedef struct {
    __IO uint8_t *buffer;                               /*!< pointer to SPI transfer buffer */
    __IO uint32_t length;                               /*!< SPI transfer length */
    __IO uint32_t pos;                                  /*!< SPI transfer position */
} hal_spi_buffer_struct;

/* @PARA: spi */
/* @STRUCT: SPI device information structure */
typedef struct {
    uint32_t                        periph;             /*!< SPI peripheral */
    hal_spi_irq_struct              spi_irq;            /*!< SPI device interrupt callback function pointer */
    hal_dma_dev_struct              *p_dma_rx;          /*!< DMA receive pointer */
    hal_dma_dev_struct              *p_dma_tx;          /*!< DMA transmit pointer */
    hal_spi_buffer_struct           txbuffer;           /*!< transmit buffer */
    hal_spi_buffer_struct           rxbuffer;           /*!< receive buffer */
    void                            *rx_callback;       /*!< receive callback function pointer */
    void                            *tx_callback;       /*!< transmit callback function pointer */
    void                            *tx_rx_callback;    /*!< transmit and receive callback function pointer */
    void                            *error_callback;    /*!< error callback function pointer */
    __IO hal_spi_run_state_enum     state;              /*!< SPI communication state */
    __IO uint32_t                   error_code;         /*!< SPI error code*/
    hal_mutex_enum                  mutex;
} hal_spi_dev_struct;

/* SPI device user callback function pointer */
typedef void (*hal_spi_user_cb)(hal_spi_dev_struct *spi);
/* SPI callback structure */
typedef struct {
    hal_spi_user_cb complete_func;                      /*!< SPI user complete callback function */
    hal_spi_user_cb error_func;                         /*!< SPI user error callback function */
} hal_spi_user_callback_struct;

/* @PARA: p_init */
/* @STRUCT: SPI initialization structure definitions */
typedef struct {
    uint32_t                 device_mode;                                       /*!< SPI master or slave */
    hal_spi_trans_mode_enum  trans_mode;                                        /*!< SPI transtype */
    uint32_t                 frame_size;                                        /*!< SPI frame size */
    uint32_t                 nss;                                               /*!< SPI NSS control by handware or software */
    uint32_t                 endian;                                            /*!< SPI big endian or little endian */
    uint32_t                 clock_polarity_phase;                              /*!< SPI clock phase and polarity */
    hal_spi_prescaler_enum   prescaler;                                         /*!< SPI prescaler value */
    uint32_t                 crc_calculation;                                   /*!< SPI CRC function selection */
    uint32_t                 crc_poly;                                          /*!< SPI CRC polynomial value */
    uint32_t                 nssp_mode;                                         /*!< SPI NSSP mode selection */
    uint32_t                 ti_mode;                                           /*!< SPI TI mode selection*/
} hal_spi_init_struct;


/* @STRUCT_MEMBER: device_mode */
/* @DEFINE: SPI mode definitions */
#define SPI_MASTER                      (SPI_CTL0_MSTMOD | SPI_CTL0_SWNSS)      /*!< SPI as master */
#define SPI_SLAVE                       ((uint32_t)0x00000000U)                 /*!< SPI as slave */

/* @STRUCT_MEMBER: frame_size */
/* @DEFINE: SPI frame size */
#define SPI_FRAMESIZE_16BIT             SPI_CTL0_FF16                           /*!< SPI frame size is 16 bits */
#define SPI_FRAMESIZE_8BIT              ((uint32_t)0x00000000U)                 /*!< SPI frame size is 8 bits */

/* @STRUCT_MEMBER: nss */
/* @DEFINE: SPI NSS control mode */
#define SPI_NSS_SOFT                    SPI_CTL0_SWNSSEN                        /*!< SPI NSS control by sofrware */
#define SPI_NSS_HARD                    ((uint32_t)0x00000000U)                 /*!< SPI NSS control by hardware */

/* @STRUCT_MEMBER: endian */
/* @DEFINE: SPI transmit way */
#define SPI_ENDIAN_LSB                  SPI_CTL0_LF                             /*!< SPI transmit way is little endian: transmit LSB first */
#define SPI_ENDIAN_MSB                  ((uint32_t)0x00000000U)                 /*!< SPI transmit way is big endian: transmit MSB first */

/* @STRUCT_MEMBER: clock_polarity_phase */
/* @DEFINE: SPI clock phase and polarity */
#define SPI_CK_PL_LOW_PH_1EDGE          ((uint32_t)0x00000000U)                 /*!< SPI clock polarity is low level and phase is first edge */
#define SPI_CK_PL_HIGH_PH_1EDGE         SPI_CTL0_CKPL                           /*!< SPI clock polarity is high level and phase is first edge */
#define SPI_CK_PL_LOW_PH_2EDGE          SPI_CTL0_CKPH                           /*!< SPI clock polarity is low level and phase is second edge */
#define SPI_CK_PL_HIGH_PH_2EDGE         (SPI_CTL0_CKPL | SPI_CTL0_CKPH)         /*!< SPI clock polarity is high level and phase is second edge */

/* @STRUCT_MEMBER: crc_calculation */
/* @DEFINE: CRC function selection */
#define SPI_CRC_ENABLE                  SPI_CTL0_CRCEN                          /*!< SPI CRC enable */
#define SPI_CRC_DISABLE                 ((uint32_t)0x00000000U)                 /*!< SPI CRC disable */

/* @STRUCT_MEMBER: crc_poly */
/* @=NULL */

/* @STRUCT_MEMBER: nssp_mode */
/* @DEFINE: NSSP mode selection */
#define SPI_NSSP_ENABLE                 SPI_CTL1_NSSP                           /*!< SPI NSSP mode disable */
#define SPI_NSSP_DISABLE                ((uint32_t)0x00000000U)                 /*!< SPI NSSP mode enable */

/* @STRUCT_MEMBER: ti_mode */
/* @DEFINE: TI mode selection */
#define SPI_TIMODE_ENABLE               SPI_CTL1_TMOD                           /*!< SPI TI mode enable */
#define SPI_TIMODE_DISABLE              ((uint32_t)0x00000000U)                 /*!< SPI TI mode disable */

/* SPI DMA constants definitions */
#define SPI_DMA_TRANSMIT                SPI_CTL1_DMATEN                         /*!< SPI transmit data use DMA */
#define SPI_DMA_RECEIVE                 SPI_CTL1_DMAREN                         /*!< SPI receive data use DMA */

/* SPI error code */
#define HAL_SPI_ERROR_NONE              0U                                      /*!< no error */
#define HAL_SPI_ERROR_MODF              BIT(0)                                  /*!< mode fault error */
#define HAL_SPI_ERROR_CRC               BIT(1)                                  /*!< CRC error */
#define HAL_SPI_ERROR_OVR               BIT(2)                                  /*!< overrun error */
#define HAL_SPI_ERROR_FRE               BIT(3)                                  /*!< frame error */
#define HAL_SPI_ERROR_DMA               BIT(4)                                  /*!< DMA transfer error */
#define HAL_SPI_ERROR_ABORT             BIT(6)                                  /*!< error during SPI abort procedure */
#define HAL_SPI_ERROR_TIMEOUT           BIT(7)                                  /*!< SPI timeout error */

/* SPI flag definitions */
#define SPI_FLAG_RBNE                   SPI_STAT_RBNE                           /*!< receive buffer not empty flag */
#define SPI_FLAG_TBE                    SPI_STAT_TBE                            /*!< transmit buffer empty flag */
#define SPI_FLAG_CRCERR                 SPI_STAT_CRCERR                         /*!< CRC error flag */
#define SPI_FLAG_CONFERR                SPI_STAT_CONFERR                        /*!< mode config error flag */
#define SPI_FLAG_RXORERR                SPI_STAT_RXORERR                        /*!< receive overrun error flag */
#define SPI_FLAG_TRANS                  SPI_STAT_TRANS                          /*!< transmit on-going flag */
#define SPI_FLAG_FERR                   SPI_STAT_FERR                           /*!< format error flag */

/* SPI/I2S interrupt enable/disable constants definitions */
#define SPI_INT_TBE                     SPI_CTL1_TBEIE                          /*!< transmit buffer empty interrupt */
#define SPI_INT_RBNE                    SPI_CTL1_RBNEIE                         /*!< receive buffer not empty interrupt */
#define SPI_INT_ERR                     SPI_CTL1_ERRIE                          /*!< error interrupt */

/* clear SPI error flag */
#define SPI_ERROR_FLAG_CONF             (0x00000000U)                           /*!< SPI configuration error */
#define SPI_ERROR_FLAG_RXORERR          (0x00000001U)                           /*!< SPI reception overrun error */
#define SPI_ERROR_FLAG_FERR             (0x00000002U)                           /*!< SPI format error */
#define SPI_ERROR_FLAG_CRCERR           (0x00000003U)                           /*!< SPI CRC error */

/* SPI interrupt flag constants definitions */
#define SPI_INT_FLAG_TBE                ((uint8_t)0x00U)                        /*!< transmit buffer empty interrupt flag */
#define SPI_INT_FLAG_RBNE               ((uint8_t)0x01U)                        /*!< receive buffer not empty interrupt flag */
#define SPI_INT_FLAG_RXORERR            ((uint8_t)0x02U)                        /*!< overrun interrupt flag */
#define SPI_INT_FLAG_CONFERR            ((uint8_t)0x03U)                        /*!< config error interrupt flag */
#define SPI_INT_FLAG_CRCERR             ((uint8_t)0x04U)                        /*!< CRC error interrupt flag */
#define SPI_INT_FLAG_FERR               ((uint8_t)0x06U)                        /*!< format error interrupt flag */

/* get SPI init value */
#define __HAL_SPI_GET_DEVICE_MODE(periph)                ((SPI_CTL0(periph)) & ((SPI_CTL0_MSTMOD) | (SPI_CTL0_SWNSS)))
#define __HAL_SPI_GET_FRAME_SIZE(periph)                 ((SPI_CTL0(periph)) & (SPI_CTL0_FF16))
#define __HAL_SPI_GET_CRC_USED(periph)                   ((SPI_CTL0(periph)) & (SPI_CTL0_CRCEN))
#define __HAL_SPI_GET_TRANS_MODE(periph)                 ((SPI_CTL0(periph)) & ((SPI_CTL0_BDEN | SPI_CTL0_BDOEN | SPI_CTL0_RO)))

/* disable\enable SPI */
#define __HAL_SPI_DISABLE(periph)                        SPI_CTL0(periph) = (SPI_CTL0(periph) & (~SPI_CTL0_SPIEN))
#define __HAL_SPI_ENABLE(periph)                         SPI_CTL0(periph) = (SPI_CTL0(periph) | (SPI_CTL0_SPIEN))

/* disable\enable SPI CRC function */
#define __HAL_SPI_CRC_OFF(periph)                        SPI_CTL0(periph) = (SPI_CTL0(periph) & (~SPI_CTL0_CRCEN))
#define __HAL_SPI_CRC_ON(periph)                         SPI_CTL0(periph) = (SPI_CTL0(periph) | (SPI_CTL0_CRCEN))
/* next data is CRC value */
#define __HAL_SPI_CRC_NEXT(periph)                       SPI_CTL0(periph) = (SPI_CTL0(periph) | (SPI_CTL0_CRCNT))

/* disable\enable SPI interrupt function */
#define __HAL_SPI_INT_DISABLE(periph, interrupt)         SPI_CTL1(periph) = (SPI_CTL1(periph) & ~(interrupt))
#define __HAL_SPI_INT_ENABLE(periph, interrupt)          SPI_CTL1(periph) = (SPI_CTL1(periph) | (interrupt))

/* disable\enable SPI DMA function */
#define __HAL_SPI_DMA_DISABLE(periph, dma)               SPI_CTL1(periph) = (SPI_CTL1(periph) & ~(dma))
#define __HAL_SPI_DMA_ENABLE(periph, dma)                SPI_CTL1(periph) = (SPI_CTL1(periph) | (dma))

/* function declarations */
/* SPI deinitialization and initialization functions */
/* deinitialize SPI */
void hal_spi_deinit(hal_spi_dev_struct *spi);
/* initialize SPI structure */
void hal_spi_struct_init(hal_spi_struct_type_enum hal_struct_type, void *p_struct);
/* @FUNCTION: initialize SPI */
int32_t hal_spi_init(hal_spi_dev_struct *spi, uint32_t periph, hal_spi_init_struct *p_init);
/* @END */

/* transmit amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_spi_transmit_poll(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint32_t length, uint32_t timeout_ms);
/* receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_spi_receive_poll(hal_spi_dev_struct *spi, uint8_t *p_rxbuffer, uint32_t length, uint32_t timeout_ms);
/* transmit and receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_spi_transmit_receive_poll(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint8_t *p_rxbuffer,
                                      uint32_t length, uint32_t timeout_ms);

/* transmit amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_spi_transmit_interrupt(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint32_t length,
                                   hal_spi_user_callback_struct *p_user_func);
/* receive amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_spi_receive_interrupt(hal_spi_dev_struct *spi, uint8_t *p_rxbuffer, uint32_t length,
                                  hal_spi_user_callback_struct *p_user_func);
/* transmit and receive amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_spi_transmit_receive_interrupt(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint8_t *p_rxbuffer,
        uint32_t length, hal_spi_user_callback_struct *p_user_func);

/* transmit amounts of data by DMA method */
/* the function is non-blocking */
int32_t hal_spi_transmit_dma(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint16_t length,
                             hal_spi_user_callback_struct *p_user_func);
/* receive amounts of data by DMA method */
/* the function is non-blocking */
int32_t hal_spi_receive_dma(hal_spi_dev_struct *spi, uint8_t *p_rxbuffer, uint16_t length,
                            hal_spi_user_callback_struct *p_user_func);
/* transmit and receive amounts of data by DMA method */
/* the function is non-blocking */
int32_t hal_spi_transmit_receive_dma(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint8_t *p_rxbuffer, uint16_t length,
                                     hal_spi_user_callback_struct *p_user_func);

/* SPI interrupt handler content function,which is merely used in spi_handler */
void hal_spi_irq(hal_spi_dev_struct *spi);
/* set user-defined interrupt callback function,
which will be registered and called when corresponding interrupt be triggered */
void hal_spi_irq_handle_set(hal_spi_dev_struct *spi, hal_spi_irq_struct *p_irq);
/* reset all user-defined interrupt callback function,
which will be registered and called when corresponding interrupt be triggered */
void hal_spi_irq_handle_all_reset(hal_spi_dev_struct *spi);

/* start SPI module function */
void hal_spi_start(hal_spi_dev_struct *spi);
/* stop SPI module function */
void hal_spi_stop(hal_spi_dev_struct *spi);

/* SPI abort function */
int32_t hal_spi_abort(hal_spi_dev_struct *spi);

/* SPI DMA pause function */
void hal_spi_dma_pause(hal_spi_dev_struct *spi);
/* SPI DMA resume function */
void hal_spi_dma_resume(hal_spi_dev_struct *spi);
/* SPI DMA stop function */
void hal_spi_dma_stop(hal_spi_dev_struct *spi);

#endif /* GD32F3X0_HAL_SPI_H */
