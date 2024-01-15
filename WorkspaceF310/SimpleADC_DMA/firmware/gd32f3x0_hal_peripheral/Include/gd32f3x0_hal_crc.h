/*!
    \file    gd32f3x0_hal_crc.h
    \brief   definitions for the CRC

    \version 2023-08-01, V1.0.0, HAL firmware for GD32F3x0
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

#ifndef GD32F3X0_HAL_CRC_H
#define GD32F3X0_HAL_CRC_H

#include "gd32f3x0_hal.h"

/* CRC definitions */
#define CRC                            CRC_BASE                    /*!< DMA base address */

/* registers definitions */
#define CRC_DATA                       REG32(CRC + 0x00000000U)            /*!< CRC data register */
#define CRC_FDATA                      REG32(CRC + 0x00000004U)            /*!< CRC free data register */
#define CRC_CTL                        REG32(CRC + 0x00000008U)            /*!< CRC control register */
#define CRC_IDATA                      REG32(CRC + 0x00000010U)            /*!< CRC initialization data register */
#define CRC_POLY                       REG32(CRC + 0x00000014U)            /*!< CRC polynomial register */

/* bits definitions */
/* CRC_DATA */
#define CRC_DATA_DATA                  BITS(0,31)                      /*!< CRC data bits */

/* CRC_FDATA */
#define CRC_FDATA_FDATA                BITS(0,7)                       /*!< CRC free data bits */

/* CRC_CTL */
#define CRC_CTL_RST                    BIT(0)                          /*!< CRC reset bit */
#define CRC_CTL_PS                     BITS(3,4)                       /*!< size of polynomial function bits */
#define CRC_CTL_REV_I                  BITS(5,6)                       /*!< input data reverse function bits */
#define CRC_CTL_REV_O                  BIT(7)                          /*!< output data reverse function bit */

/* CRC_INIT */
#define CRC_IDATA_IDATA                BITS(0,31)                      /*!< CRC initialization data bits */

/* CRC_POLY */
#define CRC_POLY_POLY                  BITS(0,31)                      /*!< CRC polynomial value bits */

/* constants definitions */
/* size of polynomial function */
#define CTL_PS(regval)                 (BITS(3, 4) & ((regval) << 3))
#define CRC_CTL_PS_32                  CTL_PS(0)                       /*!< 32-bit polynomial for CRC calculation */
#define CRC_CTL_PS_16                  CTL_PS(1)                       /*!< 16-bit polynomial for CRC calculation */
#define CRC_CTL_PS_8                   CTL_PS(2)                       /*!< 8-bit polynomial for CRC calculation */
#define CRC_CTL_PS_7                   CTL_PS(3)                       /*!< 7-bit polynomial for CRC calculation */

/* input data format */
#define INPUT_FORMAT_WORD              0U                              /*!< input data in word format */
#define INPUT_FORMAT_HALFWORD          1U                              /*!< input data in half-word format */
#define INPUT_FORMAT_BYTE              2U                              /*!< input data in byte format */

/* input data reverse function */
#define CTL_REV_I(regval)              (BITS(5, 6) & ((regval) << 5))

/* CRC calculation unit */
#define CRC_DEFAULT_POLYNOMIAL_VALUE      (0x04C11DB7U)                     /*!< CRC default polynomial value */
#define CRC_DEFAULT_INIT_DATA_VALUE       (0xFFFFFFFFU)                     /*!< CRC default initialize value */

/* constants definitions */
/* CRC state enum */
typedef enum {
    HAL_CRC_STATE_NONE = 0,                                         /*!< NONE(default value) */
    HAL_CRC_STATE_RESET,                                            /*!< RESET */
    HAL_CRC_STATE_BUSY,                                             /*!< BUSY */
    HAL_CRC_STATE_TIMEOUT,                                          /*!< TIMEOUT */
    HAL_CRC_STATE_ERROR,                                            /*!< ERROR */
    HAL_CRC_STATE_READY,                                            /*!< READY */
} hal_crc_state_enum;

/* CRC structure type enum */
typedef enum {
    HAL_CRC_INIT_STRUCT,                                         /*!< CRC initialization structure */
    HAL_CRC_DEV_STRUCT,                                          /*!< CRC device information structure */
} hal_crc_struct_type_enum;

/* CRC device information structrue */
typedef struct {
    hal_crc_state_enum               state;                     /*!< CRC state */
    hal_mutex_enum                   mutex;                     /*!< CRC mutex */
    void                             *priv;                     /*!< priv data */
} hal_crc_dev_struct;

/* CRC settings */
/* @PARA: p_crc_init */
/* @STRUCT: CRC basic config struct */
typedef struct{
    uint32_t polynomial_value;                                  /*!< polynomial value */
    uint32_t init_data_value;                                   /*!< init data value */
    uint32_t input_data_reverse_mode;                           /*!< input data reverse mode */
    uint32_t output_data_reverse;                               /*!< output data reverse */
    uint32_t polynomial_size;                                   /*!< polynomial size */
}hal_crc_init_struct;


/* @STRUCT_MEMBER: polynomial_value */
/* @=NULL */

/* @STRUCT_MEMBER: init_data_value */
/* @=NULL */

/* @STRUCT_MEMBER: input_data_reverse_mode */
/* @DEFINE: input data reverse mode */
#define CRC_INPUT_REVERSE_NONE              CTL_REV_I(0)                 /*!< input data not reverse */
#define CRC_INPUT_REVERSE_BYTE              CTL_REV_I(1)                 /*!< input data reversed by byte type */
#define CRC_INPUT_REVERSE_HALFWORD          CTL_REV_I(2)                 /*!< input data reversed by half-word type */
#define CRC_INPUT_REVERSE_WORD              CTL_REV_I(3)                 /*!< input data reversed by word type */

/* @STRUCT_MEMBER: output_data_reverse */
/* @DEFINE: output data reverse */
#define CRC_OUTPUT_REVERSE_DISABLE          ((uint32_t)0)               /*!< output data not reverse */
#define CRC_OUTPUT_REVERSE_ENABLE           ((uint32_t)1)               /*!< output data reverse */

/* @STRUCT_MEMBER: polynomial_size */
/* @DEFINE: polynomial size */
#define CRC_POLYNOMIAL_SIZE_32BIT         CRC_CTL_PS_32                     /*!< 32-bit polynomial for CRC calculation */
#define CRC_POLYNOMIAL_SIZE_16BIT         CRC_CTL_PS_16                     /*!< 16-bit polynomial for CRC calculation */
#define CRC_POLYNOMIAL_SIZE_8BIT          CRC_CTL_PS_8                      /*!< 8-bit polynomial for CRC calculation */
#define CRC_POLYNOMIAL_SIZE_7BIT          CRC_CTL_PS_7                      /*!< 7-bit polynomial for CRC calculation */

/* function declarations */
/* initialize the CRC structure with the default values */
int32_t hal_crc_struct_init(hal_crc_struct_type_enum hal_struct_type, void *p_struct);

/* @FUNCTION: initialize CRC */
int32_t hal_crc_init(hal_crc_dev_struct *crc_dev, hal_crc_init_struct *p_crc_init);
/* @END */

/* deinitialize CRC */
int32_t hal_crc_deinit(hal_crc_dev_struct *crc_dev);
/* CRC calculate single data */
uint32_t hal_crc_single_data_calculate(hal_crc_dev_struct *crc_dev, uint32_t sdata, uint8_t data_format);
/* CRC calculate a data array */
uint32_t hal_crc_block_data_calculate(hal_crc_dev_struct *crc_dev, void *array, uint32_t size, uint8_t data_format);
/* enable the reverse operation of output data */
void hals_crc_reverse_output_data_enable(void);
/* disable the reverse operation of output data */
void hals_crc_reverse_output_data_disable(void);

/* reset data register to the value of initializaiton data register */
void hals_crc_data_register_reset(void);
/* read the data register  */
uint32_t hals_crc_data_register_read(void);

/* read the free data register */
uint8_t hals_crc_free_data_register_read(void);
/* write the free data register */
void hals_crc_free_data_register_write(uint8_t free_data);

/* write the initial value register */
void hals_crc_init_data_register_write(uint32_t init_data);
/* configure the CRC input data function */
void hals_crc_input_data_reverse_config(uint32_t data_reverse);

/* configure the CRC size of polynomial function */
void hals_crc_polynomial_size_set(uint32_t poly_size);
/* configure the CRC polynomial value function */
void hals_crc_polynomial_set(uint32_t poly);
/* reset data register */
void hals_crc_calculate_reset(void);

#endif /* GD32F3X0_HAL_CRC_H */
