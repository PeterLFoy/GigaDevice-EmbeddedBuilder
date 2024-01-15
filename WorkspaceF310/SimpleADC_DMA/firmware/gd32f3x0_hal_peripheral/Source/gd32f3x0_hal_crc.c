/*!
    \file    gd32f3x0_hal_crc.c
    \brief   CRC driver

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

#include "gd32f3x0_hal.h"

/*!
    \brief      initialize the CRC structure with the default values
    \param[in]  p_struct: point to the structure to be deinitialized
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd323x0_hal.h
*/
int32_t hal_crc_struct_init(hal_crc_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check p_crc_init address */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer [p_struct] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    switch(hal_struct_type) {
    case HAL_CRC_INIT_STRUCT:
        ((hal_crc_init_struct *)p_struct)->input_data_reverse_mode    = CRC_INPUT_REVERSE_NONE;
        ((hal_crc_init_struct *)p_struct)->output_data_reverse        = DISABLE;
        ((hal_crc_init_struct *)p_struct)->polynomial_size            = CRC_POLYNOMIAL_SIZE_32BIT;
        ((hal_crc_init_struct *)p_struct)->polynomial_value           = CRC_DEFAULT_POLYNOMIAL_VALUE;
        ((hal_crc_init_struct *)p_struct)->init_data_value            = CRC_DEFAULT_INIT_DATA_VALUE;
        break;
    case HAL_CRC_DEV_STRUCT:
        ((hal_crc_dev_struct *)p_struct)->state             = HAL_CRC_STATE_NONE;
        ((hal_crc_dev_struct *)p_struct)->mutex             = HAL_MUTEX_UNLOCKED;
        ((hal_crc_dev_struct *)p_struct)->priv              = NULL;
        break;
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize CRC
    \param[in]  crc_dev: CRC device information structrue
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_crc_init: specify input data reverse function
      \arg        output_reverse: ENABLE / DISABLE
      \arg        input_reverse: CRC_INPUT_REVERSE_NONE, CRC_INPUT_REVERSE_BYTE
                                 CRC_INPUT_REVERSE_HALFWORD, CRC_INPUT_REVERSE_WORD
      \arg        polynomial_size: CRC_POLYNOMIAL_SIZE_32BIT, CRC_POLYNOMIAL_SIZE_16BIT
                                   CRC_POLYNOMIAL_SIZE_8BIT, CRC_POLYNOMIAL_SIZE_7BIT
      \arg        polynomial: CRC_DEFAULT_POLYNOMIAL_VALUE(if don't care this parameter)
      \arg        initdata: CRC_DEFAULT_INIT_DATA_VALUE(if don't care this parameter)
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_crc_init(hal_crc_dev_struct *crc_dev, hal_crc_init_struct *p_crc_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check p_crc_init address */
    if(NULL == crc_dev || NULL == p_crc_init) {
        HAL_DEBUGE("pointer [*p_crc_init] or pointer [*crc_dev] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    crc_dev->state = HAL_CRC_STATE_BUSY;
    /* reset crc data register */
    hals_crc_data_register_reset();

    CRC_IDATA = p_crc_init->init_data_value;
    CRC_POLY = p_crc_init->polynomial_value;

    /* enable the reverse operation of output data */
    if(ENABLE == p_crc_init->output_data_reverse) {
        CRC_CTL |= (uint32_t)CRC_CTL_REV_O;
    } else {
        CRC_CTL &= (uint32_t)(~ CRC_CTL_REV_O);
    }

    /* configure the CRC input data function */
    hals_crc_input_data_reverse_config(p_crc_init->input_data_reverse_mode);
    /* configure the CRC size of polynomial function */
    hals_crc_polynomial_size_set(p_crc_init->polynomial_size);
    /* change CRC state */
    crc_dev->state = HAL_CRC_STATE_READY;
    /* return function state */
    return HAL_ERR_NONE;
}



/*!
    \brief      deinitialize CRC
    \param[in]  crc_dev: CRC device information structrue
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
int32_t hal_crc_deinit(hal_crc_dev_struct *crc_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check p_crc_init address */
    if(NULL == crc_dev) {
        HAL_DEBUGE("pointer [*crc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    /* check CRV state */
    if(HAL_CRC_STATE_BUSY == crc_dev->state) {
        return HAL_ERR_BUSY;
    }
    /* Set CRC state */
    crc_dev->state = HAL_CRC_STATE_BUSY;

    CRC_IDATA = (uint32_t)0xFFFFFFFFU;
    CRC_DATA  = (uint32_t)0xFFFFFFFFU;
    CRC_FDATA = (uint32_t)0x00000000U;
    CRC_POLY  = (uint32_t)0x04C11DB7U;
    CRC_CTL   = (uint32_t)CRC_CTL_RST;

    /* Set CRC state */
    crc_dev->state = HAL_CRC_STATE_RESET;

    /* unlock CRC */
    HAL_UNLOCK(crc_dev);

    return HAL_ERR_NONE;
}

/*!
    \brief      CRC calculate single data
    \param[in]  crc_dev: CRC device information structrue
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  sdata: specify input data data
    \param[in]  data_format: input data format
                only one parameter can be selected which is shown as below:
      \arg        INPUT_FORMAT_WORD: input data in word format
      \arg        INPUT_FORMAT_HALFWORD: input data in half-word format
      \arg        INPUT_FORMAT_BYTE: input data in byte format
    \param[out] none
    \retval     CRC calculate value
*/
uint32_t hal_crc_single_data_calculate(hal_crc_dev_struct *crc_dev, uint32_t sdata, uint8_t data_format)
{
    /* Set CRC state */
    crc_dev->state = HAL_CRC_STATE_BUSY;

    if(INPUT_FORMAT_WORD == data_format) {
        REG32(CRC) = sdata;
    } else if(INPUT_FORMAT_HALFWORD == data_format) {
        REG16(CRC) = (uint16_t)sdata;
    } else {
        REG8(CRC) = (uint8_t)sdata;
    }
    /* Set CRC state */
    crc_dev->state = HAL_CRC_STATE_READY;
    return(CRC_DATA);
}

/*!
    \brief      CRC calculate a data array
    \param[in]  crc_dev: CRC device information structrue
                  the structure is not necessary to be reconfigured after structure initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  array: pointer to the input data array
    \param[in]  size: size of the array
    \param[in]  data_format: input data format
                only one parameter can be selected which is shown as below:
      \arg        INPUT_FORMAT_WORD: input data in word format
      \arg        INPUT_FORMAT_HALFWORD: input data in half-word format
      \arg        INPUT_FORMAT_BYTE: input data in byte format
    \param[out] none
    \retval     CRC calculate value
*/
uint32_t hal_crc_block_data_calculate(hal_crc_dev_struct *crc_dev, void *array, uint32_t size, uint8_t data_format)
{
    uint8_t *data8;
    uint16_t *data16;
    uint32_t *data32;
    uint32_t index;

    /* Set CRC state */
    crc_dev->state = HAL_CRC_STATE_BUSY;

    if(INPUT_FORMAT_WORD == data_format) {
        data32 = (uint32_t *)array;
        for(index = 0U; index < size; index++) {
            REG32(CRC) = data32[index];
        }
    } else if(INPUT_FORMAT_HALFWORD == data_format) {
        data16 = (uint16_t *)array;
        for(index = 0U; index < size; index++) {
            REG16(CRC) = data16[index];
        }
    } else {
        data8 = (uint8_t *)array;
        for(index = 0U; index < size; index++) {
            REG8(CRC) =  data8[index];
        }
    }
    /* Set CRC state */
    crc_dev->state = HAL_CRC_STATE_READY;

    return (CRC_DATA);
}

/*!
    \brief      enable the reverse operation of output data
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_crc_reverse_output_data_enable(void)
{
    CRC_CTL &= (uint32_t)(~ CRC_CTL_REV_O);
    CRC_CTL |= (uint32_t)CRC_CTL_REV_O;
}

/*!
    \brief      disable the reverse operation of output data
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_crc_reverse_output_data_disable(void)
{
    CRC_CTL &= (uint32_t)(~ CRC_CTL_REV_O);
}

/*!
    \brief      reset data register to the value of initializaiton data register
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_crc_data_register_reset(void)
{
    CRC_CTL |= (uint32_t)CRC_CTL_RST;
}

/*!
    \brief      read the data register
    \param[in]  none
    \param[out] none
    \retval     32-bit value of the data register
*/
uint32_t hals_crc_data_register_read(void)
{
    uint32_t data;
    data = CRC_DATA;
    return (data);
}

/*!
    \brief      read the free data register
    \param[in]  none
    \param[out] none
    \retval     8-bit value of the free data register
*/
uint8_t hals_crc_free_data_read(void)
{
    uint8_t fdata;
    fdata = (uint8_t)CRC_FDATA;
    return (fdata);
}

/*!
    \brief      write the free data register
    \param[in]  data: specify 8-bit data
    \param[out] none
    \retval     none
*/
void hals_crc_free_data_write(uint8_t data)
{
    CRC_FDATA = (uint32_t)data;
}

/*!
    \brief      write the initializaiton data register
    \param[in]  init_data:specify 32-bit data
    \param[out] none
    \retval     none
*/
void hals_crc_init_data_register_write(uint32_t init_data)
{
    CRC_IDATA = (uint32_t)init_data;
}

/*!
    \brief      configure the CRC input data function
    \param[in]  data_reverse: specify input data reverse function
                only one parameter can be selected which is shown as below:
      \arg        CRC_INPUT_DATA_NOT: input data is not reversed
      \arg        CRC_INPUT_DATA_BYTE: input data is reversed on 8 bits
      \arg        CRC_INPUT_DATA_HALFWORD: input data is reversed on 16 bits
      \arg        CRC_INPUT_DATA_WORD: input data is reversed on 32 bits
    \param[out] none
    \retval     none
*/
void hals_crc_input_data_reverse_config(uint32_t data_reverse)
{
    CRC_CTL &= (uint32_t)(~CRC_CTL_REV_I);
    CRC_CTL |= (uint32_t)data_reverse;
}

/*!
    \brief      configure the CRC size of polynomial function
    \param[in]  poly_size: size of polynomial
                only one parameter can be selected which is shown as below:
      \arg        CRC_CTL_PS_32: 32-bit polynomial for CRC calculation
      \arg        CRC_CTL_PS_16: 16-bit polynomial for CRC calculation
      \arg        CRC_CTL_PS_8: 8-bit polynomial for CRC calculation
      \arg        CRC_CTL_PS_7: 7-bit polynomial for CRC calculation
    \param[out] none
    \retval     none
*/
void hals_crc_polynomial_size_set(uint32_t poly_size)
{
    CRC_CTL &= (uint32_t)(~(CRC_CTL_PS));
    CRC_CTL |= (uint32_t)poly_size;
}

/*!
    \brief      configure the CRC polynomial value function
    \param[in]  poly: configurable polynomial value
    \param[out] none
    \retval     none
*/
void hals_crc_polynomial_set(uint32_t poly)
{
    CRC_POLY &= (uint32_t)(~CRC_POLY_POLY);
    CRC_POLY = poly;
}

/*!
    \brief      reset data register to the value of initializaiton data register
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hals_crc_calculate_reset(void)
{
    CRC_CTL |= (uint32_t)CRC_CTL_RST;
}
