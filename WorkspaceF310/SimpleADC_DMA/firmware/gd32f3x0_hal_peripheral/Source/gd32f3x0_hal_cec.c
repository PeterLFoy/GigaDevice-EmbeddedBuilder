/*!
    \file    gd32f3x0_hal_cec.c
    \brief   CEC driver

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
#include "gd32f3x0_hal.h"

/* CEC handler in transmit mode */
static void _cec_transmit_interrupt(void *cec);
/* CEC handler in receive mode */
static void _cec_receive_interrupt(void *cec);
/* CEC enable interrupt */
static void _cec_interrupt_enable(uint32_t flag);
/* CEC disable interrupt */
static void _cec_interrupt_disable(uint32_t flag);
/* CEC get int flag and status */
static FlagStatus _cec_interrupt_flag_get(uint32_t flag);
/* CEC clear int flag and status */
static void _cec_interrupt_flag_clear(uint32_t flag);

/*!
    \brief      initialize the CEC structure with the default values
    \param[in]  hal_struct_type: the argument could be selected from enumeration <hal_cec_struct_type_enum>
    \param[in]  p_struct: pointer to CEC structure that contains the configuration information
    \param[out] none
    \retval     none
*/
void hal_cec_struct_init(hal_cec_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer [*p_struct] value is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_CEC_INIT_STRUCT:
        /* initialize CEC initialization structure with the default values */
        ((hal_cec_init_struct *)p_struct)->signal_free_time                        = CEC_SFT_PROTOCOL_PERIOD;
        ((hal_cec_init_struct *)p_struct)->reception_bit_timing_tolerance          = CEC_STANTARD_RTOL;
        ((hal_cec_init_struct *)p_struct)->sft_start_option_bit                    = CEC_SFT_START_STAOM;
        ((hal_cec_init_struct *)p_struct)->listen_mode                             = CEC_PARTIAL_LISTENING_MODE;
        ((hal_cec_init_struct *)p_struct)->own_address                             = CEC_OWN_ADDRESS0;
        ((hal_cec_init_struct *)p_struct)->bre_stop_receive                        = CEC_NOT_STOP_RECEPTION;
        ((hal_cec_init_struct *)p_struct)->bre_generate_error                      = CEC_NO_RISING_PERIOD_ERROR;
        ((hal_cec_init_struct *)p_struct)->blpe_generate_error                     = CEC_NO_LONG_PERIOD_ERROR;
        ((hal_cec_init_struct *)p_struct)->not_generate_error_broadcast            = CEC_GEN_BROADCAST_ERROR;
        break;
    case HAL_CEC_IRQ_STRUCT:
        /* initialize CEC IRQ structure with the default values */
        ((hal_cec_irq_struct *)p_struct)->cec_tx_handle                            = NULL;
        ((hal_cec_irq_struct *)p_struct)->cec_rx_handle                            = NULL;
        break;
    case HAL_CEC_DEV_STRUCT:
        /* initialize CEC device structure with the default values */
        ((hal_cec_dev_struct *)p_struct)->cec_irq.cec_tx_handle                    = NULL;
        ((hal_cec_dev_struct *)p_struct)->cec_irq.cec_rx_handle                    = NULL;
        ((hal_cec_dev_struct *)p_struct)->state                                    = HAL_CEC_STATE_NONE;
        ((hal_cec_dev_struct *)p_struct)->tx_buffer                                = NULL;
        ((hal_cec_dev_struct *)p_struct)->tx_count                                 = 0;
        ((hal_cec_dev_struct *)p_struct)->rx_buffer                                = NULL;
        ((hal_cec_dev_struct *)p_struct)->rx_count                                 = 0;
        ((hal_cec_dev_struct *)p_struct)->rx_buffer                                = NULL;
        ((hal_cec_dev_struct *)p_struct)->tx_callback                              = NULL;
        ((hal_cec_dev_struct *)p_struct)->rx_callback                              = NULL;
        ((hal_cec_dev_struct *)p_struct)->err_callback                             = NULL;
        ((hal_cec_dev_struct *)p_struct)->mutex                                    = HAL_MUTEX_UNLOCKED;
        ((hal_cec_dev_struct *)p_struct)->priv                                     = NULL;
        break;
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      initialize CEC
    \param[in]  cec_dev: CEC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  cec: the pointer of CEC init structure
                  signal_free_time:
                  only one parameter can be selected which is shown as below:
      \arg          CEC_SFT_PROTOCOL_PERIOD: the signal free time will perform as HDMI-CEC protocol description
      \arg          CEC_SFT_1POINT5_PERIOD: 1.5 nominal data bit periods
      \arg          CEC_SFT_2POINT5_PERIOD: 2.5 nominal data bit periods
      \arg          CEC_SFT_3POINT5_PERIOD: 3.5 nominal data bit periods
      \arg          CEC_SFT_4POINT5_PERIOD: 4.5 nominal data bit periods
      \arg          CEC_SFT_5POINT5_PERIOD: 5.5 nominal data bit periods
      \arg          CEC_SFT_6POINT5_PERIOD: 6.5 nominal data bit periods
      \arg          CEC_SFT_7POINT5_PERIOD: 7.5 nominal data bit periods
                  reception_bit_timing_tolerance:
                  only one parameter can be selected which is shown as below:
      \arg          CEC_STANTARD_RTOL: Extended bit timing tolerance
      \arg          CEC_EXTENDED_RTOL: Standard bit timing tolerance
                  sft_start_option_bit:
                  only one parameter can be selected which is shown as below:
      \arg          CEC_SFT_START_STAOM: signal free time counter starts counting when STAOM is asserted
      \arg          CEC_SFT_START_LAST: signal free time counter starts automatically after transmission/reception end
                  listen_mode:
                  only one parameter can be selected which is shown as below:
      \arg          CEC_PARTIAL_LISTENING_MODE: Only receive broadcast and singlecast in OAD address with appropriate ACK
      \arg          CEC_WHOLE_LISTENING_MODE: Receive broadcast and singlecast in OAD address with appropriate ACK and receive
                                              message whose destination address is not in OAD without feedback ACK
                  address:
                  only one parameter can be selected which is shown as below:
      \arg          CEC_OWN_ADDRESS0: own address is 0
      \arg          CEC_OWN_ADDRESS1: own address is 1
      \arg          CEC_OWN_ADDRESS2: own address is 2
      \arg          CEC_OWN_ADDRESS3: own address is 3
      \arg          CEC_OWN_ADDRESS4: own address is 4
      \arg          CEC_OWN_ADDRESS5: own address is 5
      \arg          CEC_OWN_ADDRESS6: own address is 6
      \arg          CEC_OWN_ADDRESS7: own address is 7
      \arg          CEC_OWN_ADDRESS8: own address is 8
      \arg          CEC_OWN_ADDRESS9: own address is 9
      \arg          CEC_OWN_ADDRESS10: own address is 10
      \arg          CEC_OWN_ADDRESS11: own address is 11
      \arg          CEC_OWN_ADDRESS12: own address is 12
      \arg          CEC_OWN_ADDRESS13: own address is 13
      \arg          CEC_OWN_ADDRESS14: own address is 14
                  bre_stop_receive:
                  only one parameter can be selected which is shown as below:
      \arg          CEC_NOT_STOP_RECEPTION: do not stop reception when detected bit rising error
      \arg          CEC_STOP_RECEPTION: stop reception when detected bit rising error
                  bre_generate_error:
                  only one parameter can be selected which is shown as below:
      \arg          CEC_NO_RISING_PERIOD_ERROR: do not generate Error-bit on bit rising error
      \arg          CEC_GEN_RISING_PERIOD_ERROR: generate Error-bit on bit rising error
                  blpe_generate_error:
                  only one parameter can be selected which is shown as below:
      \arg          CEC_NO_LONG_PERIOD_ERROR: do not generate Error-bit on long bit period error
      \arg          CEC_GEN_LONG_PERIOD_ERROR: generate Error-bit on long bit period error
                  not_generate_error_broadcast:
                  only one parameter can be selected which is shown as below:
      \arg          CEC_GEN_BROADCAST_ERROR: generate Error-bit in broadcast
      \arg          CEC_NO_BROADCAST_ERROR: do not generate Error-bit in broadcast
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_cec_init(hal_cec_dev_struct *cec_dev, hal_cec_init_struct *cec)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == cec_dev) || (NULL == cec)) {
        HAL_DEBUGE("pointer [*cec_dev] or pointer [*cec] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    cec_dev->state = HAL_CEC_STATE_BUSY;

    /* disable cec */
    CEC_CTL &= ~CEC_CTL_CECEN;
    /* configure cec */
    CEC_CFG = cec->blpe_generate_error | cec->bre_generate_error | cec->bre_stop_receive |  \
              cec->listen_mode | cec->not_generate_error_broadcast | cec->own_address |     \
              cec->reception_bit_timing_tolerance | cec->sft_start_option_bit | cec->signal_free_time;

    /* change CEC error state */
    cec_dev->error_state = HAL_CEC_ERROR_NONE;
    /* change CEC state */
    cec_dev->state = HAL_CEC_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}


/*!
    \brief      deinitialize CEC device structure
    \param[in]  cec_dev: CEC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_cec_deinit(hal_cec_dev_struct *cec_dev)
{

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cec_dev) {
        HAL_DEBUGE("pointer [cec_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* deactivate CEC interrupts associated to the set of TX and RX flags */
    _cec_interrupt_disable(CEC_INTEN_RENDIE | CEC_INTEN_BRIE | CEC_INTEN_TENDIE | CEC_INTEN_TBRIE);

    /* deactivate CEC interrupts associated to the set of TX and RX error */
    _cec_interrupt_disable(CEC_INTEN_ROIE | CEC_INTEN_BREIE | CEC_INTEN_BPSEIE | \
                           CEC_INTEN_BPLEIE | CEC_INTEN_RAEIE | CEC_INTEN_ARBFIE | \
                           CEC_INTEN_TUIE | CEC_INTEN_TERRIE | CEC_INTEN_TAERRIE);

    cec_dev->state = HAL_CEC_STATE_BUSY;

    /* change CEC error state and state */
    cec_dev->error_state = HAL_CEC_ERROR_NONE;
    cec_dev->state = HAL_CEC_STATE_RESET;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      start CEC module function
    \param[in]  cec_dev: CEC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_cec_start(hal_cec_dev_struct *cec_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cec_dev) {
        HAL_DEBUGE("pointer [cec_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    cec_dev->state = HAL_CEC_STATE_BUSY;

    /* enable cec */
    CEC_CTL |= CEC_CTL_CECEN;

    cec_dev->state = HAL_CEC_STATE_READY;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      stop CEC module function
    \param[in]  cec_dev: CEC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_cec_stop(hal_cec_dev_struct *cec_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cec_dev) {
        HAL_DEBUGE("pointer [cec_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* disable cec */
    CEC_CTL &= ~CEC_CTL_CECEN;

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by interrupt method
    \param[in]  cec_dev: CEC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  tx_length: length of data to be sent
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  header_addr: header address
    \param[in]  destination_addr: destination address
    \param[in]  p_func: CEC transmit interrupt callback function structure
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_cec_transmit_interrupt(hal_cec_dev_struct *cec_dev, uint32_t tx_length, uint8_t *p_buffer, \
                                   uint8_t header_addr, uint8_t destination_addr, hal_cec_user_cb p_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == cec_dev)) {
        HAL_DEBUGE("pointer [cec_dev] or pointer [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    if(cec_dev->state == HAL_CEC_STATE_READY) {
        /* lock CEC */
        HAL_LOCK(cec_dev);

        /* disable cec */
        CEC_CTL &= ~CEC_CTL_CECEN;

        cec_dev->tx_count = tx_length;

        cec_dev->tx_buffer = p_buffer;

        cec_dev->state = HAL_CEC_STATE_BUSY;
        cec_dev->cec_irq.cec_tx_handle = _cec_transmit_interrupt;

        cec_dev->tx_callback = p_func;

        /* clear the specified CEC interrupt flag */
        _cec_interrupt_flag_clear(CEC_INT_FLAG_TEND | CEC_INT_FLAG_TBR);
        /* activate CEC interrupts associated to the set of TX and RX flags */
        _cec_interrupt_enable(CEC_INTEN_TENDIE | CEC_INTEN_TBRIE);

        /* enable cec */
        CEC_CTL |= CEC_CTL_CECEN;


        /* (length = 0) Set TX End of Message (ENDOM) bit, must be set before writing data to TDATA */
        if(tx_length == 0U) {
            CEC_CTL |= CEC_CTL_ENDOM;
        }

        /* send header block */
        CEC_TDATA = ((uint8_t)(header_addr << CEC_HEADER_ADDR_POS) | (uint8_t) destination_addr);
        /* Set TX Start of Message (STAOM) bit */
        CEC_CTL |= CEC_CTL_STAOM;

        /* unlock CEC */
        HAL_UNLOCK(cec_dev);
    }

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by interrupt method
    \param[in]  cec_dev: CEC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  p_func: CEC transmit interrupt callback function structure
    \param[out] none
    \retval     HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32f3x0_hal.h
*/
int32_t hal_cec_receive_interrupt(hal_cec_dev_struct *cec_dev, uint8_t *p_buffer, hal_cec_user_cb p_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == cec_dev)) {
        HAL_DEBUGE("pointer [cec_dev] or pointer [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    if(cec_dev->state == HAL_CEC_STATE_READY) {
        /* lock CEC */
        HAL_LOCK(cec_dev);

        cec_dev->state = HAL_CEC_STATE_BUSY;
        cec_dev->cec_irq.cec_rx_handle = _cec_receive_interrupt;

        cec_dev->rx_callback = p_func;

        cec_dev->rx_buffer = p_buffer;

        _cec_interrupt_flag_clear(CEC_INT_FLAG_BR | CEC_INT_FLAG_REND | CEC_INT_FLAG_ARBF | \
                                  CEC_INT_FLAG_TU | CEC_INT_FLAG_TERR | CEC_INT_FLAG_TAERR | \
                                  CEC_INT_FLAG_RO | CEC_INT_FLAG_BRE | CEC_INT_FLAG_BPSE | \
                                  CEC_INT_FLAG_BPLE | CEC_INT_FLAG_RAE);

        /* activate CEC interrupts associated to the set of RX flags */
        _cec_interrupt_enable(CEC_INTEN_RENDIE | CEC_INTEN_BRIE);

        /* activate CEC interrupts associated to the set of TX and RX error */
        _cec_interrupt_enable(CEC_INTEN_ROIE | CEC_INTEN_BREIE | CEC_INTEN_BPSEIE | \
                              CEC_INTEN_BPLEIE | CEC_INTEN_RAEIE | CEC_INTEN_ARBFIE | \
                              CEC_INTEN_TUIE | CEC_INTEN_TERRIE | CEC_INTEN_TAERRIE);

        /* unlock CEC */
        HAL_UNLOCK(cec_dev);
    }

    /* return function state */
    return HAL_ERR_NONE;
}

/*!
    \brief      set user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  cec_dev: CEC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to CEC interrupt callback functions structure
                  The structure member can be assigned as following parameters:
      \arg        hal_irq_handle_cb: the function is user-defined, the corresponding callback
                    mechanism is in use, and enable corresponding interrupt
      \arg        NULL: The corresponding callback mechanism is out of use, and disable
                    corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_cec_irq_handle_set(hal_cec_dev_struct *cec_dev, hal_cec_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == cec_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [cec_dev] or pointer [p_irq] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* CEC tx interrupt handler set */
    if(NULL != p_irq->cec_tx_handle) {
        cec_dev->cec_irq.cec_tx_handle = p_irq->cec_tx_handle;
        _cec_interrupt_enable(CEC_INTEN_TENDIE | CEC_INTEN_TBRIE);
    } else {
        cec_dev->cec_irq.cec_tx_handle = NULL;
        _cec_interrupt_disable(CEC_INTEN_TENDIE | CEC_INTEN_TBRIE);
    }

    /* CEC rx interrupt handler set */
    if(NULL != p_irq->cec_rx_handle) {
        cec_dev->cec_irq.cec_rx_handle = p_irq->cec_rx_handle;
        _cec_interrupt_enable(CEC_INTEN_RENDIE | CEC_INTEN_BRIE);
    } else {
        cec_dev->cec_irq.cec_rx_handle = NULL;
        _cec_interrupt_disable(CEC_INTEN_RENDIE | CEC_INTEN_BRIE);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  cec_dev: CEC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_cec_irq_handle_all_reset(hal_cec_dev_struct *cec_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cec_dev) {
        HAL_DEBUGE("pointer [cec_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* CEC interrupt handler reset */
    cec_dev->cec_irq.cec_tx_handle = NULL;
    cec_dev->cec_irq.cec_rx_handle = NULL;
}

/*!
    \brief      CEC interrupt handler content function, which is merely used in cec_handler
    \param[in]  cec_dev: CEC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_cec_irq(hal_cec_dev_struct *cec_dev)
{
    hal_cec_dev_struct *p_cec = cec_dev;
    hal_cec_user_cb perr_func = (hal_cec_user_cb)p_cec->err_callback;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cec_dev) {
        HAL_DEBUGE("pointer [cec_dev] address is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* check if a reception error occured */
    if(_cec_interrupt_flag_get(CEC_INT_FLAG_RO)) {
        _cec_interrupt_flag_clear(CEC_INT_FLAG_RO);
        p_cec->error_state = HAL_CEC_ERROR_RO;
        p_cec->state = HAL_CEC_STATE_READY;
        p_cec->rx_count = 0U;
    }

    /* check if a bit rising error occured */
    if(_cec_interrupt_flag_get(CEC_INT_FLAG_BRE)) {
        _cec_interrupt_flag_clear(CEC_INT_FLAG_BRE);
        p_cec->error_state = HAL_CEC_ERROR_BRE;
        p_cec->state = HAL_CEC_STATE_READY;
        p_cec->rx_count = 0U;
    }

    /* check if a short bit period error occured */
    if(_cec_interrupt_flag_get(CEC_INT_FLAG_BPSE)) {
        _cec_interrupt_flag_clear(CEC_INT_FLAG_BPSE);
        p_cec->error_state = HAL_CEC_ERROR_BPSE;
        p_cec->state = HAL_CEC_STATE_READY;
        p_cec->rx_count = 0U;
    }

    /* check if a long bit period error occured */
    if(_cec_interrupt_flag_get(CEC_INT_FLAG_BPLE)) {
        _cec_interrupt_flag_clear(CEC_INT_FLAG_BPLE);
        p_cec->error_state = HAL_CEC_ERROR_BPLE;
        p_cec->state = HAL_CEC_STATE_READY;
        p_cec->rx_count = 0U;
    }

    /* check if a Rx ACK error occured */
    if(_cec_interrupt_flag_get(CEC_INT_FLAG_RAE)) {
        _cec_interrupt_flag_clear(CEC_INT_FLAG_RAE);
        p_cec->error_state = HAL_CEC_ERROR_RAE;
        p_cec->state = HAL_CEC_STATE_READY;
        p_cec->rx_count = 0U;
    }

    /* check if a arbitration fail error occurred */
    if(_cec_interrupt_flag_get(CEC_INT_FLAG_ARBF)) {
        _cec_interrupt_flag_clear(CEC_INT_FLAG_ARBF);
        p_cec->error_state = HAL_CEC_ERROR_ARBF;
    } else {
        /* check if a Tx data buffer underrun error occurred */
        if(_cec_interrupt_flag_get(CEC_INT_FLAG_TU)) {
            _cec_interrupt_flag_clear(CEC_INT_FLAG_TU);
            p_cec->error_state = HAL_CEC_ERROR_TU;
        }

        /* check if a Tx error occurred */
        if(_cec_interrupt_flag_get(CEC_INT_FLAG_TERR)) {
            _cec_interrupt_flag_clear(CEC_INT_FLAG_TERR);
            p_cec->error_state = HAL_CEC_ERROR_TERR;
        }

        /* check if a Tx ACK error occurred */
        if(_cec_interrupt_flag_get(CEC_INT_FLAG_TAERR)) {
            _cec_interrupt_flag_clear(CEC_INT_FLAG_TAERR);
            p_cec->error_state = HAL_CEC_ERROR_TAERR;
            p_cec->rx_count = 0U;
        }

        p_cec->state = HAL_CEC_STATE_READY;
    }

    if(cec_dev->cec_irq.cec_rx_handle != NULL) {
        cec_dev->cec_irq.cec_rx_handle(cec_dev);
    }

    if(cec_dev->cec_irq.cec_tx_handle != NULL) {
        cec_dev->cec_irq.cec_tx_handle(cec_dev);
    }

    if(p_cec->error_state != HAL_CEC_ERROR_NONE) {
        if(NULL != perr_func) {
            perr_func(p_cec);
        }
    }
}

/*!
    \brief      handler in CEC transmit mode
    \param[in]  cec: CEC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _cec_transmit_interrupt(void *cec)
{
    static uint8_t send_inc = 0U;
    uint32_t length = 0;

    hal_cec_dev_struct *p_cec = cec;
    hal_cec_user_cb ptx_func = (hal_cec_user_cb)p_cec->tx_callback;

    length = p_cec->tx_count;

    /* check if end of message bit is set in the data to be transmitted */
    if(_cec_interrupt_flag_get(CEC_INT_FLAG_TEND)) {
        _cec_interrupt_flag_clear(CEC_INT_FLAG_TEND | CEC_INT_FLAG_TBR);
        p_cec->state = HAL_CEC_STATE_READY;
        p_cec->error_state = HAL_CEC_ERROR_NONE;
        send_inc = 0U;

        if(NULL != ptx_func) {
            ptx_func(p_cec);
        }
    } else if(_cec_interrupt_flag_get(CEC_INT_FLAG_TBR)) {
        /* set EOM bit if the byte to be transmitted is the last one of the senddata */
        if(send_inc++ == (length - 1U)) {
            CEC_CTL |= CEC_CTL_ENDOM;
            CEC_TDATA = (*p_cec->tx_buffer++);
        } else {
            /* send the byte in the transdata */
            CEC_TDATA = (*p_cec->tx_buffer++);
        }
        p_cec->state = HAL_CEC_STATE_BUSY_TX;
        _cec_interrupt_flag_clear(CEC_INT_FLAG_TBR);
    }
}

/*!
    \brief      handler in CEC receive mode
    \param[in]  cec: CEC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _cec_receive_interrupt(void *cec)
{
    hal_cec_dev_struct *p_cec = cec;
    hal_cec_user_cb prx_func = (hal_cec_user_cb)p_cec->rx_callback;

    /* receive data */
    if(_cec_interrupt_flag_get(CEC_INT_FLAG_BR)) {
        p_cec->state = HAL_CEC_STATE_BUSY_RX;
        p_cec->rx_buffer[p_cec->rx_count] = (uint8_t)CEC_RDATA;
        p_cec->rx_count++;
        _cec_interrupt_flag_clear(CEC_INT_FLAG_BR);
    }
    /* check if the byte received is the last one of the message */
    if(_cec_interrupt_flag_get(CEC_INT_FLAG_REND)) {
        p_cec->state = HAL_CEC_STATE_READY;
        p_cec->error_state = HAL_CEC_ERROR_NONE;
        p_cec->rx_count = 0U;
        _cec_interrupt_flag_clear(CEC_INT_FLAG_REND);

        if(NULL != prx_func) {
            prx_func(p_cec);
        }
    }
}

/*!
    \brief      enable interrupt
    \param[in]  flag: specify which flag
                one or more parameters can be selected which are shown as below:
      \arg        CEC_INT_BR: enable Rx-byte data received interrupt
      \arg        CEC_INT_REND: enable end of reception interrupt
      \arg        CEC_INT_RO: enable RX overrun interrupt
      \arg        CEC_INT_BRE: enable bit rising error interrupt
      \arg        CEC_INT_BPSE: enable short bit period error interrupt
      \arg        CEC_INT_BPLE: enable long bit period error interrupt
      \arg        CEC_INT_RAE: enable Rx ACK error interrupt
      \arg        CEC_INT_ARBF: enable arbitration lost interrupt
      \arg        CEC_INT_TBR: enable Tx-byte data request interrupt
      \arg        CEC_INT_TEND: enable transmission successfully end interrupt
      \arg        CEC_INT_TU: enable Tx data buffer underrun interrupt
      \arg        CEC_INT_TERR: enable Tx-error interrupt
      \arg        CEC_INT_TAERR: enable Tx ACK error interrupt
    \param[out] none
    \retval     none
*/
static void _cec_interrupt_enable(uint32_t flag)
{
    CEC_INTEN |= flag;
}

/*!
    \brief      disable interrupt
    \param[in]  flag: specify which flag
                one or more parameters can be selected which are shown as below:
      \arg        CEC_INT_BR: disable Rx-byte data received interrupt
      \arg        CEC_INT_REND: disable end of reception interrupt
      \arg        CEC_INT_RO: disable RX overrun interrupt
      \arg        CEC_INT_BRE: disable bit rising error interrupt
      \arg        CEC_INT_BPSE: disable short bit period error interrupt
      \arg        CEC_INT_BPLE: disable long bit period error interrupt
      \arg        CEC_INT_RAE: disable Rx ACK error interrupt
      \arg        CEC_INT_ARBF: disable arbitration lost interrupt
      \arg        CEC_INT_TBR: disable Tx-byte data request interrupt
      \arg        CEC_INT_TEND: disable transmission successfully end interrupt
      \arg        CEC_INT_TU: disable Tx data buffer underrun interrupt
      \arg        CEC_INT_TERR: disable Tx-error interrupt
      \arg        CEC_INT_TAERR: disable Tx ACK error  interrupt

    \param[out] none
    \retval     none
*/
static void _cec_interrupt_disable(uint32_t flag)
{
    CEC_INTEN &= ~flag;
}

/*!
    \brief      get CEC int flag and status
    \param[in]  flag:  specify which flag
                one or more parameters can be selected which are shown as below:
      \arg        CEC_INT_FLAG_BR: Rx-byte data received
      \arg        CEC_INT_FLAG_REND: end of reception
      \arg        CEC_INT_FLAG_RO: RX overrun
      \arg        CEC_INT_FLAG_BRE: bit rising error
      \arg        CEC_INT_FLAG_BPSE: short bit period error
      \arg        CEC_INT_FLAG_BPLE: long bit period error
      \arg        CEC_INT_FLAG_RAE: Rx ACK error
      \arg        CEC_INT_FLAG_ARBF: arbitration lost
      \arg        CEC_INT_FLAG_TBR: Tx-byte data request
      \arg        CEC_INT_FLAG_TEND: transmission successfully end
      \arg        CEC_INT_FLAG_TU: Tx data buffer underrun
      \arg        CEC_INT_FLAG_TERR: Tx-error
      \arg        CEC_INT_FLAG_TAERR: Tx ACK error flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
static FlagStatus _cec_interrupt_flag_get(uint32_t flag)
{
    uint32_t interrupt_enable = 0U, interrupt_flag = 0U;
    interrupt_flag = (CEC_INTF & flag);
    interrupt_enable = (CEC_INTEN & flag);
    if(interrupt_flag && interrupt_enable) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear CEC int flag and status
    \param[in]  flag:  specify which flag
                one or more parameters can be selected which are shown as below:
      \arg        CEC_INT_FLAG_BR: Rx-byte data received
      \arg        CEC_INT_FLAG_REND: end of reception
      \arg        CEC_INT_FLAG_RO: RX overrun
      \arg        CEC_INT_FLAG_BRE: bit rising error
      \arg        CEC_INT_FLAG_BPSE: short bit period error
      \arg        CEC_INT_FLAG_BPLE: long bit period error
      \arg        CEC_INT_FLAG_RAE: Rx ACK error
      \arg        CEC_INT_FLAG_ARBF: arbitration lost
      \arg        CEC_INT_FLAG_TBR: Tx-byte data request
      \arg        CEC_INT_FLAG_TEND: transmission successfully end
      \arg        CEC_INT_FLAG_TU: Tx data buffer underrun
      \arg        CEC_INT_FLAG_TERR: Tx-error
      \arg        CEC_INT_FLAG_TAERR: Tx ACK error flag
    \param[out] none
    \retval     none
*/
static void _cec_interrupt_flag_clear(uint32_t flag)
{
    CEC_INTF = flag;
}

#endif /* GD32F350 */
