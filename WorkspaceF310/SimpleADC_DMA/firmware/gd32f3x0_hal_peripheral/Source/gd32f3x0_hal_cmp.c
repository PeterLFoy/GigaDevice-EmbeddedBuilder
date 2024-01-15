/*!
    \file    gd32f3x0_hal_cmp.c
    \brief   CMP driver

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
    \brief      initialize CMP
    \param[in]  cmp_dev: CMP device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_init: the pointer to CMP init structure
                  mode:
                  only one parameter can be selected which is shown as below:
      \arg          CMP_MODE_HIGHSPEED: high speed and power mode
      \arg          CMP_MODE_MIDDLESPEED: middle speed and power mode
      \arg          CMP_MODE_LOWSPEED: low speed and power mode
      \arg          CMP_MODE_VERYLOWSPEED: very low speed and power mode
                  polarity:
                  only one parameter can be selected which is shown as below:
      \arg          CMP_OUTPUT_POLARITY_NONINVERTED: CMP output not inverted
      \arg          CMP_OUTPUT_POLARITY_INVERTED: CMP output inverted
                  hysteresis:
                  only one parameter can be selected which is shown as below:
      \arg          CMP_HYSTERESIS_NO: input no hysteresis
      \arg          CMP_HYSTERESIS_LOW: input hysteresis level low
      \arg          CMP_HYSTERESIS_MIDDLE: input hysteresis level middle
      \arg          CMP_HYSTERESIS_HIGH: input hysteresis level high
                  inverting_input:
                  only one parameter can be selected which is shown as below:
      \arg          CMP_INVERTING_INPUT_1_4VREFINT: inverting input 1/4 Vrefint
      \arg          CMP_INVERTING_INPUT_1_2VREFINT: inverting input 1/2 Vrefint
      \arg          CMP_INVERTING_INPUT_3_4VREFINT: inverting input 3/4 Vrefint
      \arg          CMP_INVERTING_INPUT_VREFINT: inverting input Vrefint
      \arg          CMP_INVERTING_INPUT_PA4: inverting input PA4(DAC0_OUT0)
      \arg          CMP_INVERTING_INPUT_PA5: inverting input PA5
      \arg          CMP_INVERTING_INPUT_PA0_PA2: inverting input PA0 for CMP0 or PA2 for CMP1
                  noninverting_input:
                    the argument could be selected from enumeration <hal_cmp_noninverting_input_enum>
                  outputsel:
                  only one parameter can be selected which is shown as below:
      \arg          CMP_OUTPUT_NONE: output none
      \arg          CMP_OUTPUT_TIMER0_BKIN: output TIMER0 break input
      \arg          CMP_OUTPUT_TIMER0_IC0: output TIMER0_CH0 input capture
      \arg          CMP_OUTPUT_TIMER0_OCPRECLR: output TIMER0 OCPRE_CLR input
      \arg          CMP_OUTPUT_TIMER1_IC3: output TIMER1_CH3 input capture
      \arg          CMP_OUTPUT_TIMER1_OCPRECLR: output TIMER1 OCPRE_CLR input
      \arg          CMP_OUTPUT_TIMER2_IC0: output TIMER2_CH0 input capture
      \arg          CMP_OUTPUT_TIMER2_OCPRECLR: output TIMER2 OCPRE_CLR input
                  exti_type:
                    the argument could be selected from enumeration <hal_cmp_exti_type_enum>
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK, details refer to gd32f3x0_hal.h
*/
int32_t hal_cmp_init(hal_cmp_dev_struct *cmp_dev, uint32_t periph, hal_cmp_init_struct *p_init)
{
    uint32_t temp_val;
    uint32_t bit_shift;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == cmp_dev) || (NULL == p_init)) {
        HAL_DEBUGE("pointer address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(!IS_CMP_PERIPHERAL(cmp_dev->periph)) {
        HAL_DEBUGE("CMP peripheral is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* initialize mutex to unlocked  */
    cmp_dev->mutex = HAL_MUTEX_UNLOCKED;

    /* initialize cmp_dev periph */
    cmp_dev->periph = periph;

    /* initialize the CMP */
    if(HAL_CMP_STATE_RESET == cmp_dev->state) {
        /* calculate bit shift of CMP0 or CMP1 */
        if(CMP0 == cmp_dev->periph) {
            bit_shift = 0U;
        } else {
            bit_shift = 16U;
        }

        /* check if CMP locked */
        if(CMP_CS & (CMP_CS_CMP0LK << bit_shift)) {
            return HAL_ERR_LOCK;
        }

        /* clear bits */
        temp_val = CMP_CS;
        temp_val &= ~((uint32_t)0x0000FFFFU << bit_shift);

        /* configure the operating mode, output selection, inverting_input and hysteresis */
        temp_val |= (uint32_t)(p_init->mode | \
                               p_init->inverting_input | \
                               p_init->outputsel | \
                               p_init->hysteresis) << bit_shift;

        /* configure the output polarity */
        if(CMP_OUTPUT_POLARITY_INVERTED == p_init->polarity) {
            temp_val |= (uint32_t)CMP_CS_CMP0PL << bit_shift;
        } else {
            temp_val &= ~((uint32_t)CMP_CS_CMP0PL << bit_shift);
        }

        /* configure the noninverting input port */
        if(CMP0 == cmp_dev->periph) {
            if(CMP0_IP_PA1 == p_init->noninverting_input) {
                /* disable switch close */
                temp_val &= ~CMP_CS_CMP0SW;
            } else if(CMP0_IP_PA4_SWCLOSE == p_init->noninverting_input) {
                /* enable switch close */
                temp_val |= CMP_CS_CMP0SW;
            } else {
            }
        } else {
            if(CMP1_IP_PA3 == p_init->noninverting_input) {
                /* disable window mode */
                temp_val &= ~CMP_CS_WNDEN;
            } else if(CMP1_IP_OF_CMP0 == p_init->noninverting_input) {
                /* enable window mode */
                temp_val |= CMP_CS_WNDEN;
            } else {
            }
        }

        /* write register */
        CMP_CS = temp_val;

        /* configure EXTI type: falling, rising or both */
        if(CMP0 == cmp_dev->periph) {
            hal_exti_internal_init((hal_exti_internal_line_enum)EXTI_CMP_OUTPUT_21, (hal_exti_type_enum)p_init->exti_type);
            /* disable CMP EXTI interrupt until calling hal_cmp_start_interrupt */
            EXTI_INTEN &= ~EXTI_CMP_OUTPUT_21;
        } else {
            hal_exti_internal_init((hal_exti_internal_line_enum)EXTI_CMP_OUTPUT_22, (hal_exti_type_enum)p_init->exti_type);
            /* disable CMP EXTI interrupt until calling hal_cmp_start_interrupt */
            EXTI_INTEN &= ~EXTI_CMP_OUTPUT_22;
        }

        /* change CMP state */
        cmp_dev->state = HAL_CMP_STATE_READY;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      initialize the CMP structure with the default values
    \param[in]  hal_struct_type: the type of the structure
    \param[in]  p_struct: the pointer of the structure
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL,
                details refer to gd32f3x0_hal.h
*/
int32_t hal_cmp_struct_init(hal_cmp_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct) {
        HAL_DEBUGE("pointer address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(hal_struct_type) {
    case HAL_CMP_INIT_STRUCT:
        /* initialize CMP initialization structure with the default values */
        ((hal_cmp_init_struct *)p_struct)->noninverting_input = CMP0_IP_PA1;
        ((hal_cmp_init_struct *)p_struct)->inverting_input   = CMP_INVERTING_INPUT_1_4VREFINT;
        ((hal_cmp_init_struct *)p_struct)->mode              = CMP_MODE_HIGHSPEED;
        ((hal_cmp_init_struct *)p_struct)->hysteresis        = CMP_HYSTERESIS_NO;
        ((hal_cmp_init_struct *)p_struct)->outputsel         = CMP_OUTPUT_NONE;
        ((hal_cmp_init_struct *)p_struct)->polarity          = CMP_OUTPUT_POLARITY_NONINVERTED;
        break;

    case HAL_CMP_DEV_STRUCT:
        /* initialize CMP device information structure with the default values */
        ((hal_cmp_dev_struct *)p_struct)->periph             = CMP0;
        ((hal_cmp_dev_struct *)p_struct)->state              = HAL_CMP_STATE_RESET;
        ((hal_cmp_dev_struct *)p_struct)->output_level       = CMP_OUTPUTLEVEL_LOW;
        ((hal_cmp_dev_struct *)p_struct)->cmp_irq.output_changed_handle = NULL;
        break;
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        return HAL_ERR_VAL;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      deinitialize CMP
    \param[in]  cmp_dev: CMP device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_LOCK,
                details refer to gd32f3x0_hal.h
*/
int32_t hal_cmp_deinit(hal_cmp_dev_struct *cmp_dev)
{
    uint32_t bit_shift;
    int32_t tmp_hal_err = HAL_ERR_NONE;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp_dev) {
        HAL_DEBUGE("pointer [cmp_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(!IS_CMP_PERIPHERAL(cmp_dev->periph)) {
        HAL_DEBUGE("CMP peripheral is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    /* change CMP state */
    cmp_dev->state = HAL_CMP_STATE_BUSY;

    /* calculate bit shift of CMP0 or CMP1 */
    if(CMP0 == cmp_dev->periph) {
        bit_shift = 0U;
    } else {
        bit_shift = 16U;
    }

    /* deinitialize the CMP */
    /* check if CMP locked */
    if(CMP_CS & (CMP_CS_CMP0LK << bit_shift)) {
        tmp_hal_err = HAL_ERR_LOCK;
        /* change CMP state */
        cmp_dev->state = HAL_CMP_STATE_READY;
    } else {
        /* deinitialize CMP register */
        CMP_CS &= ~((uint32_t)0x0000FFFFU << bit_shift);
        /* change CMP state */
        cmp_dev->state = HAL_CMP_STATE_RESET;
    }

    /* deinit EXTI type: falling, rising or both */
    if(CMP0 == cmp_dev->periph) {
        hal_exti_internal_deinit((hal_exti_internal_line_enum)EXTI_CMP_OUTPUT_21);
    } else {
        hal_exti_internal_deinit((hal_exti_internal_line_enum)EXTI_CMP_OUTPUT_22);
    }

    /* release lock */
    HAL_UNLOCK(cmp_dev);

    return tmp_hal_err;
}

/*!
    \brief      start CMP module function
    \param[in]  cmp_dev: CMP device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK,
                details refer to gd32f3x0_hal.h
*/
int32_t hal_cmp_start(hal_cmp_dev_struct *cmp_dev)
{
    uint32_t bit_shift;
    int32_t tmp_hal_err = HAL_ERR_NONE;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp_dev) {
        HAL_DEBUGE("pointer [cmp_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(!IS_CMP_PERIPHERAL(cmp_dev->periph)) {
        HAL_DEBUGE("CMP peripheral is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* process locked */
    HAL_LOCK(cmp_dev);

    /* change CMP state */
    cmp_dev->state = HAL_CMP_STATE_BUSY;

    /* calculate bit shift of CMP0 or CMP1 */
    if(CMP0 == cmp_dev->periph) {
        bit_shift = 0U;
    } else {
        bit_shift = 16U;
    }

    /* check if CMP locked */
    if(CMP_CS & (CMP_CS_CMP0LK << bit_shift)) {
        tmp_hal_err = HAL_ERR_LOCK;
    } else {
        /* enable CMP module */
        CMP_CS |= CMP_CS_CMP0EN << bit_shift;
    }

    /* change CMP state */
    cmp_dev->state = HAL_CMP_STATE_READY;

    /* process unlocked */
    HAL_UNLOCK(cmp_dev);

    return tmp_hal_err;
}

/*!
    \brief      stop cmp module function
    \param[in]  cmp_dev: CMP device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK,
                details refer to gd32f3x0_hal.h
*/
int32_t hal_cmp_stop(hal_cmp_dev_struct *cmp_dev)
{
    uint32_t bit_shift;
    int32_t tmp_hal_err = HAL_ERR_NONE;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp_dev) {
        HAL_DEBUGE("pointer [cmp_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(!IS_CMP_PERIPHERAL(cmp_dev->periph)) {
        HAL_DEBUGE("CMP peripheral is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    /* calculate bit shift of CMP0 or CMP1 */
    if(CMP0 == cmp_dev->periph) {
        bit_shift = 0U;
    } else {
        bit_shift = 16U;
    }

    /* check if CMP locked */
    if(CMP_CS & (CMP_CS_CMP0LK << bit_shift)) {
        tmp_hal_err = HAL_ERR_LOCK;
    } else {
        /* disable CMP module */
        CMP_CS &= ~(CMP_CS_CMP0EN << bit_shift);
    }

    /* change CMP state */
    cmp_dev->state = HAL_CMP_STATE_READY;

    return tmp_hal_err;
}

/*!
    \brief      start CMP module function in interrupt mode
    \param[in]  cmp_dev: CMP device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK,
                details refer to gd32f3x0_hal.h
*/
int32_t hal_cmp_start_interrupt(hal_cmp_dev_struct *cmp_dev, hal_cmp_irq_struct *p_irq)
{
    uint32_t bit_shift;
    int32_t tmp_hal_err = HAL_ERR_NONE;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == cmp_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [cmp_dev] or pointer [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(!IS_CMP_PERIPHERAL(cmp_dev->periph)) {
        HAL_DEBUGE("CMP peripheral is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* process locked */
    HAL_LOCK(cmp_dev);

    /* change CMP state */
    cmp_dev->state = HAL_CMP_STATE_BUSY;

    /* configure interrupt callback function */
    cmp_dev->cmp_irq.output_changed_handle = p_irq->output_changed_handle;

    /* start cmp module */
    /* calculate bit shift of CMP0 or CMP1 */
    if(CMP0 == cmp_dev->periph) {
        bit_shift = 0U;
    } else {
        bit_shift = 16U;
    }

    /* check if CMP locked */
    if(CMP_CS & (CMP_CS_CMP0LK << bit_shift)) {
        tmp_hal_err = HAL_ERR_LOCK;
    } else {
        /* enable CMP module */
        CMP_CS |= CMP_CS_CMP0EN << bit_shift;
    }

    /* enable CMP EXTI line interrupt */
    /* configure EXTI trigger type: falling, rising or both edge */
    if(CMP0 == cmp_dev->periph) {
        /* clear CMP EXTI interrupt flag */
        hals_exti_interrupt_flag_clear(EXTI_CMP_OUTPUT_21);
        EXTI_INTEN |= EXTI_CMP_OUTPUT_21;
    } else {
        /* clear CMP EXTI interrupt flag */
        hals_exti_interrupt_flag_clear(EXTI_CMP_OUTPUT_22);
        EXTI_INTEN |= EXTI_CMP_OUTPUT_22;
    }

    /* change CMP state */
    cmp_dev->state = HAL_CMP_STATE_READY;

    /* process unlocked */
    HAL_UNLOCK(cmp_dev);

    return tmp_hal_err;
}

/*!
    \brief      stop CMP module function in interrupt mode
    \param[in]  cmp_dev: CMP device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK, HAL_ERR_VAL,
                details refer to gd32f3x0_hal.h
*/
int32_t hal_cmp_stop_interrupt(hal_cmp_dev_struct *cmp_dev)
{
    uint32_t bit_shift;
    int32_t tmp_hal_err = HAL_ERR_NONE;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp_dev) {
        HAL_DEBUGE("pointer [cmp_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(!IS_CMP_PERIPHERAL(cmp_dev->periph)) {
        HAL_DEBUGE("CMP peripheral is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* reset interrupt callback function */
    cmp_dev->cmp_irq.output_changed_handle = NULL;

    /* stop cmp module */
    /* calculate bit shift of CMP0 or CMP1 */
    if(CMP0 == cmp_dev->periph) {
        bit_shift = 0U;
    } else {
        bit_shift = 16U;
    }

    /* check if CMP locked */
    if(CMP_CS & (CMP_CS_CMP0LK << bit_shift)) {
        tmp_hal_err = HAL_ERR_LOCK;
    } else {
        /* disable CMP module */
        CMP_CS &= ~(CMP_CS_CMP0EN << bit_shift);
    }

    /* disable CMP EXTI interrupt */
    /* configure EXTI trigger type: falling, rising or both edge */
    if(CMP0 == cmp_dev->periph) {
        /* disable CMP EXTI interrupt */
        EXTI_INTEN &= ~EXTI_CMP_OUTPUT_21;
    } else {
        /* disable CMP EXTI interrupt */
        EXTI_INTEN &= ~EXTI_CMP_OUTPUT_22;
    }

    return tmp_hal_err;
}

/*!
    \brief      set the CMP external trigger interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  cmp_dev: CMP device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to ADC interrupt callback functions structure
      \arg        hal_irq_handle_cb function pointer: the function is user-defined,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
      \arg        NULL: The corresponding callback mechanism is out of use, and
                    disable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_cmp_irq_handle_set(hal_cmp_dev_struct *cmp_dev, hal_cmp_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == cmp_dev) || (NULL == p_irq)) {
        HAL_DEBUGE("pointer [cmp_dev] or pointer [p_irq] address is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* CMP output changed interrupt handler set */
    if(NULL != p_irq->output_changed_handle) {
        cmp_dev->cmp_irq.output_changed_handle = p_irq->output_changed_handle;
        /* enable CMP EXTI interrupt */
        if(CMP0 == cmp_dev->periph) {
            /* enable CMP EXTI interrupt */
            EXTI_INTEN |= EXTI_CMP_OUTPUT_21;
        } else {
            /* enable CMP EXTI interrupt */
            EXTI_INTEN |= EXTI_CMP_OUTPUT_22;
        }
    } else {
        cmp_dev->cmp_irq.output_changed_handle = NULL;
        /* disable CMP EXTI interrupt */
        if(CMP0 == cmp_dev->periph) {
            /* disable CMP EXTI interrupt */
            EXTI_INTEN &= ~EXTI_CMP_OUTPUT_21;
        } else {
            /* disable CMP EXTI interrupt */
            EXTI_INTEN &= ~EXTI_CMP_OUTPUT_22;
        }
    }
}

/*!
    \brief      reset the CMP external trigger interrupt callback function,
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  cmp_dev: CMP device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_cmp_irq_handle_all_reset(hal_cmp_dev_struct *cmp_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp_dev) {
        HAL_DEBUGE("pointer [cmp_dev] address is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* reset interrupt callback function */
    cmp_dev->cmp_irq.output_changed_handle = NULL;
}

/*!
    \brief      CMP interrupt handler content function,
                which is merely used in EXTI_IRQHandler
    \param[in]  cmp_dev: CMP device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_cmp_irq(hal_cmp_dev_struct *cmp_dev)
{
    hal_exti_line_enum linex;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp_dev) {
        HAL_DEBUGE("pointer [cmp_dev] address is invalid");
    }
    if(!IS_CMP_PERIPHERAL(cmp_dev->periph)) {
        HAL_DEBUGE("CMP peripheral is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    if(CMP0 == cmp_dev->periph) {
        linex = EXTI_CMP_OUTPUT_21;
    } else {
        linex = EXTI_CMP_OUTPUT_22;
    }

    /* check EXTI interrupt flag */
    if(SET == hals_exti_interrupt_flag_get(linex)) {
        hals_exti_interrupt_flag_clear(linex);

        if(NULL != cmp_dev->cmp_irq.output_changed_handle) {
            cmp_dev->cmp_irq.output_changed_handle(cmp_dev);
        }
    }
}

/*!
    \brief      lock the CMP
    \note       the register is read only if locked, and only can be unlocked by MCU reset
    \param[in]  cmp_dev: CMP device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_ALREADY_DONE, HAL_ERR_VAL
                details refer to gd32f3x0_hal.h
*/
int32_t hal_cmp_lock(hal_cmp_dev_struct *cmp_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp_dev) {
        HAL_DEBUGE("pointer [cmp_dev] address is invalid");
        /* return function state */
        return HAL_ERR_ADDRESS;
    }
    if(!IS_CMP_PERIPHERAL(cmp_dev->periph)) {
        HAL_DEBUGE("CMP peripheral is invalid");
        /* return function state */
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    if(CMP0 == cmp_dev->periph) {
        /* check if aready locked */
        if(CMP_CS & CMP_CS_CMP0LK) {
            return HAL_ERR_ALREADY_DONE;
        }

        /* lock CMP0 */
        CMP_CS |= (uint32_t)CMP_CS_CMP0LK;
    } else {
        /* check if aready locked */
        if(CMP_CS & CMP_CS_CMP1LK) {
            return HAL_ERR_ALREADY_DONE;
        }
        /* lock CMP1 */
        CMP_CS |= (uint32_t)CMP_CS_CMP1LK;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      get output level of CMP
    \param[in]  cmp_dev: CMP device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the output level
*/
uint32_t hal_cmp_output_level_get(hal_cmp_dev_struct *cmp_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp_dev) {
        HAL_DEBUGE("pointer [cmp_dev] address is invalid");
    }
    if(!IS_CMP_PERIPHERAL(cmp_dev->periph)) {
        HAL_DEBUGE("CMP peripheral is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    if(CMP0 == cmp_dev->periph) {
        /* get output level of CMP0 */
        cmp_dev->output_level = (CMP_CS & CMP_CS_CMP0O) ? CMP_OUTPUTLEVEL_HIGH : CMP_OUTPUTLEVEL_LOW;
    } else {
        /* get output level of CMP1 */
        cmp_dev->output_level = (CMP_CS & CMP_CS_CMP1O) ? CMP_OUTPUTLEVEL_HIGH : CMP_OUTPUTLEVEL_LOW;
    }

    return cmp_dev->output_level;
}

/*!
    \brief      get the state of CMP device
    \param[in]  cmp_dev: CMP device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     hal_cmp_state_enum: HAL_CMP_STATE_NONE, HAL_CMP_STATE_RESET, HAL_CMP_STATE_BUSY, HAL_CMP_STATE_READY,
                                    HAL_CMP_STATE_TIMEOUT, HAL_CMP_STATE_ERROR
*/
hal_cmp_state_enum hal_cmp_state_get(hal_cmp_dev_struct *cmp_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp_dev) {
        HAL_DEBUGE("pointer [cmp_dev] address is invalid");
        return HAL_CMP_STATE_NONE;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    return cmp_dev->state;
}
