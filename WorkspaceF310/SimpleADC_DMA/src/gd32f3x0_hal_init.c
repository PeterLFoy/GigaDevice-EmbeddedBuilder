/*
    \file  gd32f3x0_hal_init.c
*/
/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    All rights reserved.

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
#include "gd32f3x0_hal_init.h"
/* user code [global 0] begin */

/* user code [global 0] end */
hal_adc_dev_struct adc_info;
hal_dma_dev_struct dma_adc_info;
hal_uart_dev_struct uart0_info;

void msd_system_init(void)
{
    /* user code [system_init local 0] begin */
    /* user code [system_init local 0] end */
    hal_rcu_periph_clk_enable(RCU_CFGCMP);
    hal_nvic_irq_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    hal_sys_debug_init(SYS_DEBUG_SERIAL_WIRE);
    hal_sys_timesource_init(SYS_TIMEBASE_SOURCE_SYSTICK);

    hal_nvic_set_priority(NonMaskableInt_IRQn, 0, 0);
    hal_nvic_set_priority(SVCall_IRQn, 0, 0);
    hal_nvic_set_priority(PendSV_IRQn, 0, 0);
    hal_nvic_set_priority(SysTick_IRQn, 0, 0);
    /* user code [system_init local 1] begin */
    /* user code [system_init local 1] end */
}

void msd_clock_init(void)
{
    /* user code [clock_init local 0] begin */
    /* user code [clock_init local 0] end */
    hal_rcu_clk_struct rcu_clk_parameter;
    hal_rcu_osci_struct rcu_osci_parameter;
    hal_rcu_periphclk_struct rcu_periphclk_parameter;

    hal_rcu_struct_init(HAL_RCU_CLK_STRUCT, &rcu_clk_parameter);
    hal_rcu_struct_init(HAL_RCU_OSCI_STRUCT, &rcu_osci_parameter);
    hal_rcu_struct_init(HAL_RCU_PERIPHCLK_STRUCT, &rcu_periphclk_parameter);

    rcu_osci_parameter.hxtal.need_configure = ENABLE;
    rcu_osci_parameter.hxtal.state = RCU_OSC_ON;
    rcu_osci_parameter.lxtal.need_configure = ENABLE;
    rcu_osci_parameter.lxtal.state = RCU_OSC_ON;
    rcu_osci_parameter.irc8m.need_configure = ENABLE;
    rcu_osci_parameter.irc8m.state = RCU_OSC_ON;
    rcu_osci_parameter.irc8m.adjust_value = 0;
    rcu_osci_parameter.pll.need_configure = ENABLE;
    rcu_osci_parameter.pll.state = RCU_OSC_ON;
    rcu_osci_parameter.pll.pll_source = RCU_PLL_SRC_HXTAL_IRC48M;
    rcu_osci_parameter.pll.pll_mul = RCU_PLL_MULT3;
    rcu_osci_parameter.pll.pll_presel = RCU_PLL_PRESEL_HXTAL;
    rcu_osci_parameter.pll.pre_div = RCU_PLL_PREDIV1;
    if(HAL_ERR_NONE != hal_rcu_osci_config(&rcu_osci_parameter)){
        while(1);
    }

    rcu_clk_parameter.clock_type = RCU_CLKTYPE_SYSCLK | RCU_CLKTYPE_AHBCLK | RCU_CLKTYPE_APB1CLK | RCU_CLKTYPE_APB2CLK | RCU_CLKTYPE_CK48MCLK;
    rcu_clk_parameter.sysclk_source = RCU_SYSCLK_SRC_PLL;
    rcu_clk_parameter.ahbclk_divider = RCU_SYSCLK_AHBDIV1;
    rcu_clk_parameter.apb1clk_divider = RCU_AHBCLK_APB1DIV1;
    rcu_clk_parameter.apb2clk_divider = RCU_AHBCLK_APB2DIV1;
    rcu_clk_parameter.ck48mclk_source = RCU_USB_CK48MSRC_IRC48M;
    if(HAL_ERR_NONE != hal_rcu_clock_config(&rcu_clk_parameter)){
        while(1);
    }

    rcu_periphclk_parameter.periph_clock_type = RCU_PERIPH_CLKTYPE_ADC;
    rcu_periphclk_parameter.adc_clock_source = RCU_ADCCK_APB2_DIV2;
    if(HAL_ERR_NONE != hal_rcu_periph_clock_config(&rcu_periphclk_parameter)){
        while(1);
    }

    rcu_periphclk_parameter.periph_clock_type = RCU_PERIPH_CLKTYPE_USART0;
    rcu_periphclk_parameter.usart0_clock_source = RCU_USART0_CLKSRC_APB2;
    if(HAL_ERR_NONE != hal_rcu_periph_clock_config(&rcu_periphclk_parameter)){
        while(1);
    }

    /* user code [clock_init local 1] begin */
    /* user code [clock_init local 1] end */
}

void msd_dma_init(void)
{
    /* user code [dma_init local 0] begin */
    /* user code [dma_init local 0] end */
    hal_rcu_periph_clk_enable(RCU_DMA);
    hal_nvic_irq_enable(DMA_Channel0_IRQn, 0, 0);
    /* user code [dma_init local 1] begin */
    /* user code [dma_init local 1] end */
}

void msd_dma_deinit(void)
{
    /* user code [dma_deinit local 0] begin */
    /* user code [dma_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_DMA);
    /* user code [dma_deinit local 1] begin */
    /* user code [dma_deinit local 1] end */
}

void msd_gpio_init(void)
{
    /* user code [gpio_init local 0] begin */
    /* user code [gpio_init local 0] end */
    hal_rcu_periph_clk_enable(RCU_GPIOC);
    hal_rcu_periph_clk_enable(RCU_GPIOF);
    hal_rcu_periph_clk_enable(RCU_GPIOA);
    /* user code [gpio_init local 1] begin */
    /* user code [gpio_init local 1] end */
}

void msd_gpio_deinit(void)
{
    /* user code [gpio_deinit local 0] begin */
    /* user code [gpio_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_GPIOC);
    hal_rcu_periph_clk_disable(RCU_GPIOF);
    hal_rcu_periph_clk_disable(RCU_GPIOA);
    /* user code [gpio_deinit local 1] begin */
    /* user code [gpio_deinit local 1] end */
}

void msd_adc_init(void)
{
    /* user code [adc_init local 0] begin */
    /* user code [adc_init local 0] end */
    hal_gpio_init_struct gpio_init_parameter;
    hal_dma_init_struct dma_adc_init_parameter;

    hal_adc_init_struct adc_init_parameter;
    hal_adc_routine_config_struct adc_routine_config_parameter;
    hal_adc_routine_rank_config_struct adc_routine_rank_config_parameter;

    hal_rcu_periph_clk_enable(RCU_ADC);
    hal_gpio_struct_init(&gpio_init_parameter);

    gpio_init_parameter.mode = GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = GPIO_PULL_NONE;
    gpio_init_parameter.ospeed = GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_2, &gpio_init_parameter);

    hal_adc_struct_init(HAL_ADC_INIT_STRUCT, &adc_init_parameter);
    hal_adc_struct_init(HAL_ADC_DEV_STRUCT, &adc_info);
    hal_adc_struct_init(HAL_ADC_ROUTINE_CONFIG_STRUCT, &adc_routine_config_parameter);
    hal_adc_struct_init(HAL_ADC_ROUTINE_RANK_CONFIG_STRUCT, &adc_routine_rank_config_parameter);

    adc_init_parameter.data_alignment = ADC_LSB_ALIGNMENT;
    adc_init_parameter.resolution = ADC_RESOLUTION_12B;
    adc_init_parameter.scan_mode = DISABLE;
    adc_init_parameter.hardware_oversampling = DISABLE;
    hal_adc_init(&adc_info,&adc_init_parameter);

    adc_routine_config_parameter.routine_sequence_conversions = ENABLE;
    adc_routine_config_parameter.routine_sequence_length = 1;
    adc_routine_config_parameter.routine_sequence_external_trigger_select = ADC_EXTTRIG_ROUTINE_NONE;
    adc_routine_config_parameter.continuous_mode = ENABLE;
    adc_routine_config_parameter.discontinuous_mode = DISABLE;
    hal_adc_routine_channel_config(&adc_info,&adc_routine_config_parameter);

    adc_routine_rank_config_parameter.channel = ADC_CHANNEL_2;
    adc_routine_rank_config_parameter.sampling_time = ADC_SAMPLETIME_55POINT5;
    adc_routine_rank_config_parameter.routine_sequence = ADC_ROUTINE_SEQUENCE_0;
    hal_adc_routine_rank_config(&adc_info,&adc_routine_rank_config_parameter);

    hal_dma_struct_init(HAL_DMA_INIT_STRUCT, &dma_adc_init_parameter);
    hal_dma_struct_init(HAL_DMA_DEV_STRUCT, &dma_adc_info);

    dma_adc_init_parameter.direction = DMA_DIR_PERIPH_TO_MEMORY;
    dma_adc_init_parameter.periph_inc = DISABLE;
    dma_adc_init_parameter.memory_inc = ENABLE;
    dma_adc_init_parameter.periph_width = DMA_PERIPH_SIZE_16BITS;
    dma_adc_init_parameter.memory_width = DMA_MEMORY_SIZE_16BITS;
    dma_adc_init_parameter.mode = DMA_CIRCULAR_MODE;
    dma_adc_init_parameter.priority = DMA_PRIORITY_LEVEL_HIGH;
    hal_dma_init(&dma_adc_info, DMA_CH0, &dma_adc_init_parameter);

    hal_periph_dma_info_bind(adc_info, p_dma_adc, dma_adc_info);
    /* user code [adc_init local 1] begin */
    /* ADC trigger config */
    hals_adc_external_trigger_config(ADC_ROUTINE_CHANNEL, ENABLE);
    /* enable ADC interface */
    hals_adc_enable();
    /* ADC calibration and reset calibration */
    hal_sys_basetick_delay_ms(1U);
    hals_adc_calibration_enable();
    /* ADC DMA function enable */
    hals_adc_dma_mode_enable();
    /* ADC software trigger enable */
    hals_adc_software_trigger_enable(ADC_ROUTINE_CHANNEL);
    /* user code [adc_init local 1] end */
}

void msd_adc_deinit(void)
{
    /* user code [adc_deinit local 0] begin */
    /* user code [adc_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_ADC);
    hal_gpio_deinit(GPIOA, GPIO_PIN_2);
    hal_adc_deinit(&adc_info);
    hal_dma_deinit(&dma_adc_info);
    /* user code [adc_deinit local 1] begin */
    /* user code [adc_deinit local 1] end */
}

void msd_usart0_init(void)
{
    /* user code [usart0_init local 0] begin */
    /* user code [usart0_init local 0] end */
    hal_gpio_init_struct gpio_init_parameter;
    hal_uart_init_struct uart0_init_parameter;

    hal_rcu_periph_clk_enable(RCU_USART0);
    hal_gpio_struct_init(&gpio_init_parameter);

    gpio_init_parameter.mode = GPIO_MODE_AF_PP;
    gpio_init_parameter.pull = GPIO_PULL_NONE;
    gpio_init_parameter.ospeed = GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = GPIO_AF_1;
    hal_gpio_init(GPIOA, GPIO_PIN_10, &gpio_init_parameter);

    gpio_init_parameter.mode = GPIO_MODE_AF_PP;
    gpio_init_parameter.pull = GPIO_PULL_NONE;
    gpio_init_parameter.ospeed = GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = GPIO_AF_1;
    hal_gpio_init(GPIOA, GPIO_PIN_9, &gpio_init_parameter);

    hal_uart_struct_init(HAL_UART_INIT_STRUCT, &uart0_init_parameter);
    hal_uart_struct_init(HAL_UART_DEV_STRUCT, &uart0_info);

    uart0_init_parameter.work_mode = UART_WORK_MODE_ASYN;
    uart0_init_parameter.baudrate = 115200;
    uart0_init_parameter.parity = UART_PARITY_NONE;
    uart0_init_parameter.word_length = UART_WORD_LENGTH_8BIT;
    uart0_init_parameter.stop_bit = UART_STOP_BIT_1;
    uart0_init_parameter.direction = UART_DIRECTION_RX_TX;
    uart0_init_parameter.over_sample = UART_OVER_SAMPLE_16;
    uart0_init_parameter.sample_method = UART_THREE_SAMPLE_BIT;
    uart0_init_parameter.hardware_flow = UART_HARDWARE_FLOW_NONE;
    uart0_init_parameter.rx_fifo_en = DISABLE;
    uart0_init_parameter.timeout_enable = DISABLE;
    uart0_init_parameter.first_bit_msb = DISABLE;
    uart0_init_parameter.tx_rx_swap = DISABLE;
    uart0_init_parameter.rx_level_invert = DISABLE;
    uart0_init_parameter.tx_level_invert = DISABLE;
    uart0_init_parameter.data_bit_invert = DISABLE;
    uart0_init_parameter.overrun_disable = DISABLE;
    uart0_init_parameter.rx_error_dma_stop = DISABLE;
    uart0_init_parameter.rs485_mode = DISABLE;
    hal_uart_init(&uart0_info,USART0,&uart0_init_parameter);

    /* user code [usart0_init local 1] begin */
    /* user code [usart0_init local 1] end */
}

void msd_usart0_deinit(void)
{
    /* user code [usart0_deinit local 0] begin */
    /* user code [usart0_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_USART0);
    hal_gpio_deinit(GPIOA, GPIO_PIN_10);
    hal_gpio_deinit(GPIOA, GPIO_PIN_9);
    hal_uart_deinit(&uart0_info);
    /* user code [usart0_deinit local 1] begin */
    /* user code [usart0_deinit local 1] end */
}

/* user code [global 1] begin */

/* user code [global 1] end */
