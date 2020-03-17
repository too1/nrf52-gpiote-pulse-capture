/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
* @defgroup temperature_example_main main.c
* @{
* @ingroup temperature_example
* @brief Temperature Example Application main file.
* @details
* This file contains the source code for a sample application using the temperature sensor.
* This contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31. PAN 43 is not covered.
*  - PAN_028 rev2.0A anomaly 28 - TEMP: Negative measured values are not represented correctly
*  - PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register.
*  - PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs.
*  - PAN_028 rev2.0A anomaly 31 - TEMP: Temperature offset value has to be manually loaded to the TEMP module
*  - PAN_028 rev2.0A anomaly 43 - TEMP: Using PPI between DATARDY event and START task is not functional.
*
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

const nrf_drv_timer_t capture_timer = NRF_DRV_TIMER_INSTANCE(3);

#define SAMPLE_PIN                      3
#define TIMER_PRESCALER                 NRF_TIMER_FREQ_1MHz
#define GPIOTE_CH_CAPTURE               0
#define GPIOTE_CH_RESTART               1
#define GPIOTE_CH_SELFTEST              2

#define SELF_TEST                       1

#if SELF_TEST
const nrf_drv_timer_t test_timer = NRF_DRV_TIMER_INSTANCE(4);

static void run_self_test_pin(uint32_t pin_no, uint32_t us_on, uint32_t us_off)
{
    uint32_t err_code;
    static nrf_ppi_channel_t ppi_ch_test_pin_run, ppi_ch_test_pin_run2;
    
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&test_timer, &timer_cfg, 0);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_timer_extended_compare(&test_timer, 0, us_on,          0,                               false);
    nrf_drv_timer_extended_compare(&test_timer, 1, us_on + us_off, TIMER_SHORTS_COMPARE1_CLEAR_Msk, false);
    
    NRF_GPIOTE->CONFIG[GPIOTE_CH_SELFTEST] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
                                             GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                                             pin_no << GPIOTE_CONFIG_PSEL_Pos;
                                             
    nrfx_ppi_channel_alloc(&ppi_ch_test_pin_run);
    nrfx_ppi_channel_assign(ppi_ch_test_pin_run,
                            nrf_drv_timer_compare_event_address_get(&test_timer, 0),
                            (uint32_t)&NRF_GPIOTE->TASKS_CLR[GPIOTE_CH_SELFTEST]);
    nrfx_ppi_channel_enable(ppi_ch_test_pin_run);
    
    nrfx_ppi_channel_alloc(&ppi_ch_test_pin_run2);
    nrfx_ppi_channel_assign(ppi_ch_test_pin_run2,
                            nrf_drv_timer_compare_event_address_get(&test_timer, 1),
                            (uint32_t)&NRF_GPIOTE->TASKS_SET[GPIOTE_CH_SELFTEST]);
    nrfx_ppi_channel_enable(ppi_ch_test_pin_run2);
    
    nrf_drv_timer_clear(&test_timer);
    nrf_drv_timer_resume(&test_timer);
}
#endif

static void gpiote_capture_init(void)
{
    uint32_t err_code;
    static nrf_ppi_channel_t ppi_ch_gpiote_capture;
    static nrf_ppi_channel_t ppi_ch_gpiote_restart;
    
    // Optionally, enable pullup or pulldown on the input pin
    //nrf_gpio_cfg_input(SAMPLE_PIN, NRF_GPIO_PIN_PULLUP);

    // Allocate two PPI channels
    nrfx_ppi_channel_alloc(&ppi_ch_gpiote_capture);
    nrfx_ppi_channel_alloc(&ppi_ch_gpiote_restart);
    
    // Configure the capture timer
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = TIMER_PRESCALER;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&capture_timer, &timer_cfg, 0);
    APP_ERROR_CHECK(err_code);

    // The GPIOTE driver doesn't support two GPIOTE channels on the same pin, so direct register access is necessary
    NRF_GPIOTE->CONFIG[GPIOTE_CH_CAPTURE] = GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos |
                                            GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos |
                                            SAMPLE_PIN << GPIOTE_CONFIG_PSEL_Pos;
    NRF_GPIOTE->CONFIG[GPIOTE_CH_RESTART] = GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos |
                                            GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos |
                                            SAMPLE_PIN << GPIOTE_CONFIG_PSEL_Pos;

    // Assign a PPI channel to capture the current timer state and store it in CC register 0
    nrfx_ppi_channel_assign(ppi_ch_gpiote_capture, 
                            (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_CAPTURE], 
                            nrf_drv_timer_capture_task_address_get(&capture_timer, 0));
    
    // Assign a second PPI channel to restart the timer when a new pulse is detected
    nrfx_ppi_channel_assign(ppi_ch_gpiote_restart, 
                            (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_RESTART], 
                            nrf_drv_timer_task_address_get(&capture_timer, NRF_TIMER_TASK_CLEAR));
    
    // Enable both PPI channels                
    nrfx_ppi_channel_enable(ppi_ch_gpiote_capture);
    nrfx_ppi_channel_enable(ppi_ch_gpiote_restart);

    // Make sure the GPIOTE capture in event is cleared. This will be  used to detect if a capture has occured. 
    NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_CAPTURE] = 0;
    
    // Start the timer
    nrfx_timer_resume(&capture_timer);
}


static uint32_t timer_capture_value_get(void)
{
    // Make sure the capture event occured before checking the capture register
    if(NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_CAPTURE] != 0)
    {
        // Clear the capture event
        NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_CAPTURE] = 0;
        
        // Return the stored capture value in the timer
        return nrf_drv_timer_capture_get(&capture_timer, 0);
    }
    else
    {
        // In case no capture occured, return 0
        return 0;
    }
}

/** @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    NRF_LOG_INFO("Pulse capture example started");


    gpiote_capture_init();
    
#if SELF_TEST
    run_self_test_pin(LED_1, 678, 10);
#endif

    while (true)
    {
        uint32_t captured_pulse_length = timer_capture_value_get();
        
        if(captured_pulse_length > 0)
        {
            NRF_LOG_INFO("Capture value: %i us", (captured_pulse_length * (1 << TIMER_PRESCALER) / 16));
        }
        else NRF_LOG_INFO("No capture detected");
        
        nrf_delay_ms(500);
        NRF_LOG_FLUSH();
    }
}


/** @} */
