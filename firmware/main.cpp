/*  Copyright (C) 2022  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* Beginnings of firmware for RugRover robot. */
#include <stdio.h>
#include <errno.h>
#include <app_error.h>
#include <nrf_assert.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <nrf_drv_gpiote.h>
#include <sys/stat.h>
#include <core/mri.h>
#include "SAADCScanner/SAADCScanner.h"

// Bangle.js2 analog battery voltage is connected to this pin through a 1/4 resistor divider.
#define BATTERY_VOLTAGE_PIN 3



#ifdef UNDONE
static void enteringDebuggerHook(void* pvContext);
static void leavingDebuggerHook(void* pvContext);
#endif // UNDONE

// Analog to digital converter object.
static SAADCScanner g_adc(NRF_SAADC_RESOLUTION_12BIT, false, _PRIO_APP_LOWEST);

volatile uint32_t g_count = 0;

int main(void)
{
    // Exercise the semihosting support a bit.
    printf("Exercising Semihosting Support...\r\n");

    FILE* pFile = fopen("non-existent.file", "r");
    if (pFile || errno != ENOENT)
    {
        fprintf(stderr, "error: Didn't fail to open non-existent.file as expected\r\n");
        return -1;
    }

    pFile = fopen("dmri", "r");
    if (!pFile)
    {
        fprintf(stderr, "error: Failed to open dmri script\r\n");
        return -1;
    }
    struct stat st;
    int result = fstat(__sfileno(pFile), &st);
    if (result)
    {
        fprintf(stderr, "error: Failed fstat call\r\n");
        return -1;
    }

    printf("dmri file statistics\r\n");
    printf("  size = %ld\r\n", st.st_size);

    uint8_t* pBuff = (uint8_t*)malloc(st.st_size) + 1;
    size_t bytesRead = fread(pBuff, 1, st.st_size, pFile);
    if (bytesRead != (size_t)st.st_size)
    {
        fprintf(stderr, "error: Failed read call\r\n");
        return -1;
    }
    pBuff[st.st_size] = '\0';
    printf("dmri contents\r\n");
    printf("%s\r\n", pBuff);

    fclose(pFile);
    printf("Semihosting exercise completed...\r\n");

    while (true)
    {
    }



    // Initialize the ADC object which scans all of the configured ADC channels manually.
    result = g_adc.init();
    ASSERT ( result );
    SAADCScanner::Channel* pBatteryVoltage = g_adc.addChannel(BATTERY_VOLTAGE_PIN,
                                                               NRF_SAADC_RESISTOR_DISABLED,
                                                               NRF_SAADC_GAIN1_4,
                                                               NRF_SAADC_REFERENCE_VDD4,
                                                               NRF_SAADC_ACQTIME_3US,
                                                               SAADC_LOWER_LIMIT_DISABLED,
                                                               SAADC_UPPER_LIMIT_DISABLED,
                                                               NULL);
    ASSERT ( pBatteryVoltage != NULL );

    while (true)
    {
        // Kick off the next ADC sampling cycle so that the sample is ready by the time it is needed later in this loop.
        g_adc.startScanning();
        nrf_delay_ms(10);

        // Read the battery voltage.
        ASSERT ( pBatteryVoltage->getSampleCount() > 0 );
        SAADCScanner::Channel::Reading reading = pBatteryVoltage->read();
        float batteryVoltage = 3.3f * 4.0f * reading.mean / 4095.0f;

        printf("g_count=%lu, Vbatt=%.1f\n", g_count++, batteryVoltage + 0.05f);
    }

#ifdef UNDONE
    /* Toggle LEDs. */
    const uint32_t leds[4] = { 13, 14, 15, 16 };
    for (size_t i = 0 ; i < 4 ; i++) {
        nrf_gpio_cfg_output(leds[i]);
    }

    /* Interrupt on button 3 presses. */
    const uint32_t button3Pin = 24;
    nrf_drv_gpiote_in_config_t gpioteConfig =
    {
        .sense = NRF_GPIOTE_POLARITY_HITOLO,
        .pull = NRF_GPIO_PIN_PULLUP,
        .is_watcher = false,
        .hi_accuracy = false
    };
    uint32_t errorCode = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(errorCode);
    errorCode = nrf_drv_gpiote_in_init(button3Pin, &gpioteConfig, gpioteHandler);
    APP_ERROR_CHECK(errorCode);
    nrf_drv_gpiote_in_event_enable(button3Pin, true);

    mriSetDebuggerHooks(enteringDebuggerHook, leavingDebuggerHook, NULL);
    while (true)
    {
        //*(volatile uint32_t*)0xFFFFFFFF; // = 0xbaadf00d;

        //__disable_irq();
        //__set_BASEPRI(0x06<<(8-__NVIC_PRIO_BITS));
            g_count++;
        //__enable_irq();
        //__set_BASEPRI(0x00);

        for (int i = 0; i < 4; i++)
        {
            printf("Toggle\r\n");
            nrf_gpio_pin_toggle(leds[i]);
            nrf_delay_ms(500);
        }
    }
#endif // UNDONE
}


#ifdef UNDONE
/* Flag set to flag that MRI hooks should be disabled when writing to stdout. */
extern volatile int g_mriDisableHooks;

static void enteringDebuggerHook(void* pvContext)
{
    if (g_mriDisableHooks)
    {
        return;
    }

    // Turn both LEDs off when entering debugger.
    nrf_gpio_pin_set(19);
    nrf_gpio_pin_set(20);
}

static void leavingDebuggerHook(void* pvContext)
{
    if (g_mriDisableHooks)
    {
        return;
    }
}
#endif // UNDONE


// Break into debugger if any errors/asserts are detected at runtime.
// This handler is called when an application error is encountered in BLE stack or application code.
extern "C" void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    { __asm volatile ("bkpt #0"); }
}
