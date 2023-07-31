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
#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <nrf_drv_gpiote.h>
#include <sys/stat.h>
#include <core/mri.h>

static void gpioteHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void enteringDebuggerHook(void* pvContext);
static void leavingDebuggerHook(void* pvContext);

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

    uint8_t* pBuff = malloc(st.st_size) + 1;
    size_t bytesRead = fread(pBuff, 1, st.st_size, pFile);
    if (bytesRead != st.st_size)
    {
        fprintf(stderr, "error: Failed read call\r\n");
        return -1;
    }
    pBuff[st.st_size] = '\0';
    printf("dmri contents\r\n");
    printf("%s\r\n", pBuff);

    fclose(pFile);
    printf("Semihosting exercise completed...\r\n");


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
}

// ISR for button 3 press running at priority level of 2 (higher priority than MRI).
static void gpioteHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // Crash when button 3 is pressed.
    *(volatile uint32_t*)0xFFFFFFFF; // = 0xbaadf00d;
}

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
