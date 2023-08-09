/*  Copyright (C) 2023  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
// Driver for the LPM013M126C Transflective LCD on the Bangle.js2 SmartWatch.
// Learned a lot from:
//      https://github.com/BigCorvus/ColorMemLCD code by Arthur Jordan (https://github.com/BigCorvus)
//      https://github.com/espruino/Espruino
#include <app_util_platform.h>
#include <nrf_drv_spi.h>
#include <nrf_drv_pwm.h>
#include <Adafruit_GFX.h>


class ColorMemLCD : public Adafruit_GFX
{
    public:
        // The dimensions of the LCD.
        static const int32_t DISP_WIDTH = 176;
        static const int32_t DISP_HEIGHT = 176;

        // All of the 3-bit colours supported by this LCD.
        static const uint16_t COLOR_BLACK = 0x00;
        static const uint16_t COLOR_BLUE = 0x04;
        static const uint16_t COLOR_GREEN = 0x02;
        static const uint16_t COLOR_CYAN = 0x06;
        static const uint16_t COLOR_RED = 0x01;
        static const uint16_t COLOR_MAGENTA = 0x05;
        static const uint16_t COLOR_YELLOW = 0x03;
        static const uint16_t COLOR_WHITE = 0x07;

        enum BlinkModes
        {
            // No Blink
            BLINKMODE_NONE = 0x00,
            // Blink to White.
            BLINKMODE_WHITE = 0x01,
            // Blink to Black.
            BLINKMODE_BLACK = 0x02,
            // Blink to Inverted.
            BLINKMODE_INVERSE = 0x03
        };

        ColorMemLCD(const nrf_drv_spi_t* pSpiInstance, nrf_drv_pwm_t* pPWM,
                    uint8_t sclkPin, uint8_t siPin, uint8_t scsPin,
                    uint8_t extcominPin, uint8_t dispPin, uint8_t backlightPin,
                    app_irq_priority_t irqPriority = APP_IRQ_PRIORITY_LOWEST);

        void init();

        void turnOn();
        void turnOff();
        bool isOn();

        void turnBacklightOn();
        void turnBacklightOff();
        bool isBacklightOn();

        void clearDisplay();
        void cls(uint16_t color);
        void drawPixel(int16_t x, int16_t y, uint16_t color);
        void setBlinkMode(BlinkModes mode);
        int printf(const char* pFormat, ...);

        void refresh(bool block = true);
        bool isRefreshInProgress()
        {
            return m_inProgress;
        }
        void waitForRefreshToComplete()
        {
            while (isRefreshInProgress())
            {
                __WFE();
            }
        }

    protected:
        static const int32_t BITS_PER_PIXEL = 3;
        static const int32_t BITS_PER_BYTE = 8;
        // Actual amount of pixel data in row. Doesn't include M and AG row header bits.
        static const int32_t BYTES_PER_ROW = DISP_WIDTH * BITS_PER_PIXEL / BITS_PER_BYTE;
        // Each row of the display takes this many bytes when sent over SPI.
        // It includes 16-bits (2-bytes) for the M and AG bit header at the start of each row.
        static const int32_t ROW_SPAN = 2 + BYTES_PER_ROW;

        void handleSpiEvent(nrf_drv_spi_evt_t const * pEvent);
        static void staticHandleSpiEvent(nrf_drv_spi_evt_t const * pEvent);
        void handlePwmEvent(nrf_drv_pwm_evt_type_t eventType);
        static void staticHandlePwmEvent(nrf_drv_pwm_evt_type_t eventType);
        void updateExtcominPwmFrequency();
        void disableExtcominPwm();
        void sendCommandBuffer(const uint8_t* pBuffer, size_t bufferLength, bool block);

        // Pointer to SPI peripheral to be used for communicating with OLED driver.
        const nrf_drv_spi_t* m_pSpi;

        // Pointer to PWM peripheral to be used for toggling the EXTCOMIN pin.
        nrf_drv_pwm_t* m_pPWM;

        // This is a pointer into m_buffer where the first pixel of the first row can be found, after that row's header.
        uint8_t* m_pFrameBuffer = &m_buffer[2];

        // The priority level at which the SPI ISR runs.
        app_irq_priority_t m_irqPriority;

        // PWM duty cycles for the EXTCOMIN pulses.
        nrf_pwm_values_individual_t m_dutyCycles;

        // Tracks the topmost and bottommost row that has been touched since the last refresh().
        // Only these rows actually need to be sent ot the LCD on the next refresh().
        uint8_t m_minDirtyRow = 0;
        uint8_t m_maxDirtyRow = DISP_HEIGHT - 1;

        // Is there a SPI transfer in progress? Gets cleared by handleSpiEvent() ISR handler.
        volatile bool m_inProgress = false;

        // Is the EXTCOMIN PWM instance running?
        bool m_isPwmRunning = false;

        // CPU pins connected to the LCD.
        uint8_t m_scsPin = NRF_DRV_SPI_PIN_NOT_USED;
        uint8_t m_sclkPin = NRF_DRV_SPI_PIN_NOT_USED;
        uint8_t m_siPin = NRF_DRV_SPI_PIN_NOT_USED;
        uint8_t m_extcominPin = NRF_DRV_SPI_PIN_NOT_USED;
        uint8_t m_dispPin = NRF_DRV_SPI_PIN_NOT_USED;
        uint8_t m_backlightPin = NRF_DRV_SPI_PIN_NOT_USED;

        // The pixel data is stored in this buffer, along with the header and trailer bytes required when sending
        // the frame over SPI.
        uint8_t m_buffer[ROW_SPAN*DISP_HEIGHT+2];
};
