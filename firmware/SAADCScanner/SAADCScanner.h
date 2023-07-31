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
// Driver to use the SAADC peripheral to continously scan the configured analog pins and track the latest stats
// (min, max, mean, latest) of analog measurements made since the last read request.
#ifndef SAADC_SCANNER_H_
#define SAADC_SCANNER_H_

#include <app_util_platform.h>
#include <nrf_saadc.h>


// Pass into addChannel() method for lowLimit and/or highLimit parameter to disable this feature.
#define SAADC_LOWER_LIMIT_DISABLED 0x8000
#define SAADC_UPPER_LIMIT_DISABLED 0x7FFF


// The SAADC ISR will be declared as a friend of this class so that it can access some of the protected fields.
extern "C" void SAADC_IRQHandler(void);


class ISAADCScannerNotification
{
    public:
        virtual void notifyLimitExceeded(bool lowLimitExceeded, bool highLimitExceeded) = 0;
};

class SAADCScanner
{
    public:
        SAADCScanner(nrf_saadc_resolution_t resolution = NRF_SAADC_RESOLUTION_12BIT,
                     bool sampleContinuously = true,
                     uint8_t irqPriority = _PRIO_APP_LOWEST);

        bool init();

        class Channel
        {
            public:
                int32_t getSampleCount()
                {
                    return m_count;
                }

                struct Reading
                {
                    int32_t count;
                    float   mean;
                    int16_t latest;
                    int16_t min;
                    int16_t max;
                    bool    lowLimitExceeded;
                    bool    highLimitExceeded;
                };

                Reading read()
                {
                    disableInterrupt();
                        Reading reading =
                        {
                            .count = m_count,
                            .mean = (float)m_sum/(float)m_count,
                            .latest = m_latest,
                            .min = m_min,
                            .max = m_max,
                            .lowLimitExceeded = m_lowLimitExceeded,
                            .highLimitExceeded = m_highLimitExceeded
                        };
                        clear();
                    enableInterrupt();

                    return reading;
                }

            protected:
                friend class SAADCScanner;
                friend void SAADC_IRQHandler(void);

                Channel();

                bool init(uint8_t pin,
                          nrf_saadc_resistor_t pullUpDown,
                          nrf_saadc_gain_t gain,
                          nrf_saadc_reference_t reference,
                          nrf_saadc_acqtime_t acquisitionTime,
                          int16_t lowLimit,
                          int16_t highLimit,
                          ISAADCScannerNotification* pNotification,
                          uint8_t irqPriority);

                void update(int16_t latestSample);

                void limitExceeded(nrf_saadc_limit_t limitHit);

                void clear()
                {
                    m_sum = 0;
                    m_count = 0;
                    m_latest = 0;
                    m_min = (int16_t)0x7FFF;
                    m_max = (int16_t)0x8000;
                    m_lowLimitExceeded = false;
                    m_highLimitExceeded = false;
                }

                void disableInterrupt()
                {
                    m_origPriority = __get_BASEPRI();
                    __set_BASEPRI(m_irqPriority << (8-__NVIC_PRIO_BITS));
                }
                void enableInterrupt()
                {
                    __set_BASEPRI(m_origPriority);
                }

                ISAADCScannerNotification* m_pNotification;
                size_t           m_index;
                volatile int32_t m_sum;
                volatile int32_t m_count;
                volatile int16_t m_latest;
                volatile int16_t m_min;
                volatile int16_t m_max;
                int16_t          m_lowLimit;
                int16_t          m_highLimit;
                uint8_t          m_irqPriority;
                uint8_t          m_origPriority;
                volatile bool    m_lowLimitExceeded;
                volatile bool    m_highLimitExceeded;
        };

        Channel* addChannel(uint8_t pin,
                            nrf_saadc_resistor_t pullUpDown = NRF_SAADC_RESISTOR_DISABLED,
                            nrf_saadc_gain_t gain = NRF_SAADC_GAIN1_4,
                            nrf_saadc_reference_t reference = NRF_SAADC_REFERENCE_VDD4,
                            nrf_saadc_acqtime_t acquisitionTime = NRF_SAADC_ACQTIME_3US,
                            int16_t lowLimit = SAADC_LOWER_LIMIT_DISABLED,
                            int16_t highLimit = SAADC_UPPER_LIMIT_DISABLED,
                            ISAADCScannerNotification* pNotification = NULL);

        void startScanning();

    protected:
        friend void SAADC_IRQHandler(void);

        Channel* findFreeChannel();

        size_t                  m_usedChannelCount;
        nrf_saadc_value_t       m_buffer[NRF_SAADC_CHANNEL_COUNT];
        Channel                 m_channels[NRF_SAADC_CHANNEL_COUNT];
        nrf_saadc_resolution_t  m_resolution;
        bool                    m_sampleContinuously;
        uint8_t                 m_irqPriority;
};

#endif // SAADC_SCANNER_H_
