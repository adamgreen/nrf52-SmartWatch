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
#include <nrf_drv_common.h>
#include "SAADCScanner.h"

SAADCScanner* g_pThis = NULL;


SAADCScanner::SAADCScanner(nrf_saadc_resolution_t resolution /* = NRF_SAADC_RESOLUTION_12BIT */,
                           bool sampleContinuously /* = true */,
                           uint8_t irqPriority /* = _PRIO_APP_LOWEST*/)
{
    // Can only instantiate one SAADCScanner object.
    ASSERT ( g_pThis == NULL );

    m_usedChannelCount = 0;
    memset(m_buffer, 0, sizeof(m_buffer));
    m_resolution = resolution;
    m_sampleContinuously = sampleContinuously;
    m_irqPriority = irqPriority;
    g_pThis = this;

    for (size_t i = 0 ; i < sizeof(m_channels)/sizeof(m_channels[0]) ; i++)
    {
        m_channels[i].m_index = i;
    }
}

bool SAADCScanner::init()
{
    nrf_saadc_resolution_set(m_resolution);
    nrf_saadc_oversample_set(NRF_SAADC_OVERSAMPLE_DISABLED);

    nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_drv_common_irq_enable(SAADC_IRQn, m_irqPriority);
    nrf_saadc_int_enable(NRF_SAADC_INT_END);
    nrf_saadc_enable();

    return true;
}

SAADCScanner::Channel* SAADCScanner::addChannel(uint8_t pin,
                                                nrf_saadc_resistor_t pullUpDown /* = NRF_SAADC_RESISTOR_DISABLED */,
                                                nrf_saadc_gain_t gain /* =NRF_SAADC_GAIN1_4 */,
                                                nrf_saadc_reference_t reference /* =NRF_SAADC_REFERENCE_VDD4 */,
                                                nrf_saadc_acqtime_t acquisitionTime /* =NRF_SAADC_ACQTIME_3US */,
                                                int16_t lowLimit /* =NRF_DRV_SAADC_LIMITH_DISABLED */,
                                                int16_t highLimit /* =NRF_DRV_SAADC_LIMITL_DISABLED */,
                                                ISAADCScannerNotification* pNotification /* = NULL */)
{
    Channel* pChannel = findFreeChannel();
    if (pChannel == NULL)
    {
        return NULL;
    }

    bool result = pChannel->init(pin, pullUpDown, gain, reference, acquisitionTime, lowLimit, highLimit, pNotification, m_irqPriority);
    if (!result)
    {
        return NULL;
    }

    return pChannel;
}

SAADCScanner::Channel* SAADCScanner::findFreeChannel()
{
    if (m_usedChannelCount >= ARRAY_SIZE(m_channels))
    {
        return NULL;
    }
    return &m_channels[m_usedChannelCount++];
}

void SAADCScanner::startScanning()
{
    nrf_saadc_buffer_init(m_buffer, m_usedChannelCount);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);

    nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);
}

extern "C" void SAADC_IRQHandler(void)
{
    if (!nrf_saadc_event_check(NRF_SAADC_EVENT_END))
    {
        return;
    }
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);

    for (size_t i = 0 ; i < g_pThis->m_usedChannelCount ; i++)
    {
        g_pThis->m_channels[i].update(g_pThis->m_buffer[i]);
    }
    if (g_pThis->m_sampleContinuously)
    {
        g_pThis->startScanning();
    }
}



SAADCScanner::Channel::Channel()
{
    clear();
    m_pNotification = NULL;
    m_irqPriority = 0;
    m_origPriority = 0;
}

bool SAADCScanner::Channel::init(uint8_t pin,
                   nrf_saadc_resistor_t pullUpDown,
                   nrf_saadc_gain_t gain,
                   nrf_saadc_reference_t reference,
                   nrf_saadc_acqtime_t acquisitionTime,
                   int16_t lowLimit,
                   int16_t highLimit,
                   ISAADCScannerNotification* pNotification,
                   uint8_t irqPriority)
{
    nrf_saadc_input_t analogPin = NRF_SAADC_INPUT_DISABLED;
    switch (pin)
    {
        case 2:
            analogPin = NRF_SAADC_INPUT_AIN0;
            break;
        case 3:
            analogPin = NRF_SAADC_INPUT_AIN1;
            break;
        case 4:
            analogPin = NRF_SAADC_INPUT_AIN2;
            break;
        case 5:
            analogPin = NRF_SAADC_INPUT_AIN3;
            break;
        case 28:
            analogPin = NRF_SAADC_INPUT_AIN4;
            break;
        case 29:
            analogPin = NRF_SAADC_INPUT_AIN5;
            break;
        case 30:
            analogPin = NRF_SAADC_INPUT_AIN6;
            break;
        case 31:
            analogPin = NRF_SAADC_INPUT_AIN7;
            break;
        default:
            // Not a valid Analog In pin.
            ASSERT ( false );
            return false;
    }
    m_irqPriority = irqPriority;
    m_pNotification = pNotification;

    nrf_saadc_channel_config_t config =
    {
        .resistor_p = pullUpDown,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain = gain,
        .reference = reference,
        .acq_time = acquisitionTime,
        .mode = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst = NRF_SAADC_BURST_DISABLED,
        .pin_p = analogPin,
        .pin_n = NRF_SAADC_INPUT_DISABLED
    };
    nrf_saadc_channel_init(m_index, &config);

    m_lowLimit = lowLimit;
    m_highLimit = highLimit;

    return true;
}

void SAADCScanner::Channel::update(int16_t latestSample)
{
    m_count++;
    m_sum += latestSample;
    m_latest = latestSample;
    if (latestSample < m_min)
    {
        m_min = latestSample;
    }
    if (latestSample > m_max)
    {
        m_max = latestSample;
    }

    bool limitExceeded = false;
    if (latestSample < m_lowLimit)
    {
        m_lowLimitExceeded = true;
        limitExceeded = true;
    }
    if (latestSample > m_highLimit)
    {
        m_highLimitExceeded = true;
        limitExceeded = true;
    }
    if (limitExceeded && m_pNotification != NULL)
    {
        m_pNotification->notifyLimitExceeded(m_lowLimitExceeded, m_highLimitExceeded);
    }
}
